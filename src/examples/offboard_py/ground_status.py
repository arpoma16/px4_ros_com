#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.srv import VehicleCommand as VehicleCommandSrv
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck, VehicleLocalPosition, VehicleStatus
from std_srvs.srv import Empty, Trigger
from enum import Enum

import math
import functools


class UAVState(Enum):
    """Enumeration for UAV states."""
    IDLE = 0
    ARMING = 1
    TAKEOFF = 2
    LOITER = 3
    OFFBOARD = 4
    LANDING = 5
    EMERGENCY = 6

# Define a simple class to hold drone-specific data


class Drone:
    """Helper class to encapsulate data and state for each drone."""

    def __init__(self, drone_id: int):
        self.id = drone_id
        # Publishers (will be initialized in the main node)
        self.trajectory_setpoint_publisher = None

        # Subscribers (will be initialized in the main node)
        self.vehicle_local_position_subscriber = None
        self.vehicle_status_subscriber = None

        # drone call services
        self.offboard_cmd_service = None
        # Drone's current state
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # Control specific variables
        self.offboard_setpoint_counter = 0
        self.current_target_position = [0.0, 0.0, 0.0]  # x, y, z
        self.unavailable = False  # Flag to indicate if the drone is unavailable
        self.is_takeoff = False
        self.is_landing = False
        # Flag to indicate if the drone has completed its landing sequence
        self.has_landed = False
        self.is_hovering = False  # Flag to indicate if the drone is hovering at a position
        self.hovering_count = 0  # Counter to manage hovering state
        # Duration to hover before moving again (in seconds)
        self.hovering_duration = 30
        self.takeoff_height = -5  # take off altitude
        self.uav_state = UAVState.IDLE  # Current state of the drone

    def get_id(self) -> int:
        """Return the drone's ID."""
        return self.id


class MultiDroneOffboardControl(Node):
    """
    Node for controlling multiple vehicles in offboard mode to form and move a triangle,
    and land upon a service call.
    """

    def __init__(self) -> None:
        super().__init__('multi_drone_offboard_control')

        self.get_logger().info('Initializing MultiDroneOffboardControl node...')

        # Configure QoS profile for publishing and subscribing
        # Best effort reliability for control messages, transient local durability for status messages
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Number of drones to control
        self.list_drone_id = [1,2,3]

        self.num_drones = len(self.list_drone_id)
        self.get_logger().info(
            f"Configuring for {self.num_drones} drones with IDs: {self.list_drone_id}")
        # Z-coordinate for takeoff (negative for NED frame)
        self.takeoff_height = -5.0

        # Triangle formation parameters
        self.triangle_radius = 5.0  # Radius of the circle on which triangle vertices lie
        self.triangle_center_x = 0.0
        self.triangle_center_y = 0.0
        # Initial rotation angle for the triangle (radians)
        self.triangle_rotation_angle = 0.0
        # How fast the triangle rotates (radians per timer tick)
        self.rotation_speed = 0.01
        # Counter for how many rounds of triangle formation have been completed
        self.round_count = 0
        # Duration for each round of triangle formation (in seconds)
        self.round_duration = 2
        # List to hold data for each drone
        self.drones = []

        # Initialize publishers, subscribers, and drone data for each drone
        for i in self.list_drone_id:
            drone_id = i
            drone = Drone(drone_id)

            # Define topic names with drone ID prefix
            drone_namespace = f"px4_{drone_id}"
            drone.takeoff_height = self.takeoff_height - i

            # Create publishers for each drone
            drone.trajectory_setpoint_publisher = self.create_publisher(
                TrajectorySetpoint, f'/{drone_namespace}/fmu/in/trajectory_setpoint_offboard', qos_profile)
            drone.vehicle_command_publisher = self.create_publisher(
                VehicleCommand, f'/{drone_namespace}/fmu/in/vehicle_command_offboard', qos_profile)

            # Create service clients for each drone
            drone.offboard_cmd_service = self.create_client(
                VehicleCommandSrv, f'/{drone_namespace}/takeoff')

            # Create subscribers for each drone
            drone.vehicle_local_position_subscriber = self.create_subscription(
                VehicleLocalPosition, f'/{drone_namespace}/fmu/out/vehicle_local_position',
                functools.partial(self.vehicle_local_position_callback, drone=drone), qos_profile)
            drone.vehicle_status_subscriber = self.create_subscription(
                VehicleStatus, f'/{drone_namespace}/fmu/out/vehicle_status',
                functools.partial(self.vehicle_status_callback, drone=drone), qos_profile)

            drone.vehicle_command_offboard_ack_subscriber = self.create_subscription(
                VehicleCommandAck, f'/{drone_namespace}/fmu/out/vehicle_command_offboard_ack',
                functools.partial(self.vehicle_command_offboard_ack_callback, drone=drone), qos_profile)

            self.drones.append(drone)
            self.get_logger().info(
                f"Initialized publishers and subscribers for drone {drone_id} under namespace /{drone_namespace}")

        # Create a timer to publish control commands and manage drone states
        self.timer = self.create_timer(0.1, self.timer_callback)  # 100ms timer
        self.get_logger().info('Timer for control commands created (100ms).')

    def send_takeoff_command(self, drone: Drone):
        """
        Send a takeoff command to the specified drone.
        """
        self.get_logger().info(
            f"Sending takeoff command for Drone {drone.id} to height {drone.takeoff_height:.2f} m")

        self.call_command_service(
            drone, VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=drone.takeoff_height)

    def send_land_command(self, drone: Drone):
        """
        Send a land command to the specified drone.
        """
        self.get_logger().info(f"Sending land command for Drone {drone.id}")
        self.call_command_service(drone, VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def call_command_service(self, drone: Drone, command: int, **params):
        """ Call the vehicle command service for the specified drone.
        """

        if drone.offboard_cmd_service.wait_for_service(timeout_sec=1.0):
            request = VehicleCommandSrv.Request()
            request.request.command = command
            request.request.param1 = params.get('param1', 0.0)
            request.request.param2 = params.get('param2', 0.0)
            request.request.param3 = params.get('param3', 0.0)
            request.request.param4 = params.get('param4', 0.0)
            request.request.param5 = params.get('param5', 0.0)
            request.request.param6 = params.get('param6', 0.0)
            request.request.param7 = params.get('param7', 0.0)
            request.request.target_system = 1
            request.request.target_component = 1
            request.request.source_system = 1
            request.request.source_component = 1
            request.request.from_external = True
            request.request.timestamp = self.get_clock().now().nanoseconds // 1000
            future = drone.offboard_cmd_service.call_async(request)
            # Don't block - just add a callback to handle the response
            # rclpy.spin_until_future_complete(self, future)  this blocks the node, so we use a callback instead
            # Use a lambda to pass the drone object to the callback

            future.add_done_callback(
                lambda f: self._handle_offboard_cmd_response(f, drone))
        else:
            self.get_logger().error(
                f"Takeoff service not available for Drone {drone.id}")
            drone.unavailable = True  # Mark drone as unavailable if service is not available

    def _handle_offboard_cmd_response(self, future, drone: Drone):
        """
        Handle the response from the offboard command service call.
        """
        try:
            response = future.result()
            if response.reply.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                self.get_logger().info(
                    f"Offboard command {response.reply.command} for drone {drone.id} sent successfully.")
            else:
                self.get_logger().error(
                    f"Failed to send offboard command {response.reply.command} for Drone {drone.id}: {response.message}")
        except Exception as e:
            self.get_logger().error(
                f"Exception in offboard command service call for Drone {drone.id}: {str(e)}")
            drone.unavailable = True  # Mark drone as unavailable if service call fails

    def vehicle_local_position_callback(self, vehicle_local_position: VehicleLocalPosition, drone: Drone):
        """
        Callback function for vehicle_local_position topic subscriber.
        Updates the local position for the specific drone.
        """
        drone.vehicle_local_position = vehicle_local_position
        # self.get_logger().debug(f"Drone {drone.id} position: x={vehicle_local_position.x:.2f}, y={vehicle_local_position.y:.2f}, z={vehicle_local_position.z:.2f}")

    def vehicle_status_callback(self, vehicle_status: VehicleStatus, drone: Drone):
        """
        Callback function for vehicle_status topic subscriber.
        Updates the status for the specific drone.
        """
        drone.vehicle_status = vehicle_status
        # self.get_logger().debug(f"Drone {drone.id} status: nav_state={vehicle_status.nav_state}, arm_state={vehicle_status.arming_state}")

    def vehicle_command_offboard_ack_callback(self, vehicle_cmdResponse, drone: Drone):
        """Callback function for vehicle_command_ack topic subscriber."""
        self.get_logger().info(
            f"Drone {drone.id}: Cmd response: {vehicle_cmdResponse.command} result: {vehicle_cmdResponse.result}")

        if vehicle_cmdResponse.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            if vehicle_cmdResponse.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                self.get_logger().info(
                    f"Drone {drone.id}: Vehicle armed successfully")
            else:
                self.get_logger().warn(
                    f"Drone {drone.id}: Vehicle failed to arm")
        if vehicle_cmdResponse.command == VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF:
            if vehicle_cmdResponse.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                self.get_logger().info(
                    f"Drone {drone.id}: Takeoff command accepted")
            else:
                self.get_logger().warn(
                    f"Drone {drone.id}: Takeoff command failed")
        if vehicle_cmdResponse.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            if vehicle_cmdResponse.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                self.get_logger().info(
                    f"Drone {drone.id}: Land command accepted")
            else:
                self.get_logger().warn(
                    f"Drone {drone.id}: Land command failed")

    def land_all_drones_callback(self, request: Empty.Request, response: Empty.Response):
        """
        Callback function for the land_all_drones service.
        Sets the is_landing flag for all drones.
        """
        self.get_logger().info(
            'Land All Drones service called. Initiating landing sequence for all drones.')
        for drone in self.drones:
            if not drone.has_landed:  # Only try to land if not already landed
                drone.is_landing = True
        return Empty.Response()

    def arm(self, drone: Drone):
        """Send an arm command to the vehicle."""
        self._publish_vehicle_command(
            drone, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        #self.get_logger().info(f'Arm command sent for Drone {drone.id}')

    def disarm(self, drone: Drone):
        """Send a disarm command to the vehicle."""
        self._publish_vehicle_command(
            drone, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info(f'Disarm command sent for Drone {drone.id}')

    def engage_offboard_mode(self, drone: Drone):
        """Switch to offboard mode."""
        self._publish_vehicle_command(
            # 1.0 for custom mode, 6.0 for Offboard
            drone, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info(f"Switching Drone {drone.id} to offboard mode")

    def takeoff(self, drone: Drone, altitude: float = 5.0):
        """Send a takeoff command to the vehicle."""
        self._publish_vehicle_command(
            drone, VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=altitude)
        self.get_logger().info(
            f"Takeoff command sent with altitude: {altitude}")

    def land(self, drone: Drone):
        """Switch to land mode."""
        # This command will make the drone switch to land mode and attempt to land
        self._publish_vehicle_command(
            drone, VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info(f"Switching Drone {drone.id} to land mode")

    def _publish_offboard_control_heartbeat_signal(self, drone: Drone):
        """Publish the offboard control mode. This is a heartbeat signal to keep offboard mode active."""
        msg = OffboardControlMode()
        msg.position = True       # Enable position control
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        drone.offboard_control_mode_publisher.publish(msg)

    def _publish_position_setpoint(self, drone: Drone, x: float, y: float, z: float):
        """Publish the trajectory setpoint for a specific drone."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 1.57079  # Fixed yaw (90 degrees, facing positive Y in NED)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        drone.trajectory_setpoint_publisher.publish(msg)
        # self.get_logger().debug(f"Drone {drone.id}: Publishing position setpoints {[x:.2f}, {y:.2f}, {z:.2f]}")

    def _publish_vehicle_command(self, drone: Drone, command: int, **params) -> None:
        """Publish a vehicle command for a specific drone."""
        msg = VehicleCommand()
        msg.command = command
        # Populate command parameters, using defaults if not provided
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)

        # msg.target_system = drone.get_id() + 1           # Target system ID (usually 1 for PX4)
        msg.target_system = 1           # Target system ID (usually 1 for PX4)
        # Target component ID (usually 1 for autopilot)
        msg.target_component = 1
        msg.source_system = 1           # Source system ID
        msg.source_component = 1        # Source component ID
        # Command from external source (e.g., ROS 2)
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        drone.vehicle_command_publisher.publish(msg)

    def _update_triangle_target_positions(self):
        """
        Calculates and updates the target positions for the three drones
        to form a rotating triangle.
        """
        # Vertices of an equilateral triangle inscribed in a circle
        # Offset by 2*pi/3 (120 degrees) for each drone
        angle_offsets = [0, 2 * math.pi / 3, 4 * math.pi / 3]
        # offset off initial position of each drone
        vehicle_offset_x = [0, -2, -4]

        for i, drone in enumerate(self.drones):
            # Calculate angle for current drone with rotation
            current_angle = self.triangle_rotation_angle + angle_offsets[i]

            # Calculate x and y coordinates relative to the triangle center
            rel_x = self.triangle_radius * math.cos(current_angle)
            rel_y = self.triangle_radius * math.sin(current_angle)

            # Add global center offset
            target_x = self.triangle_center_x + rel_x + vehicle_offset_x[i]
            target_y = self.triangle_center_y + rel_y
            target_z = drone.takeoff_height  # Maintain constant altitude

            drone.current_target_position = [target_x, target_y, target_z]

        # Increment rotation angle for next iteration
        self.triangle_rotation_angle += self.rotation_speed
        # Keep angle within [0, 2*pi)
        if self.triangle_rotation_angle >= 2 * math.pi:
            self.triangle_rotation_angle -= 2 * math.pi
            self.round_count += 1
            self.get_logger().info(
                f"Completed round {self.round_count} of triangle formation.")

    def offboard_commanding(self, drone: Drone):
        if (drone.vehicle_local_position.z > drone.takeoff_height + 1 or drone.is_hovering) and self.round_count == 0:
            self._publish_position_setpoint(
                drone, 0.0, 0.0, drone.takeoff_height)
            drone.is_hovering = True
            drone.hovering_count += 1
            if drone.hovering_count >= drone.hovering_duration * 10:  # 10 ticks per second
                drone.is_hovering = False
                drone.hovering_count = 0  # Reset hover count after duration
            self.get_logger().info(
                f"Drone {drone.id}: Hovering at takeoff height {drone.takeoff_height:.2f} m. Hover count: {drone.hovering_count}")
        else:
            if self.round_count < self.round_duration:
                # Once at takeoff height, update triangle positions and publish
                self._update_triangle_target_positions()
                self._publish_position_setpoint(
                    drone,
                    drone.current_target_position[0],
                    drone.current_target_position[1],
                    drone.current_target_position[2]
                )
                self.get_logger().debug(
                    f"Drone {drone.id}: Moving to triangle pos {drone.current_target_position}")
            else:
                # After completing the rounds, initiate landing
                drone.uav_state = UAVState.LANDING
                self.get_logger().info(
                    f"Drone {drone.id}: Completed triangle rounds. Initiating landing.")

    def timer_callback(self) -> None:
        """
        Callback function for the main control timer.
        Manages the state of each drone (arming, offboard, position control, landing).
        """
        all_drones_landed = True  # Assume all drones have landed unless proven otherwise

        for drone in self.drones:
            self.get_logger().info(
                f"Drone {drone.id}: Current state {drone.uav_state}")

            if drone.uav_state == UAVState.IDLE:
                # check is uav is available
                drone.uav_state = UAVState.ARMING
            if drone.uav_state == UAVState.ARMING:
                self.arm(drone)
                if drone.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                    drone.uav_state = UAVState.TAKEOFF

            if drone.uav_state == UAVState.TAKEOFF:
                self.arm(drone)
                self.takeoff(drone, drone.takeoff_height)
                # if drone.vehicle_local_position.z <= drone.takeoff_height + 0.5: # Allow some tolerance
                if drone.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                    drone.uav_state = UAVState.LOITER

            if drone.uav_state == UAVState.LOITER:
                self.arm(drone)
                if (drone.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER):
                    drone.uav_state = UAVState.OFFBOARD
                    self.get_logger().info(f"Loiter, Offboard")

            if drone.uav_state == UAVState.OFFBOARD:
                if drone.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                    self.engage_offboard_mode(drone)
                if drone.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    self.offboard_commanding(drone)

            if drone.uav_state == UAVState.LANDING:
                self.land(drone)
                if drone.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                    drone.has_landed = True
                    self.get_logger().info(
                        f"Drone {drone.id} has landed and disarmed.")
                    continue  # Skip further processing for this drone

            # Only process if the drone hasn't completely landed yet
            if drone.has_landed or drone.unavailable:
                continue  # Skip this drone, it's done
            else:
                all_drones_landed = False  # At least one drone is still active

      # Shut down the node if all drones have landed
        if all_drones_landed:  # Avoid shutting down immediately on start
            self.get_logger().info('All drones have landed. Shutting down node.')
            exit(0)


def main(args=None) -> None:
    try:
        rclpy.init(args=args)
        multi_drone_control = MultiDroneOffboardControl()
        rclpy.spin(multi_drone_control)
    except KeyboardInterrupt:
        print("Node stopped by user (KeyboardInterrupt)")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        print('ROS 2 node shutdown complete.')


if __name__ == '__main__':
    main()
