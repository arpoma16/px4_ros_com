#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from std_srvs.srv import Empty
import math
import functools

# Define a simple class to hold drone-specific data
class Drone:
    """Helper class to encapsulate data and state for each drone."""
    def __init__(self, drone_id: int, takeoff_height: float = -5.0):
        self.id = drone_id
        self.takeoff_height = takeoff_height - (drone_id-1) *2 # Z-coordinate for takeoff (negative for NED frame)
        
        # Publishers (will be initialized in the main node)
        self.offboard_control_mode_publisher = None
        self.trajectory_setpoint_publisher = None
        self.vehicle_command_publisher = None

        # Subscribers (will be initialized in the main node)
        self.vehicle_local_position_subscriber = None
        self.vehicle_status_subscriber = None

        # Drone's current state
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # Control specific variables
        self.offboard_setpoint_counter = 0
        self.current_target_position = [0.0, 0.0, 0.0]  # x, y, z
        self.is_landing = False
        self.has_landed = False # Flag to indicate if the drone has completed its landing sequence
    
    def get_id(self) -> int:
        """Return the drone's ID."""
        return self.id
    
    def is_above_takeoff_height(self) -> bool:
        """Check if the drone is above its target takeoff height."""
        return self.vehicle_local_position.z > self.takeoff_height + 1

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
        self.num_drones = 3
        self.default_takeoff_height = -5.0  # Default Z-coordinate for takeoff (negative for NED frame)

        # Triangle formation parameters
        self.triangle_radius = 5.0  # Radius of the circle on which triangle vertices lie
        self.triangle_center_x = 0.0
        self.triangle_center_y = 0.0
        self.triangle_rotation_angle = 0.0 # Initial rotation angle for the triangle (radians)
        self.rotation_speed = 0.01 # How fast the triangle rotates (radians per timer tick)

        # List to hold data for each drone
        self.drones = []

        # Initialize publishers, subscribers, and drone data for each drone
        for i in range(self.num_drones):
            drone_id = i + 1
            drone = Drone(drone_id, self.default_takeoff_height)

            # Define topic names with drone ID prefix
            drone_namespace = f"px4_{drone_id}"
            
            # Create publishers for each drone
            drone.offboard_control_mode_publisher = self.create_publisher(
                OffboardControlMode, f'/{drone_namespace}/fmu/in/offboard_control_mode', qos_profile)
            drone.trajectory_setpoint_publisher = self.create_publisher(
                TrajectorySetpoint, f'/{drone_namespace}/fmu/in/trajectory_setpoint', qos_profile)
            drone.vehicle_command_publisher = self.create_publisher(
                VehicleCommand, f'/{drone_namespace}/fmu/in/vehicle_command', qos_profile)

            # Create subscribers for each drone
            # Use functools.partial to pass the drone object to the callback
            drone.vehicle_local_position_subscriber = self.create_subscription(
                VehicleLocalPosition, f'/{drone_namespace}/fmu/out/vehicle_local_position', 
                functools.partial(self.vehicle_local_position_callback, drone=drone), qos_profile)
            drone.vehicle_status_subscriber = self.create_subscription(
                VehicleStatus, f'/{drone_namespace}/fmu/out/vehicle_status', 
                functools.partial(self.vehicle_status_callback, drone=drone), qos_profile)
            
            self.drones.append(drone)
            self.get_logger().info(f"Initialized publishers and subscribers for drone {drone_id} under namespace /{drone_namespace}")

        # Create a service server for landing all drones
        self.land_service = self.create_service(Empty, 'land_all_drones', self.land_all_drones_callback)
        self.get_logger().info('ROS 2 service "land_all_drones" created.')

        # Create a timer to publish control commands and manage drone states
        self.timer = self.create_timer(0.1, self.timer_callback) # 100ms timer
        self.get_logger().info('Timer for control commands created (100ms).')

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

    def land_all_drones_callback(self, request: Empty.Request, response: Empty.Response):
        """
        Callback function for the land_all_drones service.
        Sets the is_landing flag for all drones.
        """
        self.get_logger().info('Land All Drones service called. Initiating landing sequence for all drones.')
        for drone in self.drones:
            if not drone.has_landed: # Only try to land if not already landed
                drone.is_landing = True
        return Empty.Response()

    def arm(self, drone: Drone):
        """Send an arm command to the vehicle."""
        self._publish_vehicle_command(
            drone, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info(f'Arm command sent for Drone {drone.id}')

    def disarm(self, drone: Drone):
        """Send a disarm command to the vehicle."""
        self._publish_vehicle_command(
            drone, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info(f'Disarm command sent for Drone {drone.id}')

    def engage_offboard_mode(self, drone: Drone):
        """Switch to offboard mode."""
        self._publish_vehicle_command(
            drone, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0) # 1.0 for custom mode, 6.0 for Offboard
        self.get_logger().info(f"Switching Drone {drone.id} to offboard mode")

    def land(self, drone: Drone):
        """Switch to land mode."""
        # This command will make the drone switch to land mode and attempt to land
        self._publish_vehicle_command(drone, VehicleCommand.VEHICLE_CMD_NAV_LAND)
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
        
        #msg.target_system = drone.get_id() + 1           # Target system ID (usually 1 for PX4)
        msg.target_system = 1           # Target system ID (usually 1 for PX4)
        msg.target_component = 1        # Target component ID (usually 1 for autopilot)
        msg.source_system = 1           # Source system ID
        msg.source_component = 1        # Source component ID
        msg.from_external = True        # Command from external source (e.g., ROS 2)
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
        vehicle_offset_x = [0,-2,-4]  #  offset off initial position of each drone

        for i, drone in enumerate(self.drones):
            # Calculate angle for current drone with rotation
            current_angle = self.triangle_rotation_angle + angle_offsets[i]

            # Calculate x and y coordinates relative to the triangle center
            rel_x = self.triangle_radius * math.cos(current_angle)
            rel_y = self.triangle_radius * math.sin(current_angle)

            # Add global center offset
            target_x = self.triangle_center_x + rel_x + vehicle_offset_x[i]
            target_y = self.triangle_center_y + rel_y
            target_z = drone.takeoff_height # Maintain constant altitude

            drone.current_target_position = [target_x, target_y, target_z]

        # Increment rotation angle for next iteration
        self.triangle_rotation_angle += self.rotation_speed
        # Keep angle within [0, 2*pi)
        if self.triangle_rotation_angle >= 2 * math.pi:
            self.triangle_rotation_angle -= 2 * math.pi

    def timer_callback(self) -> None:
        """
        Callback function for the main control timer.
        Manages the state of each drone (arming, offboard, position control, landing).
        """
        all_drones_landed = True

        for drone in self.drones:
            # Publish offboard control heartbeat for each drone
            self._publish_offboard_control_heartbeat_signal(drone)

            # Only process if the drone hasn't completely landed yet
            if drone.has_landed:
                continue # Skip this drone, it's done
            else:
                all_drones_landed = False # At least one drone is still active

            # --- Offboard Mode Engagement and Arming Sequence ---
            # For the first 10 cycles, send offboard commands to allow PX4 to enter offboard mode
            if drone.offboard_setpoint_counter < 10:
                drone.offboard_setpoint_counter += 1
                # Publish initial target position for takeoff phase
                self._publish_position_setpoint(drone, 0.0, 0.0, drone.takeoff_height)
            elif drone.offboard_setpoint_counter == 10:
                self.engage_offboard_mode(drone) # Request offboard mode
                self.arm(drone)                   # Arm the drone
                drone.offboard_setpoint_counter += 1 # Increment to prevent repeated calls
                self.get_logger().info(f"Drone {drone.id}: Offboard mode and arming initiated.")

            # --- Main Control Logic (Triangle Formation or Landing) ---
            # Check if drone is in offboard mode AND not in the process of landing
            if drone.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and not drone.is_landing:
                # If current altitude is not at takeoff height, move towards it first
                if drone.is_above_takeoff_height():
                    # Move to takeoff height first
                    self._publish_position_setpoint(drone, 0.0, 0.0, drone.takeoff_height)
                    self.get_logger().info(f"Drone {drone.id}: Adjusting to takeoff height {drone.takeoff_height:.2f} m. Current altitude: {drone.vehicle_local_position.z:.2f} m")
                else:
                    # Once at takeoff height, update triangle positions and publish
                    self._update_triangle_target_positions()
                    self._publish_position_setpoint(
                        drone,
                        drone.current_target_position[0],
                        drone.current_target_position[1],
                        drone.current_target_position[2]
                    )
                    self.get_logger().debug(f"Drone {drone.id}: Moving to triangle pos {drone.current_target_position}")

            # --- Landing Logic ---
            elif drone.is_landing and not drone.has_landed:
                # Command to land if the landing service was called
                self.land(drone)
                # Check if drone has landed (e.g., very close to ground and disarmed)
                # Note: Actual disarming happens automatically after landing in PX4,
                # but we can add a check for altitude close to 0 to confirm.
                if drone.vehicle_local_position.z > -0.5: # Assuming Z=0 is ground level (NED frame)
                     self.get_logger().info(f"Drone {drone.id}: Landing in progress...")
                elif drone.vehicle_local_position.z <= -0.5 and drone.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                    self.get_logger().info(f"Drone {drone.id}: Successfully landed and disarmed.")
                    drone.has_landed = True # Mark drone as landed

        # Shut down the node if all drones have landed
        if all_drones_landed and not self.get_clock().now().nanoseconds == 0: # Avoid shutting down immediately on start
            self.get_logger().info('All drones have landed. Shutting down node.')
            rclpy.shutdown()


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
