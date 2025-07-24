#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from std_srvs.srv import Trigger


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self, drone_id=3, ns='px4_3') -> None:
        super().__init__(f'offboard_control_{drone_id}')
        self.get_logger().info('Offboard control node initialized.')

        self.drone_id = drone_id
        self.ns = ns

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'/{ns}/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'/{ns}/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'/{ns}/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.gcs_logger_subscriber = self.create_subscription(
            TrajectorySetpoint, f'/gcs/{ns}/trajectory_setpoint', self.gcs_position_callback, qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, f'/{ns}/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'/{ns}/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        self.land_service = self.create_service(
            Trigger, f'/{self.ns}/land', self.handle_land_request)
        self.takeoff_service = self.create_service(
            Trigger, f'/{self.ns}/takeoff', self.handle_takeoff_request)





        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.command_position = [0.0, 0.0, self.takeoff_height]

        self.doneTakeoff = False
        self.commandLand = False
        self.commandTakeoff = False
        self.armed = False


        # Create a timer to publish control commands

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Timer for control commands created (100ms).')

    def handle_land_request(self, request, response):
        self.commandLand = True
        response.success = True
        response.message = "Land command triggered"
        self.get_logger().info("Land service called: commandLand set to True")
        return response

    def handle_takeoff_request(self, request, response):
        self.commandTakeoff = True
        response.success = True
        response.message = "Takeoff command triggered"
        self.get_logger().info("Takeoff service called: commandTakeoff set to True")
        return response

    def gcs_position_callback(self, gcs_position):
        """Callback function for GCS position topic subscriber."""
        self.get_logger().info(f"Rcv pos: {gcs_position.position}")
        # gcs_position.position is already a list of 3 floats [x, y, z]
        # Ensure values are float type
        self.command_position[0] = float(gcs_position.position[0])
        self.command_position[1] = float(gcs_position.position[1])
        self.command_position[2] = float(gcs_position.position[2])

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')
        self.armed = True

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')
        self.armed = False

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def takeoff(self):
        """Send a takeoff command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7= self.takeoff_height)
        self.get_logger().info("Takeoff command sent")
    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        # Ensure all values are explicitly float type
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = self.drone_id
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10 and self.commandTakeoff:
            self.get_logger().info("Engaging offboard mode and arming the vehicle")
            self.engage_offboard_mode()
            self.arm()
        
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.armed:
            if not self.doneTakeoff:
                #self.takeoff()
                self.publish_position_setpoint(0.0,0.0, self.takeoff_height)

                if self.vehicle_local_position.z < self.takeoff_height+ 1:
                    self.doneTakeoff = True
            else:
                if not self.commandLand:
                    self.publish_position_setpoint(self.command_position[0], self.command_position[1], self.command_position[2])
                else:
                    self.get_logger().info("Landing command received, moving to landing position")
                    self.publish_position_setpoint(0.0,0.0, self.takeoff_height)
                    if abs(self.vehicle_local_position.x) > 0.5 or abs(self.vehicle_local_position.y) > 0.2 or abs(self.vehicle_local_position.z - self.takeoff_height) > 0.5:
                        self.get_logger().info("Waiting to reach landing position (0,0,{})".format(self.takeoff_height))
                        self.land()
                        if self.vehicle_local_position.z > -1:
                            self.get_logger().info("Landing completed")
                            self.commandTakeoff = False
                            self.commandLand = False
                            self.doneTakeoff = False
                            self.offboard_setpoint_counter = 0
                            self.armed = False
                            exit(0)

        if self.offboard_setpoint_counter < 11 and self.commandTakeoff:
            self.offboard_setpoint_counter += 1


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
