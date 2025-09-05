#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck,VehicleLocalPosition, VehicleStatus
from std_srvs.srv import Trigger
from px4_msgs.srv import VehicleCommand as VehicleCommandSrv
from enum import Enum


class UAVState(Enum):
    """Enumeration for UAV states."""
    IDLE = 0
    TAKEOFF = 1
    LANDING = 2
    FLYING = 3
    EMERGENCY = 4

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self, drone_id=3, ns='px4_3') -> None:
        super().__init__(f'offboard_control_{drone_id}')

        self.get_logger().info('Offboard control node initialized.')

        # Declare and retrieve the namespace parameter
        self.declare_parameter('namespace', '')  # Default to empty namespace
        self.namespace = self.get_parameter('namespace').value
        self.ns = f'/{self.namespace}' if self.namespace else ''
        
        self.declare_parameter('MAV_SYS_ID', 1)
        self.drone_id = int( self.get_parameter('MAV_SYS_ID').value)
        #self.ns = ns
        self.get_logger().info(f"Drone {self.ns} mav sys id {self.drone_id}")



        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'{self.ns}/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'{self.ns}/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'{self.ns}/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.gcs_logger_subscriber = self.create_subscription(
            TrajectorySetpoint, f'/gcs{self.ns}/trajectory_setpoint', self.gcs_position_callback, qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, f'{self.ns}/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'{self.ns}/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_cmdResponse_subscriber = self.create_subscription(
            VehicleCommandAck, f'{self.ns}/fmu/out/vehicle_command_ack', self.vehicle_cmdResponse_callback, qos_profile)

        # Create services for takeoff and landing
        self.land_service = self.create_service(
            Trigger, f'{self.ns}/land', self.handle_land_request)
        self.takeoff_service = self.create_service(
            VehicleCommandSrv, f'{self.ns}/takeoff', self.handle_takeoff_request)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.default_takeoff_height = -5.0  # Default takeoff height
        self.command_position = [0.0, 0.0, self.takeoff_height]

        self.uav_state = UAVState.IDLE

        self.armed = False


        # Create a timer to publish control commands

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Timer for control commands created (100ms).')

    def handle_land_request(self, request, response):
        self.uav_state = UAVState.LANDING
        response.success = True
        response.message = "Land command triggered"
        self.get_logger().info("Land service called: commandLand set to True")
        return response

    def handle_takeoff_request(self, request, reply):
        self.takeoff_height = request.request.param1 if request.request.param1 else self.default_takeoff_height
        reply.reply.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        self.get_logger().info(f"Takeoff srv with height {self.takeoff_height}   ")
        self.uav_state = UAVState.TAKEOFF
        #if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.armed:
        reply.reply.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        #else:
        #    reply.reply.result = VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED
        #    self.get_logger().info("Takeoff command rejected: Vehicle not in OFFBOARD mode or not armed")
        return reply

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
    def vehicle_cmdResponse_callback(self, vehicle_cmdResponse):
        """Callback function for vehicle_command_ack topic subscriber."""
        self.get_logger().info(f"Vehicle command response: {vehicle_cmdResponse.command} result: {vehicle_cmdResponse.result}")

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
        msg.target_system = self.drone_id + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        
        #if self.uav_state == UAVState.IDLE:
        #    self.publish_position_setpoint(0.0, 0.0, 0.0)
        #    self.get_logger().info("UAV is in IDLE state, holding position at (0,0,0)")
            
        if self.uav_state == UAVState.TAKEOFF:
            self.get_logger().info("Takeoff command received, moving to takeoff height")
            
            if self.offboard_setpoint_counter == 10 :
                self.get_logger().info("Engaging offboard mode and arming the vehicle")
                self.engage_offboard_mode()
                self.arm()
            
            if self.offboard_setpoint_counter < 90:
                self.offboard_setpoint_counter += 1

            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            
            if self.vehicle_local_position.z < self.takeoff_height + 1:
                self.get_logger().info("Takeoff completed")
                self.uav_state = UAVState.FLYING

        if self.uav_state == UAVState.FLYING:
            self.get_logger().info(f"Flying to position: {self.command_position}")
            self.publish_position_setpoint(self.command_position[0], self.command_position[1], self.command_position[2])

        if self.uav_state == UAVState.LANDING:
            if (abs(self.vehicle_local_position.x) > 0.5 or abs(self.vehicle_local_position.y) > 0.5 or self.vehicle_local_position.z > self.takeoff_height + 0.5):
                self.get_logger().info(
                    "Waiting to reach landing position (0,0,{}) | Current position: x={:.2f}, y={:.2f}, z={:.2f}".format(
                        self.takeoff_height,
                        self.vehicle_local_position.x,
                        self.vehicle_local_position.y,
                        self.vehicle_local_position.z
                    )
                )
                self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
            else:
                self.get_logger().info("Landing...")
                self.land()
                self.uav_state = UAVState.IDLE
                self.offboard_setpoint_counter = 0
                exit(0)
                
            #if self.vehicle_local_position.z > -1:
            #    self.get_logger().info("Landing completed")
            #    self.disarm()

        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.uav_state == UAVState.FLYING:
            self.uav_state = UAVState.EMERGENCY
            self.get_logger().info("Emergency: Vehicle exited OFFBOARD mode during flight")

        if self.uav_state == UAVState.EMERGENCY:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)       
        
        if self.offboard_setpoint_counter == 50:
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info("Vehicle is armed")
            else:
                self.get_logger().warn("Vehicle failed to arm")
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().info("Vehicle is in OFFBOARD mode")
            else:
                self.get_logger().warn("Vehicle failed to enter OFFBOARD mode")




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
