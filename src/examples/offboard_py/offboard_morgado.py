#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TelemetryStatus, OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck, VehicleLocalPosition, VehicleStatus, GotoSetpoint, VehicleGlobalPosition


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self, drone_id=3, ns='px4_3') -> None:

        super().__init__(f'offboard_control_{drone_id}')

        self.get_logger().info(' Offboard control node initialized.')

        # Declare and retrieve the namespace parameter
        self.declare_parameter('namespace', '')  # Default to empty namespace
        self.namespace = self.get_parameter('namespace').value
        self.ns = f'/{self.namespace}' if self.namespace else ''

        self.declare_parameter('MAV_SYS_ID', 1)
        self.drone_id = int(self.get_parameter('MAV_SYS_ID').value)

        self.get_logger().info(f"🚀 Drone {self.ns} mav sys id {self.drone_id}")

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

        # Create subscribers
        self.gcs_logger_subscriber = self.create_subscription(
            TrajectorySetpoint, f'{self.ns}/fmu/in/trajectory_setpoint_offboard', self.gcs_trajectory_setpoint_callback, qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, f'{self.ns}/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'{self.ns}/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        # Initialize variables
        self.vehicle_pos_cmd = TrajectorySetpoint()
        self.ready = False
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Timer for control commands created (100ms).')


    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
    
    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        if not self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.vehicle_pos_cmd.position[0] = vehicle_local_position.x
            self.vehicle_pos_cmd.position[1] = vehicle_local_position.y
            self.vehicle_pos_cmd.position[2] = vehicle_local_position.z
            self.ready = True



    def gcs_trajectory_setpoint_callback(self, msg: TrajectorySetpoint):
        """Callback function for GCS position topic subscriber."""
        self.vehicle_pos_cmd=msg

    def offboard_heartbet(self) :
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)


    def timer_callback(self) -> None:
        """Callback function for the timer."""
        if not self.ready:
            return
        self.offboard_heartbet()
        self.trajectory_setpoint_publisher.publish(self.vehicle_pos_cmd)

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