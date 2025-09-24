#!/usr/bin/env python3
# this use gotosetpoint for goto waypoints
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TelemetryStatus,OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleCommandAck,VehicleLocalPosition, VehicleStatus,GotoSetpoint
from std_srvs.srv import Trigger
from px4_msgs.srv import VehicleCommand as VehicleCommandSrv
from enum import Enum

def fmt_float(val):
    """Format a float to 2 decimal places as string."""
    return f"{val:.2f}"

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
        self.goto_setpoint_publisher = self.create_publisher(
            GotoSetpoint, f'{self.ns}/fmu/in/goto_setpoint', qos_profile)
        self.telemetry_publisher = self.create_publisher(
            TelemetryStatus, f'{self.ns}/fmu/in/telemetry_status', qos_profile)

        # Create subscribers for GCS commands
        self.gcs_logger_subscriber = self.create_subscription(
            TrajectorySetpoint, f'{self.ns}/fmu/in/trajectory_setpoint_offboard', self.gcs_position_callback, qos_profile)
        # Create subscribers of vehicle topics
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, f'{self.ns}/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'{self.ns}/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_cmdResponse_subscriber = self.create_subscription(
            VehicleCommandAck, f'{self.ns}/fmu/out/vehicle_command_ack', self.vehicle_cmdResponse_callback, qos_profile)
        
        # create subscriber for vehicle_command_offboard
        self.vehicle_command_offboard_subscriber = self.create_subscription(
            VehicleCommand, f'{self.ns}/fmu/in/vehicle_command_offboard', self.vehicle_cmd_offboard_callback, qos_profile)
        #create a subscriber for vehicle_command_offboard_ack
        self.vehicle_command_offboard_ack_publisher = self.create_publisher(
            VehicleCommandAck, f'{self.ns}/fmu/out/vehicle_command_offboard_ack', qos_profile)

        # Create services for takeoff and landing
        self.vehicle_cmd_offboard_service = self.create_service(
            VehicleCommandSrv, f'{self.ns}/vehicle_command_offboard', self.handle_vehicle_cmd_request)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.waypoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.takeoff_height = -5.0
        self.default_takeoff_height = -5.0  # Default takeoff height
        self.command_position = [0.0, 0.0, self.takeoff_height]
        self.commanded_position = [0.0, 0.0, 0.0]
        self.takeoff_point = [0.0, 0.0, 0.0]

        self.uav_state = UAVState.IDLE


        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Timer for control commands created (100ms).')

    def vehicle_cmd_offboard_callback(self, vehicle_command):
        """Callback function for vehicle_command_offboard topic subscriber."""
        self.get_logger().info(f"Rcv vehicle command offboard: {vehicle_command.command} param1: {vehicle_command.param1} param2: {vehicle_command.param2} param3: {vehicle_command.param3} param4: {vehicle_command.param4} param5: {vehicle_command.param5} param6: {vehicle_command.param6} param7: {vehicle_command.param7}")

        if vehicle_command.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            if vehicle_command.param1 == 1.0:
                self.arm()
            elif vehicle_command.param1 == 0.0:
                self.disarm()
        
        elif vehicle_command.command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            if vehicle_command.param1 == 1.0 and vehicle_command.param2 == 6.0:
                self.engage_offboard_mode()
        
        elif vehicle_command.command == VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF:
            height = vehicle_command.param7 if vehicle_command.param7 != 0.0 else self.default_takeoff_height
            self.takeoff_height = height
            self.takeoff_point = [self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z]
            self.get_logger().info(f"Takeoff command received with height {self.takeoff_height}")
            self.uav_state = UAVState.TAKEOFF
        
        elif vehicle_command.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            self.get_logger().info("Land command received")
            self.uav_state = UAVState.LANDING
        else:
            self.get_logger().info(f"Command {vehicle_command.command} not handled in offboard control")
            self.publish_vehicle_command(vehicle_command.command, param1=vehicle_command.param1, param2=vehicle_command.param2, param3=vehicle_command.param3, param4=vehicle_command.param4, param5=vehicle_command.param5, param6=vehicle_command.param6, param7=vehicle_command.param7)


    def handle_vehicle_cmd_request(self, request, reply):
        if request.request.command == VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF:
            return self.handle_takeoff_request(request, reply)
        elif request.request.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            return self.handle_land_request(request, reply)
        else:
            reply.reply.command = request.request.command
            reply.reply.result = VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED
            self.get_logger().info(f"Vehicle command offboard {request.request.command} not supported")
            return reply

    def handle_land_request(self, request, reply):
        self.uav_state = UAVState.LANDING
        reply.reply.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        reply.reply.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        return reply

    def handle_takeoff_request(self, request, reply):
        self.takeoff_height = request.request.param1 if request.request.param1 else self.default_takeoff_height
        self.takeoff_point = [self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z]
        self.get_logger().info(f"Takeoff srv with height {self.takeoff_height}   ")
        self.uav_state = UAVState.TAKEOFF
        reply.reply.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        reply.reply.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
        return reply

    def gcs_position_callback(self, gcs_position):
        """Callback function for GCS position topic subscriber."""
        # gcs_position.position is already a list of 3 floats [x, y, z]
        self.command_position[0] = float(gcs_position.position[0])
        self.command_position[1] = float(gcs_position.position[1])
        self.command_position[2] = float(gcs_position.position[2])


    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def finish_takeoff(self, success: bool = True):
        response = VehicleCommandAck()
        response.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        response.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED if success else VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED
        self.vehicle_command_offboard_ack_publisher.publish(response)

    def finish_landing(self, success: bool = True):
        response = VehicleCommandAck()
        response.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        response.result = VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED if success else VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED
        self.vehicle_command_offboard_ack_publisher.publish(response)

    def vehicle_cmdResponse_callback(self, vehicle_cmdResponse):
        """Callback function for vehicle_command_ack topic subscriber."""
        self.get_logger().info(f"Cmd response: {vehicle_cmdResponse.command} result: {vehicle_cmdResponse.result}")
        if vehicle_cmdResponse.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            self.vehicle_command_offboard_ack_publisher.publish(vehicle_cmdResponse)
            if vehicle_cmdResponse.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                self.get_logger().info("Vehicle armed successfully")
            else:
                self.get_logger().warn("Vehicle failed to arm")
        if vehicle_cmdResponse.command == VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF:
            self.vehicle_command_offboard_ack_publisher.publish(vehicle_cmdResponse)
            if vehicle_cmdResponse.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                self.get_logger().info("Takeoff command accepted")
            else:
                self.get_logger().warn("Takeoff command failed")
                self.uav_state = UAVState.IDLE
        if vehicle_cmdResponse.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
            self.vehicle_command_offboard_ack_publisher.publish(vehicle_cmdResponse)
            if vehicle_cmdResponse.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED:
                self.get_logger().info("Land command accepted")
            else:
                self.get_logger().warn("Land command failed")
                self.uav_state = UAVState.FLYING
    

    def telemetry_publish(self):
        msg = TelemetryStatus()

        # Tiempo en microsegundos (timestamp relativo al boot del sistema)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # Tipo de enlace
        msg.type = TelemetryStatus.LINK_TYPE_WIRE   # Ej: conexión cableada
        msg.mode = 0

        # Configuración básica
        msg.flow_control = True
        msg.forwarding = True
        msg.mavlink_v2 = True
        msg.ftp = True
        msg.streams = 0

        # Parámetros de data rate simulados
        msg.data_rate = 57600.0
        msg.rate_multiplier = 1.0
        msg.tx_rate_avg = 2000.0
        msg.tx_error_rate_avg = 0.0
        msg.tx_message_count = 100
        msg.tx_buffer_overruns = 0
        msg.rx_rate_avg = 2000.0
        msg.rx_message_count = 100
        msg.rx_message_lost_count = 0
        msg.rx_buffer_overruns = 0
        msg.rx_parse_errors = 0
        msg.rx_packet_drop_count = 0
        msg.rx_message_lost_rate = 0.0

        # Heartbeats → aquí es donde indicamos que hay una GCS
        msg.heartbeat_type_gcs = True

        # Otros heartbeats opcionales
        msg.heartbeat_type_onboard_controller = False
        msg.heartbeat_type_camera = False
        msg.heartbeat_component_telemetry_radio = False
        msg.heartbeat_component_udp_bridge = False

        # Estado de sistemas extra
        msg.open_drone_id_system_healthy = True
        msg.parachute_system_healthy = True

        # Publicar
        self.telemetry_publisher.publish(msg)
        #self.get_logger().info(
        #    f"Publicado TelemetryStatus → type={msg.type}, GCS={msg.heartbeat_type_gcs}"
        #)

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def takeoff(self, altitude: float = 5.0):
        """Send a takeoff command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,param1=1.0,param7=altitude)
        self.get_logger().info(f"Takeoff command sent with altitude: {altitude}")

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

    def publish_position_setpoint(self, x: float, y: float, z: float,vx:float =float('nan'), vy:float =float('nan'), vz:float =float('nan')) -> None:
        """Publish the trajectory setpoint."""
        self.commanded_position[0] = x
        self.commanded_position[1] = y
        self.commanded_position[2] = z

        msg = TrajectorySetpoint()
        # Ensure all values are explicitly float type
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        #self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")
    
    def publish_goto_setpoint(self, x: float, y: float, z: float) -> None:
        msg = GotoSetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.heading = 1.57079  # (90 degree)
        msg.max_horizontal_speed = 2.0
        msg.max_vertical_speed = 1.0
        msg.max_heading_rate = 30.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.goto_setpoint_publisher.publish(msg)

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
        self.telemetry_publish()

        #self.get_logger().info(
        #    f"M: {self.uav_state} - s: {self.vehicle_status.nav_state} - Cmded: [{fmt_float(self.commanded_position[0])}, {fmt_float(self.commanded_position[1])}, {fmt_float(self.commanded_position[2])}] "
        #    f"- Real: [{fmt_float(self.vehicle_local_position.x)}, {fmt_float(self.vehicle_local_position.y)}, {fmt_float(self.vehicle_local_position.z)}]"
        #)

        if self.uav_state == UAVState.IDLE:
            self.uav_state = UAVState.TAKEOFF
            #self.publish_position_setpoint(0.0, 0.0, 0.0)
            #self.get_logger().info("UAV is in IDLE state, holding position at (0,0,0)")

        if self.uav_state == UAVState.TAKEOFF:
            #self.get_logger().info("Takeoff command received, moving to takeoff height")
            if self.offboard_setpoint_counter == 10 :
                self.get_logger().info("Engaging offboard mode and arming the vehicle")
                self.engage_offboard_mode()
            if self.offboard_setpoint_counter == 11 :
                self.arm()
                self.publish_goto_setpoint(self.takeoff_point[0], self.takeoff_point[1], - 1.0)
            
            if self.offboard_setpoint_counter == 14 :
                if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                    self.get_logger().info("Vehicle is in OFFBOARD mode and armed, proceeding with takeoff")
                else:
                    self.get_logger().warn("Vehicle is not in OFFBOARD mode or not armed yet, waiting...")

            if self.offboard_setpoint_counter > 14 :
                self.publish_goto_setpoint(self.takeoff_point[0], self.takeoff_point[1], -10 )

            if self.vehicle_local_position.z < -10 + 0.5:
                self.get_logger().info("Takeoff completed")
                self.uav_state = UAVState.FLYING

            if self.offboard_setpoint_counter < 90:
                self.offboard_setpoint_counter += 1

        if self.uav_state == UAVState.FLYING:
            if self.waypoint_counter == 0:
                self.publish_goto_setpoint(5.0, 0.0, -10.0)
                if (abs(self.vehicle_local_position.x - 5.0) < 0.5 and abs(self.vehicle_local_position.y) < 0.5 and abs(self.vehicle_local_position.z + 10.0) < 0.5):
                    self.get_logger().info("Reached waypoint (5,0,-10), hovering for 5 seconds")
                    self.waypoint_counter += 1
        if self.waypoint_counter == 1:
            self.publish_goto_setpoint(5.0, 5.0, -10.0)
            if (abs(self.vehicle_local_position.x - 5.0) < 0.5 and abs(self.vehicle_local_position.y - 5.0) < 0.5 and abs(self.vehicle_local_position.z + 10.0) < 0.5):
                self.get_logger().info("Reached waypoint (5,5,-10), hovering for 5 seconds")
                self.waypoint_counter += 1
        if self.waypoint_counter == 2:
            self.publish_goto_setpoint(0.0, 5.0, -10.0)
            if (abs(self.vehicle_local_position.x) < 0.5 and abs(self.vehicle_local_position.y - 5.0) < 0.5 and abs(self.vehicle_local_position.z + 10.0) < 0.5):
                self.get_logger().info("Reached waypoint (0,5,-10), hovering for 5 seconds")
                self.waypoint_counter += 1
        if self.waypoint_counter == 3:
            self.publish_goto_setpoint(0.0, 0.0, -10.0)
            if (abs(self.vehicle_local_position.x) < 0.5 and abs(self.vehicle_local_position.y) < 0.5 and abs(self.vehicle_local_position.z + 10.0) < 0.5):
                self.get_logger().info("Reached waypoint (0,0,-10), hovering for 5 seconds")
                self.waypoint_counter = 0
                self.uav_state = UAVState.LANDING


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
                self.publish_goto_setpoint(0.0, 0.0, self.takeoff_height)
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
