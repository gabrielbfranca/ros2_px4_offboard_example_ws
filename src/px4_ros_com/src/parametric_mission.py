#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
import time
import math

class ParametricMission(Node):
    def __init__(self):
        super().__init__('parametric_mission')

        # Declaração dos parâmetros ROS 2
        self.get_logger().info("setting up parameters")
        self.declare_parameters(
            namespace='',
            parameters=[
                ('altitude', -5.0),
                ('velocity', 2.0),
                ('destination', [10.0, 10.0]),
                ('position_tolerance', 0.5),
                ('max_acceleration', 3.0),
                ('max_yaw_angle', 30.0),  # Max yaw angle in degrees
                ('climb_rate', 1.0),  # Rate of ascent (m/s)
                ('descent_rate', 1.0),  # Rate of descent (m/s)
                ('pid_p', 0.1),
                ('pid_i', 0.01),
                ('pid_d', 0.05)
            ]
        )

        # Publicadores
        self.get_logger().info("creating publishers")
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = "init"
        self.current_position = [0.0, 0.0, 0.0]  # Placeholder for current position

    def arm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def disarm(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)

    def set_offboard_mode(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)

    def land(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Landing initiated")
        self.state = "landing"

    def send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(time.time() * 1e6)
        self.vehicle_command_publisher.publish(msg)

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(time.time() * 1e6)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self):
        destination = self.get_parameter('destination').get_parameter_value().double_array_value
        altitude = self.get_parameter('altitude').get_parameter_value().double_value
        velocity = self.get_parameter('velocity').get_parameter_value().double_value
        max_accel = self.get_parameter('max_acceleration').get_parameter_value().double_value
        max_yaw_angle = self.get_parameter('max_yaw_angle').get_parameter_value().double_value

        msg = TrajectorySetpoint()
        msg.position = [destination[0], destination[1], altitude]
        msg.velocity = [velocity, velocity, 0.0]  # Use configured velocity
        msg.acceleration = [max_accel, max_accel, 0.0]  # Use configured acceleration
        msg.yaw = math.radians(max_yaw_angle)  # Convert angle to radians
        msg.timestamp = int(time.time() * 1e6)
        self.trajectory_setpoint_publisher.publish(msg)

    def has_reached_destination(self):
        """Check if the drone is within the tolerance of the destination."""
        destination = self.get_parameter('destination').get_parameter_value().double_array_value
        position_tolerance = self.get_parameter('position_tolerance').get_parameter_value().double_value

        distance = math.sqrt(
            (self.current_position[0] - destination[0]) ** 2 +
            (self.current_position[1] - destination[1]) ** 2
        )

        return distance <= position_tolerance

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        if self.state == "init":
            self.set_offboard_mode()
            self.state = "offboard_requested"
        elif self.state == "offboard_requested":
            self.arm()
            self.state = "armed"
        elif self.state == "armed":
            if self.has_reached_destination():
                self.get_logger().info("Destination reached! Preparing to land...")
                self.land()


def main(args=None):
    rclpy.init(args=args)
    mission = ParametricMission()
    rclpy.spin(mission)
    mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
