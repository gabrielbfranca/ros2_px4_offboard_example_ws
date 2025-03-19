#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, VehicleStatus
import sys
import termios
import tty
import threading

class KeyboardDroneControl(Node):
    def __init__(self):
        super().__init__('keyboard_drone_control')

        # Publishers
        self.trajectory_publisher = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        # Subscriber to check vehicle status
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, 10)

        self.offboard_mode_enabled = False
        self.armed = False

        self.get_logger().info("Keyboard control node started. Initializing...")

        # Drone movement parameters
        self.velocity = 5.0  # m/s
        self.altitude_step = 0.5  # m
        self.position = [0.0, 0.0, -5.0] 

        # Timers for OFFBOARD mode and trajectory publishing
        self.timer = self.create_timer(0.1, self.send_trajectory_setpoint)  
        self.timer2 = self.create_timer(0.5, self.arm_and_offboard)

        # Start keyboard thread
        self.running = True
        self.keyboard_thread = threading.Thread(target=self.keyboard_control_loop)
        self.keyboard_thread.start()

    def vehicle_status_callback(self, msg):
        """Check if drone is in OFFBOARD mode and armed."""
        if msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.offboard_mode_enabled = True
            self.get_logger().info("OFFBOARD mode enabled")
        if msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.armed = True
            self.get_logger().info("Drone armed")

    def send_trajectory_setpoint(self):
        """Publish trajectory setpoints only when the position changes."""
        msg = TrajectorySetpoint()
        msg.position = self.position
        msg.yaw = 0.0
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # Convert to microseconds
        self.trajectory_publisher.publish(msg)

        # Log only when a new position is received via keyboard
        if hasattr(self, 'last_position') and self.last_position != self.position:
            self.get_logger().info(f"📡 New trajectory setpoint: {self.position}")
    
        # Store last position
        self.last_position = list(self.position)


    def send_vehicle_command(self, command, param1=0.0):
        """Send vehicle commands like takeoff, land, arm, and switch modes, but only when necessary."""
        
        # Avoid spamming the same command
        if command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE and self.current_mode == param1:
            return  # Skip if already in this mode
        if command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM and self.armed == bool(param1):
            return  # Skip if already armed/disarmed
    
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # Convert to microseconds
        
        self.command_publisher.publish(msg)
    
        # Update internal state tracking
        if command == VehicleCommand.VEHICLE_CMD_DO_SET_MODE:
            self.current_mode = param1
            self.get_logger().info(f"🛠 Switched to mode: {param1}")
        elif command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
            self.armed = bool(param1)
            self.get_logger().info(f"🔧 Drone {'Armed' if self.armed else 'Disarmed'}")


    def arm_and_offboard(self):
        """Attempt to arm and switch to OFFBOARD mode (retries every 0.5s)."""
        if not self.armed:
            self.get_logger().info("Arming drone...")
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

        if not self.offboard_mode_enabled:
            self.get_logger().info("Setting OFFBOARD mode...")
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # 6 = OFFBOARD mode

    def get_key(self):
        """Read a single key press from the terminal (non-blocking)."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def keyboard_control_loop(self):
        """Continuously read keyboard input and move the drone."""
        self.get_logger().info("Keyboard control active! Use WASD to move, Q/E for altitude, T/G for takeoff/land.")

        while self.running:
            key = self.get_key()

            if key == 'w':  # Move forward
                self.position[0] += self.velocity
            elif key == 's':  # Move backward
                self.position[0] -= self.velocity
            elif key == 'a':  # Move left
                self.position[1] -= self.velocity
            elif key == 'd':  # Move right
                self.position[1] += self.velocity
            elif key == 'q':  # Increase altitude
                self.position[2] -= self.altitude_step
            elif key == 'e':  # Decrease altitude
                self.position[2] += self.altitude_step
            elif key == 't':  # Takeoff
                self.get_logger().info("Takeoff command sent!")
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, 5.0)
            elif key == 'g':  # Land
                self.get_logger().info("Landing command sent!")
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            elif key == ' ':  # Emergency stop
                self.get_logger().info("Emergency stop!")
                self.position = [self.position[0], self.position[1], self.position[2]]  # Stop movement
            elif key == '\x03':  # Ctrl+C to exit
                self.running = False
                break

            # Publish new trajectory setpoint
            self.send_trajectory_setpoint()
            self.get_logger().info(f"New position -> X: {self.position[0]}, Y: {self.position[1]}, Z: {self.position[2]}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardDroneControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
