#!/usr/bin/env python3
"""
Test flight script: Takeoff, hover for 10 seconds, and land.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus
import time


class FlightTestNode(Node):
    """ROS 2 node for testing drone takeoff and landing."""

    def __init__(self):
        super().__init__('flight_test_node')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0
        self.takeoff_height = -5.0  # 5 meters (NED frame, negative is up)

        # Create timer for control loop (100 Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.get_logger().info('Flight test node initialized')

    def vehicle_status_callback(self, msg):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = msg

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Switching to offboard mode')

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Switching to land mode')

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def publish_offboard_control_mode(self):
        """Publish offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=-5.0):
        """Publish trajectory setpoint (NED coordinates)."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def timer_callback(self):
        """Callback function for the timer."""
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint(0.0, 0.0, self.takeoff_height)
        self.offboard_setpoint_counter += 1


def run_flight_test():
    """Run the complete flight test sequence."""
    rclpy.init()

    flight_test_node = FlightTestNode()

    print("Starting flight test...")
    print("Step 1: Publishing offboard setpoints...")

    # Publish a few setpoints before starting offboard mode
    for _ in range(10):
        rclpy.spin_once(flight_test_node)
        time.sleep(0.1)

    print("Step 2: Arming vehicle...")
    flight_test_node.arm()
    time.sleep(2)

    print("Step 3: Engaging offboard mode...")
    flight_test_node.engage_offboard_mode()
    time.sleep(2)

    print("Step 4: Taking off...")
    start_time = time.time()

    # Continue publishing setpoints and spinning for 10 seconds
    while time.time() - start_time < 10.0:
        rclpy.spin_once(flight_test_node, timeout_sec=0.01)
        elapsed = time.time() - start_time
        if int(elapsed) != int(elapsed - 0.1):  # Log every second
            print(f"Hovering... {elapsed:.1f}s / 10.0s")

    print("Step 5: Landing...")
    flight_test_node.land()

    # Continue spinning for a few more seconds to ensure landing
    land_start = time.time()
    while time.time() - land_start < 15.0:
        rclpy.spin_once(flight_test_node, timeout_sec=0.01)

    print("Step 6: Disarming...")
    flight_test_node.disarm()
    time.sleep(1)

    print("Flight test completed successfully!")

    flight_test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        run_flight_test()
    except KeyboardInterrupt:
        print("\nFlight test interrupted by user")
    except Exception as e:
        print(f"Error during flight test: {e}")
        import traceback
        traceback.print_exc()
