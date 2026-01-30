#!/usr/bin/env python3
"""
ROS2 Control Hardware Interface Validator
Validates sensor readings, actuator commands, and hardware interface reliability
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import math
from collections import deque


class ControlValidator(Node):
    def __init__(self):
        super().__init__('control_validator')
        
        # Robot parameters
        self.declare_parameter('wheelbase', 0.6)
        self.declare_parameter('wheel_radius', 0.25)
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('max_velocity', 2.0)
        
        self.wheelbase = self.get_parameter('wheelbase').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.max_velocity = self.get_parameter('max_velocity').value
        
        # Data storage
        self.joint_states = None
        self.last_joint_time = None
        self.command = None
        self.last_command_time = None
        self.odom = None
        
        # Statistics
        self.update_times = deque(maxlen=100)
        self.command_errors = []
        self.validation_count = 0
        self.error_count = 0
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.cmd_sub = self.create_subscription(
            TwistStamped, '/ackermann_steering_controller/reference',
            self.command_callback, 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, '/ackermann_steering_controller/odometry',
            self.odom_callback, 10)
        
        # Validation timer
        self.timer = self.create_timer(2.0, self.validate_and_report)
        
        self.get_logger().info('='*60)
        self.get_logger().info('ROS2 Control Hardware Validator Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Wheelbase: {self.wheelbase} m')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius} m')
        self.get_logger().info('Monitoring hardware interfaces...\n')

    def joint_state_callback(self, msg):
        current_time = self.get_clock().now()
        if self.last_joint_time is not None:
            dt = (current_time - self.last_joint_time).nanoseconds / 1e9
            self.update_times.append(dt)
        
        self.joint_states = msg
        self.last_joint_time = current_time
        
    def command_callback(self, msg):
        self.command = msg
        self.last_command_time = self.get_clock().now()
        
    def odom_callback(self, msg):
        self.odom = msg

    def validate_and_report(self):
        """Main validation routine with detailed reporting"""
        self.validation_count += 1
        
        print('\n' + '='*60)
        print(f'VALIDATION REPORT #{self.validation_count}')
        print('='*60)
        
        # 1. Check data freshness
        if not self.check_data_freshness():
            self.error_count += 1
            return
        
        # 2. Validate joint states
        self.validate_joint_states()
        
        # 3. Validate command execution
        if self.command is not None:
            self.validate_command_execution()
        
        # 4. Validate odometry
        if self.odom is not None:
            self.validate_odometry()
        
        # 5. Performance metrics
        self.print_performance_metrics()
        
        print('='*60 + '\n')

    def check_data_freshness(self):
        """Check that data is being received and is recent"""
        current_time = self.get_clock().now()
        
        print('\n📡 DATA FRESHNESS CHECK:')
        
        # Joint states check
        if self.last_joint_time is None:
            print('  ❌ No joint states received')
            return False
        
        time_since_joint = (current_time - self.last_joint_time).nanoseconds / 1e9
        if time_since_joint > 0.5:
            print(f'  ❌ Joint states stale ({time_since_joint:.3f}s old)')
            return False
        else:
            print(f'  ✅ Joint states fresh ({time_since_joint*1000:.1f}ms old)')
        
        # Command check
        if self.last_command_time is not None:
            time_since_cmd = (current_time - self.last_command_time).nanoseconds / 1e9
            print(f'  ✅ Commands fresh ({time_since_cmd*1000:.1f}ms old)')
        else:
            print('  ⚠️  No commands received yet')
        
        return True

    def validate_joint_states(self):
        """Validate joint state values are physically reasonable"""
        print('\n🔧 JOINT STATE VALIDATION:')
        
        js = self.joint_states
        all_valid = True
        
        # Front steering joints
        print('  Front Steering Joints:')
        for i, name in enumerate(js.name):
            if 'front' in name and i < len(js.position):
                angle_rad = js.position[i]
                angle_deg = math.degrees(angle_rad)
                
                if abs(angle_rad) > self.max_steering_angle:
                    print(f'    ❌ {name}: {angle_deg:.1f}° ({angle_rad:.3f} rad) - EXCEEDS LIMIT')
                    all_valid = False
                else:
                    print(f'    ✅ {name}: {angle_deg:.1f}° ({angle_rad:.3f} rad)')
        
        # Rear wheel velocities
        print('  Rear Wheel Velocities:')
        for i, name in enumerate(js.name):
            if 'rear' in name and i < len(js.velocity):
                angular_vel = js.velocity[i]
                linear_vel = angular_vel * self.wheel_radius
                
                print(f'    {name}:')
                print(f'      Angular: {angular_vel:.3f} rad/s')
                print(f'      Linear:  {linear_vel:.3f} m/s')
                
                if abs(linear_vel) > self.max_velocity:
                    print(f'      ❌ EXCEEDS LIMIT ({self.max_velocity} m/s)')
                    all_valid = False
                else:
                    print(f'      ✅ Within limits')
        
        if all_valid:
            print('  ✅ All joint states valid')
        
        return all_valid

    def validate_command_execution(self):
        """Validate commanded velocities match actual outputs"""
        print('\n⚙️  COMMAND EXECUTION VALIDATION:')
        
        cmd_linear = self.command.twist.linear.x
        cmd_angular = self.command.twist.angular.z
        
        print(f'  Commanded:')
        print(f'    Linear velocity:  {cmd_linear:.3f} m/s')
        print(f'    Angular velocity: {cmd_angular:.3f} rad/s')
        
        # Calculate expected values
        expected_wheel_vel = cmd_linear / self.wheel_radius
        
        if abs(cmd_linear) > 0.01:
            expected_steering = math.atan((cmd_angular * self.wheelbase) / cmd_linear)
        else:
            expected_steering = 0.0
        
        print(f'  Expected:')
        print(f'    Wheel angular vel: {expected_wheel_vel:.3f} rad/s')
        print(f'    Steering angle:    {math.degrees(expected_steering):.1f}° ({expected_steering:.3f} rad)')
        
        # Compare with actual
        js = self.joint_states
        
        # Check rear wheels
        print(f'  Actual:')
        for i, name in enumerate(js.name):
            if 'rear' in name and i < len(js.velocity):
                actual_vel = js.velocity[i]
                error = abs(actual_vel - expected_wheel_vel)
                error_pct = (error / max(abs(expected_wheel_vel), 0.01)) * 100
                
                if error > 0.5:
                    print(f'    ❌ {name}: {actual_vel:.3f} rad/s (error: {error:.3f}, {error_pct:.1f}%)')
                else:
                    print(f'    ✅ {name}: {actual_vel:.3f} rad/s (error: {error:.3f}, {error_pct:.1f}%)')
        
        # Check steering
        for i, name in enumerate(js.name):
            if 'front' in name and i < len(js.position):
                actual_steering = js.position[i]
                error = abs(actual_steering - expected_steering)
                error_deg = math.degrees(error)
                
                if error > 0.1:
                    print(f'    ❌ {name}: {math.degrees(actual_steering):.1f}° (error: {error_deg:.1f}°)')
                else:
                    print(f'    ✅ {name}: {math.degrees(actual_steering):.1f}° (error: {error_deg:.1f}°)')

    def validate_odometry(self):
        """Validate odometry consistency"""
        print('\n🗺️  ODOMETRY VALIDATION:')
        
        odom_linear = self.odom.twist.twist.linear.x
        odom_angular = self.odom.twist.twist.angular.z
        
        print(f'  Odometry velocity:')
        print(f'    Linear:  {odom_linear:.3f} m/s')
        print(f'    Angular: {odom_angular:.3f} rad/s')
        
        if self.command is not None:
            cmd_linear = self.command.twist.linear.x
            cmd_angular = self.command.twist.angular.z
            
            linear_error = abs(odom_linear - cmd_linear)
            angular_error = abs(odom_angular - cmd_angular)
            
            if linear_error < 0.1:
                print(f'    ✅ Linear matches command (error: {linear_error:.3f} m/s)')
            else:
                print(f'    ⚠️  Linear differs from command (error: {linear_error:.3f} m/s)')
            
            if angular_error < 0.1:
                print(f'    ✅ Angular matches command (error: {angular_error:.3f} rad/s)')
            else:
                print(f'    ⚠️  Angular differs from command (error: {angular_error:.3f} rad/s)')

    def print_performance_metrics(self):
        """Print real-time performance metrics"""
        print('\n📊 PERFORMANCE METRICS:')
        
        if len(self.update_times) > 0:
            avg_update = sum(self.update_times) / len(self.update_times)
            max_update = max(self.update_times)
            min_update = min(self.update_times)
            update_rate = 1.0 / avg_update if avg_update > 0 else 0
            
            print(f'  Joint state update rate: {update_rate:.1f} Hz')
            print(f'  Update period: avg={avg_update*1000:.2f}ms, '
                  f'min={min_update*1000:.2f}ms, max={max_update*1000:.2f}ms')
            
            if max_update > 0.15:  # 150ms
                print(f'    ⚠️  Maximum update period high ({max_update*1000:.1f}ms)')
            else:
                print(f'    ✅ Update timing consistent')
        
        print(f'  Total validations: {self.validation_count}')
        print(f'  Errors detected: {self.error_count}')
        success_rate = ((self.validation_count - self.error_count) / 
                       max(self.validation_count, 1)) * 100
        print(f'  Success rate: {success_rate:.1f}%')


def main(args=None):
    rclpy.init(args=args)
    validator = ControlValidator()
    
    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        print('\n\n' + '='*60)
        print('FINAL VALIDATION SUMMARY')
        print('='*60)
        print(f'Total validations performed: {validator.validation_count}')
        print(f'Total errors detected: {validator.error_count}')
        success_rate = ((validator.validation_count - validator.error_count) / 
                       max(validator.validation_count, 1)) * 100
        print(f'Overall success rate: {success_rate:.1f}%')
        print('='*60)
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()