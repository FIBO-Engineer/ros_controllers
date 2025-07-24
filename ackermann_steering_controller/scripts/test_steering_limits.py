#!/usr/bin/env python3

"""
Simple test script to verify steering velocity limits functionality.
This script publishes steering commands and monitors the response.
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math

class SteeringLimitsTest:
    def __init__(self):
        rospy.init_node('steering_limits_test')
        
        # Publishers and subscribers
        self.cmd_pub = rospy.Publisher('/ackermann_steering_bot_controller/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/ackermann_steering_bot_controller/odom', Odometry, self.odom_callback)
        
        self.last_odom = None
        self.start_time = None
        
        rospy.loginfo("Steering limits test initialized")
    
    def odom_callback(self, msg):
        self.last_odom = msg
    
    def send_steering_command(self, linear_vel, angular_vel, duration):
        """Send a steering command for a specified duration"""
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        
        start_time = time.time()
        rate = rospy.Rate(20)  # 20 Hz
        
        rospy.loginfo(f"Sending command: linear={linear_vel}, angular={angular_vel} for {duration}s")
        
        while time.time() - start_time < duration and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            rate.sleep()
    
    def stop(self):
        """Send stop command"""
        twist = Twist()
        self.cmd_pub.publish(twist)
    
    def test_sudden_steering_change(self):
        """Test that sudden steering changes are limited"""
        rospy.loginfo("=== Testing sudden steering change ===")
        
        # Start with zero
        self.stop()
        time.sleep(2.0)
        
        # Record initial state
        initial_time = time.time()
        
        # Send a large steering command suddenly
        rospy.loginfo("Sending large steering command (angular.z = 5.0 rad/s)")
        self.send_steering_command(0.1, 5.0, 2.0)  # Small linear, large angular
        
        # Stop and analyze
        self.stop()
        
        if self.last_odom:
            rospy.loginfo(f"Final angular velocity: {self.last_odom.twist.twist.angular.z}")
            rospy.loginfo("If steering limits are working, this should be limited (< 5.0)")
        
        time.sleep(1.0)
    
    def test_gradual_steering_increase(self):
        """Test gradual steering increase to verify limits work properly"""
        rospy.loginfo("=== Testing gradual steering increase ===")
        
        # Start with zero
        self.stop()
        time.sleep(2.0)
        
        # Gradually increase steering command
        for angular_vel in [1.0, 2.0, 3.0, 4.0]:
            rospy.loginfo(f"Setting angular velocity to {angular_vel}")
            self.send_steering_command(0.1, angular_vel, 1.0)
            
            if self.last_odom:
                actual_angular = self.last_odom.twist.twist.angular.z
                rospy.loginfo(f"Actual angular velocity: {actual_angular}")
            
            time.sleep(0.5)
        
        # Stop
        self.stop()
        time.sleep(1.0)
    
    def run_tests(self):
        """Run all steering limits tests"""
        rospy.loginfo("Starting steering limits tests...")
        rospy.loginfo("Make sure the ackermann_steering_controller is running with steering limits enabled!")
        
        # Wait for controller to be ready
        rospy.loginfo("Waiting for controller...")
        time.sleep(3.0)
        
        try:
            self.test_sudden_steering_change()
            self.test_gradual_steering_increase()
            
            rospy.loginfo("=== Tests completed ===")
            rospy.loginfo("Check the log output to verify steering limits are working.")
            rospy.loginfo("With limits enabled, large commands should be gradually applied.")
            
        except KeyboardInterrupt:
            rospy.loginfo("Test interrupted by user")
        finally:
            self.stop()

if __name__ == '__main__':
    try:
        test = SteeringLimitsTest()
        test.run_tests()
    except rospy.ROSInterruptException:
        pass