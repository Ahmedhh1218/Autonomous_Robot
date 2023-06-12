#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rospy
from motor_control.msg import MotorCommand  # Import the custom message type
from std_msgs.msg import Float32
from math import sin, cos
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

rospy.init_node('motor_control_publisher')  # Initialize the ROS node
rate = rospy.Rate(10)  # Set the publishing rate in Hz

class Control():
    def __init__(self):
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.speed = 0.75
        self.steer = 0.0
        self.obstacle_distance = 0.0
        self.turn_direction = 1.0  # 1.0 for right, -1.0 for left
        self.is_turning = False

        # Read the value of the "section" parameter from the ROS parameter server
        self.section = rospy.get_param("~section", "default")

        # Create ROS subscribers for the encoder_speed, heading, and obstacle_distance topics
        rospy.Subscriber('encoder_speed', Float32, self.speed_callback)
        rospy.Subscriber('heading', Float32, self.heading_callback)
        rospy.Subscriber('obstacle_distance', Float32, self.distance_callback)

        # Create a ROS publisher for the motor command
        self.pub = rospy.Publisher('/motor_command', MotorCommand, queue_size=10)

    def speed_callback(self, msg):
        # Update the x and y positions based on the speed and heading values
        self.x_pos += msg.data * cos(self.steer)
        self.y_pos += msg.data * sin(self.steer)

        # Check the section parameter to determine the desired behavior
        if self.section == "phase1":
            if self.y_pos < 5.0:
                if abs(self.x_pos) <= 0.02:
                    self.steer = 0.0
                elif self.x_pos > 0.02:
                    self.steer = -1.0
                elif self.x_pos < -0.02:
                    self.steer = 1.0
                else:
                    self.speed = 0.0
                    self.steer = 0.0
        elif self.section == "phase2":
            if self.obstacle_distance == 20.0 and not self.is_turning:
                self.is_turning = True
                self.turn_direction = -1.0
            elif self.is_turning and self.x_pos <= -0.2:
                self.is_turning = False
                self.turn_direction = 1.0
            elif self.obstacle_distance == 20.0 and not self.is_turning:
                self.is_turning = True
                self.turn_direction = 1.0
            elif self.is_turning and self.x_pos >= 0.2:
                self.is_turning = False
                self.turn_direction = -1.0

            if self.is_turning:
                self.speed = 0.75
                self.steer = self.turn_direction
            else:
                if abs(self.x_pos) <= 0.02:
                    self.steer = 0.0
                elif self.x_pos > 0.02:
                    self.steer = -1.0
                elif self.x_pos < -0.02:
                    self.steer = 1.0
                else:
                    self.speed = 0.0
                    self.steer = 0.0
        else:
            # Default behavior if the section parameter is not recognized
            self.steer = 0.0
            self.speed = 0.0

        # Create an instance of the MotorCommand message and set the values
        command = MotorCommand()
        command.speed = self.speed
        command.steer = self.steer

        # Publish the motor command
        self.pub.publish(command)

    def heading_callback(self, msg):
        self.steer = msg.data

    def distance_callback(self, msg):
        self.obstacle_distance = msg.data

def main():
    motor = Control()

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
