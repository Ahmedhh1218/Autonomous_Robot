#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import threading
import rospy
from std_msgs.msg import Float32
import math

# Set GPIO pin numbers
encoder_pin_A = 5
encoder_pin_B = 6

PULSE_PER_REV = 10
WheelRadius = 0.0225

# Set initial pulse count and timestamp
past_pulse_count = 0
past_time = time.time()
current_pulse_count = 0
pps = 0

# Callback function for encoder pulse detection
def encoderA_callback(channel):
    global current_pulse_count
    current_pulse_count += 1

def encoderB_callback(channel):
    global current_pulse_count
    current_pulse_count += 1

# Function to calculate PPS
def calculate_pps():
    global current_pulse_count, past_pulse_count, past_time, pps
    
    current_time = time.time()

    # Calculate the time difference in seconds
    time_diff = current_time - past_time

    # Calculate the pulse per second (PPS)
    pps = (current_pulse_count - past_pulse_count) / (time_diff*2)

    # Update the past count and timestamp for the next calculation
    past_pulse_count = current_pulse_count
    past_time = current_time
   
    current_pulse_count = past_pulse_count
    threading.Timer(0.1, calculate_pps).start()

if __name__ == '__main__':
    
    # Set GPIO mode and setup event detection
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(encoder_pin_A, GPIO.IN)
    GPIO.add_event_detect(encoder_pin_A, GPIO.RISING, callback=encoderA_callback)
    GPIO.setup(encoder_pin_B, GPIO.IN)
    GPIO.add_event_detect(encoder_pin_B, GPIO.RISING, callback=encoderB_callback)

    # Create a timer that calls the calculate_pps function every 1 millisecond
    interval = 0.001  # 1 millisecond
    threading.Timer(0.1, calculate_pps).start()

    rospy.init_node('encoder_speed_node')
    pub = rospy.Publisher('encoder_speed', Float32, queue_size = 10)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        speed = (pps / PULSE_PER_REV) * 2 * math.pi * WheelRadius
        print("speed: ", speed)
        pub.publish(speed)
        rate.sleep()
    print('close')

    GPIO.cleanup()

