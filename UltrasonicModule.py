#!/usr/bin/env python3

#Libraries
import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Float32

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

 
#set GPIO Pins
GPIO_TRIGGER = 15
GPIO_ECHO = 14
 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('obstacle_distance_publisher', anonymous=True)

        # Create a publisher with topic name "obstacle_distance" and message type Float32
        pub = rospy.Publisher('obstacle_distance', Float32, queue_size=10)

        # Set the publishing rate
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            dist = distance()

            # Publish the distance on the topic
            pub.publish(dist)

            rospy.loginfo("Measured Distance = %.1f cm" % dist)

            rate.sleep()

        # Reset by pressing CTRL + C (no change)
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()