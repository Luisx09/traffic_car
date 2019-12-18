#!/usr/bin/env python
import rospy
from picar import front_wheels, back_wheels
from picar.SunFounder_PCA9685 import Servo
import picar
from time import sleep
import numpy as np
from std_msgs.msg import Float64

bw = back_wheels.Back_Wheels()
fw = front_wheels.Front_Wheels()
pan_servo = Servo.Servo(2)
tilt_servo = Servo.Servo(1)
picar.setup()

# Keep unused servos at static positions
bw.speed = 0
fw.turn(90)
pan_servo.write(45)
tilt_servo.write(90)

# Constant for motor dead zone (Value range where movement is mostly negligible)
DEAD_ZONE = 10.0

def fw_movement(data):
    curr_speed = data.data
    
    if abs(curr_speed) < DEAD_ZONE:
        if curr_speed >= (DEAD_ZONE / 2.0):
            curr_speed = DEAD_ZONE
        elif curr_speed > (-1.0 * DEAD_ZONE / 2.0):
            curr_speed = 0
        else:
            curr_speed = -1.0 * DEAD_ZONE

    if curr_speed > 0:
        if curr_speed > 100:
            curr_speed = 100
        bw.speed = int(round(curr_speed))
        bw.backward()
    elif curr_speed < 0:
        if -curr_speed > 100:
            curr_speed = -100
        bw.speed = int(round(-curr_speed))
        bw.forward()
    else:
        bw.stop()

def estop():
    bw.speed = 0
    fw.turn(90)
    pan_servo.write(90)
    tilt_servo.write(90)

def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pi_car', anonymous=True)

    rospy.Subscriber("/drv_vel", Float64, fw_movement)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
