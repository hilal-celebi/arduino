#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 20 00:46:03 2024

@author: hilal
"""
import rospy
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive
import time

command_pub = rospy.Publisher('command', String, queue_size=10)

# Zaman
blink_interval = 0.3
last_blink_time = time.time()
left_turn_blink_state = False
right_turn_blink_state = False
left_turn_active = False
right_turn_active = False

speed_blink_state = False
acceleration_blink_state = False
current_speed = 0

# İvme
last_speed = 0.0
last_time = time.time()

def ackermann_callback(ackermann_data):
    global left_turn_active, right_turn_active, last_blink_time, left_turn_blink_state, right_turn_blink_state, last_speed, last_time
    global speed_blink_state, acceleration_blink_state, current_speed

    steering_angle = ackermann_data.steering_angle
    current_speed = ackermann_data.speed
    
    # İvme hesaplama
    current_time = time.time()
    acceleration = (current_speed - last_speed) / (current_time - last_time)
    last_speed = current_speed
    last_time = current_time

    # hız negatifse veya eşit
    if current_speed <= 0:
        if not speed_blink_state:
            rospy.loginfo("Speed decrease LED ON")
            command_pub.publish("brake")
        speed_blink_state = True
    else:
        rospy.loginfo("Speed decrease LED OFF")
        command_pub.publish("stop_brake")
        speed_blink_state = False

        # İvme negatif kontrolü
        if acceleration < 0:
            if not acceleration_blink_state:
                rospy.loginfo("Acceleration decrease LED ON")
                command_pub.publish("brake")
            acceleration_blink_state = True
        else:
            rospy.loginfo("Acceleration decrease LED OFF")
            command_pub.publish("stop_brake")
            acceleration_blink_state = False

    # Direksiyon kontrolü
    if steering_angle < -0.15:
        left_turn_active = False
        if not right_turn_active:
            right_turn_active = True
            right_turn_blink_state = True
            last_blink_time = time.time()
    elif steering_angle > 0.15:
        right_turn_active = False
        if not left_turn_active:
            left_turn_active = True
            left_turn_blink_state = True
            last_blink_time = time.time()
    else:
        left_turn_active = False
        right_turn_active = False
        command_pub.publish("stop_right_turn")
        command_pub.publish("stop_left_turn")

    rospy.loginfo("Steering angle: %s", steering_angle)
    rospy.loginfo("Current speed: %s, Last speed: %s, Acceleration: %s", current_speed, last_speed, acceleration)

def blink_leds():
    global last_blink_time, left_turn_blink_state, right_turn_blink_state

    current_time = time.time()

    if current_time - last_blink_time >= blink_interval:
        last_blink_time = current_time

        if left_turn_active:
            if left_turn_blink_state:
                command_pub.publish("left_turn")
            else:
                command_pub.publish("stop_left_turn")
            left_turn_blink_state = not left_turn_blink_state

        if right_turn_active:
            if right_turn_blink_state:
                command_pub.publish("right_turn")
            else:
                command_pub.publish("stop_right_turn")
            right_turn_blink_state = not right_turn_blink_state

def blink_leds_speed_acceleration():
    global last_speed, last_time, acceleration_blink_state, speed_blink_state, current_speed

    current_time = time.time()
    acceleration = (current_speed - last_speed) / (current_time - last_time)

    # Hız azalması kontrolü (hız negatifse)
    if current_speed <= 0:
        if not speed_blink_state:
            rospy.loginfo("Speed decrease LED ON")
            command_pub.publish("brake")
        speed_blink_state = True
    else:
        rospy.loginfo("Speed decrease LED OFF")
        command_pub.publish("stop_brake")
        speed_blink_state = False

        # İvme negatif kontrolü
        if acceleration < 0:
            if not acceleration_blink_state:
                rospy.loginfo("Acceleration decrease LED ON")
                command_pub.publish("brake")
            acceleration_blink_state = True
        else:
            rospy.loginfo("Acceleration decrease LED OFF")
            command_pub.publish("stop_brake")
            acceleration_blink_state = False

    last_speed = current_speed
    last_time = current_time

rospy.init_node('control_node')
ackermann = rospy.Subscriber('ackermann_cmd', AckermannDrive, ackermann_callback)

rate = rospy.Rate(10)  # 10 Hz
while not rospy.is_shutdown():
    blink_leds()
    blink_leds_speed_acceleration()
    rate.sleep()
