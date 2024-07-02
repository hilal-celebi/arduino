#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from ackermann_msgs.msg import AckermannDrive
import time

class ControlNode:
    def __init__(self):
        rospy.init_node('control_node')
        self.command_pub = rospy.Publisher('command', String, queue_size=10)

        # Zaman
        self.blink_interval = 0.3
        self.last_blink_time = time.time()
        self.last_speed_blink_time = time.time()

        self.left_turn_blink_state = False
        self.right_turn_blink_state = False
        self.left_turn_active = False
        self.right_turn_active = False

        self.speed_blink_state = False
        self.current_speed = 0

        # İvme
        self.last_speed = 0.0
        self.last_time = time.time()

        rospy.Subscriber('ackermann_cmd', AckermannDrive, self.ackermann_callback)

        self.rate = rospy.Rate(10)  # 10 Hz

    def ackermann_callback(self, ackermann_data):
        self.steering_angle = ackermann_data.steering_angle
        self.current_speed = ackermann_data.speed

        if self.current_speed < -0.01:
            self.speed_blink_state = True
            self.last_speed_blink_time = time.time()
        elif self.current_speed > 0.01:
            self.speed_blink_state = False
        else:  # Hız sıfırsa
            self.speed_blink_state = False

        # Direksiyon kontrolü
        if self.steering_angle < -0.15:
            self.left_turn_active = False
            if not self.right_turn_active:
                self.right_turn_active = True
                self.right_turn_blink_state = True
                self.last_blink_time = time.time()

        elif self.steering_angle > 0.15:
            self.right_turn_active = False
            if not self.left_turn_active:
                self.left_turn_active = True
                self.left_turn_blink_state = True
                self.last_blink_time = time.time()
        else:
            self.left_turn_active = False
            self.right_turn_active = False
            self.command_pub.publish("stop_right_turn")
            self.command_pub.publish("stop_left_turn")


    def blink_leds(self):
        self.current_time = time.time()

        if self.current_time - self.last_blink_time >= self.blink_interval:
            self.last_blink_time = self.current_time

            if self.left_turn_active:
                if self.left_turn_blink_state:
                    self.command_pub.publish("left_turn")
                else:
                    self.command_pub.publish("stop_left_turn")
                self.left_turn_blink_state = not self.left_turn_blink_state

            if self.right_turn_active:
                if self.right_turn_blink_state:
                    self.command_pub.publish("right_turn")
                else:
                    self.command_pub.publish("stop_right_turn")
                self.right_turn_blink_state = not self.right_turn_blink_state


    def blink_leds_speed_acceleration(self):
        self.current_time = time.time()
        self.acceleration = (self.current_speed - self.last_speed) / (self.current_time - self.last_time)
    
        #  # İvme negatif kontrolü
        if self.acceleration < 0:
            rospy.loginfo("Acceleration decrease LED ON")
            self.command_pub.publish("brake")
    
        self.last_speed = self.current_speed
        self.last_time = self.current_time
    

    def stop_led(self):

        self.current_speed_time = time.time()

        if self.current_speed_time - self.last_speed_blink_time >= self.blink_interval:
            self.last_speed_blink_time = self.current_speed_time

        # Hız azalması kontrolü (hız negatifse)
        if self.current_speed < -0.01:
            if self.speed_blink_state:
                # rospy.loginfo("current_speed < 0 : Speed decrease LED ON")
                self.command_pub.publish("brake")
            else:
                self.command_pub.publish("stop_brake")
            self.speed_blink_state = not self.speed_blink_state

        elif self.current_speed > 0.01:  # Hız pozitifse
            # rospy.loginfo("current_speed > 0 : Speed decrease LED OFF")
            self.command_pub.publish("stop_brake")
            self.speed_blink_state = False
        else:  # Hız sıfırsa
            # rospy.loginfo("Speed is zero, LED ON")
            self.command_pub.publish("brake")

    def run(self):
            while not rospy.is_shutdown():
                self.blink_leds()
                self.stop_led()
                self.blink_leds_speed_acceleration()
                self.rate.sleep()

if __name__ == '__main__':
   node = ControlNode()
   node.run()
    
