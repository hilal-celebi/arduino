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
        self.left_turn_blink_state = False
        self.right_turn_blink_state = False
        self.left_turn_active = False
        self.right_turn_active = False

        self.speed_blink_state = False
        self.acceleration_blink_state = False
        self.current_speed = 0

        # İvme
        self.last_speed = 0.0
        self.last_time = time.time()

        rospy.Subscriber('ackermann_cmd', AckermannDrive, self.ackermann_callback)

        self.rate = rospy.Rate(10)  # 10 Hz

    def ackermann_callback(self, ackermann_data):
        self.steering_angle = ackermann_data.steering_angle
        self.current_speed = ackermann_data.speed

        # İvme hesaplama
        self.current_time = time.time()
        self.acceleration = (self.current_speed - self.last_speed) / (self.current_time - self.last_time)
        self.last_speed = self.current_speed
        self.last_time = self.current_time

        # hız negatifse veya eşit
        if self.current_speed < 0:
            if not self.speed_blink_state:
                rospy.loginfo("Speed decrease LED ON")
                self.command_pub.publish("brake")
            self.speed_blink_state = True
        elif self.current_speed == 0:  # Hız sıfırsa
            if not self.speed_blink_state:
                rospy.loginfo("Speed is zero, LED ON")
                self.command_pub.publish("brake")
            self.speed_blink_state = True
        else:  # Hız pozitifse
            rospy.loginfo("Speed decrease LED OFF")
            self.command_pub.publish("stop_brake")
            self.speed_blink_state = False

        # İvme negatif kontrolü
        if self.acceleration < 0:
            if not self.acceleration_blink_state:
                rospy.loginfo("Acceleration decrease LED ON")
                self.command_pub.publish("brake")
            self.acceleration_blink_state = True
        else:
            rospy.loginfo("Acceleration decrease LED OFF")
            self.command_pub.publish("stop_brake")
            self.acceleration_blink_state = False

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

        # Hız azalması kontrolü (hız negatifse veya sıfırsa)
        if self.current_speed <= 0:
            if not self.speed_blink_state:
                rospy.loginfo("Speed decrease LED ON")
                self.command_pub.publish("brake")
            self.speed_blink_state = True
        else:
            rospy.loginfo("Speed decrease LED OFF")
            self.command_pub.publish("stop_brake")
            self.speed_blink_state = False

        # İvme negatif kontrolü
        if self.acceleration < 0:
            if not self.acceleration_blink_state:
                rospy.loginfo("Acceleration decrease LED ON")
                self.command_pub.publish("brake")
            self.acceleration_blink_state = True
        else:
            rospy.loginfo("Acceleration decrease LED OFF")
            self.command_pub.publish("stop_brake")
            self.acceleration_blink_state = False

        self.last_speed = self.current_speed
        self.last_time = self.current_time

    def check_speed_zero(self):
        if self.current_speed == 0:
          if not self.speed_blink_state:
            print("Speed is zero, LED ON")
            self.command_pub.publish("brake")
          self.speed_blink_state = True
        else:
          if self.speed_blink_state:
             print("Speed is not zero, LED OFF")
             self.command_pub.publish("stop_brake")
        self.speed_blink_state = False


    def run(self):
        while not rospy.is_shutdown():
            self.blink_leds()
            self.check_speed_zero()
            self.blink_leds_speed_acceleration()
            self.rate.sleep()

if __name__ == '__main__':
    node = ControlNode()
    node.run()
