
#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/Float32.h>

const int ledPin = 12;
const float speedThreshold = 0.40;  // Hız eşiği (m/s)

ros::NodeHandle nh;

void speedCallback(const std_msgs::Float32& msg) {
  float currentSpeed = msg.data;
  if (currentSpeed >= speedThreshold) {
    digitalWrite(ledPin, HIGH);  // LED'i yak
  } else {
    digitalWrite(ledPin, LOW);   // LED'i söndür
  }
}

ros::Subscriber<std_msgs::Float32> sub("current_speed", speedCallback);

void setup() {
  nh.getHardware()->setBaud(115200);
  pinMode(ledPin, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}

