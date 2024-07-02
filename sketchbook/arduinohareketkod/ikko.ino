#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

const int rightTurnLED = 12; // Sağ dönüş için LED
const int brakeLedPin = 11;     // ivme seviyesi LED
const int leftTurnLED = 10; //sol dönüş için

void commandCallback(const std_msgs::String& cmd_msg) {
  // Seri monitöre gelen komutu yazdır
  Serial.print("Received command: ");
  Serial.println(cmd_msg.data);

  if (strcmp(cmd_msg.data, "right_turn") == 0) {
    digitalWrite(rightTurnLED, HIGH);
  } else if (strcmp(cmd_msg.data, "stop_right_turn") == 0) {
    digitalWrite(rightTurnLED, LOW);
  } else if (strcmp(cmd_msg.data, "left_turn") == 0) {
    digitalWrite(leftTurnLED, HIGH);
  } else if (strcmp(cmd_msg.data, "stop_left_turn") == 0) {
    digitalWrite(leftTurnLED, LOW);
  } else if (strcmp(cmd_msg.data, "stop_brake") == 0) {
    digitalWrite(brakeLedPin, LOW);
  } else if (strcmp(cmd_msg.data, "brake") == 0) {
    digitalWrite(brakeLedPin, HIGH);
  }
}
    


ros::Subscriber<std_msgs::String> sub("command", &commandCallback);

void setup() {
  pinMode(rightTurnLED, OUTPUT);
  pinMode(brakeLedPin, OUTPUT);
  pinMode(leftTurnLED, OUTPUT); 
  Serial.begin(9600); // Seri haberleşmeyi başlat

  nh.initNode();
  nh.subscribe(sub);
} 

void loop() {
  nh.spinOnce();
  delay(10);
}
