#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

#define BUTTON 7
#define LED 13

ros::NodeHandle node_handle;

//std_msgs::String button_msg;
//std_msgs::UInt16 led_msg;

std_msgs::UInt16 position;
std_msgs::UInt16 torque;

//float position, velocity;
//float torque;

//void subscriberCallback(const std_msgs::UInt16& led_msg) {
void subscriberCallback(const std_msgs::UInt16& torque) {
  if (torque.data  == 0) {
    digitalWrite(LED, HIGH); 
  } else {
    digitalWrite(LED, LOW);
  }
}

//ros::Publisher button_publisher("button_press", &button_msg);
//ros::Subscriber<std_msgs::UInt16> led_subscriber("toggle_led", &subscriberCallback);

ros::Publisher position_publisher("position_feedback", &position);
ros::Subscriber<std_msgs::UInt16> torque_subscriber("toggle_led", &subscriberCallback);

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT);
  
  node_handle.initNode();
  node_handle.advertise(position_publisher);
  node_handle.subscribe(torque_subscriber);
}

void loop()
{ 
  if (digitalRead(BUTTON) == HIGH) {
    position.data = 1;//"Pressed";
  } else {
    position.data = 0;//"NOT pressed";
  }

  position_publisher.publish( &position );
  node_handle.spinOnce();
  
  delay(100);
}







//void setup() {
//  // put your setup code here, to run once:
//
//}
//
//void loop() {
//  // put your main code here, to run repeatedly:
//
//}
