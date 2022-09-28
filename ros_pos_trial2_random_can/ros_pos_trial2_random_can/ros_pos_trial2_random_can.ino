#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
struct can_frame canMsg2;
MCP2515 mcp2515(10);


#define BUTTON 7
#define LED 13

ros::NodeHandle node_handle;
//The rosserial provides a very standard platform that allows makers and developers to seamlessly interface between Arduino and ROS. It enables the use of two very famous and useful technologies to explore new opportunities in robotics. This method allows for distributed computing, centralized control, control abstraction, and several other benefits to roboti

//std_msgs::String button_msg;
//std_msgs::UInt16 led_msg;

//std_msgs::UInt16 position;
//std_msgs::UInt16 torque;
std_msgs::Float32 position, velocity, torque;

//float position, velocity;
//float torque;

//void subscriberCallback(const std_msgs::UInt16& led_msg) {
void subscriberCallback(const std_msgs::Float32& torque) {
  if (torque.data  == 0.0) {
    digitalWrite(LED, HIGH); 
  } else {
    digitalWrite(LED, LOW);
  }
}

//ros::Publisher button_publisher("button_press", &button_msg);
//ros::Subscriber<std_msgs::UInt16> led_subscriber("toggle_led", &subscriberCallback);

ros::Publisher position_publisher("position_feedback", &position);
ros::Publisher velocity_publisher("velocity_feedback", &velocity);
ros::Subscriber<std_msgs::Float32> torque_subscriber("toggle_led", &subscriberCallback);

void setup()
{
//  pinMode(LED, OUTPUT);
//  pinMode(BUTTON, INPUT);
  
  node_handle.initNode();
  node_handle.advertise(position_publisher);
  node_handle.advertise(velocity_publisher);
  node_handle.subscribe(torque_subscriber);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  open_brake();     
  delay(6000);


}

void loop()
{ 
  
  sendToMotor(1, 0, 0, 0, 0, 0.5);   

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
//    unpack_reply(canMsg);
    unpack_reply2();
//  }
//  if (digitalRead(BUTTON) == HIGH) {
//    position.data = 1;//"Pressed";
//  } else {
//    position.data = 0;//"NOT pressed";
//  }

  position_publisher.publish( &position );
  velocity_publisher.publish( &velocity );
  node_handle.spinOnce();
  
  delay(100);
}





/////////////////////////////////////////////




void open_brake()
{
  canMsg2.can_id  = 1; 
  canMsg2.can_dlc = 8;
  canMsg2.data[0] = 0xFF;
  canMsg2.data[1] = 0xFF;
  canMsg2.data[2] = 0xFF;
  canMsg2.data[3] = 0xFF;
  canMsg2.data[4] = 0xFF;
  canMsg2.data[5] = 0xFF;
  canMsg2.data[6] = 0xFF;
  canMsg2.data[7] = 0xFC;
  Serial.println("Open Brake");
  mcp2515.sendMessage(&canMsg2);
}
