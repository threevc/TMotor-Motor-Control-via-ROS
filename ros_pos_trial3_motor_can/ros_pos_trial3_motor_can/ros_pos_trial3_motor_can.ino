#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>


#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg2;
struct can_frame canMsg;
MCP2515 mcp2515(10);


float pos,vel;
float torq2mot=0.5;

#define BUTTON 7
#define LED 13
#define LIMIT_PIN 7


ros::NodeHandle node_handle;



std_msgs::Float32 position,velocity; 
std_msgs::Float32 torque;



void subscriberCallback(const std_msgs::Float32& torque) {
    torq2mot=torque.data;
}

ros::Publisher position_publisher("position_feedback", &position);
ros::Publisher velocity_publisher("velocity_feedback", &velocity);


ros::Subscriber<std_msgs::Float32> torque_subscriber("torque_topic", &subscriberCallback);
//ros::Subscriber<std_msgs::Float32> pos_subscriber("pos_topic",       &subscriberCallback);
//ros::Subscriber<std_msgs::Float32> vel_subscriber("vel_topic",       &subscriberCallback);
//ros::Subscriber<std_msgs::Float32> kp_subscriber("kp_topic",         &subscriberCallback);
//ros::Subscriber<std_msgs::Float32> kd_subscriber("kd_topic",         &subscriberCallback);




#define AK80_64 1
//#define AK60_6 1


void setup()
{
  
  node_handle.initNode();
  node_handle.advertise(position_publisher);
  node_handle.advertise(velocity_publisher);
  node_handle.subscribe(torque_subscriber);

  pinMode(LIMIT_PIN,INPUT);

  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  open_brake();     
  delay(6000);
  //homing_sequence();
}

void loop()
{ 



while (limit_switch_is_not_pressed()){
  sendToMotor(1, 0, 0, 0, 0, torq2mot); // Mot Id, Pos, Vel, Kp, Kd, Torque
//  sendToMotor(1, 0, 0, 0, 0, 0); // Mot Id, Pos, Vel, Kp, Kd, Torque
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      unpack_reply(canMsg);  
  }

  position_publisher.publish( &position );
  velocity_publisher.publish( &velocity );
  node_handle.spinOnce();
}
sendToMotor(1, 0, 0, 0, 0, 0); 
// delay(10);
  
}
