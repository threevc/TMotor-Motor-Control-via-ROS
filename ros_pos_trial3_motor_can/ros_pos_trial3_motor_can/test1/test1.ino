#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg2;
MCP2515 mcp2515(10);

void setup()
{
  delay(3000);
  while (!Serial);
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  open_brake();     
  delay(6000);
}

void loop()
{
  sendToMotor(3, 0, 0, 0, 0, 0.5);   
  /*delay(2000);
  open_brake(); 
  delay(2000);*/
}
