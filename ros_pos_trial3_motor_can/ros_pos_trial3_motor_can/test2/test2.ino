#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg2;
MCP2515 mcp2515(10);
volatile bool codeFlow = true;

void setup()
{
  delay(3000);
  while (!Serial);
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

   pinMode(3, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(3), isrBrake, CHANGE);

  open_brake();     
  delay(6000);
}

void isrBrake()
{
  sendToMotor(0,0,0,0,0,0);
  codeFlow = !codeFlow;
}

void loop()
{
  if(codeFlow)
  {
    sendToMotor(3, 0, -12, 0, 5, 0);
  }
  else
  {
    sendToMotor(0,0,0,0,0,0);
  }
  
    Serial.println(codeFlow);
}
