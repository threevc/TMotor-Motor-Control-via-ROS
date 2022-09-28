////float velocity,postion,torque;
// #define P_MIN -12.5f
// #define P_MAX 12.5f
// #define V_MIN -50.0f
// #define V_MAX 50.0f
// #define KP_MIN 0.0f
// #define KP_MAX 500.0f
// #define KD_MIN 0.0f
// #define KD_MAX 5.0f
// #define T_MIN -10.0f
// #define T_MAX 10.0f
//
//float uint_to_float(int x_int, float x_min, float x_max, int bits)
//{
//  /// converts unsigned int to float, given range and number of bits /// 
//  float span = x_max- x_min; 
//  float offset = x_min; 
//  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
//}
//
//void unpack_reply(can_frame msg)
//{ 
//  /// unpack ints from can buffer 
//  int id = msg.data[0]; 
//  //驱动 ID 号
//  int p_int = (msg.data[1]<<8)|msg.data[2]; 
//  int v_int = (msg.data[3]<<4)|(msg.data[4]>>4);
//  int i_int = ((msg.data[4]&0xF)<<8)|msg.data[5];
//  /// convert ints to floats /// 
//  float p = uint_to_float(p_int, P_MIN, P_MAX, 16); 
//  float v = uint_to_float(v_int, V_MIN, V_MAX, 12); 
//  float i = uint_to_float(i_int,-T_MAX, T_MAX, 12); 
//  if(id == 1)
//  { 
//    position.data = p; 
//    velocity.data = millis()/2;
////  velocity = v; 
////  torque = i; 
//
////  Serial.print(velocity);
////  Serial.print('\t');
////  Serial.print(torque);
////  Serial.print('\t');
////  Serial.println(position);
//  
//  }
//}
//
//
//
//

//
//void sendToMotor(int mot_id, float pos, float vel, float kp, float kd, float torq){
//  struct can_frame cf;
//
//  unsigned int con_pos = float_to_uint(constrain(pos, P_MIN, P_MAX), P_MIN, P_MAX, 16);
//  unsigned int con_vel = float_to_uint(constrain(vel, V_MIN, V_MAX), V_MIN, V_MAX, 12);
//  unsigned int con_kp = float_to_uint(constrain(kp, KP_MIN, KP_MAX), KP_MIN, KP_MAX, 12);
//  unsigned int con_kd = float_to_uint(constrain(kd, KD_MIN, KD_MAX), KD_MIN, KD_MAX, 12);
//  unsigned int con_torq = float_to_uint(constrain(torq, T_MIN, T_MAX), T_MIN, T_MAX, 12);
//  cf.can_id  = mot_id;
//  cf.can_dlc = 8;
//  cf.data[0] = con_pos>>8;
//  cf.data[1] = con_pos & 0xFF;
//  cf.data[2] = con_vel >> 4;
//  cf.data[3] = ((con_vel&0xF)<<4) | (con_kp>>8);
//  cf.data[4] = con_kp&0xFF;
//  cf.data[5] = con_kd>>4;
//  cf.data[6] = ((con_kd&0xF)<<4) | (con_torq>>8);
//  cf.data[7] = con_torq&0xFF;
//  mcp2515.sendMessage(&cf);
//}
//
//
////Function by Ben Katz: 
////https://os.mbed.com/users/benkatz/code/CanMasterTest//file/d24fd64d1fcb/math_ops.cpp/
//int float_to_uint(float x, float x_min, float x_max, int bits){
//    // Converts a float to an unsigned int, given range and number of bits 
//    float span = x_max - x_min;
//    float offset = x_min;
//    unsigned int pgg = 0;
//    if(bits == 12){
//      pgg = (unsigned int) ((x-offset)*4095.0/span);
//    }else if(bits == 16){
//      pgg = (unsigned int) ((x-offset)*65535.0/span);
//    }
//    return pgg;
//}
