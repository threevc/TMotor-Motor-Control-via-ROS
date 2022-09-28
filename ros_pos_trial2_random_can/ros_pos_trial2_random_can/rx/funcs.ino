float speed,postion,torque;
 #define P_MIN -12.5f
 #define P_MAX 12.5f
 #define V_MIN -50.0f
 #define V_MAX 50.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -10.0f
 #define T_MAX 10.0f

 float uint_to_float(int x_int, float x_min, float x_max, int bits)
 {
  /// converts unsigned int to float, given range and number of bits /// 
  float span = x_max- x_min; 
  float offset = x_min; 
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
 }

void unpack_reply(can_frame msg)
{ 
  /// unpack ints from can buffer 
  int id = msg.data[0]; 
  //驱动 ID 号
  int p_int = (msg.data[1]<<8)|msg.data[2]; 
  int v_int = (msg.data[3]<<4)|(msg.data[4]>>4);
  int i_int = ((msg.data[4]&0xF)<<8)|msg.data[5];
  /// convert ints to floats /// 
  float p = uint_to_float(p_int, P_MIN, P_MAX, 16); 
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12); 
  float i = uint_to_float(i_int,-T_MAX, T_MAX, 12); 
  if(id == 1)
  { 
    postion = p; 
  //Motor position data 
  // Motor speed data
  // Motor torque data
  // Read the corresponding data according to the ID code 
  speed = v; torque = i; 
  Serial.print(speed);
  Serial.print('\t');
  Serial.print(torque);
  Serial.print('\t');
  Serial.println(postion);
  
  }
}
