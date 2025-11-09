  #include "main.h"
  #include "M3508.hpp"
  #include "PID.hpp"
  #include "HW_can.hpp"
  
  

M3508 M3508_motors[4] = {1, 2, 3, 4};   /*定义全局变量M3508_motors，用来存放各个电机的状态*/

M3508::M3508(uint16_t id, float control_period) 
  : id_(id), 
    input_(0), 
    vel_(0), 
    angle_(0),
    pid_output(0),
    control_period_(control_period),
    enabled_(true),
    speed_pid_(Pid(PidParams{
      .kp = 15.0f,
      .ki = 0.5f,
      .kd = 0.2f, 
      .integral_limit = 3000.0f,
      .output_limit = 16000.0f 
    }))
{
 
}

void M3508::setPidParams(float kp, float ki, float kd, 
                        float integral_limit, float output_limit) 
{
  PidParams params = {kp, ki, kd, integral_limit, output_limit};
  speed_pid_.setParams(params);
}


void M3508::setInput(float input_vel) 
{
  if (enabled_) {
    input_ = input_vel;
  } else {
    input_ = 0;
  }
}

void M3508::control() {
    if (!enabled_) {
        input_ = 0;
        return;
    }
  pid_output = speed_pid_.pidCalc(input_, vel_, control_period_);
  //  int16_t current_int = static_cast<int16_t>(current);


  // uint8_t can_data[8];
  //if (!M3508_motors[id_-1].encode(can_data))
  //    Error_Handler();

  //  CAN_Send_Msg(&hcan2, can_data, 0x200, 8);
}

bool M3508::encode(uint8_t *data) 
{
    // 限制电流范围
    if (input_ > 16384) input_= 16384;
    if (input_< -16384) input_= -16384;
    
    // 转换为CAN数据格式
    int16_t current_can = static_cast<int16_t>(input_);
    
    /*拆分数据*/
    uint8_t high_byte = (current_can >> 8) & 0xFF;
    uint8_t low_byte = current_can & 0xFF;    
    /*确定数组中的位置*/
    // 电机ID 1-4 对应数据位置 0-1, 2-3, 4-5, 6-7
    int position = (id_ - 1) % 4;  
    int byte_offset = position * 2;   
    
    data[byte_offset] = high_byte;
    data[byte_offset + 1] = low_byte;
    
    return true;
}

bool M3508::decode(uint8_t *data) 
{

    uint16_t raw_angle = (data[0] << 8) | data[1];
    angle_  = (raw_angle * 360.0f) / 8191.0f / 19.0f ; 

    int16_t raw_speed = (data[2] << 8) | data[3];
    vel_ = static_cast<float>(raw_speed) / 19.0f;


    int16_t raw_current = (data[4] << 8) | data[5];
    current_ = raw_current * (20.0f / 16384.0f);  // 转换为实际电流(A)
    
    /*解析温度*/
    temp_ = static_cast<float>(data[6]);
    
    
    return true;
}



void M3508::emergencyStop() 
{
  input_ = 0;
  enabled_ = false;
}

void M3508::enable() {
    enabled_ = true;
}

void M3508::disable() {
    enabled_ = false;
    input_ = 0;
}
