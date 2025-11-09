#include "Chassis.hpp"
#include "HW_can.hpp"
#include "main.h"
#include "can.h"
#include <cmath>
#include "imu_task.hpp"

extern float euler_angles[3];
extern M3508 M3508_motors[4];


void Chassis::init()
{
  //零漂计算
  uint32_t start_time = HAL_GetTick();
  float yaw_sum = 0.0f;
  int sample_count = 0;
        
  // 3秒校准期
  while (HAL_GetTick() - start_time < 3000) {
    ImuUpdate();
    yaw_sum += euler_angles[0];
    sample_count++;
            
    // 发送零指令确保安全
    uint8_t zero_data[8] = {0};
    CAN_Send_Msg(&hcan2, zero_data, 0x200, 8);
            
    HAL_Delay(10);
    }
        
  float yaw_offset_ = yaw_sum / sample_count;

  M3508_motors[0].enable();
  M3508_motors[1].enable();
  M3508_motors[2].enable();
  M3508_motors[3].enable();
}

void Chassis::update(const DT7& remote) 
{

  SwitchState left_switch = remote.rc_l_switch();
  switch (left_switch) {
      case kSwitchStateUp:   current_mode_ = SEPARATE_MODE; break;
      case kSwitchStateMid:  current_mode_ = FOLLOW_MODE; break;
      case kSwitchStateDown: current_mode_ = GYRO_MODE; break;
  }

  if(remote.rc_r_switch() == kSwitchStateDown){
    M3508_motors[0].disable();
    M3508_motors[1].disable();
    M3508_motors[2].disable();
    M3508_motors[3].disable();
    return ;
  }else{
    M3508_motors[0].enable();
    M3508_motors[1].enable();
    M3508_motors[2].enable();
    M3508_motors[3].enable();
  }

  float vx, vy, wz;
    
  switch (current_mode_) {
    case SEPARATE_MODE:
      vx = remote.rc_lv() * 1.0f;   // 左摇杆垂直 -> 前后速度
      vy = remote.rc_lh() * 1.0f;   // 左摇杆水平 -> 左右速度
      wz = remote.rc_rv() * 3.14f;  // 右摇杆垂直 -> 旋转速度
        break;
            
    case FOLLOW_MODE:
      vx = remote.rc_lv() * 1.0f;
      vy = remote.rc_lh() * 1.0f;
      wz = remote.rc_rv() * 3.14f; 
        break;
            
    case GYRO_MODE:
      vx = remote.rc_lv() * 1.0f;
      vy = remote.rc_lh() * 1.0f;
      wz = 3.0f;  // 固定旋转速度
        break;
    }
    
  //运动学计算
  calculate(vx, vy, wz);
    
  //发送
  send();
}

void Chassis::calculate(float vx, float vy, float wz) 
{

  const float L = 0.21691f;  // 轮子到中心的距离
    
  // 获取底盘朝向角度θ(减去零漂)
  float theta = euler_angles[0] - yaw_offset_;
    
  // 计算三角函数值
  float cos_theta = cosf(theta);
  float sin_theta = sinf(theta);
    
  // 计算矩阵中的各项系数
  float m11 = -cos_theta - sin_theta;
  float m12 = -sin_theta + cos_theta;
    
  float m21 = -cos_theta + sin_theta;
  float m22 = -sin_theta - cos_theta;
    
  float m31 = cos_theta + sin_theta;
  float m32 = sin_theta - cos_theta;
    
  float m41 = -cos_theta - sin_theta;
  float m42 = sin_theta + cos_theta;
    
  // 应用运动学逆解公式
  float w1 = m11 * vx + m12 * vy + L * wz;
  float w2 = m21 * vx + m22 * vy + L * wz;
  float w3 = m31 * vx + m32 * vy + L * wz;
  float w4 = m41 * vx + m42 * vy + L * wz;
    
  // 设置电机目标速度
  M3508_motors[0].setInput(w1);
  M3508_motors[1].setInput(w2);
  M3508_motors[2].setInput(w3);
  M3508_motors[3].setInput(w4);
    
    // 计算PID
  M3508_motors[0].control();
  M3508_motors[1].control();
  M3508_motors[2].control();
  M3508_motors[3].control();
}

void Chassis::send() 
{
  uint8_t can_data[8] = {0};
    
  // 编码四个电机的电流数据
  if(!M3508_motors[0].encode(can_data))
    Error_Handler();
  if(!M3508_motors[1].encode(can_data))
    Error_Handler();
  if(!M3508_motors[2].encode(can_data))
    Error_Handler();
  if(!M3508_motors[3].encode(can_data))
    Error_Handler();

    
  // 发送CAN指令
  CAN_Send_Msg(&hcan2, can_data, 0x200, 8);
}

