#ifndef CHASSIS_HPP_
#define CHASSIS_HPP_

#include "M3508.hpp"
#include "DT7.hpp"

extern M3508 M3508_motors[4];

using namespace hello_world::devices::remote_control;

class Chassis {
public:
  enum Mode {
      SEPARATE_MODE,  // 分离模式
      FOLLOW_MODE,    // 跟随模式
      GYRO_MODE       // 小陀螺模式
  };

  void init();
  void update(const DT7& remote);
    
private:
  void calculate(float vx, float vy, float wz);
  void send();
    
  float yaw_offset_;  //底盘零漂
  Mode current_mode_ = SEPARATE_MODE;
};



#endif