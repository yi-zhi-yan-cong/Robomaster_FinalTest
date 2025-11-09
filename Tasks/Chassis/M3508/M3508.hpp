#ifndef M3508_HPP_
#define M3508_HPP_

#include "main.h"
#include "PID.hpp" 

class M3508 {
private:
  uint16_t id_;              
  Pid speed_pid_;             
  float input_;
  float angle_;
  float vel_;
  float current_;
  float temp_;
  float pid_output;
  float control_period_;      
  bool enabled_;             

public:
  M3508(uint16_t id, float control_period = 0.01f);
  void setPidParams(float kp, float ki, float kd, 
                     float integral_limit, float output_limit);
  void setInput(float input_vel) ;
  void control();
  bool encode(uint8_t *data);
  bool decode(uint8_t *data);
    

  void emergencyStop();
  void enable();
  void disable();
    
  uint16_t getId() const { return id_; }
  float getinput() const { return input_; }
  float getVel() const { return vel_; }
  float getAngle() const { return angle_; }
  bool isEnabled() const { return enabled_; }

};

#endif