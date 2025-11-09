#ifndef _PID_H_
#define _PID_H_

#include "main.h"

struct PidParams{
  float kp;
  float ki;
  float kd;
  float integral_limit; 
  float output_limit;  
};

struct PidData{
  float integral;
  float last_error;
  float last_fdb;
};

class Pid {
  public:
    Pid(const PidParams &params) { params_ = params; 
                             datas_.integral = datas_.last_error = 0;};
    ~Pid() = default;
    void setParams(PidParams &params);
    PidParams getParams(void);
    float pidCalc(const float ref, const float fdb, const float T);
  private:
    PidParams params_;
    PidData datas_;

};

#endif