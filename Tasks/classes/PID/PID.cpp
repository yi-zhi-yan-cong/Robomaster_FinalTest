#include "PID.hpp"
#include "math.h"

void Pid::setParams(PidParams &params)
{
  params_ = params;
}

PidParams Pid::getParams(void) 
{
  return params_;
}

float Pid::pidCalc(const float ref, const float fdb, const float T)
{
    float error = ref - fdb;
    
    // 积分分离
    float integral = datas_.integral;
    //if (fabs(error) < 35.0f) { 
    //    integral += (error + datas_.last_error) / 2 * T;
    //}
    
    // 微分先行
    float derivative = (datas_.last_fdb - fdb) / T; // 对测量值微分
    
    datas_.integral = integral;
    if (datas_.integral > params_.integral_limit) 
        datas_.integral = params_.integral_limit;
    else if (datas_.integral < -params_.integral_limit) 
        datas_.integral = -params_.integral_limit;
    
    float output = params_.kp * error 
                   + params_.ki * integral 
                   + params_.kd * derivative;
    
    datas_.last_error = error;
    datas_.last_fdb = fdb; // 保存当前测量值

    if (output > params_.output_limit) 
        output = params_.output_limit;
    else if (output < -params_.output_limit) 
        output = -params_.output_limit;

    return output;
}