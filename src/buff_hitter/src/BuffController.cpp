#include "BuffController.hpp"
#include <cmath>

//打印的相同。原始和setpoint
namespace power_rune {
    BuffController::BuffController() {   }

    BuffController::~BuffController() {   }
    BuffControlResult BuffController::buff_control
    (const ParsedSerialData& parsedData, std::shared_ptr<float> buff_pitch, std::shared_ptr<float> buff_yaw) const //TODO
    {
        BuffControlResult result;
        result.yaw_setpoint = parsedData.yaw_now;
        result.pitch_setpoint = parsedData.pitch_now;
        result.pitch_actual_want = parsedData.pitch_now;
        result.yaw_actual_want = parsedData.yaw_now;
        result.valid = false;
        result.shoot_flag = false; 

        if (*buff_pitch > 89.0 && *buff_yaw > 89.0)
        {
            std::cout<<"buff_fail"<<std::endl;
            result.yaw_setpoint = parsedData.yaw_now;
            result.pitch_setpoint = parsedData.pitch_now;
            result.pitch_actual_want = parsedData.pitch_now;
            result.yaw_actual_want = parsedData.yaw_now;
            result.valid = false;
            result.shoot_flag = false; 
        }
        else
        {
            std::cout<<"buff_yaw:"<<*buff_yaw<<std::endl;
            result.yaw_setpoint = *buff_yaw;
            //转换成最近角。yaw会但pitch不用
            //actual->上游想要的角度。
            //set->发送给下位机的
            result.yaw_setpoint = parsedData.yaw_now + std::remainder(result.yaw_setpoint - parsedData.yaw_now, 360.0);
            result.yaw_actual_want = *buff_yaw;
            
            std::cout<<"buff_pitch:"<<*buff_pitch<<std::endl;
            result.pitch_setpoint = *buff_pitch;          
            result.pitch_actual_want = *buff_pitch;
   
            result.valid = true;
            result.shoot_flag = true; 
            
            std::cout<<"aim_pitch: "<<result.pitch_setpoint<<"aim_yaw: "<<result.yaw_setpoint<<std::endl;
        }
        return result;
    }
}
