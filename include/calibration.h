#pragma once
#include <vector>
#include <ros/ros.h>
#include <fstream>
#include "laser_filter.h"
#include "pots_coordinates/error.h"
#include "laser_filter.h"

const float Resolution = 1080 /270;//雷达分辨率 (束/°)
const float BoardWidth = 0.52;//m
class Calibrate
{
    public:
        bool enable = true;//是否开启标定
        bool success = false;//标定完成标志位

        float x_error =0;
        float y_error =0;
        float yaw_error =0;

        pots_coordinates::error error_msg;//error消息类型类
        

        Calibrate();
        ~Calibrate();
        void save(std::string date);
        void calibrate(std::vector<float> &laser_data,
                       std::vector<float> &theta,
                       Filter filter);


    private:     
        bool calibrate_x_yaw = true;//标定(x,yaw)标志位
        bool calibrate_y =false;//标定(y)标志位
        

        //标定板左最左侧离起点的距离
        const float DrAction2Calib_x=1.0;
        const float DrAction2Calib_y=0;

        //雷达获得数据的x y yaw
        float laser_x =0;
        float laser_yaw =0;
        float laser_y =0;

        //标定后的结果
        float DrAction2DrLaser_x=0.0;
        float DrAction2DrLaser_y=0.0;
        float DrActionYaw=0.0;
        
};
