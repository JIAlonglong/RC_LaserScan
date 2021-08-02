/*
 * @Description: 
 * @Version: 1.0
 * @Autor: bonbon
 * @Date: 2021-08-01 19:58:22
 * @LastEditors: bonbon
 * @LastEditTime: 2021-08-02 13:51:36
 */

#pragma once
#include <vector>
#include <ros/ros.h>
#include <fstream>
#include "filter.h"
#include "rc_laserscan/error.h"


const float Resolution = 1080 /270;//雷达分辨率 (束/°)
const float BoardWidth = 0.52;//标定板宽度 单位米

/**
 * 标定是标定雷达距离机器人中心的x，y以及雷达自身的旋转角度
 * 采用方式是将标定板中心放在世界坐标中一点
 * 然后遍历标定板上所有点计算出x，y和旋转角度
 * 不断迭代直到error降到指定值
 **/
class Calibrate
{
    public:
        bool enable = false;//是否开启标定
        bool success = false;//标定完成标志位
        bool isSave =false;//保存标志位

        //标定过程中的error
        float x_error =0;
        float y_error =0;
        float yaw_error =0;

        //标定后的结果
        float DrAction2DrLaser_x=0.0;
        float DrAction2DrLaser_y=0.0;
        float DrActionYaw=0.0;

        rc_laserscan::error error_msg;//error消息类型类
        
        std::string save_path ="./src/rc_laserscan/config/calib.yaml";//保存结果路径

        Calibrate();
        ~Calibrate();
	    void init(bool flag);
        void save();
        void read();
        void calibrate(std::vector<float> &laser_data,
                       std::vector<float> &theta,
                       Filter &filter);

    private:     
        bool calibrate_x_yaw = true;//标定(x,yaw)标志位
        bool calibrate_y =false;//标定(y)标志位
        
        //标定板中心离起点的距离
        const float DrAction2Calib_x=1.0;
        const float DrAction2Calib_y=0;

        //标定过程中 雷达数据计算出的x y yaw
        float laser_x =0;
        float laser_yaw =0;
        float laser_y =0;

};
