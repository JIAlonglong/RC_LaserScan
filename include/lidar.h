/*
 * @Description: 
 * @Version: 1.0
 * @Autor: bonbon
 * @Date: 2021-08-01 19:58:22
 * @LastEditors: bonbon
 * @LastEditTime: 2021-08-02 13:56:23
 */

#pragma once 
#include <iostream>
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "filter.h"

class Lidar
{  
    public:      

        std::vector<float> THETA{std::vector<float>(DATA_NUM)}; //角度数据
        std::vector<float> nowData{std::vector<float>(DATA_NUM)};//目前的数据
        std::vector<float> lastData{std::vector<float>(DATA_NUM)};//上一次的数据

        sensor_msgs::LaserScan result;//用于发布

        Lidar();
        ~Lidar();
        void init();
        void getData(const sensor_msgs::LaserScan::ConstPtr& scan);
        void prePublish(const sensor_msgs::LaserScan::ConstPtr& scan);

    private:

};
