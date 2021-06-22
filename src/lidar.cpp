#include "lidar.h"
void Lidar::init()
{
    // 初始化角度数据   
    for(int i = 0;i < DATA_NUM;i++)
    {
        THETA[i] =(-3*PI)/4 + ANGLE_INCREMENT * i;
    }   
}

void Lidar::getData(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    nowData = std::vector<float>(scan->ranges);
}

void Lidar::prePublish(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //发布滤波后的数据
    result.header.stamp = ros::Time::now();
    result.header.frame_id = "laser";
    result.angle_min = (-3*PI)/4;
    result.angle_max = ( 3*PI)/4;
    result.angle_increment = scan->angle_increment;
    result.time_increment = scan->time_increment;
    result.scan_time = scan->scan_time;
    result.range_min = scan->range_min;
    result.range_max = scan->range_max; 

    result.ranges = std::vector<float>(nowData);
    result.intensities = std::vector<float>(scan->intensities);
    /*  下面这种写法在rviz会显示一个贼酷炫的螺旋，虽然我也不知道有什么区别
    for(int i = 0; i < DATA_NUM; i++)
    {
        result.ranges.push_back(nowData[i]);
        result.intensities.push_back(scan->intensities[i]);
    }
    */
}
Lidar::Lidar()
{

}

Lidar::~Lidar()
{
    
}
