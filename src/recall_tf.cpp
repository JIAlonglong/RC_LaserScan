#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include "action.h"
#include "calibration.h"

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "robot_tf_boardcaster");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建tf的广播器
    static tf::TransformBroadcaster br;

    // Action和标定结果
    Action action;
    Calibrate calibrate;
    ros::Rate rate(5);
    while (nh.ok())
    {
        
        ROS_INFO_STREAM_ONCE("Success!");
        
        // 初始化tf数据  
        tf::Transform odom_to_laser_transform;

        // 读取标定数据
        calibrate.read();
        odom_to_laser_transform.setOrigin(tf::Vector3(
            calibrate.DrAction2DrLaser_x,
            calibrate.DrAction2DrLaser_y,
            0));
        
        odom_to_laser_transform.setOrigin(tf::Vector3(calibrate.DrAction2DrLaser_x,calibrate.DrAction2DrLaser_y,0));
        tf::Quaternion p;
        p.setRPY(0, 0, calibrate.DrActionYaw);
        odom_to_laser_transform.setRotation(p);

        // 广播odom与laser坐标系之间的tf数据
        br.sendTransform(tf::StampedTransform(odom_to_laser_transform, ros::Time::now(), "my_odom", "laser"));
        rate.sleep();
    }
    return 0;
};
