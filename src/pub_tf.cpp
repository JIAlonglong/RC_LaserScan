#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "DR_communication.h"
#include "action.h"
#include "calibration.h"

#define PI acos(-1)

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "robot_tf_boardcaster");

    // 订阅base_link的位置话题
    ros::NodeHandle nh;

    // 创建tf的广播器
    static tf::TransformBroadcaster br;

    // DR串口初始化
    DR_SerialInit();

    // Action和标定结果
    Action action;
    Calibrate calibrate;

    while (nh.ok())
    {
        // 读取串口数据
        DR_SerialRead(action.x, action.y, action.yaw, action.flag);
        
        // 初始化tf数据
        tf::Transform world_to_odom_transform;
        world_to_odom_transform.setOrigin(tf::Vector3(action.x, action.y, 0));
        
	    calibrate.read(); // 读取标定数据
        tf::Transform odom_to_laser_transform;
        odom_to_laser_transform.setOrigin(tf::Vector3(
            calibrate.DrAction2DrLaser_x,
            calibrate.DrAction2DrLaser_y,
            0));

        // 因为车子转了超过PI 溢出action(-PI,PI) 所以最终为 向左偏为-PI+theta 向右为PI-theta
        float yaw;// 偏航角 偏左为正 偏右为负
        if ((action.yaw == 31415) || (action.yaw == -31415) || (action.yaw == 0)){yaw = 0;}
        else if (action.yaw < 0){yaw = action.yaw / 10000.0 + PI;}
        else {yaw = action.yaw / 10000.0 - PI;}

        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        world_to_odom_transform.setRotation(q);

        tf::Quaternion p;
        p.setRPY(0, 0, calibrate.DrActionYaw);
        odom_to_laser_transform.setRotation(p);

        // 广播world与odom坐标系之间的tf数据
        br.sendTransform(tf::StampedTransform(world_to_odom_transform, ros::Time::now(), "world", "my_odom"));
        // 广播odom与laser坐标系之间的tf数据
        br.sendTransform(tf::StampedTransform(odom_to_laser_transform, ros::Time::now(), "my_odom", "laser"));
    }
    return 0;
};
