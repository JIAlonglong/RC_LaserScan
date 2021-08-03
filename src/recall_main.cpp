#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "coordinate.h"
#include "save.h"
#include "lidar.h"
#include "action.h"
#include "calibration.h"
#include "filter.h"
#include "get_center.h"
#include "pot_grab.h"

ros::Publisher laser_pub;
ros::Publisher coordinate_info_pub;

Action action;
Calibrate calibrate;
Lidar lidar;
Filter filter;
Coordinate coordinate;
PotGrab pot;

tf2_ros::Buffer tfBuffer;
geometry_msgs::TransformStamped transformStamped;

void laser_callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{

    //获取数据
    lidar.getData(scan);

    //读出标定的x y yaw
    calibrate.read();

    try
    {
        transformStamped = tfBuffer.lookupTransform("world", "my_odom", ros::Time(0)); //获取转换信息
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("Received an exception trying to transform a point from \"world\" to \"my_odom\": %s", ex.what());
    }    
    action.x = (short)transformStamped.transform.translation.x;
    action.y = (short)transformStamped.transform.translation.y;
    
    //四元数转欧拉角
    tf::Quaternion RQ2;
    tf::quaternionMsgToTF(transformStamped.transform.rotation, RQ2);
    double roll, pitch, yaw;
    tf::Matrix3x3(RQ2).getRPY(roll, pitch, yaw);

    // 将标定的x y旋转到世界坐标
    float DrAction2DrLaser_x=calibrate.DrAction2DrLaser_x;
    float DrAction2DrLaser_y=calibrate.DrAction2DrLaser_y;
    coordinate_rotation(&DrAction2DrLaser_x,&DrAction2DrLaser_y,yaw);

    //卡尔曼滤波
    filter.kalman_filter(
        lidar.nowData,lidar.lastData,lidar.THETA,
        action.x,action.y,yaw+calibrate.DrActionYaw,
        &action.last_x,&action.last_y);

    //根据全场定位滤波
    float min_x = 0;
    float max_x = (5.35 - (action.y / 1000.0) - (DrAction2DrLaser_x));
    float min_y = (-3.0 - (-action.x / 1000.0) - (DrAction2DrLaser_y));
    float max_y = (4.0 - (-action.x / 1000.0) - (DrAction2DrLaser_y));
    float min_angle = -PI / 2 - yaw - calibrate.DrActionYaw;
    float max_angle = PI / 2 - yaw - calibrate.DrActionYaw;
    filter.easy_filter(lidar.nowData, lidar.THETA,
                        false, 0.0,
                        true, min_angle, max_angle,
                        true, min_x, max_x, min_y, max_y, yaw);

    // 去除离群点outliers
    filter.remove_outlier(lidar.nowData, lidar.THETA, 0.07, 3);

    // 滤出圆弧
    filter.get_circle(lidar.nowData, lidar.THETA, 0.03); //0.008

    //数量滤波
    int DataMinNum = 20;
    filter.num_filter(lidar.nowData, DataMinNum);

    //获得连续段
    std::vector<float> start_index; //连续段的起始坐标
    std::vector<float> end_index;   //连续段的终止坐标
    splinter_continuous_part(lidar.nowData, start_index, end_index);

    std::vector<float> left(2), middle(2), right(2);
    distributeData(start_index, end_index,
                    left, middle, right,
                    lidar.THETA, action.x);

    //通过滤波结果计算出圆心
    index2center(left, lidar.nowData, coordinate.left_xyR);
    index2center(middle, lidar.nowData, coordinate.middle_xyR);
    index2center(right, lidar.nowData, coordinate.right_xyR);

    // (action 偏左为正 偏右为负)、(函数传入正值右转)
    coordinate_rotation(
        &(coordinate.left_xyR[0]),
        &(coordinate.left_xyR[1]),
        yaw + calibrate.DrActionYaw);
    coordinate_rotation(
        &(coordinate.middle_xyR[0]),
        &(coordinate.middle_xyR[1]),
        yaw + calibrate.DrActionYaw);
    coordinate_rotation(
        &(coordinate.right_xyR[0]),
        &(coordinate.right_xyR[1]),
        yaw + calibrate.DrActionYaw);

    //转换到以TR起点的世界坐标
    int left_x,left_y,middle_x,middle_y,right_x,right_y;
    change2worldCoordinate(coordinate.left_xyR[0], coordinate.left_xyR[1],
                            action.x, action.y,
                            DrAction2DrLaser_x, DrAction2DrLaser_y,
                            &left_x, &left_y);
    change2worldCoordinate(coordinate.middle_xyR[0], coordinate.middle_xyR[1],
                            action.x, action.y,
                            DrAction2DrLaser_x, DrAction2DrLaser_y,
                            &middle_x, &middle_y);
    change2worldCoordinate(coordinate.right_xyR[0], coordinate.right_xyR[1],
                            action.x, action.y,
                            DrAction2DrLaser_x, DrAction2DrLaser_y,
                            &right_x, &right_y);

    left_x=limit(left_x,-6000,-5300);//-5850 -5500
    left_y=limit(left_y,7500,8500);//7650 8350
    middle_x=limit(middle_x,-6000,-5300);//-5850 -5500
    middle_y=limit(middle_y,5000,6000);//5150 5850
    right_x=limit(right_x,-6000,-5300);//-5850 -5500
    right_y=limit(right_y,2500,3500);//2650 3350

    if(coordinate.left_x!=0 &&coordinate.left_y!=0)
    {
        if(abs(left_x-coordinate.left_x)<1000 && abs(left_y-coordinate.left_y)<1000)
        {
            coordinate.left_x=left_x;
            coordinate.left_y=left_y;
        }
        else
        {
            coordinate.left_x=0;
            coordinate.left_y=0;
        }
    }
    else
    {
        coordinate.left_x=left_x;
        coordinate.left_y=left_y;
    }

    if(coordinate.middle_x!=0 &&coordinate.middle_y!=0)
    {
        if(abs(middle_x-coordinate.middle_x)<1000 && abs(middle_y-coordinate.middle_y)<1000)
        {
            coordinate.middle_x=middle_x;
            coordinate.middle_y=middle_y;
        }
        else
        {
            coordinate.left_x=0;
            coordinate.left_y=0;
        }
    }
    else
    {
        coordinate.middle_x=middle_x;
        coordinate.middle_y=middle_y;
    }

    if(coordinate.right_x!=0 &&coordinate.right_y!=0)
    {
        if(abs(right_x-coordinate.right_x)<1000 && abs(right_y-coordinate.right_y)<1000)
        {
            coordinate.right_x=right_x;
            coordinate.right_y=right_y;
        }
        else
        {
            coordinate.right_x=0;
            coordinate.right_y=0;
        }
    }
    else
    {
        coordinate.right_x=right_x;
        coordinate.right_y=right_y;
    }
    

        //串口发送 如果为0即为识别不到
    ROS_INFO("left:%d %d,middle:%d %d right:%d %d",
            coordinate.left_x, coordinate.left_y,
            coordinate.middle_x,coordinate.middle_y,
            coordinate.right_x, coordinate.right_y);


        //发送DR要抓的壶的信息
        DistanceIndex left_struct, middle_struct, right_struct;
        left_struct.init(coordinate.left_xyR, 0x03);
        middle_struct.init(coordinate.middle_xyR, 0x02);
        right_struct.init(coordinate.right_xyR, 0x01);

        std::vector<DistanceIndex> GetMinVec;
        if (left_struct.distance != 0)
        {
            GetMinVec.push_back(left_struct);
        }
        if (middle_struct.distance != 0)
        {
            GetMinVec.push_back(middle_struct);
        }
        if (right_struct.distance != 0)
        {
            GetMinVec.push_back(right_struct);
        }

        if (GetMinVec.size() > 0)
        {
            std::sort(GetMinVec.begin(), GetMinVec.end(), [](DistanceIndex a, DistanceIndex b)
                      { return a.distance < b.distance; }); //升序排序
            pot.preSend(GetMinVec[0].index, coordinate);
            //DR_SerialWrite(pot.x,pot.y,pot.alpha*10000.0,pot.index);
            //ROS_INFO("%d %d %f %d", pot.x, pot.y, pot.alpha * 180 / PI, pot.index); //发送的数据要转到DR坐标系
        }
        else
        {
            //DR_SerialWrite(0,0,0,0x00);
        }

    //坐标发布
    coordinate.prePublish();
    coordinate_info_pub.publish(coordinate.coordinate_msg);

    //发布数据用于rviz
    lidar.prePublish(scan);
    laser_pub.publish(lidar.result);
}

int main(int argc, char **argv)
{

    // 初始化ROS节点
    ros::init(argc, argv, "laser");

    // 创建节点句柄
    ros::NodeHandle n;
    
    tf2_ros::TransformListener tfListerner(tfBuffer);

    //雷达初始化
    lidar.init();

    //定义Publisher 发布滤波后坐标信息
    coordinate_info_pub = n.advertise<rc_laserscan::coordinate>("/coordinate_info", 10);

    //定义Publisher 发布滤波后雷达信息
    laser_pub = n.advertise<sensor_msgs::LaserScan>("/filter", 10);

    // 创建一个Subscriber 回调函数
    ros::Subscriber pose_sub = n.subscribe("/new_scan", 10, laser_callback);

    //等待回调函数
    ros::spin();


    return 0;
}
