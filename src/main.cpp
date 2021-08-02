#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "TR_communication.h"
#include "DR_communication.h"
#include "coordinate.h"
#include "save.h"
#include "lidar.h"
#include "action.h"
#include "calibration.h"
#include "filter.h"
#include "get_center.h"
#include "pot_grab.h"



//Publisher 发布雷达数据(用于rviz可视化) 坐标数据、标定error数据(用于python可视化)
ros::Publisher laser_pub;
ros::Publisher coordinate_info_pub;
ros::Publisher error_info_pub;

std::string run_code_date; //代码开始运行的日期 作为保存的文件名

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

    // *标定
    if (calibrate.enable)
    {
        if (!calibrate.success)
        {
            calibrate.calibrate(lidar.nowData, lidar.THETA, filter);
            error_info_pub.publish(calibrate.error_msg);
        }
        else
        {
            if (calibrate.isSave)
            {
                calibrate.save();
            }
        }
    }

    // *运行代码
    else
    {     
        //读出标定的x y yaw
        calibrate.read();

        // 从TF上获取Action数据
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
        double yaw,roll, pitch;  
        tf::Quaternion RQ2;
        tf::quaternionMsgToTF(transformStamped.transform.rotation, RQ2);
        tf::Matrix3x3(RQ2).getRPY(roll, pitch, yaw);

        //如果进入内场
        if(action.y>3500 && action.x<3500 &&action.x >-4500)//
        {
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
            float max_x = (5.4 - (action.y / 1000.0) - (DrAction2DrLaser_x));
            float min_y = (-3.0 - (-action.x / 1000.0) - (DrAction2DrLaser_y));
            float max_y = (4.0 - (-action.x / 1000.0) - (DrAction2DrLaser_y));
            float min_angle = -PI / 2 - yaw - calibrate.DrActionYaw + PI/16;
            float max_angle = PI / 2 - yaw - calibrate.DrActionYaw- PI/16;
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

            // 通过滤波结果计算出圆心
            index2center(left, lidar.nowData, coordinate.left_xyR);
            index2center(middle, lidar.nowData, coordinate.middle_xyR);
            index2center(right, lidar.nowData, coordinate.right_xyR);

            // 将雷达的x y旋转到世界坐标
            coordinate_rotation(
                &(coordinate.left_xyR[0]),
                &(coordinate.left_xyR[1]),
                yaw+calibrate.DrActionYaw);
            coordinate_rotation(
                &(coordinate.middle_xyR[0]),
                &(coordinate.middle_xyR[1]),
                yaw+calibrate.DrActionYaw);
            coordinate_rotation(
                &(coordinate.right_xyR[0]),
                &(coordinate.right_xyR[1]),
                yaw+calibrate.DrActionYaw);

            // 转换到以TR起点的世界坐标
            change2worldCoordinate(coordinate.left_xyR[0], coordinate.left_xyR[1],
                                action.x, action.y,
                                DrAction2DrLaser_x, DrAction2DrLaser_y,
                                &coordinate.left_x, &coordinate.left_y);
            change2worldCoordinate(coordinate.middle_xyR[0], coordinate.middle_xyR[1],
                                action.x, action.y,
                                DrAction2DrLaser_x, DrAction2DrLaser_y,
                                &coordinate.middle_x, &coordinate.middle_y);
            change2worldCoordinate(coordinate.right_xyR[0], coordinate.right_xyR[1],
                                action.x, action.y,
                                DrAction2DrLaser_x, DrAction2DrLaser_y,
                                &coordinate.right_x, &coordinate.right_y);
            
            // 限幅
            coordinate.left_x=limit(coordinate.left_x,-6000,-5300);//-5850 -5500
            coordinate.left_y=limit(coordinate.left_y,7500,8500);//7650 8350
            coordinate.middle_x=limit(coordinate.middle_x,-6000,-5300);//-5850 -5500
            coordinate.middle_y=limit(coordinate.middle_y,5000,6000);//5150 5850
            coordinate.right_x=limit(coordinate.right_x,-6000,-5300);//-5850 -5500
            coordinate.right_y=limit(coordinate.right_y,2500,3500);//2650 3350

            //串口发送 如果为0即为识别不到
            TR_SerialWrite(coordinate.left_x, coordinate.left_y,
                        coordinate.middle_x, coordinate.middle_y,
                        coordinate.right_x, coordinate.right_y, 0x07);


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
                DR_SerialWrite(pot.x,pot.y,pot.alpha*10000.0,pot.index);
                // ROS_INFO("%d %d %f %d", pot.x, pot.y, pot.alpha * 180 / PI, pot.index); //发送的数据要转到DR坐标系
            }
            else
            {
                DR_SerialWrite(0,0,0,0x00);
            }

            //坐标发布
            coordinate.prePublish();
            coordinate_info_pub.publish(coordinate.coordinate_msg);
  
        }

        //保存
        std::string path = "./src/rc_laserscan/data/log/log" + run_code_date + ".txt";
        std::vector<float> raw_data(scan->ranges);
        save_data(path, DATA_NUM,
                  raw_data, lidar.nowData,
                  coordinate.left_x, coordinate.left_y,
                  coordinate.middle_x, coordinate.middle_y,
                  coordinate.right_x, coordinate.right_y,
                  coordinate.left_xyR[0], coordinate.left_xyR[1],
                  coordinate.middle_xyR[0], coordinate.middle_xyR[1],
                  coordinate.right_xyR[0], coordinate.right_xyR[1],
                  action.x, action.y, yaw,
                  pot.x, pot.y, pot.alpha, pot.index);
        
        

        // //打印等待信息
        // printf("\rNow is running code...");
        // fflush(stdout);
    }

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

    tf2_ros::TransformListener tfListener(tfBuffer);
    //TR无线串口初始化
    TR_SerialInit();

    //雷达初始化
    lidar.init();

    // 初始化日期 用于保存
    run_code_date = date_init();


    //读取命令行 eg. _calib:=true
    bool isCalib;
    ros::param::get("main/calib", isCalib);
    calibrate.init(isCalib);

    std::string scan_name;
    if(isCalib)
    {
        scan_name = "/scan";
    }
    else{
        scan_name ="/new_scan";
    }

    //定义Publisher 发布滤波后坐标信息
    coordinate_info_pub = n.advertise<rc_laserscan::coordinate>("/coordinate_info", 10);

    //定义Publisher 发布标定error信息
    error_info_pub = n.advertise<rc_laserscan::error>("/error_info", 10);

    //定义Publisher 发布滤波后雷达信息
    laser_pub = n.advertise<sensor_msgs::LaserScan>("/filter", 10);

    //创建Subscriber 
    ros::Subscriber laser_sub = n.subscribe(scan_name, 10, laser_callback);

    //等待回调
    ros::spin();

    return 0;
}
