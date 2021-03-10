#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "kalman.h"
#include "laser_filter.h"
#include "TR_communication.h"
#include "coordinate_convert.h"

/**************
 * 本份代码用于雷达 确定三个转过的壶的全场坐标
 * 输入 DR相对于DR启动区的全场坐标 场地图 以右为x正 以上为y正
 * 输出 三个转动的壶相对TR启动区的全场坐标
 * **************/

/*
! 三等分是以车子在正中心的解决办法
TODO 偏航角目前为0 后续需将其加入代码中
*/

Kalman kalman_vec[DATA_NUM];
void laser_callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    // 初始化角度数据
    std::vector<float> THETA(DATA_NUM);   
    for(int i =0;i<DATA_NUM;i++)
   {
       THETA[i] =-3*PI/4 +ANGLE_INCREMENT*i;
   }

    //数据
    std::vector<float> laser_data(scan->ranges);
  
    //选择所要处理数据的范围 半径
    for (int i =0;i<DATA_NUM;i++)
    {
        if (laser_data[i] >DISTANCE_MAX)
        {
            laser_data[i] = 0;
        }
    }

    //选择所要处理数据的范围 角度
    for (int i =0;i<DATA_NUM;i++)
    {
        if (THETA[i]< -PI/2 ||THETA[i]>PI/2)
        {
            laser_data[i] = 0;  
        }
    }
   
    // 去除离群点outliers
    remove_outlier(laser_data,0.07,3);

    // 滤出圆弧
    get_circle(laser_data,0.03);//0.008

    //数量滤波
    num_filter(laser_data);

    //卡尔曼滤波
    for(int i =0;i<DATA_NUM;i++)
    {
        if(laser_data[i]==0)
        {
            continue;
        }
        else
        {
            kalman_vec[i].KalmanFilter(laser_data[i]);
            laser_data[i] = kalman_vec[i].filterValue;
        }      
    }

    //获得连续段
    std::vector<float> final_start_index;
    std::vector<float> final_end_index;
    splinter_continuous_part(laser_data,final_start_index,final_end_index);

    //滤出圆弧的角度范围 三等分
    float split_min_angle = -PI / 6;
    float split_max_angle = PI / 6;
    float split_min_index = (split_min_angle + 3 * PI / 4) / (3 * PI / 2) * 1080;
    float split_max_index = (split_max_angle + 3 * PI / 4) / (3 * PI / 2) * 1080;
    std::vector<float> left,middle,right;
    for(int i =0;i<final_start_index.size();i++)
    {
        if(final_start_index[i]<split_min_index)
        {
            right.push_back(final_start_index[i]);
            right.push_back(final_end_index[i]);
        }
        else if(final_start_index[i]<split_max_index)
        {
            middle.push_back(final_start_index[i]);
            middle.push_back(final_end_index[i]);
        }
        else
        {
            left.push_back(final_start_index[i]);
            left.push_back(final_end_index[i]);
        }
    }

    if(left.size() %2 !=0 || middle.size() %2 !=0 || right.size() %2 !=0)
    {
        ROS_ERROR(" 取余2!=0");
    }

    if(left.size() >4 || middle.size() >4 || right.size() >4)
    {
        ROS_ERROR(" two more in one part!");
    }

    if(left.size() ==0 || middle.size() ==0 || right.size() ==0)
    {
        ROS_ERROR(" zero in one part!");
    }
    
    //分别获得 左中右三个壶的坐标
    std::vector<float> left_xyR(3),middle_xyR(3),right_xyR(3);
    index2center(left,laser_data,left_xyR);
    index2center(middle,laser_data,middle_xyR);
    index2center(right,laser_data,right_xyR);
    ROS_INFO("left:%f,%f,middle:%f,%f,right:%f,%f",
    left_xyR[0],left_xyR[1],middle_xyR[0],middle_xyR[1],right_xyR[0],right_xyR[1]);
    
    //转换到TR世界坐标系
    float left_x,left_y,middle_x,middle_y,right_x,right_y;
    change_to_TR_coordinate(left_xyR[0],left_xyR[1],10900,5000,&left_x,&left_y);
    change_to_TR_coordinate(middle_xyR[0],middle_xyR[1],10900,5000,&left_x,&left_y);
    change_to_TR_coordinate(right_xyR[0],right_xyR[1],10900,5000,&left_x,&left_y);

    //打印并串口发送
    ROS_INFO("left:%f,%f,middle:%f,%f,right:%f,%f",left_x,left_y,middle_x,middle_y,right_x,right_y);
     if(left_xyR[0]*left_xyR[1]*middle_xyR[0]*middle_xyR[1]*right_xyR[0]*right_xyR[1]!=0)
    {
        TR_SerialWrite(left_x,left_y,middle_x,middle_y,right_x,right_y,0x07);
    }

}

int main(int argc, char **argv)
{
    //串口初始化
    TR_SerialInit();

    // 初始化ROS节点
    ros::init(argc, argv, "Laser");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber
    ros::Subscriber pose_sub = n.subscribe("/scan", 10, laser_callback);
    
    // 循环等待回调函数
    ros::spin();

    return 0;
}
