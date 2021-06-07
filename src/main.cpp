#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "kalman.h"
#include "laser_filter.h"
#include "TR_communication.h"
#include "coordinate_convert.h"
#include "save_data.h"
#include "pots_coordinates/coordinate.h"
#include "DR_communication.h"
#include "action.h"

/**************
 * 本份代码用于雷达 确定三个转过的壶的全场坐标
 * 输入 DR相对于DR启动区的全场坐标 场地图 以右为x正 以上为y正
 * 输出 三个转动的壶相对TR启动区的全场坐标
 * **************/

/*
! 三等分是以车子在正中心的解决办法 已经写好根据全场定位的解决办法 待调试
! 要根据全场定位坐标 滤去对方区域的壶
TODO 偏航角目前为0 后续需将其加入代码中
* error 约等于0.01 失误率8％
*/

bool calibrate = true;//是否标定
std::string  run_code_date;
ros::Publisher laser_pub; //发布滤波后的雷达数据的Publisher 用于rviz查看滤波效果
ros::Publisher coordinate_info_pub;//发布雷达获取到左中右三个壶的坐标 用于画图可视化
pots_coordinates::coordinate coordinate_msg;//坐标消息类型类
Action action;//Action

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    // 初始化角度数据
    std::vector<float> THETA(DATA_NUM);   
    for(int i = 0;i < DATA_NUM;i++)
    {
        THETA[i] =(-3*PI)/4 + ANGLE_INCREMENT * i;
    }
    
    //获取数据
    std::vector<float> laser_data(scan->ranges);

    /*************标定**************/
    if(calibrate)
    {
        const float DistanceMax_Calib =4.0;
        for(int i = 0;i < DATA_NUM;i++)
        {
            if(laser_data[i] > DistanceMax_Calib){laser_data[i] = 0;} //半径滤波
            if((THETA[i]<(-PI)/4) || (THETA[i]>(PI)/4)){laser_data[i] = 0;}//角度滤波
        }
        // 去除离群点outliers
        remove_outlier(laser_data,THETA,0.07,3);

        // 滤出圆弧
        get_circle(laser_data,THETA,0.03);//0.008

        //数量滤波
        const int DataMinNum_Calib =static_cast<int>(2 * asin(R / DistanceMax_Calib) / ANGLE_INCREMENT);
        num_filter(laser_data,DataMinNum_Calib); 

        //获得连续段
        std::vector<float> calib_start_index;
        std::vector<float> calib_end_index;
        splinter_continuous_part(laser_data,calib_start_index,calib_end_index);    

        std::vector<float> calib_left,calib_right;
        if(calib_start_index.size()==2)
        {
            calib_right.push_back(calib_start_index[0]);
            calib_right.push_back(calib_end_index[0]);
            calib_left.push_back(calib_start_index[1]);
            calib_left.push_back(calib_end_index[1]);
        }
        else
        {
            ROS_ERROR("calib error");
        }

        std::vector<float> calib_left_xyR(3),calib_right_xyR(3);
        index2center(calib_left,laser_data,calib_left_xyR);
        index2center(calib_right,laser_data,calib_right_xyR);
        // ROS_INFO("left:%f %f,right:%f %f",
        //             calib_left_xyR[0],calib_left_xyR[1],
        //             calib_right_xyR[0],calib_right_xyR[1]);

        //理论机器人中心离两个壶的距离 按DR坐标系下x右y
        int DrStartPoint2Left_x  = -3500;
        int DrStartPoint2Left_y  = -780;
        int DrStartPoint2Right_x = -3500;
        int DrStartPoint2Right_y = -220;

        // //右x 上y
        ROS_INFO("%f %f %f %f",
        -DrStartPoint2Left_x  - calib_left_xyR[0]*1000,
        DrStartPoint2Left_y  + calib_left_xyR[1]*1000, //负数偏左 整数偏右
        -DrStartPoint2Right_x - calib_right_xyR[0]*1000,
        DrStartPoint2Right_y + calib_right_xyR[1]*1000);
        //发布滤波后的数据
        sensor_msgs::LaserScan filter_result;
        filter_result.header.stamp = ros::Time::now();
        filter_result.header.frame_id = "laser";
        filter_result.angle_min = (-3*PI)/4;
        filter_result.angle_max = ( 3*PI)/4;
        filter_result.angle_increment = scan->angle_increment;
        filter_result.time_increment = scan->time_increment;
        filter_result.scan_time = scan->scan_time;
        filter_result.range_min = scan->range_min;
        filter_result.range_max = scan->range_max;  
        for(int i = 0; i < DATA_NUM; i++)
        {
        filter_result.ranges.push_back(laser_data[i]);
        filter_result.intensities.push_back(scan->intensities[i]);
        }
        laser_pub.publish(filter_result);

    }
    else
    {
        //接受action数据
        // DR_SerialRead(action.x,action.y,action.yaw,action.flag);
    
        //选择所要处理数据的范围
        const float DistanceMax = 4.0 ; //车子离圆柱中心的最大距离
        const int DataMinNum = static_cast<int>(2 * asin(R / DistanceMax) / ANGLE_INCREMENT);

        for(int i = 0;i < DATA_NUM;i++)
        {
            if(laser_data[i] > DistanceMax){laser_data[i] = 0;} //半径滤波
            if((THETA[i]<(-PI)/2) || (THETA[i]>PI/2)){laser_data[i] = 0;}//角度滤波
            //TODO 按全场坐标 
            if(fabs(cos(THETA[i])*laser_data[i]) > 1.2){laser_data[i] =0;}//滤去对面的壶
        }
        
        // 去除离群点outliers
        remove_outlier(laser_data,THETA,0.07,3);

        // 滤出圆弧
        get_circle(laser_data,THETA,0.03);//0.008

        //数量滤波
        num_filter(laser_data,DataMinNum);


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

        //********************根据全场定位测试*******************//
        // if(final_start_index.size() ==3)
        // {
        //     left.push_back(final_start_index[0]);
        //     left.push_back(final_end_index[0]);
        //     middle.push_back(final_start_index[1]);
        //     middle.push_back(final_end_index[1]);
        //     right.push_back(final_start_index[2]);
        //     right.push_back(final_end_index[2]);
        // }
        // else if(final_start_index.size() ==2)
        // {
        //     //TODO 根据全场定位确定是左中 还是中右壶
        // }
        // else if(final_start_index.size() ==1)
        // {
        //     //TODO 根据全场定位确定哪一个壶
        // }
        // else
        // {
        //     ROS_ERROR(" 检测不到壶或者多于3个壶！");
        // }

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
        //TODO 后续可能要改
        index2center(left,laser_data,left_xyR);
        index2center(middle,laser_data,middle_xyR);
        index2center(right,laser_data,right_xyR);

        //转换到TR世界坐标系
        int left_x,left_y,middle_x,middle_y,right_x,right_y;
        change_to_TR_coordinate(left_xyR[0],left_xyR[1],500,-4300,&left_x,&left_y);
        change_to_TR_coordinate(middle_xyR[0],middle_xyR[1],500,-4300,&middle_x,&middle_y);
        change_to_TR_coordinate(right_xyR[0],right_xyR[1],500,-4300,&right_x,&right_y);

        //保存
        std::string path ="./data/log/log"+run_code_date+".txt";
        std::vector<float> raw_data(scan->ranges);
        save_data(path,DATA_NUM,raw_data,laser_data,left_x,left_y,middle_x,middle_y,right_x,right_y);
        
        //发布坐标数据 用于python画图
        if(left_x*left_y*middle_x*middle_y*right_x*right_y!=0)
        {
            //坐标发布
            coordinate_msg.left_x = left_x;
            coordinate_msg.left_y = left_y;
            coordinate_msg.middle_x =middle_x;
            coordinate_msg.middle_y =middle_y;
            coordinate_msg.right_x =right_x;
            coordinate_msg.right_y =right_y;
            coordinate_info_pub.publish(coordinate_msg);

            //串口发送 
            TR_SerialWrite(left_x,left_y,middle_x,middle_y,right_x,right_y,0x07);       
        }

        //发布滤波后的数据
        sensor_msgs::LaserScan filter_result;
        filter_result.header.stamp = ros::Time::now();
        filter_result.header.frame_id = "laser";
        filter_result.angle_min = (-3*PI)/4;
        filter_result.angle_max = ( 3*PI)/4;
        filter_result.angle_increment = scan->angle_increment;
        filter_result.time_increment = scan->time_increment;
        filter_result.scan_time = scan->scan_time;
        filter_result.range_min = scan->range_min;
        filter_result.range_max = scan->range_max;  
        for(int i = 0; i < DATA_NUM; i++)
        {
        filter_result.ranges.push_back(laser_data[i]);
        filter_result.intensities.push_back(scan->intensities[i]);
        }
        laser_pub.publish(filter_result);

        //打印等待信息
        printf("\rNow is running code...");
        fflush(stdout);
    }
}


int main(int argc, char **argv)
{
    //DR串口初始化
    DR_SerialInit();
    
    //TR无线串口初始化
    TR_SerialInit();
    
    // 初始化日期
    run_code_date = date_init();

    // 初始化ROS节点
    ros::init(argc, argv, "Laser");

    // 创建节点句柄
    ros::NodeHandle n;

    //定义Publisher 发布滤波后坐标信息
    coordinate_info_pub = n.advertise<pots_coordinates::coordinate>("/coordinate_info", 10);

    //定义Publisher 发布滤波后雷达信息
    laser_pub = n.advertise<sensor_msgs::LaserScan>("/filter", 10);

    //更新调试次数
    update_times();

    // 创建一个Subscriber 回调函数
    ros::Subscriber pose_sub = n.subscribe("/scan", 1, laser_callback);

    // 定义Subscriber队列长度为1 通过设置循环频率 即可设置回调函数频率
    // 雷达接受fps大概为30 消息循环频率大概最高是20Hz
    // 但是存在的问题就是ctrl+c后界面一直存在
    ros::spin();
    // ros::Rate loop_rate(20);
    // while(true){ros::spinOnce();loop_rate.sleep();}

    return 0;
  
}
