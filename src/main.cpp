#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "kalman.h"
#include "laser_filter.h"
#include "TR_communication.h"
#include "coordinate_convert.h"
#include "save_data.h"
#include "pots_coordinates/coordinate.h"
#include "lidar.h"
#include "DR_communication.h"
#include "action.h"
#include "calibration.h"

/**************
 * 本份代码用于雷达 确定三个转过的壶的全场坐标
 * 输入 DR相对于DR启动区的全场坐标 场地图 以右为x正 以上为y正
 * 输出 三个转动的壶相对TR启动区的全场坐标
 * **************/
// TODO 标定y的误差还有点大
/*
! 三等分是以车子在正中心的解决办法 已经写好根据全场定位的解决办法 待调试
! 要根据全场定位坐标 滤去对方区域的壶
TODO 偏航角目前为0 后续需将其加入代码中
* error 约等于0.01 失误率8％
*/


std::string  run_code_date;

ros::Publisher laser_pub; //发布滤波后的雷达数据的Publisher 用于rviz查看滤波效果
ros::Publisher coordinate_info_pub;//发布雷达获取到左中右三个壶的坐标 用于画图可视化
ros::Publisher error_info_pub;//发布滤波时的error信息 用于画图可视化

pots_coordinates::coordinate coordinate_msg;//坐标消息类型类
pots_coordinates::error error_msg;//error消息类型类

Action action;
Calibrate calibrate;
Lidar lidar;
Filter filter;

std::vector<float> last_laser_data(DATA_NUM);
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //获取数据
    lidar.getData(scan);

    //时域中值滤波
    filter.median_filter(lidar.nowData,lidar.lastData);
    
    

    /*************标定**************/
    if(calibrate.enable)
    {
        if(!calibrate.success)
        {
            calibrate.calibrate(lidar.nowData,lidar.THETA,filter);
            error_info_pub.publish(calibrate.error_msg);
        }
        else
        {
            calibrate.save(run_code_date);
        }
	}

    /***************************/
    else
    {
        //接受action数据
        // DR_SerialRead(action.x,action.y,action.yaw,action.flag);
    
        //选择所要处理数据的范围
        float DistanceMax = 4.0 ; //车子离圆柱中心的最大距离
        filter.easy_filter(lidar.nowData,lidar.THETA,
                            true,DistanceMax,
                            true,(-PI/2),(PI/2),
                            true,0,1.2,
                            false,0,0);
   
 
        // 去除离群点outliers
        filter.remove_outlier(lidar.nowData,lidar.THETA,0.07,3);

        // 滤出圆弧
        filter.get_circle(lidar.nowData,lidar.THETA,0.03);//0.008

        //数量滤波
        int DataMinNum = static_cast<int>(2 * asin(R / DistanceMax) / ANGLE_INCREMENT);
        filter.num_filter(lidar.nowData,DataMinNum);


        //获得连续段
        std::vector<float> final_start_index;
        std::vector<float> final_end_index;
        filter.splinter_continuous_part(lidar.nowData,final_start_index,final_end_index);

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
        filter.index2center(left,lidar.nowData,left_xyR);
        filter.index2center(middle,lidar.nowData,middle_xyR);
        filter.index2center(right,lidar.nowData,right_xyR);

        //转换到TR世界坐标系
        int left_x,left_y,middle_x,middle_y,right_x,right_y;
        change_to_TR_coordinate(left_xyR[0],left_xyR[1],500,-4300,&left_x,&left_y);
        change_to_TR_coordinate(middle_xyR[0],middle_xyR[1],500,-4300,&middle_x,&middle_y);
        change_to_TR_coordinate(right_xyR[0],right_xyR[1],500,-4300,&right_x,&right_y);

        //保存
        std::string path ="./data/log/log"+run_code_date+".txt";
        std::vector<float> raw_data(scan->ranges);
        save_data(path,DATA_NUM,raw_data,lidar.nowData,left_x,left_y,middle_x,middle_y,right_x,right_y);
        
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

        

        //打印等待信息
        printf("\rNow is running code...");
        fflush(stdout);
    }

    //发布数据用于rviz
    lidar.prePublish(scan);
    laser_pub.publish(lidar.result);

}


int main(int argc, char **argv)
{
    //DR串口初始化
    DR_SerialInit();
    
    //TR无线串口初始化
    TR_SerialInit();

    lidar.init();

    
    // 初始化日期
    run_code_date = date_init();

    // 初始化ROS节点
    ros::init(argc, argv, "Laser");

    // 创建节点句柄
    ros::NodeHandle n;

    //定义Publisher 发布滤波后坐标信息
    coordinate_info_pub = n.advertise<pots_coordinates::coordinate>("/coordinate_info", 10);
    
    //定义Publisher 发布滤波后坐标信息
    error_info_pub = n.advertise<pots_coordinates::error>("/error_info", 10);

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
