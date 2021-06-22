#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "TR_communication.h"
#include "DR_communication.h"
#include "coordinate.h"
#include "save.h"
#include "lidar.h"
#include "action.h"
#include "calibration.h"
#include "filter.h"
#include "get_center.h"

/**************
 * 本份代码用于雷达 确定三个转过的壶的全场坐标 TR坐标系 左y上x
 * 输入 DR相对于DR启动区的全场坐标 场地图 以右为x正 以上为y正
 * 输出 三个转动的壶相对TR启动区的全场坐标
 * **************/
//TODO 改代码 
/////TODO 1.标定y的误差还有点大 
/////TODO 2.坐标系变换要改
/////TODO 3.将读取enable改成读取命令行参数
//TODO 4.试试加上卡尔曼
/////TODO 5.参数服务器有bug
/////TODO 6.将标定读取从文件改为从参数服务器读取

//TODO 测试
//TODO 1.测试action数据格式
//TODO 2.测试根据action滤波效果
//TODO 3.测试移动时代码效果



//Publisher 发布雷达数据(用于rviz可视化) 坐标数据、标定error数据(用于python可视化)
ros::Publisher laser_pub;
ros::Publisher coordinate_info_pub;
ros::Publisher error_info_pub;

std::string  run_code_date;//代码开始运行的日期 作为保存的文件名

Action action;
Calibrate calibrate;
Lidar lidar;
Filter filter;
Coordinate coordinate;

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    //获取数据
    lidar.getData(scan);

    //时域中值滤波
    filter.median_filter(lidar.nowData,lidar.lastData);
    
    
    // *标定
    if(calibrate.enable)
    {
        if(!calibrate.success)
        {
            calibrate.calibrate(lidar.nowData,lidar.THETA,filter);
            error_info_pub.publish(calibrate.error_msg);
        }
        else
        {
            if(calibrate.isSave)
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

        //接受action数据
        DR_SerialRead(action.x,action.y,action.yaw,action.flag);
    
        //滤波
        float DistanceMax = 4.0 ; //车子离圆柱中心的最大距离
        filter.easy_filter(lidar.nowData,lidar.THETA,
                            true,DistanceMax,
                            true,(-PI/2),(PI/2),
                            true,0,(6,0-0.5)-(-action.x/1000)-(calibrate.DrAction2DrLaser_x),
                            false,0,0);
   
        // 去除离群点outliers
        filter.remove_outlier(lidar.nowData,lidar.THETA,0.07,3);

        // 滤出圆弧
        filter.get_circle(lidar.nowData,lidar.THETA,0.03);//0.008

        //数量滤波
        int DataMinNum = static_cast<int>(2 * asin(R / DistanceMax) / ANGLE_INCREMENT);
        filter.num_filter(lidar.nowData,DataMinNum);


        //获得连续段
        std::vector<float> start_index;//连续段的起始坐标
        std::vector<float> end_index;//连续段的终止坐标
        splinter_continuous_part(lidar.nowData,start_index,end_index);

        // //滤出圆弧的角度范围 三等分
        // float split_min_angle = -PI / 6;
        // float split_max_angle = PI / 6;
        // float split_min_index = (split_min_angle + 3 * PI / 4) / (3 * PI / 2) * 1080;
        // float split_max_index = (split_max_angle + 3 * PI / 4) / (3 * PI / 2) * 1080;
        // std::vector<float> left,middle,right;
        // for(int i =0;i<final_start_index.size();i++)
        // {
        //     if(final_start_index[i]<split_min_index)
        //     {
        //         right.push_back(final_start_index[i]);
        //         right.push_back(final_end_index[i]);
        //     }
        //     else if(final_start_index[i]<split_max_index)
        //     {
        //         middle.push_back(final_start_index[i]);
        //         middle.push_back(final_end_index[i]);
        //     }
        //     else
        //     {
        //         left.push_back(final_start_index[i]);
        //         left.push_back(final_end_index[i]);
        //     }
        // }

        //********************根据全场定位测试*******************//
        std::vector<float> left(2),middle(2),right(2);
        distributeData(start_index,end_index,
                       left,middle,right,
                       lidar.THETA,action.y);
        
        //通过滤波结果计算出圆心
        index2center(left,lidar.nowData,coordinate.left_xyR);
        index2center(middle,lidar.nowData,coordinate.middle_xyR);
        index2center(right,lidar.nowData,coordinate.right_xyR);


        /* 
        TODO 获得的坐标是雷达坐标系下的坐标
        TODO 需要先坐标旋转(action的yaw和标定的yaw)
        TODO 然后转到世界坐标(action的x y和标定的x和y) 
        */
        coordinate_rotation(&(coordinate.left_xyR[0]),&(coordinate.left_xyR[1]),action.yaw/10000);
        coordinate_rotation(&(coordinate.middle_xyR[0]),&(coordinate.middle_xyR[1]),action.yaw/10000);
        coordinate_rotation(&(coordinate.right_xyR[0]),&(coordinate.right_xyR[1]),action.yaw/10000);
        
        //转换到以tr起点的世界坐标
        // //转换到TR世界坐标系
        //int left_x,left_y,middle_x,middle_y,right_x,right_y;
        change2worldCoordinate(coordinate.left_xyR[0],coordinate.left_xyR[1],
                                action.x,action.y,
                                calibrate.DrAction2DrLaser_x,calibrate.DrAction2DrLaser_y,
                                &coordinate.left_x,&coordinate.left_y);
        change2worldCoordinate(coordinate.middle_xyR[0],coordinate.middle_xyR[1],
                                action.x,action.y,
                                calibrate.DrAction2DrLaser_x,calibrate.DrAction2DrLaser_y,
                                &coordinate.middle_x,&coordinate.middle_y);
        change2worldCoordinate(coordinate.right_xyR[0],coordinate.right_xyR[1],
                                action.x,action.y,
                                calibrate.DrAction2DrLaser_x,calibrate.DrAction2DrLaser_y,
                                &coordinate.right_x,&coordinate.right_y);

        //保存
        std::string path ="./data/log/log"+run_code_date+".txt";
        std::vector<float> raw_data(scan->ranges);
        save_data(path,DATA_NUM,
                  raw_data,lidar.nowData,
                  coordinate.left_x,coordinate.left_y,
                  coordinate.middle_x,coordinate.middle_y,
                  coordinate.right_x,coordinate.right_y);
        
        //坐标发布
        coordinate.prePublish();
        coordinate_info_pub.publish(coordinate.coordinate_msg);

        //串口发送 如果为0即为识别不到
        TR_SerialWrite(coordinate.left_x,coordinate.left_y,
                        coordinate.middle_x,coordinate.middle_y,
                        coordinate.right_x,coordinate.right_y,0x07);       
        
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

    //雷达初始化
    lidar.init();
   
    // 初始化日期 用于保存
    run_code_date = date_init();

    // 初始化ROS节点
    ros::init(argc, argv, "Laser");

    // 创建节点句柄
    ros::NodeHandle n;

    //设置参数服务器中标定的标志位 默认false
    ros::param::set("calib",false);

    //读取命令行 eg. _calib:=true
    bool isCalib;
    ros::param::get("Laser/calib",isCalib);
    calibrate.init(isCalib);

    //定义Publisher 发布滤波后坐标信息
    coordinate_info_pub = n.advertise<pots_coordinates::coordinate>("/coordinate_info", 10);
    
    //定义Publisher 发布滤波后坐标信息
    error_info_pub = n.advertise<pots_coordinates::error>("/error_info", 10);

    //定义Publisher 发布滤波后雷达信息
    laser_pub = n.advertise<sensor_msgs::LaserScan>("/filter", 10);

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
