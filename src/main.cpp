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
#include "pot_grab.h"

/**************
 * 本份代码用于雷达 确定三个转过的壶的全场坐标 
 * TR坐标系 上x左y
 * DR坐标系 右x上y
 * 输入 DR相对于DR启动区的全场坐标 场地图
 * 输出 三个转动的壶相对TR启动区的全场坐标
 * **************/


// 改代码 
//TODO 试试加上卡尔曼
//TODO 标定边沿处理有一点问题
/////TODO rviz 没法save
//TODO 保存坐标溢出 
//TODO 运动补偿
//TODO 代码优雅 ctrl+win+t ctrl+win+i
/////TODO 角度滤波要改成世界坐标
//!todo 壶坐标突变90° 

// 测试
//todo 

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
PotGrab pot;
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
	
#if 0
printf("calib data :x:%f y:%f yaw:%f\n",calibrate.DrAction2DrLaser_x,calibrate.DrAction2DrLaser_y,calibrate.DrActionYaw);
fflush(stdout);
#endif

	    DR_SerialRead(action.x,action.y,action.yaw,action.flag); 
		// action.x+=1000;
        // action.y+=4300;
        // if(action.yaw>0){action.yaw-=PI*10000;}
        // else{action.yaw+=PI*10000;}
     
        // 因为车子转了超过PI 溢出action(-PI,PI) 所以最终为 向左偏为-PI+theta 向右为PI-theta
        // 处理-PI到PI的突变
        // 车子的偏航角 action 偏左为正 偏右为负
        float yaw;
        if((action.yaw==31415) || (action.yaw==-31415) || (action.yaw==0))
        {
            yaw=0;
        }
        else if(action.yaw<0)
        {
            yaw = action.yaw/10000.0+PI;
        }
        else // >0
        {
            yaw = action.yaw/10000.0-PI;
        }

        // 将标定的x y旋转到世界坐标
        float distance =sqrt(
            calibrate.DrAction2DrLaser_x*calibrate.DrAction2DrLaser_x+
            calibrate.DrAction2DrLaser_y*calibrate.DrAction2DrLaser_y);
        float DrAction2DrLaser_x = distance *cos(yaw);
        float DrAction2DrLaser_y = distance *sin(yaw); 

        //根据全场定位滤波
        float min_x = 0;
        float max_x = ( 5.4-(action.y/1000.0)-(DrAction2DrLaser_x));//5.5
        float min_y = (-3.0-(-action.x/1000.0)-(DrAction2DrLaser_y));//-3.5
        float max_y = ( 4.0-(-action.x/1000.0)-(DrAction2DrLaser_y));//4.5
        float min_angle =-PI/2-yaw-calibrate.DrActionYaw;
        float max_angle = PI/2-yaw-calibrate.DrActionYaw;
        filter.easy_filter(lidar.nowData,lidar.THETA,
                            false,0.0,
                            true,min_angle,max_angle,
			                true,min_x,max_x,min_y,max_y,yaw);
   
        // 去除离群点outliers
        filter.remove_outlier(lidar.nowData,lidar.THETA,0.07,3);

        // 滤出圆弧
        filter.get_circle(lidar.nowData,lidar.THETA,0.03);//0.008

        //数量滤波
        int DataMinNum = 20;
        filter.num_filter(lidar.nowData,DataMinNum);

        //卡尔曼滤波
        //filter.kalman_filter(lidar.nowData,lidar.lastData,lidar.THETA,action.x,action.y);

        //获得连续段
        std::vector<float> start_index;//连续段的起始坐标
        std::vector<float> end_index;//连续段的终止坐标
        splinter_continuous_part(lidar.nowData,start_index,end_index);

        std::vector<float> left(2),middle(2),right(2);
        distributeData(start_index,end_index,
                       left,middle,right,
                       lidar.THETA,action.x);

        //通过滤波结果计算出圆心
        index2center(left,lidar.nowData,coordinate.left_xyR);
        index2center(middle,lidar.nowData,coordinate.middle_xyR);
        index2center(right,lidar.nowData,coordinate.right_xyR);

        // (action 偏左为正 偏右为负)、(函数传入正值右转)
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

        
        //转换到以TR起点的世界坐标
        change2worldCoordinate(coordinate.left_xyR[0],coordinate.left_xyR[1],
                                action.x,action.y,
                                DrAction2DrLaser_x,DrAction2DrLaser_y,
                                &coordinate.left_x,&coordinate.left_y);
        change2worldCoordinate(coordinate.middle_xyR[0],coordinate.middle_xyR[1],
                                action.x,action.y,
                                DrAction2DrLaser_x,DrAction2DrLaser_y,
                                &coordinate.middle_x,&coordinate.middle_y);
        change2worldCoordinate(coordinate.right_xyR[0],coordinate.right_xyR[1],
                                action.x,action.y,
                                DrAction2DrLaser_x,DrAction2DrLaser_y,
                                &coordinate.right_x,&coordinate.right_y);

        

        //串口发送 如果为0即为识别不到
        TR_SerialWrite(coordinate.left_x,coordinate.left_y,
                       coordinate.middle_x,coordinate.middle_y,
                       coordinate.right_x,coordinate.right_y,0x07);       
        
        

#if 1
        //发送DR要抓的壶的信息
        DistanceIndex left_struct,middle_struct,right_struct;
        left_struct.init(coordinate.left_xyR,0x03);
        middle_struct.init(coordinate.middle_xyR,0x02);
        right_struct.init(coordinate.right_xyR,0x01);


        std::vector<DistanceIndex> GetMinVec;
        if(left_struct.distance!=0){GetMinVec.push_back(left_struct);}
        if(middle_struct.distance!=0){GetMinVec.push_back(middle_struct);}
        if(right_struct.distance!=0){GetMinVec.push_back(right_struct);}

        if(GetMinVec.size()>0)
        {
            std::sort(GetMinVec.begin(),GetMinVec.end(),[](DistanceIndex a,DistanceIndex b){return a.distance<b.distance;});//升序排序
            pot.preSend(GetMinVec[0].index,coordinate);
            DR_SerialWrite(pot.x,pot.y,pot.alpha*10000.0,pot.index); 
            ROS_INFO("%d %d %f %d",pot.x,pot.y,pot.alpha*180/PI,pot.index);//发送的数据要转到DR坐标系
        }
        else
        {
            DR_SerialWrite(0,0,0,0x00); 
        }
#endif

        //保存 
        std::string path ="./data/log/log"+run_code_date+".txt";
        std::vector<float> raw_data(scan->ranges);
        save_data(path,DATA_NUM,
                  raw_data,lidar.nowData,
                  coordinate.left_x,coordinate.left_y,
                  coordinate.middle_x,coordinate.middle_y,
                  coordinate.right_x,coordinate.right_y,
                  coordinate.left_xyR[0],coordinate.left_xyR[1],
                  coordinate.middle_xyR[0],coordinate.middle_xyR[1],
                  coordinate.right_xyR[0],coordinate.right_xyR[1],
                  action.x,action.y,action.yaw,
                  pot.x,pot.y,pot.alpha,pot.index);
        
        //坐标发布
        coordinate.prePublish();
        coordinate_info_pub.publish(coordinate.coordinate_msg);

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

    // 初始化ROS节点
    ros::init(argc, argv, "Laser");

    // 创建节点句柄
    ros::NodeHandle n;

    //DR串口初始化
    DR_SerialInit();
 
    //TR无线串口初始化
    TR_SerialInit();

    //雷达初始化
    lidar.init();
   
    // 初始化日期 用于保存
    run_code_date = date_init();

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
