#include "calibration.h"

/**
 * @brief: 根据flag决定是否标定
 * @param {bool} flag
 * @return {*}
 * @author: bonbon
 */
void Calibrate::init(bool flag)
{
    enable = flag;
    if(enable)
    {
        printf("Calibrate mode\n");
        fflush(stdout);
    }
    else
    {
        printf("Running code mode\n");
        fflush(stdout);
    }
}

/**
 * @brief: 标定函数
 * @param laser_data 雷达数据
 * @param theta 角度数据
 * @param filter 滤波类
 * @return {*}
 * @author: bonbon
 */
void Calibrate::calibrate(std::vector<float> &laser_data,
                          std::vector<float> &theta,
                          Filter &filter)
{
    //滤波
    float CalibDistanceMax =1.0;
    filter.easy_filter(laser_data,theta,
                            true,CalibDistanceMax,
                            true,(-PI/2),(PI/2),
                            false,0,0,0,0,0);
	
    //获得连续段
    std::vector<float> start_index;//连续段开始坐标
    std::vector<float> end_index;//连续段结束坐标
    splinter_continuous_part(laser_data,start_index,end_index);

    if(start_index.size()!=1 ||end_index.size()!=1)
    {
        ROS_ERROR("Calibration filter error");
    }
    else
    {
        /*******标定x和yaw(yaw左偏为正 右偏为负)*********/
        if(calibrate_x_yaw)
        {
            int MiddleIndex = (DATA_NUM+1)/2-1;//以雷达最中央采样 即雷达的0度
            int index1 = std::min(
                abs(start_index[0]-MiddleIndex),
                abs(end_index[0]-MiddleIndex))
                -5;//平板标定最旁边的数据跳变很大要舍去
         
            float x_sum= 0;
            float yaw_sum= 0;
            for(int i=1;i<=index1;i++)
            {
                float r1 =laser_data[MiddleIndex - i];
                float r2 =laser_data[MiddleIndex + i];
            
                float theta1 = (1 / Resolution)*(PI / 180) * i; //rad
                float alpha =asin(
                    (sin(2*theta1)*r1)/
                    (sqrt(r1*r1+r2*r2-2*r1*r2*cos(2*theta1)))
                    );
                
                float x;
                float yaw;

                x = r2 * sin(alpha);
                yaw = PI/2 - alpha - theta1;//alpha<PI/2的话就是负数
                
                if(!(std::isnan(x)) && !(std::isnan(yaw)))
                {
                    x_sum += x;
                    yaw_sum += yaw;   
                }
            }
        
            if(laser_x!=0 && laser_yaw!=0)
            {
                x_error = fabs(x_sum/index1 -laser_x);
                yaw_error = fabs(yaw_sum/index1-laser_yaw);

                if(x_error <5*1e-5 && yaw_error<(PI/180)/16)
                {
                    calibrate_x_yaw = false;
                    calibrate_y = true;
                    printf("calibrate (x,yaw) successfully laser_x:%4f laser_yaw:%4f\n",laser_x,laser_yaw);
		            fflush(stdout);
                }
                else
                {
                    laser_x = (laser_x+ x_sum/index1)/2;
                    laser_yaw =(laser_yaw +yaw_sum/index1)/2;
                }       
            }
            else
            {
                laser_x =x_sum/index1;
                laser_yaw =yaw_sum/index1;
            }          
        }
            
        if(calibrate_y)
        {  
	    
            float y_sum=0;
            int offset = 5; //平板标定最旁边的数据跳变很大要舍去
            int index2=end_index[0] - start_index[0] - 2*offset;
            

            for(int i=0;i<index2;i++)
            {
                float r3 = laser_data[end_index[0]-offset-i];
                float theta2 = theta[end_index[0]-offset-i]+laser_yaw;
                float y1 =sin(theta2)*r3;
                float theta3 =(1 / Resolution)*(PI / 180) * (i+offset);
                float theta4 = PI/2 - theta2 -theta3;
                float y2 = r3*sin(theta3)/sin(theta4);
                float y= y1 + y2;

                // float r3 = laser_data[end_index[0]-offset];
                // float r4 = laser_data[end_index[0]-offset-i];
                // float theta2 = (1 / Resolution)*(PI / 180) * i;
                // float y1 =sqrt(r3*r3+r4*r4-2*r3*r4*cos(theta2));
                // float y2 =r4*sin(theta[end_index[0]-offset-i]+laser_yaw);
                // float delta_theta =(1 / Resolution)*(PI / 180) * offset;
                // float deltay=r3*sin(delta_theta)/(sin(asin(laser_x/r3)+delta_theta));		
                // float y = y1+y2+deltay;
                
                if(!(std::isnan(y)))
                {
                    y_sum+=y;
                }
            }
    
            if(laser_y!=0)
            {
                y_error = fabs(y_sum/index2 -laser_y);

                if(y_error <1e-5)
                {
                    calibrate_y = false;
                    success = true;
                    printf("calibrate y successfully laser_y:%4f\n",laser_y);
                    fflush(stdout);
                    isSave =true;

                    DrActionYaw = laser_yaw;
                    DrAction2DrLaser_x=DrAction2Calib_x-laser_x;//偏上为正
                    DrAction2DrLaser_y=DrAction2Calib_y-(laser_y-BoardWidth/2);//偏右为正
                }
                else
                {
                    laser_y = (laser_y+ y_sum/index2)/2;
                }
            }
            else
            {
                laser_y =y_sum/index2;
            }
        }

        error_msg.calib_x_error=x_error;
        error_msg.calib_y_error=y_error;
        error_msg.calib_yaw_error=yaw_error;
    
    }
}

/**
 * @brief: 保存标定数据到yaml里 直接ros参数服务器读取
 * @param {*}
 * @return {*}
 * @author: bonbon
 */
void Calibrate::save()
{  
    std::ofstream out(save_path,std::ios::out);
    out <<"x: "<<DrAction2DrLaser_x<<"\n"
        <<"y: "<<DrAction2DrLaser_y<<"\n"
        <<"yaw: "<<DrActionYaw<<"\n";
    out.close();

    printf("success x:%f y:%f yaw:%f\n",
            DrAction2DrLaser_x,
            DrAction2DrLaser_y,
            DrActionYaw);
    fflush(stdout);
    
    isSave = false;
}

/**
 * @brief: 读取标定数据
 * @param {*}
 * @return {*}
 * @author: bonbon
 */
void Calibrate::read()
{
    ros::param::get("x",DrAction2DrLaser_x);
    ros::param::get("y",DrAction2DrLaser_y);
    ros::param::get("yaw",DrActionYaw);
}


/**
 * @brief: 构造函数初始化标定error
 * @param {*}
 * @return {*}
 * @author: bonbon
 */
Calibrate::Calibrate()
{
    error_msg.calib_x_error=0;
    error_msg.calib_y_error=0;
    error_msg.calib_yaw_error=0;
}


Calibrate::~Calibrate()
{

}
