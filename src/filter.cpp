#include "filter.h"
//雷达数据逆时针-3pi/4到 3pi/4

/**
 * @brief: 去除离散点 指定半径数据少于k个则认为是离散点
 * @param {float} radius 半径
 * @param {int} k 数据个数
 * @return {*}
 * @author: bonbon
 */
void Filter::remove_outlier(std::vector<float>&data,std::vector<float>&theta,float radius,int k)
{
    for(int i =0;i<DATA_NUM;i++)
    {
        if (data[i] ==0)
        {
            continue;
        }

        int cnt =0;
        for(int j =0;j<DATA_NUM;j++)
        {
            if(i==j)
            {
                continue;
            }
            if (sqrt(data[i] * data[i] + data[j] * data[j]- 2 * fabs(cos(theta[i] - theta[j]))* data[i] * data[j]) < radius)
            {
                cnt ++;
            }              
        }
        if (cnt <= k)
        {
            data[i] = 0;
        }
    }   
}



/**
 * @brief: 分离出圆弧数据
 * @param {float} deviation 离散点离正确值的差值阈值
 * @return {*}
 * @author: bonbon
 */
void Filter::get_circle(std::vector<float>&data,std::vector<float>&theta,float deviation)
{
    std::vector<float>start_index;
    std::vector<float>end_index;

    splinter_continuous_part(data,start_index,end_index);
    for(int i=0;i<start_index.size();i++)
    {
        int middle = static_cast<int>((start_index[i] + end_index[i]-1)/ 2);
        float x0 = (data[middle] + R) * cos(theta[middle]);
        float y0 = (data[middle] + R) * sin(theta[middle]);
        for(int j =start_index[i];j<end_index[i];j++)
        {
            float x1 = data[j] * cos(theta[j]);
            float y1 = data[j] * sin(theta[j]);
            if ((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1) - R * R > deviation)
            {
                data[j] = 0;
            }                
        }
    }
}

/**
 * @brief: 根据连续段数量滤波
 * @param {const int} MinNumber 数据个数最小值
 * @return {*}
 * @author: bonbon
 */
void Filter::num_filter(std::vector<float>&data,const int MinNumber)
{
    std::vector<float>start_index;
    std::vector<float>end_index;

    splinter_continuous_part(data,start_index,end_index);

    for(int i =0;i<start_index.size();i++)
    {
        if(end_index[i] - start_index[i] - 1 < MinNumber)
        {
            for(int j =start_index[i];j<end_index[i];j++)
            {
                data[j] =0;
            }
        }
    }
}

/**
 * @brief: 时域中值滤波
 * @param 
 * @return {*}
 * @author: bonbon
 */
void Filter::median_filter(std::vector<float> & nowData,std::vector<float> & lastData)
{
    //时域均值滤波
    for(int i=0;i<DATA_NUM;i++)
    {
        if(lastData[i]!=0)
        {
            nowData[i] = (lastData[i]+nowData[i])/2;
        }
        lastData[i] =nowData[i];
    }
}

/**
 * @brief: 简单滤去对应半径、角度、xy以外的数据
 * @param {bool} radiusFilter 是否半径滤波
 * @param {float} r 半径滤波半径
 * @param {bool} angleFilter 是否角度滤波
 * @param {float} startAngle 角度滤波开始角度
 * @param {float} endAngle 角度滤波终止角度
 * @param {bool} xyFilter 是否xy滤波
 * @param {float} startX x滤波x开始值
 * @param {float} endX x滤波x结束值
 * @param {float} startY y滤波y开始值
 * @param {float} endY y滤波y结束值
 * @param {float} angle 车子偏航
 * @return {*}
 * @author: bonbon
 */
void Filter::easy_filter(std::vector<float> &data,std::vector<float>&theta,
                        bool radiusFilter,float r,
                        bool angleFilter,float startAngle,float endAngle,
                        bool xyFilter,
                        float startX,float endX,
                        float startY,float endY,
                        float angle
                        )
{
    for(int i = 0;i < DATA_NUM;i++)
    {
        //半径滤波
        if(radiusFilter)
        {
            if(data[i] > r)
            {
                data[i] = 0;
            }
        }

        //角度滤波
        if(angleFilter)
        {
            if((theta[i]<startAngle) || (theta[i]>endAngle))
            {
                data[i] = 0;
            }
        }

        
        //xy滤波
        if(xyFilter)
        {
            float x=cos(theta[i])*data[i];
            float y =sin(theta[i])*data[i];
            coordinate_rotation(&x,&y,angle);

            //x滤波
            if((x<startX) || (x>endX))
            {
                data[i] =0;
            }
        
            //y滤波
            if((y<startY) || (y>endY))
            {
                data[i] =0;
            }
        }        
    }
}

/**
 * @brief: 卡尔曼滤波
 * @param {short} x 目前全场定位x
 * @param {short} y 目前全场定位y
 * @param {double} yaw 目前全场定位偏航
 * @param {short} *last_x 上一次全场定位x地址
 * @param {short} *last_y 上一次全场定位y地址
 * @return {*}
 * @author: bonbon
 */
void Filter::kalman_filter(std::vector<float> & nowData,
                    std::vector<float> & lastData,
                    std::vector<float>& theta,
                    short x,short y,double yaw,
                    short *last_x,short *last_y)
{
    
    for(int i=0;i<DATA_NUM;i++)
    {
        //计算车子距离上一次x，y位移
        float delta_x = (x - *last_x)/1000.0;
        float delta_y = (y - *last_y)/1000.0;

        //计算目前雷达获取的x，y坐标
        float x0 =(cos(theta[i])*lastData[i]);
        float y0 =(sin(theta[i])*lastData[i]);

        //坐标系旋转
        coordinate_rotation(&x0,&y0,yaw);

        //预测
        float s = sqrt((x0-delta_x)*(x0-delta_x)+(y0-delta_y)*(y0-delta_y));

        //计算方差
        float var =sqrt(last_var[i]*last_var[i]+predict_var*predict_var);

        //计算卡尔曼增益
        float kalman_gain = (var*var)/(var*var+measure_var*measure_var);
        
        //计算最优值 并输出
        nowData[i] = s + kalman_gain*(nowData[i]-s);

        //更新偏差
        last_var[i] = sqrt((1-kalman_gain)*var*var);    
    }

    lastData =nowData;
    *last_x =x;
    *last_y =y;
}


Filter::Filter()
{

}
Filter::~Filter()
{

}
