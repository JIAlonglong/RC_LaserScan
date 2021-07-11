#include "filter.h"
//雷达数据逆时针-3pi/4到 3pi/4

/********************************************************
 * @brief 指定半径数据少于k个则认为是离散点
 * @param data 数据
 * @param theta 角度对应数据
 * @param radius 半径 
 * @param k 半径 
 * ******************************************************/

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



/********************************************************
 * @brief 分离出圆弧数据
 * @param data 数据 
 * @param theta 对应角度数据
 * @param deviation 离散点离正确值的差值阈值
 * ******************************************************/
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

/********************************************************
 * @brief 根据连续段数量滤波
 * @param data 数据
 * ******************************************************/
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

void Filter::kalman_filter(std::vector<float> & nowData,
                    std::vector<float> & lastData,
                    std::vector<float>& theta,
                    short x,short y)
{
    
    for(int i=0;i<DATA_NUM;i++)
    {
    
        //预测
        float s = sqrt(
                    pow(((x/1000.0)-(cos(theta[i])*lastData[i])),2) +
                    pow(((y/1000.0)-(sin(theta[i])*lastData[i])),2)
                    );

        //计算方差
        float var =sqrt(last_var[i]*last_var[i]+predict_var*predict_var);

        //计算最优值 并输出
        nowData[i] =((var*var)*nowData[i] + (measure_var*measure_var)*s)/
                    (var*var+measure_var*measure_var);

        //更新偏差
        last_var[i] =sqrt((1-var*var/(var*var+measure_var*measure_var)*var*var));
    }
}


Filter::Filter()
{

}
Filter::~Filter()
{

}
