#include "get_var.h"

/**
 * @brief: 获取雷达测量方差 用于卡尔曼滤波
 * @param scan 雷达数据
 * @return {*}
 * @author: bonbon
 */
void get_var(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  
    now_num++;

    if(now_num<iter_num)
    {
        std::vector<float> data(scan->ranges);//目前的数据
        for(int i=0;i<DATA_NUM;i++)
        {
            datavec_Vec[i].push_back(data[i]);
        }
    }
    else
    {
        float sum=0;
        float sum2=0;
        for(int i=0;i<DATA_NUM;i++)
        {
            for(int j=0;j< datavec_Vec[i].size();j++)
            {
                sum += datavec_Vec[i][j];
                sum2 += datavec_Vec[i][j]*datavec_Vec[i][j];
            }
        }

        float EX =sum/datavec_Vec[0].size();
        float EX2 =sum2/datavec_Vec[0].size();

        float var = EX2 -EX*EX;

        printf("var:%f",var); 
        fflush(stdout);

        std::ofstream out(path,std::ios::out);
        out<<"var: "<<var;
        
        exit(0);
        
    }

}