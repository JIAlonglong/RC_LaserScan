#pragma once 
#include <iostream>
#include <math.h>
#include <vector>
#include "get_center.h"
#define PI acos(-1)

const int DATA_NUM = 1081; //数据个数
const float R = 0.315 / 2 ; // 圆柱半径
const float ANGLE_INCREMENT = (3 * PI / 2) / 1080;  // 角度增量
//const float DISTANCE_BETWEEN_CIRCLE =0.56;//两个圆柱之间的距离

class Filter
{
    public:
        
    
        Filter();
        ~Filter();

        void remove_outlier(std::vector<float>&data,std::vector<float>&theta,float radius,int k);
        void get_circle(std::vector<float>&data,std::vector<float>&theta,float deviation);
        void num_filter(std::vector<float>&data,const int MinNumber);
        void median_filter(std::vector<float> & nowData,std::vector<float> & lastData);
        void easy_filter(std::vector<float> & data,std::vector<float>&theta,
                        bool radiusFilter,float r,
                        bool angleFilter,float startAngle,float endAngle,
                        bool xFilter,float startX,float endx,
                        bool yFilter,float startY,float endY);
    private:
};
