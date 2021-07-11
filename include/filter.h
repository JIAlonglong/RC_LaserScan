/*
 * @Description: 
 * @Version: 1.0
 * @Autor: bonbon
 * @Date: 2021-07-09 02:35:14
 * @LastEditors: bonbon
 * @LastEditTime: 2021-07-09 09:04:36
 */
#pragma once 
#include <iostream>
#include <math.h>
#include <vector>
#include "get_center.h"
#include "coordinate.h"
#define PI acos(-1)

const int DATA_NUM = 1081; //数据个数
const float R = 0.315 / 2 ; // 圆柱半径
const float ANGLE_INCREMENT = (3 * PI / 2) / 1080;  // 角度增量
//const float DISTANCE_BETWEEN_CIRCLE =0.56;//两个圆柱之间的距离

class Filter
{
    public:
        
        //记录卡尔曼滤波上一次的偏差 不要为0
        std::vector<float> last_var{std::vector<float>(DATA_NUM,1.0)};
        float predict_var =0.0;//预测偏差 影响收敛速度 调试给出
        float measure_var =0.0;//雷达测量的方差 采样获得

        Filter();
        ~Filter();

        void remove_outlier(std::vector<float>&data,std::vector<float>&theta,float radius,int k);
        void get_circle(std::vector<float>&data,std::vector<float>&theta,float deviation);
        void num_filter(std::vector<float>&data,const int MinNumber);
        void median_filter(std::vector<float> & nowData,std::vector<float> & lastData);
        void easy_filter(std::vector<float> &data,std::vector<float>&theta,
                        bool radiusFilter,float r,
                        bool angleFilter,float startAngle,float endAngle,
                        bool xyFilter,
                        float startX,float endX,
                        float startY,float endY,
                        float angle
                        );
        void kalman_filter(std::vector<float> & nowData,
                           std::vector<float> & lastData,
                           std::vector<float>&theta,
                           short x,short y);
    private:
};
