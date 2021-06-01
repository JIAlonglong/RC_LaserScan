#pragma once 
#include <iostream>
#include <math.h>
#include <vector>

#define PI acos(-1)

const int DATA_NUM = 1081; //数据个数
const float R = 0.315 / 2 ; // 圆柱半径
const float DISTANCE_MAX = 4.0 ; //车子离圆柱中心的最大距离
//const float DISTANCE_BETWEEN_CIRCLE =0.56;//两个圆柱之间的距离
const float ANGLE_INCREMENT = (3 * PI / 2) / 1080;  // 角度增量
const int LASER_DATA_NUM_MIN = static_cast<int>(2 * asin(R / DISTANCE_MAX) / ANGLE_INCREMENT);


void remove_outlier(std::vector<float>&data,std::vector<float>&theta,float radius,int k);
void splinter_continuous_part(std::vector<float>&data,std::vector<float>&start_index,std::vector<float>&end_index);
void get_circle(std::vector<float>&data,std::vector<float>&theta,float deviation);
void num_filter(std::vector<float>&data,const int MinNumber);
void get_circle_center(std::vector<float> & vec,std::vector<float> &xyR);
void index2center(std::vector<float> & start_end_index,std::vector<float>&data,std::vector<float>&xyR);
