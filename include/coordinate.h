/*
 * @Description: 
 * @Version: 1.0
 * @Autor: bonbon
 * @Date: 2021-08-01 19:58:22
 * @LastEditors: bonbon
 * @LastEditTime: 2021-08-02 13:53:03
 */

#pragma once 
#include <iostream>
#include <vector>
#include "rc_laserscan/coordinate.h"

class Coordinate
{
    public:
        int left_x=0;
        int left_y=0;
        int middle_x=0;
        int middle_y=0;
        int right_x=0;
        int right_y=0;

        std::vector<float> left_xyR{std::vector<float>(3)};
        std::vector<float> middle_xyR{std::vector<float>(3)};
        std::vector<float> right_xyR{std::vector<float>(3)};
   
        rc_laserscan::coordinate coordinate_msg;//坐标消息类型类

        Coordinate();
        ~Coordinate();


        void prePublish();
        bool check();

    private:
  
};

void coordinate_rotation(float *x,float *y,float theta);
void change2worldCoordinate(float pot_laser_x,float pot_laser_y,
                            short DR_x,short DR_y,
                            float DrAction2DrLaser_x,float DrAction2DrLaser_y,
                            int *result_x,int *result_y);
short limit(short value,short min_value,short max_value);
double dlimit(double value,double min_value,double max_value);




