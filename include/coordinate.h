#pragma once 
#include <iostream>
#include <vector>
#include "pots_coordinates/coordinate.h"
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
   
        pots_coordinates::coordinate coordinate_msg;//坐标消息类型类

        Coordinate();
        ~Coordinate();


        void prePublish();
        bool check();

    private:
  
};

void coordinate_rotation(float *x,float *y,float yaw);
void change2worldCoordinate(float pot_laser_x,float pot_laser_y,
                            short DR_x,short DR_y,
                            float DrAction2DrLaser_x,float DrAction2DrLaser_y,
                            int *result_x,int *result_y);

/*****************
 *         y
 *            ^
 *             |                         以DR启动区中心为全场坐标原点
 *             |                    雷达正向对着三个壶是x 从左往右以此是1、2、3壶     
 *             |                                                                      
 *             |____________> x
 *             |□
 *             |
 *             |
 *             |
 *
 *  **/



