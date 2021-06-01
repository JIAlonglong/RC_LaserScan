#pragma once 
#include <iostream>
#include <vector>
void coordinate_rotation(float x1,float y1,float yaw,float *x,float *y);
void change_to_TR_coordinate(float pot_laser_x,float pot_laser_y,short DR_x,short DR_y,int *result_x,int *result_y);

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



