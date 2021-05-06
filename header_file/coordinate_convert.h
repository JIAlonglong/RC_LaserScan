#pragma once 
#include <iostream>
#include <vector>
void coordinate_rotation(float x1,float y1,float yaw,float *x,float *y);
void change_to_TR_coordinate(float pot_laser_x,float pot_laser_y,float DR_DrDepartureZone_x,float DR_DrDepartureZone_y,int *pot_TrDepartureZone_x,int *pot_TrDepartureZone_y);

/*****************
 *         y
 *            ^
 *             |                                                      以DR启动区中心为全场坐标原点
 *             |                                        雷达正向对着三个壶 从左往右以此是1、2、3壶     
 *             |                                                                      
 *             |____________> x
 *             |□
 *             |
 *             |
 *             |
 *
 *  **/

