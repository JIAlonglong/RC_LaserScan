/*
 * @Description: 
 * @Version: 1.0
 * @Autor: bonbon
 * @Date: 2021-08-01 19:58:22
 * @LastEditors: bonbon
 * @LastEditTime: 2021-08-02 13:53:13
 */

#pragma once 
#include <ros/ros.h>
#include <ros/time.h>
#include <boost/asio.hpp>


extern void DR_SerialInit();
extern void DR_SerialWrite(int x_pos, int y_pos,int theta,unsigned char ctrlFlag);
extern bool DR_SerialRead(short &x,short &y,short &yaw,unsigned char &ctrlFlag);
unsigned char DR_getCrc8(unsigned char *ptr, unsigned short len);


