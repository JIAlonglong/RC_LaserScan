#pragma once 
#include <ros/ros.h>
#include <ros/time.h>
#include <boost/asio.hpp>


extern void TR_SerialInit();
extern void TR_SerialWrite(int left_x_pos, int left_y_pos,int middle_x_pos,int middle_y_pos,int right_x_pos,int right_y_pos,unsigned char ctrlFlag);
unsigned char TR_getCrc8(unsigned char *ptr, unsigned short len);

