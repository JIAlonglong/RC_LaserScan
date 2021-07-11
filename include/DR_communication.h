#pragma once 
#include <ros/ros.h>
#include <ros/time.h>
#include <boost/asio.hpp>


extern void DR_SerialInit();
extern void DR_SerialWrite(int x_pos, int y_pos,int theta,unsigned char ctrlFlag);
extern bool DR_SerialRead(short &x,short &y,short &yaw,unsigned char &ctrlFlag);
unsigned char DR_getCrc8(unsigned char *ptr, unsigned short len);


