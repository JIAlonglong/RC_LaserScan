#ifndef MBOT_LINUX_SERIAL_H
#define MBOT_LINUX_SERIAL_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>

extern void DR_SerialInit();
extern void DR_SerialWrite(short x, short y,unsigned char ctrlFlag);
extern bool DR_SerialRead(short &x,short &y,short &yaw,unsigned char &ctrlFlag);
unsigned char DR_getCrc8(unsigned char *ptr, unsigned short len);

#endif
