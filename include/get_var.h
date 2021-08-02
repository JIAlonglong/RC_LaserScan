/*
 * @Description: 
 * @Version: 1.0
 * @Autor: bonbon
 * @Date: 2021-08-01 19:58:22
 * @LastEditors: bonbon
 * @LastEditTime: 2021-08-02 13:56:01
 */

#pragma once
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "filter.h"
#include <fstream>

long int iter_num =1e10;//迭代次数
long int now_num =0;//目前次数
std::vector<std::vector<float>> datavec_Vec;//储存数据的vector
std::string path ="./src/pots_coordinates/config/var.yaml";//保存路径

void get_var(const sensor_msgs::LaserScan::ConstPtr& scan);


