#pragma once 
#include <iostream>
#include <fstream>
#include <vector>
int update_times(void);
void save_data(std::string path,int num,std::vector<float> &raw_data,std::vector<float> &filter_data,int left_x,int left_y,int middle_x,int middle_y,int right_x,int right_y);