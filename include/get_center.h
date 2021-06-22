#pragma once 
#include <vector>
#include "filter.h"
#include <ros/ros.h>
void index2center(std::vector<float> & start_end_index,std::vector<float>&data,std::vector<float>&xyR);
void get_circle_center(std::vector<float> & vec,std::vector<float> &xyR);
void splinter_continuous_part(std::vector<float>&data,std::vector<float>&start_index,std::vector<float>&end_index);
void distributeData(std::vector<float> &start_index,
                    std::vector<float> &end_index,
                    std::vector<float> &left,
                    std::vector<float> &middle,
                    std::vector<float> &right,
                    std::vector<float> &theta,
                    float y);