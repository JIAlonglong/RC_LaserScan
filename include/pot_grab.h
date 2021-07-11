#pragma once
#include "coordinate.h"
#include <ros/ros.h>
/*
 * @Description: 
 * @Version: 1.0
 * @Autor: bonbon
 * @Date: 2021-07-09 22:02:37
 * @LastEditors: bonbon
 * @LastEditTime: 2021-07-10 00:54:43
 */

class DistanceIndex
{
    public:
        double distance;
        unsigned char index;

        void init(std::vector<float> &xyR,unsigned char num);
};

class PotGrab
{
    public:
        double alpha;
        int x;
        int y;
        unsigned char index;

    void preSend(unsigned char num,Coordinate coordinate);


};
