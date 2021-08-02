/*
 * @Description: 
 * @Version: 1.0
 * @Autor: bonbon
 * @Date: 2021-08-01 19:58:22
 * @LastEditors: bonbon
 * @LastEditTime: 2021-08-02 13:56:35
 */

#pragma once
#include "coordinate.h"
#include "filter.h"
#include <ros/ros.h>


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
