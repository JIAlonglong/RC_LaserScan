/*
 * @Description: 
 * @Version: 1.0
 * @Autor: bonbon
 * @Date: 2021-08-01 19:58:22
 * @LastEditors: bonbon
 * @LastEditTime: 2021-08-02 13:42:18
 */

#pragma once

//全场定位数据
class Action
{ 
    public:
        short x;
        short y;
        short yaw;
        unsigned char flag;

        short last_x;
        short last_y;

    private:
        
};