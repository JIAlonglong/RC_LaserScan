#pragma once

//Action数据 DR坐标系 下x右y
class Action
{
    public:
        short x;
        short y;
        short yaw;
        unsigned char flag;

        Action();
        ~Action();

    private:
        
};