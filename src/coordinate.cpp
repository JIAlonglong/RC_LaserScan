#include "coordinate.h"
#include <math.h>

/********************************************************
 * @brief 坐标系旋转 雷达相对坐标系旋转到世界坐标系
 * @param x1 雷达相对坐标系x坐标
 * @param y1 雷达相对坐标系y坐标
 * @param theta 偏航角 向左转为负 向右转为正
 * ******************************************************/
void coordinate_rotation(float *x,float *y,float theta)
{
    // float rho = sqrt((*x)*(*x)+(*y)*(*y));
    // float x1 = rho *cos(atan((*y)/(*x))+theta);
    // float y1 = rho *sin(atan((*y)/(*x))+theta);
    
    float x1 =(*x)*cos(theta)-(*y)*sin(theta);
    float y1 =(*y)*cos(theta)+(*x)*sin(theta);

    *x = x1;
    *y = y1;
}


/********************************************************
 * @brief 世界坐标系：相对雷达坐标->相对TR全场坐标原点
 * @param pot_laser_x 壶相对雷达坐标x坐标
 * @param pot_laser_y 壶相对雷达坐标y坐标
 * @param DR_x 
 * @param DR_y 
 * @param result_x 
 * @param result_y 
 * ******************************************************/
void change2worldCoordinate(float pot_laser_x,float pot_laser_y,
                            short DR_x,short DR_y,
                            float DrAction2DrLaser_x,float DrAction2DrLaser_y,
                            int *result_x,int *result_y)
{
    if((pot_laser_x ==0) && (pot_laser_y ==0))
    {
        *result_x=0;
        *result_y=0;
    }
    else
    {     
        /*
        场地图纸中
        雷达坐标系   上x 左y
        DR坐标系    右x 上y  DR原点  // 
        TR坐标系    上x 左y  TR原点
        最终以TR坐标系 将数据发给TR
        */

        // 以TR坐标系 TR起点和 DR起点的偏移
        const int DrStartPoint2TrStartPoint_x = -11000;
        const int DrStartPoint2TrStartPoint_y =  5000; 

        //米换毫米 float型乘完变成int型了
        // pot_laser_x *= 1000;
        // pot_laser_y *= 1000;
        
        *result_x = static_cast<int>(
                        pot_laser_x*1000 + //雷达x对应TR坐标系x
                        (DR_y) +     //DR(y)对应TR坐标系x
                        (DrStartPoint2TrStartPoint_x) + //DR和TR原点偏移量
                        (DrAction2DrLaser_x*1000.0)  //action和雷达偏移量
                        );
        *result_y = static_cast<int>(
                        pot_laser_y*1000 + //雷达y对应TR坐标系y
                        (-DR_x) +     //DR(-x)对应TR坐标系y
                        (DrStartPoint2TrStartPoint_y) + //DR和TR原点偏移量
                        (DrAction2DrLaser_y*1000.0) //action和雷达偏移量
                        );
    }
}



void Coordinate::prePublish()
{
    coordinate_msg.left_x = left_x;
    coordinate_msg.left_y = left_y;
    coordinate_msg.middle_x =middle_x;
    coordinate_msg.middle_y =middle_y;
    coordinate_msg.right_x =right_x;
    coordinate_msg.right_y =right_y;
}
bool Coordinate::check()
{
    if(
        (left_x!=0)   && 
        (left_y!=0)   && 
        (middle_x!=0) && 
        (middle_y!=0) && 
        (right_x!=0)  && 
        (right_y!=0) 
    )
    {   
        return true;
    }

    return false;
}


Coordinate::Coordinate()
{

}

Coordinate::~Coordinate()
{

}
