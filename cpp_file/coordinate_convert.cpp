#include "coordinate_convert.h"
#include <math.h>

/********************************************************
 * @brief 坐标系旋转 雷达相对坐标系旋转到世界坐标系
 * @param x1 雷达相对坐标系x坐标
 * @param y1 雷达相对坐标系y坐标
 * @param yaw 偏航角 往左为正 往右为负
 * @param x 世界坐标系x坐标
 * @param y 世界坐标系y坐标
 * ******************************************************/
void coordinate_rotation(float x1,float y1,float yaw,float *x,float *y)
{
    float rho =sqrt(x1*x1+y1*y1);
    *x =rho *cos(atan(y1/x1)+yaw);
    *y =rho *sin(atan(y1/x1)+yaw);
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
void change_to_TR_coordinate(float pot_laser_x,float pot_laser_y,short DR_x,short DR_y,int *result_x,int *result_y)
{
    /*
    场地图纸中
    雷达坐标系  上x 左y
    DR坐标系    左x 下y  DR原点 500 -4300
    TR坐标系    上x 左y  TR原点
    最终以TR坐标系 将数据发给TR
    */

    // 以TR坐标系 TR起点和 DR起点的偏移
    const int DrStartPoint2TrStartPoint_x = -11000;
    const int DrStartPoint2TrStartPoint_y =  5000; 

    // 以TR坐标系 雷达和全场定位的偏移 
    const int DrAction2DrLaser_x =0;
    const int DrAction2DrLaser_y =0; 

    //米换毫米
    pot_laser_x*=1000;
    pot_laser_y*=1000;
    
    *result_x = static_cast<int>(
                    pot_laser_x + //雷达x对应TR坐标系x
                    (-DR_y) +     //DR(-y)对应TR坐标系x
                    DrStartPoint2TrStartPoint_x + //DR和TR原点偏移量
                    DrAction2DrLaser_x  //action和雷达偏移量
                    );
    *result_y = static_cast<int>(
                    pot_laser_y + //雷达y对应TR坐标系y
                    (DR_x) +     //DR(x)对应TR坐标系y
                    DrStartPoint2TrStartPoint_y + //DR和TR原点偏移量
                    DrAction2DrLaser_y //action和雷达偏移量
                    );
}