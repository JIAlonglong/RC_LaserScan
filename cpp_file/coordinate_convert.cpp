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
 * @param DR_DrDepartureZone_x DR相对DR全场坐标原点x坐标
 * @param DR_DrDepartureZone_y DR相对DR全场坐标原点y坐标
 * @param pot_TrDepartureZone_x 壶相对TR全场坐标原点x坐标
 * @param pot_TrDepartureZone_y 壶相对TR全场坐标原点y坐标
 * ******************************************************/
void change_to_TR_coordinate(float pot_laser_x,float pot_laser_y,float DR_DrDepartureZone_x,float DR_DrDepartureZone_y,int *pot_TrDepartureZone_x,int *pot_TrDepartureZone_y)
{
    // DR坐标系 正对比赛场地 右x 上y
    // TR坐标系 正对比赛场地 下x 右y

    //米换毫米
    pot_laser_x*=1000;
    pot_laser_y*=1000;

    //转换到TR坐标系原点 4800 6000
    pot_laser_x=4300-0+pot_laser_x;
    pot_laser_y=5500 -0 +pot_laser_y;
    
    // 转换到TR坐标系：y变成x x变成-y
    *pot_TrDepartureZone_x = static_cast<int>(pot_laser_x);
    *pot_TrDepartureZone_y = static_cast<int>(pot_laser_y);
}
