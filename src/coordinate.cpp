#include "coordinate.h"
#include <math.h>

/**
 * @brief: 坐标系旋转
 * @param {float} *x x坐标地址
 * @param {float} *y y坐标地址
 * @param {float} theta 旋转角 向左转为负 向右转为正
 * @return {*}
 * @author: bonbon
 */
void coordinate_rotation(float *x,float *y,float theta)
{   
    float x1 =(*x)*cos(theta)-(*y)*sin(theta);
    float y1 =(*y)*cos(theta)+(*x)*sin(theta);

    *x = x1;
    *y = y1;
}

/**
 * @brief: 旋转到世界坐标系
 * @param {float} pot_laser_x 雷达坐标系x
 * @param {float} pot_laser_y 雷达坐标系y
 * @param {short} DR_x DR全场定位x
 * @param {short} DR_y DR全场定位y
 * @param {float} DrAction2DrLaser_x 全场定位距离雷达x
 * @param {float} DrAction2DrLaser_y 全场定位距离雷达y
 * @param {int} *result_x 输出x
 * @param {int} *result_y 输出y
 * @return {*}
 * @author: bonbon
 */
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
        场地图纸中 以DR面对最中间的壶的方向为上
        雷达坐标系   上x 左y
        DR坐标系    右x 上y  DR原点  // 
        TR坐标系    上x 左y  TR原点
        最终以TR坐标系 将数据发给TR
        */

        // 以TR坐标系 TR起点和 DR起点的偏移
        const int DrStartPoint2TrStartPoint_x = -11000;
        const int DrStartPoint2TrStartPoint_y =  5000; 
        
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



/**
 * @brief: 预发布 将发布的数据赋值
 * @param {*}
 * @return {*}
 * @author: bonbon
 */
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

/**
 * @brief: 
 * @param {short} value 输入值
 * @param {short} min_value 限幅最小值
 * @param {short} max_value 限幅最大值
 * @return {short} value 限幅后结果
 * @author: bonbon
 */
short limit(short value,short min_value,short max_value)
{
    if(value==0){return 0;}

    if(value > max_value){value = max_value;}
    if(value < min_value){value = min_value;}

    return value;
}

//写成重载就报错T~T
/**
 * @brief: 
 * @param {double} value 输入值
 * @param {double} min_value 限幅最小值
 * @param {double} max_value 限幅最大值
 * @return {short} value 限幅后结果 
 * @author: bonbon
 */
double dlimit(double value,double min_value,double max_value)
{
    
    if(value > max_value){value = max_value;}
    if(value < min_value){value = min_value;}

    return value;
}
