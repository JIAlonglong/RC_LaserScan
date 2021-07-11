#include "pot_grab.h"
void DistanceIndex::init(std::vector<float> &xyR,unsigned char num)
{
    //只用于比较就不用sqrt
    distance =xyR[0]*xyR[0]+xyR[1]*xyR[1];
    index =num;
}

void PotGrab::preSend(unsigned char num,Coordinate coordinate)
{
    //左负角
    switch(num)
    {      
        int x0,y0;
        case 0x01:
        {
            x0=-5500;
            y0=3000;

            //注意转到DR坐标系 y=x  x=-y
            y = coordinate.right_xyR[0]*1000.0;
            x = -coordinate.right_xyR[1]*1000.0;
            index = num;

            if(x0==coordinate.right_x)
            {
                alpha=0;
            }
            else
            {
                alpha =-atan(
                    (y0-coordinate.right_y)*1.0/
                    (x0-coordinate.right_x)
                    );
            }
            // ROS_INFO("%d %d %d %d %f",coordinate.right_x,x0,coordinate.right_y,y0,alpha);
            break;
        }
        case 0x02:
        {
            x0= -5500;
            y0= 5500;

            //注意转到DR坐标系 y=x  x=-y
            y = coordinate.middle_xyR[0]*1000.0;
            x = -coordinate.middle_xyR[1]*1000.0;
            index = num;

            
            if(x0==coordinate.middle_x)
            {
                alpha=0;
            }
            else
            {
                alpha =-atan(
                    (y0-coordinate.middle_y)*1.0/
                    (x0-coordinate.middle_x)
                    );
            }
            // ROS_INFO("%d %d %d %d %f",coordinate.middle_x,x0,coordinate.middle_y,y0,alpha);
            break;
        }
        case 0x03:
        {
            x0= -5500;
            y0= 8000;

            //注意转到DR坐标系 y=x  x=-y
            y = coordinate.left_xyR[0]*1000.0;
            x = -coordinate.left_xyR[1]*1000.0;
            index = num;

            if(x0==coordinate.left_x)
            {
                alpha=0;
            }
            else
            {
                alpha =-atan(
                    (y0-coordinate.left_y)*1.0/
                    (x0-coordinate.left_x)
                    );
            }
            // ROS_INFO("%d %d %d %d %f",coordinate.left_x,x0,coordinate.left_y,y0,alpha);
            break;
        }
        default:
        {
            x=0;
            y=0;
            alpha=0;
            index=0x00;
            ROS_ERROR("pot's index to DR error！");
        }
    }
}