#include "get_center.h"
/********************************************************
 * @brief 根据起始下标 确定壶心坐标
 * @param start_end_index 起始下标
 * @param data 数据 
 * @param xyR 拟合输出的圆心坐标和半径
 * ******************************************************/
void index2center(std::vector<float> & start_end_index,std::vector<float>&data,std::vector<float>&xyR)
{
    if((start_end_index[0]==0) && (start_end_index[1]==0))
    {
        xyR[0]=0;
        xyR[1]=0;
        xyR[2]=0;
    }
    else
    {   
        if (start_end_index.size() == 2)
        {
            std::vector<float> circle(DATA_NUM);
            for(int i = 0;i < DATA_NUM;i++)
            {
                if(i>=start_end_index[0] && i<start_end_index[1])
                {
                    circle[i] =data[i];
                }
                else
                {
                    circle[i] =0;
                }
            }       
            get_circle_center(circle,xyR);
        }
    }
    
    /*
    else if(start_end_index.size() ==4)
    {
        std::vector<float> circle1(DATA_NUM),circle2(DATA_NUM);
        for(int i =0;i<DATA_NUM;i++)
        {
            if(i>=start_end_index[0] && i<start_end_index[1])
            {
                circle1[i] =data[i];
            }
            else
            {
                circle1[i] =0;
            }
        }
        for(int i =0;i<DATA_NUM;i++)
        {
            if(i>=start_end_index[2] && i<start_end_index[3])
            {
                circle2[i] =data[i];
            }
            else
            {
                circle2[i] =0;
            }
        }
        std::vector<float> xyR1(3),xyR2(3);
        get_circle_center(circle1,xyR1);
        get_circle_center(circle2,xyR2);
        if(xyR1[0]<xyR2[0])
        {
            xyR[0]=xyR1[0];
            xyR[1]=xyR1[1];
            xyR[2]=xyR1[2];
        }
        else
        {
            xyR[0]=xyR2[0];
            xyR[1]=xyR2[1];
            xyR[2]=xyR2[2];
        }       
    }
    */
   
}


/********************************************************
 * @brief 最小二乘法拟合圆心
 * @param vec 数据 
 * @param xyR 拟合输出的圆心坐标和半径
 * ******************************************************/
void get_circle_center(std::vector<float> & vec,std::vector<float> &xyR)
{
	int num = 0; //用于计算被过滤掉的数据个数

	float center_x =0, center_y =0, radius = 0; //定义圆心的坐标及半径 

	//定义所需要用到的计算值
    float sum_x = 0, sum_y = 0, sum_x2 = 0, sum_y2 = 0;
	float sum_x3 = 0, sum_y3 =0, sum_xy = 0, sum_x1y2 =0, sum_x2y1 = 0; 
    
	// 通过ranges中数据的个数进行雷达数据的遍历
    for (int i = 0; i < vec.size(); i++)
    {
		float range1 = vec[i];     // 定义遍历到的首个在范围内的点
		
		//当该点不在所需范围时，过滤掉点		
		if(range1>10 || range1<1e-4) 
		{
			num++;
			continue;
		}
		
		float angle1 = -3*PI/4 + ANGLE_INCREMENT * i; //角度的计算，用于后面计算x和y

		float x = range1 * cos(angle1);  //计算x和y
		float y = range1 * sin(angle1);
		
		float x2 = x*x;
		float y2 = y*y;
		
		sum_x += x;
        sum_y += y;
        sum_x2 += x2;
        sum_y2 += y2;
        sum_x3 += x2 * x;
        sum_y3 += y2 * y;
        sum_xy += x * y;
        sum_x1y2 += x * y2;
        sum_x2y1 += x2 * y;
    }
	float C, D, E, G, H;
	float a, b, c;

	int N = vec.size() - num; //得到数据个数

 	C = N * sum_x2 - sum_x * sum_x;
 	D = N * sum_xy - sum_x * sum_y;
 	E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
 	G = N * sum_y2 - sum_y * sum_y;
	H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
	a = (H * D - E * G) / (C * G - D * D);
	b = (H * C - E * D) / (D * D - G * C);
	c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

 	xyR[0] = a / (-2);
 	xyR[1] = b / (-2);
 	xyR[2] = sqrt(a * a + b * b - 4 * c) / 2;
}



/********************************************************
 * @brief 分离出连续段
 * @param data 数据
 * @param start_index 连续段开始下标vector
 * @param end_index 连续段结束下标vector
 * ******************************************************/
void splinter_continuous_part(std::vector<float>&data,std::vector<float>&start_index,std::vector<float>&end_index)
{  
    bool start_sign = true;//开始的标志
    for(int i = 0;i < DATA_NUM;i++)
    {
        
        if(data[i] == 0)
        {
            if (!start_sign)
            {
                //没有continue 唯一会导致start_sign = true
                //即end后又一轮重新开始
                end_index.push_back(i);
            }
            else
            {
                continue;
            }            
        }
        else
        {
            if(start_sign)
            {
                start_index.push_back(i);
                start_sign = false;
                continue;
            }
            else
            {
                continue;
            }            
        }
        start_sign = true;        
    }
    
    // 避免最后连续段刚好在数据段末尾
    if(start_index.size()!=end_index.size())
    {
        end_index.push_back(DATA_NUM - 1);
    }
}


void distributeData(std::vector<float> &start_index,
                    std::vector<float> &end_index,
                    std::vector<float> &left,
                    std::vector<float> &middle,
                    std::vector<float> &right,
                    std::vector<float> &theta,
                    short x)
{
		//TR坐标系 上x 左y
        if(start_index.size() ==3)
        {
            right[0]=start_index[0];
            right[1]=end_index[0];
            middle[0]=start_index[1];
            middle[1]=end_index[1];
            left[0]=start_index[2];
            left[1]=end_index[2];
        }
		
        else if(start_index.size() ==2)
        {
            if(x>-500)
            {
                right[0]=start_index[0];
                right[1]=end_index[0];
                middle[0]=start_index[1];
                middle[1]=end_index[1];
                left[0]=0;
                left[1]=0;
            }
            else
            {
                right[0]=0;
                right[1]=0;
                middle[0]=start_index[0];
                middle[1]=end_index[0];
                left[0]=start_index[1];
                left[1]=end_index[1];
            }    
        }
		
        else if(start_index.size() ==1)
        {
            if(x<-3000)
            {
                right[0]=0;
                right[1]=0;
                middle[0]=0;
                middle[1]=0;
                left[0]=start_index[0];
                left[1]=end_index[0];         
            }
            else if(x<-500)
            {
                if(theta[start_index[0]]<0)
                {
                    right[0]=0;
                    right[1]=0;
                    middle[0]=start_index[0];
                    middle[1]=end_index[0];
                    left[0]=0;
                    left[1]=0;
                }
                else
                {
                    right[0]=0;
                    right[1]=0;
                    middle[0]=0;
                    middle[1]=0;
                    left[0]=start_index[0];
                    left[1]=end_index[0];
                }
            }
            else if(x<2000)
            {
                if(theta[start_index[0]]<0)
                {
                    right[0]=start_index[0];
                    right[1]=end_index[0];
                    middle[0]=0;
                    middle[1]=0;
                    left[0]=0;
                    left[1]=0;
                }
                else
                {
                    right[0]=0;
                    right[1]=0;
                    middle[0]=start_index[0];
                    middle[1]=end_index[0];
                    left[0]=0;
                    left[1]=0;
                }
            }
            else
            {
                right[0]=start_index[0];
                right[1]=end_index[0]; 
                middle[0]=0;
                middle[1]=0;
                left[0]=0;
                left[1]=0;
            }
        }
        else
        {
            ROS_ERROR("distributeData error");
        }
}
