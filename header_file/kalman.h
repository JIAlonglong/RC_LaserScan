#pragma once

class Kalman
{
public:
        float filterValue;  //k-1时刻的滤波值，即是k-1时刻的值
        float kalmanGain;   //   Kalamn增益
        float A;   // x(n)=A*x(n-1)+u(n),u(n)~N(0,Q)
        float H;   // z(n)=H*x(n)+w(n),w(n)~N(0,R)
        float Q;   //预测过程噪声偏差的方差  影响收敛速率
        float R;   //测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
        float P;   //估计误差协方差
	
        float KalmanFilter(float lastMeasurement,float Q,float R);
        Kalman();
	~Kalman();		
};


