#include "kalman.h"

Kalman::Kalman(){
	      A = 1;  //标量卡尔曼
        H = 1;  //
        P = 10;  //后验状态估计值误差的方差的初始值（不要为0问题不大）
        Q =kalman_Q;
        R =kalman_R;
        filterValue = 0;// 测量的初始值
}

Kalman::~Kalman() {

}

float Kalman::KalmanFilter(float lastMeasurement)
{
        //预测下一时刻的值
        //x的先验估计由上一个时间点的后验估计值和输入信息给出，此处需要根据基站>高度做一个修改
        float predictValue = A* filterValue;

        //求协方差
        P = A*A*P + Q;  //计算先验均方差 p(n|n-1)=A^2*p(n-1|n-1)+q
        float preValue = filterValue;  //记录上次实际坐标的值

        //计算kalman增益
          //Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R)
        kalmanGain = P*H / (P*H*H + R);

        //修正结果，即计算滤波值
          //利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))
        filterValue = predictValue + (lastMeasurement - predictValue)*kalmanGain;
        //更新后验估计
        P = (1 - kalmanGain*H)*P;//计算后验均方差  P[n|n]=(1-K[n]*H)*P[n|n-1]
}



