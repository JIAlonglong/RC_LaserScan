#include "DR_communication.h"

using namespace std;
using namespace boost::asio;
//串口相关对象
boost::asio::io_service dr_iosev;
boost::asio::serial_port dr_sp(dr_iosev, "/dev/ttyUSB0");
boost::system::error_code dr_err;
/********************************************************
            串口发送接收相关常量、变量、共用体对象
********************************************************/
const unsigned char ender[2] = {0x0d, 0x0a};
const unsigned char header[2] = {0x55, 0xaa};

//发送共用体
union sendData
{
	short d;
	unsigned char data[2];
}x_destination,y_destination;

//接收共用体（-32767 - +32768）
union receiveData
{
	short d;
	unsigned char data[2];
}dr_x,dr_y,dr_yaw;

/********************************************************
函数功能：串口参数初始化
入口参数：无
出口参数：
********************************************************/
void DR_SerialInit()
{
    dr_sp.set_option(serial_port::baud_rate(115200));
    dr_sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    dr_sp.set_option(serial_port::parity(serial_port::parity::none));
    dr_sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    dr_sp.set_option(serial_port::character_size(8));    
}

/********************************************************
函数功能：将x目的距离，y目的距离发送
入口参数：x目的距离，y目的距离
出口参数：
********************************************************/
void DR_SerialWrite(short x, short y,unsigned char ctrlFlag)
{
    unsigned char buf[11] = {0};//
    int i, length = 0;

    x_destination.d  = x;
    y_destination.d = y;

    // 设置消息头
    for(i = 0; i < 2; i++)
        buf[i] = header[i];             //buf[0]  buf[1]
    
    // 设置机器人左右轮速度
    length = 5;
    buf[2] = length;                    //buf[2]
    for(i = 0; i < 2; i++)
    {
        buf[i + 3] = x_destination.data[i];  //buf[3] buf[4]
        buf[i + 5] = y_destination.data[i]; //buf[5] buf[6]
    }
    // 预留控制指令
    buf[3 + length - 1] = ctrlFlag;       //buf[7]

    // 设置校验值、消息尾
    buf[3 + length] = DR_getCrc8(buf, 3 + length);//buf[8]
    buf[3 + length + 1] = ender[0];     //buf[9]
    buf[3 + length + 2] = ender[1];     //buf[10]

    // 通过串口下发数据
    boost::asio::write(dr_sp, boost::asio::buffer(buf));
}
/********************************************************
函数功能：从下位机读取dr全场x，y，yaw
入口参数：机器人x、y、yaw，预留控制位
出口参数：bool
********************************************************/
bool DR_SerialRead(short &x,short &y,short &yaw,unsigned char &ctrlFlag)
{
    char i, length = 0;
    unsigned char checkSum;
    unsigned char buf[150]={0};
    //=========================================================
    //此段代码可以读数据的结尾，进而来进行读取数据的头部
    try
    {
        boost::asio::streambuf response;
        boost::asio::read_until(dr_sp, response, "\r\n",dr_err);   
        copy(istream_iterator<unsigned char>(istream(&response)>>noskipws),
        istream_iterator<unsigned char>(),
        buf); 
    }  
    catch(boost::system::system_error &err)
    {
        ROS_INFO("read_until error");
    } 
    //=========================================================        

    // 检查信息头
    if (buf[0]!= header[0] || buf[1] != header[1])   //buf[0] buf[1]
    {
        ROS_ERROR("Received message header error!");
        return false;
    }
    // 数据长度
    length = buf[2];                                 //buf[2]

    // 检查信息校验值
    checkSum = DR_getCrc8(buf, 3 + length);             //buf[10] 计算得出
    if (checkSum != buf[3 + length])                 //buf[10] 串口接收
    {
        ROS_ERROR("Received data check sum error!");
        return false;
    }    

    // 读取速度值
    for(i = 0; i < 2; i++)
    {
        dr_x.data[i]  = buf[i + 3]; //buf[3] buf[4]
        dr_y.data[i] = buf[i + 5]; //buf[5] buf[6]
        dr_yaw.data[i]    = buf[i + 7]; //buf[7] buf[8]
    }

    // 读取控制标志位
    ctrlFlag = buf[9];
    
    x  =dr_x.d;
    y =dr_y.d;
    yaw   =dr_yaw.d;

    return true;
}
/********************************************************
函数功能：获得8位循环冗余校验值
入口参数：数组地址、长度
出口参数：校验值
********************************************************/
unsigned char DR_getCrc8(unsigned char *ptr, unsigned short len)
{
    unsigned char crc;
    unsigned char i;
    crc = 0;
    while(len--)
    {
        crc ^= *ptr++;
        for(i = 0; i < 8; i++)
        {
            if(crc&0x01)
                crc=(crc>>1)^0x8C;
            else 
                crc >>= 1;
        }
    }
    return crc;
}
