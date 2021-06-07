#include "TR_communication.h"

using namespace std;
using namespace boost::asio;
//串口相关对象
boost::asio::io_service tr_iosev;
boost::asio::serial_port tr_sp(tr_iosev, "/dev/ttyACM0");
boost::system::error_code tr_err;
/********************************************************
            串口发送接收相关常量、变量、共用体对象
********************************************************/
const unsigned char ender[2] = {0x0d, 0x0a};
const unsigned char header[2] = {0x55, 0xaa};

//发送当前位置x,y和偏航角yaw共用体（-32767 - +32768）
union sendData
{
	short d;
	unsigned char data[2];
}left_x,left_y,middle_x,middle_y,right_x,right_y;


/********************************************************
函数功能：串口参数初始化
入口参数：无
出口参数：
********************************************************/
void TR_SerialInit()
{
    tr_sp.set_option(serial_port::baud_rate(115200));
    tr_sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    tr_sp.set_option(serial_port::parity(serial_port::parity::none));
    tr_sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    tr_sp.set_option(serial_port::character_size(8));    
}

/********************************************************
函数功能：将对机器人的x,y,yaw打包发送给下位机
入口参数：x,y,yaw,控制位
出口参数：
********************************************************/
void TR_SerialWrite(int left_x_pos, int left_y_pos,int middle_x_pos,int middle_y_pos,int right_x_pos,int right_y_pos,unsigned char ctrlFlag)
{
    unsigned char buf[19] = {0};//
    int i, length = 0;

    left_x.d = left_x_pos;
    left_y.d = left_y_pos;
    middle_x.d = middle_x_pos;
    middle_y.d = middle_y_pos;
    right_x.d = right_x_pos;
    right_y.d = right_y_pos;

    // 设置消息头
    for(i = 0; i < 2; i++)
        buf[i] = header[i];             //buf[0]  buf[1]
    
    // 设置发送长度
    length = 13;
    buf[2] = length;                    //buf[2]
    
    //发送的数据
    for(i = 0; i < 2; i++)
    {
        buf[i + 3] = left_x.data[i];  //buf[3] buf[4]
        buf[i + 5] = left_y.data[i]; //buf[5] buf[6]
	    buf[i + 7] = middle_x.data[i]; //buf[7] buf[8]
        buf[i + 9] = middle_y.data[i]; //buf[9] buf[10]
        buf[i + 11] = right_x.data[i]; //buf[11] buf[12]
        buf[i + 13] = right_y.data[i]; //buf[13] buf[14]
    }
    // 预留控制指令
    buf[3 + length - 1] = ctrlFlag;       //buf[15]

    // 设置校验值、消息尾
    buf[3 + length] = TR_getCrc8(buf, 3 + length);//buf[16]
    buf[3 + length + 1] = ender[0];     //buf[17]
    buf[3 + length + 2] = ender[1];     //buf[18]

    // 通过串口下发数据
    boost::asio::write(tr_sp, boost::asio::buffer(buf));
}

/********************************************************
函数功能：获得8位循环冗余校验值
入口参数：数组地址、长度
出口参数：校验值
********************************************************/
unsigned char TR_getCrc8(unsigned char *ptr, unsigned short len)
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
