#include "save_data.h"
/********************************************************
 * @brief 根据txt文件更新调试次数
 * @return 目前调试次数 
 * ******************************************************/
int update_times(void)
{
    //打开文件读取调试次数
	std::ifstream in("./data/times.txt");
	if(in.fail())return -1;
	std::string times_str;
	in>>times_str;
	int times_int =atoi(times_str.c_str());
	in.close();

    //自增一后写入文件为下一次调试做准备
	times_int ++;
	std::ofstream out1("./data/times.txt");
	if(out1.fail())return -1;
	out1<<times_int;
    out1.close();	
	
	return times_int;
}

void save_data(std::string path,int num,std::vector<float> &raw_data,std::vector<float> &filter_data,int left_x,int left_y,int middle_x,int middle_y,int right_x,int right_y)
{
	std::ofstream out(path,std::ios::app);
    out<<"********************原始数据********************\n";
    for(int i=0;i<num;i++)
    {
        out<<raw_data[i];
        out<<",";
    }
    out<<"\n";
    out<<"********************滤波数据********************\n";
    for(int i=0;i<num;i++)
    {
        out<<filter_data[i];
        out<<",";
    }
    out<<"\n";
    out<<"********************壶坐标********************\n";
    out<<"left:"<<left_x<<","<<left_y<<";"<<
                 "middle:"<<middle_x<<","<<middle_y<<";"<<
                 "right:"<<right_x<<","<<right_y<<std::endl;
    out<<"\n";
    out.close();
}