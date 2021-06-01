#include "save_data.h"
/********************************************************
 * @brief 根据txt文件更新调试次数
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
	std::ofstream out("./data/times.txt");
	if(out.fail())return -1;
	out<<times_int;
    	out.close();

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

std::string date_init(void)
{
    time_t now = time(0);
    tm *ltm = localtime(&now);
    std::string run_code_date =  std::to_string(1900 + ltm->tm_year) +"-" + \
                            std::to_string(1 + ltm->tm_mon) +"-"+ \
                            std::to_string(ltm->tm_mday) + "-" + \
                            std::to_string(ltm->tm_hour) +":"+\
                            std::to_string(ltm->tm_min)+":"+\
                            std::to_string(ltm->tm_sec);
    return run_code_date;
}