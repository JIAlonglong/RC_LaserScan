#include "save.h"

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