# !usr/bin/env python
# -*- coding:utf-8 -*-

'''
 Description: 
 Version: 1.0
 Autor: bonbon
 Date: 2021-06-20 19:31:27
LastEditors: bonbon
LastEditTime: 2021-08-02 14:07:39
'''

import numpy as np
class Error:
    def __init__(self,col,row):
        self.t = np.linspace(0, 100, 100)# 时间轴
        self.data = []#数据
        self.data_pic_num = col * row
        self.temp_data =[0,0,0]#暂存数据列表
        for _ in range(self.data_pic_num):
            self.data.append(self.t*0)#初始化数据

    #由于更新数据要在Subscribe回调函数中进行 所以需要一个函数和暂存数据列表
    def update_data(self, msg):
        self.temp_data[0] = msg.calib_x_error
        self.temp_data[1] = msg.calib_y_error
        self.temp_data[2] = msg.calib_yaw_error

    #用于给画动图的函数
    def update(self):
        # roll
        for i in range(self.data_pic_num):
            self.data[i][:] = np.roll(self.data[i],-1)

        self.data[0][-1] = self.temp_data[0]            
        self.data[1][-1] = self.temp_data[1]  
        self.data[2][-1] = self.temp_data[2]        
  
        
        yield tuple(self.data)

