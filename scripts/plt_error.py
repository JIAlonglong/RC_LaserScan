# !usr/bin/env python
# -*- coding:utf-8 -*-

'''
Description: 
Version: 1.0
Autor: bonbon
Date: 2021-08-01 19:58:22
LastEditors: bonbon
LastEditTime: 2021-08-02 14:08:32
'''

import rospy
from pots_coordinates.msg import error
from error import Error
from display import Display
import matplotlib.pyplot as plt
import matplotlib.animation as animation

col =3 #三列
row =1 #一行
error_data = Error(col,row) #Error类

display = Display(error_data,col,row)

def callback(msg):
    
    #更新画图数据    
    error_data.update_data(msg)
    
    #打印数据
    

if __name__ == '__main__':

    #初始化动图
    ani = animation.FuncAnimation(display.fig, display.update, error_data.update, interval=10)

    # ROS节点初始化
    rospy.init_node('error_subscriber', anonymous=True)

    # 创建一个Subscriber 订阅画图/coordinate_info
    rospy.Subscriber("/error_info", error, callback)
    
    #捕获图像被关闭的异常
    try:
        plt.show()
    except:
        exit(1)

    # 循环等待回调函数
    rospy.spin()
