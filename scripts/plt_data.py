#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
from pots_coordinates.msg import coordinate
from data import Data
from display import Display
import matplotlib.pyplot as plt
import matplotlib.animation as animation

col =3 #三列
row =2 #两行
data = Data(col,row) #数据类

display = Display(data,col,row)

def callback(msg):

    #更新画图数据    
    data.update_data(msg)
    display.dynamic_set_lim(msg)
    
    #打印数据
    rospy.loginfo("left:%4d,%4d; middle:%4d,%4d; right:%4d,%4d",
                  msg.left_x, msg.left_y, msg.middle_x, msg.middle_y, msg.right_x, msg.right_y)


if __name__ == '__main__':

    #初始化动图
    ani = animation.FuncAnimation(display.fig, display.update, data.update, interval=10)

    # ROS节点初始化
    rospy.init_node('coordinate_subscriber', anonymous=True)

    # 创建一个Subscriber 订阅画图/coordinate_info
    rospy.Subscriber("/coordinate_info", coordinate, callback)
    
    #捕获图像被关闭的异常
    try:
        plt.show()
    except:
        exit(1)

    # 循环等待回调函数
    rospy.spin()
