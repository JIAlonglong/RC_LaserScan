# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
class Display:
    def __init__(self, monitor,col,row):
        self.col = col
        self.row = row
        self.fig, self.ax = plt.subplots(self.row, self.col, figsize=(15, 10))
        self.line =[]
        for row in range(self.row):
            for col in range(self.col):
                line, =(self.ax[row][col].plot(monitor.t, monitor.data[col +row*self.col]))
                self.line.append(line) 

    def update(self, data):
        for i in range(6):
            self.line[i].set_ydata(data[i])
    
    def dynamic_set_lim(self,data):

        offset = 500
        k = 100

        # left
        left_x = (data.left_x//k)*k
        left_y = (data.left_y//k)*k
        self.ax[0][0].set_ylim(left_x-offset,left_x+offset)
        self.ax[1][0].set_ylim(left_y-offset,left_y+offset)
            
        # middle
        middle_x = (data.middle_x//k)*k
        middle_y = (data.middle_y//k)*k
        self.ax[0][1].set_ylim(middle_x-offset,middle_x+offset)       
        self.ax[1][1].set_ylim(middle_y-offset,middle_y+offset)
            
        # right
        right_x = (data.right_x//k)*k
        right_y = (data.right_y//k)*k
        self.ax[0][2].set_ylim(right_x-offset,right_x+offset)
        self.ax[1][2].set_ylim(right_y-offset,right_y+offset)
