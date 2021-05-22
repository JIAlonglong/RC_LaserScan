# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
class Display:
    def __init__(self, monitor,col,row):
        self.col = col
        self.row = row
        self.fig, self._ax = plt.subplots(self.row, self.col, figsize=(10, 10))
        self.line =[]
        for row in range(self.row):
            for col in range(self.col):
                line, =(self._ax[row][col].plot(monitor.t, monitor.data[col +row*self.col]))
                self.line.append(line) 
        # left
        self._ax[0][0].set_ylim(4500,5500)
        self._ax[1][0].set_ylim(7700,8300)
        
        #middle
        self._ax[0][1].set_ylim(5000,5500)       
        self._ax[1][1].set_ylim(5200,5700)
        
        #right
        self._ax[0][2].set_ylim(5000,5500)
        self._ax[1][2].set_ylim(2500,3500)


    def update(self, data):
        for i in range(6):
            self.line[i].set_ydata(data[i])
