### 概述

ROBOCON 2021投壶行殇，重点就是外区TR投壶得分，而内区的二、三型壶是会被转动的，所以需要实时获取转动壶的位置。该工程是基于ROS、部署在DR机器人，使用雷达获得转动壶的全场坐标，无线串口通信给TR用于瞄准。另外获取机器人相对于最近的壶的x，y以及壶转过的角度，用于DR快速对准并抓取壶。因为内区DR是运动的，该工程加入了雷达运动补偿以及卡尔曼滤波，保证了动态响应性能。基于ROS，该工程每次运行都会将雷达数据以及全场定位的值保存成bag文件，也编写了对应的读取、处理代码，前期可用于验证算法，后期便于找出问题。另外可视化计算出来的坐标随着时间推移的变化，有利于直观观察数据变化。



### 硬件支持

需要一台上位机，支持ROS，网口接雷达，无线串口和有线串口都接USB。



### 目录结构描述

```text
.
├── bag //保存bag的文件夹
│   └── 2021-07-19-03-13-46.bag 
├── CMakeLists.txt
├── config
│   └── calib.yaml //标定后的数据
├── data
│   └── log //保存的log信息的文件夹
├── include
│   ├── action.h //全场定位信息
│   ├── calibration.h
│   ├── coordinate.h
│   ├── DR_communication.h
│   ├── filter.h
│   ├── get_center.h
│   ├── get_var.h
│   ├── lidar.h
│   ├── pot_grab.h
│   ├── rc_laserscan
│   ├── save.h
│   └── TR_communication.h
├── launch
│   ├── calibrate.launch //跑标定的launch
│   ├── recall.launch //读取bag文件跑对应代码的launch
│   └── start.launch //正常读取硬件信息的跑代码的launch
├── msg
│   ├── coordinate.msg //自定义坐标信息
│   └── error.msg //自定义error信息
├── package.xml
├── README.md //README文件
├── RunCode.sh //执行代码的sh文件(需要放到工作空间路径才能执行)
├── scripts
│   ├── data.py //数据类
│   ├── display.py //显示类
│   ├── error.py //标定误差类
│   ├── plt_data.py //画图显示坐标随时间变化
│   └── plt_error.py //画图显示标定误差随时间变化
└── src
    ├── calibration.cpp //标定类
    ├── coordinate.cpp //坐标类
    ├── DR_communication.cpp //和DR通信
    ├── filter.cpp //滤波类
    ├── get_center.cpp //获取壶心
    ├── get_var.cpp //获取方差
    ├── lidar.cpp //雷达类
    ├── main.cpp //读取机器人信息跑的主函数
    ├── pot_grab.cpp //所要抓取的壶的信息
    ├── pub_tf.cpp //读取机器人信息跑的主函数对应的tf
    ├── recall_main.cpp //读取bag信息跑的主函数
    ├── recall_tf.cpp //读取bag信息跑的主函数对应的tf
    ├── save.cpp //保存
    ├── transform.cpp //运动补偿transform
    └── TR_communication.cpp //和TR通信
```



（不论跑任何模式都需要先把RunCode.sh文件放到工作空间目录下）

### 标定

标定需要将机器人放在世界坐标原点，将标定板放在世界坐标另一点（尽可能近），然后修改calibrate.h里面的        DrAction2Calib_x、DrAction2Calib_y。另外需要去calibrate.cpp修改滤波的最大半径CalibDistanceMax，尽可能保证机器人在半径以内只有标定板无其他物体。

然后切换到工作空间，执行

```
sh  RunCode.sh y
```

注意：如果没有可视化图形，则说明python文件没有在权限处允许文件作为程序执行



### 上场使用

读取机器人硬件信息，运行在机器人只需要切换路径到工作空间，然后执行（可以设置开机自启动）

```bash
sh  RunCode.sh
```

如果没有可视化图形同理



### 读取bag使用

读取bag信息需要切换到./src/rc_laserscan/bag路径 执行

```bash
rosbag play xxx.bag
```

然后切换到工作空间，执行

```
roslaunch rc_laserscan recall.launch
```



### 代码思路

1.pub_tf文件读取全场定位数据，然后广播成tf数据，用于transform中进行运动补偿，将原始雷达数据通过运动补偿处理后发布成new_scan。

2.main回调函数接受new_scan，对运动补偿后的数据进行处理。

3.读取标定数据，也就是雷达相对于机器人中心的数据。

4.读取全场定位数据。

5.卡尔曼滤波。

6.滤去除了我方DR内场以外的数据。

7.对壶的数据进行处理获得雷达坐标系下的坐标。

8.坐标系转换，将坐标转换成TR所需瞄准的坐标。

9.计算与最近壶的x，y，以及该壶转过的角度，方便DR快速对准抓壶。

10.保存数据以及发布数据用于可视化。



### 坐标系说明

场地图纸中 以DR面对最中间的壶的方向为上

雷达坐标系：上x 左y

DR坐标系：右x 上y

TR坐标系：上x 左y

最终以TR坐标系 将数据发给TR



### recall

在实际过程中，调试条件起始很苛刻，比如机器出故障需要维修，比如控制系统出故障，都会使得调试无法进行。所以我们将调试过程的雷达数据和全场定位数据记录成bag文件，这样子就可以完全脱离机器人，验证算法以及调试代码，方便有效。

原先的pub_tf发布：世界坐标系到里程计坐标系的转换关系、里程计坐标系到雷达坐标系的转换关系
现在的recall_tf发布：里程计坐标系到雷达坐标系的转换关系（世界坐标系到里程计坐标系的转换关系已在bag文件播放时发布）