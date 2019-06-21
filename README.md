# Linear-Control

- 控制程序 *Matlab/*
- Matlab 仿真见 *Simulation/*
- 实测数据见 *ViconData/*

本项目将 Vicon 系统（用于定位）、蓝牙小车进行了封装，并在此基础上定义了定位系统、小车、控制器三个对象，并开发了实时显示车辆轨迹、鼠标点击设置目标位置等功能，并能提供运行中产生的定位数据用于后期分析，为两轮小车控制系统的测试与开发提供了简单易用的平台。

用于获取位置信息的对象定义在 *Matlab/VData.m*。

用于控制小车蓝牙连接及车轮转速的对象定义在 *Matlab/Car.m* 。

接收被控对象位置、目标位置、障碍物位置，计算出被控对象两车轮转速的对象定义在 *Matlab/Controller.m* 。

平台基础功能介绍见 *小车控制系统文档.pdf*

![系统结构](https://raw.githubusercontent.com/tobeatraceur/Linear-Control/master/Presentation/Resources/SystemStructure.jpg)
