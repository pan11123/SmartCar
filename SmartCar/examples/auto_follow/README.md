# 自动跟随示例 (AutoFollow)

## 功能介绍

本示例演示了如何使用SmartCar库实现智能小车的自动跟随功能。小车会通过超声波传感器测量与前方障碍物的距离，并自动调整运动状态，使其与障碍物保持在设定的目标距离（默认30厘米）。

## 工作原理

1. 设定一个固定的目标距离（30厘米）
2. 通过超声波传感器实时测量小车与前方障碍物的距离
3. 根据测量距离与目标距离的差值，控制小车的运动状态：
   - 当距离接近目标距离（误差在±3厘米内）时，小车保持静止
   - 当距离小于目标距离时，小车后退
   - 当距离大于目标距离时，小车前进
4. 根据距离误差的大小动态调整小车的运动速度，距离误差越大，速度越快

## 硬件连接

- 左电机前进引脚：5
- 左电机后退引脚：9
- 右电机前进引脚：6
- 右电机后退引脚：10
- 超声波传感器触发引脚：A4
- 超声波传感器回声引脚：A5
- 按键引脚：A0
- 蜂鸣器引脚：A1

## 使用方法

1. 将代码上传到Arduino板
2. 打开串口监视器（波特率9600）
3. 按下按键启动自动跟随功能
4. 在小车前方放置一个可移动的障碍物
5. 移动障碍物，观察小车的跟随行为：
   - 障碍物不动时，小车保持静止
   - 障碍物向前移动（远离小车）时，小车前进跟随
   - 障碍物向后移动（靠近小车）时，小车后退

## 参数调整

可以根据实际需求调整以下参数：

- `TARGET_DISTANCE`：目标跟随距离，默认为30厘米
- `DISTANCE_TOLERANCE`：距离误差容忍范围，默认为±3厘米
- `NORMAL_SPEED`：正常运行速度，默认为150
- `SLOW_SPEED`：接近目标时的慢速，默认为100

## 注意事项

- 确保超声波传感器安装牢固，并正确朝向前方
- 避免在强光或强电磁干扰环境下使用，可能影响超声波测距的准确性
- 如果小车运动不稳定，可以尝试调整速度参数或增大距离误差容忍范围