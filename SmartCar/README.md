# SmartCar 智能小车库

## 简介

这是一个用于Arduino智能小车控制的综合库，提供了电机控制、传感器操作和运动模式等功能，使开发者能够快速实现各种智能小车应用。

## 功能特点

- **模块化设计**：将电机控制、传感器等功能模块化，便于扩展和维护
- **丰富的运动控制**：支持前进、后退、转向、旋转等多种运动模式
- **传感器支持**：集成超声波测距、红外循迹和舵机控制功能
- **多方向避障**：通过舵机控制超声波传感器检测前方、左侧和右侧障碍物，智能决策行驶方向

- **高度可配置**：可根据需要灵活配置各种传感器和功能
- **代码复用**：遵循腾讯C++代码规范，提高代码复用度

## 库结构

```
SmartCar/
├── examples/                 # 示例代码目录
│   ├── BasicMovement/       # 基本运动示例
│   ├── LineTracking/        # 黑线循迹示例
│   └── ObstacleAvoidance/   # 超声波避障示例
├── src/                     # 库源代码目录
│   ├── Motor.h              # 电机控制类头文件
│   ├── Motor.cpp            # 电机控制类实现文件
│   ├── UltrasonicSensor.h   # 超声波传感器类头文件
│   ├── UltrasonicSensor.cpp # 超声波传感器类实现文件
│   ├── InfraredSensor.h     # 红外传感器类头文件
│   ├── InfraredSensor.cpp   # 红外传感器类实现文件
│   ├── Servo.h              # 舵机控制类头文件
│   ├── Servo.cpp            # 舵机控制类实现文件
│   ├── SmartCar.h           # 智能小车主类头文件
│   └── SmartCar.cpp         # 智能小车主类实现文件
├── keywords.txt             # 关键字定义文件
├── library.properties       # 库描述文件
└── README.md                # 说明文档
```

## 安装方法

1. 下载本库的ZIP压缩包
2. 打开Arduino IDE，选择「项目」->「加载库」->「添加.ZIP库」
3. 选择下载的ZIP文件，点击「打开」完成安装

## 快速入门

### 基本使用

```cpp
#include <SmartCar.h>

// 定义引脚
const int LEFT_MOTOR_GO = 5;      // 左电机前进引脚
const int LEFT_MOTOR_BACK = 9;    // 左电机后退引脚
const int RIGHT_MOTOR_GO = 6;     // 右电机前进引脚
const int RIGHT_MOTOR_BACK = 10;  // 右电机后退引脚

// 创建SmartCar对象
SmartCar car(LEFT_MOTOR_GO, LEFT_MOTOR_BACK, RIGHT_MOTOR_GO, RIGHT_MOTOR_BACK);

void setup() {
  // 初始化小车
  car.begin();
  
  // 设置电机速度
  car.setSpeed(200, 200);
}

void loop() {
  // 前进1秒
  car.forward(10);
  
  // 停止0.5秒
  car.stop(5);
  
  // 后退1秒
  car.backward(10);
  
  // 停止0.5秒
  car.stop(5);
}
```

### 添加传感器

```cpp
// 设置超声波传感器
car.setupUltrasonicSensor(TRIG_PIN, ECHO_PIN);

// 设置红外传感器
car.setupInfraredSensor(LEFT_IR_PIN, RIGHT_IR_PIN);

// 获取前方距离
float distance = car.getDistance();

// 执行黑线循迹
int trackStatus = car.trackLine();
```

## 类说明

### SmartCar 类

智能小车主类，整合了电机控制、传感器等功能。

### Motor 类

电机控制类，负责控制小车的电机，实现基本的运动功能。

### UltrasonicSensor 类

超声波传感器类，负责超声波传感器的控制和距离测量。

### InfraredSensor 类

红外传感器类，负责红外传感器的控制和黑线循迹。

## 示例说明

- **BasicMovement**：演示基本的运动控制，包括前进、后退、转向和旋转
- **LineTracking**：演示如何使用红外传感器实现黑线循迹功能
- **ObstacleAvoidance**：演示如何使用超声波传感器实现避障功能

## 注意事项

- 使用前请确保正确连接电机和传感器
- 电机速度范围为0-255，建议根据实际情况调整
- 超声波测距有效范围为2-400厘米
    