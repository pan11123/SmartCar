# SmartCar 智能小车库

## 简介

这是一个用于Arduino智能小车控制的综合库，提供了电机控制、传感器操作和运动模式等功能，使开发者能够快速实现各种智能小车应用。

## 功能特点

- **模块化设计**：将电机控制、传感器等功能模块化，便于扩展和维护
- **丰富的运动控制**：支持前进、后退、转向、旋转等多种运动模式
- **传感器支持**：集成超声波测距、红外循迹和舵机控制功能
- **多方向避障**：通过舵机控制超声波传感器检测前方、左侧和右侧障碍物，智能决策行驶方向
- **高度可配置**：可根据需要灵活配置各种传感器和功能
- **代码复用**：遵循现代C++代码规范，提高代码复用度

## 编码规范

本库严格遵循以下命名规范：
- 文件命名：小写+下划线，如 smart_car.h
- 类名：大写开头驼峰式，如 SmartCar
- 函数/方法：大写开头驼峰式，如 GetDistance()
- 成员变量：前缀m_，如 m_leftPin
- 全局常量：全大写+下划线，如 MAX_SPEED
- 局部变量：小写+下划线，如 left_sensor

## 库结构

```
SmartCar/
├── examples/                 # 示例代码目录
│   ├── basic_movement/       # 基本运动示例
│   ├── line_tracking/        # 黑线循迹示例
│   └── obstacle_avoidance/   # 超声波避障示例
├── src/                     # 库源代码目录
│   ├── motor.h              # 电机控制类头文件
│   ├── motor.cpp            # 电机控制类实现文件
│   ├── ultrasonic_sensor.h   # 超声波传感器类头文件
│   ├── ultrasonic_sensor.cpp # 超声波传感器类实现文件
│   ├── infrared_sensor.h     # 红外传感器类头文件
│   ├── infrared_sensor.cpp   # 红外传感器类实现文件
│   ├── servo.h              # 舵机控制类头文件
│   ├── servo.cpp            # 舵机控制类实现文件
│   ├── smart_car.h           # 智能小车主类头文件
│   └── smart_car.cpp         # 智能小车主类实现文件
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
#include <smart_car.h>

// 定义引脚
const int LEFT_MOTOR_GO = 5;      // 左电机前进引脚
const int LEFT_MOTOR_BACK = 9;    // 左电机后退引脚
const int RIGHT_MOTOR_GO = 6;     // 右电机前进引脚
const int RIGHT_MOTOR_BACK = 10;  // 右电机后退引脚

// 创建SmartCar对象
SmartCar car(LEFT_MOTOR_GO, LEFT_MOTOR_BACK, RIGHT_MOTOR_GO, RIGHT_MOTOR_BACK);

void setup() {
  // 初始化小车
  car.Begin();
  
  // 设置电机速度
  car.SetSpeed(200, 200);
}

void loop() {
  // 前进1秒
  car.Forward(10);
  
  // 停止0.5秒
  car.Stop(5);
  
  // 后退1秒
  car.Backward(10);
  
  // 停止0.5秒
  car.Stop(5);
}
```

### 添加传感器

```cpp
// 设置超声波传感器
car.SetupUltrasonicSensor(TRIG_PIN, ECHO_PIN);

// 设置红外传感器
car.SetupInfraredSensor(LEFT_IR_PIN, RIGHT_IR_PIN);

// 获取前方距离
float distance = car.GetDistance();

// 执行黑线循迹
int track_status = car.TrackLine();
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

### Servo 类

舵机控制类，负责舵机的控制，用于多方向避障功能。

## 示例说明

- **basic_movement**：演示基本的运动控制，包括前进、后退、转向和旋转
- **line_tracking**：演示如何使用红外传感器实现黑线循迹功能
- **obstacle_avoidance**：演示如何使用超声波传感器实现避障功能

## 注意事项

- 使用前请确保正确连接电机和传感器
- 电机速度范围为0-255，建议根据实际情况调整
- 超声波测距有效范围为2-400厘米
    