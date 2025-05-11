/**
 * @file ObstacleAvoidance.ino
 * @brief 智能小车超声波避障示例(动态绕障)
 * @details 演示如何使用SmartCar库实现动态超声波避障功能，能够智能检测障碍物边缘
 */

#include <SmartCar.h>

// 定义引脚
const int LEFT_MOTOR_GO = 5;      // 左电机前进引脚
const int LEFT_MOTOR_BACK = 9;    // 左电机后退引脚
const int RIGHT_MOTOR_GO = 6;     // 右电机前进引脚
const int RIGHT_MOTOR_BACK = 10;  // 右电机后退引脚

// 定义超声波传感器引脚
const int TRIG_PIN = A4;          // 超声波触发引脚
const int ECHO_PIN = A5;          // 超声波回声引脚

// 定义舵机引脚
const int SERVO_PIN = 2;          // 舵机控制引脚

// 定义安全距离(厘米)
const float SAFE_DISTANCE = 30.0;

// 左轮90度转弯相关参数
const float SPIN_TIME_90_DEGREE_LEFT = 3.0;  // 旋转90度所需时间(单位:100ms)
// 右轮90度转弯相关参数
const float SPIN_TIME_90_DEGREE_RIGHT = 3.5;

// 创建SmartCar对象
SmartCar car(LEFT_MOTOR_GO, LEFT_MOTOR_BACK, RIGHT_MOTOR_GO, RIGHT_MOTOR_BACK);

void setup() {
  // 初始化串口
  Serial.begin(9600);
  Serial.println("SmartCar动态超声波避障示例");
  
  // 设置超声波传感器
  car.SetupUltrasonicSensor(TRIG_PIN, ECHO_PIN);
  // 设置舵机
  car.SetupServo(SERVO_PIN);
  
  // 初始化小车
  car.Begin();
  
  // 设置电机速度
  car.SetSpeed(180, 145);
}

void loop() {
  // 使用动态避障功能
  car.DynamicAvoidObstacle(SAFE_DISTANCE, SPIN_TIME_90_DEGREE_LEFT, SPIN_TIME_90_DEGREE_RIGHT);
  
  // 短暂延时
  delay(100);
}
