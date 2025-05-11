/**
 * @file line_tracking.ino
 * @brief 智能小车黑线循迹示例
 * @details 演示如何使用SmartCar库实现小车沿黑线行驶的功能
 */

#include <smart_car.h>

// 定义引脚
const int LEFT_MOTOR_GO = 5;      // 左电机前进引脚
const int LEFT_MOTOR_BACK = 9;    // 左电机后退引脚
const int RIGHT_MOTOR_GO = 6;     // 右电机前进引脚
const int RIGHT_MOTOR_BACK = 10;  // 右电机后退引脚

// 定义红外传感器引脚
const int LEFT_IR_SENSOR = A3;   // 左红外传感器引脚
const int RIGHT_IR_SENSOR = A2;  // 右红外传感器引脚

// 创建SmartCar对象
SmartCar car(LEFT_MOTOR_GO, LEFT_MOTOR_BACK, RIGHT_MOTOR_GO, RIGHT_MOTOR_BACK);

// 自定义函数声明
void FollowLine();

void setup() {
  // 初始化串口
  Serial.begin(9600);
  Serial.println("SmartCar黑线循迹示例");
  
  // 设置红外传感器
  car.SetupInfraredSensor(LEFT_IR_SENSOR, RIGHT_IR_SENSOR);
  // 初始化小车
  car.Begin();
  // 设置电机速度
  car.SetSpeed(100, 100);
}

void loop() {
  // 执行循迹功能
  car.TrackLine();

  // 短暂延时
  delay(10);
}
