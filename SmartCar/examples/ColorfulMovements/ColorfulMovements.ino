/**
 * @file ColorfulMovements.ino
 * @brief 智能小车花式动作示例
 * @details 演示如何使用SmartCar库实现各种花式动作
 */

#include <SmartCar.h>

// 定义引脚
const int LEFT_MOTOR_GO = 5;      // 左电机前进引脚
const int LEFT_MOTOR_BACK = 9;    // 左电机后退引脚
const int RIGHT_MOTOR_GO = 6;     // 右电机前进引脚
const int RIGHT_MOTOR_BACK = 10;  // 右电机后退引脚

// 创建SmartCar对象
SmartCar car(LEFT_MOTOR_GO, LEFT_MOTOR_BACK, RIGHT_MOTOR_GO, RIGHT_MOTOR_BACK);

void setup() {
  // 初始化串口
  Serial.begin(9600);
  Serial.println("SmartCar花式动作示例");
  
  // 初始化小车
  car.begin();
  
  // 设置电机速度
  car.setSpeed(200, 200);
  
  // 等待2秒后开始
  delay(2000);
}

void loop() {
  // 使用SmartCar库中的花式动作表演功能
  car.performColorfulMovements();
  
  // 完成一个循环后等待5秒
  delay(5000);
}