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
  int i;
  
  // 全速前进急停后退
  Serial.println("全速前进急停后退");
  car.forward(10);
  car.backward(10);
  car.stop(5);
  
  // 小车间断性前进5步
  Serial.println("间断性前进");
  for (i = 0; i < 5; i++) {
    car.forward(10);
    car.stop(1);
  }
  
  // 小车间断性后退5步
  Serial.println("间断性后退");
  for (i = 0; i < 5; i++) {
    car.backward(10);
    car.stop(1);
  }
  
  // 大弯套小弯连续左旋转
  Serial.println("大弯套小弯连续左旋转");
  for (i = 0; i < 5; i++) {
    car.turnLeft(10);
    car.spinLeft(5);
  }
  
  // 大弯套小弯连续右旋转
  Serial.println("大弯套小弯连续右旋转");
  for (i = 0; i < 5; i++) {
    car.turnRight(10);
    car.spinRight(5);
  }
  
  // 间断性原地右转弯
  Serial.println("间断性原地右转弯");
  for (i = 0; i < 10; i++) {
    car.turnRight(1);
    car.stop(1);
  }
  
  // 间断性原地左转弯
  Serial.println("间断性原地左转弯");
  for (i = 0; i < 10; i++) {
    car.turnLeft(1);
    car.stop(1);
  }
  
  // 走S形前进
  Serial.println("走S形前进");
  for (i = 0; i < 10; i++) {
    car.turnLeft(3);
    car.turnRight(3);
  }
  
  // 间断性原地左打转
  Serial.println("间断性原地左打转");
  for (i = 0; i < 10; i++) {
    car.spinLeft(3);
    car.stop(3);
  }
  
  // 间断性原地右打转
  Serial.println("间断性原地右打转");
  for (i = 0; i < 10; i++) {
    car.spinRight(3);
    car.stop(3);
  }
  
  // 完成一个循环后等待5秒
  car.stop(0);
  delay(5000);
}