/**
 * @file LineTracking.ino
 * @brief 智能小车黑线循迹示例
 * @details 演示如何使用SmartCar库实现黑线循迹功能
 */

#include <SmartCar.h>

// 定义引脚
const int LEFT_MOTOR_GO = 5;      // 左电机前进引脚
const int LEFT_MOTOR_BACK = 9;    // 左电机后退引脚
const int RIGHT_MOTOR_GO = 6;     // 右电机前进引脚
const int RIGHT_MOTOR_BACK = 10;  // 右电机后退引脚

// 定义红外传感器引脚
const int LEFT_IR_PIN = A3;       // 左侧红外传感器引脚
const int RIGHT_IR_PIN = A2;      // 右侧红外传感器引脚

// 定义按键和蜂鸣器引脚
const int KEY_PIN = A0;           // 按键引脚
const int BEEP_PIN = A1;          // 蜂鸣器引脚

// 创建SmartCar对象
SmartCar car(LEFT_MOTOR_GO, LEFT_MOTOR_BACK, RIGHT_MOTOR_GO, RIGHT_MOTOR_BACK);

// 循迹状态
int trackStatus = 0;

void setup() {
  // 初始化串口
  Serial.begin(9600);
  Serial.println("SmartCar黑线循迹示例");
  
  // 初始化按键和蜂鸣器
  pinMode(KEY_PIN, INPUT);
  pinMode(BEEP_PIN, OUTPUT);
  
  // 初始化小车
  car.begin();
  
  // 设置红外传感器
  car.setupInfraredSensor(LEFT_IR_PIN, RIGHT_IR_PIN);
  
  // 设置电机速度
  car.setSpeed(150, 150);
  
  // 等待按键按下后开始
  Serial.println("按下按键开始循迹");
  keyScan();
}

void loop() {
  // 执行黑线循迹
  trackStatus = car.trackLine();
  
  // 输出循迹状态
  switch (trackStatus) {
    case 0:
      Serial.println("直行");
      break;
    case 1:
      Serial.println("左转");
      break;
    case 2:
      Serial.println("右转");
      break;
    case 3:
      Serial.println("停止");
      break;
    default:
      Serial.println("未知状态");
      break;
  }
  
  // 短暂延时
  delay(10);
}

// 按键扫描函数
void keyScan() {
  int val;
  val = digitalRead(KEY_PIN);
  while (!digitalRead(KEY_PIN)) {
    val = digitalRead(KEY_PIN);
  }
  while (digitalRead(KEY_PIN)) {
    delay(10);
    val = digitalRead(KEY_PIN);
    if (val == HIGH) {
      digitalWrite(BEEP_PIN, HIGH);
      while (!digitalRead(KEY_PIN)) {
        digitalWrite(BEEP_PIN, LOW);
      }
    } else {
      digitalWrite(BEEP_PIN, LOW);
    }
  }
}