/**
 * @file LineTracking.ino
 * @brief 智能小车黑线循迹示例
 * @details 演示如何使用SmartCar库实现小车沿黑线行驶的功能
 */

#include <SmartCar.h>

// 定义引脚
const int LEFT_MOTOR_GO = 5;      // 左电机前进引脚
const int LEFT_MOTOR_BACK = 9;    // 左电机后退引脚
const int RIGHT_MOTOR_GO = 6;     // 右电机前进引脚
const int RIGHT_MOTOR_BACK = 10;  // 右电机后退引脚

// 定义循迹传感器引脚
const int LEFT_TRACK_SENSOR = A3;   // 左循迹传感器引脚
const int RIGHT_TRACK_SENSOR = A2;  // 右循迹传感器引脚

// 创建SmartCar对象
SmartCar car(LEFT_MOTOR_GO, LEFT_MOTOR_BACK, RIGHT_MOTOR_GO, RIGHT_MOTOR_BACK);

// 自定义函数声明
void followLine();
void keyScan();

void setup() {
  // 初始化串口
  Serial.begin(9600);
  Serial.println("SmartCar黑线循迹示例");
  
  // 初始化小车
  car.begin();
  
  // 设置电机速度
  car.setSpeed(150, 150);
  
  // 设置循迹传感器
  car.setupTrackingSensors(LEFT_TRACK_SENSOR, RIGHT_TRACK_SENSOR);
  
  // 按键和蜂鸣器初始化
  pinMode(A0, INPUT);  // 按键
  pinMode(A1, OUTPUT); // 蜂鸣器
  
  // 等待按键按下开始
  Serial.println("按下按键开始循迹...");
  keyScan();
}

void loop() {
  // 执行循迹功能
  followLine();
  
  // 短暂延时
  delay(10);
}

/**
 * @brief 黑线循迹功能
 */
void followLine() {
  // 读取循迹传感器状态
  bool leftSensor = car.readLeftTrackSensor();
  bool rightSensor = car.readRightTrackSensor();
  
  // 根据传感器状态进行控制
  // 有信号为LOW (黑线), 无信号为HIGH (白色区域)
  if (leftSensor == LOW && rightSensor == LOW) {
    // 两个传感器都在黑线上，直行
    Serial.println("两侧都检测到黑线，直行");
    car.forward(0);
  } else if (leftSensor == HIGH && rightSensor == LOW) {
    // 左传感器在白色区域，右传感器在黑线上，车偏右，需向左转
    Serial.println("左侧偏离黑线，向左转");
    car.turnLeft(0);
  } else if (leftSensor == LOW && rightSensor == HIGH) {
    // 左传感器在黑线上，右传感器在白色区域，车偏左，需向右转
    Serial.println("右侧偏离黑线，向右转");
    car.turnRight(0);
  } else {
    // 两个传感器都在白色区域，停止
    Serial.println("未检测到黑线，停止");
    car.stop(0);
  }
}

/**
 * @brief 按键扫描函数
 */
void keyScan() {
  int val;
  val = digitalRead(A0); // 读取按键状态
  
  // 等待按键按下
  while (!digitalRead(A0)) {
    val = digitalRead(A0);
  }
  
  // 当按键被按下时
  while (digitalRead(A0)) {
    delay(10);  // 延时10ms
    val = digitalRead(A0);
    if (val == HIGH) {
      // 按键确认被按下，蜂鸣器响一下
      digitalWrite(A1, HIGH);
      while (!digitalRead(A0)) {
        // 等待按键松开
      }
      digitalWrite(A1, LOW);
    } else {
      digitalWrite(A1, LOW);
    }
  }
  
  Serial.println("按键已按下，开始循迹!");
} 