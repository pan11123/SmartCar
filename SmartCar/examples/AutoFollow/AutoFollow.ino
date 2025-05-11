/**
 * @file AutoFollow.ino
 * @brief 优化版智能小车自动跟随程序
 * @details 解决反应慢和间歇性行驶问题，提高响应速度
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

// 定义跟随参数
const float TARGET_DISTANCE = 25.0;   // 目标跟随距离(厘米)
const float TOLERANCE = 3.0;          // 允许误差范围(厘米)
const int MAX_SPEED = 220;            // 最大速度(0-255)
const int MIN_SPEED = 180;            // 最小速度(0-255)
const int STOP_DELAY = 20;            // 停止延时(毫秒)

// 运动状态枚举
enum MoveState {
  STATE_STOP,
  STATE_FORWARD,
  STATE_BACKWARD
};

// 全局变量
int Distance = 0;
MoveState currentState = STATE_STOP;
unsigned long lastMeasureTime = 0;
unsigned long lastChangeTime = 0;

// 创建SmartCar对象
SmartCar car(LEFT_MOTOR_GO, LEFT_MOTOR_BACK, RIGHT_MOTOR_GO, RIGHT_MOTOR_BACK);

void setup() {
  // 初始化串口
  Serial.begin(9600);
   
  // 首先设置传感器
  car.setupUltrasonicSensor(TRIG_PIN, ECHO_PIN);
  
  // 然后初始化小车
  car.begin();
  
  // 设置较高的初始速度以减少启动延迟
  car.setSpeed(MAX_SPEED, MAX_SPEED);

}

void loop() {
  // 测量距离 - 尽量减少不必要的测量以提高响应速度
  unsigned long currentTime = millis();
  if (currentTime - lastMeasureTime >= 50) { // 每50ms测量一次
    measureDistance();
    lastMeasureTime = currentTime;
    
    // 根据距离调整行为
    updateMovement();
  }
}

// 测量距离
void measureDistance() {
  Distance = car.getDistance();
}

// 更新移动状态
void updateMovement() {
  // 检查距离是否有效
  if (Distance < 2 || Distance > 300) {
    stopMoving();
    Serial.println("距离无效，停止");
    return;
  }
  
  // 计算与目标距离的误差
  float error = Distance - TARGET_DISTANCE;
  
  // 决定移动方向和速度
  if (abs(error) <= TOLERANCE) {
    // 在误差范围内，停止
    if (currentState != STATE_STOP) {
      stopMoving();
    }
  } 
  else if (error > 0) {
    // 目标太远，前进
    if (currentState != STATE_FORWARD) {
      // 状态变化时有声音提示
      unsigned long now = millis();
      if (now - lastChangeTime > 500) {
        lastChangeTime = now;
      }
      
      // 计算速度 - 距离越远速度越大
      int speed = calculateSpeed(error);
      car.setSpeed(speed, speed);
      
      // 前进并记录状态
      car.forward(0);
      currentState = STATE_FORWARD;
    }
  } 
  else {
    // 目标太近，后退
    if (currentState != STATE_BACKWARD) {
      unsigned long now = millis();
      if (now - lastChangeTime > 500) {
        lastChangeTime = now;
      }
      
      // 计算速度 - 距离偏差越大速度越大
      int speed = calculateSpeed(abs(error));
      car.setSpeed(speed, speed);
      
      // 后退并记录状态
      car.backward(0);
      currentState = STATE_BACKWARD;
    }
  }
}

// 计算电机速度
int calculateSpeed(float error) {
  // 使用平方根函数可以使小误差时有更小的速度增量，大误差时有更大的速度增量
  int speed = MIN_SPEED + sqrt(error) * 10;
  
  // 限制在最小和最大速度范围内
  if (speed > MAX_SPEED) speed = MAX_SPEED;
  if (speed < MIN_SPEED) speed = MIN_SPEED;
  
  return speed;
}

// 停止移动
void stopMoving() {
  car.stop(0);
  currentState = STATE_STOP;
  delay(STOP_DELAY); // 短暂延时确保完全停止
}
