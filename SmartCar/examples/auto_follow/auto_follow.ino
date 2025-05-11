/**
 * @file AutoFollow.ino
 * @brief 优化版智能小车自动跟随程序
 * @details 解决反应慢和间歇性行驶问题，提高响应速度
 */

#include <smart_car.h>

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

// 全局变量
int g_distance = 0;
MoveState g_current_state = STATE_STOP;
unsigned long g_last_measure_time = 0;
unsigned long g_last_change_time = 0;

// 创建SmartCar对象
SmartCar car(LEFT_MOTOR_GO, LEFT_MOTOR_BACK, RIGHT_MOTOR_GO, RIGHT_MOTOR_BACK);

// 函数声明
void MeasureDistance();
void UpdateMovement();
int CalculateSpeed(float error);
void StopMoving();

void setup() {
  // 初始化串口
  Serial.begin(9600);
   
  // 首先设置传感器
  car.SetupUltrasonicSensor(TRIG_PIN, ECHO_PIN);
  
  // 然后初始化小车
  car.Begin();
  
  // 设置较高的初始速度以减少启动延迟
  car.SetSpeed(MAX_SPEED, MAX_SPEED);
}

void loop() {
  // 测量距离 - 尽量减少不必要的测量以提高响应速度
  unsigned long current_time = millis();
  if (current_time - g_last_measure_time >= 50) { // 每50ms测量一次
    MeasureDistance();
    g_last_measure_time = current_time;
    
    // 根据距离调整行为
    UpdateMovement();
  }
}

// 测量距离
void MeasureDistance() {
  g_distance = car.GetDistance();
}

// 更新移动状态
void UpdateMovement() {
  // 检查距离是否有效
  if (g_distance < 2 || g_distance > 300) {
    StopMoving();
    Serial.println("距离无效，停止");
    return;
  }
  
  // 计算与目标距离的误差
  float error = g_distance - TARGET_DISTANCE;
  
  // 决定移动方向和速度
  if (abs(error) <= TOLERANCE) {
    // 在误差范围内，停止
    if (g_current_state != STATE_STOP) {
      StopMoving();
    }
  } 
  else if (error > 0) {
    // 目标太远，前进
    if (g_current_state != STATE_FORWARD) {
      // 状态变化时有声音提示
      unsigned long now = millis();
      if (now - g_last_change_time > 500) {
        g_last_change_time = now;
      }
      
      // 计算速度 - 距离越远速度越大
      int speed = CalculateSpeed(error);
      car.SetSpeed(speed, speed);
      
      // 前进并记录状态
      car.Forward(0);
      g_current_state = STATE_FORWARD;
    }
  } 
  else {
    // 目标太近，后退
    if (g_current_state != STATE_BACKWARD) {
      unsigned long now = millis();
      if (now - g_last_change_time > 500) {
        g_last_change_time = now;
      }
      
      // 计算速度 - 距离偏差越大速度越大
      int speed = CalculateSpeed(abs(error));
      car.SetSpeed(speed, speed);
      
      // 后退并记录状态
      car.Backward(0);
      g_current_state = STATE_BACKWARD;
    }
  }
}

// 计算电机速度
int CalculateSpeed(float error) {
  // 使用平方根函数可以使小误差时有更小的速度增量，大误差时有更大的速度增量
  int speed = MIN_SPEED + sqrt(error) * 10;
  
  // 限制在最小和最大速度范围内
  if (speed > MAX_SPEED) speed = MAX_SPEED;
  if (speed < MIN_SPEED) speed = MIN_SPEED;
  
  return speed;
}

// 停止移动
void StopMoving() {
  car.Stop(0);
  g_current_state = STATE_STOP;
  delay(STOP_DELAY); // 短暂延时确保完全停止
}
