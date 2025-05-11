/**
 * @file distance_detection.ino
 * @brief 超声波测距示例程序
 * @details 演示如何使用SmartCar库的超声波传感器来检测距离
 */

#include <smart_car.h>

// 定义引脚
#define LEFT_MOTOR_GO     8  // 左电机前进引脚
#define LEFT_MOTOR_BACK   9  // 左电机后退引脚
#define RIGHT_MOTOR_GO    10 // 右电机前进引脚
#define RIGHT_MOTOR_BACK  11 // 右电机后退引脚
#define ECHO_PIN          2  // 超声波Echo引脚
#define TRIG_PIN          3  // 超声波Trig引脚

// 定义常量
#define CHECK_INTERVAL    500  // 检测间隔(毫秒)
#define SAFE_DISTANCE     30.0 // 安全距离(厘米)
#define WARNING_DISTANCE  15.0 // 警告距离(厘米)

// 创建SmartCar对象
SmartCar car(LEFT_MOTOR_GO, LEFT_MOTOR_BACK, RIGHT_MOTOR_GO, RIGHT_MOTOR_BACK);

// 功能函数声明
void CheckDistance();
void DisplayDistanceStatus(float distance);

// 全局变量
unsigned long g_lastCheckTime = 0; // 上次检测时间

void setup() {
  // 初始化串口
  Serial.begin(9600);
  
  // 初始化小车
  car.Begin();
  
  // 设置超声波传感器
  car.SetupUltrasonicSensor(TRIG_PIN, ECHO_PIN);
  
  // 输出提示信息
  Serial.println(F("超声波测距示例程序"));
  Serial.println(F("---------------------"));
  Serial.println(F("本示例将实时检测前方障碍物距离"));
  Serial.println(F("安全距离: >30厘米"));
  Serial.println(F("警告距离: 15-30厘米"));
  Serial.println(F("危险距离: <15厘米"));
  Serial.println(F("---------------------"));
  
  // 等待2秒，方便查看提示信息
  delay(2000);
}

void loop() {
  // 定时检测距离
  unsigned long current_time = millis();
  if (current_time - g_lastCheckTime >= CHECK_INTERVAL) {
    CheckDistance();
    g_lastCheckTime = current_time;
  }
}

/**
 * @brief 检测并显示距离
 */
void CheckDistance() {
  // 获取前方障碍物距离
  float distance = car.GetDistance();
  
  // 显示距离状态
  DisplayDistanceStatus(distance);
}

/**
 * @brief 显示距离状态
 * @param distance 测量的距离(厘米)
 */
void DisplayDistanceStatus(float distance) {
  // 打印当前距离
  Serial.print(F("当前距离: "));
  Serial.print(distance);
  Serial.println(F(" 厘米"));
  
  // 根据距离显示不同状态
  if (distance < WARNING_DISTANCE) {
    Serial.println(F("状态: [危险] 距离过近!"));
    // 危险状态可以添加LED闪烁或蜂鸣器报警等
  } else if (distance < SAFE_DISTANCE) {
    Serial.println(F("状态: [警告] 请注意前方障碍物"));
  } else {
    Serial.println(F("状态: [安全] 前方道路畅通"));
  }
  
  Serial.println(F("---------------------"));
} 