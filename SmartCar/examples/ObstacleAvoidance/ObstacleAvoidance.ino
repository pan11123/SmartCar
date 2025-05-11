/**
 * @file ObstacleAvoidance.ino
 * @brief 智能小车超声波避障示例(带舵机和精确转弯)
 * @details 演示如何使用SmartCar库实现带舵机的多方向超声波避障功能，具有90度精确转弯和实时障碍物检测
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
const float SAFE_DISTANCE = 32.0;

// 90度转弯相关参数
const int SPIN_TIME_90_DEGREE = 5;  // 旋转90度所需时间(单位:100ms)

// 创建SmartCar对象
SmartCar car(LEFT_MOTOR_GO, LEFT_MOTOR_BACK, RIGHT_MOTOR_GO, RIGHT_MOTOR_BACK);

// 自定义函数声明
void avoidObstacleWithPreciseTurning(float safeDistance);
boolean isPathClear(float minSafeDistance);

void setup() {
  // 初始化串口
  Serial.begin(9600);
  Serial.println("SmartCar超声波避障示例(带精确90度转弯)");
  
  // 设置超声波传感器
  car.setupUltrasonicSensor(TRIG_PIN, ECHO_PIN);
  // 设置舵机
  car.setupServo(SERVO_PIN);
  
  // 初始化小车
  car.begin();
  
  // 设置电机速度
  car.setSpeed(160, 165);
}

void loop() {
  // 使用精确转弯的避障功能
  avoidObstacleWithPreciseTurning(SAFE_DISTANCE);
  
  // 短暂延时
  delay(100);
}

/**
 * @brief 带精确90度转弯的避障功能
 * @param safeDistance 安全距离(厘米)
 */
void avoidObstacleWithPreciseTurning(float safeDistance) {
  // 如果没有配置超声波传感器和舵机，则直接返回
  if (!car.getDistance() || car.detectFront() < 0) {
    return;
  }
  
  // 检测前方距离
  float frontDistance = car.detectFront();
  Serial.print("前方距离: ");
  Serial.println(frontDistance);
  
  // 判断是否需要避障
  if (frontDistance < safeDistance) {
    // 停止并后退
    car.stop(1);
    car.backward(2);
    car.stop(1);
    
    // 检测左右两侧距离
    float leftDistance = car.detectLeft();
    Serial.print("左侧距离: ");
    Serial.println(leftDistance);
    
    float rightDistance = car.detectRight();
    Serial.print("右侧距离: ");
    Serial.println(rightDistance);
    
    // 将舵机转回前方
    car.detectFront();
    
    // 根据左右距离决定转向
    if (leftDistance < safeDistance && rightDistance < safeDistance) {
      // 左右两侧都有障碍物，掉头(180度)
      Serial.println("左右有障碍，掉头");
      car.spinLeft(SPIN_TIME_90_DEGREE * 2);
      car.stop(1);
    } else if (leftDistance > rightDistance) {
      // 左侧空间更大，向左转90度
      Serial.println("向左转90度");
      car.spinLeft(SPIN_TIME_90_DEGREE);
      car.stop(1);
      
      // 转弯后检查前方是否通畅
      if (!isPathClear(safeDistance)) {
        // 如果前方仍有障碍物，继续向左转90度
        Serial.println("前方仍有障碍，继续向左转90度");
        car.spinLeft(SPIN_TIME_90_DEGREE);
        car.stop(1);
      }
    } else {
      // 右侧空间更大，向右转90度
      Serial.println("向右转90度");
      car.spinRight(SPIN_TIME_90_DEGREE);
      car.stop(1);
      
      // 转弯后检查前方是否通畅
      if (!isPathClear(safeDistance)) {
        // 如果前方仍有障碍物，继续向右转90度
        Serial.println("前方仍有障碍，继续向右转90度");
        car.spinRight(SPIN_TIME_90_DEGREE);
        car.stop(1);
      }
    }
  } else {
    // 前方无障碍物，继续前进
    car.forward(0);
  }
}

/**
 * @brief 检测当前路径是否通畅
 * @param minSafeDistance 最小安全距离
 * @return 如果路径通畅返回true，否则返回false
 */
boolean isPathClear(float minSafeDistance) {
  // 检测前方距离
  float frontDistance = car.detectFront();
  Serial.print("检测路径通畅 距离: ");
  Serial.println(frontDistance);
  
  // 如果距离大于安全距离，则路径通畅
  return frontDistance >= minSafeDistance;
}

