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
const float SAFE_DISTANCE = 30.0;

// 左轮90度转弯相关参数
const int SPIN_TIME_90_DEGREE_LEFT = 3;  // 旋转90度所需时间(单位:100ms)
// 右轮90度转弯相关参数
const int SPIN_TIME_90_DEGREE_RIGHT = 3.5;
// 创建SmartCar对象
SmartCar car(LEFT_MOTOR_GO, LEFT_MOTOR_BACK, RIGHT_MOTOR_GO, RIGHT_MOTOR_BACK);

// 自定义函数声明
void avoidObstacleWithPreciseTurning(float safeDistance);
boolean isPathClear(float minSafeDistance);

void setup() {
  // 初始化串口
  Serial.begin(9600);
  Serial.println("SmartCar超声波避障示例");
  
  // 设置超声波传感器
  car.setupUltrasonicSensor(TRIG_PIN, ECHO_PIN);
  // 设置舵机
  car.setupServo(SERVO_PIN);
  
  // 初始化小车
  car.begin();
  
  // 设置电机速度
  car.setSpeed(180, 145);
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
  
  // 判断是否需要避障
  if (frontDistance < safeDistance) {
    // 停止并后退
    car.stop(1);
    car.backward(2);
    car.stop(1);
    
    // 检测左右两侧距离
    float leftDistance = car.detectLeft();
    
    float rightDistance = car.detectRight();
    
    // 将舵机转回前方
    car.detectFront();
    
    // 根据左右距离决定转向
    if (leftDistance < safeDistance && rightDistance < safeDistance) {
      // 左右两侧都有障碍物，掉头(180度)
      car.spinLeft(SPIN_TIME_90_DEGREE_RIGHT * 2);
      car.stop(1);
    } else if (leftDistance > rightDistance) {
      // 左侧空间更大，向左转90度
      car.spinLeft(SPIN_TIME_90_DEGREE_RIGHT);
      car.stop(1); // 确保停止并稳定姿态
      
      // 初始化绕行状态
      bool obstacleCleared = false;  // 是否已绕过障碍物
      int moveCount = 0;  // 移动计数器
      const int MAX_MOVE_COUNT = 40;  // 最大移动次数（安全措施）
      const int MIN_MOVE_COUNT = 1;   // 最小移动次数（确保至少前进一小段距离）
      
      // 前进并持续检测右侧障碍物
      while (!obstacleCleared && moveCount < MAX_MOVE_COUNT) {
        // 前进一小步
        car.forward(5);
        
        // 检测右侧（原障碍物方向）距离
        float rightSideDistance = car.detectRight();
        
        // 计数器增加
        moveCount++;
        
        // 判断是否已经绕过障碍物
        // 条件：已经前进了最小距离 + 右侧距离变大（说明已经绕过障碍物边缘）
        if (moveCount > MIN_MOVE_COUNT && rightSideDistance > safeDistance * 1.5) {
          obstacleCleared = true;
        }
        
        // 每次检测后短暂延时
        delay(100);
      }
      
      car.stop(1); // 停车稳定
      
      if (obstacleCleared) {
        // 已绕过障碍物，转回原来方向
        car.spinRight(SPIN_TIME_90_DEGREE_LEFT);
        car.stop(1);
      }
      car.detectFront(); 
      car.forward(0); // 持续前进
    } else {
      // 右侧空间更大（或与左侧相等，则默认向右），向右转90度
      car.spinRight(SPIN_TIME_90_DEGREE_LEFT);
      car.stop(1); // 确保停止并稳定姿态

      // 使用动态检测方式绕过障碍物
      
      // 初始化绕行状态
      bool obstacleCleared = false;  // 是否已绕过障碍物
      int moveCount = 0;  // 移动计数器
      const int MAX_MOVE_COUNT = 40;  // 最大移动次数（安全措施）
      const int MIN_MOVE_COUNT = 5;   // 最小移动次数（确保至少前进一小段距离）
      
      // 前进并持续检测左侧障碍物
      while (!obstacleCleared && moveCount < MAX_MOVE_COUNT) {
        // 前进一小步
        car.forward(5);
        
        // 检测左侧（原障碍物方向）距离
        float leftSideDistance = car.detectLeft();
        
        // 计数器增加
        moveCount++;
        
        // 判断是否已经绕过障碍物
        // 条件：已经前进了最小距离 + 左侧距离变大（说明已经绕过障碍物边缘）
        if (moveCount > MIN_MOVE_COUNT && leftSideDistance > safeDistance * 1.5) {
          obstacleCleared = true;
        }
        
        // 每次检测后短暂延时
        delay(100);
      }
      
      car.stop(1); // 停车稳定
      
      if (obstacleCleared) {
        // 已绕过障碍物，转回原来方向
        car.spinLeft(SPIN_TIME_90_DEGREE_RIGHT);
        car.stop(1);
      }
      
      // 将舵机转回前方并继续前进
      car.detectFront();
      car.forward(0); // 持续前进
    }
  } else {
    // 前方无障碍物，继续前进
    car.forward(0);
  }
}
