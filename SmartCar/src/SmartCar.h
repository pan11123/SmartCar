/**
 * @file smart_car.h
 * @brief 智能小车主类的头文件
 * @details 整合电机控制、传感器等功能，提供统一的接口
 */

#ifndef SMART_CAR_H
#define SMART_CAR_H

#include <Arduino.h>
#include "motor.h"
#include "ultrasonic_sensor.h"
#include "infrared_sensor.h"
#include "servo.h"
#include "tracking_sensor.h"

// 运动状态枚举
enum MoveState {
  STATE_STOP,
  STATE_FORWARD,
  STATE_BACKWARD,
  STATE_TURN_LEFT,
  STATE_TURN_RIGHT
};

/**
 * @class SmartCar
 * @brief 智能小车主类
 * @details 整合了电机控制、超声波传感器、红外传感器等功能
 */
class SmartCar {
public:
    /**
     * @brief 构造函数
     * @param leftMotorGo 左电机前进引脚
     * @param leftMotorBack 左电机后退引脚
     * @param rightMotorGo 右电机前进引脚
     * @param rightMotorBack 右电机后退引脚
     */
    SmartCar(uint8_t leftMotorGo, uint8_t leftMotorBack, 
             uint8_t rightMotorGo, uint8_t rightMotorBack);
    
    /**
     * @brief 初始化智能小车
     */
    void Begin();
    
    /**
     * @brief 设置超声波传感器
     * @param trigPin 触发引脚
     * @param echoPin 回声引脚
     */
    void SetupUltrasonicSensor(uint8_t trigPin, uint8_t echoPin);
    
    /**
     * @brief 设置红外传感器
     * @param leftPin 左侧红外传感器引脚
     * @param rightPin 右侧红外传感器引脚
     */
    void SetupInfraredSensor(uint8_t leftPin, uint8_t rightPin);
    
    /**
     * @brief 设置循迹传感器
     * @param leftPin 左侧循迹传感器引脚
     * @param rightPin 右侧循迹传感器引脚
     */
    void SetupTrackingSensors(uint8_t leftPin, uint8_t rightPin);
    
    /**
     * @brief 设置电机速度
     * @param leftSpeed 左电机速度(0-255)
     * @param rightSpeed 右电机速度(0-255)
     */
    void SetSpeed(uint8_t leftSpeed, uint8_t rightSpeed);
    
    /**
     * @brief 前进
     * @param time 前进时间(单位:100ms)，0表示持续运行
     */
    void Forward(uint16_t time = 0);
    
    /**
     * @brief 后退
     * @param time 后退时间(单位:100ms)，0表示持续运行
     */
    void Backward(uint16_t time = 0);
    
    /**
     * @brief 左转
     * @param time 左转时间(单位:100ms)，0表示持续运行
     */
    void TurnLeft(uint16_t time = 0);
    
    /**
     * @brief 右转
     * @param time 右转时间(单位:100ms)，0表示持续运行
     */
    void TurnRight(uint16_t time = 0);
    
    /**
     * @brief 左旋转
     * @param time 左旋转时间(单位:100ms)，0表示持续运行
     */
    void SpinLeft(uint16_t time = 0);
    
    /**
     * @brief 右旋转
     * @param time 右旋转时间(单位:100ms)，0表示持续运行
     */
    void SpinRight(uint16_t time = 0);
    
    /**
     * @brief 停止
     * @param time 停止时间(单位:100ms)，0表示持续停止
     */
    void Stop(uint16_t time = 0);
    
    /**
     * @brief 获取前方距离
     * @return 返回前方距离，单位为厘米
     */
    float GetDistance();
    
    /**
     * @brief 执行黑线循迹
     * @return 返回循迹状态，0表示直行，1表示左转，2表示右转，3表示停止
     */
    int TrackLine();
    
    /**
     * @brief 读取左侧循迹传感器状态
     * @return 传感器读数，LOW表示检测到黑线，HIGH表示检测到白色
     */
    bool ReadLeftTrackSensor();
    
    /**
     * @brief 读取右侧循迹传感器状态
     * @return 传感器读数，LOW表示检测到黑线，HIGH表示检测到白色
     */
    bool ReadRightTrackSensor();
    
    /**
     * @brief 避障行驶
     * @param safeDistance 安全距离，单位为厘米
     */
    void AvoidObstacle(float safeDistance = 30.0);
    
    /**
     * @brief 设置舵机
     * @param servoPin 舵机控制引脚
     */
    void SetupServo(uint8_t servoPin);
    
    /**
     * @brief 使用舵机进行多方向避障
     * @param safeDistance 安全距离，单位为厘米
     */
    void AvoidObstacleWithServo(float safeDistance = 32.0);
    
    /**
     * @brief 使用舵机进行动态避障
     * @param safeDistance 安全距离，单位为厘米
     * @param spinTimeLeftDegree90 左转90度所需时间(单位:100ms)
     * @param spinTimeRightDegree90 右转90度所需时间(单位:100ms)
     */
    void DynamicAvoidObstacle(float safeDistance = 30.0, 
                             float spinTimeLeftDegree90 = 3.5, 
                             float spinTimeRightDegree90 = 3.5);
    
    /**
     * @brief 检测前方距离
     * @return 前方距离(厘米)
     */
    float DetectFront();
    
    /**
     * @brief 检测左侧距离
     * @return 左侧距离(厘米)
     */
    float DetectLeft();
    
    /**
     * @brief 检测右侧距离
     * @return 右侧距离(厘米)
     */
    float DetectRight();
    
    /**
     * @brief 自动跟随
     * @param targetDistance 目标跟随距离(厘米)
     * @param tolerance 允许误差范围(厘米)
     * @param maxSpeed 最大速度(0-255)
     * @param minSpeed 最小速度(0-255)
     */
    void AutoFollow(float targetDistance = 25.0, float tolerance = 3.0, 
                   int maxSpeed = 220, int minSpeed = 180);
                   
    /**
     * @brief 执行花式动作：全速前进急停后退
     */
    void PerformFastForwardBackward();
    
    /**
     * @brief 执行花式动作：间断性前进
     * @param steps 步数
     * @param moveTime 每步移动时间(单位:100ms)
     * @param stopTime 每步停止时间(单位:100ms)
     */
    void PerformIntermittentForward(int steps = 5, int moveTime = 10, int stopTime = 1);
    
    /**
     * @brief 执行花式动作：间断性后退
     * @param steps 步数
     * @param moveTime 每步移动时间(单位:100ms)
     * @param stopTime 每步停止时间(单位:100ms)
     */
    void PerformIntermittentBackward(int steps = 5, int moveTime = 10, int stopTime = 1);
    
    /**
     * @brief 执行花式动作：大弯套小弯连续左旋转
     * @param cycles 循环次数
     */
    void PerformLeftSpinCombination(int cycles = 5);
    
    /**
     * @brief 执行花式动作：大弯套小弯连续右旋转
     * @param cycles 循环次数
     */
    void PerformRightSpinCombination(int cycles = 5);
    
    /**
     * @brief 执行花式动作：间断性原地左/右转弯
     * @param direction 方向，0为左，1为右
     * @param cycles 循环次数
     */
    void PerformIntermittentTurn(int direction = 0, int cycles = 10);
    
    /**
     * @brief 执行花式动作：走S形前进
     * @param cycles 循环次数
     */
    void PerformSShapedMovement(int cycles = 10);
    
    /**
     * @brief 执行花式动作：间断性原地左/右打转
     * @param direction 方向，0为左，1为右
     * @param cycles 循环次数
     */
    void PerformIntermittentSpin(int direction = 0, int cycles = 10);
    
    /**
     * @brief 执行完整的花式动作表演
     */
    void PerformColorfulMovements();
    
private:
    /**
     * @brief 向左动态绕障
     * @param safeDistance 安全距离，单位为厘米
     * @param spinTimeLeftDegree90 左转90度所需时间(单位:100ms)
     * @param spinTimeRightDegree90 右转90度所需时间(单位:100ms)
     * @return 是否成功绕过障碍物
     */
    bool DynamicAvoidLeft(float safeDistance, float spinTimeLeftDegree90, float spinTimeRightDegree90);
    
    /**
     * @brief 向右动态绕障
     * @param safeDistance 安全距离，单位为厘米
     * @param spinTimeLeftDegree90 左转90度所需时间(单位:100ms)
     * @param spinTimeRightDegree90 右转90度所需时间(单位:100ms)
     * @return 是否成功绕过障碍物
     */
    bool DynamicAvoidRight(float safeDistance, float spinTimeLeftDegree90, float spinTimeRightDegree90);
    
    /**
     * @brief 计算自动跟随的电机速度
     * @param error 误差值(厘米)
     * @param minSpeed 最小速度
     * @param maxSpeed 最大速度
     * @return 计算后的速度值
     */
    int CalculateFollowSpeed(float error, int minSpeed, int maxSpeed);
    
    Motor* m_motor;                      ///< 电机控制对象
    UltrasonicSensor* m_ultrasonicSensor;  ///< 超声波传感器对象
    InfraredSensor* m_infraredSensor;     ///< 红外传感器对象
    TrackingSensor* m_trackingSensor;     ///< 循迹传感器对象
    Servo* m_servo;                      ///< 舵机控制对象
    
    bool m_hasUltrasonicSensor;          ///< 是否配置了超声波传感器
    bool m_hasInfraredSensor;            ///< 是否配置了红外传感器
    bool m_hasTrackingSensor;            ///< 是否配置了循迹传感器
    bool m_hasServo;                     ///< 是否配置了舵机
    
    MoveState m_currentState;            ///< 当前运动状态
};

#endif // SMART_CAR_H