/**
 * @file Servo.h
 * @brief 舵机控制类的头文件
 * @details 提供舵机控制相关功能
 */

#ifndef SERVO_H
#define SERVO_H

#include <Arduino.h>

/**
 * @class Servo
 * @brief 舵机控制类
 * @details 负责舵机的控制和角度调整
 */
class Servo {
public:
    /**
     * @brief 构造函数
     * @param servoPin 舵机控制引脚
     */
    Servo(uint8_t servoPin);
    
    /**
     * @brief 初始化舵机
     */
    void begin();
    
    /**
     * @brief 设置舵机角度
     * @param angle 舵机角度(0-180)
     * @param pulseCount 产生PWM脉冲的次数
     */
    void setAngle(uint8_t angle, uint8_t pulseCount = 10);
    
private:
    uint8_t _servoPin;  ///< 舵机控制引脚
    
    /**
     * @brief 产生单个PWM脉冲
     * @param angle 舵机角度(0-180)
     */
    void servoPulse(uint8_t angle);
};

#endif // SERVO_H