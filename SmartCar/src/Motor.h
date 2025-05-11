/**
 * @file motor.h
 * @brief 电机控制类的头文件
 * @details 提供电机控制相关功能，包括前进、后退、转向等基本动作
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

/**
 * @class Motor
 * @brief 电机控制类
 * @details 负责控制小车的电机，实现基本的运动功能
 */
class Motor {
public:
    /**
     * @brief 构造函数
     * @param leftMotorGo 左电机前进引脚
     * @param leftMotorBack 左电机后退引脚
     * @param rightMotorGo 右电机前进引脚
     * @param rightMotorBack 右电机后退引脚
     */
    Motor(uint8_t leftMotorGo, uint8_t leftMotorBack, uint8_t rightMotorGo, uint8_t rightMotorBack);
    
    /**
     * @brief 初始化电机
     */
    void Begin();
    
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
     * @brief 左转(左轮不动，右轮前进)
     * @param time 左转时间(单位:100ms)，0表示持续运行
     */
    void TurnLeft(uint16_t time = 0);
    
    /**
     * @brief 右转(右轮不动，左轮前进)
     * @param time 右转时间(单位:100ms)，0表示持续运行
     */
    void TurnRight(uint16_t time = 0);
    
    /**
     * @brief 左旋转(左轮后退，右轮前进)
     * @param time 左旋转时间(单位:100ms)，0表示持续运行
     */
    void SpinLeft(uint16_t time = 0);
    
    /**
     * @brief 右旋转(右轮后退，左轮前进)
     * @param time 右旋转时间(单位:100ms)，0表示持续运行
     */
    void SpinRight(uint16_t time = 0);
    
    /**
     * @brief 停止
     * @param time 停止时间(单位:100ms)，0表示持续停止
     */
    void Stop(uint16_t time = 0);
    
private:
    uint8_t m_leftMotorGo;    ///< 左电机前进引脚
    uint8_t m_leftMotorBack;  ///< 左电机后退引脚
    uint8_t m_rightMotorGo;   ///< 右电机前进引脚
    uint8_t m_rightMotorBack; ///< 右电机后退引脚
    uint8_t m_leftSpeed;      ///< 左电机速度
    uint8_t m_rightSpeed;     ///< 右电机速度
};

#endif // MOTOR_H