/**
 * @file servo.cpp
 * @brief 舵机控制类的实现文件
 */

#include "servo.h"

Servo::Servo(uint8_t servoPin) : m_servoPin(servoPin) {
}

void Servo::Begin() {
    // 初始化舵机引脚为输出模式
    pinMode(m_servoPin, OUTPUT);
}

void Servo::SetAngle(uint8_t angle, uint8_t pulseCount) {
    // 限制角度范围在0-180之间
    if (angle > 180) angle = 180;
    
    // 产生多个PWM脉冲，确保舵机转到指定角度
    for (int i = 0; i < pulseCount; i++) {
        ServoPulse(angle);
    }
}

void Servo::ServoPulse(uint8_t angle) {
    // 将角度转换为脉宽值(500-2480微秒)
    int pulseWidth = (angle * 11) + 500;
    
    // 产生PWM信号
    digitalWrite(m_servoPin, HIGH);       // 将舵机接口电平置高
    delayMicroseconds(pulseWidth);       // 延时脉宽值的微秒数
    digitalWrite(m_servoPin, LOW);        // 将舵机接口电平置低
    delay(20 - pulseWidth / 1000);       // 延时周期内剩余时间
}