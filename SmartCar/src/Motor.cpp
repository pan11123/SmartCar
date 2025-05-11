/**
 * @file motor.cpp
 * @brief 电机控制类的实现文件
 */

#include "motor.h"

Motor::Motor(uint8_t leftMotorGo, uint8_t leftMotorBack, uint8_t rightMotorGo, uint8_t rightMotorBack)
    : m_leftMotorGo(leftMotorGo), m_leftMotorBack(leftMotorBack),
      m_rightMotorGo(rightMotorGo), m_rightMotorBack(rightMotorBack),
      m_leftSpeed(200), m_rightSpeed(200) {
}

void Motor::Begin() {
    // 初始化电机引脚为输出模式
    pinMode(m_leftMotorGo, OUTPUT);
    pinMode(m_leftMotorBack, OUTPUT);
    pinMode(m_rightMotorGo, OUTPUT);
    pinMode(m_rightMotorBack, OUTPUT);
    
    // 初始状态为停止
    Stop(0);
}

void Motor::SetSpeed(uint8_t leftSpeed, uint8_t rightSpeed) {
    m_leftSpeed = leftSpeed;
    m_rightSpeed = rightSpeed;
}

void Motor::Forward(uint16_t time) {
    // 右电机前进
    digitalWrite(m_rightMotorBack, LOW);
    analogWrite(m_rightMotorGo, m_rightSpeed);
    
    // 左电机前进
    digitalWrite(m_leftMotorBack, LOW);
    analogWrite(m_leftMotorGo, m_leftSpeed);
    
    // 如果指定了时间，则延时后停止
    if (time > 0) {
        delay(time * 100);
        Stop(0);
    }
}

void Motor::Backward(uint16_t time) {
    // 右电机后退
    digitalWrite(m_rightMotorGo, LOW);
    analogWrite(m_rightMotorBack, m_rightSpeed);
    
    // 左电机后退
    digitalWrite(m_leftMotorGo, LOW);
    analogWrite(m_leftMotorBack, m_leftSpeed);
    
    // 如果指定了时间，则延时后停止
    if (time > 0) {
        delay(time * 100);
        Stop(0);
    }
}

void Motor::TurnLeft(uint16_t time) {
    // 右电机前进
    digitalWrite(m_rightMotorBack, LOW);
    analogWrite(m_rightMotorGo, m_rightSpeed);
    
    // 左电机停止
    digitalWrite(m_leftMotorGo, LOW);
    digitalWrite(m_leftMotorBack, LOW);
    
    // 如果指定了时间，则延时后停止
    if (time > 0) {
        delay(time * 100);
        Stop(0);
    }
}

void Motor::TurnRight(uint16_t time) {
    // 右电机停止
    digitalWrite(m_rightMotorGo, LOW);
    digitalWrite(m_rightMotorBack, LOW);
    
    // 左电机前进
    digitalWrite(m_leftMotorBack, LOW);
    analogWrite(m_leftMotorGo, m_leftSpeed);
    
    // 如果指定了时间，则延时后停止
    if (time > 0) {
        delay(time * 100);
        Stop(0);
    }
}

void Motor::SpinLeft(uint16_t time) {
    // 右电机前进
    digitalWrite(m_rightMotorBack, LOW);
    analogWrite(m_rightMotorGo, m_rightSpeed);
    
    // 左电机后退
    digitalWrite(m_leftMotorGo, LOW);
    analogWrite(m_leftMotorBack, m_leftSpeed);
    
    // 如果指定了时间，则延时后停止
    if (time > 0) {
        delay(time * 100);
        Stop(0);
    }
}

void Motor::SpinRight(uint16_t time) {
    // 右电机后退
    digitalWrite(m_rightMotorGo, LOW);
    analogWrite(m_rightMotorBack, m_rightSpeed);
    
    // 左电机前进
    digitalWrite(m_leftMotorBack, LOW);
    analogWrite(m_leftMotorGo, m_leftSpeed);
    
    // 如果指定了时间，则延时后停止
    if (time > 0) {
        delay(time * 100);
        Stop(0);
    }
}

void Motor::Stop(uint16_t time) {
    // 停止所有电机
    digitalWrite(m_rightMotorGo, LOW);
    digitalWrite(m_rightMotorBack, LOW);
    digitalWrite(m_leftMotorGo, LOW);
    digitalWrite(m_leftMotorBack, LOW);
    
    // 如果指定了时间，则延时指定时间
    if (time > 0) {
        delay(time * 100);
    }
}