/**
 * @file Motor.cpp
 * @brief 电机控制类的实现文件
 */

#include "Motor.h"

Motor::Motor(uint8_t leftMotorGo, uint8_t leftMotorBack, uint8_t rightMotorGo, uint8_t rightMotorBack)
    : _leftMotorGo(leftMotorGo), _leftMotorBack(leftMotorBack),
      _rightMotorGo(rightMotorGo), _rightMotorBack(rightMotorBack),
      _leftSpeed(200), _rightSpeed(200) {
}

void Motor::begin() {
    // 初始化电机引脚为输出模式
    pinMode(_leftMotorGo, OUTPUT);
    pinMode(_leftMotorBack, OUTPUT);
    pinMode(_rightMotorGo, OUTPUT);
    pinMode(_rightMotorBack, OUTPUT);
    
    // 初始状态为停止
    stop(0);
}

void Motor::setSpeed(uint8_t leftSpeed, uint8_t rightSpeed) {
    _leftSpeed = leftSpeed;
    _rightSpeed = rightSpeed;
}

void Motor::forward(uint16_t time) {
    // 右电机前进
    digitalWrite(_rightMotorBack, LOW);
    analogWrite(_rightMotorGo, _rightSpeed);
    
    // 左电机前进
    digitalWrite(_leftMotorBack, LOW);
    analogWrite(_leftMotorGo, _leftSpeed);
    
    // 如果指定了时间，则延时后停止
    if (time > 0) {
        delay(time * 100);
        stop(0);
    }
}

void Motor::backward(uint16_t time) {
    // 右电机后退
    digitalWrite(_rightMotorGo, LOW);
    analogWrite(_rightMotorBack, _rightSpeed);
    
    // 左电机后退
    digitalWrite(_leftMotorGo, LOW);
    analogWrite(_leftMotorBack, _leftSpeed);
    
    // 如果指定了时间，则延时后停止
    if (time > 0) {
        delay(time * 100);
        stop(0);
    }
}

void Motor::turnLeft(uint16_t time) {
    // 右电机前进
    digitalWrite(_rightMotorBack, LOW);
    analogWrite(_rightMotorGo, _rightSpeed);
    
    // 左电机停止
    digitalWrite(_leftMotorGo, LOW);
    digitalWrite(_leftMotorBack, LOW);
    
    // 如果指定了时间，则延时后停止
    if (time > 0) {
        delay(time * 100);
        stop(0);
    }
}

void Motor::turnRight(uint16_t time) {
    // 右电机停止
    digitalWrite(_rightMotorGo, LOW);
    digitalWrite(_rightMotorBack, LOW);
    
    // 左电机前进
    digitalWrite(_leftMotorBack, LOW);
    analogWrite(_leftMotorGo, _leftSpeed);
    
    // 如果指定了时间，则延时后停止
    if (time > 0) {
        delay(time * 100);
        stop(0);
    }
}

void Motor::spinLeft(uint16_t time) {
    // 右电机前进
    digitalWrite(_rightMotorBack, LOW);
    analogWrite(_rightMotorGo, _rightSpeed);
    
    // 左电机后退
    digitalWrite(_leftMotorGo, LOW);
    analogWrite(_leftMotorBack, _leftSpeed);
    
    // 如果指定了时间，则延时后停止
    if (time > 0) {
        delay(time * 100);
        stop(0);
    }
}

void Motor::spinRight(uint16_t time) {
    // 右电机后退
    digitalWrite(_rightMotorGo, LOW);
    analogWrite(_rightMotorBack, _rightSpeed);
    
    // 左电机前进
    digitalWrite(_leftMotorBack, LOW);
    analogWrite(_leftMotorGo, _leftSpeed);
    
    // 如果指定了时间，则延时后停止
    if (time > 0) {
        delay(time * 100);
        stop(0);
    }
}

void Motor::stop(uint16_t time) {
    // 停止所有电机
    digitalWrite(_rightMotorGo, LOW);
    digitalWrite(_rightMotorBack, LOW);
    digitalWrite(_leftMotorGo, LOW);
    digitalWrite(_leftMotorBack, LOW);
    
    // 如果指定了时间，则延时指定时间
    if (time > 0) {
        delay(time * 100);
    }
}