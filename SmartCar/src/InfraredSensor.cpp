/**
 * @file InfraredSensor.cpp
 * @brief 红外传感器类的实现文件
 */

#include "InfraredSensor.h"

InfraredSensor::InfraredSensor(uint8_t leftPin, uint8_t rightPin)
    : _leftPin(leftPin), _rightPin(rightPin) {
}

void InfraredSensor::begin() {
    // 初始化红外传感器引脚为输入模式
    pinMode(_leftPin, INPUT);
    pinMode(_rightPin, INPUT);
}

int InfraredSensor::getLeftSensorStatus() {
    // 读取左侧传感器状态
    return digitalRead(_leftPin);
}

int InfraredSensor::getRightSensorStatus() {
    // 读取右侧传感器状态
    return digitalRead(_rightPin);
}

bool InfraredSensor::isBothOnLine() {
    // 判断两侧是否都检测到黑线
    // LOW表示检测到黑线
    return (getLeftSensorStatus() == LOW && getRightSensorStatus() == LOW);
}

bool InfraredSensor::isBothOffLine() {
    // 判断两侧是否都检测到白色区域
    // HIGH表示检测到白色区域
    return (getLeftSensorStatus() == HIGH && getRightSensorStatus() == HIGH);
}

bool InfraredSensor::isLeftOnLine() {
    // 判断左侧是否检测到黑线，右侧检测到白色区域
    return (getLeftSensorStatus() == LOW && getRightSensorStatus() == HIGH);
}

bool InfraredSensor::isRightOnLine() {
    // 判断右侧是否检测到黑线，左侧检测到白色区域
    return (getLeftSensorStatus() == HIGH && getRightSensorStatus() == LOW);
}