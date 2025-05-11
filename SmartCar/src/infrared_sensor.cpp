/**
 * @file infrared_sensor.cpp
 * @brief 红外传感器类的实现文件
 */

#include "infrared_sensor.h"

InfraredSensor::InfraredSensor(uint8_t leftPin, uint8_t rightPin)
    : m_leftPin(leftPin), m_rightPin(rightPin) {
}

void InfraredSensor::Begin() {
    // 初始化红外传感器引脚为输入模式
    pinMode(m_leftPin, INPUT);
    pinMode(m_rightPin, INPUT);
}

int InfraredSensor::GetLeftSensorStatus() {
    // 读取左侧传感器状态
    return digitalRead(m_leftPin);
}

int InfraredSensor::GetRightSensorStatus() {
    // 读取右侧传感器状态
    return digitalRead(m_rightPin);
}

bool InfraredSensor::IsBothOnLine() {
    // 判断两侧是否都检测到黑线
    // LOW表示检测到黑线
    return (GetLeftSensorStatus() == LOW && GetRightSensorStatus() == LOW);
}

bool InfraredSensor::IsBothOffLine() {
    // 判断两侧是否都检测到白色区域
    // HIGH表示检测到白色区域
    return (GetLeftSensorStatus() == HIGH && GetRightSensorStatus() == HIGH);
}

bool InfraredSensor::IsLeftOnLine() {
    // 判断左侧是否检测到黑线，右侧检测到白色区域
    return (GetLeftSensorStatus() == LOW && GetRightSensorStatus() == HIGH);
}

bool InfraredSensor::IsRightOnLine() {
    // 判断右侧是否检测到黑线，左侧检测到白色区域
    return (GetLeftSensorStatus() == HIGH && GetRightSensorStatus() == LOW);
}