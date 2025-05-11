 /**
 * @file TrackingSensor.cpp
 * @brief 循迹传感器类的实现文件
 */

#include "TrackingSensor.h"

/**
 * @brief 构造函数
 */
TrackingSensor::TrackingSensor() {
    _isInitialized = false;
}

/**
 * @brief 初始化循迹传感器
 * @param leftPin 左循迹传感器引脚
 * @param rightPin 右循迹传感器引脚
 */
void TrackingSensor::begin(uint8_t leftPin, uint8_t rightPin) {
    _leftPin = leftPin;
    _rightPin = rightPin;
    
    // 设置引脚为输入模式
    pinMode(_leftPin, INPUT);
    pinMode(_rightPin, INPUT);
    
    _isInitialized = true;
}

/**
 * @brief 读取左侧循迹传感器状态
 * @return 传感器读数，LOW表示检测到黑线，HIGH表示检测到白色
 */
bool TrackingSensor::readLeft() {
    if (!_isInitialized) {
        return HIGH; // 如果未初始化，返回无黑线的状态
    }
    
    return digitalRead(_leftPin);
}

/**
 * @brief 读取右侧循迹传感器状态
 * @return 传感器读数，LOW表示检测到黑线，HIGH表示检测到白色
 */
bool TrackingSensor::readRight() {
    if (!_isInitialized) {
        return HIGH; // 如果未初始化，返回无黑线的状态
    }
    
    return digitalRead(_rightPin);
}

/**
 * @brief 获取循迹状态
 * @return 循迹状态，0表示直行，1表示左转，2表示右转，3表示停止
 */
int TrackingSensor::getTrackingState() {
    if (!_isInitialized) {
        return 3; // 如果未初始化，返回停止状态
    }
    
    bool leftSensor = readLeft();
    bool rightSensor = readRight();
    
    // 状态判断
    if (leftSensor == LOW && rightSensor == LOW) {
        // 两个传感器都在黑线上，直行
        return 0;
    } else if (leftSensor == HIGH && rightSensor == LOW) {
        // 左传感器在白色区域，右传感器在黑线上，向左转
        return 1;
    } else if (leftSensor == LOW && rightSensor == HIGH) {
        // 左传感器在黑线上，右传感器在白色区域，向右转
        return 2;
    } else {
        // 两个传感器都在白色区域，停止
        return 3;
    }
} 