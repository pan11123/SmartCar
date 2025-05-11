/**
 * @file ultrasonic_sensor.cpp
 * @brief 超声波传感器类的实现文件
 */

#include "ultrasonic_sensor.h"

UltrasonicSensor::UltrasonicSensor(uint8_t trigPin, uint8_t echoPin)
    : m_trigPin(trigPin), m_echoPin(echoPin), m_minRange(2.0), m_maxRange(400.0) {
}

void UltrasonicSensor::Begin() {
    // 初始化超声波传感器引脚
    pinMode(m_trigPin, OUTPUT);
    pinMode(m_echoPin, INPUT);
}

float UltrasonicSensor::GetDistance() {
    // 完全复制ultrasonic.ino中的Distance_test函数实现
    digitalWrite(m_trigPin, LOW);   // 给触发脚低电平2μs
    delayMicroseconds(2);
    digitalWrite(m_trigPin, HIGH);  // 给触发脚高电平10μs，这里至少是10μs
    delayMicroseconds(10);
    digitalWrite(m_trigPin, LOW);    // 持续给触发脚低电
    float distance = pulseIn(m_echoPin, HIGH);  // 读取高电平时间(单位：微秒)
    distance = distance/58;       //为什么除以58等于厘米，  Y米=（X秒*344）/2
    // X秒=（ 2*Y米）/344 ==》X秒=0.0058*Y米 ==》厘米=微秒/58
    Serial.print("Distance:");      //输出距离（单位：厘米）
    Serial.println(distance);      //显示距离  
    return distance;
}

bool UltrasonicSensor::IsValidRange(float distance) {
    // 判断距离是否在有效范围内
    return (distance > m_minRange) && (distance < m_maxRange);
}