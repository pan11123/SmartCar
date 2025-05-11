/**
 * @file UltrasonicSensor.cpp
 * @brief 超声波传感器类的实现文件
 */

#include "UltrasonicSensor.h"

UltrasonicSensor::UltrasonicSensor(uint8_t trigPin, uint8_t echoPin)
    : _trigPin(trigPin), _echoPin(echoPin), _minRange(2.0), _maxRange(400.0) {
}

void UltrasonicSensor::begin() {
    // 初始化超声波传感器引脚
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
}

float UltrasonicSensor::getDistance() {
    // 完全复制ultrasonic.ino中的Distance_test函数实现
    digitalWrite(_trigPin, LOW);   // 给触发脚低电平2μs
    delayMicroseconds(2);
    digitalWrite(_trigPin, HIGH);  // 给触发脚高电平10μs，这里至少是10μs
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);    // 持续给触发脚低电
    float Fdistance = pulseIn(_echoPin, HIGH);  // 读取高电平时间(单位：微秒)
    Fdistance= Fdistance/58;       //为什么除以58等于厘米，  Y米=（X秒*344）/2
    // X秒=（ 2*Y米）/344 ==》X秒=0.0058*Y米 ==》厘米=微秒/58
    Serial.print("Distance:");      //输出距离（单位：厘米）
    Serial.println(Fdistance);      //显示距离  
    return Fdistance;
}

bool UltrasonicSensor::isValidRange(float distance) {
    // 判断距离是否在有效范围内
    return (distance > _minRange) && (distance < _maxRange);
}