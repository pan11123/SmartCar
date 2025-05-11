/**
 * @file UltrasonicSensor.h
 * @brief 超声波传感器类的头文件
 * @details 提供超声波测距相关功能
 */

#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

// 取消注释下面这行可以启用超声波传感器的调试输出
#define DEBUG_ULTRASONIC

/**
 * @class UltrasonicSensor
 * @brief 超声波传感器类
 * @details 负责超声波传感器的控制和距离测量
 */
class UltrasonicSensor {
public:
    /**
     * @brief 构造函数
     * @param trigPin 触发引脚
     * @param echoPin 回声引脚
     */
    UltrasonicSensor(uint8_t trigPin, uint8_t echoPin);
    
    /**
     * @brief 初始化超声波传感器
     */
    void begin();
    
    /**
     * @brief 测量距离
     * @return 返回测量的距离，单位为厘米
     */
    float getDistance();
    
    /**
     * @brief 判断是否在有效测量范围内
     * @param distance 距离值
     * @return 如果在有效范围内返回true，否则返回false
     */
    bool isValidRange(float distance);
    
private:
    uint8_t _trigPin; ///< 触发引脚
    uint8_t _echoPin; ///< 回声引脚
    float _minRange;  ///< 最小有效测量范围(厘米)
    float _maxRange;  ///< 最大有效测量范围(厘米)
};

#endif // ULTRASONIC_SENSOR_H