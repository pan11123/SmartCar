/**
 * @file TrackingSensor.h
 * @brief 循迹传感器类的头文件
 * @details 提供了获取循迹传感器状态的接口
 */

#ifndef TRACKING_SENSOR_H
#define TRACKING_SENSOR_H

#include <Arduino.h>

/**
 * @class TrackingSensor
 * @brief 循迹传感器类
 * @details 用于黑线循迹功能的传感器封装
 */
class TrackingSensor {
public:
    /**
     * @brief 构造函数
     */
    TrackingSensor();
    
    /**
     * @brief 初始化循迹传感器
     * @param leftPin 左循迹传感器引脚
     * @param rightPin 右循迹传感器引脚
     */
    void begin(uint8_t leftPin, uint8_t rightPin);
    
    /**
     * @brief 读取左侧循迹传感器状态
     * @return 传感器读数，LOW表示检测到黑线，HIGH表示检测到白色
     */
    bool readLeft();
    
    /**
     * @brief 读取右侧循迹传感器状态
     * @return 传感器读数，LOW表示检测到黑线，HIGH表示检测到白色
     */
    bool readRight();
    
    /**
     * @brief 获取循迹状态
     * @return 循迹状态，0表示直行(两侧都是黑线)，1表示左转(左白右黑)，2表示右转(左黑右白)，3表示停止(两侧都是白色)
     */
    int getTrackingState();
    
private:
    uint8_t _leftPin;     ///< 左循迹传感器引脚
    uint8_t _rightPin;    ///< 右循迹传感器引脚
    bool _isInitialized;  ///< 是否已初始化
};

#endif // TRACKING_SENSOR_H