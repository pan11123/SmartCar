/**
 * @file InfraredSensor.h
 * @brief 红外传感器类的头文件
 * @details 提供红外循迹相关功能
 */

#ifndef INFRARED_SENSOR_H
#define INFRARED_SENSOR_H

#include <Arduino.h>

/**
 * @class InfraredSensor
 * @brief 红外传感器类
 * @details 负责红外传感器的控制和黑线循迹
 */
class InfraredSensor {
public:
    /**
     * @brief 构造函数
     * @param leftPin 左侧红外传感器引脚
     * @param rightPin 右侧红外传感器引脚
     */
    InfraredSensor(uint8_t leftPin, uint8_t rightPin);
    
    /**
     * @brief 初始化红外传感器
     */
    void begin();
    
    /**
     * @brief 获取左侧传感器状态
     * @return 返回传感器状态，LOW表示检测到黑线，HIGH表示检测到白色区域
     */
    int getLeftSensorStatus();
    
    /**
     * @brief 获取右侧传感器状态
     * @return 返回传感器状态，LOW表示检测到黑线，HIGH表示检测到白色区域
     */
    int getRightSensorStatus();
    
    /**
     * @brief 判断是否两侧都检测到黑线
     * @return 如果两侧都检测到黑线返回true，否则返回false
     */
    bool isBothOnLine();
    
    /**
     * @brief 判断是否两侧都检测到白色区域
     * @return 如果两侧都检测到白色区域返回true，否则返回false
     */
    bool isBothOffLine();
    
    /**
     * @brief 判断是否左侧检测到黑线，右侧检测到白色区域
     * @return 如果左侧检测到黑线，右侧检测到白色区域返回true，否则返回false
     */
    bool isLeftOnLine();
    
    /**
     * @brief 判断是否右侧检测到黑线，左侧检测到白色区域
     * @return 如果右侧检测到黑线，左侧检测到白色区域返回true，否则返回false
     */
    bool isRightOnLine();
    
private:
    uint8_t _leftPin;  ///< 左侧红外传感器引脚
    uint8_t _rightPin; ///< 右侧红外传感器引脚
};

#endif // INFRARED_SENSOR_H