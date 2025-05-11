/**
 * @file SmartCar.cpp
 * @brief 智能小车主类的实现文件
 */

#include "SmartCar.h"

SmartCar::SmartCar(uint8_t leftMotorGo, uint8_t leftMotorBack, 
                   uint8_t rightMotorGo, uint8_t rightMotorBack)
    : _hasUltrasonicSensor(false), _hasInfraredSensor(false), _hasServo(false) {
    // 创建电机控制对象
    _motor = new Motor(leftMotorGo, leftMotorBack, rightMotorGo, rightMotorBack);
}

void SmartCar::begin() {
    // 初始化电机
    _motor->begin();
    
    // 如果配置了超声波传感器，则初始化
    if (_hasUltrasonicSensor) {
        _ultrasonicSensor->begin();
    }
    
    // 如果配置了红外传感器，则初始化
    if (_hasInfraredSensor) {
        _infraredSensor->begin();
    }
    
    // 如果配置了舵机，则初始化
    if (_hasServo) {
        _servo->begin();
    }
}

void SmartCar::setupUltrasonicSensor(uint8_t trigPin, uint8_t echoPin) {
    // 创建超声波传感器对象
    _ultrasonicSensor = new UltrasonicSensor(trigPin, echoPin);
    _hasUltrasonicSensor = true;
}

void SmartCar::setupInfraredSensor(uint8_t leftPin, uint8_t rightPin) {
    // 创建红外传感器对象
    _infraredSensor = new InfraredSensor(leftPin, rightPin);
    _hasInfraredSensor = true;
}

void SmartCar::setSpeed(uint8_t leftSpeed, uint8_t rightSpeed) {
    // 设置电机速度
    _motor->setSpeed(leftSpeed, rightSpeed);
}

void SmartCar::forward(uint16_t time) {
    // 前进
    _motor->forward(time);
}

void SmartCar::backward(uint16_t time) {
    // 后退
    _motor->backward(time);
}

void SmartCar::turnLeft(uint16_t time) {
    // 左转
    _motor->turnLeft(time);
}

void SmartCar::turnRight(uint16_t time) {
    // 右转
    _motor->turnRight(time);
}

void SmartCar::spinLeft(uint16_t time) {
    // 左旋转
    _motor->spinLeft(time);
}

void SmartCar::spinRight(uint16_t time) {
    // 右旋转
    _motor->spinRight(time);
}

void SmartCar::stop(uint16_t time) {
    // 停止
    _motor->stop(time);
}

float SmartCar::getDistance() {
    // 如果没有配置超声波传感器，则返回-1
    if (!_hasUltrasonicSensor) {
        return -1.0;
    }
    
    // 获取距离
    return _ultrasonicSensor->getDistance();
}

int SmartCar::trackLine() {
    // 如果没有配置红外传感器，则返回-1
    if (!_hasInfraredSensor) {
        return -1;
    }
    
    // 根据红外传感器状态执行循迹
    if (_infraredSensor->isBothOnLine()) {
        // 两侧都检测到黑线，直行
        forward(0);
        return 0;
    } else if (_infraredSensor->isLeftOnLine()) {
        // 左侧检测到黑线，右侧检测到白色区域，向右偏离，需要左转
        turnLeft(0);
        return 1;
    } else if (_infraredSensor->isRightOnLine()) {
        // 右侧检测到黑线，左侧检测到白色区域，向左偏离，需要右转
        turnRight(0);
        return 2;
    } else {
        // 两侧都检测到白色区域，停止
        stop(0);
        return 3;
    }
}

void SmartCar::setupServo(uint8_t servoPin) {
    // 创建舵机控制对象
    _servo = new Servo(servoPin);
    _hasServo = true;
}

float SmartCar::detectFront() {
    // 如果没有配置舵机或超声波传感器，则直接返回-1
    if (!_hasServo || !_hasUltrasonicSensor) {
        return -1.0;
    }
    
    // 将舵机转到90度(前方)
    _servo->setAngle(90, 5);
    
    // 获取距离
    return getDistance();
}

float SmartCar::detectLeft() {
    // 如果没有配置舵机或超声波传感器，则直接返回-1
    if (!_hasServo || !_hasUltrasonicSensor) {
        return -1.0;
    }
    
    // 将舵机转到175度(左侧)
    _servo->setAngle(175, 15);
    
    // 获取距离
    return getDistance();
}

float SmartCar::detectRight() {
    // 如果没有配置舵机或超声波传感器，则直接返回-1
    if (!_hasServo || !_hasUltrasonicSensor) {
        return -1.0;
    }
    
    // 将舵机转到0度(右侧)
    _servo->setAngle(0, 15);
    
    // 获取距离
    return getDistance();
}

void SmartCar::avoidObstacle(float safeDistance) {
    // 如果配置了舵机，则使用多方向避障
    if (_hasServo) {
        avoidObstacleWithServo(safeDistance);
        return;
    }
    
    // 如果没有配置超声波传感器，则直接返回
    if (!_hasUltrasonicSensor) {
        return;
    }
    
    // 获取前方距离
    float distance = getDistance();
    
    // 判断是否在有效测量范围内
    if (_ultrasonicSensor->isValidRange(distance)) {
        // 如果距离小于安全距离，则停止并后退
        if (distance < safeDistance) {
            stop(0);
            backward(5);
            
            // 随机选择左转或右转
            if (random(2) == 0) {
                spinLeft(10);
            } else {
                spinRight(10);
            }
        } else {
            // 如果距离大于安全距离，则继续前进
            forward(0);
        }
    } else {
        // 如果不在有效测量范围内，则停止
        stop(0);
    }
}

void SmartCar::avoidObstacleWithServo(float safeDistance) {
    // 如果没有配置舵机或超声波传感器，则直接返回
    if (!_hasServo || !_hasUltrasonicSensor) {
        return;
    }
    
    // 检测前方距离
    float frontDistance = detectFront();
    
    // 判断是否在有效测量范围内
    if (_ultrasonicSensor->isValidRange(frontDistance)) {
        // 如果前方距离小于安全距离，则执行避障
        if (frontDistance < safeDistance) {
            // 后退减速
            backward(2);
            stop(2);
            
            // 检测左右两侧距离
            float leftDistance = detectLeft();
            
            float rightDistance = detectRight();
            
            // 根据左右两侧距离决定转向
            if ((leftDistance < 35) && (rightDistance < 35)) {
                // 左右两侧都有障碍物，掉头
                spinLeft(7);
            } else if (leftDistance > rightDistance) {
                // 左侧空间更大，向左转
                turnLeft(3);
                stop(1);
            } else {
                // 右侧空间更大，向右转
                turnRight(3);
                stop(1);
            }
        } else {
            // 如果前方距离大于安全距离，则继续前进
            forward(0);
        }
    } else {
        // 如果不在有效测量范围内，则停止
        stop(0);
    }
}
