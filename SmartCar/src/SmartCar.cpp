/**
 * @file SmartCar.cpp
 * @brief 智能小车主类的实现文件
 */

#include "SmartCar.h"

SmartCar::SmartCar(uint8_t leftMotorGo, uint8_t leftMotorBack, 
                   uint8_t rightMotorGo, uint8_t rightMotorBack)
    : _hasUltrasonicSensor(false), _hasInfraredSensor(false), _hasTrackingSensor(false), _hasServo(false) {
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
    
    // 如果配置了循迹传感器，则初始化
    if (_hasTrackingSensor) {
        // 循迹传感器已在setupTrackingSensors中初始化
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

void SmartCar::setupTrackingSensors(uint8_t leftPin, uint8_t rightPin) {
    // 创建循迹传感器对象
    _trackingSensor = new TrackingSensor();
    _trackingSensor->begin(leftPin, rightPin);
    _hasTrackingSensor = true;
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

bool SmartCar::readLeftTrackSensor() {
    // 如果没有配置循迹传感器，则返回HIGH（表示未检测到黑线）
    if (!_hasTrackingSensor) {
        return HIGH;
    }
    
    // 读取左侧循迹传感器状态
    return _trackingSensor->readLeft();
}

bool SmartCar::readRightTrackSensor() {
    // 如果没有配置循迹传感器，则返回HIGH（表示未检测到黑线）
    if (!_hasTrackingSensor) {
        return HIGH;
    }
    
    // 读取右侧循迹传感器状态
    return _trackingSensor->readRight();
}

int SmartCar::trackLine() {
    // 先检查是否使用专用循迹传感器
    if (_hasTrackingSensor) {
        // 读取循迹传感器状态
        bool leftSensor = readLeftTrackSensor();
        bool rightSensor = readRightTrackSensor();
        
        // 根据传感器状态进行控制
        // 有信号为LOW (黑线), 无信号为HIGH (白色区域)
        if (leftSensor == LOW && rightSensor == LOW) {
            // 两个传感器都在黑线上，直行
            forward(0);
            return 0;
        } else if (leftSensor == HIGH && rightSensor == LOW) {
            // 左传感器在白色区域，右传感器在黑线上，车偏右，需向左转
            turnLeft(0);
            return 1;
        } else if (leftSensor == LOW && rightSensor == HIGH) {
            // 左传感器在黑线上，右传感器在白色区域，车偏左，需向右转
            turnRight(0);
            return 2;
        } else {
            // 两个传感器都在白色区域，停止
            stop(0);
            return 3;
        }
    }
    
    // 如果没有配置专用循迹传感器，则尝试使用红外传感器
    if (_hasInfraredSensor) {
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
    
    // 如果未配置任何传感器，则返回-1
    return -1;
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

void SmartCar::dynamicAvoidObstacle(float safeDistance, float spinTimeLeftDegree90, float spinTimeRightDegree90) {
    // 如果没有配置超声波传感器和舵机，则直接返回
    if (!_hasUltrasonicSensor || !_hasServo) {
        return;
    }
    
    // 检测前方距离
    float frontDistance = detectFront();
    
    // 判断是否需要避障
    if (frontDistance < safeDistance) {
        // 停止并后退
        stop(1);
        backward(2);
        stop(1);
        
        // 检测左右两侧距离
        float leftDistance = detectLeft();
        float rightDistance = detectRight();
        
        // 将舵机转回前方
        detectFront();
        
        // 根据左右距离决定转向
        if (leftDistance < safeDistance && rightDistance < safeDistance) {
            // 左右两侧都有障碍物，掉头(180度)
            spinLeft(spinTimeLeftDegree90 * 2);
            stop(1);
        } else if (leftDistance > rightDistance) {
            // 左侧空间更大，向左动态绕障
            dynamicAvoidLeft(safeDistance, spinTimeLeftDegree90, spinTimeRightDegree90);
        } else {
            // 右侧空间更大，向右动态绕障
            dynamicAvoidRight(safeDistance, spinTimeLeftDegree90, spinTimeRightDegree90);
        }
    } else {
        // 前方无障碍物，继续前进
        forward(0);
    }
}

bool SmartCar::dynamicAvoidLeft(float safeDistance, float spinTimeLeftDegree90, float spinTimeRightDegree90) {
    // 左侧空间更大，向左转90度
    spinLeft(spinTimeLeftDegree90);
    stop(1); // 确保停止并稳定姿态
    
    // 初始化绕行状态
    bool obstacleCleared = false;  // 是否已绕过障碍物
    int moveCount = 0;  // 移动计数器
    const int MAX_MOVE_COUNT = 40;  // 最大移动次数（安全措施）
    const int MIN_MOVE_COUNT = 1;   // 最小移动次数（确保至少前进一小段距离）
    
    // 前进并持续检测右侧障碍物
    while (!obstacleCleared && moveCount < MAX_MOVE_COUNT) {
        // 前进一小步
        forward(5);
        
        // 检测右侧（原障碍物方向）距离
        float rightSideDistance = detectRight();
        
        // 计数器增加
        moveCount++;
        
        // 判断是否已经绕过障碍物
        // 条件：已经前进了最小距离 + 右侧距离变大（说明已经绕过障碍物边缘）
        if (moveCount > MIN_MOVE_COUNT && rightSideDistance > safeDistance * 1.5) {
            obstacleCleared = true;
        }
        
        // 每次检测后短暂延时
        delay(100);
    }
    
    stop(1); // 停车稳定
    
    if (obstacleCleared) {
        // 已绕过障碍物，转回原来方向
        spinRight(spinTimeRightDegree90);
        stop(1);
    }
    
    // 将舵机转回前方并继续前进
    detectFront(); 
    forward(0); // 持续前进
    
    return obstacleCleared;
}

bool SmartCar::dynamicAvoidRight(float safeDistance, float spinTimeLeftDegree90, float spinTimeRightDegree90) {
    // 右侧空间更大，向右转90度
    spinRight(spinTimeRightDegree90);
    stop(1); // 确保停止并稳定姿态
    
    // 初始化绕行状态
    bool obstacleCleared = false;  // 是否已绕过障碍物
    int moveCount = 0;  // 移动计数器
    const int MAX_MOVE_COUNT = 40;  // 最大移动次数（安全措施）
    const int MIN_MOVE_COUNT = 1;   // 最小移动次数（确保至少前进一小段距离）
    
    // 前进并持续检测左侧障碍物
    while (!obstacleCleared && moveCount < MAX_MOVE_COUNT) {
        // 前进一小步
        forward(5);
        
        // 检测左侧（原障碍物方向）距离
        float leftSideDistance = detectLeft();
        
        // 计数器增加
        moveCount++;
        
        // 判断是否已经绕过障碍物
        // 条件：已经前进了最小距离 + 左侧距离变大（说明已经绕过障碍物边缘）
        if (moveCount > MIN_MOVE_COUNT && leftSideDistance > safeDistance * 1.5) {
            obstacleCleared = true;
        }
        
        // 每次检测后短暂延时
        delay(100);
    }
    
    stop(1); // 停车稳定
    
    if (obstacleCleared) {
        // 已绕过障碍物，转回原来方向
        spinLeft(spinTimeLeftDegree90);
        stop(1);
    }
    
    // 将舵机转回前方并继续前进
    detectFront();
    forward(0); // 持续前进
    
    return obstacleCleared;
}

// 自动跟随功能实现
void SmartCar::autoFollow(float targetDistance, float tolerance, int maxSpeed, int minSpeed) {
    // 如果没有配置超声波传感器，则直接返回
    if (!_hasUltrasonicSensor) {
        return;
    }
    
    // 测量距离
    float distance = getDistance();
    
    // 检查距离是否有效
    if (distance < 2 || distance > 300) {
        stop(0);
        _currentState = STATE_STOP;
        return;
    }
    
    // 计算与目标距离的误差
    float error = distance - targetDistance;
    
    // 决定移动方向和速度
    if (abs(error) <= tolerance) {
        // 在误差范围内，停止
        if (_currentState != STATE_STOP) {
            stop(0);
            _currentState = STATE_STOP;
            delay(20); // 短暂延时确保完全停止
        }
    } 
    else if (error > 0) {
        // 目标太远，前进
        if (_currentState != STATE_FORWARD) {
            // 计算速度 - 距离越远速度越大
            int speed = calculateFollowSpeed(error, minSpeed, maxSpeed);
            setSpeed(speed, speed);
            
            // 前进并记录状态
            forward(0);
            _currentState = STATE_FORWARD;
        }
    } 
    else {
        // 目标太近，后退
        if (_currentState != STATE_BACKWARD) {
            // 计算速度 - 距离偏差越大速度越大
            int speed = calculateFollowSpeed(abs(error), minSpeed, maxSpeed);
            setSpeed(speed, speed);
            
            // 后退并记录状态
            backward(0);
            _currentState = STATE_BACKWARD;
        }
    }
}

// 计算跟随速度
int SmartCar::calculateFollowSpeed(float error, int minSpeed, int maxSpeed) {
    // 使用平方根函数可以使小误差时有更小的速度增量，大误差时有更大的速度增量
    int speed = minSpeed + sqrt(error) * 10;
    
    // 限制在最小和最大速度范围内
    if (speed > maxSpeed) speed = maxSpeed;
    if (speed < minSpeed) speed = minSpeed;
    
    return speed;
}

// 花式动作：全速前进急停后退
void SmartCar::performFastForwardBackward() {
    forward(10);
    backward(10);
    stop(5);
}

// 花式动作：间断性前进
void SmartCar::performIntermittentForward(int steps, int moveTime, int stopTime) {
    for (int i = 0; i < steps; i++) {
        forward(moveTime);
        stop(stopTime);
    }
}

// 花式动作：间断性后退
void SmartCar::performIntermittentBackward(int steps, int moveTime, int stopTime) {
    for (int i = 0; i < steps; i++) {
        backward(moveTime);
        stop(stopTime);
    }
}

// 花式动作：大弯套小弯连续左旋转
void SmartCar::performLeftSpinCombination(int cycles) {
    for (int i = 0; i < cycles; i++) {
        turnLeft(10);
        spinLeft(5);
    }
}

// 花式动作：大弯套小弯连续右旋转
void SmartCar::performRightSpinCombination(int cycles) {
    for (int i = 0; i < cycles; i++) {
        turnRight(10);
        spinRight(5);
    }
}

// 花式动作：间断性原地左/右转弯
void SmartCar::performIntermittentTurn(int direction, int cycles) {
    for (int i = 0; i < cycles; i++) {
        if (direction == 0) {
            turnLeft(1);
        } else {
            turnRight(1);
        }
        stop(1);
    }
}

// 花式动作：走S形前进
void SmartCar::performSShapedMovement(int cycles) {
    for (int i = 0; i < cycles; i++) {
        turnLeft(3);
        turnRight(3);
    }
}

// 花式动作：间断性原地左/右打转
void SmartCar::performIntermittentSpin(int direction, int cycles) {
    for (int i = 0; i < cycles; i++) {
        if (direction == 0) {
            spinLeft(3);
        } else {
            spinRight(3);
        }
        stop(3);
    }
}

// 执行完整的花式动作表演
void SmartCar::performColorfulMovements() {
    // 全速前进急停后退
    performFastForwardBackward();
    
    // 小车间断性前进5步
    performIntermittentForward();
    
    // 小车间断性后退5步
    performIntermittentBackward();
    
    // 大弯套小弯连续左旋转
    performLeftSpinCombination();
    
    // 大弯套小弯连续右旋转
    performRightSpinCombination();
    
    // 间断性原地右转弯
    performIntermittentTurn(1);
    
    // 间断性原地左转弯
    performIntermittentTurn(0);
    
    // 走S形前进
    performSShapedMovement();
    
    // 间断性原地左打转
    performIntermittentSpin(0);
    
    // 间断性原地右打转
    performIntermittentSpin(1);
    
    // 完成后停止
    stop(0);
}
