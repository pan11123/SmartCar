/**
 * @file smart_car.cpp
 * @brief 智能小车主类的实现文件
 */

#include "smart_car.h"

SmartCar::SmartCar(uint8_t leftMotorGo, uint8_t leftMotorBack, 
                   uint8_t rightMotorGo, uint8_t rightMotorBack)
    : m_hasUltrasonicSensor(false), m_hasInfraredSensor(false), m_hasServo(false) {
    // 创建电机控制对象
    m_motor = new Motor(leftMotorGo, leftMotorBack, rightMotorGo, rightMotorBack);
}

void SmartCar::Begin() {
    // 初始化电机
    m_motor->Begin();
    
    // 如果配置了超声波传感器，则初始化
    if (m_hasUltrasonicSensor) {
        m_ultrasonicSensor->Begin();
    }
    
    // 如果配置了红外传感器，则初始化
    if (m_hasInfraredSensor) {
        m_infraredSensor->Begin();
    }
    
    // 如果配置了舵机，则初始化
    if (m_hasServo) {
        m_servo->Begin();
    }
}

void SmartCar::SetupUltrasonicSensor(uint8_t trigPin, uint8_t echoPin) {
    // 创建超声波传感器对象
    m_ultrasonicSensor = new UltrasonicSensor(trigPin, echoPin);
    m_hasUltrasonicSensor = true;
}

void SmartCar::SetupInfraredSensor(uint8_t leftPin, uint8_t rightPin) {
    // 创建红外传感器对象
    m_infraredSensor = new InfraredSensor(leftPin, rightPin);
    m_hasInfraredSensor = true;
}

void SmartCar::SetSpeed(uint8_t leftSpeed, uint8_t rightSpeed) {
    // 设置电机速度
    m_motor->SetSpeed(leftSpeed, rightSpeed);
}

void SmartCar::Forward(uint16_t time) {
    // 前进
    m_motor->Forward(time);
}

void SmartCar::Backward(uint16_t time) {
    // 后退
    m_motor->Backward(time);
}

void SmartCar::TurnLeft(uint16_t time) {
    // 左转
    m_motor->TurnLeft(time);
}

void SmartCar::TurnRight(uint16_t time) {
    // 右转
    m_motor->TurnRight(time);
}

void SmartCar::SpinLeft(uint16_t time) {
    // 左旋转
    m_motor->SpinLeft(time);
}

void SmartCar::SpinRight(uint16_t time) {
    // 右旋转
    m_motor->SpinRight(time);
}

void SmartCar::Stop(uint16_t time) {
    // 停止
    m_motor->Stop(time);
}

float SmartCar::GetDistance() {
    // 如果没有配置超声波传感器，则返回-1
    if (!m_hasUltrasonicSensor) {
        return -1.0;
    }
    
    // 获取距离
    return m_ultrasonicSensor->GetDistance();
}

bool SmartCar::ReadLeftTrackSensor() {
    // 如果没有配置红外传感器，则返回HIGH（表示未检测到黑线）
    if (!m_hasInfraredSensor) {
        return HIGH;
    }
    
    // 读取左侧红外传感器状态
    return m_infraredSensor->GetLeftSensorStatus();
}

bool SmartCar::ReadRightTrackSensor() {
    // 如果没有配置红外传感器，则返回HIGH（表示未检测到黑线）
    if (!m_hasInfraredSensor) {
        return HIGH;
    }
    
    // 读取右侧红外传感器状态
    return m_infraredSensor->GetRightSensorStatus();
}

int SmartCar::TrackLine() {
    // 如果没有配置红外传感器，则返回-1
    if (!m_hasInfraredSensor) {
        return -1;
    }
    
    // 根据红外传感器状态执行循迹
    if (m_infraredSensor->IsBothOnLine()) {
        // 两侧都检测到黑线，直行
        Forward(0);
        return 0;
    } else if (m_infraredSensor->IsLeftOnLine()) {
        // 左侧检测到黑线，右侧检测到白色区域，向右偏离，需要左转
        TurnLeft(0);
        return 1;
    } else if (m_infraredSensor->IsRightOnLine()) {
        // 右侧检测到黑线，左侧检测到白色区域，向左偏离，需要右转
        TurnRight(0);
        return 2;
    } else {
        // 两侧都检测到白色区域，停止
        Stop(0);
        return 3;
    }
}

void SmartCar::SetupServo(uint8_t servoPin) {
    // 创建舵机控制对象
    m_servo = new Servo(servoPin);
    m_hasServo = true;
}

float SmartCar::DetectFront() {
    // 如果没有配置舵机或超声波传感器，则直接返回-1
    if (!m_hasServo || !m_hasUltrasonicSensor) {
        return -1.0;
    }
    
    // 将舵机转到90度(前方)
    m_servo->SetAngle(90, 5);
    
    // 获取距离
    return GetDistance();
}

float SmartCar::DetectLeft() {
    // 如果没有配置舵机或超声波传感器，则直接返回-1
    if (!m_hasServo || !m_hasUltrasonicSensor) {
        return -1.0;
    }
    
    // 将舵机转到175度(左侧)
    m_servo->SetAngle(175, 15);
    
    // 获取距离
    return GetDistance();
}

float SmartCar::DetectRight() {
    // 如果没有配置舵机或超声波传感器，则直接返回-1
    if (!m_hasServo || !m_hasUltrasonicSensor) {
        return -1.0;
    }
    
    // 将舵机转到0度(右侧)
    m_servo->SetAngle(0, 15);
    
    // 获取距离
    return GetDistance();
}

void SmartCar::AvoidObstacle(float safeDistance) {
    // 如果配置了舵机，则使用多方向避障
    if (m_hasServo) {
        AvoidObstacleWithServo(safeDistance);
        return;
    }
    
    // 如果没有配置超声波传感器，则直接返回
    if (!m_hasUltrasonicSensor) {
        return;
    }
    
    // 获取前方距离
    float distance = GetDistance();
    
    // 判断是否在有效测量范围内
    if (m_ultrasonicSensor->IsValidRange(distance)) {
        // 如果距离小于安全距离，则停止并后退
        if (distance < safeDistance) {
            Stop(0);
            Backward(5);
            
            // 随机选择左转或右转
            if (random(2) == 0) {
                SpinLeft(10);
            } else {
                SpinRight(10);
            }
        } else {
            // 如果距离大于安全距离，则继续前进
            Forward(0);
        }
    } else {
        // 如果不在有效测量范围内，则停止
        Stop(0);
    }
}

void SmartCar::AvoidObstacleWithServo(float safeDistance) {
    // 如果没有配置舵机或超声波传感器，则直接返回
    if (!m_hasServo || !m_hasUltrasonicSensor) {
        return;
    }
    
    // 检测前方距离
    float frontDistance = DetectFront();
    
    // 判断是否在有效测量范围内
    if (m_ultrasonicSensor->IsValidRange(frontDistance)) {
        // 如果前方距离小于安全距离，则执行避障
        if (frontDistance < safeDistance) {
            // 后退减速
            Backward(2);
            Stop(2);
            
            // 检测左右两侧距离
            float leftDistance = DetectLeft();
            
            float rightDistance = DetectRight();
            
            // 根据左右两侧距离决定转向
            if ((leftDistance < 35) && (rightDistance < 35)) {
                // 左右两侧都有障碍物，掉头
                SpinLeft(7);
            } else if (leftDistance > rightDistance) {
                // 左侧空间更大，向左转
                TurnLeft(3);
                Stop(1);
            } else {
                // 右侧空间更大，向右转
                TurnRight(3);
                Stop(1);
            }
        } else {
            // 如果前方距离大于安全距离，则继续前进
            Forward(0);
        }
    } else {
        // 如果不在有效测量范围内，则停止
        Stop(0);
    }
}

void SmartCar::DynamicAvoidObstacle(float safeDistance, float spinTimeLeftDegree90, float spinTimeRightDegree90) {
    // 如果没有配置超声波传感器和舵机，则直接返回
    if (!m_hasUltrasonicSensor || !m_hasServo) {
        return;
    }
    
    // 检测前方距离
    float frontDistance = DetectFront();
    
    // 判断是否需要避障
    if (frontDistance < safeDistance) {
        // 停止并后退
        Stop(1);
        Backward(2);
        Stop(1);
        
        // 检测左右两侧距离
        float leftDistance = DetectLeft();
        float rightDistance = DetectRight();
        
        // 将舵机转回前方
        DetectFront();
        
        // 根据左右距离决定转向
        if (leftDistance < safeDistance && rightDistance < safeDistance) {
            // 左右两侧都有障碍物，掉头(180度)
            SpinLeft(spinTimeLeftDegree90 * 2);
            Stop(1);
        } else if (leftDistance > rightDistance) {
            // 左侧空间更大，向左动态绕障
            DynamicAvoidLeft(safeDistance, spinTimeLeftDegree90, spinTimeRightDegree90);
        } else {
            // 右侧空间更大，向右动态绕障
            DynamicAvoidRight(safeDistance, spinTimeLeftDegree90, spinTimeRightDegree90);
        }
    } else {
        // 前方无障碍物，继续前进
        Forward(0);
    }
}

bool SmartCar::DynamicAvoidLeft(float safeDistance, float spinTimeLeftDegree90, float spinTimeRightDegree90) {
    // 左侧空间更大，向左转90度
    SpinLeft(spinTimeLeftDegree90);
    Stop(1); // 确保停止并稳定姿态
    
    // 初始化绕行状态
    bool obstacleCleared = false;  // 是否已绕过障碍物
    int moveCount = 0;  // 移动计数器
    const int MAX_MOVE_COUNT = 40;  // 最大移动次数（安全措施）
    const int MIN_MOVE_COUNT = 1;   // 最小移动次数（确保至少前进一小段距离）
    
    // 前进并持续检测右侧障碍物
    while (!obstacleCleared && moveCount < MAX_MOVE_COUNT) {
        // 前进一小步
        Forward(5);
        
        // 检测右侧（原障碍物方向）距离
        float rightSideDistance = DetectRight();
        
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
    
    Stop(1); // 停车稳定
    
    if (obstacleCleared) {
        // 已绕过障碍物，转回原来方向
        SpinRight(spinTimeRightDegree90);
        Stop(1);
    }
    
    // 将舵机转回前方并继续前进
    DetectFront(); 
    Forward(0); // 持续前进
    
    return obstacleCleared;
}

bool SmartCar::DynamicAvoidRight(float safeDistance, float spinTimeLeftDegree90, float spinTimeRightDegree90) {
    // 右侧空间更大，向右转90度
    SpinRight(spinTimeRightDegree90);
    Stop(1); // 确保停止并稳定姿态
    
    // 初始化绕行状态
    bool obstacleCleared = false;  // 是否已绕过障碍物
    int moveCount = 0;  // 移动计数器
    const int MAX_MOVE_COUNT = 40;  // 最大移动次数（安全措施）
    const int MIN_MOVE_COUNT = 1;   // 最小移动次数（确保至少前进一小段距离）
    
    // 前进并持续检测左侧障碍物
    while (!obstacleCleared && moveCount < MAX_MOVE_COUNT) {
        // 前进一小步
        Forward(5);
        
        // 检测左侧（原障碍物方向）距离
        float leftSideDistance = DetectLeft();
        
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
    
    Stop(1); // 停车稳定
    
    if (obstacleCleared) {
        // 已绕过障碍物，转回原来方向
        SpinLeft(spinTimeLeftDegree90);
        Stop(1);
    }
    
    // 将舵机转回前方并继续前进
    DetectFront();
    Forward(0); // 持续前进
    
    return obstacleCleared;
}

// 自动跟随功能实现
void SmartCar::AutoFollow(float targetDistance, float tolerance, int maxSpeed, int minSpeed) {
    // 如果没有配置超声波传感器，则直接返回
    if (!m_hasUltrasonicSensor) {
        return;
    }
    
    // 测量距离
    float distance = GetDistance();
    
    // 检查距离是否有效
    if (distance < 2 || distance > 300) {
        Stop(0);
        m_currentState = STATE_STOP;
        return;
    }
    
    // 计算与目标距离的误差
    float error = distance - targetDistance;
    
    // 决定移动方向和速度
    if (abs(error) <= tolerance) {
        // 在误差范围内，停止
        if (m_currentState != STATE_STOP) {
            Stop(0);
            m_currentState = STATE_STOP;
            delay(20); // 短暂延时确保完全停止
        }
    } 
    else if (error > 0) {
        // 目标太远，前进
        if (m_currentState != STATE_FORWARD) {
            // 计算速度 - 距离越远速度越大
            int speed = CalculateFollowSpeed(error, minSpeed, maxSpeed);
            SetSpeed(speed, speed);
            
            // 前进并记录状态
            Forward(0);
            m_currentState = STATE_FORWARD;
        }
    } 
    else {
        // 目标太近，后退
        if (m_currentState != STATE_BACKWARD) {
            // 计算速度 - 距离偏差越大速度越大
            int speed = CalculateFollowSpeed(abs(error), minSpeed, maxSpeed);
            SetSpeed(speed, speed);
            
            // 后退并记录状态
            Backward(0);
            m_currentState = STATE_BACKWARD;
        }
    }
}

// 计算跟随速度
int SmartCar::CalculateFollowSpeed(float error, int minSpeed, int maxSpeed) {
    // 使用平方根函数可以使小误差时有更小的速度增量，大误差时有更大的速度增量
    int speed = minSpeed + sqrt(error) * 10;
    
    // 限制在最小和最大速度范围内
    if (speed > maxSpeed) speed = maxSpeed;
    if (speed < minSpeed) speed = minSpeed;
    
    return speed;
}

// 花式动作：全速前进急停后退
void SmartCar::PerformFastForwardBackward() {
    Forward(10);
    Backward(10);
    Stop(5);
}

// 花式动作：间断性前进
void SmartCar::PerformIntermittentForward(int steps, int moveTime, int stopTime) {
    for (int i = 0; i < steps; i++) {
        Forward(moveTime);
        Stop(stopTime);
    }
}

// 花式动作：间断性后退
void SmartCar::PerformIntermittentBackward(int steps, int moveTime, int stopTime) {
    for (int i = 0; i < steps; i++) {
        Backward(moveTime);
        Stop(stopTime);
    }
}

// 花式动作：大弯套小弯连续左旋转
void SmartCar::PerformLeftSpinCombination(int cycles) {
    for (int i = 0; i < cycles; i++) {
        TurnLeft(10);
        SpinLeft(5);
    }
}

// 花式动作：大弯套小弯连续右旋转
void SmartCar::PerformRightSpinCombination(int cycles) {
    for (int i = 0; i < cycles; i++) {
        TurnRight(10);
        SpinRight(5);
    }
}

// 花式动作：间断性原地左/右转弯
void SmartCar::PerformIntermittentTurn(int direction, int cycles) {
    for (int i = 0; i < cycles; i++) {
        if (direction == 0) {
            TurnLeft(1);
        } else {
            TurnRight(1);
        }
        Stop(1);
    }
}

// 花式动作：走S形前进
void SmartCar::PerformSShapedMovement(int cycles) {
    for (int i = 0; i < cycles; i++) {
        TurnLeft(3);
        TurnRight(3);
    }
}

// 花式动作：间断性原地左/右打转
void SmartCar::PerformIntermittentSpin(int direction, int cycles) {
    for (int i = 0; i < cycles; i++) {
        if (direction == 0) {
            SpinLeft(3);
        } else {
            SpinRight(3);
        }
        Stop(3);
    }
}

// 执行完整的花式动作表演
void SmartCar::PerformColorfulMovements() {
    // 全速前进急停后退
    PerformFastForwardBackward();
    
    // 小车间断性前进5步
    PerformIntermittentForward();
    
    // 小车间断性后退5步
    PerformIntermittentBackward();
    
    // 大弯套小弯连续左旋转
    PerformLeftSpinCombination();
    
    // 大弯套小弯连续右旋转
    PerformRightSpinCombination();
    
    // 间断性原地右转弯
    PerformIntermittentTurn(1);
    
    // 间断性原地左转弯
    PerformIntermittentTurn(0);
    
    // 走S形前进
    PerformSShapedMovement();
    
    // 间断性原地左打转
    PerformIntermittentSpin(0);
    
    // 间断性原地右打转
    PerformIntermittentSpin(1);
    
    // 完成后停止
    Stop(0);
}
