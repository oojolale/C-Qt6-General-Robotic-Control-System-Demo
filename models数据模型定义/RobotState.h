// models/RobotState.h
#pragma once
#include <array>
#include <vector>
#include <QString>
#include <QDateTime>
#include <QVector3D>
#include <QQuaternion>
#include <QMatrix4x4>

namespace Galaxy {

// ============================================================
// 关节类型枚举
// ============================================================
enum class JointType {
    Revolute,       // 旋转关节
    Prismatic,      // 移动关节
    Continuous,     // 连续旋转
    Fixed           // 固定关节
};

// ============================================================
// 单关节状态
// ============================================================
struct JointState {
    int      id{0};
    QString  name;
    JointType type{JointType::Revolute};

    // 运动学状态
    double position{0.0};       // rad 或 m
    double velocity{0.0};       // rad/s 或 m/s
    double acceleration{0.0};   // rad/s² 或 m/s²
    double effort{0.0};         // Nm 或 N（力矩/力）

    // 目标值
    double targetPosition{0.0};
    double targetVelocity{0.0};
    double targetEffort{0.0};

    // 限位
    double posMin{-3.14159};
    double posMax{3.14159};
    double velMax{3.0};         // rad/s
    double effortMax{100.0};    // Nm

    // 温度/电气
    double temperature{25.0};   // ℃
    double current{0.0};        // A
    double voltage{48.0};       // V

    // 健康状态
    bool enabled{true};
    bool faultActive{false};
    uint32_t faultCode{0};
    QString  faultMsg;

    // 工具函数
    double positionDeg() const {
        return position * 180.0 / 3.14159265358979;
    }
    double utilizationRatio() const {
        return (effortMax > 0) ? std::abs(effort) / effortMax : 0.0;
    }
};

// ============================================================
// 末端执行器状态
// ============================================================
struct EndEffectorState {
    QVector3D    position;          // 世界坐标系位置 (m)
    QQuaternion  orientation;       // 姿态四元数
    QVector3D    linearVelocity;    // 线速度 (m/s)
    QVector3D    angularVelocity;   // 角速度 (rad/s)
    QVector3D    force;             // 末端力 (N)
    QVector3D    torque;            // 末端力矩 (Nm)

    // 夹爪状态
    double gripperOpening{0.0};     // 0.0=关闭 1.0=全开
    double gripperForce{0.0};       // N
    bool   objectGrasped{false};

    // 工具中心点（TCP）变换
    QMatrix4x4 tcpTransform;

    QVector3D euler() const {
        // 从四元数提取欧拉角（roll, pitch, yaw）
        auto e = orientation.toEulerAngles();
        return {e.x(), e.y(), e.z()};
    }
};

// ============================================================
// 底盘状态（移动底盘）
// ============================================================
struct BaseState {
    // 里程计位姿（局部坐标）
    double x{0.0}, y{0.0}, theta{0.0};  // m, m, rad

    // 速度
    double vx{0.0}, vy{0.0}, omega{0.0}; // m/s, m/s, rad/s

    // 全局定位（SLAM/GPS融合）
    double globalX{0.0}, globalY{0.0};
    double globalTheta{0.0};
    double localizationConfidence{0.0};   // [0,1]

    // 底盘健康
    double batteryVoltage{48.0};
    double batterySOC{100.0};            // %
    double leftWheelSpeed{0.0};          // m/s
    double rightWheelSpeed{0.0};
    bool   eStopActive{false};
    bool   bumperTriggered{false};
};

// ============================================================
// 传感器数据
// ============================================================
struct SensorData {
    // IMU
    QVector3D imuLinearAcc;             // m/s²
    QVector3D imuAngularVel;            // rad/s
    QVector3D imuMagnet;                // μT
    QQuaternion imuOrientation;
    bool imuValid{false};

    // 激光雷达（简化：存储距离数组）
    QVector<float> lidarRanges;         // m
    float lidarAngleMin{-3.14159f};
    float lidarAngleMax{3.14159f};
    float lidarAngleIncrement{0.01f};
    bool  lidarValid{false};

    // RGB-D相机
    bool   cameraValid{false};
    double depthFps{0.0};

    // 力传感器（末端）
    QVector3D ftForce;                  // N
    QVector3D ftTorque;                 // Nm
    bool ftValid{false};
};

// ============================================================
// 完整机器人状态
// ============================================================
struct RobotState {
    // 基本信息
    QString      robotId{"GALAXY-001"};
    QString      robotModel{"GR-1"};     // 银河通用GR系列
    QDateTime    timestamp;
    double       uptime{0.0};            // 秒

    // 运行模式
    enum class OperationMode {
        Idle, Manual, SemiAuto, FullAuto, SafeStop, EmergencyStop
    } mode{OperationMode::Idle};

    // 关节组（人形机器人：全身关节）
    // 典型配置：躯干2+左臂7+右臂7+左腿6+右腿6+头部2 = 30DOF
    static constexpr int NUM_JOINTS = 30;
    std::array<JointState, NUM_JOINTS> joints;

    // 末端执行器（双臂）
    EndEffectorState leftEef;
    EndEffectorState rightEef;

    // 底盘
    BaseState base;

    // 传感器
    SensorData sensors;

    // 系统资源
    double cpuUsage{0.0};       // %
    double memUsage{0.0};       // %
    double gpuUsage{0.0};       // %（感知计算）
    double diskFree{0.0};       // GB

    // 任务状态
    QString currentTask;
    double  taskProgress{0.0};  // [0,1]
    int     taskQueueSize{0};

    // 全局健康评分
    double healthScore{100.0};  // [0,100]

    // 工具函数
    QString modeString() const {
        static const char* names[] = {
            "待机", "手动", "半自动", "全自动", "安全停止", "紧急停止"
        };
        return names[static_cast<int>(mode)];
    }

    bool isEmergency() const {
        return mode == OperationMode::EmergencyStop;
    }
};

// ============================================================
// 任务定义
// ============================================================
enum class TaskType {
    MoveToPoint,         // 移动到目标点
    PickAndPlace,        // 拾取放置
    Inspection,          // 巡检
    HumanInteraction,    // 人机交互
    ChargingDock,        // 自主充电
    MapBuilding,         // 建图
    PatrolRoute,         // 路线巡逻
    CustomScript         // 自定义脚本
};

struct TaskDefinition {
    QString   taskId;
    QString   taskName;
    TaskType  type{TaskType::MoveToPoint};
    int       priority{5};          // 1-10
    bool      preemptable{true};

    // 目标参数
    QVector3D  targetPosition;
    QQuaternion targetOrientation;
    double     approachDistance{0.1};  // m

    // 拾取放置参数
    QString    objectId;
    double     graspWidth{0.05};       // m
    double     liftHeight{0.3};        // m

    // 巡逻参数
    QVector<QVector3D> waypoints;
    bool               loopRoute{false};
    int                patrolCount{1};

    // 状态
    enum class Status {
        Pending, Running, Paused, Completed, Failed, Cancelled
    } status{Status::Pending};

    double progress{0.0};
    QString statusMessage;
    QDateTime startTime;
    QDateTime endTime;
};

// ============================================================
// 通信协议帧
// ============================================================
#pragma pack(push, 1)
struct RobotTelemetryFrame {
    uint16_t syncWord{0xAA55};
    uint8_t  frameType{0x01};      // 遥测帧
    uint32_t frameSeq{0};
    uint32_t timestamp_ms{0};
    uint8_t  robotMode{0};

    // 关节数据（压缩：位置×32个关节）
    float    jointPositions[30]{};
    float    jointVelocities[30]{};
    float    jointEfforts[30]{};

    // 底盘
    float    baseX{0.f}, baseY{0.f}, baseTheta{0.f};
    float    baseVx{0.f}, baseVy{0.f}, baseOmega{0.f};
    float    batterySOC{100.f};

    // 末端
    float    leftEefPos[3]{};
    float    rightEefPos[3]{};

    // 系统
    float    cpuUsage{0.f};
    float    memUsage{0.f};

    uint8_t  healthFlags{0xFF};
    uint16_t checksum{0};
};

struct RobotCommandFrame {
    uint16_t syncWord{0xAA56};
    uint8_t  frameType{0x02};      // 指令帧
    uint32_t frameSeq{0};
    uint32_t timestamp_ms{0};
    uint8_t  cmdCode{0};

    // 关节指令
    float    targetPositions[30]{};
    float    targetVelocities[30]{};

    // 底盘指令
    float    cmdVx{0.f}, cmdVy{0.f}, cmdOmega{0.f};

    // 夹爪指令
    float    leftGripper{0.f};
    float    rightGripper{0.f};

    uint8_t  modeCmd{0};
    uint8_t  reserved[8]{};
    uint16_t checksum{0};
};
#pragma pack(pop)

// 指令码
enum class RobotCmdCode : uint8_t {
    NOP              = 0x00,
    SET_JOINT_POS    = 0x01,    // 设置关节位置
    SET_BASE_VEL     = 0x02,    // 设置底盘速度
    SET_EEF_POSE     = 0x03,    // 设置末端位姿（逆运动学）
    SET_GRIPPER      = 0x04,    // 控制夹爪
    SET_MODE         = 0x10,    // 切换模式
    EXECUTE_TASK     = 0x20,    // 执行任务
    PAUSE_TASK       = 0x21,    // 暂停任务
    RESUME_TASK      = 0x22,    // 恢复任务
    CANCEL_TASK      = 0x23,    // 取消任务
    ESTOP            = 0xFE,    // 紧急停止
    RESET            = 0xFF,    // 重置
};

} // namespace Galaxy
