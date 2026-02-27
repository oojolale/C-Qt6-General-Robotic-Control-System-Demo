// core/RobotModel.h
#pragma once
#include <QObject>
#include <QMatrix4x4>
#include <QVector3D>
#include <QQuaternion>
#include <array>
#include <vector>
#include "models/RobotState.h"

namespace Galaxy {

// ============================================================
// DH参数（Denavit-Hartenberg）
// 描述机器人连杆几何关系
// ============================================================
struct DHParams {
    double a{0.0};       // 连杆长度 (m)
    double alpha{0.0};   // 连杆扭角 (rad)
    double d{0.0};       // 关节偏移 (m)
    double theta{0.0};   // 关节角偏置 (rad)
};

// ============================================================
// 6-DOF机械臂运动学（适用于GR系列手臂）
// ============================================================
class ArmKinematics {
public:
    static constexpr int DOF = 7; // 7自由度手臂

    // 初始化DH参数（以类人形机器人手臂为例）
    // 参考：典型7DOF人形臂配置
    explicit ArmKinematics(bool isLeft = true);

    // 正运动学：关节角 → 末端位姿
    QMatrix4x4 forwardKinematics(
        const std::array<double, DOF>& q) const;

    // 单关节变换矩阵
    QMatrix4x4 dhTransform(const DHParams& dh, double q) const;

    // 逆运动学（数值法：Jacobian伪逆迭代）
    bool inverseKinematics(
        const QMatrix4x4& targetPose,
        std::array<double, DOF>& qOut,
        const std::array<double, DOF>& qInit,
        int maxIter = 100,
        double tol = 1e-4) const;

    // Jacobian矩阵（数值微分）
    void computeJacobian(
        const std::array<double, DOF>& q,
        double J[6][DOF]) const;

    // 关节限位检查
    bool checkJointLimits(const std::array<double, DOF>& q) const;

    // 奇异性判断
    double manipulability(const std::array<double, DOF>& q) const;

    struct JointLimit {
        double min, max; // rad
    };
    std::array<JointLimit, DOF> limits;

private:
    std::array<DHParams, DOF> m_dh;
    bool m_isLeft;

    static constexpr double DEG2RAD = 3.14159265358979 / 180.0;
};

// ============================================================
// 完整机器人运动学模型
// ============================================================
class RobotModel : public QObject {
    Q_OBJECT

public:
    explicit RobotModel(QObject* parent = nullptr);

    // 正运动学更新（更新所有连杆变换矩阵）
    void updateForwardKinematics(const RobotState& state);

    // 获取连杆变换（用于3D显示）
    QMatrix4x4 getLinkTransform(int linkIndex) const;

    // 末端位姿
    QMatrix4x4 getLeftEefPose()  const { return m_leftEefPose; }
    QMatrix4x4 getRightEefPose() const { return m_rightEefPose; }

    // 工作空间检查（是否在可达范围内）
    bool isReachable(const QVector3D& pos, bool isLeft = true) const;

    // 碰撞检测（简化球模型）
    bool checkCollision(const RobotState& state) const;

    // 质心估算
    QVector3D estimateCenterOfMass(const RobotState& state) const;

    // 全身关节配置（30DOF人形）
    enum JointGroup {
        TORSO = 0,       // 躯干 (0-1)
        LEFT_ARM = 2,    // 左臂 (2-8)
        RIGHT_ARM = 9,   // 右臂 (9-15)
        LEFT_LEG = 16,   // 左腿 (16-21)
        RIGHT_LEG = 22,  // 右腿 (22-27)
        HEAD = 28        // 头部 (28-29)
    };

    // 机器人物理参数
    struct PhysicalParams {
        double totalMass{50.0};     // kg（银河GR-1约60kg）
        double height{1.73};        // m
        double armSpan{1.6};        // m
        double maxPayload{5.0};     // kg 单臂最大负载
        double maxReach{0.7};       // m 单臂最大伸展
    } physParams;

signals:
    void kinematicsUpdated();
    void collisionDetected(int link1, int link2);

private:
    ArmKinematics m_leftArm{true};
    ArmKinematics m_rightArm{false};

    // 连杆变换矩阵缓存
    std::vector<QMatrix4x4> m_linkTransforms;

    QMatrix4x4 m_leftEefPose;
    QMatrix4x4 m_rightEefPose;

    // 底盘到躯干变换
    QMatrix4x4 m_baseTorsoTransform;
};

} // namespace Galaxy
