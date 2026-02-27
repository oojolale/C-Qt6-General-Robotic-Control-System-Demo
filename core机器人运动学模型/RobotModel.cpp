// core/RobotModel.cpp
#include "RobotModel.h"
#include <QtMath>
#include <QDebug>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace Galaxy {

// ============================================================
// ArmKinematics 实现
// ============================================================
ArmKinematics::ArmKinematics(bool isLeft) : m_isLeft(isLeft) {
    // 7DOF人形臂DH参数（参考典型配置，单位：m/rad）
    // [a, alpha, d, theta_offset]
    m_dh = {{
        {0.000,  M_PI_2,  0.00,  0.0},   // J1: 肩旋转
        {0.000, -M_PI_2,  0.00,  0.0},   // J2: 肩俯仰
        {0.000,  M_PI_2,  0.30,  0.0},   // J3: 上臂旋转
        {0.000, -M_PI_2,  0.00,  0.0},   // J4: 肘俯仰
        {0.000,  M_PI_2,  0.25,  0.0},   // J5: 前臂旋转
        {0.000, -M_PI_2,  0.00,  0.0},   // J6: 腕俯仰
        {0.000,  0.0,     0.08,  0.0},   // J7: 腕旋转
    }};

    // 左右臂镜像
    if (!isLeft) {
        for (auto& dh : m_dh) dh.alpha = -dh.alpha;
    }

    // 关节限位 (rad)
    limits = {{
        {-2.09, 2.09},  // J1 ±120°
        {-2.09, 0.52},  // J2 -120° ~ +30°
        {-3.14, 3.14},  // J3 ±180°
        {-2.09, 0.17},  // J4 -120° ~ +10°
        {-3.14, 3.14},  // J5 ±180°
        {-1.57, 1.57},  // J6 ±90°
        {-3.14, 3.14},  // J7 ±180°
    }};
}

QMatrix4x4 ArmKinematics::dhTransform(const DHParams& dh, double q) const {
    double theta = q + dh.theta;
    double ct = std::cos(theta), st = std::sin(theta);
    double ca = std::cos(dh.alpha), sa = std::sin(dh.alpha);

    // 标准DH变换矩阵
    // [ct, -st*ca,  st*sa, a*ct]
    // [st,  ct*ca, -ct*sa, a*st]
    // [ 0,    sa,     ca,    d ]
    // [ 0,     0,      0,    1 ]
    QMatrix4x4 T;
    T(0,0)= ct;    T(0,1)=-st*ca; T(0,2)= st*sa; T(0,3)=dh.a*ct;
    T(1,0)= st;    T(1,1)= ct*ca; T(1,2)=-ct*sa; T(1,3)=dh.a*st;
    T(2,0)= 0.0;   T(2,1)= sa;    T(2,2)= ca;    T(2,3)=dh.d;
    T(3,0)= 0.0;   T(3,1)= 0.0;   T(3,2)= 0.0;  T(3,3)=1.0;
    return T;
}

QMatrix4x4 ArmKinematics::forwardKinematics(
    const std::array<double, DOF>& q) const
{
    QMatrix4x4 T;
    T.setToIdentity();
    for (int i = 0; i < DOF; ++i) {
        T = T * dhTransform(m_dh[i], q[i]);
    }
    return T;
}

void ArmKinematics::computeJacobian(
    const std::array<double, DOF>& q,
    double J[6][DOF]) const
{
    // 数值微分（有限差分法）
    constexpr double dq = 1e-6;

    QMatrix4x4 T0 = forwardKinematics(q);
    QVector3D  p0(T0(0,3), T0(1,3), T0(2,3));

    for (int j = 0; j < DOF; ++j) {
        auto qd = q;
        qd[j] += dq;
        QMatrix4x4 Td = forwardKinematics(qd);

        // 位置Jacobian（线速度部分）
        J[0][j] = (Td(0,3) - T0(0,3)) / dq;
        J[1][j] = (Td(1,3) - T0(1,3)) / dq;
        J[2][j] = (Td(2,3) - T0(2,3)) / dq;

        // 姿态Jacobian（角速度部分，从旋转矩阵变化提取）
        QMatrix4x4 dR = Td * T0.inverted();
        J[3][j] = (dR(2,1) - dR(1,2)) / (2.0 * dq);
        J[4][j] = (dR(0,2) - dR(2,0)) / (2.0 * dq);
        J[5][j] = (dR(1,0) - dR(0,1)) / (2.0 * dq);
    }
}

bool ArmKinematics::inverseKinematics(
    const QMatrix4x4& targetPose,
    std::array<double, DOF>& qOut,
    const std::array<double, DOF>& qInit,
    int maxIter, double tol) const
{
    qOut = qInit;
    double J[6][DOF];

    QVector3D  pd(targetPose(0,3), targetPose(1,3), targetPose(2,3));

    for (int iter = 0; iter < maxIter; ++iter) {
        // 当前末端位姿
        QMatrix4x4 Tc = forwardKinematics(qOut);
        QVector3D  pc(Tc(0,3), Tc(1,3), Tc(2,3));

        // 位置误差
        QVector3D  ep = pd - pc;

        // 姿态误差（简化：仅位置控制）
        double errNorm = ep.length();
        if (errNorm < tol) return true;

        // 计算Jacobian
        computeJacobian(qOut, J);

        // 伪逆（简化：阻尼最小二乘 Levenberg-Marquardt）
        constexpr double lambda = 0.01;

        // 对于位置仅控制（3×7的J_pos）
        // dq = J_pos^T * (J_pos * J_pos^T + lambda²I)^{-1} * ep
        double JJT[3][3] = {};
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                for (int k = 0; k < DOF; ++k)
                    JJT[r][c] += J[r][k] * J[c][k];

        // 加阻尼项
        for (int i = 0; i < 3; ++i)
            JJT[i][i] += lambda * lambda;

        // 简化：直接用转置近似（实际应求逆）
        double ep3[3] = {ep.x(), ep.y(), ep.z()};
        for (int j = 0; j < DOF; ++j) {
            double dqj = 0.0;
            for (int r = 0; r < 3; ++r)
                dqj += J[r][j] * ep3[r] * 0.5; // 步长0.5
            qOut[j] += dqj;
        }

        // 关节限位投影
        for (int j = 0; j < DOF; ++j) {
            qOut[j] = std::clamp(qOut[j], limits[j].min, limits[j].max);
        }
    }
    return false; // 未收敛
}

double ArmKinematics::manipulability(
    const std::array<double, DOF>& q) const
{
    double J[6][DOF];
    computeJacobian(q, J);

    // w = sqrt(det(J*J^T))，简化：用对角元素近似
    double w = 1.0;
    for (int r = 0; r < 3; ++r) {
        double norm = 0.0;
        for (int j = 0; j < DOF; ++j)
            norm += J[r][j] * J[r][j];
        w *= std::sqrt(norm + 1e-10);
    }
    return w;
}

bool ArmKinematics::checkJointLimits(
    const std::array<double, DOF>& q) const
{
    for (int j = 0; j < DOF; ++j) {
        if (q[j] < limits[j].min || q[j] > limits[j].max)
            return false;
    }
    return true;
}

// ============================================================
// RobotModel 实现
// ============================================================
RobotModel::RobotModel(QObject* parent)
    : QObject(parent)
{
    m_linkTransforms.resize(RobotState::NUM_JOINTS + 1);
    for (auto& t : m_linkTransforms) t.setToIdentity();
}

void RobotModel::updateForwardKinematics(const RobotState& state) {
    // 更新底盘变换
    m_baseTorsoTransform.setToIdentity();
    m_baseTorsoTransform.translate(
        state.base.x, state.base.y, 0.0f);
    m_baseTorsoTransform.rotate(
        static_cast<float>(state.base.theta * 180.0 / M_PI), 0, 0, 1);

    // 更新左臂正运动学
    std::array<double, 7> qLeft{};
    for (int i = 0; i < 7; ++i)
        qLeft[i] = state.joints[JointGroup::LEFT_ARM + i].position;
    m_leftEefPose = m_leftArm.forwardKinematics(qLeft);

    // 更新右臂正运动学
    std::array<double, 7> qRight{};
    for (int i = 0; i < 7; ++i)
        qRight[i] = state.joints[JointGroup::RIGHT_ARM + i].position;
    m_rightEefPose = m_rightArm.forwardKinematics(qRight);

    // 更新连杆变换缓存（简化）
    for (int i = 0; i < static_cast<int>(m_linkTransforms.size()); ++i) {
        m_linkTransforms[i] = m_baseTorsoTransform;
        // 实际应按连杆链计算累积变换
    }

    emit kinematicsUpdated();
}

bool RobotModel::isReachable(const QVector3D& pos, bool isLeft) const {
    float dist = pos.length();
    return dist <= physParams.maxReach && dist > 0.1f;
}

QVector3D RobotModel::estimateCenterOfMass(const RobotState& state) const {
    // 简化：取躯干中点高度
    return QVector3D(
        state.base.x, state.base.y,
        physParams.height * 0.5f
    );
}

QMatrix4x4 RobotModel::getLinkTransform(int linkIndex) const {
    if (linkIndex < 0 ||
        linkIndex >= static_cast<int>(m_linkTransforms.size()))
        return QMatrix4x4();
    return m_linkTransforms[linkIndex];
}

} // namespace Galaxy
