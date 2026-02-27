// core/BehaviorTree.h
#pragma once
#include <QObject>
#include <QString>
#include <QVariantMap>
#include <memory>
#include <vector>
#include <functional>

namespace Galaxy {

// ============================================================
// 行为树节点状态
// ============================================================
enum class BTStatus {
    Success, Failure, Running
};

// ============================================================
// 行为树节点基类
// ============================================================
class BTNode {
public:
    explicit BTNode(const QString& name) : m_name(name) {}
    virtual ~BTNode() = default;

    virtual BTStatus tick() = 0;
    virtual void reset() { m_status = BTStatus::Running; }

    QString  name() const { return m_name; }
    BTStatus status() const { return m_status; }

protected:
    QString  m_name;
    BTStatus m_status{BTStatus::Running};
};

// ============================================================
// 组合节点
// ============================================================

// Sequence：所有子节点成功才成功（AND逻辑）
class SequenceNode : public BTNode {
public:
    explicit SequenceNode(const QString& name)
        : BTNode(name) {}

    void addChild(std::shared_ptr<BTNode> child) {
        m_children.push_back(child);
    }

    BTStatus tick() override {
        for (auto& child : m_children) {
            BTStatus s = child->tick();
            if (s == BTStatus::Failure) return m_status = BTStatus::Failure;
            if (s == BTStatus::Running)  return m_status = BTStatus::Running;
        }
        return m_status = BTStatus::Success;
    }

    void reset() override {
        BTNode::reset();
        for (auto& c : m_children) c->reset();
    }

private:
    std::vector<std::shared_ptr<BTNode>> m_children;
};

// Selector：任一子节点成功即成功（OR逻辑）
class SelectorNode : public BTNode {
public:
    explicit SelectorNode(const QString& name)
        : BTNode(name) {}

    void addChild(std::shared_ptr<BTNode> child) {
        m_children.push_back(child);
    }

    BTStatus tick() override {
        for (auto& child : m_children) {
            BTStatus s = child->tick();
            if (s == BTStatus::Success) return m_status = BTStatus::Success;
            if (s == BTStatus::Running)  return m_status = BTStatus::Running;
        }
        return m_status = BTStatus::Failure;
    }

    void reset() override {
        BTNode::reset();
        for (auto& c : m_children) c->reset();
    }

private:
    std::vector<std::shared_ptr<BTNode>> m_children;
};

// Parallel：并行执行（N个成功则成功）
class ParallelNode : public BTNode {
public:
    ParallelNode(const QString& name, int successThreshold = 1)
        : BTNode(name), m_threshold(successThreshold) {}

    void addChild(std::shared_ptr<BTNode> child) {
        m_children.push_back(child);
    }

    BTStatus tick() override {
        int successCount = 0;
        int failCount    = 0;
        for (auto& child : m_children) {
            BTStatus s = child->tick();
            if (s == BTStatus::Success) successCount++;
            if (s == BTStatus::Failure) failCount++;
        }
        if (successCount >= m_threshold)
            return m_status = BTStatus::Success;
        if (failCount > static_cast<int>(m_children.size()) - m_threshold)
            return m_status = BTStatus::Failure;
        return m_status = BTStatus::Running;
    }

    void reset() override {
        BTNode::reset();
        for (auto& c : m_children) c->reset();
    }

private:
    int m_threshold;
    std::vector<std::shared_ptr<BTNode>> m_children;
};

// ============================================================
// 装饰节点
// ============================================================

// Inverter：取反
class InverterNode : public BTNode {
public:
    InverterNode(const QString& name,
                 std::shared_ptr<BTNode> child)
        : BTNode(name), m_child(child) {}

    BTStatus tick() override {
        BTStatus s = m_child->tick();
        if (s == BTStatus::Success) return m_status = BTStatus::Failure;
        if (s == BTStatus::Failure) return m_status = BTStatus::Success;
        return m_status = BTStatus::Running;
    }

private:
    std::shared_ptr<BTNode> m_child;
};

// Repeat：重复N次
class RepeatNode : public BTNode {
public:
    RepeatNode(const QString& name,
               std::shared_ptr<BTNode> child, int times = -1)
        : BTNode(name), m_child(child), m_maxTimes(times) {}

    BTStatus tick() override {
        if (m_maxTimes > 0 && m_count >= m_maxTimes)
            return m_status = BTStatus::Success;

        BTStatus s = m_child->tick();
        if (s == BTStatus::Success) {
            m_count++;
            m_child->reset();
        }
        return m_status = BTStatus::Running;
    }

    void reset() override {
        BTNode::reset();
        m_count = 0;
        m_child->reset();
    }

private:
    std::shared_ptr<BTNode> m_child;
    int m_maxTimes{-1};
    int m_count{0};
};

// ============================================================
// 叶子节点（动作/条件）
// ============================================================

// 动作节点：Lambda包装
class ActionNode : public BTNode {
public:
    using ActionFn = std::function<BTStatus()>;

    ActionNode(const QString& name, ActionFn fn)
        : BTNode(name), m_fn(fn) {}

    BTStatus tick() override {
        return m_status = m_fn();
    }

private:
    ActionFn m_fn;
};

// 条件节点：检查条件
class ConditionNode : public BTNode {
public:
    using ConditionFn = std::function<bool()>;

    ConditionNode(const QString& name, ConditionFn fn)
        : BTNode(name), m_fn(fn) {}

    BTStatus tick() override {
        return m_status = m_fn() ? BTStatus::Success : BTStatus::Failure;
    }

private:
    ConditionFn m_fn;
};

// ============================================================
// 行为树引擎
// ============================================================
class BehaviorTreeEngine : public QObject {
    Q_OBJECT

public:
    explicit BehaviorTreeEngine(QObject* parent = nullptr);

    // 设置根节点
    void setRoot(std::shared_ptr<BTNode> root) { m_root = root; }

    // 构建预定义行为树
    void buildPickAndPlaceTree(
        std::function<BTStatus()> detectObj,
        std::function<BTStatus()> moveToObj,
        std::function<BTStatus()> grasp,
        std::function<BTStatus()> moveToPlace,
        std::function<BTStatus()> release);

    void buildPatrolTree(
        const QVector<QVector3D>& waypoints,
        std::function<BTStatus(const QVector3D&)> moveTo,
        std::function<BTStatus()> inspect);

    void buildChargingTree(
        std::function<BTStatus()> findDock,
        std::function<BTStatus()> approachDock,
        std::function<BTStatus()> dock,
        std::function<BTStatus()> isCharged);

    // 执行一次Tick
    BTStatus tick();

    // 重置
    void reset();

    // 获取当前执行路径（用于可视化）
    QStringList currentPath() const { return m_currentPath; }

    bool isRunning() const { return m_running; }

public slots:
    void start();
    void stop();
    void pause();
    void resume();

signals:
    void statusChanged(BTStatus status);
    void nodeActivated(const QString& nodeName);
    void taskCompleted(bool success);

private:
    std::shared_ptr<BTNode> m_root;
    QStringList m_currentPath;
    bool m_running{false};
    bool m_paused{false};
    QTimer* m_tickTimer{nullptr};
};

} // namespace Galaxy
