// ui/MainWindow.cpp
#include "MainWindow.h"
#include "RobotViewer3D.h"
#include "JointPanel.h"
#include "TaskPanel.h"
#include "MapView.h"
#include "DiagnosticPanel.h"
#include "communication/RobotComm.h"
#include "core/RobotModel.h"
#include "core/TaskScheduler.h"
#include <QApplication>
#include <QMenuBar>
#include <QToolBar>
#include <QDockWidget>
#include <QStatusBar>
#include <QSplitter>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QGroupBox>
#include <QComboBox>
#include <QSlider>
#include <QTimer>
#include <QSettings>
#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>
#include <QDateTime>
#include <QStyle>
#include <QDebug>
#include <QtMath>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("银河通用机器人控制平台 GR-Control v1.0");
    setMinimumSize(1600, 950);
    resize(1800, 1050);

    // 核心对象
    m_robotModel    = new Galaxy::RobotModel(this);
    m_comm          = new Galaxy::RobotComm(this);
    m_taskScheduler = new Galaxy::TaskScheduler(this);

    setupCentralWidget();
    setupMenuBar();
    setupToolBar();
    setupDockWidgets();
    setupStatusBar();
    setupConnections();
    applyDarkTheme();

    // 初始化模拟数据（无真实机器人时演示用）
    initSimulationMode();

    // 定时器：50Hz UI刷新
    m_uiTimer = new QTimer(this);
    connect(m_uiTimer, &QTimer::timeout, this, &MainWindow::onUiTimerTick);
    m_uiTimer->start(20);
}

void MainWindow::setupCentralWidget() {
    m_mainSplitter = new QSplitter(Qt::Horizontal, this);

    // 左：3D视图 + 地图（垂直分割）
    auto* leftSplitter = new QSplitter(Qt::Vertical, this);

    m_robotViewer = new RobotViewer3D(this);
    m_robotViewer->setMinimumSize(700, 400);

    m_mapView = new MapView(this);
    m_mapView->setMinimumHeight(250);

    leftSplitter->addWidget(m_robotViewer);
    leftSplitter->addWidget(m_mapView);
    leftSplitter->setStretchFactor(0, 3);
    leftSplitter->setStretchFactor(1, 1);

    m_mainSplitter->addWidget(leftSplitter);
    setCentralWidget(m_mainSplitter);
}

void MainWindow::setupMenuBar() {
    // 机器人菜单
    auto* robotMenu = menuBar()->addMenu(tr("机器人(&R)"));
    robotMenu->addAction(tr("连接机器人..."), this, &MainWindow::onConnect);
    robotMenu->addAction(tr("断开连接"),      this, &MainWindow::onDisconnect);
    robotMenu->addSeparator();
    robotMenu->addAction(tr("加载URDF模型..."),
        this, [this]{ onLoadUrdf(); });
    robotMenu->addAction(tr("校准关节零位"),
        this, &MainWindow::onCalibrate);
    robotMenu->addSeparator();
    robotMenu->addAction(tr("设置..."), this, &MainWindow::onSettings);

    // 控制菜单
    auto* ctrlMenu = menuBar()->addMenu(tr("控制(&C)"));
    ctrlMenu->addAction(tr("切换手动模式"),   this, &MainWindow::onModeManual);
    ctrlMenu->addAction(tr("切换自动模式"),   this, &MainWindow::onModeAuto);
    ctrlMenu->addSeparator();
    ctrlMenu->addAction(tr("紧急停止 [F12]"), this, &MainWindow::onEStop,
                        Qt::Key_F12);
    ctrlMenu->addAction(tr("复位"),           this, &MainWindow::onReset);

    // 任务菜单
    auto* taskMenu = menuBar()->addMenu(tr("任务(&T)"));
    taskMenu->addAction(tr("新建任务..."),   this, &MainWindow::onNewTask);
    taskMenu->addAction(tr("导入任务..."),   this, [this]{
        auto fn = QFileDialog::getOpenFileName(
            this, "导入任务", "", "任务文件 (*.json *.yaml)");
        if (!fn.isEmpty()) onLoadTask(fn);
    });
    taskMenu->addAction(tr("开始任务"),     this, &MainWindow::onStartTask);
    taskMenu->addAction(tr("暂停任务"),     this, &MainWindow::onPauseTask);
    taskMenu->addAction(tr("停止任务"),     this, &MainWindow::onStopTask);

    // 数据菜单
    auto* dataMenu = menuBar()->addMenu(tr("数据(&D)"));
    dataMenu->addAction(tr("开始记录"),     this, &MainWindow::onStartLog);
    dataMenu->addAction(tr("停止记录"),     this, &MainWindow::onStopLog);
    dataMenu->addAction(tr("导出数据..."),  this, &MainWindow::onExportData);
}

void MainWindow::setupToolBar() {
    auto* tb = addToolBar("主工具栏");
    tb->setMovable(false);
    tb->setIconSize({28, 28});
    tb->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

    // 连接状态
    m_connBtn = new QPushButton(tr("连接"), this);
    m_connBtn->setIcon(style()->standardIcon(QStyle::SP_DriveNetIcon));
    m_connBtn->setFixedWidth(80);
    connect(m_connBtn, &QPushButton::clicked, this, &MainWindow::onConnect);
    tb->addWidget(m_connBtn);
    tb->addSeparator();

    // 模式选择
    m_modeCombo = new QComboBox(this);
    m_modeCombo->addItems({"待机", "手动", "半自动", "全自动"});
    m_modeCombo->setFixedWidth(90);
    connect(m_modeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onModeChanged);
    tb->addWidget(new QLabel(" 模式: "));
    tb->addWidget(m_modeCombo);
    tb->addSeparator();

    // 紧急停止（红色大按钮）
    m_eStopBtn = new QPushButton("⛔ 紧急停止", this);
    m_eStopBtn->setStyleSheet(
        "QPushButton { background:#D32F2F; color:white; "
        "font-weight:bold; font-size:12pt; border-radius:4px; padding:4px 12px;}"
        "QPushButton:pressed { background:#B71C1C; }");
    m_eStopBtn->setFixedWidth(130);
    connect(m_eStopBtn, &QPushButton::clicked, this, &MainWindow::onEStop);
    tb->addWidget(m_eStopBtn);
    tb->addSeparator();

    // 任务控制
    auto* startBtn = tb->addAction(
        style()->standardIcon(QStyle::SP_MediaPlay), "开始任务");
    auto* pauseBtn = tb->addAction(
        style()->standardIcon(QStyle::SP_MediaPause), "暂停");
    auto* stopBtn  = tb->addAction(
        style()->standardIcon(QStyle::SP_MediaStop), "停止");
    connect(startBtn, &QAction::triggered, this, &MainWindow::onStartTask);
    connect(pauseBtn, &QAction::triggered, this, &MainWindow::onPauseTask);
    connect(stopBtn,  &QAction::triggered, this, &MainWindow::onStopTask);
    tb->addSeparator();

    // 电池电量
    auto* battLabel = new QLabel(" 🔋 ", this);
    m_batteryBar = new QProgressBar(this);
    m_batteryBar->setRange(0, 100);
    m_batteryBar->setValue(85);
    m_batteryBar->setFixedWidth(100);
    m_batteryBar->setFormat("%v%");
    m_batteryBar->setStyleSheet(
        "QProgressBar::chunk { background: #4CAF50; }");
    tb->addWidget(battLabel);
    tb->addWidget(m_batteryBar);
    tb->addSeparator();

    // 任务进度
    tb->addWidget(new QLabel(" 进度: "));
    m_taskProgressBar = new QProgressBar(this);
    m_taskProgressBar->setRange(0, 100);
    m_taskProgressBar->setValue(0);
    m_taskProgressBar->setFixedWidth(120);
    m_taskProgressBar->setStyleSheet(
        "QProgressBar::chunk { background: #2196F3; }");
    tb->addWidget(m_taskProgressBar);
}

void MainWindow::setupDockWidgets() {
    setDockOptions(QMainWindow::AllowTabbedDocks |
                   QMainWindow::AllowNestedDocks);

    // ---- 关节控制面板（左侧Dock）----
    m_jointPanel = new JointPanel(this);
    auto* jointDock = new QDockWidget("关节控制", this);
    jointDock->setWidget(m_jointPanel);
    jointDock->setMinimumWidth(300);
    addDockWidget(Qt::LeftDockWidgetArea, jointDock);

    // ---- 任务面板（右侧Dock）----
    m_taskPanel = new TaskPanel(this);
    auto* taskDock = new QDockWidget("任务管理", this);
    taskDock->setWidget(m_taskPanel);
    taskDock->setMinimumWidth(320);
    addDockWidget(Qt::RightDockWidgetArea, taskDock);

    // ---- 诊断面板（右侧Dock，与任务面板Tab）----
    m_diagPanel = new DiagnosticPanel(this);
    auto* diagDock = new QDockWidget("系统诊断", this);
    diagDock->setWidget(m_diagPanel);
    tabifyDockWidget(taskDock, diagDock);

    // 将主分割器加入中间
    m_mainSplitter->setParent(this);
}

void MainWindow::setupStatusBar() {
    // 连接状态
    m_connStatusLabel = new QLabel("  ● 未连接  ");
    m_connStatusLabel->setStyleSheet("color:#F44336; font-weight:bold;");

    // 机器人模式
    m_modeStatusLabel = new QLabel("  模式: 待机  ");
    m_modeStatusLabel->setStyleSheet("color:#FF9800; font-weight:bold;");

    // 末端位置
    m_eefLabel = new QLabel("  左臂: (--,--,--)  右臂: (--,--,--)  ");

    // 时钟
    m_clockLabel = new QLabel();

    // FPS
    m_fpsLabel = new QLabel("  FPS:0  ");
    m_fpsLabel->setStyleSheet("color:#76FF03;");

    statusBar()->addWidget(m_connStatusLabel);
    statusBar()->addWidget(new QLabel("|"));
    statusBar()->addWidget(m_modeStatusLabel);
    statusBar()->addWidget(new QLabel("|"));
    statusBar()->addWidget(m_eefLabel);
    statusBar()->addPermanentWidget(m_fpsLabel);
    statusBar()->addPermanentWidget(new QLabel("|"));
    statusBar()->addPermanentWidget(m_clockLabel);

    auto* clockTimer = new QTimer(this);
    connect(clockTimer, &QTimer::timeout, this, [this]{
        m_clockLabel->setText(
            QDateTime::currentDateTime().toString(
                "  yyyy-MM-dd hh:mm:ss  "));
    });
    clockTimer->start(1000);
}

void MainWindow::setupConnections() {
    // 通信层
    connect(m_comm, &Galaxy::RobotComm::stateReceived,
            this, &MainWindow::onRobotStateReceived);
    connect(m_comm, &Galaxy::RobotComm::connectionChanged,
            this, &MainWindow::onConnectionChanged);
    connect(m_comm, &Galaxy::RobotComm::commError,
            this, [this](const QString& e){
        statusBar()->showMessage("通信错误: " + e, 5000);
    });

    // 关节面板 → 通信
    connect(m_jointPanel, &JointPanel::jointCommandRequested,
            this, [this](int id, double pos){
        m_comm->sendJointCommand(id, pos);
    });

    // 底盘速度控制
    connect(m_jointPanel, &JointPanel::baseVelRequested,
            this, [this](double vx, double vy, double omega){
        m_comm->sendBaseVelocity(vx, vy, omega);
    });

    // 任务调度
    connect(m_taskPanel, &TaskPanel::taskSubmitted,
            m_taskScheduler, &Galaxy::TaskScheduler::addTask);
    connect(m_taskScheduler, &Galaxy::TaskScheduler::taskStatusChanged,
            m_taskPanel, &TaskPanel::onTaskStatusChanged);

    // 运动学更新 → 3D视图
    connect(m_robotModel, &Galaxy::RobotModel::kinematicsUpdated,
            m_robotViewer, &RobotViewer3D::onKinematicsUpdated);
}

void MainWindow::onRobotStateReceived(const Galaxy::RobotState& state) {
    m_currentState = state;

    // 更新运动学
    m_robotModel->updateForwardKinematics(state);

    // 更新关节面板
    m_jointPanel->updateState(state);

    // 更新地图
    m_mapView->updateRobotPose(
        state.base.globalX,
        state.base.globalY,
        state.base.globalTheta);

    // 更新诊断
    m_diagPanel->updateState(state);

    // 更新3D视图
    m_robotViewer->updateRobotState(state);

    // 更新电池
    int soc = static_cast<int>(state.base.batterySOC);
    m_batteryBar->setValue(soc);
    QString battStyle = soc > 50 ? "#4CAF50" :
                        soc > 20 ? "#FF9800" : "#F44336";
    m_batteryBar->setStyleSheet(
        QString("QProgressBar::chunk { background: %1; }").arg(battStyle));

    // 更新任务进度
    m_taskProgressBar->setValue(
        static_cast<int>(state.taskProgress * 100));

    // 更新末端位置状态栏
    auto lp = m_robotModel->getLeftEefPose();
    auto rp = m_robotModel->getRightEefPose();
    m_eefLabel->setText(
        QString("  左臂:(%1,%2,%3)  右臂:(%4,%5,%6)  ")
        .arg(lp(0,3), 5, 'f', 2)
        .arg(lp(1,3), 5, 'f', 2)
        .arg(lp(2,3), 5, 'f', 2)
        .arg(rp(0,3), 5, 'f', 2)
        .arg(rp(1,3), 5, 'f', 2)
        .arg(rp(2,3), 5, 'f', 2));
}

void MainWindow::onConnectionChanged(bool connected) {
    if (connected) {
        m_connStatusLabel->setText("  ● 已连接  ");
        m_connStatusLabel->setStyleSheet("color:#4CAF50; font-weight:bold;");
        m_connBtn->setText("断开");
        disconnect(m_connBtn, &QPushButton::clicked, this, nullptr);
        connect(m_connBtn, &QPushButton::clicked, this, &MainWindow::onDisconnect);
    } else {
        m_connStatusLabel->setText("  ● 未连接  ");
        m_connStatusLabel->setStyleSheet("color:#F44336; font-weight:bold;");
        m_connBtn->setText("连接");
        disconnect(m_connBtn, &QPushButton::clicked, this, nullptr);
        connect(m_connBtn, &QPushButton::clicked, this, &MainWindow::onConnect);
    }
}

void MainWindow::onEStop() {
    auto ret = QMessageBox::critical(
        this, "紧急停止",
        "确认发送紧急停止指令？\n机器人将立即停止所有运动！",
        QMessageBox::Yes | QMessageBox::No,
        QMessageBox::No);
    if (ret == QMessageBox::Yes) {
        m_comm->sendCommand(Galaxy::RobotCmdCode::ESTOP);
        m_modeStatusLabel->setText("  ⚠️ 紧急停止  ");
        m_modeStatusLabel->setStyleSheet("color:#F44336; font-weight:bold;");
        statusBar()->showMessage("紧急停止已发送！", 10000);
    }
}

void MainWindow::onModeChanged(int index) {
    static const char* modeNames[] = {"待机", "手动", "半自动", "全自动"};
    static const char* modeColors[] = {
        "#607D8B", "#FF9800", "#2196F3", "#4CAF50"};
    if (index < 0 || index > 3) return;

    m_comm->sendCommand(
        Galaxy::RobotCmdCode::SET_MODE, {}, {},
        static_cast<uint8_t>(index));

    m_modeStatusLabel->setText(
        QString("  模式: %1  ").arg(modeNames[index]));
    m_modeStatusLabel->setStyleSheet(
        QString("color:%1; font-weight:bold;").arg(modeColors[index]));
}

void MainWindow::onUiTimerTick() {
    static int fpsCount = 0;
    static QElapsedTimer fpsTimer;
    if (!fpsTimer.isValid()) fpsTimer.start();
    fpsCount++;
    if (fpsTimer.elapsed() > 1000) {
        m_fpsLabel->setText(QString("  FPS:%1  ").arg(fpsCount));
        fpsCount = 0;
        fpsTimer.restart();
    }

    // 仿真模式：生成模拟数据
    if (m_simulationMode) {
        updateSimulation();
    }
}

void MainWindow::initSimulationMode() {
    m_simulationMode = true;

    // 初始化模拟机器人状态
    m_simState.robotId    = "GALAXY-SIM-001";
    m_simState.robotModel = "GR-1";
    m_simState.mode = Galaxy::RobotState::OperationMode::Idle;

    // 初始化关节
    static const char* jointNames[] = {
        "Torso_J1","Torso_J2",
        "L_Shoulder_J1","L_Shoulder_J2","L_Shoulder_J3",
        "L_Elbow_J1","L_Elbow_J2","L_Wrist_J1","L_Wrist_J2",
        "R_Shoulder_J1","R_Shoulder_J2","R_Shoulder_J3",
        "R_Elbow_J1","R_Elbow_J2","R_Wrist_J1","R_Wrist_J2",
        "L_Hip_J1","L_Hip_J2","L_Hip_J3",
        "L_Knee_J1","L_Ankle_J1","L_Ankle_J2",
        "R_Hip_J1","R_Hip_J2","R_Hip_J3",
        "R_Knee_J1","R_Ankle_J1","R_Ankle_J2",
        "Head_J1","Head_J2"
    };

    for (int i = 0; i < Galaxy::RobotState::NUM_JOINTS; ++i) {
        m_simState.joints[i].id   = i;
        m_simState.joints[i].name = jointNames[i];
        m_simState.joints[i].position   = 0.0;
        m_simState.joints[i].voltage    = 48.0;
        m_simState.joints[i].temperature = 25.0;
    }

    m_simState.base.batterySOC     = 85.0;
    m_simState.base.batteryVoltage = 48.0;
    m_simState.cpuUsage = 32.0;
    m_simState.memUsage = 48.0;
    m_simState.healthScore = 97.5;

    statusBar()->showMessage("仿真模式：已加载 GR-1 机器人模型", 5000);
}

void MainWindow::updateSimulation() {
    static double t = 0.0;
    t += 0.02; // 50Hz

    // 模拟关节正弦运动
    m_simState.joints[2].position = 0.3 * std::sin(0.5 * t);
    m_simState.joints[3].position = 0.2 * std::sin(0.3 * t + 0.5);
    m_simState.joints[9].position = 0.3 * std::sin(0.5 * t + M_PI);
    m_simState.joints[10].position = 0.2 * std::sin(0.3 * t + 0.5 + M_PI);

    // 腿部行走动作模拟
    double gait = std::sin(2.0 * t);
    m_simState.joints[16].position =  0.1 * gait;
    m_simState.joints[22].position = -0.1 * gait;
    m_simState.joints[19].position = -0.15 * std::abs(gait);
    m_simState.joints[25].position = -0.15 * std::abs(-gait);

    // 模拟底盘移动
    m_simState.base.x = 2.0 * std::sin(0.1 * t);
    m_simState.base.y = 2.0 * (1.0 - std::cos(0.1 * t));
    m_simState.base.theta = 0.1 * t;
    m_simState.base.globalX = m_simState.base.x;
    m_simState.base.globalY = m_simState.base.y;
    m_simState.base.globalTheta = m_simState.base.theta;

    // 模拟电池放电
    m_simState.base.batterySOC =
        std::max(0.0, 85.0 - t * 0.01);

    // 模拟温度变化
    for (auto& j : m_simState.joints) {
        j.temperature = 25.0 + 15.0 * (1.0 - std::exp(-t / 300.0));
        j.effort = 5.0 * std::sin(t + j.id * 0.3);
    }

    // CPU/内存波动
    m_simState.cpuUsage = 32.0 + 10.0 * std::sin(0.2 * t);
    m_simState.timestamp = QDateTime::currentDateTime();
    m_simState.uptime = t;

    // 触发UI更新
    onRobotStateReceived(m_simState);
}

void MainWindow::applyDarkTheme() {
    qApp->setStyle("Fusion");
    QPalette p;
    p.setColor(QPalette::Window,          QColor(26,26,46));
    p.setColor(QPalette::WindowText,      QColor(224,224,224));
    p.setColor(QPalette::Base,            QColor(15,52,96));
    p.setColor(QPalette::AlternateBase,   QColor(22,33,62));
    p.setColor(QPalette::Text,            QColor(224,224,224));
    p.setColor(QPalette::Button,          QColor(22,33,62));
    p.setColor(QPalette::ButtonText,      QColor(224,224,224));
    p.setColor(QPalette::Highlight,       QColor(233,69,96));
    p.setColor(QPalette::HighlightedText, Qt::white);
    p.setColor(QPalette::Link,            QColor(100,181,246));
    qApp->setPalette(p);

    setStyleSheet(R"(
        QMainWindow, QDialog { background:#1a1a2e; }
        QDockWidget::title {
            background:#16213e; color:#90CAF9;
            padding:5px; font-weight:bold;
            border-bottom: 2px solid #e94560;
        }
        QToolBar {
            background:#16213e;
            border-bottom: 2px solid #e94560;
            padding:3px; spacing:3px;
        }
        QMenuBar {
            background:#16213e; color:#e0e0e0;
        }
        QMenuBar::item:selected { background:#0f3460; }
        QMenu {
            background:#16213e; color:#e0e0e0;
            border:1px solid #0f3460;
        }
        QMenu::item:selected { background:#0f3460; }
        QStatusBar {
            background:#16213e; color:#90CAF9;
            border-top:1px solid #0f3460;
        }
        QGroupBox {
            color:#90CAF9; border:1px solid #1e3a5f;
            border-radius:4px; margin-top:6px;
        }
        QGroupBox::title {
            subcontrol-origin:margin; left:8px;
            font-weight:bold;
        }
        QSlider::groove:horizontal {
            background:#0f3460; height:6px; border-radius:3px;
        }
        QSlider::handle:horizontal {
            background:#e94560; width:14px; height:14px;
            border-radius:7px; margin:-4px 0;
        }
        QTabWidget::pane {
            border:1px solid #0f3460; background:#1a1a2e;
        }
        QTabBar::tab {
            background:#16213e; color:#e0e0e0;
            padding:5px 12px; border:1px solid #0f3460;
        }
        QTabBar::tab:selected { background:#0f3460; color:#e94560; }
    )");
}

// 其余槽函数简实现
void MainWindow::onConnect() {
    bool ok;
    QString host = QInputDialog::getText(
        this, "连接机器人", "机器人IP:", QLineEdit::Normal, "192.168.1.100", &ok);
    if (ok && !host.isEmpty()) {
        m_simulationMode = false;
        if (!m_comm->connectToRobot(host, 9090)) {
            QMessageBox::warning(this, "连接失败",
                "无法连接，已切换到仿真模式");
            m_simulationMode = true;
        }
    }
}
void MainWindow::onDisconnect()  { m_comm->disconnect(); m_simulationMode = true; }
void MainWindow::onReset()       { m_comm->sendCommand(Galaxy::RobotCmdCode::RESET); }
void MainWindow::onModeManual()  { m_modeCombo->setCurrentIndex(1); }
void MainWindow::onModeAuto()    { m_modeCombo->setCurrentIndex(3); }
void MainWindow::onCalibrate()   { statusBar()->showMessage("关节校准中...", 3000); }
void MainWindow::onLoadUrdf()    { statusBar()->showMessage("加载URDF...", 2000); }
void MainWindow::onNewTask()     { m_taskPanel->showNewTaskDialog(); }
void MainWindow::onStartTask()   { m_taskScheduler->startCurrentTask(); }
void MainWindow::onPauseTask()   { m_taskScheduler->pauseCurrentTask(); }
void MainWindow::onStopTask()    { m_taskScheduler->stopCurrentTask(); }
void MainWindow::onLoadTask(const QString&) {}
void MainWindow::onStartLog()    { m_logging = true; statusBar()->showMessage("开始记录", 2000); }
void MainWindow::onStopLog()     { m_logging = false; statusBar()->showMessage("停止记录", 2000); }
void MainWindow::onExportData()  { statusBar()->showMessage("导出数据...", 2000); }
void MainWindow::onSettings()    {}
