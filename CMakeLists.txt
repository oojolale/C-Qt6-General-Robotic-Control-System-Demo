cmake_minimum_required(VERSION 3.21)
project(GalaxyRobotControl VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS
    Core Widgets Network Charts
    3DCore 3DRender 3DExtras 3DInput
    SerialPort Concurrent Sql
    OpenGL OpenGLWidgets
)

# 可选：Eigen用于矩阵运算（运动学）
find_package(Eigen3 QUIET)

set(SOURCES
    main.cpp
    core/RobotModel.cpp
    core/JointController.cpp
    core/TrajectoryPlanner.cpp
    core/BehaviorTree.cpp
    core/TaskScheduler.cpp
    perception/SensorFusion.cpp
    communication/RobotComm.cpp
    ui/MainWindow.cpp
    ui/RobotViewer3D.cpp
    ui/JointPanel.cpp
    ui/TaskPanel.cpp
    ui/MapView.cpp
    ui/DiagnosticPanel.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES} resources.qrc)

target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt6::Core Qt6::Widgets Qt6::Network
    Qt6::Charts Qt6::3DCore Qt6::3DRender
    Qt6::3DExtras Qt6::3DInput
    Qt6::SerialPort Qt6::Concurrent Qt6::Sql
    Qt6::OpenGL Qt6::OpenGLWidgets
)

if(Eigen3_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
    target_compile_definitions(${PROJECT_NAME} PRIVATE USE_EIGEN)
endif()
