// main.cpp
#include <QApplication>
#include <QSplashScreen>
#include <QTimer>
#include <QFont>
#include <QFontDatabase>
#include "ui/MainWindow.h"

int main(int argc, char* argv[]) {
    // 高DPI支持（Qt6默认启用）
    QApplication app(argc, argv);
    app.setApplicationName("GalaxyRobotControl");
    app.setApplicationVersion("1.0.0");
    app.setOrganizationName("Galaxy General Robot");
    app.setOrganizationDomain("galaxy-robot.com");

    // 启动画面
    QPixmap splashPix(400, 250);
    splashPix.fill(QColor("#1a1a2e"));
    QSplashScreen splash(splashPix);
    splash.show();
    splash.showMessage(
        "银河通用机器人控制平台\n正在初始化...",
        Qt::AlignCenter, QColor("#90CAF9"));
    app.processEvents();

    // 延迟1.5s后关闭启动画面
    QTimer::singleShot(1500, &splash, &QSplashScreen::close);

    MainWindow w;
    w.show();

    return app.exec();
}
