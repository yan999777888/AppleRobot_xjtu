#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "qt_ros_nav_gui");
  QApplication app(argc, argv);
  MainWindow window(argc, argv);
  window.show();
  return app.exec();
}