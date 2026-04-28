#include "mainwindow.h"
#include <QApplication>
#include <QMetaType>
#include <QPointF>
#include <QVector>
#include <ros/ros.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "qt_ros_nav_gui");
  QApplication app(argc, argv);
  qRegisterMetaType<QVector<QPointF>>("QVector<QPointF>");
  MainWindow window(argc, argv);
  window.show();
  return app.exec();
}
