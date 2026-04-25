#include "serial_comm.h"
#include <QDebug>

SerialComm::SerialComm(QObject* parent) : QObject(parent), serial_port_(new QSerialPort(this)) {}

SerialComm::~SerialComm() {
  if (serial_port_->isOpen()) serial_port_->close();
}

bool SerialComm::init(const QString& port_name, qint32 baud_rate, QSerialPort::Parity parity,
                      QSerialPort::DataBits data_bits, QSerialPort::StopBits stop_bits) {
  serial_port_->setPortName(port_name);
  serial_port_->setBaudRate(baud_rate);
  serial_port_->setParity(parity);
  serial_port_->setDataBits(data_bits);
  serial_port_->setStopBits(stop_bits);
  if (serial_port_->open(QIODevice::ReadWrite)) {
    qDebug() << "Serial port opened:" << port_name;
    return true;
  } else {
    qDebug() << "Failed to open serial port:" << port_name;
    return false;
  }
}

void SerialComm::close() {
  if (serial_port_->isOpen()) {
    serial_port_->close();
    qDebug() << "Serial port closed";
  }
}

void SerialComm::slot_sendCmdVel(float linear, float angular) {
  QByteArray data = QString("xjtu,%1,%2,0xDD").arg(linear).arg(angular).toUtf8();
  if (serial_port_->isOpen()) {
    serial_port_->write(data);
    serial_port_->flush();
  }
}