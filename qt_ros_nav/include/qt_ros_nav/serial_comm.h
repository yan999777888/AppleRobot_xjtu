#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <QObject>
#include <QSerialPort>

class SerialComm : public QObject {
  Q_OBJECT

public:
  explicit SerialComm(QObject* parent = nullptr);
  ~SerialComm();
  bool init(const QString& port_name, qint32 baud_rate, QSerialPort::Parity parity,
            QSerialPort::DataBits data_bits, QSerialPort::StopBits stop_bits);
  void close();

public slots:
  void slot_sendCmdVel(float linear, float angular);

private:
  QSerialPort* serial_port_;
};

#endif // SERIAL_COMM_H