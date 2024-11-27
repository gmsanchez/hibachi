
#ifndef __HIBACHI_BASE_SERIAL_PORT_H__
#define __HIBACHI_BASE_SERIAL_PORT_H__

#include "hibachi_base/message.h"
#include <boost/chrono.hpp>
#include <serial/serial.h>
#include <list>

#define AMPRU_SERIAL_MAX_FRAME_LENGTH    64
#define AMPRU_SERIAL_DEFAULT_BUFFER_SIZE 32

namespace hibachi_base {
class SerialPort {
public:
  SerialPort(const std::string &port = "/dev/ttyACM0");
  ~SerialPort();

public:
  void setPort(const std::string &port);
  void setTxBufferSize(size_t size);
  void setRxBufferSize(size_t size);
  void setMaxFrameLength(size_t length);
  bool openConnection();
  void closeConnection();
  void sendMessage(Message *message);
  Message *receiveMessage();
  Message *receiveMessage(uint16_t messageType);
  Message *waitMessage(double timeout);
  Message *waitMessage(uint16_t messageType, double timeout);

private:
  void read();
  void writeFrame(const uint8_t *data, size_t length);
  void encodeByte(uint8_t data, bool escape = true);
  void decodeByte(uint8_t data);
  uint16_t crc_ccitt_update(uint16_t crc, uint8_t data);

private:
  std::string _serialPort;
  serial::Serial _serial;
  boost::chrono::system_clock::time_point _lastConnectTime;
  std::list<Message *> _messageQueue;
  uint8_t *_txBuffer;
  size_t _txBufferSize;
  size_t _txBufferLength;
  uint8_t *_rxBuffer;
  size_t _rxBufferSize;
  size_t _rxBufferLength;
  bool _rxEscapeByte;
  uint8_t *_frameBuffer;
  size_t _frameBufferSize;
  size_t _frameBufferLength;
  uint16_t _frameChecksum;
};  // class
}  // namespace hibachi_base

#endif  // !__HIBACHI_BASE_SERIAL_PORT_H__
