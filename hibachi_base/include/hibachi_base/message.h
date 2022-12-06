
#ifndef __HIBACHI_BASE_MESSAGE_H__
#define __HIBACHI_BASE_MESSAGE_H__

#include <stddef.h>
#include <stdint.h>

namespace hibachi_base {
class Message {
public:
  static Message *parse(const uint8_t *data, size_t length);

protected:
  Message(const uint8_t *data, size_t length);
  Message(uint16_t messageType, size_t payloadLength);
  virtual ~Message();

public:
  uint16_t getType() const;
  uint8_t *getData() const;
  size_t getSize() const;

public:
  int8_t getI8(size_t offset) const;
  int16_t getI16(size_t offset) const;
  int32_t getI32(size_t offset) const;
  uint8_t getU8(size_t offset) const;
  uint16_t getU16(size_t offset) const;
  uint32_t getU32(size_t offset) const;
  void setI8(size_t offset, int8_t value);
  void setI16(size_t offset, int16_t value);
  void setI32(size_t offset, int32_t value);
  void setU8(size_t offset, uint8_t value);
  void setU16(size_t offset, uint16_t value);
  void setU32(size_t offset, uint32_t value);

private:
  uint8_t *_data;
  size_t _size;
};  // class

class SetSkidSteerMotorSpeed : public Message {
public:
  enum { MESSAGE_TYPE = 0x0005 };
  enum {
    FRONT_LEFT_SPEED = 0,
    FRONT_RIGHT_SPEED = 2,
    REAR_LEFT_SPEED = 4,
    REAR_RIGHT_SPEED = 6,
    PAYLOAD_LEN = 8,
  };

public:
  SetSkidSteerMotorSpeed(double frontLeftWheel,
                         double frontRightWheel,
                         double rearLeftWheel,
                         double rearRightWheel);
};  // class

class GetFourWheelEncoder : public Message {
public:
  enum { MESSAGE_TYPE = 0x0004 };
  enum {
    PAYLOAD_LEN = 0,
  };

public:
  GetFourWheelEncoder();
};  // class

class FourWheelEncoderPosition : public Message {
public:
  enum { MESSAGE_TYPE = 0x7004 };
  enum {
    FRONT_LEFT_ANGPOS = 0,
    FRONT_RIGHT_ANGPOS = 2,
    REAR_LEFT_ANGPOS = 4,
    REAR_RIGHT_ANGPOS = 6,
    PAYLOAD_LEN = 8,
  };

public:
  FourWheelEncoderPosition(const uint8_t *data, size_t length);

public:
  bool getWheelsAngPosition(double &front_left,
                            double &front_right,
                            double &rear_left,
                            double &rear_right);
};  // class

class FourWheelEncoderSpeed : public Message {
public:
  enum { MESSAGE_TYPE = 0x7006 };
  enum {
    FRONT_LEFT_ANGSPEED = 0,
    FRONT_RIGHT_ANGSPEED = 2,
    REAR_LEFT_ANGSPEED = 4,
    REAR_RIGHT_ANGSPEED = 6,
    PAYLOAD_LEN = 8,
  };

public:
  FourWheelEncoderSpeed(const uint8_t *data, size_t length);

public:
  bool getWheelsAngSpeed(double &front_left,
                         double &front_right,
                         double &rear_left,
                         double &rear_right);
};  // class

class ResetOdometry : public Message {
public:
  enum { MESSAGE_TYPE = 0x0006 };
  enum {
    PAYLOAD_LEN = 0,
  };

public:
  ResetOdometry();
};  // class

class SetPIDGains : public Message{
public:
  enum { MESSAGE_TYPE = 0x0010 };
  enum {
    WHEEL_SELECTOR = 0,
    PID_KP_GAIN = 1,
    PID_KI_GAIN = 3,
    PID_KD_GAIN = 5,
    PAYLOAD_LEN = 7,
  };

public:
  SetPIDGains(uint8_t wheel, double kP, double kI, double kD);
};

// class EchoMessage : public Message {
// public:
//   enum { MESSAGE_TYPE = 0x00AA };
//   enum {
//     VALUE = 0,
//     PAYLOAD_LEN = 2,
//   };

// public:
//   EchoMessage(uint32_t value);
//   EchoMessage(const uint8_t *data, size_t length);
//   uint32_t getValue() const;

// };  // class
}  // namespace hibachi_base

#endif  // !__HIBACHI_BASE_MESSAGE_H__
