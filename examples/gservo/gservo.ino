#include "DynamixelMotor.h"
#include "SoftwareSerial.h"

#include "gservo.h"

namespace gservo {
Set defSettings()
{
    return {
            0.0f,                       // homing pull off, deg
            FVec{15000.0f, 15000.0f},   // speed, {x,y}, deg/min
            FVec{2000.0f, 2000.0f},     // acceleration, {x,y}, deg/sec^2
            FVec{0.0f, 0.0f},           // zero position, {x,y}, deg
            FVec::ofConst(0.05f),       // proportional gain
            FVec::ofConst(0.0f),        // integral gain 
            FVec::ofConst(0.01f),       // derivative gain
            FVec::ofConst(0.0f),        // punch
            FVec::ofConst(1.0f),        // torque
    };
}
}

const unsigned long dynamixel_baudrate = 1000000;
const unsigned long serial_baudrate = 9600;

SoftwareDynamixelInterface di_{2, 3};
gservo::Motors motors_{&di_};
gservo::CallbacksImpl cb_{&Serial, &motors_};
gservo::Parser parser_{&cb_};

void setup() {  
  Serial.begin(serial_baudrate);  
  Serial.print(F("Starting..."));
  di_.begin(dynamixel_baudrate);
  motors_.changeBaud();
  di_.begin(serial_baudrate);
  motors_.led(true);
  cb_.begin();
  Serial.print(F("\rStarted\n"));
  motors_.led(false);
}

void loop() {
  if (Serial.available()) {
    char buff[128] {};
    const auto read = Serial.readBytesUntil('\n', buff, 127);
    buff[read] = '\n';
    parser_.parse(buff, read);
  }  
  cb_.loop();
}
