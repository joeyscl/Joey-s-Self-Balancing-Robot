// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#include <Wire.h>
#include "I2Cdev/I2Cdev.h"
#include <MPU6050_6Axis_MotionApps20.h>
 
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69 This is the ONLY way I can make my cheap banggood.com board work, wile also connectiong AD0 to VCC
MPU6050 mpu(0x69);
 
/* Connect VCC, GND, SDA, SCL and the MPU-6050's INT pin to Arduino's external interrupt #0.
   On the Arduino Uno and Mega 2560, this is digital I/O pin 2. */
 
// MPU control/status vars
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64];          // FIFO storage buffer
 
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
 
VectorFloat gravity;    // [x, y, z]            gravity vector
float yawPitchRoll[3];  // [z, y, x]   yaw/pitch/roll container
int16_t gyros[3];      // [x, y, z] gyros container
 
bool mpu_setup0() {
    Wire.begin(); // join I2C bus (I2Cdev library doesn't do this automatically)
 
    Serial.print(F("Init I2C "));
    mpu.initialize();
 
    Serial.print(F("ID")); Serial.println(mpu.getDeviceID());
 
    // verify connection
    Serial.println(F("Test conns"));
    Serial.println(mpu.testConnection() ? F("Conn success") : F("Conn failed"));
 
    // load and configure the DMP
    Serial.println(F("Init DMP"));
    uint8_t devStatus = mpu.dmpInitialize();
 
    // 0 = success, !0 = error
    if (devStatus == 0) {
        Serial.println(F("Enable DMP"));
        mpu.setDMPEnabled(true);
 
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
 
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("Set INTrrpts"));
 
        return true;
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP ERR ")); Serial.println(devStatus);
 
        return false;
    }
}
 
int8_t mpu_getData(AngleGyroData* data){
    uint8_t mpuIntStatus = mpu.getIntStatus();
 
    // get current FIFO count (all bytes currently in the FIFO)
    uint16_t fifoCount = mpu.getFIFOCount();
 
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {
        mpu.resetFIFO(); // reset so we can continue cleanly
        return -1; //FIFO overflow
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        if (fifoCount >= packetSize) {
          // read 1st packet from FIFO, don't bother with the rest
          mpu.getFIFOBytes(fifoBuffer, packetSize);
          mpu.resetFIFO(); // reset so we can continue cleanly
        }
 
        // in case there were plenty of packets in the FIFO, only transform the last one, to avoid wasting CPU for nothing
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(yawPitchRoll, &q, &gravity);
        mpu.dmpGetGyro(gyros, fifoBuffer);
 
        // all we care is the Y axis data
        data->angle = yawPitchRoll[1];
        data->gyro =  gyros[1];
 
        return 0; // all good
    }else {
      return -2; //Wrong Status
    }
}
 
// The temperature sensor is -40 to +85 degrees Celsius.
// It is a signed integer.
// According to the datasheet: 340 per degrees Celsius, -512 at 35 degrees.
// At 0 degrees: -512 - (340 * 35) = -12412
int8_t mpu_getTemp(){
  return (mpu.getTemperature() + 12412.0) / 340.0;
}
 