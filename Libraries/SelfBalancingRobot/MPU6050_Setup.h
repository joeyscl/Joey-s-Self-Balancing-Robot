#include <Wire.h>
#include "I2Cdev/I2Cdev.h"
#include <MPU6050_6Axis_MotionApps20.h>


MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64];          // FIFO storage buffer
 
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
 
VectorFloat gravity;    // [x, y, z]            gravity vector
float yawPitchRoll[3];  // [z, y, x]   yaw/pitch/roll container
int16_t gyros[3];      // [x, y, z] gyros container


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


bool mpu_setup() {

	Wire.begin();

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    //Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately


    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    // while (Serial.available() && Serial.read()); // empty buffer
    // while (!Serial.available());                 // wait for data
    // while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    // 0 = success, !0 = error
    if (devStatus == 0) {
        Serial.println(F("Enable DMP"));
        mpu.setDMPEnabled(true);

        dmpReady = true;
 
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
 
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("Set INTerrupts"));
 
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

	mpuInterrupt = false;
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