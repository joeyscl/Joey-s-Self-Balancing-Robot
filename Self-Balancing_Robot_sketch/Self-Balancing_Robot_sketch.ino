#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "BButil.h"
#include "MPU6050_Setup.h"

#define MOTOR_A_BRAKE_PIN 9
#define MOTOR_B_BRAKE_PIN 8

#define MOTOR_A_DIR_PIN 12
#define MOTOR_B_DIR_PIN 13

#define MOTOR_A_PWM 3
#define MOTOR_B_PWM 11

#define OFF_Pin 7
int OFF_State = 0;

int Kp_Pin = A1;
int Ki_Pin = A3;
int Kd_Pin = A2;

// angle is in radians
// gyro is in radians per sec, doesn't normally go beyond 1000
// the 2 have opposite signs
AngleGyroData _angleData = {0, 0};

float _initAngle;
int16_t _initGyro;

float Kp = 750;
float Kd = 0.5;
float Ki = 0;



void setup() {

  Serial.begin(57600);
  mpu_setup();
  
  pinMode(OFF_Pin, INPUT);

  pinMode(MOTOR_A_BRAKE_PIN, OUTPUT);
  pinMode(MOTOR_A_DIR_PIN, OUTPUT);
  
  pinMode(MOTOR_B_BRAKE_PIN, OUTPUT);
  pinMode(MOTOR_B_DIR_PIN, OUTPUT);

  // configure the initial angle, which is assumed to be the "stable" position
  delay(500);

  while (mpu_getData(&_angleData) != 0); //wait to get valid data
  _initAngle = _angleData.angle;
  _initGyro = _angleData.gyro;

  // disengage motor brakes
  digitalWrite(MOTOR_A_BRAKE_PIN, LOW);
  digitalWrite(MOTOR_B_BRAKE_PIN, LOW);
  
//***THIS SECION IS FOR WHEN ROBOT IS SETUP WITH POTENTIOMETERS FOR ADJUSTING PID PARAMS***
// Calculate Kp Ki Kd from analog input (adjustable with POT)
  //Calc Kp
//  float Kp_POT_factor;
//  Kp_POT_factor = analogRead(Kp_Pin);
//  Kp_POT_factor /= 675; //675 instead of 1023 because I'm using 3.3V (instead of 5V) as Vin on the POT
//  Kp = Kp_POT_factor * 4000;  
//  //Calc Ki
//  float Ki_POT_factor;
//  Ki_POT_factor = analogRead(Ki_Pin);
//  Ki_POT_factor /= 675; //675 instead of 1023 because I'm using 3.3V (instead of 5V) as Vin on the POT
//  Ki = Ki_POT_factor * 2;
//  //Calc Ki
//  float Kd_POT_factor;
//  Kd_POT_factor = analogRead(Kd_Pin);
//  Kd_POT_factor /= 675; //675 instead of 1023 because I'm using 3.3V (instead of 5V) as Vin on the POT
//  Kd = Kd_POT_factor * 10;  
  
  
  Serial.println("Kp: ");
  Serial.println(Kp);
  
  Serial.println("Kd: ");
  Serial.println(Kd);
  
  Serial.println("Ki: ");
  Serial.println(Ki);
  
  Serial.println("All ready");

}

long currentTime, lastTime = 0;
float integralErr = 0, INTEGRAL_ERR_MAX = 10, actualAngle, ANGLE_OFFSET = 0.15;

void loop() {

  //if (!dmpReady) return;
  
  currentTime = millis();
  
  if (currentTime - lastTime > 1) {
    // get the MPU data if available
    int8_t res = mpu_getData(&_angleData);

    // if res != 0 the data is not yet ready or there were errors, so ignore and keep trying
    if (res == 0) {
      actualAngle = _angleData.angle - ANGLE_OFFSET;
      Serial.println(actualAngle); //this line for testing MPU output

      // 1. update the speed, apply PID algo (for the D, we don't need to do it manually as the sensor already gives us the gyro value which is the derivative of the angle)
      int16_t speed = Kp * actualAngle + 0 * integralErr - Kd * (_angleData.gyro - 1); //1 is the gyro-offset for my MPU
      //if(speed > -50 && speed < 50) speed = 0;
      speed = constrain(speed, -255, 255);
      //Serial.println(speed); //this line for testing speed output
      
      analogWrite(MOTOR_A_PWM, map(abs(speed), 0, 255, 20, 255));
      analogWrite(MOTOR_B_PWM, map(abs(speed), 0, 255, 20, 255));
      integralErr = constrain(integralErr + actualAngle, -INTEGRAL_ERR_MAX, INTEGRAL_ERR_MAX);

      // 2. figure out which DIRECTION to go
      if (speed < 0) {
        digitalWrite(MOTOR_A_DIR_PIN, HIGH);
        digitalWrite(MOTOR_B_DIR_PIN, LOW);
        //digitalWrite(LED_PIN, LOW);

      } else {
        digitalWrite(MOTOR_A_DIR_PIN, LOW);
        digitalWrite(MOTOR_B_DIR_PIN, HIGH);

        //digitalWrite(LED_PIN, HIGH);
      }
      //Serial.print(_angleData.angle); Serial.print(" / "); Serial.print(_angleData.gyro); Serial.print(" @ "); Serial.println(speed);

      // 3. keep track of the timings so that we do the update at regular intervals
      lastTime = currentTime;
    }
  }
}



