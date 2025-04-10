#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "math.h"
#include <AccelStepper.h>
#include <MultiStepper.h>

#define motorInterfaceType 1

Adafruit_MPU6050 mpu;
MultiStepper steppers;
const int RightDIR = 14;
const int RightSTEP = 27;
const int LeftDIR = 4;
const int LeftSTEP = 17;
const int  steps_per_rev = 200; 
const int redledpin = 32;
const int blueledpin = 34;
const int greenledpin = 35;
const int buzzerPin = 12; 
AccelStepper leftStepper(motorInterfaceType, LeftSTEP, LeftDIR);
AccelStepper rightStepper(motorInterfaceType, RightSTEP, RightDIR);
float accAngle;

int kp = 5; //proportional constant
int ki = 0.5; //integral constant
int kd = 1; //derivative constant

//variables for PID control
float target = -4;
float error = 0;
float integral = 0;
float derivative = 0;
float last_error = 0;

 void moveForward(long steps, int delayTime)
  {
     long positions[2] = {leftStepper.currentPosition() +  steps, rightStepper.currentPosition() + steps}; // Assuming you want both steppers to move the same number of steps
  // Assuming moveTo expects an array of long values for positions of multiple steppers
  //  steppers.moveTo([leftSteps, rightSteps]);
    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    delay(20);
  }

  void moveBackward(long steps, int delayTime)
  {
    long positions[2] = {leftStepper.currentPosition() +  steps, rightStepper.currentPosition() + steps}; // Assuming you want both steppers to move the same number of steps    steppers.moveTo(positions);
    steppers.runSpeedToPosition();
    delay(20);
  }

void stop()
  {
    leftStepper.stop();
    rightStepper.stop();
  }

void setup()
  { 
    pinMode(redledpin, OUTPUT);
    pinMode(blueledpin, OUTPUT);
    pinMode(greenledpin, OUTPUT);
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(redledpin, HIGH);
    tone(buzzerPin, 1000);
    Serial.begin(38400);
    while (!Serial)
    delay(10);
    leftStepper.setMaxSpeed(1000);
    leftStepper.setAcceleration(1000);
    leftStepper.setSpeed(1000);
    leftStepper.moveTo(0);
    leftStepper.runToPosition();

    rightStepper.setMaxSpeed(1000);
    rightStepper.setAcceleration(1000);
    rightStepper.setSpeed(1000);
    rightStepper.moveTo(0);
    rightStepper.runToPosition();

  steppers.addStepper(leftStepper);
  steppers.addStepper(rightStepper);

  Serial.println("Adafruit MPU6050 test!");
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
  } // setup
 
void loop()
  {
  

    /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");


  Serial.println("");

   accAngle = atan2(a.acceleration.x, a.acceleration.z)*RAD_TO_DEG;
  
  if(isnan(accAngle));
  else
    Serial.println(accAngle);

  error = target - accAngle;
  integral = integral + error; //integral
  derivative = error - last_error; //derivative
  long PID =(error * kp) + (integral * ki) + (derivative * kd);
  Serial.println("PID: ");
Serial.println( PID);
  if(PID > 0)
  {
    moveForward(PID, 10);
    // for (int i = 0; i < PID; i++)
    // {
    //   long targetPos[2] = {leftStepper.currentPosition() + i  , rightStepper.currentPosition() + i };
    // steppers.moveTo(targetPos); 
    // steppers.run();
    //   delay(2);
    // }
  }
  else
  {
    moveBackward(PID, 10);
    // for (int i = 0; i > PID; i--)
    // {
    //   long targetPos[2] = {i  , i };
    // steppers.moveTo(targetPos); 
    //   steppers.run();
    //   delay(2);
    // }
  }
  error = last_error;
delay(50);
} // loop
  
 


  void turnRight(int steps, int delayTime)
  {
    digitalWrite(RightDIR, HIGH);
    digitalWrite(LeftDIR, LOW);
    for(int x = 0; x < steps; x++)
    {
      digitalWrite(RightSTEP, HIGH);
       digitalWrite(LeftSTEP, HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(RightSTEP, LOW);
       digitalWrite(LeftSTEP, LOW);
      delayMicroseconds(delayTime);
    }
  }

  void turnLeft(int steps, int delayTime)
  {
    digitalWrite(RightDIR, LOW);
    digitalWrite(LeftDIR, HIGH);
    for(int x = 0; x < steps; x++)
    {
      digitalWrite(RightSTEP, HIGH);
       digitalWrite(LeftSTEP, HIGH);
      delayMicroseconds(delayTime);
      digitalWrite(RightSTEP, LOW);
       digitalWrite(LeftSTEP, LOW);
      delayMicroseconds(delayTime);
    }
  }

  
   