/**
 * @file    Explorer.ino
 * @author  dani.gutierrez@gmail.com
 * @version V1.0.0
 * @date    2024/03/02
 * @brief   Description: mBot Ranger Explorer.
 *
 */
#include <stdlib.h>
#include "MeAuriga.h"
#include <Math.h>
#include <Wire.h>
#include <TFminiS.h>
#include <Ewma.h>

// Modules and sensors
MeUltrasonicSensor ultraSensor(PORT_10); // Ultrasonic module
MeEncoderOnBoard motor1(SLOT1); // Motor encoder 1
MeEncoderOnBoard motor2(SLOT2); // Motor encoder 2
MeGyro gyro(1, 0x69); // Giroscope
#define tfSerial Serial2// Lidar
TFminiS tfmini(tfSerial); // Lidar
//--

#define TARGET_ROTATION 900

/*****************/
/*      Common   */
/*****************/
int state = 0; // 0 - stopped; 1 - forward; 2 - backguard; 3 - rotating right; 4 - rotating left
int distance = 0;
int moveSpeed = 100;
float currentRotation = 0;
Ewma gyroAdcFilter(0.2);  // Gyro moving average. More smoothing - less prone to noise, but slower to detect changes
float gyroRotX;

/*****************/
/*      Lidar    */
/*****************/
void updateDistance()
{
  tfmini.readSensor();

  int dist = tfmini.getDistance();
  int strength = tfmini.getStrength();
  int temperature = tfmini.getTemperature();

  // Check for and handle any errors.
  if (dist < 0) {
    //Serial.println(TFminiS::getErrorString(dist));
  } else {
    distance = dist;
  }
}

/*****************/
/*      Gyro     */
/*****************/
void updateGyro()
{
  gyro.update();

  int gyroReading = gyro.getGyroX() * 100;
  gyroRotX = gyroAdcFilter.filter(fabs(gyroReading)) / 100;
  
  if (gyroRotX > 1) 
  {
    currentRotation += gyroRotX;
  }  
}

/*****************/
/*    Movement   */
/*****************/
// Enable the motor to know where it is
void isr_process_motor1(void) // count the ticks - i.e. how far the motor has moved
{
  if(digitalRead(motor1.getPortB()) == 0)
  {
	  motor1.pulsePosMinus();
  }
  else
  {
	  motor1.pulsePosPlus();
  }
}

void isr_process_motor2(void) // count the ticks - i.e. how far the motor has moved
{
  if(digitalRead(motor2.getPortB()) == 0)
  {
	  motor2.pulsePosMinus();
  }
  else
  {
	  motor2.pulsePosPlus();
  }
}

void turnLeft(void)
{
  motor1.setMotorPwm(-moveSpeed);
  motor2.setMotorPwm(-moveSpeed);
}

void turnRight(void)
{
  motor1.setMotorPwm(moveSpeed);
  motor2.setMotorPwm(moveSpeed);
}

void forward(void)
{
  motor1.setMotorPwm(-moveSpeed); // setMotorPwm writes to the encoder controller
  motor2.setMotorPwm(moveSpeed);  // so setting the speed change instantly
}

void Backward(void)
{
  motor1.setMotorPwm(moveSpeed);
  motor2.setMotorPwm(-moveSpeed);
}

/*******************/
/* Extension Comms */
/*******************/
String receivedMessage = "";
int angle;
void receiveEvent(int howMany) 
{
  // while (Wire.available()) 
  // {
  //   //char c = Wire.read(); // Read each byte from the buffer
  //   //receivedMessage += c; // Append to the string

  // }
  angle = Wire.read();
}

/*****************/
/*    MAIN       */
/*****************/
void setup()
{
  // Serial
  Serial.begin(115200);

  // Communications
  Wire.begin(1);
  Wire.onReceive(receiveEvent);

  // Lidar
  tfSerial.begin(115200);
  
  // Gyro
  gyro.begin();
  
  // enable the motor to know where it is
  attachInterrupt(motor1.getIntNum(), isr_process_motor1, RISING); 
  attachInterrupt(motor2.getIntNum(), isr_process_motor2, RISING);

  // Set motors PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
  //--

  // Motors encoders configuration
  motor1.setPulse(9);
  motor2.setPulse(9);
  motor1.setRatio(39.267);
  motor2.setRatio(39.267);
  motor1.setPosPid(1.8,0,1.2);
  motor2.setPosPid(1.8,0,1.2);
  motor1.setSpeedPid(0.18,0,0);
  motor2.setSpeedPid(0.18,0,0);
  //--
  
}

void loop()
{
  updateDistance();
  updateGyro();

  if (currentRotation < TARGET_ROTATION)
  {
    turnLeft();
  }
  else
  {
    
  }

  Serial.print("State:");
  Serial.print(state);
  Serial.print(", CurrentRotation:");
  Serial.print(currentRotation);
  Serial.print(", Distance:");
  Serial.print(distance);
  Serial.print(", GyroscopeAvgX:");
  Serial.print(gyroRotX);
  Serial.print(", Servo Angle:");
  Serial.print(angle);
    
  // if (receivedMessage.length() > 0) 
  // {
  //   Serial.print(", Servo Angle: " + receivedMessage);
  //   receivedMessage = ""; // Clear the message after processing
  // }
  
  Serial.println();
}

