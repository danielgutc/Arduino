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

/*****************/
/*      Common   */
/*****************/
#define WEIGHT_SERVO_ROTATION 1
#define WEIGHT_DISTANCE 1
#define MIN_DISTANCE 50
#define MIN_OPEN_ANGLE 27
#define MAX_SPEED 75
#define MIN_ANGLE 0
#define MAX_ANGLE 180
#define MAX_ANGLE_TOLERANCE 5
#define ANGLE_TO_MOTOR_MULTIPLIER 6 

int state = 0;
int leftMotorPosition = 0;
int rightMotorPosition = 0;
float currentRotation = 0;
Ewma gyroAdcFilter(0.2);  // Gyro moving average. More smoothing - less prone to noise, but slower to detect changes
float gyroRotX;
String receivedMessage = "";
float angle;

int distances[180];
int startOpenAngle = 0;
int endOpenAngle = 0;
int scanState = 0;
int targetAngle = -1;
int frontDistance = 0;
int distance = 0;

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
  
  if (gyroRotX > 0.2) 
  {
    currentRotation += gyroReading;
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


void move(float leftSpeed, float rightSpeed)
{
  motor1.setMotorPwm((int)-leftSpeed); 
  motor2.setMotorPwm((int)rightSpeed);
}

void stop()
{
  motor1.setMotorPwm(0);
  motor2.setMotorPwm(0);
}

/*******************/
/* Extension Comms */
/*******************/

void receiveEvent(int howMany) 
{
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
  frontDistance = ultraSensor.distanceCm();

  if (state == 0) // Move forward
  {
    if (frontDistance < MIN_DISTANCE) // Stop moving and 
    {
      stop();
      state = 1;
      scanState = 0;
    }
    else
    {
      move(MAX_SPEED, MAX_SPEED);
    }
  }
  else if (state == 1) // Steering direction
  {
    int intAngle = (int)angle; 
    if (intAngle >= (MIN_ANGLE + MAX_ANGLE_TOLERANCE) && scanState == 0)
    {
      scanState = 1;
    }

    if (scanState == 1) // Collecting distances
    {
      distances[intAngle - 1] = distance;
    }

    if (scanState == 1 && intAngle >= MAX_ANGLE - MAX_ANGLE_TOLERANCE)
    {
      scanState = 2; // All angles scanned
    }

    if (scanState == 2)
    {
      targetAngle = -1;
      for (int i = MIN_ANGLE; i < MAX_ANGLE - MIN_OPEN_ANGLE ; i++)
      {
        if (targetAngle != -1)
        {
          state = 3;
          break;
        }

        int window_size = 0;
        targetAngle = i + (MIN_OPEN_ANGLE / 2);
        for (int j = i; j < i + MIN_OPEN_ANGLE; j++)
        {
          if (distances[j] < MIN_DISTANCE)
          {
            targetAngle = -1;
            break;
          }
        }
      }

      if (targetAngle == -1) // No open angle to move found, turn 90 degrees to the left
      {
        targetAngle = 0;
        state = 3;
      }
    }
  }
  else if (state == 3) // Rotation to the target direction
  {
    motor2.setTarPWM(ANGLE_TO_MOTOR_MULTIPLIER * (targetAngle - 90)); // Decrease 90 degrees since the target degree is based on the servo angle
    motor1.setTarPWM(ANGLE_TO_MOTOR_MULTIPLIER * (targetAngle - 90));
    motor1.loop();
    motor2.loop();

    state = 0;
  }

  Serial.print("State:");
  Serial.print(state);
  Serial.print(", Distance:");
  Serial.print(distance);
  Serial.print(", FrontDistance:");
  Serial.print(frontDistance);
  Serial.print(", Servo Angle:");
  Serial.print(angle);
  Serial.print(", Target Angle:");
  Serial.print(targetAngle);
  
  
  Serial.println();
}

