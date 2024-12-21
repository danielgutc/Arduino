#include <Wire.h>
#include <time.h>
#include <stdlib.h>
#include <Servo.h>

#define SENSOR_1    1
#define WIRE_START  -1
#define SERVO_SPEED 0.1

Servo servo;
float angle = 0;
int direction = 1;
int x = 0;

void setup() {
  // I2C messaging
  Wire.begin(); 
  Wire.setWireTimeout();
  Serial.begin(115200);

  // Servo
  servo.attach(9);
}

void loop() {
  // Servo motor
  angle = angle + (SERVO_SPEED * direction);
  if (angle > 180 || angle < 0) {
    direction = direction * -1;
  }
  servo.write(angle);

  // Messaging
  Wire.beginTransmission(1); // transmit to device #1
  Wire.write((int) angle);
  Wire.endTransmission();    // stop transmitting
 
  Serial.print(", Angle: ");
  Serial.print(angle);
  Serial.println();
  
  delay(10);

}
