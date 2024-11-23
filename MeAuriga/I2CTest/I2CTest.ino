#include <Wire.h>

String receivedMessage = "";

void setup() 
{
  // put your setup code here, to run once:
  Wire.begin(1);
  Serial.begin(115200);
  Serial.println("nI2C Scanner");
  Wire.onReceive(receiveEvent);
}

void receiveEvent(int howMany) 
{
  while (Wire.available()) 
  {
    char c = Wire.read(); // Read each byte from the buffer
    receivedMessage += c; // Append to the string
  }
}

void loop() 
{
  if (receivedMessage.length() > 0) 
  {
    Serial.println("Message received: " + receivedMessage);
    receivedMessage = ""; // Clear the message after processing
  }
}