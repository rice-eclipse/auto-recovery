// Arduino9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Arduino9x_TX
  
#include <SPI.h>
#include <RH_RF95.h>
#include <Servo.h>
 
#define RFM95_CS 4
#define RFM95_RST 5
#define RFM95_INT 3
 
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 434.0
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
 
// Blinky on receipt
#define LED 13

Servo servo; //servo object to control servo
int pos = 90; //initial servo position
 
void setup() 
{
  while (!Serial);
  Serial.begin(9600);
  servo.attach(9); //attach servo to pin 9
  //servo.write(pos); //set servo to default center position
}
 
void loop()
{
  servo.write(90);
  delay(2000);
  servo.write(160);
  delay(2000);
  servo.write(90);
  delay(2000);
  servo.write(20);
  delay(2000);
}
