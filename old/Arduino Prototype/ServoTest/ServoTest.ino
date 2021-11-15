/*
  Servo Basic
  Understanding the basic of servo in arduino projects. Sweep servo to 0, 90, 180, 90 and 0.

  Coded by: arduinoprojects101.com
*/

// include the library code:
#include <Servo.h>

Servo myservo;  // create servo object to control a servo

void setup(){
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop(){
  myservo.write(0);     // sets the servo at 0 degree position
  delay(1000);          // waits for the servo to get there
  myservo.write(90);    // sets the servo at 90 degree position
  delay(1000);          // waits for the servo to get there
  myservo.write(180);   // sets the servo at 180 degree position
  delay(1000);          // waits for the servo to get there
  myservo.write(90);    // sets the servo at 90 degree position
  delay(1000);          // waits for the servo to get there
}
