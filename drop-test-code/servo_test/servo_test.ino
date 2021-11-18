#include <Servo.h>

Servo servo; 
void setup() 
{
  while (!Serial);
  Serial.begin(9600);
  servo.attach(9); //attach servo to pin 9
  servo.write(90); //set servo to default center position
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
