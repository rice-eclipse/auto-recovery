#include <Servo.h>
int bias = -4;
Servo servo; 
void setup() 
{
  while (!Serial);
  Serial.begin(9600);
  servo.attach(9); //attach servo to pin 9
  servo.write(90+bias); //set servo to default center position
}
 
void loop()
{
  servo.write(90+bias);
  delay(2000);
  servo.write(160+bias);
  delay(2000);
  servo.write(90+bias);
  delay(2000);
  servo.write(20+bias);
  delay(2000);
}
