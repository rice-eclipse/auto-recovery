#include <Servo.h>
const int BIAS = -4;
Servo servo; 
void setup() 
{
  Serial.begin(9600);
  servo.attach(9); //attach servo to pin 9
  servo.write(90); //set servo to default center position
  Serial.println("started");
}
 
void loop()
{
  servo.write(90+BIAS);
  delay(2000);
  servo.write(160+BIAS);
  delay(2000);
  servo.write(90+BIAS);
  delay(2000);
  servo.write(20+BIAS);
  delay(2000);
}