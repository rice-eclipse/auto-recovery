#include <Geometry.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <Servo.h>
#include <BasicLinearAlgebra.h>

#define G_TO_FT 32.15

int cs_m = 11;
int cs_ag = 10;
int servo_pin = 9;

using namespace Geometry;
using namespace BLA;

LSM9DS1 imu;
Matrix<3,3> R_inv = {1, 0, 0, 0, 1, 0, 0, 0, 1};
Matrix<3,1> pos = {0, 0, 0};
Matrix<3,1> velocity = {0, 0, 0};
unsigned long last_time = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);

  Wire.begin();

  if(imu.begin(0x6A, 0x1C) == false){
    Serial.println("IMU bad");
    while(1);
  }

  Serial.println("Accumulating data...");
  for(int i = 0; i < 120; i++){
    delay(1000);
    Serial.print(i);
    Serial.println(" / 120 seconds");
  }
  Serial.println("Calibrating uwu...");
  imu.calibrate();
  Serial.println("Calibrated owo");
  last_time = micros();
}

int counter = 0;

const double MUL = -1 * PI / 180000000.0;

void loop() {
  if(imu.gyroAvailable()){
    imu.readGyro();
    
    Matrix<3,1> reading = {imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz)};

    unsigned long now = micros();
    int elapsed = now - last_time;
    last_time = now;
    reading *= MUL * elapsed;
    Rotation D_inv = exp(reading);
    R_inv = R_inv * D_inv;
    Matrix<3, 1>  unit = {0, 1, 0};
    counter += 1;
    if(counter % 100 == 0) {
      Serial << reading << "\n";
      Serial << R_inv * unit << "\n";
    }
  }
  if(imu.accelAvailable()){
    imu.readAccel();
  }

  // Adjust the rotation transformation matrix and vector
  /*
  int time_difference = millis() - last_time;
  last_time = millis();
  Matrix<3,1> r = {imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz)};
  Matrix<3,1> sum = {0,0,0};
  for(int i = 0; i < 2000; i++){
    Matrix<3,1> r = {imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz)};
    sum = sum + r;
    delay(1);
  }
  sum = sum / 2000;
  Serial << sum << "\n";
  */
  //Serial.print(imu.calcGyro(imu.gx));
  //Serial.print(" ");
  //Serial.print(imu.calcGyro(imu.gy));
  //Serial.print(" ");
  //Serial.println(imu.calcGyro(imu.gz));
  //delay(10);

  /*
  r *= -1 * PI / 180.0 * time_difference / 1000;
  Rotation D_inv = exp(r);
  R_inv = R_inv * D_inv;
  Matrix<3, 1>  unit = {0, 1, 0};
  delay(100);
  */
  
  // Adjust the velocity and position vectors

  /*
  Matrix<3,1> acceleration = {imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az)};
  acceleration = R_inv * acceleration * G_TO_FT;
  velocity = velocity + acceleration * time_difference;
  pos = pos + velocity * time_difference;
  Serial << pos << "\n";
  */
}
