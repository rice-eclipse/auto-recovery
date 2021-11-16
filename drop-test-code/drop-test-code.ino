#include <Geometry.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <Servo.h>
#include <BasicLinearAlgebra.h>

int cs_m = 11;
int cs_ag = 10;
int servo_pin = 9;

using namespace Geometry;
using namespace BLA;

int last_time = millis();

LSM9DS1 imu;
Matrix<3,3> R_inv = {1, 0, 0, 0, 1, 0, 0, 0, 1};

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

  Matrix<3,1> A = {0.0, 0.0, 3.14159};
  AngularVelocity w(A);
  Rotation R = exp(w);
  Serial << R << "\n";
  AngularVelocity also_w = log(R);

}

void loop() {
  if(imu.gyroAvailable()){
    imu.readGyro();
  }
  if(imu.accelAvailable()){
    imu.readAccel();
  }
  int time_difference = millis() - last_time;
  last_time = millis();
  Matrix<3,1> r = (imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz));
  r *= -1 * PI / 180.0 * time_difference / 1000;
  Rotation D_inv = exp(r);
  R_inv = R_inv * D_inv;
  Matrix<3, 1>  unit = {0, 1, 0};
  Serial << R_inv * unit << "\n";
}
