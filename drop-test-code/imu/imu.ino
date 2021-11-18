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
const Matrix<3,1> GRAVITY = {0, 0, -1};
const Matrix<3,1> UNIT = {0, 1, 0};
unsigned long last_time_gyro = 0;
unsigned long last_time_accel = 0;
unsigned long last_time_loop = 0;

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
  int seconds = 2;
  for(int i = 0; i < seconds; i++){
    delay(1000);
    Serial.print(i);
    Serial.print(" / ");
    Serial.println(seconds);
  }
  Serial.println("Calibrating uwu...");
  imu.calibrate();
  Serial.println("Calibrated owo");
  Serial.print("Gyro Bias: ");
  for(int i = 0; i < 3; i++){
    Serial.print(imu.gBias[i]);
    Serial.print(" ");
  }
  Serial.print("\n Acceleration Bias: ");
  for(int i = 0; i < 3; i++){
    Serial.print(imu.aBias[i]);
    Serial.print(" ");
  }
  Serial.print("\n");
  last_time_gyro = micros();
  last_time_accel = micros();
  last_time_loop = micros();

  imu.setGyroODR(6);
  imu.setAccelODR(6);
}

int counter = 0;

const double ONE_MILLIONTH = 1 / 1000000.0;
const double MUL = -1 * PI / 180000000.0;
const double G = 9.81;

void loop() { 
  if(imu.gyroAvailable()){
    //imu.readGyro();
    
    Matrix<3,1> reading_g = {imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz)};

    unsigned long now_g = micros();
    unsigned long elapsed_g = now_g - last_time_gyro;
    last_time_gyro = now_g;
    reading_g *= MUL * elapsed_g;
    Rotation D_inv = exp(reading_g);
    R_inv = R_inv * D_inv;
  }else{
    Serial.println("Gyro cringe");
  }
  
  if(imu.accelAvailable()){
    //imu.readAccel();
    Matrix<3,1> reading_a = {imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az)};
    //reading_a = (R_inv * reading_a + GRAVITY) * G;
    reading_a = (reading_a + GRAVITY) * G;
    
    unsigned long now_a = micros();
    unsigned long elapsed_a = now_a - last_time_accel;;
    last_time_accel = now_a;
    velocity += reading_a * elapsed_a * ONE_MILLIONTH;
    pos += velocity * elapsed_a * ONE_MILLIONTH;
  }else{
    Serial.println("Accel cringe");
  }
  counter += 1;
  if(counter % 250 == 0) {
    unsigned long now = micros();
    unsigned long elapsed = now - last_time_loop;
    Serial.println(elapsed);
    last_time_loop = now;
    Serial << "Acceleration: " << imu.calcAccel(imu.ax) << " " << imu.calcAccel(imu.ay) << " " << imu.calcAccel(imu.az) << "\n";
    Serial << "Pointing y: " << R_inv * UNIT << "\n";
    Serial << "Velocity: " << velocity << "\n";
    Serial << "Position: " << pos << "\n";
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
