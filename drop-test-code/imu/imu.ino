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
Matrix<3,1> vel = {0, 0, 0};
const Matrix<3,1> GRAVITY = {0, 0, -1};
const Matrix<3,1> UNIT = {0, 1, 0};
unsigned long last_time = 0;

void setup() {
  Serial.begin(115200);

  Wire.begin();

  if(imu.begin(0x6A, 0x1C) == false){
    Serial.println("IMU bad");
    while(1);
  }
  imu.setGyroODR(4);
  imu.setAccelODR(4);

  Serial.println("Accumulating data...");
  const int CALIBRATE_SECONDS = 60;
  const int CALIBRATE_LOOP_DELAY = 100;
  const int ITERATIONS = CALIBRATE_SECONDS * (1000 / CALIBRATE_LOOP_DELAY);
  for(int i = 1; i <= ITERATIONS ; i++){
    if (imu.gyroAvailable() && imu.accelAvailable()) {
      imu.readGyro();
      imu.readAccel();
    }
    
    //imu.readMag();
    delay(CALIBRATE_LOOP_DELAY);
    Serial.print(i);
    Serial.print(" / ");
    Serial.println(ITERATIONS );
  }
  Serial.println("Calibrating...");
  //imu.calibrate(true);
  Serial.println("Calibrated");
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
  unsigned long now = micros();
  last_time = now;
}

int counter = 0;
const double ROT_MUL = -1 * PI / 180.0;
const double G = 9.81;

Matrix<3,1> reading_g = {0, 0, 0};
Matrix<3,1> reading_m = {0, 0, 0};
Matrix<3,1> reading_a = {0, 0, 0};
double heading = 0.0;

const float DECLINATION = -2.12 * PI / 180.0; // Declination https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#declination
double getHeading(float x, float y, float z) {
  float heading;
  if (y == 0)
    heading = (x < 0) ? PI : 0;
  else
    heading = atan2(x, y);

  heading -= DECLINATION;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  
  return heading;
}

void loop() {
  const unsigned long now = micros();
  const double elapsed = (now - last_time) / 1000000.0;
  
  last_time = now;
  if (!imu.gyroAvailable() || !imu.accelAvailable()) {
    return;
  }
  
  imu.readGyro();
  imu.readAccel();
  
  
  reading_g = {imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz)};
  Rotation D_inv = exp(reading_g * elapsed * ROT_MUL);
  R_inv *= D_inv;
  
  reading_a = {imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az)};
  vel += (reading_a + R_inv * GRAVITY) * (elapsed * G);
  pos += vel * elapsed;

  if(imu.magAvailable()) {
    imu.readMag();
    float x = imu.calcMag(imu.mx);
    float y = imu.calcMag(imu.my);
    float z = imu.calcMag(imu.mz);
    reading_m = {x, y, z};
    heading = getHeading(x, y, z);
    
  }

  counter += 1;
  if(counter % 250 == 0) {
    Serial.println(1.0 / elapsed);
    Serial << "Pointing y: " << R_inv * UNIT << "\n";
    Serial << "Velocity: " << vel << "\n";
    Serial << "Position: " << pos << "\n";
    Serial << "Mag: " << reading_m << "\n";
    Serial << "Acc: " << reading_a << "\n";
    Serial << "Gyr: " << reading_g << "\n";
    Serial.println(heading * 180.0 / PI);
  }
}
