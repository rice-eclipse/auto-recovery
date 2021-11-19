#include <Geometry.h>
#include <BasicLinearAlgebra.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;

unsigned long last_time = 0;

void setup() {
  Serial.begin(115200);

  Wire.begin();

  if(imu.begin(0x6A, 0x1C) == false){
    Serial.println("IMU bad");
    while(1);
  }
  imu.calibrate();
  imu.setGyroODR(4);
  imu.setAccelODR(4);
  imu.setMagODR(7);
  last_time = micros();
  Serial.println("GyroX\tGyroY\tGyroZ");
}

unsigned long counter = 0;
float sum_x = 0;
float sum_y = 0;
float sum_z = 0;
void loop() {
  if (imu.gyroAvailable()) {
    imu.readGyro();
    sum_x += imu.calcGyro(imu.gx);
    sum_y += imu.calcGyro(imu.gy);
    sum_z += imu.calcGyro(imu.gz);
    if (counter % 100 == 0) {
      Serial.print(sum_x / 100.0);
      Serial.print("\t");
      Serial.print(sum_y / 100.0);
      Serial.print("\t");
      Serial.print(sum_z / 100.0);
      Serial.println("");
      sum_x = 0;
      sum_y = 0;
      sum_z = 0;
    }
    counter += 1;
    
    unsigned long now = micros();
    //Serial.print(1000000.0 / (now - last_time));
    last_time = now;
  }

}
