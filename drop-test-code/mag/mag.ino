#include <Geometry.h>
#include <BasicLinearAlgebra.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;

void setup() {
  Serial.begin(115200);

  Wire.begin();

  if(imu.begin(0x6A, 0x1C) == false){
    Serial.println("IMU bad");
    while(1);
  }
  Serial.println("Calibrating");
  imu.calibrate();
  Serial.println("Calibrated");
  imu.setGyroODR(4);
  imu.setAccelODR(4);
  imu.setMagODR(7);
  //Serial.println("MagX\tMagY\tMagZ\tHeading");
  Serial.println("Heading");
}

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
  if (imu.magAvailable()) {
    imu.readMag();
    float x = imu.calcMag(imu.mx);
    float y = imu.calcMag(imu.my);
    float z = imu.calcMag(imu.mz);
//    Serial.print(x);
//    Serial.print("\t");
//    Serial.print(y);
//    Serial.print("\t");
//    Serial.print(z);
//    Serial.print("\t");
    Serial.print(getHeading(x, y ,z));
    Serial.println("");
  }

}
