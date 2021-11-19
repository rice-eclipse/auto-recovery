#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <Servo.h>
#include <EEPROM.h>

unsigned long start_time = 0;

#define SRAM_FRAMES 56
#define EEPROM_FRAMES 54

LSM9DS1 imu;
Servo servo;

struct Frame {
  int16_t imu[9];
};
Frame buff[SRAM_FRAMES];
int idx = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  servo.attach(9);
  if(imu.begin(0x6A, 0x1C) == false){
    Serial.println("IMU bad");
    while(1);
  }
  Serial.println("Calibrating");
  imu.calibrate();
  Serial.println("Calibrated");
  imu.setGyroODR(1);
  imu.setAccelODR(1);
  imu.setMagODR(3);
  start_time = micros();
  Serial.println("Drop in 10 sec...");
  delay(10000);
}

void loop() {
  unsigned long now = micros() - start_time;
  if (!imu.magAvailable() || !imu.gyroAvailable() || !imu.accelAvailable()) {
    Serial.println("WTF");
    return;
  }
  imu.readMag();
  imu.readGyro();
  imu.readAccel();
  Frame frame = {
    .imu = {imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, imu.mx, imu.my, imu.mz},
  };
  if (idx < EEPROM_FRAMES) {
    EEPROM.put(sizeof(Frame) * idx, frame);
  }
  else {
    buff[ idx - EEPROM_FRAMES ] = frame;
  }
  Serial.println(idx);
  if (idx == EEPROM_FRAMES + SRAM_FRAMES) {
    while(1) {
      Serial.println("--------");
      for (int i = 0; i < EEPROM_FRAMES; i++) {
        Frame frame;
        EEPROM.get(sizeof(Frame) * i, frame);
        for (int j = 0; j < 9; j++){
          Serial.print(frame.imu[j]);
          Serial.print(", ");
        }
        Serial.println();
      }
      for (int i = 0; i < SRAM_FRAMES; i++) {
        for (int j = 0; j < 9; j++){
          Serial.print(buff[i].imu[j]);
          Serial.print(", ");
        }
        Serial.println();
      }
      Serial.println("--------");
      delay(30000);
    }
  }
  idx++;
  delay(400);
}
