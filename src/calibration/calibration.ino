#include <Wire.h>
#include <ICM20948_WE.h>
#include "SAMDTimerInterrupt.h"
#include "SAMD_ISR_Timer.h"
#define ICM20948_ADDR 0x69
#define INTERVAL_MS      2 //period of data collection (milliseconds)
#define NUM_SAMPLES      512 //number of samples to collect


// float data[MAX_T*1000/INTERVAL][N];

ICM20948_WE imu = ICM20948_WE(ICM20948_ADDR);

int count;
bool activate = false;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);

  pinMode(1, OUTPUT);
  digitalWrite(1, LOW);
  while (!Serial) {}

  while(!imu.init()){
    Serial.println("ICM20948 does not respond... Trying again");
    Serial.println(imu.whoAmI());
    delay(1000);
  }
  Serial.println("ICM20948 connected!");


  if(!imu.initMagnetometer()){
    Serial.println("ICM20948 Magnetometer does not respond");
  }

  Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  delay(1000);
  imu.autoOffsets();
  imu.setAccRange(ICM20948_ACC_RANGE_4G);
  imu.setAccDLPF(ICM20948_DLPF_2);
  imu.setAccSampleRateDivider(1);

  imu.setGyrRange(ICM20948_GYRO_RANGE_250);  
  imu.setGyrDLPF(ICM20948_DLPF_2);

  imu.setMagOpMode(AK09916_CONT_MODE_100HZ); 

  count = 0;
}

void loop() {
  if(Serial.available() > 0){
    activate = Serial.read() == 49;
    Serial.read();
  }
  if (count > NUM_SAMPLES || !activate) { return; }
  count += 1;

  imu.readSensor();
  xyzFloat acc = imu.getGValues();
  
  xyzFloat mag = imu.getMagValues();

  float g = 9.81;
  Serial.print(acc.x*g);
  Serial.print(",");
  Serial.print(acc.y*g);
  Serial.print(",");
  Serial.print(acc.z*g);
  Serial.print(",");
  Serial.print(",");
  Serial.print(mag.x);
  Serial.print(",");
  Serial.print(mag.y);
  Serial.print(",");
  Serial.println(mag.z);
  delay(100);
}
