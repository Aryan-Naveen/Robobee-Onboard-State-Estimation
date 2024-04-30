#include <Wire.h>
#include <ICM20948_WE.h>
#include "SAMDTimerInterrupt.h"
#include "SAMD_ISR_Timer.h"
#define ICM20948_ADDR 0x69
#define INTERVAL_MS      2 //period of data collection (milliseconds)

const int N = 10; // num measurements
const int MAX_T = 10; // max flight time (seconds)

// float data[MAX_T*1000/INTERVAL][N];

unsigned long myTime;


ICM20948_WE imu = ICM20948_WE(ICM20948_ADDR);
SAMDTimer ITimer(TIMER_TC3);

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

  myTime = micros();

}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0){
    activate = Serial.read() == 49;
    Serial.read();
  }

  if(!activate || micros() - myTime < INTERVAL_MS*1000){
    return;
  }
  readSensor();
  myTime = micros();
}

void readSensor(){
  if(!activate){
    return;
  }
  imu.readSensor();
  xyzFloat acc = imu.getGValues();
  
  xyzFloat gyr = imu.getGyrValues();

  xyzFloat mag = imu.getMagValues();

  Serial.print(acc.x);
  Serial.print(" ");
  Serial.print(acc.y);
  Serial.print(" ");
  Serial.print(acc.z);
  Serial.print(" ");
  Serial.print(gyr.x);
  Serial.print(" ");
  Serial.print(gyr.y);
  Serial.print(" ");
  Serial.print(gyr.z);
  Serial.print(" ");
  Serial.print(mag.x);
  Serial.print(" ");
  Serial.print(mag.y);
  Serial.print(" ");
  Serial.print(mag.z);
  Serial.println(" |");
}
