//#include <helper_3dmath.h>
//#include <MPU6050.h>
#include <MPU6050_6Axis_MotionApps20.h>
//#include <MPU6050_6Axis_MotionApps_V6_12.h>
//#include <MPU6050_9Axis_MotionApps41.h>
#include <I2Cdev.h>

MPU6050 mpu_first;
MPU6050 mpu_second;

uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

#include "Wire.h"
extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR 0x71

// i- номер датчика/смещение, j - номер tca
void tcaselect(uint8_t i, uint8_t j) {
  if (i > 7) return;
  if(j==1){
    Wire.beginTransmission(0x70);
    Wire.write(1 << i);    
    Wire.endTransmission();
      Serial.println("OK0");
  }
  else if (j==2){
    Wire.beginTransmission(0x71);
    Wire.write(1 << i);    
    Wire.endTransmission();
    Serial.println("OK1");
  }

}

void tcaclose(uint8_t j){
  if(j==1){
    Wire.beginTransmission(0x70);
    Wire.write(0);    
    Wire.endTransmission();
      Serial.println("end_OK1");
  }
  else if (j==2){
    Wire.beginTransmission(0x71);
    Wire.write(0);    
    Wire.endTransmission();
    Serial.println("end_OK2");
  }
}

// standard Arduino setup()
void setup() {
  while (!Serial);
  delay(1000);
  Wire.begin();
  Serial.begin(115200);
  
  tcaselect(0,1);
  TWBR = 24;
  mpu_first.initialize();
  mpu_first.dmpInitialize();
  mpu_first.setXAccelOffset(-3433);
  mpu_first.setYAccelOffset(308);
  mpu_first.setZAccelOffset(1845);
  mpu_first.setXGyroOffset(49);
  mpu_first.setYGyroOffset(33);
  mpu_first.setZGyroOffset(67);
  mpu_first.setDMPEnabled(true);
  packetSize = mpu_first.dmpGetFIFOPacketSize();
  
  tcaclose(1);
  
  tcaselect(0,2);
  TWBR = 24;
  mpu_second.initialize();
  mpu_second.dmpInitialize();
  mpu_second.setXAccelOffset(-3433);
  mpu_second.setYAccelOffset(308);
  mpu_second.setZAccelOffset(1845);
  mpu_second.setXGyroOffset(49);
  mpu_second.setYGyroOffset(33);
  mpu_second.setZGyroOffset(67);
  mpu_second.setDMPEnabled(true);
  packetSize = mpu_second.dmpGetFIFOPacketSize();
  
  tcaclose(2);
}

void getDataMpu(int j) {
  delay(1);
  
  if (j==1) {
      fifoCount = mpu_first.getFIFOCount();

  if (fifoCount == 1024) {
    mpu_first.resetFIFO();
    // Serial.println(F("FIFO overflow!"));
  }
  else {
    while (fifoCount < packetSize) {
      // Serial.print(fifoCount); Serial.print(" ");
      delay(1);
      fifoCount = mpu_first.getFIFOCount();
    }
    if (fifoCount % packetSize != 0) {
      mpu_first.resetFIFO();
      //  Serial.print("resetFIFO");
    }
    else {
      while (fifoCount >= packetSize) {
        mpu_first.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
      }
      mpu_first.dmpGetQuaternion(&q, fifoBuffer);
      mpu_first.dmpGetGravity(&gravity, &q);
      mpu_first.dmpGetYawPitchRoll(ypr, &q, &gravity);

      Serial.print("n");
      Serial.print("1");
      Serial.print(" ");
      Serial.print(q.w);
      Serial.print(" ");
      Serial.print(q.x);
      Serial.print(" ");
      Serial.print(q.y);
      Serial.print(" ");
      Serial.println(q.z);

      delay(10);
        }
      }
  }
  else {
    fifoCount = mpu_first.getFIFOCount();

  if (fifoCount == 1024) {
    mpu_first.resetFIFO();
    // Serial.println(F("FIFO overflow!"));
  }
  else if (j==2){
    
    while (fifoCount < packetSize) {
      // Serial.print(fifoCount); Serial.print(" ");
      delay(1);
      fifoCount = mpu_second.getFIFOCount();
    }
    if (fifoCount % packetSize != 0) {
      mpu_second.resetFIFO();
      //  Serial.print("resetFIFO");
    }
    else {
      while (fifoCount >= packetSize) {
        mpu_second.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
      }
      mpu_second.dmpGetQuaternion(&q, fifoBuffer);
      mpu_second.dmpGetGravity(&gravity, &q);
      mpu_second.dmpGetYawPitchRoll(ypr, &q, &gravity);

      Serial.print("n");
      Serial.print("2");
      Serial.print(" ");
      Serial.print(q.w);
      Serial.print(" ");
      Serial.print(q.x);
      Serial.print(" ");
      Serial.print(q.y);
      Serial.print(" ");
      Serial.println(q.z);

      delay(10);
        }
       }
     }
}

void loop() {
  tcaselect(0,1);
  Serial.println("OK_tcaselect (0,1)");
  delay(10);
  getDataMpu(1);
  Serial.println("OK-OK_getDataMpu - 0,1"); 
  tcaclose(1);
  
  tcaselect(0,2);
  Serial.println("OK_tcaselect (0,2)");
  delay(10);
  getDataMpu(2);
  Serial.println("OK_getDataMpu - 0,2");
  tcaclose(2);

  
}
