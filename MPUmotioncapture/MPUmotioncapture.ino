/* 
 *  10.05.2019 12:22 - Программа с двумя датчиками заработала удовлетворительно
*/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;
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

void tcaselect(uint8_t i, uint8_t j) {
  if (i > 7) return;
  if(j==0){
    Wire.beginTransmission(0x70);
    Wire.write(1 << i);    
    Wire.endTransmission();
      Serial.println("OK0");
  }
  else if (j==1){
    Wire.beginTransmission(0x71);
    Wire.write(1 << i);    
    Wire.endTransmission();
    Serial.println("OK1");
  }

}

// standard Arduino setup()
void setup() {
  while (!Serial);
  delay(1000);
  Wire.begin();
  Serial.begin(115200);
  
  tcaselect(2,0);
  TWBR = 24;
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setXAccelOffset(-3433);
  mpu.setYAccelOffset(308);
  mpu.setZAccelOffset(1845);
  mpu.setXGyroOffset(49);
  mpu.setYGyroOffset(33);
  mpu.setZGyroOffset(67);
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();

  tcaselect(2,1);
  TWBR = 24;
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setXAccelOffset(-3433);
  mpu.setYAccelOffset(308);
  mpu.setZAccelOffset(1845);
  mpu.setXGyroOffset(49);
  mpu.setYGyroOffset(33);
  mpu.setZGyroOffset(67);
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
}

void getDataMpu(int j) {
  delay(1);
  fifoCount = mpu.getFIFOCount();

  if (fifoCount == 1024) {
    mpu.resetFIFO();
    // Serial.println(F("FIFO overflow!"));
  }
  else {
    //Если полученные данные меньше пакета, то задержка и считываем даные заново
    while (fifoCount < packetSize) {
      // Serial.print(fifoCount); Serial.print(" ");
      delay(1);
      fifoCount = mpu.getFIFOCount();
    }
    if (fifoCount % packetSize != 0) {
      mpu.resetFIFO();
      //  Serial.print("resetFIFO");
    }
    else {
      while (fifoCount >= packetSize) {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
      }
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      Serial.print("n");
      Serial.print(j);
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

void loop() {
  tcaselect(2,0);
  Serial.println("OK2");
  getDataMpu(2,1);
  Serial.println("OK3");
}
