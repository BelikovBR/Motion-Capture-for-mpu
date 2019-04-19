    /**
     * TCA9548 I2CScanner.pde -- I2C bus scanner for Arduino
     *
     * Based on code c. 2009, Tod E. Kurt, http://todbot.com/blog/
     *
     *  Исходное положение для корректировки
     *  Корректировка
     *  Опрос датчиков 
     *  v1: после опроса сразу отправлять на сервер
     *  v2: формировать пакет и отправлять по беспроводной связи
     *  Отправка
     *
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
    //int t_pins[] = {4,5};
    
    #include "Wire.h"
    extern "C" { 
    #include "utility/twi.h"  // from Wire library, so we can do bus scanning
    }
     
    #define TCAADDR 0x70
     
    void tcaselect(uint8_t i) {
      if (i > 7) return;
     
      Wire.beginTransmission(TCAADDR);
      Wire.write(1 << i);
      Wire.endTransmission();  
    }
     
     
    // standard Arduino setup()
void setup() {
  while (!Serial);
    delay(1000); 
    Wire.begin();
    Serial.begin(115200); 
    tcaselect(4);
    Wire.begin();
    TWBR = 24;
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setXAccelOffset(-4232);
    mpu.setYAccelOffset(-706);
    mpu.setZAccelOffset(1729);
    mpu.setXGyroOffset(173);
    mpu.setYGyroOffset(-94);
    mpu.setZGyroOffset(37);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
}
     
void loop() {
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
        // Serial.print(NUM);
         Serial.print(ypr[0], 4);
         Serial.print(" ");
         Serial.print(ypr[1], 4);
         Serial.print(" ");
         Serial.println(ypr[2], 4);
         delay(10); 
      }
  }
}
