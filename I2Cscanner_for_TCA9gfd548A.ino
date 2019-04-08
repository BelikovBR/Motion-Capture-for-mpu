    /**
     * TCA9548 I2CScanner.pde -- I2C bus scanner for Arduino
     *
     * Based on code c. 2009, Tod E. Kurt, http://todbot.com/blog/
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
    void setup()
    {
        while (!Serial);
        delay(1000);
     
        Wire.begin();
        
        Serial.begin(115200);
        Serial.println("\nTCAScanner ready!");
        
       //for (uint8_t t=0; t<8; t++) {
          tcaselect(7);
          
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
 
 
          //Serial.print("TCA Port #"); Serial.println(t);
     
       /*   for (uint8_t addr = 0; addr<=127; addr++) {
            if (addr == TCAADDR) continue;
          
            uint8_t data;
            if (! twi_writeTo(addr, &data, 0, 1, 1)) {
               Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
            }
          }
          
       // }
        Serial.println("\ndone");*/
    }
     
void loop() {

  //Código do MPU6050.

  //Ждем готовоности и прерывания - ЗДЕСЬ ПОСТАВИМ ЗАДЕРЖКУ
  delay(1);
/*  while (!mpuInterrupt) { 

    //Insira aqui o código que normalmente viria dentro da função "loop" que tudo funcionará normalmente.

  }

  //Código do MPU6050.
  mpuInterrupt = false;
*/  
  
  // считали данные с MPU
  fifoCount = mpu.getFIFOCount();

  if (fifoCount == 1024) {

    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

  }
  else {
    //Если полученные данные меньше пакета, то задержка и считываем даные заново
    while (fifoCount < packetSize) {

     // Serial.print(fifoCount); Serial.print(" ");
      delay(1);
      fifoCount = mpu.getFIFOCount();
    }
    

   // Serial.print(fifoCount); Serial.print(" * ");
    
    if (fifoCount % packetSize != 0) {
//    if ((fifoCount % packetSize != 0) || (fifoCount < packetSize)) {

      mpu.resetFIFO();
      Serial.print("resetFIFO");
    }
    else {

      while (fifoCount >= packetSize) {

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

      }

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      unsigned long T;
      static unsigned long LastT ;
      T = millis();
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180 / PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / PI);
      Serial.print("\t");
      Serial.print(ypr[2] * 180 / PI);
      Serial.print("\t");
 //     Serial.print(T);
   //   Serial.print("\t");
     // Serial.print(T - LastT);
      Serial.println();
      LastT = T;

    }

  }

}
