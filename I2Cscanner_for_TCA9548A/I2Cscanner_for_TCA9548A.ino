    /**
     * TCA9548 I2CScanner.pde -- I2C bus scanner for Arduino
     *
     * Based on code c. 2009, Tod E. Kurt, http://todbot.com/blog/
     */

/*
 * Загорается лампа - означает что за последнее некоторое кол-во отсчетов выдаваемые значанения отличаются на не более 1 градуса;
 * Далее нажимаем кнопку на ардуино, которая запускает функцию коллибровки выдаваемых данных относительно начального положения. 
 * 
 *  - Алгоритм работы с системой захвата - 
 *  1. Нажимаем кнопку и встаем в исходное положение, ждем пока лампочка не погаснет.
 *    - пока модель стоит в исходном положении, программа производит корректировку выдаваемых углов.
 *  2. Начинаем захват движения.
 *    - производится цикличный опрос датчиков и формирование пакета данных со всех датчиков за текущий опрос
 *    - отправка пакета на приемник (сервер) при помощи радиосвязи 
 *    - из принимаемого пакета данных извлекается информация каждой точки захвата для визуализации на основе декаротовых координат в трехмерном пространстве
 *  3. Выключение системы 
 */
     
     
    #include "I2Cdev.h"
    #include "MPU6050_6Axis_MotionApps20.h"
    MPU6050 mpu;
    #include "Wire.h"
    extern "C" { 
    #include "utility/twi.h"  // from Wire library, so we can do bus scanning
    }

  #include <SPI.h>                                               // Подключаем библиотеку для работы с шиной SPI
  //#include <nRF24L01.h>                                          // Подключаем файл настроек из библиотеки RF24
 // #include <RF24.h>                                              // Подключаем библиотеку для работы с nRF24L01+
 // RF24 radio(9, 10);
    
    #define TCAADDR 0x70; // адрес коммутатора по умолчанию
    const int NUMB_SENS = 1; // количество датчиков в текущем костюме
    
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
  //  float dataSens[NUMB_SENS*3];//массив данных по 3 значения с одного датчика
   
      
    void tcaselect(uint8_t i) {

     // if (i > 7) // переключаемся коммутатор по адресу 0x71
       //   TCAADDR = 0x71;
    //  else if (i > 15) { // переключаемся коммутатор по адресу 0x72
    //      TCAADDR = 0x72;
    //  }
      
      if (i > 7){   
          return;
      }
            
      Wire.beginTransmission(TCAADDR);
      Wire.write(1 << i);
      Wire.endTransmission();  
    }      
     
    void setup() {
        while (!Serial);
        delay(1000);
        
        Serial.begin(115200);
        Serial.println("\nTCAScanner ready!");
/*
        radio.begin();                                             // Инициируем работу nRF24L01+
        radio.setChannel(5);                                       // Указываем канал передачи данных (от 0 до 127), 5 - значит передача данных осуществляется на частоте 2,405 ГГц (на одном канале может быть только 1 приёмник и до 6 передатчиков)
        radio.setDataRate     (RF24_1MBPS);                        // Указываем скорость передачи данных (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS), RF24_1MBPS - 1Мбит/сек
        radio.setPALevel      (RF24_PA_HIGH);                      // Указываем мощность передатчика (RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_HIGH=-6dBm, RF24_PA_MAX=0dBm)
        radio.openWritingPipe (0xAABBCCDD11LL);                    // Открываем трубу с идентификатором 0xAABBCCDD11 для передачи данных (на одном канале может быть открыто до 6 разных труб, которые должны отличаться только последним байтом идентификатора)
    */
       for (uint8_t t=0; t < NUMB_SENS; t++) {
            tcaselect(t);
          
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

        Serial.println("\ndone");
        }
    }
/*
    // Формирование пакета данных после опроса
    void adddatapacket(uint8_t n_sens, float *ypr){
      //n_sens - номер датчика 
      //ypr - положение датчика в углах

      for (int i = 0; i<3; i++){
        dataSens[n_sens+i] = ypr[i]; 
      }
     
    }
    */
     
void loop() {
  delay(1);

    for (uint8_t t=0; t < NUMB_SENS; t++) {
      tcaselect(t);
       
      fifoCount = mpu.getFIFOCount();
    
      if (fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
      }
      else {
         while (fifoCount < packetSize) {
          delay(1);
          fifoCount = mpu.getFIFOCount();
        }
    
      if (fifoCount % packetSize != 0) {
        // if ((fifoCount % packetSize != 0) || (fifoCount < packetSize)) {
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
  
     // unsigned long T;
     //static unsigned long LastT ;
      //T = millis();
     
      Serial.print("ypr\t");
      Serial.print((ypr[0] * 180 / PI)); //
      Serial.print("\t");
      Serial.print((ypr[1] * 180 / PI));
      Serial.print("\t");
      Serial.print((ypr[2] * 180 / PI));
      Serial.print("\t");
      Serial.println();
    }

    }
       //Serial.print("\nmpu");
       //Serial.print(pins[t]);
       //Serial.println();
       
      //Сохранение данных с датчиков в пакет 
     // adddatapacket(t, ypr);
   }
/*
   //Передача пакета данных за один опрос по радиоканалу
   radio.write(&dataSens, sizeof(dataSens));
   bool ok = txStandBy(); //Returns 0 if failed. 1 if success.
   delay(50);
   */
}
