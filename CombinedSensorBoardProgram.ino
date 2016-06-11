#include <Event.h>
#include <Timer.h>
#include <SoftwareSerial.h>
//#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_LSM303_U.h>
//#include <Adafruit_BMP085_U.h>
//#include <Adafruit_L3GD20_U.h>
//#include <Adafruit_10DOF.h>

int ExpTime, SampTime, SenseConfig;
Timer SampTimer;

void setup() {
  // put your setup code here, to run once:
  //Setup Serial

  Serial.begin(9600);
  interrupts();
}

void loop() {
  // put your main code here, to run repeatedly:
  while(!Serial.available());
  if(Serial.available())
  {
    int ExpCounter = 0;   //keeps track of how many samples have been collected
    int ExpIter = 0;      //total number of samples to be taken
    ExpTime = Serial.parseInt();
    SampTime = Serial.parseInt();
    SensorConfig = Serial.parseInt();
    ExpIter = trunc(ExpTime/SampTime);

    for(ExpCounter = 0; ExpCounter<ExpIter; ExpCounter++)
    {
        SampTimer.every(SampTime, takeReading(SensorConfig));
        //transmit data
      
    }
   
  }
  
}


void takeReading(int Config)
{
    switch (Config)
  {
  
                //sen1      sen2      sen3      sen4      sen5
                //-------------------------------------------------------------------------------------
    case 0x0000:  //null case
      //Serial.println("null case");
      break;
    case 0x0001:  //gyro
      //Serial.println("gyro");
      break;
    case 0x0002:  //pressure
      //Serial.println("pressure");
      break;
    case 0x0003:  //pressure, gyro
      //Serial.println("pressure, gyro");
      break;
    case 0x0004:  //alt
      //Serial.println("alt");
      break;
    case 0x0005:  //alt,      gyro
      //Serial.println("alt, gyro");
      break;
    case 0x0006:  //alt,      pressure
      //Serial.println("alt, temp");
      break;
    case 0x0007:  //alt,      pressure, gyro
      //Serial.println("alt, pressure, gyro");
      break;
    case 0x0008:  //GPS
      //Serial.println("GPS");
      break;
    case 0x0009:  //GPS,      gryo
      //Serial.println("GPS, gyro");
      break;
    case 0x000A:  //GPS,      pressure
      //Serial.println("GPS, pressure");
      break;
    case 0x000B:  //GPS,      pressure, gyro
      //Serial.println("GPS, pressure, gyro");
      break;
    case 0x000C:  //GPS,      alt
      //Serial.println("GPS, alt");
      break;
    case 0x000D:  //GPS,      alt,      gyro
      //Serial.println("GPS, alt, gyro");
      break;
    case 0x000E:  //GPS,      alt,      pressure
      //Serial.println("GPS, alt, pressure");
      break;
    case 0x000F:  //GPS,      alt,      pressure, gyro
      //Serial.println("GPS, alt, pressure, gyro");
      break;
    case 0x0010:  //Temp
      //Serial.println("Temp");
      break;
    case 0x0011:  //Temp,     gyro
      //Serial.println("Temp, gyro");
      
      break;
    case 0x0012:  //Temp,     pressure 
      //Serial.println("Temp, pressure");
      break;
    case 0x0013:  //Temp,     pressure, gyro
      //Serial.println("Temp, pressure, gyro");
      break;
    case 0x0014:  //Temp,     alt  
      //Serial.println("Temp, alt");
      break;
    case 0x0015:  //Temp,     alt,      gyro  
      //Serial.println("Temp, alt, gyro");
      break;
    case 0x0016:  //Temp,     alt,      pressure
      //Serial.println("Temp, alt, pressure");
      break;
    case 0x0017:  //Temp,     alt,      pressure, gyro
      //Serial.println("Temp, alt, pressure, gyro");
      break;
    case 0x0018:  //Temp,     GPS
      //Serial.println("Temp, GPS");
      break;
    case 0x0019:  //Temp,     GPS,      gyro
      //Serial.println("Temp, GPS, gyro");
      break;
    case 0x001A:  //Temp,     GPS,      pressure
      //Serial.println("Temp, GPS, pressure");
      break;
    case 0x001B:  //Temp,     GPS,      pressure, gyro
      //Serial.println("Temp, GPS, pressure, gyro");
      break;
    case 0x001C:  //Temp,     GPS,      alt
      //Serial.println("Temp, GPS, alt");
      
      break;
    case 0x001D:  //Temp,     GPS,      alt,      gyro
      //Serial.println("Temp, GPS, alt, gyro");
      break;
    case 0x001E:  //Temp,     GPS,      alt,      pressure
      //Serial.println("Temp, GPS, alt, pressure");
      break;
    case 0x001F:  //Temp,     GPS,      alt,      pressure, gyro
      //Serial.println("Temp, GPS, alt, pressure, gyro");
      break;
    
    default:
      //Serial.println("uhh, what?");
      
      
      break;      //replace with specific return() for the combination of data packets


}

