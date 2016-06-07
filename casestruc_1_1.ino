//June 3, 2016
//Matt Joplin
//University of Tennessee at Chattanooga
//GigTank365 project: UTChattSat

//description: sensor configuration case structure version 1.1
// five sensors are available: Temperature, GPS, altitude, pressure, and gyroscope
// the case structure reacts to the received sensor configuration integer ({0-31, 0x00 to 0x1F, 00000 to 11111}, 5 bits, 1 bit per sensor) 
// MSBtoLSB: b5 - Temperature, b4 - GPS, b3 - Altitude, b2 - Pressure, b1 - gyro
// a 1 indicates the sensor is on. a 0 indicates the sensor is off.

//next: add sensor configuration functions

//per case: 1) configure sensors, 2) pass experiment iteration and sample timing variables (experiment iteration #) = (experiment time)/(sample time)
//          3) wait while( timer flag = 0) //timer flag =1 when sample time elapses 4) sample appropriate sensors and transmit the sample to main uC
//          5) increment iteration number 6) repeat from step 3 until experiment time has elapsed. 

//possible mods: based on the run time of the experiment, sleep when possible and resume when sample time has elapsed.

//#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_LSM303_U.h>
//#include <Adafruit_BMP085_U.h>
//#include <Adafruit_L3GD20_U.h>
//#include <Adafruit_10DOF.h>
#include <SoftwareSerial.h>

//case structure
void sensor_config_swcs(){
  int sensor = 0; 
for(sensor = 0; sensor < 0x22; sensor ++)
{
  //Serial.println(sensor);
  
  switch (sensor)
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
      break;
  }
}
  //sensor switch structure
  //switch(sensor[1]);
    
    
    return;
}




void setup() {
  // put your setup code here, to run once:
  //
  Serial.begin(9600);  
  //Serial.println("--- Start Serial Monitor SEND_RCVE ---");
  //Serial.println(" Type in Box above, . ");
  //Serial.println();
  interrupts();
}

void loop() {
  // put your main code here, to run repeatedly:
  //
  while (Serial.available() < 1) 
  {
    delay(1);
  }
  sensor_config_swcs();
}


