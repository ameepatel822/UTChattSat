#include <Event.h>
#include <Timer.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_GPS.h>

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
sensors_event_t accel_event;
sensors_event_t mag_event;
sensors_vec_t   orientation;
sensors_event_t bmp_event;

float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
unsigned int tx_d=1000;
boolean usingInterrupt = false;
float alt, temp1,prs1,roll1,pitch1,head1,alt1;
int ExpTime, SampTime, SensorConfig;
Timer SampTimer;
uint32_t timer = millis();

// Function declaration

void useInterrupt(boolean);
void initSensors();
int* gyro();
int temperature();
int pressure();
int altitude();
long* GPS_parse();



void setup() {
  // put your setup code here, to run once:
  //Setup Serial

  Serial.begin(9600);
  interrupts();
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  GPS.begin(9600);
   GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
   GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
  
  
  
  initSensors();    // Initialise the sensors 
  useInterrupt(true);
  delay(900);
  mySerial.println(PMTK_Q_RELEASE);

}


SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
//  if (GPSECHO)
//    if (c) UDR0 = c;    // writing direct to UDR0 is much much faster than Serial.print but only one character can be written at a time. 
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
    Serial.println(ExpTime);
    Serial.println(SampTime);
    Serial.println(SensorConfig);
    ExpIter = trunc(ExpTime/SampTime);

    for(ExpCounter = 0; ExpCounter<ExpIter; ExpCounter++)
    {
      // SampTimer.every(SampTime, takeReading);
        //transmit data
      takeReading();
      delay(SampTime);
    }
   
  }
  useInterrupt(true);
}


void takeReading()
{
   int Config = SensorConfig;
    switch (Config)
  {
  int *gyro_c[3], temperature_c, pressure_c, altitude_c;
  long *GPS_raw[4];
                //sen1      sen2      sen3      sen4      sen5
                //-------------------------------------------------------------------------------------
    case 0x0000:  //null case
      //Serial.println("null case");
      break;
      
    case 0x0001:  //gyro
      //*gyro_c = gyro();
      int gyro_data_c[3];
      accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation) )
  {     // 'orientation' should have valid .roll and .pitch fields */
    roll1 = (orientation.roll/2000)*(65536);  // Step number for 16 bits
    gyro_data_c[0] = roll1;

    pitch1 = (orientation.pitch/2000)*(65536);  // Step number for 16 bits
    gyro_data_c[1] = pitch1;
  }
   /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation) )
  {
     /* 'orientation' should have valid .heading data now */
    head1 = (orientation.heading/2000)*(65536);
    gyro_data_c[2] = head1;
  }
      Serial.print(gyro_data_c[0]); Serial.print(",");
      Serial.print(gyro_data_c[1]); Serial.print(",");
      Serial.print(gyro_data_c[2]); Serial.print(",");
      break;
      
    case 0x0002:  //pressure
      pressure_c = pressure();
      Serial.print(pressure_c); Serial.print(",");
      break;
      
    case 0x0003:  //pressure, gyro
     
      *gyro_c = gyro();
      pressure_c = pressure();
      Serial.print(*gyro_c[0]); Serial.print(",");
      Serial.print(*gyro_c[1]); Serial.print(",");
      Serial.print(*gyro_c[2]); Serial.print(",");
      Serial.print(pressure_c); Serial.print(",");
      break;
      
    case 0x0004:  //alt
      altitude_c = altitude();
      Serial.print(altitude_c); Serial.print(",");
      //Serial.println("alt");
      break;
      
    case 0x0005:  //alt,      gyro
      *gyro_c = gyro();
      altitude_c = altitude();
      Serial.print(*gyro_c[0]); Serial.print(",");
      Serial.print(*gyro_c[1]); Serial.print(",");
      Serial.print(*gyro_c[2]); Serial.print(",");
      Serial.print(altitude_c); Serial.print(",");
      //Serial.println("alt, gyro");
      break;
      
    case 0x0006:  //alt,      pressure
      altitude_c = altitude();
      pressure_c = pressure();
      Serial.print(altitude_c); Serial.print(",");
      Serial.print(pressure_c); Serial.print(",");
      //Serial.println("alt, temp");
      break;
      
    case 0x0007:  //alt,      pressure, gyro
      *gyro_c = gyro();
      altitude_c = altitude();
      pressure_c = pressure();
      Serial.print(*gyro_c[0]); Serial.print(",");
      Serial.print(*gyro_c[1]); Serial.print(",");
      Serial.print(*gyro_c[2]); Serial.print(",");
      Serial.print(altitude_c); Serial.print(",");
      Serial.print(pressure_c); Serial.print(",");
      //Serial.println("alt, pressure, gyro");
      break;
      
    case 0x0008:  //GPS
    
      *GPS_raw = GPS_parse();
      Serial.print(*GPS_raw[0]); Serial.print(",");
      Serial.print(*GPS_raw[1]); Serial.print(","); 
      Serial.print(*GPS_raw[2]); Serial.print(",");
      Serial.print(*GPS_raw[3]); Serial.print(",");
      //Serial.println("GPS");
      break;
      
    case 0x0009:  //GPS,      gryo
      *gyro_c = gyro();
      *GPS_raw = GPS_parse();
      Serial.print(*gyro_c[0]); Serial.print(",");
      Serial.print(*gyro_c[1]); Serial.print(",");
      Serial.print(*gyro_c[2]); Serial.print(",");
      Serial.print(*GPS_raw[0]); Serial.print(",");
      Serial.print(*GPS_raw[1]); Serial.print(","); 
      Serial.print(*GPS_raw[2]); Serial.print(",");
      Serial.print(*GPS_raw[3]); Serial.print(",");
      
      //Serial.println("GPS, gyro");
      break;
      
    case 0x000A:  //GPS,      pressure
      pressure_c = pressure();
      *GPS_raw = GPS_parse();
      Serial.print(pressure_c); Serial.print(",");
      Serial.print(*GPS_raw[0]); Serial.print(",");
      Serial.print(*GPS_raw[1]); Serial.print(","); 
      Serial.print(*GPS_raw[2]); Serial.print(",");
      Serial.print(*GPS_raw[3]); Serial.print(",");
      //Serial.println("GPS, pressure");
      break;
      
    case 0x000B:  //GPS,      pressure, gyro
      *gyro_c = gyro();
       pressure_c = pressure();
      *GPS_raw = GPS_parse();
      Serial.print(*gyro_c[0]); Serial.print(",");
      Serial.print(*gyro_c[1]); Serial.print(",");
      Serial.print(*gyro_c[2]); Serial.print(",");
      Serial.print(pressure_c); Serial.print(",");
      Serial.print(*GPS_raw[0]); Serial.print(",");
      Serial.print(*GPS_raw[1]); Serial.print(","); 
      Serial.print(*GPS_raw[2]); Serial.print(",");
      Serial.print(*GPS_raw[3]); Serial.print(",");
      //Serial.println("GPS, pressure, gyro");
      break;
      
    case 0x000C:  //GPS,      alt
      altitude_c = altitude();
      *GPS_raw = GPS_parse();
      Serial.print(altitude_c); Serial.print(",");
      Serial.print(*GPS_raw[0]); Serial.print(",");
      Serial.print(*GPS_raw[1]); Serial.print(","); 
      Serial.print(*GPS_raw[2]); Serial.print(",");
      Serial.print(*GPS_raw[3]); Serial.print(",");
      //Serial.println("GPS, alt");
      break;
      
    case 0x000D:  //GPS,      alt,      gyro
      *gyro_c = gyro();
      altitude_c = altitude();
      *GPS_raw = GPS_parse();
      Serial.print(*gyro_c[0]); Serial.print(",");
      Serial.print(*gyro_c[1]); Serial.print(",");
      Serial.print(*gyro_c[2]); Serial.print(",");
      Serial.print(altitude_c); Serial.print(",");
      Serial.print(*GPS_raw[0]); Serial.print(",");
      Serial.print(*GPS_raw[1]); Serial.print(","); 
      Serial.print(*GPS_raw[2]); Serial.print(",");
      Serial.print(*GPS_raw[3]); Serial.print(",");
      //Serial.println("GPS, alt, gyro");
      break;
      
    case 0x000E:  //GPS,      alt,      pressure
      altitude_c = altitude();
      pressure_c = pressure();
      *GPS_raw = GPS_parse();
      Serial.print(altitude_c); Serial.print(","); 
      Serial.print(pressure_c); Serial.print(",");
      Serial.print(*GPS_raw[0]); Serial.print(",");
      Serial.print(*GPS_raw[1]); Serial.print(","); 
      Serial.print(*GPS_raw[2]); Serial.print(",");
      Serial.print(*GPS_raw[3]); Serial.print(",");
      //Serial.println("GPS, alt, pressure");
      break;
      
    case 0x000F:  //GPS,      alt,      pressure, gyro
      
      *gyro_c = gyro();
      altitude_c = altitude();
      pressure_c = pressure();
      *GPS_raw = GPS_parse();
      Serial.print(*gyro_c[0]); Serial.print(",");
      Serial.print(*gyro_c[1]); Serial.print(",");
      Serial.print(*gyro_c[2]); Serial.print(",");
      Serial.print(altitude_c); Serial.print(",");
      Serial.print(pressure_c); Serial.print(",");
      Serial.print(*GPS_raw[0]); Serial.print(",");
      Serial.print(*GPS_raw[1]); Serial.print(","); 
      Serial.print(*GPS_raw[2]); Serial.print(",");
      Serial.print(*GPS_raw[3]); Serial.print(",");
      //Serial.println("GPS, alt, pressure, gyro");
      break;
      
    case 0x0010:  //Temp
      temperature_c = temperature();
      Serial.print(temperature_c); Serial.print(",");
      //Serial.println("Temp");
      break;
      
    case 0x0011:  //Temp,     gyro
      
      *gyro_c = gyro();
      temperature_c = temperature();
      Serial.print(*gyro_c[0]); Serial.print(",");
      Serial.print(*gyro_c[1]); Serial.print(",");
      Serial.print(*gyro_c[2]); Serial.print(",");
      Serial.print(temperature_c); Serial.print(",");
      //Serial.println("Temp, gyro");
      break;
   
    case 0x0012:  //Temp,     pressure 
       pressure_c = pressure();
       temperature_c = temperature();
       Serial.print(pressure_c); Serial.print(",");
       Serial.print(temperature_c); Serial.print(",");
       //Serial.println("Temp, pressure");
       break;
    
    case 0x0013:  //Temp,     pressure, gyro
      
      *gyro_c = gyro();
      pressure_c = pressure();
      temperature_c = temperature();
      Serial.print(*gyro_c[0]); Serial.print(",");
      Serial.print(*gyro_c[1]); Serial.print(",");
      Serial.print(*gyro_c[2]); Serial.print(",");
      Serial.print(pressure_c); Serial.print(",");
      Serial.print(temperature_c); Serial.print(",");
      //Serial.println("Temp, pressure, gyro");
      break;
    
    case 0x0014:  //Temp,     alt  
      altitude_c = altitude();
      temperature_c = temperature();
      Serial.print(altitude_c); Serial.print(",");
      Serial.print(temperature_c); Serial.print(",");
      //Serial.println("Temp, alt");
      break;
    
    case 0x0015:  //Temp,     alt,      gyro  
     
      *gyro_c = gyro();
      altitude_c = altitude();
      temperature_c = temperature();
      Serial.print(*gyro_c[0]); Serial.print(",");
      Serial.print(*gyro_c[1]); Serial.print(",");
      Serial.print(*gyro_c[2]); Serial.print(",");
      Serial.print(altitude_c); Serial.print(",");
      Serial.print(temperature_c); Serial.print(",");
      //Serial.println("Temp, alt, gyro");
      break;
      
    case 0x0016:  //Temp,     alt,      pressure
      altitude_c = altitude();
      pressure_c = pressure();
      temperature_c = temperature();
      Serial.print(altitude_c); Serial.print(",");
      Serial.print(pressure_c); Serial.print(",");   
      Serial.print(temperature_c); Serial.print(",");
      //Serial.println("Temp, alt, pressure");
      break;
      
    case 0x0017:  //Temp,     alt,      pressure, gyro
      *gyro_c = gyro();
      altitude_c = altitude();
      pressure_c = pressure();
      temperature_c = temperature();
      Serial.print(*gyro_c[0]); Serial.print(",");
      Serial.print(*gyro_c[1]); Serial.print(",");
      Serial.print(*gyro_c[2]); Serial.print(",");
      Serial.print(altitude_c); Serial.print(",");
      Serial.print(pressure_c); Serial.print(",");
      Serial.print(temperature_c); Serial.print(",");
      //Serial.println("Temp, alt, pressure, gyro");
      break;
      
    case 0x0018:  //Temp,     GPS
      temperature_c = temperature();
      *GPS_raw = GPS_parse();
      Serial.print(temperature_c); Serial.print(",");
      Serial.print(*GPS_raw[0]); Serial.print(",");
      Serial.print(*GPS_raw[1]); Serial.print(","); 
      Serial.print(*GPS_raw[2]); Serial.print(",");
      Serial.print(*GPS_raw[3]); Serial.print(",");
      //Serial.println("Temp, GPS");
      break;
      
    case 0x0019:  //Temp,     GPS,      gyro
      *gyro_c = gyro();
      temperature_c = temperature();
      *GPS_raw = GPS_parse();
      Serial.print(*gyro_c[0]); Serial.print(",");
      Serial.print(*gyro_c[1]); Serial.print(",");
      Serial.print(*gyro_c[2]); Serial.print(",");
      Serial.print(temperature_c); Serial.print(",");
      Serial.print(*GPS_raw[0]); Serial.print(",");
      Serial.print(*GPS_raw[1]); Serial.print(","); 
      Serial.print(*GPS_raw[2]); Serial.print(",");
      Serial.print(*GPS_raw[3]); Serial.print(",");
      //Serial.println("Temp, GPS, gyro");
      break;
      
    case 0x001A:  //Pressure,   Temp,     GPS
      pressure_c = pressure();
      temperature_c = temperature();
      *GPS_raw = GPS_parse();
      Serial.print(pressure_c); Serial.print(",");
      Serial.print(temperature_c); Serial.print(",");
      Serial.print(*GPS_raw[0]); Serial.print(",");
      Serial.print(*GPS_raw[1]); Serial.print(","); 
      Serial.print(*GPS_raw[2]); Serial.print(",");
      Serial.print(*GPS_raw[3]); Serial.print(",");
      //Serial.println("Temp, GPS, pressure");
      break;
      
    case 0x001B:  //Gyro,     Press,      Temp,   GPS
      *gyro_c = gyro();
      pressure_c = pressure();
      temperature_c = temperature();
      *GPS_raw = GPS_parse();
      Serial.print(*gyro_c[0]); Serial.print(",");
      Serial.print(*gyro_c[1]); Serial.print(",");
      Serial.print(*gyro_c[2]); Serial.print(",");
      Serial.print(pressure_c); Serial.print(",");
      Serial.print(temperature_c); Serial.print(",");
      Serial.print(*GPS_raw[0]); Serial.print(",");
      Serial.print(*GPS_raw[1]); Serial.print(","); 
      Serial.print(*GPS_raw[2]); Serial.print(",");
      Serial.print(*GPS_raw[3]); Serial.print(",");
      //Serial.println("Temp, GPS, pressure, gyro");
      break;
      
    case 0x001C:  //ALt,     Temp,      GPS
      altitude_c = altitude();
      temperature_c = temperature();
      *GPS_raw = GPS_parse();
      Serial.print(altitude_c); Serial.print(",");
      Serial.print(temperature_c); Serial.print(",");
      Serial.print(*GPS_raw[0]); Serial.print(",");
      Serial.print(*GPS_raw[1]); Serial.print(","); 
      Serial.print(*GPS_raw[2]); Serial.print(",");
      Serial.print(*GPS_raw[3]); Serial.println(",");
      //Serial.println("Temp, GPS, alt");
      break;
      
    case 0x001D:  //Gyro,     Alt,      Temp,      GPS
      *gyro_c = gyro();
      altitude_c = altitude();
      temperature_c = temperature();
      *GPS_raw = GPS_parse();
      Serial.print(*gyro_c[0]); Serial.print(",");
      Serial.print(*gyro_c[1]); Serial.print(",");
      Serial.print(*gyro_c[2]); Serial.print(",");
      Serial.print(altitude_c); Serial.print(",");
      Serial.print(temperature_c); Serial.print(",");
      Serial.print(*GPS_raw[0]); Serial.print(",");
      Serial.print(*GPS_raw[1]); Serial.print(","); 
      Serial.print(*GPS_raw[2]); Serial.print(",");
      Serial.print(*GPS_raw[3]); Serial.println(",");
      //Serial.println("Temp, GPS, alt, gyro");
      break;
      
    case 0x001E:  //Alt,     Pressure,      Temp,      GPS
      altitude_c = altitude();
      pressure_c = pressure();
      temperature_c = temperature(); 
      *GPS_raw = GPS_parse();
      Serial.print(altitude_c); Serial.print(",");
      Serial.print(pressure_c); Serial.print(",");
      Serial.print(temperature_c); Serial.print(",");
      Serial.print(*GPS_raw[0]); Serial.print(",");
      Serial.print(*GPS_raw[1]); Serial.print(","); 
      Serial.print(*GPS_raw[2]); Serial.print(",");
      Serial.print(*GPS_raw[3]); Serial.println(",");
      //Serial.println("Temp, GPS, alt, pressure");
      break;
      
    case 0x001F:  // Gyro,     Alt,      Pressure,      Temp,  GPS
      *gyro_c = gyro();
      altitude_c = altitude();
      pressure_c = pressure();
      temperature_c = temperature(); 
      *GPS_raw = GPS_parse();
      Serial.print(*gyro_c[0]); Serial.print(",");
      Serial.print(*gyro_c[1]); Serial.print(",");
      Serial.print(*gyro_c[2]); Serial.print(",");
      Serial.print(altitude_c); Serial.print(",");
      Serial.print(pressure_c); Serial.print(",");
      Serial.print(temperature_c); Serial.print(",");
      Serial.print(*GPS_raw[0]); Serial.print(",");
      Serial.print(*GPS_raw[1]); Serial.print(","); 
      Serial.print(*GPS_raw[2]); Serial.print(",");
      Serial.print(*GPS_raw[3]); Serial.println(",");
      //Serial.println("Temp, GPS, alt, pressure, gyro");
      break;
    
    default:
      //Serial.println("uhh, what?");
      
      
      break;      //replace with specific return() for the combination of data packets


}
}


int* gyro()
{
  int gyro_data_c[3];
 
   /* Calculate pitch and roll from the raw accelerometer data */   
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation) )
  {     // 'orientation' should have valid .roll and .pitch fields */
    roll1 = (orientation.roll/2000)*(65536);  // Step number for 16 bits
    gyro_data_c[0] = roll1;

    pitch1 = (orientation.pitch/2000)*(65536);  // Step number for 16 bits
    gyro_data_c[1] = pitch1;
  }
   /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation) )
  {
     /* 'orientation' should have valid .heading data now */
    head1 = (orientation.heading/2000)*(65536);
    gyro_data_c[2] = head1;
  }
  return gyro_data_c; 
}

int temperature(){
  int temp_data_c;
  bmp.getEvent(&bmp_event);
  float temperature;
  if (bmp_event.pressure)
  {
    
    /* Get ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    temp1 = (temperature/90)*256;
    temp_data_c= temp1;
    
 
  }
return temp_data_c;
}

int pressure(){
  int prs_data_c;
  bmp.getEvent(&bmp_event);
  float temperature;
   if (bmp_event.pressure)
  {
    /* Get pressure in hPa */
   prs1 = (bmp_event.pressure/1100)*256;
   prs_data_c = prs1;
   
   return prs_data_c;
  }
}

int altitude(){
  int alt_data_c;
  bmp.getEvent(&bmp_event);
  float temperature;
  if (bmp_event.pressure)   // Calculate the altitude using the barometric pressure sensor */
  {
    float temperature;
    bmp.getTemperature(&temperature);   // Get ambient temperature in C 
    alt = bmp.pressureToAltitude(seaLevelPressure,          // Convert atmospheric pressure, SLP and temp to altitude    */
                                     bmp_event.pressure,
                                     temperature);
    alt1 = (alt/9000)*65536;
    alt_data_c = alt1;
   
    return alt_data_c;;
  }
  
}

long* GPS_parse(){
  long GPS_data_raw[4];
  
  if (! usingInterrupt) {// read data from the GPS in the 'main loop'
    // char c = GPS.read();
  //if (GPSECHO)
    //  if (c) Serial.print(c);
  }
 if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA())) {  // this also sets the newNMEAreceived() flag to false
      return(0);  // we can fail to parse a sentence in which case we should just wait for another
  }
  

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
        
    GPS_data_raw[0]= (int(GPS.hour)*10000)+(int(GPS.minute)*100)+int(GPS.seconds);
   
    if (GPS.fix) {
      GPS_data_raw[1]= GPS.latitudeDegrees;
     
      GPS_data_raw[2]=GPS.longitudeDegrees;
      
      GPS_data_raw[3] = GPS.altitude;
   
    }
  }
}
   return GPS_data_raw;
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
  
}
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
}


