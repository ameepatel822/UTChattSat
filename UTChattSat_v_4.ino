#include "MsTimer2.h"
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
//int ExpTime, SampTime, SensorConfig;
uint32_t timer = millis();
int GyroRoll, GyroPitch, GyroHead, fixtime, latitude, longitude, GPS_alt;
bool gyro_flag=false, GPS_flag=false;

int CmmdWord = 0;
int SampTime = 2000;
int GetData[10];

// Function declaration

void useInterrupt(boolean);
void initSensors();
void gyro();
int temperature();
int pressure();
int altitude();
void GPS_parse();
void takeReading();



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
  //useInterrupt(true);
  //delay(900);
  mySerial.println(PMTK_Q_RELEASE);
  
  MsTimer2::set(SampTime, takeReading); // 500ms period
  MsTimer2::start();

}
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
//  if (GPSECHO)
//    if (c) UDR0 = c;    // writing direct to UDR0 is much much faster than Serial.print but only one character can be written at a time. 
}

void loop()
{
  //useInterrupt(true);
  takeReading();
  //delay (1000);
}


void takeReading()
{

  CmmdWord = random(1, 32);
  bool CompVal = 1;
  Serial.println(CmmdWord, BIN);

  for (int j = 0; j < 8; j++)
  {
    if (CompVal && bitRead(CmmdWord, j))
    {
      switch (j)
      {
        case 0 :
            gyro();
            if(gyro_flag){
              GetData[0] = roll1;
              GetData[1] = pitch1;
              GetData[2] = head1;
            }
            else{
              GetData[0] = 0;
              GetData[1] = 0;
              GetData[2] = 0;
            }
            gyro_flag = false;
            Serial.println("Gyro ON");
            break;
        case 1 :
            GetData[3] = altitude();
            Serial.println("Altitude ON");
            break;
        case 2 :
            GetData[4] = pressure();
            Serial.println("Pressure ON");
            break;
        case 3 :
            GetData[5] = temperature();
            Serial.println("Temperature ON");
            break;
        case 4 :
            useInterrupt(true);
            GPS_parse();
            if(GPS_flag){
              GetData[6] = fixtime;
              GetData[7] = latitude;
              GetData[8] = longitude;
              GetData[9] = GPS_alt;
            }
            else{
              GetData[6] = 0;
              GetData[7] = 0;
              GetData[8] = 0;
              GetData[9] = 0;
            }
            GPS_flag = false;
            Serial.println("GPS ON");
            useInterrupt(false);
           break;
        case 5 :
           Serial.println("ExtraSensor 1 ON");
           break;
        case 6 :
           Serial.println("ExtraSensor 2 ON");
           break;
        case 7 :
           Serial.println("ExtraSensor3 ON");
           break;
        default:
           Serial.println("Deafault State");
           break;        
      }
    }
    else
    {      
      switch (j)
      {
        case 0 :
            GetData[0] = 0;
            GetData[1] = 0;
            GetData[2] = 0;
            Serial.println("Gyro OFF");
            break;
        case 1 :
            GetData[3] = 0;
            Serial.println("Altitude OFF");
            break;
        case 2 :
            GetData[4] = 0;
            Serial.println("Pressure OFF");
            break;
        case 3 :
            GetData[5] = 0;
            Serial.println("Temperature OFF");
            break;
        case 4 :
            GetData[6] = 0;
            GetData[7] = 0;
            GetData[8] = 0;
            GetData[9] = 0;
            Serial.println("GPS OFF");
            break;
        case 5 :
           Serial.println("ExtraSensor 1 OFF");
           break;
        case 6 :
           Serial.println("ExtraSensor 2 OFF");
           break;
        case 7 :
           Serial.println("ExtraSensor3 OFF");
           break;
        default:
           Serial.println("Deafault State OFF");
           break;        
      }
    }
  }

for (int i=0; i<10 ; i++){
  Serial.print(GetData[i]); Serial.print(",");
}
return;
}
void gyro()
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
  gyro_flag = true;
  return; 
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

void GPS_parse(){
  float GPS_data_raw[4];
   
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
      return ;  // we can fail to parse a sentence in which case we should just wait for another
  }
  

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
        
    //GPS_data_raw[0]= (int(GPS.hour)*10000)+(int(GPS.minute)*100)+int(GPS.seconds);
   GPS_data_raw[0] = GPS.hour;
   fixtime = GPS_data_raw[0];
    if (GPS.fix) {
      GPS_data_raw[1]= (GPS.latitudeDegrees/90)*65536;
      latitude = GPS_data_raw[1];
     
      GPS_data_raw[2]=(GPS.longitudeDegrees/90)*65536;
      longitude = GPS_data_raw[2];
      
      GPS_data_raw[3] = GPS.altitude;
      GPS_alt = GPS_data_raw[3];
   
    }
  }
 
}
  GPS_flag =true;
  return ;
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
  return;
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
  return;
}




