#include "MsTimer2.h"
//#include <TimerOne.h>
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
boolean usingInterrupt = false;
float alt, temp1,prs1,roll,pitch,head,alt1;
int ExpTime, SampTime,ExpTime_milli, SampTime_milli, SensorConfig, rst;
uint32_t timer = millis();
int fixtime, latitude, longitude, GPS_alt;
bool gyro_flag=false, GPS_flag=false;
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
  
  Serial.begin(9600);                       // Serial Comm Baud Rate
  GPS.begin(9600);                          // GPS Baud Rate 
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);   // RMC and GGA ON
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);     // NMEA Update rate: 10 Hz
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);      // Fix Update rate: 5 Hz
  
  initSensors();                            // Initialise the sensors 
  useInterrupt(true);
  delay(900);
  mySerial.println(PMTK_Q_RELEASE);         // Initialize the GPS
  
}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();            // GPS read data
  return; 
}

void loop()
{
  int ExpCounter;                 //keeps track of how many samples have been collected
  int ExpIter;                    //total number of samples to be taken
  while(!Serial.available());
  if(Serial.available())
  {
    ExpCounter = 0;               // Reset the counters for a new transmission request
    ExpIter = 0;  
    ExpTime = Serial.parseInt();  // Save Experiment Time
    SampTime = Serial.parseInt(); // Save Sample Time
    SensorConfig = Serial.parseInt(); // Save Sensor Configuration
    ExpTime_milli = ExpTime * 1000;
    SampTime_milli = SampTime * 1000;
    ExpIter = trunc(ExpTime/SampTime);   
    rst = ExpTime + SampTime + SensorConfig;   // reset vector
  }

  if (ExpCounter < ExpIter){        // Check for the current transmission complete 
      for(ExpCounter = 0; ExpCounter<ExpIter; ExpCounter++)       // Run through for Experiment time 
      {
        if (!Serial.available()){
        if (rst > 0){
        takeReading();   // collect and transmit readings as per Sensor Configuration
        delay(SampTime_milli);          // stay here for Sample time
      }
      }
      }
    }
 }


void takeReading()
{

  bool CompVal = 1;                                 // Compare Value = '1' for checking whether sensor is ON
  int j;
  for (int j = 0; j < 8; j++)                       // Roll through all sensors(8); 0 = Gyroscope, 1 = Altitude, 2 = Temeprature, 3 = Temperature, 4 = GPS
  {                                                 // 5,6,7 = Extra sensor 1,2,3 respectively   
    if (CompVal && bitRead(SensorConfig, j))        // Check sensor j state (1 = ON, 0 = OFF)
    {
      switch (j)
      {
        case 0 :                                    // Gyroscope ON
            gyro();                                 // Call function to get the Gyroscope data
            if(gyro_flag){
              GetData[0] = roll;
              GetData[1] = pitch;
              GetData[2] = head;
            }
            else{
              GetData[0] = 0;
              GetData[1] = 0;
              GetData[2] = 0;
            }
            gyro_flag = false;
            break;
        case 1 :
            GetData[3] = altitude();                 // Altitude ON. Call function to get Altitude data
            break;
        case 2 :
            GetData[4] = pressure();
            break;
        case 3 :
            GetData[5] = temperature();             // Temperature ON. Call function to get Temperature data
            break;
        case 4 :
            useInterrupt(true);                     // GPS Interrupt enabled
            GPS_parse();                            // GPS ON. Call function to get GPS data
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
            useInterrupt(false);                    // GPS Interrupt disabled
            break;
        case 5 :                                    // Extra Sensor 1 ON
            break;
        case 6 :                                    // Extra Sensor 2 ON
            break;
        case 7 :                                    // Extra Sensor 3 ON
            break;
        default:                                    // Default State. Do Nothing.
            break;        
      }
    }
    else                                            // Transmit '0' for the sensors that are turned OFF 
    {      
      switch (j)
      {
        case 0 :                                    // Gyroscope OFF
            GetData[0] = 0;
            GetData[1] = 0;
            GetData[2] = 0;
            break;
        case 1 :                                    // Altitude OFF
            GetData[3] = 0;
            break;
        case 2 :                                    // Pressure OFF
            GetData[4] = 0;
            break;
        case 3 :                                    // Temperature OFF
            GetData[5] = 0;                         
            break;
        case 4 :                                    // GPS OFF
            GetData[6] = 0;
            GetData[7] = 0;
            GetData[8] = 0;
            GetData[9] = 0;
            break;
        case 5 :                                    // Extra Sensor 1 OFF
            break;
        case 6 :                                    // Extra Sensor 2 OFF
            break;
        case 7 :                                    // Extra Sensor 3 OFF
            break;
        default:                                    // Default State. Do Nothing.
            break;        
      }
    }
  }
//Serial.print("$");Serial.print(",");

  for (int i=0; i<10 ; i++){
    
    Serial.print(GetData[i]);                   // Print the data stream, each field seperated by commas
    if (i < 9 ) Serial.print(",");
    else Serial.println(" ");
      
  }
 
//  while(!Serial.available());
 // Serial.println(" ");            // Enter a new line for every transmission
  return;
}

void gyro()
{
  int gyro_data_c[3];
 
   /* Calculate pitch and roll from the raw accelerometer data */   
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation) )
  {                                                   // 'orientation' should have valid .roll and .pitch fields 
    roll = (orientation.roll/2000)*(65536);          // Step number for 16 bits
    gyro_data_c[0] = roll;

    pitch = (orientation.pitch/2000)*(65536);        // Step number for 16 bits
    gyro_data_c[1] = pitch;
  }
   /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation) )
  {
     /* 'orientation' should have valid .heading data now */
    head = (orientation.heading/2000)*(65536);       // Step number for 16 bits
    gyro_data_c[2] = head;
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
    temp1 = (temperature/90)*256;                     // Step number for 8 bits
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
   prs1 = (bmp_event.pressure/1100)*256;                // Step number for 8 bits
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
    alt1 = (alt/9000)*65536;                     // Step number for 16 bits
    alt_data_c = alt1;
   
    return alt_data_c;;
  }
  
}

void GPS_parse(){
 
 
            
  float GPS_data_raw[4];
   
  if (! usingInterrupt) {// read data from the GPS in the 'main loop'
  }
 if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA())) {  // this also sets the newNMEAreceived() flag to false
      //return ;  // we can fail to parse a sentence in which case we should just wait for another
  }
  

  // approximately every 0.2 seconds or so, print out the current stats
  if (millis() - timer > 200) { 
   //GPS_data_raw[0]= (int(GPS.hour)*10000)+(int(GPS.minute)*100)+int(GPS.seconds);
   GPS_data_raw[0] = (int(GPS.hour)*100)+int(GPS.minute);
   fixtime = GPS_data_raw[0];
    
   if (GPS.fix) {
      GPS_data_raw[1]= (GPS.latitudeDegrees/90)*65536;                     // Step number for 16 bits
      latitude = GPS_data_raw[1];
     
      GPS_data_raw[2]=(GPS.longitudeDegrees/90)*65536;                     // Step number for 16 bits
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




