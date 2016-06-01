#include <MsTimer2.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_GPS.h>

 
#include <SoftwareSerial.h> 
 

SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
float alt;
int test, temp_f, alt_f, gps_f, gyro_f, c,cg,cgps,ca, ul=0, ul_g =0, ul_gps=0, ul_a=0;
unsigned int tx_d=1000;
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
String cmmnd;

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
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
  
/**************************************************************************/
/*!

*/
/**************************************************************************/
void setup()
{
  
  Serial.begin(9600);

 
  Serial.println(F("Adafruit GPS library and 10 DOF Pitch/Roll/Heading Example")); Serial.println("");
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  GPS.begin(9600);
   GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
   GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
  
  
  /* Initialise the sensors */
  initSensors();

  
  MsTimer2::set(27000000, cut); // 2700 sec (45 min) period
  MsTimer2::start();
  delay(1000);
   
}
void cut(){
digitalWrite(4, HIGH);
test = 1;
MsTimer2::stop();
}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
}



/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/

void loop(void)
{
  
  
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;

  /* Calculate pitch and roll from the raw accelerometer data */
  
  cmmnd = Serial.readString();
  Serial.println(cmmnd);
  if (cmmnd.startsWith("Temp ON")){
    temp_f = 1; ul =1; c =1;
  if (cmmnd.startsWith("5 sec",8)){
    ul =2;    c =2;
    }
  else if (cmmnd.startsWith("10 sec",8)){
    ul =2;    c =5;
    }
  }
  else if (cmmnd.startsWith("Temp OFF")){
    temp_f = 0; ul = 0; c= 0;
  }
  else if (cmmnd.startsWith("Gyro ON")){
    gyro_f = 1; ul_g =1; cg =1;
  if (cmmnd.startsWith("5 sec",8)){
    ul_g =2;    cg =2;
    }
  else if (cmmnd.startsWith("10 sec",8)){
    ul_g =2;    cg =5;
    }
  }
  else if (cmmnd.startsWith("Gyro OFF")){
    gyro_f = 0; ul_g = 0; cg = 0;
  }
  else if (cmmnd.startsWith("GPS ON")){
    gps_f = 1; ul_gps =1; cgps =1;
  if (cmmnd.startsWith("5 sec",7)){
    ul_gps =2;    cgps =2;
    }
  else if (cmmnd.startsWith("10 sec",7)){
    ul_gps =2;    cgps =5;
    }
  }
  else if (cmmnd.startsWith("GPS OFF")){
    gps_f = 0; ul_gps = 0; cgps = 0;
  }
  else if (cmmnd.startsWith("Alt ON")){
    alt_f = 1; ul_a =1; ca =1;
  if (cmmnd.startsWith("5 sec",7)){
    ul_a =2;    ca =2;
    }
  else if (cmmnd.startsWith("10 sec",7)){
    ul_a =2;    ca =5;
    }
  }
  else if (cmmnd.startsWith("Alt OFF")){
    alt_f = 0; ul_a = 0; ca = 0;
  }
  else if (cmmnd.startsWith("Tx 10 sec")){
    tx_d = 10000;
  }
  
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation) && (gyro_f = 1) && (cg > 0))
  {
    if (ul_g >1){
      cg -= 1;
    }
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print("Roll: ");
    Serial.print(orientation.roll);
    Serial.print(F("; "));
    Serial.print(F("Pitch: "));
    Serial.print(orientation.pitch);
    Serial.print(F("; "));
  }
  
  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation) && (gyro_f = 1) && (cg > 0))
  {
     if (ul_g >1){
      cg -= 1;
    }
    /* 'orientation' should have valid .heading data now */
    Serial.print(F("Heading: "));
    Serial.print(orientation.heading);
    Serial.print(F("; "));
  }

  /* Calculate the altitude using the barometric pressure sensor */
  bmp.getEvent(&bmp_event);
  float temperature;
  if (bmp_event.pressure && (alt_f=1) && (ca>0))
  {
    if (ul_a>1){
      ca-=1;
    }
    /* Get ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    /* Convert atmospheric pressure, SLP and temp to altitude    */
    Serial.print(F("Alt: "));
    Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                   bmp_event.pressure,
                                   temperature)); 
    Serial.print(F(" m; "));
  }
   
if (bmp_event.pressure && (temp_f = 1) && (c>0))
  {
    if (ul > 1){
    c-=1;
    }
    /* Get ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    /* Display the temperature */
    Serial.print(F("Temp: "));
    Serial.print(temperature);
    Serial.print(F(" C"));
  }
     

  
  Serial.println(F(""));
if((gps_f =1) && (cgps>0))
{
  if (ul_gps>1){
    cgps -= 1;  
  }
  useInterrupt(true);
    delay(1000);
 useInterrupt(false);
}
 
 Serial.println(F(""));
  
   alt = (bmp.pressureToAltitude(seaLevelPressure,
                                   bmp_event.pressure,
                                   temperature));
    
     if (alt>=256){
      digitalWrite(4,HIGH);
       delay(10000);
      digitalWrite(4,LOW);
     
     } else if (test == 1) {
      delay(10000);
      digitalWrite(4,LOW);
      test = 0;
     }

     
  delay(tx_d);

  
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



