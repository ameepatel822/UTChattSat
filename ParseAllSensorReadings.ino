#include <SoftwareSerial.h>

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  while(!Serial);
}

void loop() {
  
float roll, pitch, head, temp, alt, pres;
int roll_r, pitch_r, head_r, temp_r, alt_r, pres_r;

//GPSfixtime, GPSlat, GPSdir_N_S, GPSlong, GPSdir_E_W, GPSfixqual, GPSNumSat, HorizDilution, Alt, GeoidHeight, DGPSUpdateTime, DGPSStationID

long int GPSfixtime;
int GPSalt;
float GPSlon, GPSlat;


float rollres = 0.03;
float pitchres = 0.03;
float headres = 0.03;
float tempres = 0.3515625;
float altres = 0.137;
float presres = 4.296;

  Serial.println("\n\n\nStarting int receive");
  while(!Serial.available());
  if (Serial.available()){
    roll_r = Serial.parseInt();
    pitch_r = Serial.parseInt();
    head_r = Serial.parseInt();
    alt_r = Serial.parseInt();
    pres_r = Serial.parseInt();
    temp_r = Serial.parseInt();
    
    roll = roll_r * rollres;
    pitch = pitch_r *pitchres;
    head = head_r*headres;
    alt = alt_r*altres;
    pres = pres_r*presres;
    temp = temp_r*tempres;
    
    GPSfixtime = Serial.parseInt();
    GPSlat = Serial.parseFloat();
    GPSlon = Serial.parseFloat();
    GPSalt = Serial.parseInt();

  }

    
    Serial.print("Roll Step #: ");
    Serial.println(roll_r);
    Serial.print("Roll Resolution: ");
    Serial.println(rollres, DEC);
    Serial.print("Roll: ");
    Serial.println(roll, DEC);
    
    Serial.print("Pitch Step #: ");
    Serial.println(pitch_r);
    Serial.print("Pitch Resolution: ");
    Serial.println(pitchres, DEC);
    Serial.print("Pitch: ");
    Serial.println(pitch, DEC);
    
    Serial.print("Heading Step #: ");
    Serial.println(head_r);
    Serial.print("Heading Resolution: ");
    Serial.println(headres, DEC);
    Serial.print("Heading: ");
    Serial.println(head, DEC);
    
    Serial.print("Temperature Step #: ");
    Serial.println(temp_r);
    Serial.print("Temperature Resolution: ");
    Serial.println(tempres, DEC);
    Serial.print("Temperature: ");
    Serial.println(temp, DEC);


    Serial.print("Altitude Step #: ");
    Serial.println(alt_r);
    Serial.print("Altitude Resolution: ");
    Serial.println(altres, DEC);
    Serial.print("Altitude: ");
    Serial.println(alt, DEC);


    Serial.print("Pressure Step #: ");
    Serial.println(pres_r);
    Serial.print("Pressure Resolution: ");
    Serial.println(presres, DEC);
    Serial.print("Pressure: ");
    Serial.println(pres, DEC);


    Serial.print("GPSFixTime: ");
    Serial.println(GPSfixtime);
    
    Serial.print("GPS Latitude: ");
    Serial.println(GPSlat);

    Serial.print("GPS Longitude: ");
    Serial.println(GPSlon);

    Serial.print("GPS Altitude: ");
    Serial.println(GPSalt);

    Serial.println("---------------------");
    
    delay(100);

}
