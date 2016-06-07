#include <SoftwareSerial.h>





void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  while(!Serial);
}

void loop() {
  
float roll, pitch, head, temp;
int roll_r, pitch_r, head_r, temp_r;

float rollres = 0.03;
float pitchres = 0.03;
float headres = 0.03;
float tempres = 0.3515625;

  Serial.println("Starting int receive");
  while(!Serial.available());
  if (Serial.available()){
    roll_r = Serial.parseInt();
    pitch_r = Serial.parseInt();
    head_r = Serial.parseInt();
    temp_r = Serial.parseInt();
    
    roll = roll_r * rollres;
    pitch = pitch_r *pitchres;
    head = head_r*headres;
    temp = temp_r*tempres;


//roll = roll_r*((2000-(-2000))/65536);
  }
    
    float twoplustwo;

    twoplustwo = 200 * rollres;
    
    Serial.println("what's two plus two?");
    Serial.println(twoplustwo);
    
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
    Serial.println("---------------------");
   

    
  // put your main code here, to run repeatedly:

}
