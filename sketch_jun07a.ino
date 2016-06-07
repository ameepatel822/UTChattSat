#include <SoftwareSerial.h>
int roll, pitch, head, temp;
int roll_r, pitch_r, head_r, temp_r;
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  if (Serial.available()){
     roll_r = Serial.parseInt();
     pitch_r = Serial.parseInt();
     head_r = Serial.parseInt();
     temp_r = Serial.parseInt();
  }
roll = roll_r*(2000/65536);
pitch = pitch_r*(2000/65536);
head = head_r*(2000/65536);
temp = temp_r*(90/256);


    
    Serial.print("Roll: ");
    Serial.println(roll);
    Serial.print("Pitch: ");
    Serial.println(pitch);
    Serial.print("Heading: ");
    Serial.println(head);
    Serial.print("Temperature: ");
    Serial.println(temp);
  
  // put your main code here, to run repeatedly:

}
