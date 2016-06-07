#include <SoftwareSerial.h>
//int roll, pitch, head, temp;
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

}

void loop() {
  if (Serial.available()){
    int roll = Serial.parseInt();
    int pitch = Serial.parseInt();
    int head = Serial.parseInt();
    int temp = Serial.parseInt();

//roll = roll_r*((2000-(-2000))/65536);


    
    Serial.print("Roll: ");
    Serial.println(roll);
    Serial.print("Pitch: ");
    Serial.println(pitch);
    Serial.print("Heading: ");
    Serial.println(head);
    Serial.print("Temperature: ");
    Serial.println(temp);
  }
  // put your main code here, to run repeatedly:

}
