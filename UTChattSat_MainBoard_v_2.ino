#include <SoftwareSerial.h> 
#include <SD.h>
 SoftwareSerial sensorcont(4,5); 
 
 
 void setup() { 
   Serial.begin(9600); 
   sensorcont.begin(9600); 
   // put your setup code here, to run once: 
   while(!Serial); 
 } 
 
 void loop() { 
    
 int SensorData[10], SensorConfig[3]; 
 int sensor_SampTime = 0, ExpIter, ExpCounter;
 
   while(!Serial.available()); 
  
   if (Serial.available()){ 
     for (int i = 0; i<3 ; i++){ 
       SensorConfig[i] = Serial.parseInt();      // Read Configuration Word form XBee, Digital 0 pin 
     } 
          
     for (int i = 0; i<3 ; i++){ 
       sensorcont.print(SensorConfig[i]);        // Write Configuration Word to Sensor Controller, Digital 5 pin  
       if (i < 2 ) sensorcont.print(","); 
       else sensorcont.println(" "); 
     }   
    
   sensor_SampTime = SensorConfig[1] * 1500;
   sensorcont.setTimeout(sensor_SampTime);
   ExpIter = trunc(SensorConfig[0]/SensorConfig[1]);
   
   while(!sensorcont.available()); 
   
   for(ExpCounter = 0; ExpCounter<ExpIter; ExpCounter++){
     int j, k; 
     for (j = 0; j<10 ; j++){ 
         SensorData[j] = sensorcont.parseInt();      // Read Data Word form Sensor Controller, Digital 4 pin 
       } 
     j=0; 
     
     for (k = 0; k<10 ; k++){ 
       Serial.print(SensorData[k]);                // Write Data Word to XBee, Digital 1 pin        
       if (k < 9 ) Serial.print(","); 
       else Serial.println(" "); 
     } 
     k=0; 
   } 
  } 
} 

