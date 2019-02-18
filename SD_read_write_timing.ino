#include <SPI.h>
#include <SD.h>

//Variables:
File myFile;

void setup() {

  //Start the serial communication
  Serial.begin(9600); //Baud rate must be the same as is on xBee module
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  unsigned long start = millis();//replace with Teensy Version ellapsedMillis see https://platformio.org/lib/show/1002/elapsedMillis/examples?file=timingComparison.ino
  int count = 1;
  myFile = SD.open("testwr.csv");
  unsigned long openTime = millis();//replace with Teensy Version timer
  if(myFile){
     while (myFile.available()) {
      Serial.println("available");
     String list = myFile.readStringUntil('\r');
      Serial.print(list);
    }
    /*writing entries onto card*/
     while(count<=999){
        myFile.print(count);
        myFile.println(",32.9367,107.0689,27565.66,48,63.58,8.49,2");
        count++;
      }
      unsigned long finishWriting = millis();//replace
      
    myFile.close();
      Serial.println(finishWriting-openTime);
      Serial.println("Success");
  }
    else{
        Serial.println("Failed");
        }
 /*reading entries from card*/
 myFile = SD.open("test.csv");
  unsigned long openTime = millis();//replace
  Serial.print("Time to open file: ");
  Serial.println(openTime-start);
  
  if (myFile) {
    while(myFile.available())
    {
      String list = myFile.readStringUntil('\r');
      //Serial.print(list);
    }
    unsigned long finishReading = millis();//replace
    Serial.print("Time to read 999 entries from file: ");
    Serial.println(finishReading-openTime);
  
    myFile.close();
    unsigned long endTime = millis();//replace
    Serial.print("Time to close file: ");
    Serial.println(endTime-finishReading);
  }  
}

void loop() {

//  nothing happens, loop is in setup
  
}
