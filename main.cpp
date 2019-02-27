#include <Arduino.h>

#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>

#define commserial Serial2

Adafruit_BMP280 bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Configuration vars
float freq = 2000;
String file_prefix = "data";  // file name prefix for datalogging
String event_file_prefix = "log";  // event log file name
String file_type = ".txt";  // file type
String header = "millis, Altitude, Pressure, Temperature, Accel_x, Accel_y, Accel_z, Mag_x, Mag_y, Mag_z, Gyro_x, Gyro_y, Gyro_z, Num_Sats, Latitude, Longitude, GPS_Speed, GPS_Altitude, GPS_Time, GPS_Date, GPS_CharsProcessed"; // CSV file header
const int card_chipSelect = BUILTIN_SDCARD;

// Functional vars
int file_num = 0;  // file number iterator
String datastring;
float ground_alt;
unsigned long previous_file_size = 0;
float temperature, pressure, alt, mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z, 
      accel_x, accel_y, accel_z, time_passed, t_previous, alt_previous, vel, accel_magnitude;

static int create_file();
static void read_data();
static void store_data();

/*GPS*/
#define mySerial Serial1

Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  true

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void setup() {
  Serial.begin(9600);
  
  if (!bno.begin())
    Serial.println("Error: BNO055 not detected.");
  if (!bmp.begin())
    Serial.println("Error: BMP280 not detected.");

  SD.begin(card_chipSelect);
  create_file();
  ground_alt = bmp.readAltitude();
  
  commserial.begin(9600);

  GPS.begin(4800);
  mySerial.begin(4800);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!

#ifdef __arm__
  usingInterrupt = false;  //NOTE - we don't want to use interrupts on the Due
#else
  useInterrupt(true);
#endif

  delay(1000);

  // initial values
  alt_previous = ground_alt;
  t_previous = millis();
}

#ifdef __AVR__
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
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
#endif //#ifdef__AVR__

uint32_t timer = millis();

void loop() {
  read_data();
  store_data();
}

static int create_file() {
  //load name of file
  String file_name = file_prefix + file_num + file_type;
  char data_file_char_array[file_name.length() + 1];
  file_name.toCharArray(data_file_char_array, file_name.length() + 1);

  //create a file
  if (SD.exists(data_file_char_array)){return 0;}
  else{
    File temp = SD.open(data_file_char_array, FILE_WRITE);
    temp.close();
  }
  return 0;
}

static void read_data()
{
  imu::Vector<3> accel_euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  accel_x = accel_euler.x();
  accel_y = accel_euler.y();
  accel_z = accel_euler.z();

  imu::Vector<3> gyro_euler = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  gyro_x = gyro_euler.x();
  gyro_y = gyro_euler.y();
  gyro_z = gyro_euler.z();

  imu::Vector<3> mag_euler = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  mag_x = mag_euler.x();
  mag_y = mag_euler.y();
  mag_z = mag_euler.z();

  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();
  alt = bmp.readAltitude() - ground_alt;

  time_passed = millis();

  // Calculate velocity and magnitude of acceleration
  vel = (alt - alt_previous)/((time_passed - t_previous)/1000);
  accel_magnitude = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  t_previous = time_passed;
  alt_previous = alt;    
   
  //Read from GPS
   // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  datastring = "S";
  datastring += String(GPS.fix);
  datastring += ",";

  // DEBUGGING
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);

      datastring += String(GPS.latitude);
      datastring += GPS.lat;
      datastring += ", ";
      datastring += String(GPS.longitude);
      datastring += GPS.lon;
      datastring += ",";
      datastring += String(GPS.speed);
      datastring += ",";
      datastring += String(GPS.angle);
      datastring += ",";
      datastring += String(GPS.altitude);
      datastring += ",";
      datastring += String(GPS.satellites);
      datastring += "\n";
    }
  
    datastring += alt;
    datastring += ",";
    datastring += String(time_passed,0);
    datastring += ",";
    datastring += temperature;
    datastring += ",";
    datastring += vel;
    datastring += ",";
    datastring += accel_magnitude;
    datastring += ",";
    datastring += "E";  // end of transmission

  // SEND TO XBEE
  commserial.println(datastring);
  }
}

static void store_data() {
  datastring += String(accel_x,6);
  datastring += ",";
  datastring += String(accel_y,6);
  datastring += ",";
  datastring += String(accel_z,6);
  datastring += ",";
  datastring += String(mag_x,6);
  datastring += ",";
  datastring += String(mag_y,6);
  datastring += ",";
  datastring += String(mag_z,6);
  datastring += ",";
  datastring += String(gyro_x,6);
  datastring += ",";
  datastring += String(gyro_y,6);
  datastring += ",";
  datastring += String(gyro_z,6);
  Serial.println(datastring); 
  
  String file_name = file_prefix + file_num + file_type;
  char data_file_char_array[file_name.length() + 1];
  file_name.toCharArray(data_file_char_array, file_name.length() + 1);

  if (SD.exists(data_file_char_array)){
   File dataFile = SD.open(data_file_char_array, FILE_WRITE);

    if (dataFile) {
      dataFile.println(datastring);  // write to SD card
      if (dataFile.size() > previous_file_size) {
        previous_file_size = dataFile.size();
      }
      dataFile.close();
    }
  }
}
