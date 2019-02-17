#include <Arduino.h>

#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <utility/imumaths.h>

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
void read_data();
static void store_data();

void setup() {
  Serial.begin(9600);
  
  if (!bno.begin())
    Serial.println("Error: BNO055 not detected.");
  if (!bmp.begin())
    Serial.println("Error: BMP280 not detected.");

  SD.begin(card_chipSelect);
  create_file();
  ground_alt = bmp.readAltitude();
  
  delay(1000);

  alt_previous = ground_alt;
  t_previous = millis();
}

void loop() {
  read_data();
  store_data();
}

static int create_file() {
  // TODO
  return 0;
}

void read_data()
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
  // TODO

  // Edit to add GPS stuff
  datastring = "";
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
  datastring += "E";

  // SEND TO XBEE
  // TODO
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