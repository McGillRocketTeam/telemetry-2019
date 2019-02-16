#include <Arduino.h>

#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <utility/imumaths.h>

#define numBiasMeas 10  // number of times to measure the bias

Adafruit_BMP280_Unified bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Configuration vars
float freq = 2000;
String file_prefix = "data";  // file name prefix for datalogging
String event_file_prefix = "log";  // event log file name
String file_type = ".txt";  // file type
String header = "millis, Altitude, Pressure, Temperature, Accel_x, Accel_y, Accel_z, Mag_x, Mag_y, Mag_z, Gyro_x, Gyro_y, Gyro_z, Num_Sats, Latitude, Longitude, GPS_Speed, GPS_Altitude, GPS_Time, GPS_Date, GPS_CharsProcessed"; // CSV file header
const int card_chipSelect = BUILTIN_SDCARD;

// Functional vars
double bias = 0.0;
float temperature, pressure, alt, mag_x, mag_y, mag_z, gyro_x, gyro_y, gyro_z, 
      accel_x, accel_y, accel_z, time_passed, t_previous, alt_previous, vel, accel_magnitude;

void setup() {
  Serial.begin(9600);
  
  if (!bno.begin())
    Serial.println("Error: BNO055 not detected.");
  if (!bmp.begin())
    Serial.println("Error: BMP280 not detected.");

  SD.begin(card_chipSelect);
  create_file();
  set_bias();

  delay(1000);
}

void loop() {
  read_data();
  store_data();
}

void set_bias() {
  for (int i = 0; i < numBiasMeas; i++) {
    bias += getPressure();
  }
  bias /= numBiasMeas;
}

static int create_file() {
  // TODO
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

  imu::Vector<3> mag_euler = bno.getVector(Adafruit_BNO055::MAGNETOMETER);
  mag_x = mag_euler.x();
  mag_y = mag_euler.y();
  mag_z = mag_euler.z();

  bmp.readTemperature(&temperature);
  bmp.readPressure(&pressure);
  alt = bmp.pressureToAltitude(bias, pressure);

  time_passed = millis();

  // Calculate velocity and magnitude of acceleration
  vel = (alt - alt_previous)/((time_passed - t_previous)/1000);
  accel_magnitude = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  t_previous = time_passed;
  alt_previous = alt;    
   
  //Read from GPS
  // TODO
}

static void store_data(){
  // TODO
}