
// HERE WE ARE GOING TO DEFINE SOME VARIABLES THAT CAN BE RECALLED LATER IN THE SCRIPT //

const int buttonPin = 2;        // declare the number of the pushbutton pin
int buttonState = 0;            // variable for reading the pushbutton status
long lastDebounceTime = 0;      // the last time the output pin was toggled
long debounceDelay = 50;        // the debounce time; increase if the output flickers
uint32_t timer = millis();      // set the counter time
Adafruit_GPS GPS(&GPSSerial);   // define some serial stuff
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);   // some details about the sensor

// HERE WE ARE GOING TO INCLUDE SOME LIBRARIES USED TO RUN THE SENSORS AND INTERPRET THE DATA //

#include <Wire.h>               // include this library for the sensor to work
#include <Adafruit_Sensor.h>    // include this library for the sensor to work
#include <Adafruit_BNO055.h>    // include this library for the sensor to work
#include <utility/imumaths.h>   // include this library for the sensor to work
#include <Adafruit_GPS.h>       // include the GPS library for the sensor to work

// HERE WE DEFINE SOME FACTORS AND ATTRIBUTES OF THE SCRIPT AND ITS COMPONENTS //

#define GPSSerial Serial1                          // define the serial hardware used here for the GPS sensor
#define BNO055_SAMPLERATE_DELAY_MS (100)           // set the time delay between accelerometer readings
#define GPSECHO false                              // set the echo to false otherwise raw data is pumped out

// HERE WE DEFINE THE SCRIPT USED TO DISPLAY THE IMU SENSOR DETAILS //

void displaySensorDetails(void)         // Function to read sensor details in the serial monitor
{
  sensor_t sensor;                      // Get the sensor details from the chip
  bno.getSensor(&sensor);               // Get the sensor details from the chip
  delay(500);
}

// HERE WE DEFINE THE FUNCTION USED TO READ AND PRINT THE STATUS OF THE IMU SENSOR //

void displaySensorStatus(void)          // Function to display sensor details in the serial monitor
{

  uint8_t system_status, self_test_results, system_error;                     // Get the system status values (mostly for debugging purposes) //
  system_status = self_test_results = system_error = 0;                       // Get the system status values (mostly for debugging purposes) //
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);     // Get the system status values (mostly for debugging purposes) //
  delay(500);
}

// HERE WE DEFINE THE FUNCTION USED TO DISPLAY THE CALIBRATION DETAILS OF THE IMU CHIP //

void displayCalStatus(void)                                   // Function to read sensor calibration details in the serial monitor
{

  uint8_t system, gyro, accel, mag;                           // Get the four calibration values (0..3) //
  system = gyro = accel = mag = 0;                            // Any sensor data reporting 0 should be ignored //
  bno.getCalibration(&system, &gyro, &accel, &mag);           // 3 means 'fully calibrated" //

  Serial.print("\t");
  if (!system)                     // The data should be ignored until the system calibration is > 0 //
  {
    Serial.print("! ");
  }
  Serial.print("Sys:");            // Display the individual values //
  Serial.print(system, DEC);       // Display the individual values //
  Serial.print(" G:");             // Display the individual values //
  Serial.print(gyro, DEC);         // Display the individual values //
  Serial.print(" A:");             // Display the individual values //
  Serial.print(accel, DEC);        // Display the individual values //
  Serial.print(" M:");             // Display the individual values //
  Serial.print(mag, DEC);          // Display the individual values //

}

// HERE IS THE FUNCTION TO OPERATE THE IMU SENSOR AND PRINT THE DETAILS IN THE SERIAL MONITOR //

void getdata() {                            // Function to read sensor details and display them in the serial monitor

  sensors_event_t event;                    // Fetch data from the sensor //
  bno.getEvent(&event);                     // Fetch data from the sensor //

 imu::Vector<3> accel2 = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);    // function to translate sensor data into useful vector information //

// HERE IS HOW WE WANT THE DATA TO BE PRINTED IN THE SERIAL MONITOR SO THAT IT CAN BE INTERRETED BY THE DCD HUB //

  Serial.print("IMU2,");         // Display the floating point data //
  Serial.print(accel2.x());      // Display the floating point data //
  Serial.print(", ");            // Display the floating point data //
  Serial.print(accel2.y());      // Display the floating point data //
  Serial.print(", ");            // Display the floating point data //
  Serial.print(accel2.z());      // Display the floating point data //
  Serial.print("\t\t");          // Display the floating point data //

  int i= i+1 ;                              // i increases - used for setting data read time //
  Serial.println("");                       // New line for the next sample //
  delay(BNO055_SAMPLERATE_DELAY_MS);        // Wait the specified delay before requesting nex data //
}

// HERE IS THE FUNCTION USED TO SETUP AND CONFIGURE THE ARDUINO ONCE IT IS POWERED ON //

void setup() {

  pinMode(buttonPin, INPUT);          // Declare the button as an input //
  Serial.begin(115200);               // Set the baud rate of the serial monitor //

  if(!bno.begin())                    // Initialise the sensor //
  bno.setExtCrystalUse(true);

// SCRIPT USED TO SETUP AND INITIALISE THE GPS SENSOR //

  GPS.begin(9600);                                              // declare GPS data read speed //
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);                 // send the NMEA capture command to the sensor for RMCGGA data //
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);                // send the NMEA capture command to the sensor for RMConly data //
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);                    // 1 Hz update rate //
  GPS.sendCommand(PGCMD_ANTENNA);                               // Display the floating point data //
  delay(1000);                                                  // Delay //
  GPSSerial.println(PMTK_Q_RELEASE);                            // print release version //
}

// HERE IS THE CONTINUOUS SCRIPT USED TO RUN THE ARDUINO ONCE THE SETUP LOOP IS FINISHED //

void loop() {

// HERE WE MAKE THE GPS CHIP SEND AND RECEIVE COMMANDS FROM THE SATELLITES, AND THEN FORMAT IT INTO USABLE INFORMATION //

  char c = GPS.read();                        // Read the GPS sensor //
  if (GPSECHO)                              // if echo function data is captured in this script //
  if (c) Serial.print(c);                   // if data is received from the sensor then parse and print it //
  if (GPS.newNMEAreceived()) {              // if new NMEA data is received //
  if (!GPS.parse(GPS.lastNMEA()))           // this also sets the newNMEAreceived() flag to false //
  return;                                   // we can fail to parse a sentence in which case we should just wait for another //

// HERE WE ASSESS HOW LONG IT HAS BEEN SINCE THE BUTTON HAS BEEN PRESSED IN ORDER TO OVERCOME NOISE AND MECHANICAL POOR PERFORMANCE (DEBOUNCE) //

  buttonState = digitalRead(buttonPin);                       // Sample the state of the button - is it pressed or not? //
  if ( (millis() - lastDebounceTime) > debounceDelay) {       // Filter out any noise by setting a time buffer //

  if (buttonState == HIGH)  {                                       // if the button has been pressed //

// HERE IS THE FUNCTION USED TO PRINT THE GPS DETAILS TO THE SERIAL MONITOR //

      Serial.print ("GPS,") ;                                       // print the GPS tag //
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);         // print the GPS latitude data //
      Serial.print(", ");                                           // print a comma //
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);      // print the GPD longtitude data //

// HERE WE CALL THE IMU READ FUNCTION TO HAPPEN 100 TIMES //

 for (int i = 0; i <= 100; i++) {                      // Call the data capture to happen in a desired number of instances of  i (rising) (100) //

// RUN THE VOID FUNCTION DECLARED EARLIER IN THE SCRIPT TO READ FROM THE IMU //

   getdata() ;                                                // Trigger the data capture function

}}

//HERE WE RESET THE CLOCK TO SEE IF THE BUTTON HAS BEEN PRESSED AGAIN //

   lastDebounceTime = millis();                         //reset the current time

  } }}
