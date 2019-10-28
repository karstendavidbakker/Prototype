
const int buttonPin = 2;        // declare the number of the pushbutton pin
int buttonState = 0;            // variable for reading the pushbutton status

long lastDebounceTime = 0;      // the last time the output pin was toggled
long debounceDelay = 50;        // the debounce time; increase if the output flickers

#include <Wire.h>               // include this library
#include <Adafruit_Sensor.h>    // include this library for the sensor
#include <Adafruit_BNO055.h>    // include this library for the sensor
#include <utility/imumaths.h>   // include this library for the sensor

#include <Adafruit_GPS.h>       // include the GPS library
#define GPSSerial Serial1       // define the serial hardware used here
Adafruit_GPS GPS(&GPSSerial);   // define some serial stuff
#define GPSECHO false
uint32_t timer = millis();

#define BNO055_SAMPLERATE_DELAY_MS (100)           // set the time delay between accelerometer readings

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);   // some details about the sensor

      void displaySensorDetails(void)    // Function to read sensor details in the serial monitor
{
  sensor_t sensor;                      // Get the sensor details from the chip
  bno.getSensor(&sensor);               // Get the sensor details from the chip

  delay(500);
}

void displaySensorStatus(void)       // Function to display sensor details in the serial monitor
{

  uint8_t system_status, self_test_results, system_error;                     // Get the system status values (mostly for debugging purposes) //
  system_status = self_test_results = system_error = 0;                       // Get the system status values (mostly for debugging purposes) //
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);     // Get the system status values (mostly for debugging purposes) //
  delay(500);
}

void displayCalStatus(void)                                   // Function to read sensor calibration details in the serial monitor
{

  uint8_t system, gyro, accel, mag;                           // Get the four calibration values (0..3) //
  system = gyro = accel = mag = 0;                            // Any sensor data reporting 0 should be ignored //
  bno.getCalibration(&system, &gyro, &accel, &mag);           // 3 means 'fully calibrated" //

  Serial.print("\t");
  if (!system)                  // The data should be ignored until the system calibration is > 0 //
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
void getdata() {                            // Function to read sensor details and display them in the serial monitor

  sensors_event_t event;                    // Fetch data from the sensor //
  bno.getEvent(&event);                     // Fetch data from the sensor //


 imu::Vector<3> accel2 = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  /* Display the floating point data */

  Serial.print("IMU2,");
  Serial.print(accel2.x());
  Serial.print(", ");
  Serial.print(accel2.y());
  Serial.print(", ");
  Serial.print(accel2.z());
  Serial.print("\t\t");

 // Serial.print( "IMU, ");
//  Serial.print(event.orientation.x, 4);     // Display the floating point data //
//  Serial.print( ",");                       // Seperate with a comma for ease of use in .csv file //
//  Serial.print(event.orientation.y, 4);     // Display the floating point data //
//  Serial.print( ",");                       // Seperate with a comma for ease of use in .csv file //
 // Serial.print(event.orientation.z, 4);     // Display the floating point data //

  int i= i+1 ;                              // i increases
  Serial.println("");                       // New line for the next sample //
  delay(BNO055_SAMPLERATE_DELAY_MS);        // Wait the specified delay before requesting nex data //
}


void setup() {

  pinMode(buttonPin, INPUT);          // Declare the button as an input //
  Serial.begin(115200);               // Set the baud rate //

  if(!bno.begin())                    // Initialise the sensor //

  bno.setExtCrystalUse(true);

  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);
}

void loop() {

char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
  if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
 //   Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another

  buttonState = digitalRead(buttonPin);                       // Sample the state of the button - is it pressed or not? //
  if ( (millis() - lastDebounceTime) > debounceDelay) {       // Filter out any noise by setting a time buffer //

  if (buttonState == HIGH)  {


      Serial.print ("GPS,") ;
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

    //if the button has been pressed,  toggle the LED from "off to on" or "on to off" //
        for (int i = 0; i <= 100; i++) {                      //100  Call the data capture to happen in a desired number of instances of  i (rising) //

   getdata() ;                                                // Trigger the data capture function

}
        }

         lastDebounceTime = millis();                         //set the current time
  }
  }}
