<span style="text-decoration:underline;">Design Brief ID5415</span>

# ID5415 - Prototyping Connected Products

## Final report- Team 6

*to use this repository run the server.py code as a service on a rpi and the working arduino code on the arduino while being connected. go to the ipadress of the pi  ipadress/5000/map4 to see the working map to which markers are added.* 

![alt_text](images/image7.png "image_tooltip")


---

## Team members:


Jordan Kelly - 4917189

Casper Beeris - 4975421

Karsten Bakker - 4278887



---
**Contents:**



1. Introduction
2. Concept description
3  Instructable
4. Recommendations
5. Reflection

---
 # 1. Introduction

This report serves to outline the design development process for the WheelCare system - a connected Internet of Things product that hopes to aid in the improvement of facilities for wheelchair users.


This project is being completed as part of the ID5415 Prototyping connected products elective as  part of the MSc Integrated Product Design course at TU Delft, Netherlands.


The project brief is as follows:


    _‘While the population of wheelchair users is growing worldwide, it becomes urgent to design supportive technologies that fit their needs. We aim to develop products for improvement of the wheelchair users’ well-being. This design is a connected product that collects data from sensors, processes it in order to actuate user interactions embedded on the wheelchair.’_


The project hopes to integrate the local computing system into an internet-based data management platform, where the gathered information can be processed and made into comprehensible insights for the users of the system. and About the Internet of Things:

---
  # 2. Concept description

  The WheelCare system is an IoT device used to identify locations and hazards that are unsafe or unsuitable for wheelchair users. Using the device, movement and location data can be captured and accessed online, where wheelchair users can be warned about treacherous or difficult journeys. Furthermore, this information can be used by government bodies and proprietors alike to make improvements to their buildings and facilities.

![alt_text](images/Button_1.png)

  Should a user encounter a potential hazard or unsuitable terrain they can log this to the online site by using the system’s button module.  An accelerometer module affixed to the chair’s frame then logs a period of movement data and transmits this to the cloud where it is processed to identify the scale or type of the hazard. This information is then available to view on a map, to chart an area’s risks and their severity for wheelchair users.

![alt_text](images/PB040299.jpg)

  The system architecture is described in the above diagram. At the heart is a Raspberry Pi, which is used to capture and send data to the cloud. An Arduino Mega microcontroller is responsible for operating the accelerometer and the push-button module.


  It is hoped that the system will be purchased by wheelchair users who want to ensure a more convenient and autonomous life by foreseeing and thus avoiding potential hazards that might otherwise require assistance. Likewise, government bodies and proprietors who wish to provide safe and wheelchair-friendly facilities can purchase this system for use in mapping and research applications.


![alt_text](images/image1.png)

  The local system is powered using a standard power bank, and can operate for up to FIXME hours on a single charge.


  The online platform makes use of TU Delft’s Data Centric Design Hub for the information processing system.


  The local system represents the integration of the coding languages C++ and Python, and the online platform utilises HTML, CSS & JavaScript.

  The key drivers of the concept are listed below.

| Wishes | Solutions |
|--------|-----------|
| Wheelchair users should be **motivated to share** their data | sharing data is allows wheelchair user to **get their problems solved** by municipalities|
|Wheelchair users should **benefit from the data** | Wheelchair users can avoid obstacles in cities And thereby **travel safer and faster** |
| Wheelchair data is of **interest to third parties**(municipalities) | Municipalities get **direct customers feedback** in a structured way |


---
# 3. Instructable
# Tutorial: How to built a “city wheelchair problem mapping” prototype?
*For Beginners*

The aim of of this tutorial is to get the basic principles of IoT connected products, learning about the (coding) requirements and setting up connections to be able to gain practical skills. This with the goal to translate connect concepts ideas in concrete functional prototypes.

*Goal & Purpose*

For this project the purpose of this prototype was to validated the accuracy of the GPS data in an outdoor environment. Additionally this prototypes allows to gather insights in the the experience of individuals marking problems in the city.

<span style="text-decoration:underline;">In this tutorial you will learn about the main following subjects:</span>



*   C++ coding of arduino GPS module
*   Collecting IMU data
*   Extracting and emitting clean data sensorry data
*   Installation of raspberry Pie
*   Set-up website server on raspberry pie
*   Connecting and processing data from serial port (arduino) to webserver
*   Integrating Google API’s in your system
*   Emitting GPS data from physical to online digital prototype
*   Collecting and storage of sensory data on a hub  

**<span style="text-decoration:underline;">Required Materials:</span>**



*   3D printer or casing prototype toolings,
*   press button,
*   bolts,
*   hot glue,
*   Wheelchair
*   wiring,
*   Resistor
*   Zero PCB
*   Soldering materials
*   IMU arduino sensor type
*   GPS Module
*   Rasberry Pi
*   Digital Button



**Tips & Tricks: General and basic instructions to take into account when executing this (IoT) project.**



*   Ensure individual pieces of the code or prototype work before setting up connections. If possible test. For example by options like: serial monitor, saving data to file, loading fake data in file, or
*   For debugging output to a web server use right mouse click “inspect” to find errors not appearing in your coding program.
*   Integrated print() functions in between lines of code to validate the output of your program
*   Note that setting up IoT connections are in general difficult. Testing pieces of code individually could ease tackling this complexity. Recommended when rewriting files is to make a backup from the last operating status.





**Hardware development**

The prototype features three 3D printed hardware components located at the following locations on the frame of the wheelchair:


![alt_text](images/image8.png "image_tooltip")


Each part was modelled using Fusion 360 and printed using a Prusa I3 MK3s FDM printer in PLA material.

The housings were employed to achieve the following goals:


*   To house the electronic components on the frame of the wheelchair, ensuring that they are held stably during folding and unfolding of the wheelchair.
*   To protect them from environmental conditions such as dirt, rain and UV damage.
*   To securely affix the IMU in place on the frame, ensuring repeatability and validity between various readings. Should the sensor move throughout its lifetime, the way in which accelerometer data recorded across the three axes may change, resulting in incorrect or inaccurate labelling.
*   To provide ease of use for users with reduced hand dexterity and motor function when interacting with small features (such as push-buttons and power switches).

In total, three iterations of housings were made. Incremental improvements were made to the system as follows:

**MK1 housings**


Rationale: for the first time integrate the components onto the wheelchair frame.


Description: this iteration proved successful in that the entire system could be integrated within the wheelchair frame, powered on its own source and operating autonomously.


To improve: The zip-tie method for affixing the parts proved unstable, and should be replaced with a bolt-on feature. The location of the electronics housing should be changed to ensure that the wheelchair can be folded and stowed.


**MK2 housings**


Rationale: incorporate bolt-on fixings and test new housing location.



Description: This iteration served as a stepping stone on which to test the relationship of the fixing hardware


To improve: The layout of the internal components must be altered to ensure that the frame can be folded properly.


**MK3 housings**


Rationale: final prototype housing production.


![alt_text](images/Housing Diagram_REV4-01.png)


Description:



![alt_text](images/image6.png)


L-R: electronics housing, button holder, IMU holder.

3. **Arduino Microcontroller**

Located on the wheelchair frame, the arduino Mega2560 microcontroller serves as the first system that the user interacts with in the WheeallCare concept

The responsibility of the Arduino Microcontroller is



*   To operate the Adafruit BN055 IMU accelerometer sensor
*   To operate the momentary push-button
*   To operate the Adafruit Ultimate GPS sensor.
*   To relay the captured information via serial connection to the Raspberry Pi.

The system was coded in such a way that:



*   The IMU and GPS sensor were constantly running to capture location and movement data.
*   At the push of a button, ten seconds of IMU data and the current GPS location was sent to the serial monitor.
*   Connected to the Pi via serial, this print of information could be translated to a .csv file by the pi for processing in the DCD hub.

The development process of the Arduino microcontroller was as follows:



1. Push-button added to display ‘1’ when pushed and ‘0’ when not pushed in the Serial monitor.
2. A de-bounce was added to the code to improve the performance of the button.
3. The IMU was added, and programmed to print X, Y and Z Euler angle data stream into the serial monitor.
4. IMU programmed to operate when the push-button is used.
5. IMU programmed to print X number of values, corresponding to ten seconds of data capture.
6. IMU programmed to print the angular acceleration in place of euler angle.
7. GPS module added to the arduino system. Raw data stream printed into the serial monitor.
8. GPS data parsed and formatted to give readable information for the DCD hub.
9. Final code formulated to include x1 GPS location and IMU data in a serial monitor readout once the button is pressed.

System diagram:



![alt_text](images/image3.png "image_tooltip")


**ARDUINO CODE**

```c+

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


```

**Raspberry Pi**

The Raspberry Pi has two jobs. It acts as a processor for the data outputted from the Arduino and sends this data to ether the dcd hub for storage or to a website to generate markers. It also acts as the server that hosts the website used by the client to inspect marked hazards.

A Raspberry pi runs on raspbian. There is a plethora of tutorials on how you can install this on your raspberry.

Once Raspbian is installed you'll need internet. Hook your rpi up to a screen, mouse and keyboard. And log in to your regular wifi via the icon in the top right corner.

once this is installed you can install the most important packages. 

git you can install by typing
```
$ sudo apt install git-all
```

after this is installed you can download this repository to you rpi by cloning it
```
sudo mkdir Prototype
cd Prototype
git init
git remote add origin https://github.com/karstendavidbakker/Prototype.git
git fetch
git pull
```
now you have all our files in a folder named Prototype.


When you have set up the arduino as stated before you can run the server.py script to start a server and start logging your gps and imu data!
you can do this by:
```
cd Prototype/web/
python3 server.py
```


The final working script is the following:
```python
#import all needed packages
#for serial port
import serial

#for dcd hub interaction
from dcd.entities.thing import Thing
from dcd.entities.property import PropertyType
from dotenv import load_dotenv
import os

# for site and websocket
from flask import Flask, request, render_template
from flask_socketio import SocketIO, emit, send

#for date time and random variables
import datetime
import time
from random import random

#for parralel processing
from threading import Thread

#load .env file
load_dotenv()
#placeholder data for gps
GPS_data = ["00.00000", "0.000000"]
#dcd thing connection setup
THING_ID = os.environ['THING_ID']
THING_TOKEN = os.environ['THING_TOKEN']

# define my thing using
my_thing = Thing(thing_id=THING_ID, token=THING_TOKEN)

#populate thing
my_thing.read()

# Start a connection to the serial port, switch port if ther is an exception.
try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
except:
    try:
        ser = serial.Serial('/dev/ttyACM1', 115200, timeout=2)
    except:
        ser = serial.Serial('/dev/cu.usbmodem14431', 115200, timeout=2)

# Read the next line from the serial port
# and update the property values
def serial_to_property_values():
    # Read one line
    line_bytes = ser.readline()
    # If the line is not empty
    if len(line_bytes) > 0:

        # Convert the bytes into string
        line = line_bytes.decode('utf-8')
        print(line)
        # property name is the first argument of the recieved string
        property_name = line[0:3]
        # Split the string using commas as separator, we get a list of strings
        if property_name == "GPS":
            #reformat incomming gps string, so google maps can read it
            values = line.replace("N","").replace("E","").replace(".","")
            values2 = values[:6]+"."+values[6:15]+"."+values[15:]
            values3 = values2.replace(":", ",").split(',')
        #if it is a IMU data point
        else:
            #format to a list type
            values3 = line.replace(":", ",").split(',')
        #strip of special tokens like /n /r enz.
        print(values3)
        # Use the first element of the list as property id
        property_name = values3.pop(0)
        # gps values get put into gps data variable
        try:
            if property_name == "GPS":
                GPS_values = [float(i) for i in values3]
                print(GPS_values)
                GPS_values[1] = GPS_values[1]+0.148
                GPS_values[0] = GPS_values[0]+0.0008
                print(GPS_values)
                return GPS_values
            # update IMU values
            else:
                print(property_name)
                prop_123 = my_thing.find_or_create_property(property_name,
                                                               PropertyType.THREE_DIMENSIONS)
                # If we find the property, we update the values (rest of the list)
                if prop_123 is not None:
                    prop_123.update_values([float(x) for x in values3])
                # # Otherwise, we show a warning
                else:
                    print('Warning: unknown property ' + property_name)
        except ValueError:
            print("no gps fix")


def serial_gps_data():
    try:
        while True:
            print("emiting gps location")
            #try:
            print("begin serial_to_property_values")
            GPS_data = serial_to_property_values()
            #except:
            #print("exception on serial_to_property_values")
            if GPS_data is not None:
                json = {"gps": GPS_data}
                print(json)
                socketio.emit('gpslocation', json, broadcast=True)
            time.sleep(1)
    except KeyboardInterrupt:
        exit()
 # haalt data (connectie) uit serial van raspberry pie, Na try leest hij die poort uit, of anders na except een andere poort.

current_time = int(datetime.datetime.utcnow().timestamp()*1000)

# name site(created by flask) app
app = Flask(__name__)

socketio = SocketIO(app)
# functions for the root folder

@app.route('/') #testcomment
# display welcome message
def hello_world():
    return 'Hello, World Vlammen!'


@app.route('/home')
def home():
    return render_template('index.html')

@app.route('/test')
def test():
    return render_template('testvisual.html')


@app.route('/go')
def go():
    return render_template('go.html')


@app.route('/gauge')
def gauge():
    return render_template('gauge.html')

@app.route('/map3')
def map3():
    return render_template('maplocation3.html')

@app.route('/map4')
def map4():
    return render_template('maplocation4.html')


thread = Thread(target=serial_gps_data)
thread.start()

try:
    if __name__ == '__main__':
        socketio.run(app, host = '0.0.0.0')
        # to give everyone from every pc access to web
except KeyboardInterrupt:
    exit()
```

**Goal**

The goal of the raspberry pi is to serve as a hub between the arduino, webserver and dcd hub. When information comes in at the RPi it will process it. The imu data will be compared to a trained model in such a way that the outcome of this model is a label that can be sent to the dcd hub which serves an interface for the municipality. Furthermore the RPi runs the web server that shows the locations of hazzards for wheelchair users from a website.

**Steps**

Development of the Rpi was done using a stepwise incremental approach. This helped reduce the complexity and determining the cause of errors that we came across. We started building connections based on random data. Saving actual data and running a blank webpage. And developed these further to combine their functionalities and converting them to services.
underneath we list the list of seperate programs and their uses that were used to build towards the final connected product.

*    Random to dcd - this is a program that runs on a computer and logs random data to the dcd hub. To test the connection to the dcd hub.

*    Serial to terminal - this script reads the serial of the arduino and prints it to the terminal of the rpi

*    Serial to CSV - this script saves the serial output of the arduino in a csv file on the rpi

*    Serial to dcd - this script logs serial input to the dcd hub testing both connections of the first two scripts

*    Blank server - this script runs a webserver on a pc or rpi without site or interaction

*    Random to server - this script sends random data to the server via a socket and tests this socket connection

*    Serial to server - this script is used to send serial information from the arduino to the rpi and sends it via a socket to the website which can then be seen by any user connected to the same wifi network.

All of these scripts were were also used when backtracking problems to see if everything in the architecture was still working as it should.

**Processing**
A functionallity that this project tried to incorporate was the prediction of the terrain that the wheelchair was rolling on. This is done by using a machine learning algorithm. To set this up you will need to follow the following steps

*Collecting

To know how what features your algorithm should take in you will first need to take in data and see what the features are that set different classes apart from eachother. Classes are in our case nominal terrain, light terrain, intermediate terrain and extreme terrain. See the pictures below for reference.
![Flat ground - nominal terrain](images/Terrain_4.jpg)
Flat ground - nominal Terrain

![Cobblestones - light terrain](images/Terrain_1.jpg)
Cobblestones - light terrain

![Grass - intermediate terrain](images/Terrain_3.jpg)
Grass - intermediate terrain

![Steps - extreme terrain](images/Terrain_2.jpg)
Steps - extreme terrain

When looking at the data of these terrain types we found two features particularly intresting. the amplitude of the wave forms and the zero crossings. These both give information about the wave form that is created by the bumping of the wheel on the terrain. We also found that information on the waveform would best be analysed in a moving window kind of fashion in stead of just comparing 1 timestep to another.

To collect this data collectmodel.py was writen. Sadly due to time issues we didn't solve the moving window code piece. This code should come down to a nested for loop that loops over the lines in a matrix and replaces the oldest with the newest array. 

When collecting make sure the 500 samples that are taken are correct examples of the terrain you're collecting at that point. Wrong data throws the machine learning model off.

*Processing*

processing of the data could be done in the same collectmodel.py and is already there.
The zero crossing script looks like this. Where data is the moving window matrix. And zero_crossings gives the amount of zero crossings found.
```python
zero_crossings = len(np.where(np.diff(np.sign(data)))[0])
```
the maximum amplitude is found in a easier way.
```python
maximum = data.max()
```
these scripts need to be adapted to the colloms that they have affect on.

*Labeling*

labeling is done automatically in the collectmodel.py script.

*Training*

Training of the model can be done by opening the training.ipnyb jupyter notebook. Adjust the time and date to that over the recording of the collection data. And make a connection to your thing in the dcd hub, this is the place where your collected and labeled data has been stored.

running the jupyter file should yield a confusion matrix that shows the certainty of the model that you trained as well as statistics about the accuracy and precision of your model. But most importantly it leaves you with a model.pickl file. This is your trained model that is used when predicting terrain. 

*Predicting*

Prediction can be done by using predict.py script. The result variable could be sent to the webserver via a socket channel and used to show the current predicted terrain or this variable could be sent to the webserver together with the gps information and be placed in the discription of the marker. 

This concludes the gathering and processing of data. 
**Services**

Services is a therm used before in this piece. It is a program that runs in the background of the raspberry pi. Giving you the ability to run several scripts at the same time. This connected product runs 3 services.

*Eduroam service*

The eduroam service makes sure that the rpi connects to wifi on startup.

*Webservice*

The webservice service runs the server.py script on startup. In this way you dont have to connect a screen and keyboard, or log in via ssh to your rpi to start the connected wheelchair functionality. 

*Ip service*

The ip service script logs the current ip adress to the dcd hub so that you can always find the current ip adress and use this to connect to your rpi remotely via ssh. Which can by done by typing ssh pi@'ip adress here' in the terminal of your laptop. After which you can authenticate and control your rpi via the command terminal.

**Current and furture state**

The Rpi is able to receive, process and send data regarding acceleration and gps to the dcd hub and site respectively. The code is robust. Reportedly running hours on end. 

future additions would be to evaluate the incomming imu data and predict what terrain type the wheelchair is rolling on. This prediction should then also be sent to the website and shown coupled to the gps marker placed.



**5. Web platform**

```
<!DOCTYPE html>
<html>
  <head>


    <meta name="viewport" content="initial-scale=1.0, user-scalable=no">
    <meta charset="utf-8">

    <meta name="viewport" content="width=device-width, initial-scale=1">

    <script src="https://kit.fontawesome.com/a076d05399.js"></script>

```

- A Browser's viewport is the area of web page in which the content is visible to the user.

- The HTML <script> src Attribute is used to specify the URL of external JavaScript file. The External JavaScript file is used to run on the same script on several pages on a website.

- The charset attribute specifies the character encoding used in an external script file.

https://fontawesome.com/ enables you to create this external JavaScript file easily for different types and styles of icons you want to place on your webpage.  This can be done in the body of the text by by. 

<i class="fas fa-map-marker-alt" style=font-size:350px;color:blue;>


Create google a interactive maps on your webpage

```
  <title>Marker Clustering</title>
    <script src="//cdnjs.cloudflare.com/ajax/libs/socket.io/2.2.0/socket.io.js"
                integrity="sha256-yr4fRk/GU1ehYJPAs8P4JlTgu0Hdsp4ZKrx8bDEDC3I="
                crossorigin="anonymous"></script>
```

For this prototype multiple GPS locations need to be marked: 

You can use the MarkerClusterer library in combination with the Maps JavaScript API to combine markers and simplify the display of markers on the map.

Find file on github: https://github.com/googlemaps/v3-utility-library/tree/master/markerclusterer


```
    <script src="https://developers.google.com/maps/documentation/javascript/examples/markerclusterer/markerclusterer.js">
    </script>
    <script async defer
    src="https://maps.googleapis.com/maps/api/js?key=AIzaSyD5Ta592tcRaPkglMFLRcFqLCyzM4oppts&callback=initMap">
    </script>
```

By this piece of code the clusterer gets added to the map
src attributes loads maps with key as API

Get the map to operate and output markers by:

STEP0: 	Create a google developers account

STEP1:		 Get google maps API key
         https://developers.google.com/maps/documentation/embed/get-api-key

STEP2: 	Get the desired map for you web application 
        https://console.cloud.google.com/google/maps-apis/api-list?folder=&organizationId=&project=wheelchaircitymap

        Recommended For this tutorial get a “places” or “geolocation” API

STEP3: 	Include an API key with every Maps JavaScript API request. 
        In the following example, replace YOUR_API_KEY with your API key:

         <script async defer
          src="https://maps.googleapis.com/maps/api/js?key=[YOUR-KEY]&callback=initMap">
        </script>

STEP3: 	 Activate account: credit card credentials required.

Note:  Often not mentioned in tutorials causing a hassle in getting API to work: Fully activate Google Developers account by linking credit card credentials (Free for first trial functions)



Create portal to the web - SOCKET.IO


```
      var map;
      var socket = io();
      socket.on('connect', function() {
        console.log("connected");
      });

      socket.on('gpslocation', function(data) {
          console.log(data);
          const latitude = data.gps[0];
          const longitude = data.gps[1]

           new google.maps.Marker({position: {lat:latitude, lng:longitude}, map: map});

```

Create variable ‘map’ to later call in the body of the script by <div id="map"></div>

- Socket.IO allows you to emit and receive custom events. Besides connect, message and disconnect.

- You can create new events like the google marker.

     By this created “portal” this piece of code communicates with:	

    “socketio.emit('gpslocation', json, broadcast=True)”    
    //Written and running in “server.py” file

RECOMMENDED:  include Console.log functions to validate output of the program.

Latitude and longitude are later in this code defined as variables as this information will change over time based on the incoming GPS data.



HTML writing functionalities

```
  <style>
    
      #map {
        height: 100%;
      }

      h1{
        font-size: 90px;
        color: #Black;
        font: Helvetica;
      }

      h2{
        font-size: 60px;
        color: blue;
        font: Helvetica;
      }

      h3{
        font-size: 60px;
        color:  Black;
        font: Helvetica;
      }

      h4{
        font-size: 20px;
        color:  #FFFFFF;
        font: Helvetica;
      }
      /* Optional: Makes the sample page fill the window. */
      html, body {
        height: 100%;
        margin: 1;
        padding: 0;
      }


  </style>
</head>
```

- Always set the map height explicitly to define the size of the div element that contains the map.

- Explain the webpage visitor how to user the prototype and button module by adding text to your webpage.

- Presetting Headers allows easy and structure writing.

- Adjust to preference by changing style: color/font/size/margins etc.

TIP: First get the google maps API Running in as an individual script loading different js. files on your webpage.


```
  <body>


    <i class="fas fa-map-marker-alt" style=font-size:100px;color:black;>       <i class="fab fa-accessible-icon"style=font-size:100px;color:black;></i></i>
    <h4>WheeallCare© City Map</h4>

    <h1><center>WheeallCare© City Map</center></h1>

    <h2><center>- Avoid inaccesible wheelchair spots</center></h2>

    <h2><center>- Share difficult obstacles with other wheelchair users </center></h2>
    <h2><center>- Allow manicuplaties to improve the city & your wheelchair trip</center></h2>
    <br>
    <br>
    <br>
    <br>
    <center><i class="fas fa-map-marker-alt" style=font-size:350px;color:blue;>       <i class="fab fa-accessible-icon"style=font-size:350x;color:blue;></i></i><h2>Drop Your Mark with WheallCare© Module</center></h2>
    <br>
    <br>
    <br>
    <br>

    <h3><center><i>Make the city Wheelchair proof & Enjoy careless rolling ...</i></center></h3>

    <div id="map"></div>
    <script>
    var i;
    var latitude; 
    var longtitude; 
    var lat //= [52.001737,52,51,56];
    var lng //= [4.372825,5,53,5];
    var loc = {};
    var locations = [];

    function initMap() {

      map = new google.maps.Map(document.getElementById('map'), {
        zoom: 18,
        center: {lat:52.001737, lng: 4.372825},
      });
    }

    </script>
  </body>
  <body style="background-color:white;">
</html>
```

Recommended to set the var’s lat and lng first to number to register if a marker is created (do not directly link this to your data file)

document.getElementById() returns the element of specified id. In this case the earlier defined map.

(Useful) Sources: 
     Geeksforgeeks.com
     Google Developers
     javatpoint.com
     W3schools.com
     datacentricdesign.github.io
     https://socket.io/docs


**The website you have created should look like this:

![Open source Webserver to all wheelchair users - Connected to Wheallcare service to mark inaccesible locations](images/4.png)

We made a new page by adding a route to the server.py script.

```python
@app.route('/map4')
def map4():
    return render_template('maplocation4.html')
```

To place markers on the map we use a google api. Which gets the gps coordinates in via a socket. By using the API for google maps this process becomes really easy. 
Making the API ready for use is as easy as importing it in your web page html file

```
    <script src="https://developers.google.com/maps/documentation/javascript/examples/markerclusterer/markerclusterer.js">
    </script>
    <script async defer
    src="https://maps.googleapis.com/maps/api/js?key=AIzaSyD5Ta592tcRaPkglMFLRcFqLCyzM4oppts&callback=initMap">
    </script>
```
making a marker from the imported data can be done like so:

```
  socket.on('gpslocation', function(data) {
          console.log(data);
          const latitude = data.gps[0];
          const longitude = data.gps[1]

           new google.maps.Marker({position: {lat:latitude, lng:longitude}, map: map});
```


---
# 4. Recommendations

The wheeallcare project has much potential. With outsiders asking if we have plans to sell it during the exposition. The case is intresting for municiplity and wheelchair users. However it is not in a final state.

The machine learning part of the system needs to be further developed to become effective. Furthermore conversation with muncipality needs to start to see what actual information they need to make places more accesable. 

The unit should also be made water resistant to enable outdoor use.

The gps only works really well outdoors but is less suitable for indoor tracking. This could be reformulated to use indoor bluetooth beacon or wifi tracking to give a more accurate position.



---
# 5. Reflection:

<table>
  <tr>
   <td>
#
   </td>
   <td>Arguments for decision / reflection on decisions / improve for future prototype
   </td>
  </tr>
  <tr>
   <td>1.
   </td>
   <td>Because our project group was familiar with working with Arduino the GPS module allowed us to rapidly make progress in gathering clean GPS data.
   </td>
   <tr>
    <td>2.
     </td>
    <td>
- We looked in to extracting GPS data by the phone as this would allow our prototype to communicate via 4G with our server. The connection from phone to hub was eventually not satisfactory because it was unclear how to import the data from the dcd hub.
   <td>
   </tr>
     <tr>
    <td>3.
     </td>
    <td>
No information on the Lora GPS module could be found to get it to work in the time available.
   <td>
   </tr>
      <tr>
    <td>4.
     </td>
    <td>
The arduino GPS data turned out to be precise but not too accurate. Additionally this module provide restrictions in freedom of movement as it required a connected to the local WiFi to collect GPS data. In the future an added SIM-card module/phone/lora connection interesting to look into. Nonetheless this solutions was feasible to for setting up our prototype
   <td>
   </tr>
   <tr>
    <td>5.
     </td>
    <td>
 The building of the connected product could have been sped up if a clear roadmap and building strategy was defined before building. Making connections requiered far more time than at first thought. 
   <td>
   </tr>
</table>


