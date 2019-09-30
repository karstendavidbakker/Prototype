<span style="text-decoration:underline;">Design Brief ID5415</span>

# Team 6 members:

Jordan Kelly		- 4917189

Casper Beers		- 4975421

Karsten Bakker	- 4278887

# Our mission:

Our mission is to develop a connected wheelchair that can help cities and disabled users to detect hazardous situations for wheelchair users. Which the city can then resolve and wheelchair users can be informed of on forehand. In such a way that the wheelchair user can make the decision to take a caregiver with him/her or go about the situation on their own.

# What is the goal of your product?

IP adress: 145.94.218.61

*   Making the lives of disabled people in wheelchairs easier by warning them for obstacles in cities
*   Extracting data about wheelchair obstacles in cities to improve the cities architecture
*   Create a wheelchair to city planning platform to discover new wheelchair design requirements.

# What features should it have?



*   Rechargeable power source
*   Fixing method
*   GPS location tracking
*   Long-range communication method (internet connectivity)
*   Accelerometer
*   Push-button

# What UX do you envision?

Button to start the recording of a hazardous situation. Website to see if buildings posses any danger. Backend in which the municipality can manage the status of different obstacles.

# What data should it collect?

Location, tilt, acceleration, caregiver or not. Can be developed towards pushing power needed. All to develop an image of the situation

What is the benefit for the user?

Using our connected product it will be possible for wheelchair users to make more informed decisions about travelling and visiting new places, avoiding ones that cause significant inconvenience and ensuring that they have adequate assistance in challenging locations.

# What is the benefit for society?

The information and data gathered using our device will reward greater societal awareness of the everyday challenges posed to wheelchair users, allowing for changes to be made that can create a safer, more enjoyable, and a more welcoming society.

# What is the desired functionality?

Our device hopes to function in both passive and active information capture modes. Using the device, it will be possible for the user to ‘pin’ locations that are considered excessively challenging for wheelchair users. This information can be represented online, allowing for the proprietors of buildings to become aware of the problems facing their visitors.

The device will also capture information about the motion of the wheelchair across a period of time. Amassing this data captured across multiple journeys and users can be combined to highlight particular routes or areas that pose inconvenience to the people.

# What is the envisioned stack?

Device - communication - cloud - server - UX

sensor

Arduino

RB

Lecture 2:
Added


Acceloratmeter - IMU: (frequenty meter, gyroscope) To clasify terrain and obstacles. Frequenty of terain, height of obstacles

GPS (Mobile Phone): Location & elavation tracking in coordinates

Capacity sensor: (Presure sensor): Duration a frequenty of handle contact

#find serial port command
ls /dev/tty*


*   bluetooth

Phone				-->UX user

wifi

Cloud

TU.Delft server

Interface				-->UX company



*   GPS Sensor
*   Wifi- gateway (RP)
*   Tilt sensor: potentio meter?
*   Force? - push resistance?

# login raspberry:
# ssh pi@goodthing.local

# cd (change to folder)
# ls (list)
# cd .. (folder above)
# python3 [filename] //to run file
# control z //to stop script

# to do:
- make a architecture
- reserve a slot?

Name of bottem right serial port raspberry;
/dev/ttyACM0

##Steps

1.[x] button running on Arduino
[]troubleshoot usb serial port error
[ ]
[ ] comment all lines in the used code
[ ] get the random-data.py script to send data to the dcd Hub
[ ] get the serial-data.py script to send data to the dcd hub.
2.[ ] Serial data to webserver on pc
3.[ ] Serial data to Rpi            
4.[ ] Accelerometer running on Arduino
5.[ ] Serial data to webserver on pc
6.[ ] Serial data to Rpi
7.[ ] Rpi to bluetooth
8.[ ] Phone to Cloud

# Is there a cloud connection on the app

# raspberry eduroam connection

# writing line of code on bluetooth

# Steps OK?

# comment
# take a slice of time and link that to a frequency. Look at amounts of zero crossing. How much is your max and your min in relation to 1 Sec.

# webserver running on raspberry pi - D3 libaries to chose from, able to adapt to what to do.

# jupiter notebook to do static analysis

# next week: intensively collect data.

# Use hub: combinate the data
