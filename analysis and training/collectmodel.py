# Import required library
from dotenv import load_dotenv
import random
import os
import serial
import time
import pandas as pd
import numpy as np

from dcd.entities.thing import Thing
from dcd.entities.property import PropertyType



# The thing ID and access token
load_dotenv()
THING_ID = os.environ['THING_ID']
THING_TOKEN = os.environ['THING_TOKEN']

# Instantiate a thing with its credential
my_thing = Thing(thing_id=THING_ID, token=THING_TOKEN)
my_thing.read()

LABEL_PROP_NAME = "Terrain"
LABEL_PROP_TYPE = PropertyType.CLASS

DATA_PROP_NAME = "IMU"
DATA_PROP_TYPE = PropertyType.THREE_DIMENSIONS

# Sitting classes
CLASSES = ["flat Terrain", "Medium Terrain", "Grassland"]

# Find label property by name, create classes if none
prop_label = my_thing.find_or_create_property(LABEL_PROP_NAME, LABEL_PROP_TYPE)
if prop_label.classes is None or len(prop_label.classes) == 0:
    prop_label.create_classes(CLASSES)
# Find data property by name
prop_data = my_thing.find_or_create_property(DATA_PROP_NAME, DATA_PROP_TYPE)


# Open a serial connection
def open_serial():
    # Start reading the serial port
        # Start reading the serial port
        try:
            ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
        except:
            try:
                ser = serial.Serial('/dev/ttyACM1', 115200, timeout=2)
            except:
                ser = serial.Serial('COM12', 115200, timeout=2)


# Read the next line from the serial port
# and update the property values
def serial_to_property_values(class_index): #,ser)
    values = []
    #repeat until values array is full
    while len(values) < 15:
    # Read one line
        # line_bytes = ser.readline()
        # # If the line is not empty
        # if len(line_bytes) > 0:
        #     # Convert the bytes into string
        #     try:
        #         line = line_bytes.decode('utf-8')
        #     except UnicodeDecodeError:
        #         return False
        #     # Split the string using commas as separator, we get a list of strings
        #     str_values = line.split(',')
        #     # Remove the first id
        #     str_values.pop(0)
        #     # Transform the array of string values into float values (numbers)
        #     try:
        #         values = [float(x) for x in str_values]
        #     except ValueError:
        #         return False
        values = (random(), random(), random())

    # part to transform x,y,z to sdt, zero crossing, enz.
        # print str_values guess("x","y","z")
        print(values)
        # save last 5 data points in array
        data = []
        data = data.append(values)
        # check if 5 lines in array
        print(data)
        if len(data) == 15:
            # use analysis on this Window
            data.to_json()
            statisticValues = np.array(data)
            #std
            #zerocross
            #
        else:
            return False
# save in form of ("std","zerocross","enz.")

# role window 1 further

        # get the current time in milliseconds
        current_ts_ms = int(round(time.time() * 1000))
        # Update values of data and label properties (send them to the DCD Hub)
        # With the same timestamp, so we can easily connect label and raw data later
        prop_label.update_values([class_index], current_ts_ms)
        prop_data.update_values(values, current_ts_ms)

        if len(data) >= 15:
            data.pop(0)

        return True
    return False


# How many samples do we want for each class
MAX_SAMPLES = 500
# How much time (in seconds) to leave between the collection of each class
DELAY_BETWEEN_POSTURE = 20


# Collect data for a given posture
# posture_index: index of the class in the array CLASSES
def collect(class_index):
    # Prompt the user to get ready and wait
    print("Get ready to collect the posture: " + CLASSES[class_index]
          + " in " + str(DELAY_BETWEEN_POSTURE) + " seconds!")
    time.sleep(DELAY_BETWEEN_POSTURE)

    # Open the serial connection
    print("Collecting data for posture " + CLASSES[class_index])
    #ser = open_serial()

    # Start reading serial port with the posture index, start at sample 0.
    sample = 0
    while sample < MAX_SAMPLES:
        if serial_to_property_values(class_index): #,ser)
            sample += 1
            print("Remaining samples: " + str(MAX_SAMPLES - sample))
    #ser.close()


class_index = 0
while class_index < len(CLASSES):
    collect(class_index)
    class_index = class_index + 1

print("Data collection done.")
