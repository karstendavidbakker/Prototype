from random import random
import time

import serial

from dcd.entities.thing import Thing
from dcd.entities.property import PropertyType
from dotenv import load_dotenv
import os

load_dotenv()

THING_ID = os.environ['THING_ID']
THING_TOKEN = os.environ['THING_TOKEN']

# define my thing using
my_thing = Thing(thing_id=THING_ID, token=THING_TOKEN)

# Start reading the serial port
ser = serial.Serial(
    port = os.environ['SERIAL'],
    baudrate = 9600,
    timeout = 2)

# Read the next line from the serial port
# and update the property values
def serial_to_property_values():
    # Read one line
    line_bytes = ser.readline()
    # If the line is not empty
    if len(line_bytes) > 0:
        # Convert the bytes into string
        line = line_bytes.decode('utf-8')
        # Split the string using commas as separator, we get a list of strings
        values = line.split(',')
        print(values)
        # Use the first element of the list as property id
        property_name = values.pop(0)
        # Get the property from the thing
        prop = my_thing.find_or_create_property(property_name, PropertyType.ONE_DIMENSIONS)
        # If we find the property, we update the values (rest of the list)
        if prop is not None:
            prop.update_values([float(x) for x in values])
        # Otherwise, we show a warning
        else:
            print('Warning: unknown property ' + property_name)



my_thing.read()

print(my_thing.to_json())
my_property = my_thing.find_or_create_property("My Serial Property", PropertyType.THREE_DIMENSIONS)

print(my_property.to_json())

while True:
    serial_to_property_values(my_property)
    time.sleep(2)
