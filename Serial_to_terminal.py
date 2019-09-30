import time
import serial

from dotenv import load_dotenv
import os

load_dotenv()

s = [0]
# Start reading the serial port
try:
  ser = serial.Serial('/dev/ttyACM0',9600, timeout=2)
except :
  ser = serial.Serial('/dev/ttyACM1',9600, timeout=2)

# Read the next line from the serial port
# and update the property values
def serial_to_property_values():
    # Read one line
    #line_bytes = ser.readline()

    # If the line is not empty
    #if len(line_bytes) > 0:
s[0] = str(int (ser.readline(),16))
print s[0]
        # Convert the bytes into string
        #line = line_bytes.decode('utf-8')
        # Split the string using commas as separatoqr, we get a list of strings
    #    values = line.split(',')
#        print(line_bytes)
        #print(line)
#        print(values)


while True:
    serial_to_property_values()
