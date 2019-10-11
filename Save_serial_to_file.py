import time
import serial

from dotenv import load_dotenv
import os

load_dotenv()
file = open("web/static/my-property.csv", "a")
# Start reading the serial port
try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
except:
    try:
        ser = serial.Serial('/dev/ttyACM1', 115200, timeout=2)
    except:
        ser = serial.Serial('COM8', 115200, timeout=2)



# Read the next line from the serial port
# and update the property values


def serial_to_property_values():

    # Read one line
    line_bytes = ser.readline()
    # If the line is not empty
    if len(line_bytes) > 0:
        # Convert the bytes into string
        line = line_bytes.decode('utf-8')
        # Split the string using commas as separatoqr, we get a list of strings
        values = line.replace(":", ",").split(',')
#        print(line_bytes)
#        print(line)
        print(values)

def write_in_csv(values):
    file.write(','.join(values))
    file.write('\n')

try:
    # finally, we call our function to start saving serial values
    while True:
        serial_to_property_values()
        current_time = int(datetime.datetime.utcnow().timestamp()*1000)
        print(current_time)
        values2 = (str(current_time), str(values))
        write_in_csv(values2)
        # have a 2-second break
        time.sleep(2)

except Exception:
    file.close()
