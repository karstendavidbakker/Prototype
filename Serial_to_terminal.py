import serial

from dotenv import load_dotenv
import os

load_dotenv()

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

        # Otherwise, we show a warning
        else:
            print('Warning: unknown property ' + property_name)


while True:
    serial_to_property_values()
    time.sleep(2)
