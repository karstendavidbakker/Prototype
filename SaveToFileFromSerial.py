import datetime
import time
from random import random
import serial

file = open("my-property.csv", "a") #we append

# Start reading the serial port
try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
except:
    try:
        ser = serial.Serial('/dev/ttyACM1', 115200, timeout=2)
    except:
        ser = serial.Serial('COM8', 115200, timeout=2)

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

def write_in_csv(values2):
    file.write(','.join(values2))
    file.write('\n')

#added:

def write_to_json(values2):
    fileJSON = open("web/static/data.json", "w") # we write
    fileJSON.write('[{"title":"x","subtitle":"TARGET motion in x direction","ranges":[20,25,30],"measures":[' + values[1] + ',23],"markers":[26]},{"title":"y","subtitle":"ORIENTATION motion in y direction","ranges":[350,500,600],"measures":['+ values[2] +',320],"markers":[550]},{"title":"z","subtitle":"BUMB motion in z direction","ranges":[5000,20000,25000],"measures":['+values[3]+',1650],"markers":[2100]}]')
    fileJSON.close()

#write to data.json file, + xxxxxx +, means provide dynamic data, values [1]
# is the random created data in the code below after values. Maybe change str(random()) to certain string from serial port.

try:
    # finally, we call our function to start generating dum values
    while True:
        serial_to_property_values()
        current_time = int(datetime.datetime.utcnow().timestamp()*1000)
        print(current_time)
        values2 = (str(current_time), str(values))
        write_in_csv(values2)

        write_to_json(values2)
        #have a 2-second break
        time.sleep(2)

except Exception:
    file.close()
    fileJSON.close()

    # have a 2-second break
    time.sleep(2)

except Exception:
    file.close()
