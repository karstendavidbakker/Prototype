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
        ser = serial.Serial('COM12', 115200, timeout=2)

def serial_to_property_values():
    line_bytes = None
    while line_bytes is None or len(line_bytes) <= 4:
    # Read one line
        line_bytes = ser.readline()
    # If the line is not empty
    if len(line_bytes) > 4:
    # Convert the bytes into string
        line = line_bytes.decode('utf-8')
        # Split the string using commas as separatoqr, we get a list of strings
        values = line.replace(":", ",").split(',')

#        print(line_bytes)
#        print(line)
    return values


def write_in_csv(values):
    file.write(','.join(values))
    file.write('\n')

#added:

def write_to_json(values):
    fileJSON = open("C:/Users/Caspe/Prototype/web/static/data.json", "w") # we write
    fileJSON.write('[{"title":"x","subtitle":"TARGET motion in x direction","ranges":[20,25,30],"measures":[' + values3[1] + ',23],"markers":[26]},{"title":"y","subtitle":"ORIENTATION motion in y direction","ranges":[350,500,600],"measures":['+ values3[2] +',320],"markers":[550]},{"title":"z","subtitle":"BUMB motion in z direction","ranges":[5000,20000,25000],"measures":['+values3[3]+',1650],"markers":[2100]}]')
    fileJSON.close()

#write to data.json file, + xxxxxx +, means provide dynamic data, values [1]
# is the random created data in the code below after values. Maybe change str(random()) to certain string from serial port.

try:
    # finally, we call our function to start generating dum values
    while True:
        values = serial_to_property_values()
        print(values)
        sensorName = values[0]
        valx = values[1]
        valy = values[2]
        valz = values[3]
        current_time = int(datetime.datetime.utcnow().timestamp()*1000)
        print(current_time)

        values2 = (str(current_time), str(sensorName), str(valx), str(valy), str(valz))
        values3 = (str(current_time), str(valx), str(valy), str(valz))
        write_in_csv(values2)
        write_to_json(values3)
        #have a 2-second break
        time.sleep(2)
except FileNotFoundError:
    print("cant save file")
    file.close()
except Exception:
    print("exception")
    file.close()
        #fileJSON.close();

        # have a 2-second break
    time.sleep(2)

#except Exception:
    #file.close()
