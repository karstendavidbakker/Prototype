import serial

from dcd.entities.thing import Thing
from dcd.entities.property import PropertyType
from dotenv import load_dotenv
import os

# import flask
from flask import Flask, request, render_template
from flask_socketio import SocketIO, emit, send

import datetime
import time
from random import random

from numpy import genfromtxt
from threading import Thread


load_dotenv()

GPS_data = ["00.00000", "0.000000"]

THING_ID = os.environ['THING_ID']
THING_TOKEN = os.environ['THING_TOKEN']

# define my thing using
my_thing = Thing(thing_id=THING_ID, token=THING_TOKEN)

my_thing.read()

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
def serial_to_property_values():
    # Read one line
    line_bytes = ser.readline()
    # lezen van lijn/functie
    # If the line is not empty
    if len(line_bytes) > 0:

        # Convert the bytes into string
        line = line_bytes.decode('utf-8')
        print(line)
        property_name = line[0:3]
        print(line[0:3])
        # Split the string using commas as separator, we get a list of strings
        if property_name == "GPS":
            values = line.replace("N","").replace("E","").replace(".","")
            values2 = values[:6]+"."+values[6:15]+"."+values[15:]
            values3 = values2.replace(":", ",").split(',')
        else:
            values3 = line.replace(":", ",").split(',')
        print(values3)

        for x in values3:
            print(x.strip())
        # Use the first element of the list as property id
        property_name = values3.pop(0)
        print(property_name)
        # gps values get put into gps data variable
        if property_name == "GPS":
            GPS_values = [float(i) for i in values3]
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

def serial_gps_data():
    try:
        while True:
            print("emiting gps location")
            #try:
            print("begin serial_to_property_values")
            GPS_data = serial_to_property_values()
            #except:
            print("exception on serial_to_property_values")
            #GPS_data = [0,0]
            print(GPS_data)
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
