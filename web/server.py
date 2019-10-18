# import flask
from flask import Flask, request, render_template
from flask_socketio import SocketIO, emit, send

import datetime
import time
from random import random

file = open("my-property.csv", "a") #we append to barchart
current_time = int(datetime.datetime.utcnow().timestamp()*1000)
values = (str(current_time), str(random()), str(random()), str(random()))

# name site(created by flask) app
app = Flask(__name__)

app.config['SECRET KEY'] = 'secret!'
socketio = SocketIO(app)
# define amount of sensors
sensors = ['sensor1', 'sensor2', 'sensor3']
# functions for the root folder


@app.route('/') #testcomment
# display welcome message
def hello_world():
    return 'Hello, World Vlammen!'


@app.route('/home')
def home():
    return render_template('index.html')


@app.route('/api/sensors', methods=['GET'])
def list():
    return str(sensors)


@app.route('/api/sensors/<path:sensor_id>', methods=['GET'])
def read(sensor_id):
    global sensors
    return sensors[sensor_id]


@app.route('/api/sensors', methods = ['POST'])


@app.route('/test')
def test():
    return render_template('testvisual.html')


@app.route('/go')
def go():
    return render_template('go.html')


@app.route('/api/sensors', methods=['POST'])

def create():
    sensors.append(request.json["sensorName"])
    return 'Added sensor!'


@app.route('/gauge')
def gauge():
    return render_template('gauge.html')

@app.route('/map')
def map():
    return render_template('maplocation4.html')


#added to test
@app.route('/barchart')
def barchart():
    xvalue = value1
    return render_template('barchart2.html', xvalue = xvalue)

@socketio.on('x-axis')
def handle_x(json):
    print(float(json['x-axis']))


#xvalue = xvalue to link the variable to the web

    #return render_template('barchart.html') Name of variable)

def write_in_csv(values):
    file.write(','.join(values))
    file.write('\n')

def write_to_json(values):
    fileJSON = open("static/data.json", "w") # we write
    fileJSON.write('[{"title":"x","subtitle":"TARGET motion in x direction","ranges":[20,25,30],"measures":[' + values[1] + ',23],"markers":[26]},{"title":"y","subtitle":"ORIENTATION motion in y direction","ranges":[350,500,600],"measures":['+ values[2] +',320],"markers":[550]},{"title":"z","subtitle":"BUMB motion in z direction","ranges":[5000,20000,25000],"measures":['+values[3]+',1650],"markers":[2100]}]')
    fileJSON.close();
    #print(values[1])
    #print(values[2])
    #print(values[3])
    global value1
    value1 = (float(values[1]))
    global value2
    value2 = (float(values[2]))
    global value3
    value3 = (float(values[3]))

    print(value1)
    print(value2)
    print(value3)

    try:
        socketio.emit('x-axis','{"axisX": "%s"}' % str(value1), broadcast=True)
    except:
        print("No thing")

        #turning it into a json object to any client that is accesing your page.
# def randomValues():
#     try:
#         # finally, we call our function to start generating dum values
#         while True:
#             current_time = int(datetime.datetime.utcnow().timestamp()*1000)
#             print(current_time)
#             values = (str(current_time), str(random()), str(random()), str(random()))
#             write_in_csv(values)
#
#             write_to_json(values)
#             #have a 2-second break
#             # time.sleep(2)
#
#
#     except Exception:
#             file.close()
#             fileJSON.close();
#
#             # have a 2-second break
#             time.sleep(2)
#
#     except Exception:
#         file.close()
write_in_csv(values)
write_to_json(values)

if __name__ == '__main__':
    socketio.run(app, host = '0.0.0.0')
    # to give everyone from every pc access to web

###############################################################################################





#added:



#write to data.json file, + xxxxxx +, means provide dynamic data, values [1]
# is the random created data in the code below after values. Maybe change str(random()) to certain string from serial port.
