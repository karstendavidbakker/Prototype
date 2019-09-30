from flask import Flask, request

app = Flask(__name__)

sensors = ['sensor1', 'sensor2', 'sensor3']

@app.route('/')
def hello_world():
    return 'Hello, World Vlammen Team!'

if __name__ == '__main__':
    app.run()

@app.route('/api/sensors', methods = ['GET'])
def list():
    return str(sensors)

@app.route('/api/sensors/<path:sensor_id>', methods = ['GET'])
def read(sensor_id):
    global sensors
    return sensors[sensor_id]

@app.route('/api/sensors', methods = ['POST'])
def create():
    sensors.append(request.json["sensorName"])
    return 'Added sensor!'

if __name__ == '__main__':
    app.run(host='0.0.0.0')

#This means that your server will accept incoming request from any source.
