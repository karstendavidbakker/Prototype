from flask import Flask, request, render_template

app = Flask(__name__)

sensors = ['sensor1', 'sensor2', 'sensor3']

@app.route('/')
def hello_world():
    return 'Hello, World Vlammen!'

@app.route('/home')
def home():
    return render_template('index.html')

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
    #to give everyone from every pc access to web
