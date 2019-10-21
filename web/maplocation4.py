
# Import required library
import pygatt  # To access BLE GATT support
import signal  # To catch the Ctrl+C and end the program properly
import os  # To access environment variables
from dotenv import \
    load_dotenv  # To load the environment variables from the .env file

from flask import Flask, request, render_template
from flask_socketio import SocketIO, emit, send



#print(distVal) EMITTING TO SOCKET CHANEL

#get information from serial
@socketio.on('gpslocation')
def handle_gpslocation(json):
    print( float(json['gpslocation']))




#ser = Serial(port, 115200, timeout = 1)

    #if it comes from the phone need to pull it from the dcd server
    #
    lat = [52.001737,52,51,56];
    lng = [4.372825,5,53,5];

    try:
       socketio.emit('marker', '{"lat": "%s", "lng": "%s"}' % str(lat[0]), broadcast=True) #emitting it to the websocket, broadcast is that everyone gets the same information
    except:
       print("No socket?") #tells there is a problem
    return distVal #...


if __name__ == '__main__':
    socketio.run(app, host = '0.0.0.0')
#
# if __name__ == '__main__':
#     # app.run(host='0.0.0.0')
