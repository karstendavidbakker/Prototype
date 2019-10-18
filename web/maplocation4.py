
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



    ser = Serial(port, 115200, timeout = 1)

    #
    #
    #
    #


    try:
       socketio.emit('marker', '{"lat": "lng"}' % str(position), broadcast=True) #emitting it to the websocket, broadcast is that everyone gets the same information
    except:
       print("No socket?") #tells there is a problem
    return distVal #...


if __name__ == '__main__':
    socketio.run(app, host = '0.0.0.0')
#
# if __name__ == '__main__':
#     # app.run(host='0.0.0.0')
