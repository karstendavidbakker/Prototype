import datetime
import time
from random import random

file = open("my-property.csv", "a") #we append

def write_in_csv(values):
    file.write(','.join(values))
    file.write('\n')

#added:

def write_to_json(values):
    fileJSON = open("web/static/data.json", "w") # we write
    fileJSON.write('[{"title":"x","subtitle":"TARGET motion in x direction","ranges":[20,25,30],"measures":[' + values[1] + ',23],"markers":[26]},{"title":"y","subtitle":"ORIENTATION motion in y direction","ranges":[350,500,600],"measures":['+ values[2] +',320],"markers":[550]},{"title":"z","subtitle":"BUMB motion in z direction","ranges":[5000,20000,25000],"measures":['+values[3]+',1650],"markers":[2100]}]')
    fileJSON.close();

#write to data.json file, + xxxxxx +, means provide dynamic data, values [1]
# is the random created data in the code below after values. Maybe change str(random()) to certain string from serial port.


try:
    #finally, we call our function to start generating dum values
    while True:
        current_time = int(datetime.datetime.utcnow().timestamp()*1000)
        print(current_time)
        values = (str(current_time), str(random()), str(random()), str(random()))
        write_in_csv(values)
        write_to_json(values)
        #have a 2-second break
        time.sleep(2)

except Exception:
        file.close()
        fileJSON.close();
