import datetime
import time
from random import random

file = open("my-property.csv", "a")

def write_in_csv(values):
    file.write(','.join(values))
    file.write('\n')

try:
    #finally, we call our function to start generating dum values
    while True:
        current_time = int(datetime.datetime.utcnow().timestamp()*1000)
        print(current_time)
        values = (str(current_time), str(random()), str(random()), str(random()))
        write_in_csv(values)
        #have a 2-second break
        time.sleep(2)

except Exception:
        file.close()
