from random import random
import time
from dcd.entities.thing import Thing
from dcd.entities.property import PropertyType
from dotenv import load_dotenv
import os

def generate_dum_property_values(the_property):
    values = (random(), random(), random())
    the_property.update_values(values)

load_dotenv()
THING_ID = os.environ['THING_ID']
THING_TOKEN = os.environ['THING_TOKEN']

my_thing = Thing(thing_id=THING_ID, token=THING_TOKEN)

my_thing.read()

print(my_thing.to_json())
my_property = my_thing.find_or_create_property("My Random Property", PropertyType.THREE_DIMENSIONS)
print(my_property.to_json())

while True:
    generate_dum_property_values(my_property)
    time.sleep(2)
