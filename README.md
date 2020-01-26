# RogyGarden
Digital Garden with hail protection

See hackster.io link for more project infomation:

https://www.hackster.io/ndrogness/digital-garden-with-hail-protection-ed38af

#Python3 modules
gpiozero -> sudo pip3 install gpiozero

Adadfruit IO -> sudo pip3 install adafruit-io

bmp280 temp sensor module -> sudo pip3 install bmp280

adafruit INA260 current sensor -> sudo pip3 install adafruit-blinka adafruit-circuitpython-ina260

# Setup MQTT communication via Adafruit IO
You will need to go to the adafruit.io site to setup an account.  The adafruit.io site
is used to received the updates/commands from the garden.  Once you have an account and
feed setup, you'll need to enter them in the config file:

shell> cp rogygarden.cfg-example rogygarden.cfg

Edit the rogygarden.cfg file, and set your configuration for communication.  

Download your favorite MQTT mobile app.  I used 'IoT MQTT Panel' on Android.  Point the app
at the Adafruit feed and setup your MQTT app to display the JSON sent from the garden.

# GPIO Pin definition

    GPIO13 = Roof motor forward pin (motor controllers forwards)
    GPIO19 = Roof motor backward pin (motor controllers backwards)
    GPIO16 = roof detect switch A detect closed pin (Limit switch 1)
    GPIO17 = roof detect switch A detect open pin (Limit switch 2)
    GPIO27 = roof detect switch B detect open pin (Limit switch 3)
    GPIO22 = roof detect switch B detect close pin (Limit switch 4)
    GPIO20 = roof closed switch comm (3v wired to each of the limit switches)
    GPIO23 = Enable motor power pin (connected to relay to enable 12V to motor controller)
    GPIO24 = Roof Lock engage pin (not implemented)

# Run 
from shell> python3 rogygarden.py
