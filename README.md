# RogyGarden
Digital Garden with hail protection

See hackster.io link for more project infomation:

https://www.hackster.io/ndrogness/digital-garden-with-hail-protection-ed38af

#Python3 modules
gpiozero -> sudo pip3 install gpiozero

Adadfruit IO -> sudo pip3 install adafruit-io

bmp280 temp sensor module -> sudo pip3 install bmp280

adafruit INA260 current sensor -> sudo pip3 install adafruit-blinka adafruit-circuitpython-ina260

# Edit Configuration file

shell> cp rogygarden.cfg-example rogygarden.cfg

Edit the config file and set your GPIO pins to for the various sensors.  Defaults are in the config file.

You will need to go to the adafruit.io site to setup an account so MQTT communication will work.  The adafruit.io site
is used to received the updates/commands from the garden.  Once you have an account and feed setup, 
you'll need to enter them in the config file:

ADAFRUIT_IO_USERNAME=your_adafruit_username

ADAFRUIT_IO_KEY=dc5866e512024830d8alwr70368c3bb1

ADAFRUIT_IO_FEED=RogyGarden

Download your favorite MQTT mobile app.  I used 'IoT MQTT Panel' on Android.  Point the app
at the Adafruit feed and setup your MQTT app to display the JSON sent from the garden.

You can enable debugging in the config file by setting DEBUG=TRUE, which will dump out extra information if you
are having problems.

# Run 
from shell> python3 rogygarden.py
