#!/usr/bin/env python3

import time
import datetime
import sys
import os
import gpiozero as ioz
from Adafruit_IO import MQTTClient
import json
from collections import namedtuple
import rogysensor


class RogyGardenRoof:
    def __init__(self, motor_enable_pin, motor_forward_pin, motor_backward_pin,
                 detect_com_pin=None, detect_a_open_pin=None, detect_a_closed_pin=None,
                 detect_b_open_pin=None, detect_b_closed_pin=None, debug=False):

        self.progress_update_json = ''
        self.debug = debug

        # Setup Motor
        self._roof_motor = ioz.Motor(motor_forward_pin, motor_backward_pin)
        self._roof_motor.stop()

        # Max time to wait for motor to run (divisible by 3)
        self.max_motor_run_duration = 24

        # Setup Motor Relay
        self._roof_motor_enable = ioz.DigitalOutputDevice(motor_enable_pin)
        self._roof_motor_enable.off()

        self.roof_closed = None

        detection = [{'a': None, 'b': None, 'stat': None}, {'a': None, 'b': None, 'stat': None}]

        if detect_com_pin is None:
            self._do_detection = False
            self._detection_successful = False
            self.roof_closed_detect_reason = 'detection disabled'
        else:
            self._do_detection = True
            self._open_switches_active = {}
            self._closed_switches_active = {}
            self._detection_successful = False
            self._max_detection_time = 3
            self.roof_closed_detect_reason = 'startup'
            self._roof_closed_detect_com = ioz.DigitalOutputDevice(detect_com_pin)
            # self._roof_closed_detectA_com = ioz.DigitalOutputDevice(detect_a_closed_pin)
            # self._roof_closed_detectB_com = ioz.DigitalOutputDevice(closed_detectB_com_pin)

        if self._do_detection is True and detect_a_open_pin is not None:
            # Set momentary limiter switch (Switch Open = Roof Open = value->False)
            self._roof_open_detectA = ioz.Button(detect_a_open_pin, pull_up=False, hold_time=self._max_detection_time-1)

            # Register callback func for closed switch
            # self._roof_open_detectA.when_held = self._roof_open_trigger
            self._roof_open_detectA.when_pressed = self._roof_open_trigger

            # Register switch
            self._open_switches_active[detect_a_open_pin] = {'name': 'A_OPEN', 'obj': self._roof_open_detectA}

        if self._do_detection is True and detect_a_closed_pin is not None:
            # Set momentary limiter switch (Switch Open = Roof Open = value->False)
            self._roof_closed_detectA = ioz.Button(detect_a_closed_pin, pull_up=False,
                                                   hold_time=self._max_detection_time-1)
            # Register callback func for closed switch
            # self._roof_closed_detectA.when_held = self._roof_closed_trigger
            self._roof_closed_detectA.when_pressed = self._roof_closed_trigger

            # Register switch
            self._closed_switches_active[detect_a_closed_pin] = {'name': 'A_CLOSED', 'obj': self._roof_closed_detectA}

        if self._do_detection is True and detect_b_open_pin is not None:
            # Set momentary limiter switch (Switch Open = Roof Open = value->False)
            self._roof_open_detectB = ioz.Button(detect_b_open_pin, pull_up=False, hold_time=self._max_detection_time-1)

            # Register callback func for closed switch
            # self._roof_open_detectB.when_held = self._roof_open_trigger
            self._roof_open_detectB.when_pressed = self._roof_open_trigger

            # Register switch
            self._open_switches_active[detect_b_open_pin] = {'name': 'B_OPEN', 'obj': self._roof_open_detectB}

        if self._do_detection is True and detect_b_closed_pin is not None:
            # Set momentary limiter switch (Switch Open = Roof Open = value->False)
            self._roof_closed_detectB = ioz.Button(detect_b_closed_pin, pull_up=False,
                                                   hold_time=self._max_detection_time-1)
            # Register callback func for closed switch
            # self._roof_closed_detectB.when_held = self._roof_closed_trigger
            self._roof_closed_detectB.when_pressed = self._roof_closed_trigger

            # Register switch
            self._closed_switches_active[detect_b_closed_pin] = {'name': 'B_CLOSED', 'obj': self._roof_closed_detectB}

    def _roof_open_trigger(self, trigger_dev):

        if self.debug is True:
            print("Received Open trigger:", trigger_dev.pin, self._open_switches_active[trigger_dev.pin.number]['name'])

        self._detection_successful = True
        self.roof_closed = False
        self.roof_closed_detect_reason = 'trigger: {0}->{1}'.format(trigger_dev.pin,
                                                                    self._open_switches_active[trigger_dev.pin.number]
                                                                    ['name'])

        # Stop Motor if both sides open
        if self._roof_motor.is_active:
            disable_motor = True

            for k in self._open_switches_active.keys():
                if self._open_switches_active[k]['obj'].is_held is False:
                    disable_motor = False

            if disable_motor is True:
                self._motor_ctl(direction='stop')

    def _roof_closed_trigger(self, trigger_dev):

        if self.debug is True:
            print("Received Closed trigger:", trigger_dev.pin,
                  self._closed_switches_active[trigger_dev.pin.number]['name'])

        self._detection_successful = True
        self.roof_closed = True
        self.roof_closed_detect_reason = 'trigger: {0}->{1}'.format(trigger_dev.pin,
                                                                    self._closed_switches_active[trigger_dev.pin.number]
                                                                    ['name'])

        # Stop Motor if both sides closed
        if self._roof_motor.is_active:
            disable_motor = True

            for k in self._closed_switches_active.keys():
                if self._closed_switches_active[k]['obj'].is_held is False:
                    disable_motor = False

            if disable_motor is True:
                self._motor_ctl(direction='stop')

    def _roof_state_detection(self, enable=True):

        if enable is True:
            self._roof_closed_detect_com.on()
        else:
            self._roof_closed_detect_com.off()
            self._detection_successful = False

    def is_roof_closed(self, poll_reason='status'):

        retval = self.roof_closed
        if self._do_detection is False:
            return retval

        # Check Closed Status
        self._roof_state_detection(enable=True)
        time.sleep(self._max_detection_time)
        if self._detection_successful is False:
            retval = None
        else:
            retval = self.roof_closed

        if self.debug is True:
            rlog = 'Roof detect switch status: Common={0}'.format(self._roof_closed_detect_com.is_active)

            rlog += ', Open Detect('
            for _swi in self._open_switches_active:
                _rswitch = self._open_switches_active[_swi]
                rlog += ',{0}={1}'.format(_rswitch['name'], _rswitch['obj'].is_pressed)
            rlog += '), Close Detect('
            for _swi in self._closed_switches_active:
                _rswitch = self._closed_switches_active[_swi]
                rlog += ',{0}={1}'.format(_rswitch['name'], _rswitch['obj'].is_pressed)
            rlog += ')'

            print(rlog)

        self._roof_state_detection(enable=False)

        self.roof_closed_detect_reason = poll_reason

        return retval

    def progress_update(self):

        garden.update_sensors()
        self.progress_update_json = '{'
        self.progress_update_json += '"sensors":{{{0}}}'.format(garden.sensors_status_json)
        self.progress_update_json += '}'

    def _motor_ctl(self, direction='open'):

        if direction == 'stop':
            if self.debug is True:
                print("Stopping motor")
            self._roof_motor.stop()
            self._roof_motor_enable.off()
            time.sleep(1)

        else:

            target_roof_closed_state = False
            self._roof_state_detection(enable=False)
            self._roof_motor_enable.on()
            if self._roof_motor.is_active is True:
                self._roof_motor.stop()
                time.sleep(1)

            self._roof_state_detection(enable=True)
            if direction == 'open':
                self._roof_motor.backward(1)
                target_roof_closed_state = False
                if self.debug is True:
                    print("motor moving backward -> roof opening")
                garden.log('roof is opening')

            elif direction == 'close':
                self._roof_motor.forward(1)
                target_roof_closed_state = True
                if self.debug is True:
                    print("motor moving forward -> roof closing")
                garden.log('roof is closing')

            # ## Broken - This was to see power impact of motors, but have async issues
            # _update_interval = 5
            # for msd in range(0, int(round(self.max_motor_run_duration/_update_interval))):
            #     self.progress_update()
            #     print('Sending Progress:', self.progress_update_json)
            #     mqt.send_message(self.progress_update_json, max_time=2)
            #     time.sleep(_update_interval)

            time.sleep(self.max_motor_run_duration)
            self._roof_state_detection(enable=False)
            self._roof_motor.stop()
            self._roof_motor_enable.off()
            self.roof_closed = self.is_roof_closed(poll_reason='timeout')
            if self.roof_closed is None:
                self.roof_closed = target_roof_closed_state

        # Ramping speed for variable rate motor control - Not used
        # for mspeed in range(int(100*self.ramp_start), 100, int(100*self.ramp_step)):
        #     if direction == 'open':
        #         self._roof_motor.forward((100-mspeed)/100)
        #         print("motor forward:", (100-mspeed)/100)
        #     elif direction == 'close':
        #         self._roof_motor.backward(mspeed/100)
        #         print("motor backward:", mspeed/100)

    def do_close(self):
        if self.roof_closed is True:
            garden.log('Roof is already closed!')
        else:
            self._motor_ctl('close')
            if self.roof_closed is not True:
                garden.log('Tried to close roof, but didnt detect it closed!')
            self.roof_closed = True

    def do_open(self):
        if self.roof_closed is False:
            garden.log('Roof is already open!')
        else:
            self._motor_ctl('open')
            if self.roof_closed is not False:
                garden.log('Tried to open roof, but didnt detect it opened!')
            self.roof_closed = False

    def do_toggleit(self):
        if self.roof_closed is True:
            self.do_open()
        elif self.roof_closed is False:
            self.do_close()
        else:
            garden.log('Cant toggle roof due to unknown state!')

    def __str__(self):
        rstr = 'Closed={0}, CDetect={1}, Motor_enabled={2}'.format(self.roof_closed, self.roof_closed_detect_reason,
                                                                   self._roof_motor_enable.is_active)

        return str(rstr)

    def read(self):
        self.roof_closed = self.is_roof_closed()
        time.sleep(1)
        if self.roof_closed is True:
            return 'roof_closed', 'yes'
        elif self.roof_closed is False:
            return 'roof_closed', 'no'
        else:
            return 'roof_closed', 'unknown'

    def free(self):
        self._roof_motor.stop()
        self._roof_motor.close()
        self._roof_motor_enable.off()
        self._roof_motor_enable.close()
        self._roof_state_detection(enable=False)
        '''
        for _switch in self._closed_switches_active:
            _switch['obj'].close()
        for _switch in self._open_switches_active:
            _switch['obj'].close()
        '''

class RogyGarden:

    sensor_types = ['temp', 'bar_pres', 'altitude', 'hail_detect', 'rain_detect',
                    'soil_moisture', 'voltage', 'current', 'power']

    def __init__(self, config):

        self.event_log = []
        self.config = config
        self.health = {
            'active': True,
            'needs_attention': False,
            'is_connected': "no",
            'roof_closed': 'unknown',
            'last_update': 'Never',
            'current_status': 'OK',
        }

        self.controllers = {
            'roof': {'active': False, 'val': 0, 'status': 'Offline'},
            'roof_lock': {'active': False, 'val': 0, 'status': 'Offline'},
        }

        self.sensors = {
            'temp': {'active': False, 'val': 0, 'status': 'Offline'},
            'bar_pres': {'active': False, 'val': 0, 'status': 'Offline'},
            'altitude': {'active': False, 'val': 0, 'status': 'Offline'},
            # 'rain_detect': {'active': False, 'val': False, 'status': 'Offline'},
            # 'hail_detect': {'active': False, 'val': False, 'status': 'Offline'},
            # 'soil_moisture': {'active': False, 'val': 0, 'status': 'Offline'},
            'roof_open': {'active': False, 'val': True, 'status': 'Offline'},
            'voltage': {'active': False, 'val': 0, 'status': 'Offline'},
            'current': {'active': False, 'val': 0, 'status': 'Offline'},
            'power': {'active': False, 'val': 0, 'status': 'Offline'},

        }
        self.status_json = ''
        self.sensors_status_json = ''
        self.controllers_status_json = ''

    def log(self, msg):
        self.event_log.append('"{0}"'.format(msg))
        # print(msg)

    def flush_log(self):
        self.event_log.clear()

    def register_controller(self, controller_type, controller_obj):

        if controller_type not in self.controllers:
            print('Unknown Controller Type', controller_type)
            return

        self.controllers[controller_type]['obj'] = controller_obj
        self.controllers[controller_type]['active'] = True
        self.controllers[controller_type]['status'] = controller_obj

        # Hack job - ugh
        # if controller_type == 'roof':
        #     self.health['is_protected'] = controller_obj.roof_closed

    def update_controllers(self):

        self.controllers_status_json = ''
        self.health['needs_attention'] = False

        for c_key in self.controllers.keys():
            if self.controllers[c_key]['active'] is True:
                hkey, hval = self.controllers[c_key]['obj'].read()
                self.controllers[c_key]['val'] = '{0}={1}'.format(hkey, hval)
                self.health[hkey] = hval
                if hval == 'unknown':
                    self.health['needs_attention'] = True
                    self.log('{0} Alarm: {1}'.format(c_key, self.controllers[c_key]['val']))
                    if self.config['DEBUG']['val'] is True:
                        print('Changing health status to needs attention:', self.controllers[c_key]['val'])

            if self.controllers_status_json == '':
                self.controllers_status_json += '"{0}":{{'.format(c_key)
            else:
                self.controllers_status_json += ',"{0}":{{'.format(c_key)

            controller_item_json = ''
            for c_item_key in self.controllers[c_key].keys():
                if c_item_key == 'obj':
                    continue

                if controller_item_json == '':
                    controller_item_json += '"{0}":"{1}"'.format(c_item_key, self.controllers[c_key][c_item_key])
                else:
                    controller_item_json += ',"{0}":"{1}"'.format(c_item_key, self.controllers[c_key][c_item_key])

            self.controllers_status_json += '{0}}}'.format(controller_item_json)

    def register_sensor(self, sensor_type, sensor_obj):

        if sensor_type not in self.sensors:
            print('Unknown Sensor Type', sensor_type)
            return

        self.sensors[sensor_type]['obj'] = sensor_obj
        self.sensors[sensor_type]['active'] = True
        self.sensors[sensor_type]['status'] = 'OK'

    def update_sensors(self):

        self.sensors_status_json = ''

        for s_key in self.sensors.keys():
            if self.sensors[s_key]['active'] is True:
                self.sensors[s_key]['val'], self.sensors[s_key]['status'] = self.sensors[s_key]['obj'].read_data_val(s_key)

            if self.sensors_status_json == '':
                self.sensors_status_json += '"{0}":{{'.format(s_key)
            else:
                self.sensors_status_json += ',"{0}":{{'.format(s_key)

            sensor_item_json = ''
            for s_item_key in self.sensors[s_key].keys():
                if s_item_key == 'obj':
                    continue

                if sensor_item_json == '':
                    sensor_item_json += '"{0}":"{1}"'.format(s_item_key, self.sensors[s_key][s_item_key])
                else:
                    sensor_item_json += ',"{0}":"{1}"'.format(s_item_key, self.sensors[s_key][s_item_key])

            self.sensors_status_json += '{0}}}'.format(sensor_item_json)

    def status_update(self):

        self.update_controllers()
        self.update_sensors()

        self.status_json = '{'
        health_json = ''
        for h_key in self.health.keys():

            if health_json == '':
                health_json += '"{0}":"{1}"'.format(h_key, self.health[h_key])
            else:
                health_json += ',"{0}":"{1}"'.format(h_key, self.health[h_key])

        self.status_json += '{0},'.format(health_json)

        self.status_json += '"sensors":{{{0}}},'.format(self.sensors_status_json)
        self.status_json += '"controllers":{{{0}}}'.format(self.controllers_status_json)

        self.status_json += ',"event_log":['
        if len(self.event_log) > 0:
            self.status_json += ','.join(self.event_log)
        self.status_json += ']'

        self.status_json += '}'
        # print(self.status_json)
        self.health['last_update'] = time.asctime()

    def process_cmd(self, json_cmd):

        cmd_success = False

        try:
            jobj = json.loads(json_cmd)
        except:
            print("Invalid JSON:", json_cmd)
            return

        if 'cmd' in jobj:

            for c_key, c_val in jobj['cmd'].items():

                if c_key == 'protect':
                    if c_val == 'yes':
                        self.log('Received protect cmd')
                        self.protect()
                        cmd_success = True
                    elif c_val == 'no':
                        self.log('Received unprotect cmd')
                        self.unprotect()
                        cmd_success = True
                    else:
                        print("Invalid protect state:", c_val)
                else:
                    print("Invalid json cmd:", c_key)

        if cmd_success is True:
            self.status_update()
            mqt.send_message(self.status_json, max_time=2)

    def protect(self):
        if self.controllers['roof']['active'] is True:
            self.controllers['roof']['obj'].do_close()
            self.status_update()

    def unprotect(self):
        if self.controllers['roof']['active'] is True:
           self.controllers['roof']['obj'].do_open()
           self.status_update()

    def __str__(self):
        rstr_h = ['{0}={1}'.format(i, self.health[i]) for i in self.health.keys()]
        rstr_s = ['{0}={1}'.format(k, self.sensors[k]['val']) for k in self.sensors.keys() if self.sensors[k]['active']]
        rstr_l = ['{0}'.format(self.event_log[i]) for i in range(0, len(self.event_log))]
        rstr = 'Health -> {0}, Sensors -> {1}, Event Log -> {2}'.format(rstr_h, rstr_s, rstr_l)

        return str(rstr)

    def free(self, reason='RogyGarden exception, shutting down'):
        _dying_gasp = '{{"current_status": "OFFLINE", "needs_attention": "True", "active": "False", ' \
                      '"is_connected": "no", "event_log":["{0}"]}}'.format(reason)

        # Update app that we are shutting down
        if mqt.is_connected is True:
            print('Sending Dying Gasp:', _dying_gasp)
            mqt.send_message(_dying_gasp, max_time=2, msg_debug=True)

        for g_key in self.sensors.keys():
            if self.sensors[g_key]['active'] is True:
                self.sensors[g_key]['obj'].free()

        for g_key in self.controllers.keys():
            if self.controllers[g_key]['active'] is True:
                self.controllers[g_key]['obj'].free()


class AdafruitIOMQT:
    '''
    Class wrapper for adafruitIO communication
    '''

    def __init__(self, username, key, feed, debug=False):

        self.username = username
        self.key = key
        self.mqt_feed = feed
        self.debug = debug

        if self.debug is True:
            print('AdafruitIO config username:{0}, key:{1}, feed: {2}'.format(self.username, self.key, self.mqt_feed))

        if self.username == 'username_undefined':
            print('Warning username not found in config, check your config file')

        # Create an MQTT client instance.
        self._mqtclient = MQTTClient(self.username, self.key)
        self.is_connected = False
        self.current_feed = self.mqt_feed
        self.last_sent_msg_ts = None

        # Setup the callback functions defined above.
        self._mqtclient.on_connect = self.on_connected
        self._mqtclient.on_disconnect = self.on_disconnected
        self._mqtclient.on_message = self.on_messaged

        # Connect to the Adafruit IO server.
        while self.is_connected is False:
            try:
                self._mqtclient.connect()
                self.is_connected = True
            except:
                print("Couldn't connect to AdafruitIO, trying again in 5...")
                self.is_connected = False
                time.sleep(5)

        # Run in background
        self._mqtclient.loop_background()

    def on_connected(self, callback_client):
        callback_client.subscribe(self.current_feed)
        print('AdafruitIO -> Got connected to feed:', self.current_feed)
        self.is_connected = True
        #self._mqtclient.subscribe(self.current_feed)
        garden.health['is_connected'] = 'yes'

    def on_disconnected(self, callback_client):
        self.is_connected = False
        garden.health['is_connected'] = 'no'

    def on_messaged(self, callback_client, feed_id, payload):
        # Message function will be called when a subscribed feed has a new value.
        # The feed_id parameter identifies the feed, and the payload parameter has
        # the new value.
        if self.debug is True:
            print('AdafruitIO -> Feed {0} received new value: {1}'.format(feed_id, payload))
        if 'cmd' in payload:
            garden.process_cmd(payload)

    def send_message(self, msg, max_time=10, msg_debug=False):

        if self.is_connected is False:
            print('AdafruitIO -> Cant publish message, mqt not connected!')
            return False

        if max_time < 2:
            max_time = 2

        if self.last_sent_msg_ts is not None:
            elapsed_time = time.mktime(time.localtime()) - self.last_sent_msg_ts
            if elapsed_time < max_time:
                print('AdafruitIO -> Cant send mqt more than once every {0} seconds, last sent: {1}'.format(
                      max_time,
                      time.asctime(time.localtime(self.last_sent_msg_ts))))
                return False

        # print("sending to mqt:", msg)
        if msg_debug is True or self.debug is True:
            print('Sending to mqt {0} -> {1}'.format(self.current_feed, msg))
        self._mqtclient.publish(self.current_feed, msg)
        self.last_sent_msg_ts = time.mktime(time.localtime())
        garden.flush_log()
        return True

    def free(self):
        if self._mqtclient.is_connected():
            self._mqtclient.disconnect()


def read_config(config_file, debug=False):
    '''
    Read Configuration File
    :param config_file: filename of config file
    :param debug: print debugging
    :return: config dictionary
    '''

    # Defaults
    config_item = namedtuple('ConfigItem', ['type', 'val', 'cast'])
    config_data = {
        'DEBUG': {'val': False, 'cast': 'boolean'},
        'ROOF_MOTOR_ENABLE': {'val': 18, 'cast': 'int'},
        'ROOF_MOTOR_FORWARD': {'val': 4, 'cast': 'int'},
        'ROOF_MOTOR_REVERSE': {'val': 17, 'cast': 'int'},
        'ROOF_DETECT_COMMON': {'val': 21, 'cast': 'int'},
        'ROOF_DETECT_A_CLOSED': {'val': 20, 'cast': 'int'},
        'ROOF_DETECT_A_OPEN': {'val': 16, 'cast': 'int'},
        'ROOF_DETECT_B_CLOSED': {'val': 24, 'cast': 'int'},
        'ROOF_DETECT_B_OPEN': {'val': 25, 'cast': 'int'},
        'ADAFRUIT_IO_USERNAME': {'val': 'username_undefined', 'cast': 'text'},
        'ADAFRUIT_IO_KEY': {'val': 'key_undefined', 'cast': 'text'},
        'ADAFRUIT_IO_FEED': {'val': 'RogyGarden', 'cast': 'text'},
    }

    num_tokens = 0

    if not os.path.isfile(config_file):
        print('WARNING: Missing config file:', config_file, ', using default config values')
        return config_data

    with open(config_file, mode='r') as f:
        config_lines = f.read().splitlines()

    for i in range(0, len(config_lines)):

        # Pickup Debug from config
        if debug is False and config_data['DEBUG']['val'] is True:
            print('Enabling Debugging from config file')
            debug = True

        if debug is True:
            print('Processing config file line {0}: {1}'.format(i, config_lines[i]))

        cline = config_lines[i].split("=")

        if cline[0] in config_data:
            if debug is True:
                print('Found Config Token', cline[0])
            if cline[1] == '':
                if debug is True:
                    print('{0} -> set to null, using default {1}'.format(cline[0], config_data[cline[0]]['val']))
            elif config_data[cline[0]]['cast'] == 'int':
                config_data[cline[0]]['val'] = int(cline[1])

            elif config_data[cline[0]]['cast'] == 'float':
                config_data[cline[0]]['val'] = float(cline[1])

            elif config_data[cline[0]]['cast'] == 'boolean':
                if cline[1] == 'TRUE' or cline[1] == 'True' or cline[1] == 'true' or cline[1] == 'ON':
                    config_data[cline[0]]['val'] = True
                elif cline[1] == 'FALSE' or cline[1] == 'False' or cline[1] == 'false' or cline[1] == 'OFF':
                    config_data[cline[0]]['val'] = False
                else:
                    print('{0} -> boolean type not TRUE/FALSE using default {1}'.format(cline[0],
                                                                                        config_data[cline[0]]['val']))

            elif config_data[cline[0]]['cast'] == 'hour':
                config_data[cline[0]]['val'] = datetime.time(hour=int(cline[1]))

            else:
                config_data[cline[0]]['val'] = cline[1]

            num_tokens += 1

    if debug is True:
        print('Final config data: ', config_data)

    return config_data


def run():
    '''
    Main run
    :return: null
    '''

    # vbat = machine.ADC(36)
    # vbat.atten(vbat.ATTN_11DB)
    # VBAT = Pin 35

    # Pin definition
    #
    # Testing rain water detector sensor - not implemented
    # gs = RogySensorAnalog(36)
    # garden.register_sensor('rain_detect', RogySensorAnalog(36))
    roof = RogyGardenRoof(motor_enable_pin=garden.config['ROOF_MOTOR_ENABLE']['val'],
                          motor_forward_pin=garden.config['ROOF_MOTOR_FORWARD']['val'],
                          motor_backward_pin=garden.config['ROOF_MOTOR_REVERSE']['val'],
                          detect_com_pin=garden.config['ROOF_DETECT_COMMON']['val'],
                          detect_a_closed_pin=garden.config['ROOF_DETECT_A_CLOSED']['val'],
                          detect_a_open_pin=garden.config['ROOF_DETECT_A_OPEN']['val'],
                          detect_b_open_pin=garden.config['ROOF_DETECT_B_OPEN']['val'],
                          detect_b_closed_pin=garden.config['ROOF_DETECT_B_CLOSED']['val'],
                          debug=cfg['DEBUG']['val']
                          )

    garden.register_controller('roof', roof)

    # Register temp sensor
    bmp280 = rogysensor.RogyBMP280(samples_per_read=5)
    if bmp280.active is True:
        garden.register_sensor('temp', bmp280)
        garden.register_sensor('bar_pres', bmp280)
        garden.register_sensor('altitude', bmp280)
    else:
        print('WARNING: BMP280 sensor offline, not measuring temperature')

    # Register battery system power usage
    ina260 = rogysensor.RogyINA260(samples_per_read=5)
    if ina260.active is True:
        garden.register_sensor('voltage', ina260)
        garden.register_sensor('current', ina260)
        garden.register_sensor('power', ina260)
    else:
        print('WARNING: INA260 sensor offline, not measuring voltage')

    counter = 0
    while True:
        garden.status_update()
        print(garden)
        # print("sending to mqt:", garden.status_json)
        mqt.send_message(garden.status_json)
        # Debugging/testing stuff
        # if counter > 1 and counter % 2 == 0:
        #    time.sleep(10)
        #     garden.protect()
        # if counter > 1 and counter % 11 == 0:
        #     garden.unprotect()
        time.sleep(30)
        counter += 1


if __name__ == '__main__':

    # Read config
    cfg = read_config(config_file='rogygarden.cfg')

    # Create a garden class to work with
    garden = RogyGarden(cfg)

    # Connect to AdafruitIO
    mqt = AdafruitIOMQT(username=cfg['ADAFRUIT_IO_USERNAME']['val'], key=cfg['ADAFRUIT_IO_KEY']['val'],
                        feed=cfg['ADAFRUIT_IO_FEED']['val'], debug=cfg['DEBUG']['val'])

    shutdown_reason = ''

    try:
        run()

    except KeyboardInterrupt:
        shutdown_reason = 'RogyGarden manually shutdown'

    except:
        shutdown_reason = 'RogyGarden exception: {0}'.format(sys.exc_info()[0])
        garden.log(shutdown_reason)

    finally:
        print('Cleaning up and exiting...')
        garden.free(reason=shutdown_reason)
        mqt.free()


