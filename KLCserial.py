#!/bin/usr/python
# Python wrapper to communicate with Thorlabs KLC101 controllers
# %% ---------------------------------------------------------------------------
# imports

import serial
import time
import glob
import sys
import numpy as np

# only set this to True when you need a lot more output
FLAG_DEBUG = False

# %% ---------------------------------------------------------------------------
# COMMANDS
commands = {
    "identify":         "23 02 00 00 50 01", # flashes the screen of the specified device
    "disconnect":       "02 00 00 00 50 01", # inform device of a pending disconnect
    "req_info":         "05 00 00 00 50 01", # get hardware info (4+84 byte)
    "req_serial":       "15 00 00 00 50 01", # get serial number (6 + 40 byte)
    "set_voltage":      "01 20 06 00 d0 01", # set output voltage (+6 byte)
    "req_voltage":      "02 20 01 01 50 01", # get output voltage (4th part could be 01 or 02) (6+6 byte)
    "set_frequency":    "05 20 06 00 d0 01", # set frequency (+6 byte)
    "req_frequency":    "06 20 01 01 50 01", # get frequency (4th part could be 01 or 02) (6+6 byte)
    "enable_ADCinmode": "08 20 01 01 50 01", # enable analog input mode
    "disable_ADCinmode":"08 20 01 00 50 01", # disable analog input mode
    "req_ADCinmode":    "09 20 01 00 50 01", # get analog input mode (4th byte is en/dis)
    "req_ADCparams":    "0e 20 01 00 50 01", # get ADC parameters (6+4 bytes)
    "req_HWchan_status":"11 02 01 00 50 01", # get HW device channel status (3rd byte defines channel)
    "enable_HWchan":    "10 02 01 01 50 01", # enable HW channel (3rd byte defines channel)
    "disable_HWchan":   "10 02 01 02 50 01", # disable HW channel (3rd byte defines channel), x02 means "off"
    "req_chan_status":  "17 02 01 00 50 01", # get device channel status (3rd byte defines channel) (not working?)
    "enable_chan":      "16 20 01 01 50 01", # enable channel (3rd byte defines channel) (not working?)
    "disable_chan":     "16 20 01 00 50 01", # disable channel (3rd byte defines channel) (not working?)
    "req_kcube_params": "81 20 01 00 50 01", # get display parameters (brightness/timeout)
    "set_kcube_params": "80 20 0a 00 d0 01", # set display parameters (+10 byte)
#    "set_output_status":"28 20 0a 00 d0 01", # set device output status (+10 byte) (not working as documented)
#    "req_output_status":"29 20 01 00 d0 01", # get device output status (6+10 byte) (not working as documented)
    "save_params":      "86 20 01 00 50 01", # save operating parameters to eeprom
    "restorefactset":   "86 06 01 00 50 01", # restore device settings to factory defaults
}

# %% ---------------------------------------------------------------------------
# CONVERSION FUNCTIONS

def hexstr_to_int(value_hex, signed=False):
    """convert voltage/frequency value from hex to a useable integer"""
    # convert hex string to bytes, e.g. ' EE 76 01 00' to b'\xee\x76\x01\x00'
    value_bytes = bytes.fromhex(value_hex)
    # convert bytes to signed integer, eg 'e8 03' becomes 1000 [mV/Hz]
    value_int = int.from_bytes(value_bytes, byteorder='little', signed=signed) 
    if FLAG_DEBUG: print(value_bytes.hex(), value_int, )
    return value_int


def int_to_hexstr(value_num : float, signed=True):
    """ convert an integer voltage/frequency to hex strings for transmitting to KLC controller"""
    value_int = int(value_num)
    # convert to bytes (e.g. b'\xee\x76\x01\x00')
    value_bytes = value_int.to_bytes(4, byteorder='little', signed=signed)
    # convert to hex string with space at the beginning and in between
    value_hex = ''
    for n in range(2):
        value_hex = value_hex + ' ' + format(value_bytes[n], '02x')
    if FLAG_DEBUG: print(value_bytes.hex(), value_hex)
    return value_hex

# %% ---------------------------------------------------------------------------
# SERIAL FUNCTIONS

# create a serial connection with the recommended parameters
def openKLC(port='', LC_ID = ''):
    """create a serial connection with the recommended parameters
    if no port is given the function will try all available serial ports
    and check whether the connected device has an ID like an IO laser. 
    """
    s = serial.Serial()
    s.baudrate = 115200
    s.bytesize = serial.EIGHTBITS
    s.parity = serial.PARITY_NONE
    s.stopbits = serial.STOPBITS_ONE # number of stop bits
    s.timeout = 5
    s.rtscts = True # enable hardware (TRS/CTS) flow control
    #print(s)
    
    if not port:
        # find available ports depending on operating system
        if sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            available_ports = glob.glob('/dev/serial/by-id/*Kinesis_LC_Controller*' + LC_ID + '*')
        elif sys.platform.startswith('win'):
            available_ports = ['COM%s' % (i + 1) for i in range(256)]
        else:
            raise EnvironmentError('Unsupported platform')
    else:
        available_ports = [port]
    
    # try to open the ports until one works
    for port in available_ports:
        try:
            print('opening port', port)
            s.port = port
            s.open()
            time.sleep(0.1)
            break
        except:
            print('failed at port', port)
            pass
    if s.is_open:
        print('is open: ', s.is_open)
    else:
        print('could not find any serial port')
        s = ''
    return s


# close serial connection
def closeKLC(s):
    if not s.is_open: print('no serial connection'); return
    s.close()
    print('is open: ', s.is_open)


# send a command
def sendcommand(s, command : str):
    if not s.is_open: print('no serial connection'); return
    splitstring = command.split() # separate in to list of hex values
    command_ints = [int(str, 16) for str in splitstring] # convert to integer
    if FLAG_DEBUG: print('sending command: ', command_ints)
    s.write(bytes(command_ints)) # send integer in binary format to stage


# receive and parse reply
def recvreply(s):
    if not s.is_open: print('no serial connection'); return
    time.sleep(0.04) # has to be at least 20 ms to work on my computer
    # print('bytes in queue: ', s.in_waiting)
    reply = ''
    while s.in_waiting > 0:
        # read every single byte (converted to hex) and add whitespace
        reply += s.read().hex()
        reply += ' '
    # print('reply: ', reply)
    return reply


# convert reply to readable info and split into individual messages
def decode_reply(reply):
    # if no reply, return
    if not reply:
        message = ''
        print('no reply')
        return message
        
    mID = reply[0:5] # get the first two bytes as message ID
    header = reply[0:17] # get the first 6 bytes as header
    message_params = ''
    if FLAG_DEBUG: print(reply)
    
    if mID == '06 00':
        message = 'hardware info' + reply[17:269]
        length = 90
    elif mID== '81 00':
        message = 'rich response' + reply[17:]
        length = 64
    elif mID == '16 00':
        message = 'serial number'
        length = 40
    elif mID == '03 20':
        message = 'output voltage'
        length = 6
    elif mID == '07 20':
        message = 'output frequency'
        length = 6
    elif mID == '0a 20':
        message = 'analog input mode'
        length = 0
        analog_mode = reply[10]
        return analog_mode
    elif mID == '0f 20':
        message = 'ADC parameters'
        length = 6
    elif (mID == '18 02') or (mID == '12 02'):
        message = 'channel status'
        length = 0
        channel = reply[7]
        channel_status = reply[10]
        return channel_status
    elif mID == '30 20':
        message = 'output status'
        print('this does not work according to documentation, should spit out 10 byte but doesnt')
        length = 10
    elif mID == '82 20':
        message = 'operating/display settings'
        length = 10
    else:
        print('not a recognised message ID:', mID)
        message = ''
        length = 6
    
    # combine message plus parameter (if more than 6 bytes)
    if length > 0:
        message_params = reply[18:18+(3*length-1)]
        return message, message_params
    else:
        return message



# %% ---------------------------------------------------------------------------
# CONTROLLER INFO/FUNCTIONS
def identify(s):
    """flash display to indicate which controller is addressed"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands["identify"])


def disconnect(s):
    """inform device of a pending disconnect"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands["disconnect"])


def get_serial(s):
    """"get controller serial number"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands["req_serial"])
    reply = recvreply(s)
    message, message_params = decode_reply(reply)
    serial_hex = message_params[12:24]  # serial number encoded in bytes 4..7 (starting at 0)
    serial_num = hexstr_to_int(serial_hex)
    print('serial number: ', f"{serial_num:010d}")  # give out integer value


def get_info(s):
    """get hardware information, see APT protocol page 46"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands["req_info"])
    reply = recvreply(s)
    message, message_params = decode_reply(reply)
    print('sorry, decoding not implemented yet, check APT protocol page 46')


def save_params(s):
    """write parameters to eeprom"""
    if not s.is_open: print('no serial connection'); return
    print('are you sure you want to save the current parameters to EEPROM? (y/n)')
    userinput = input()
    if userinput == 'y':
        sendcommand(s, commands["save_params"])
        print('saving parameters to EEPROM')
    else:
        print('aborted')


def restore_factory_settings(s):
    """restore factory settings"""
    if not s.is_open: print('no serial connection'); return
    print('are you sure you want to restore factory settings? (y/n)')
    userinput = input()
    if userinput == 'y':
        sendcommand(s, commands["restorefactset"])
        print('restoring factory settings')
    else:
        print('aborted')


def get_disp_params(s):
    """get display settings (brightness/timeout)"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands["req_kcube_params"])
    reply = recvreply(s)
    message, message_params = decode_reply(reply)
    disp_brightness = hexstr_to_int(message_params[6:11])
    disp_timeout = hexstr_to_int(message_params[12:17], signed=True)
    print('display brightness: ', disp_brightness)
    if int(disp_timeout) == -1:
        print('display timeout: never')
    else:
        print('display timeout: ', disp_timeout)
    return disp_brightness, disp_timeout

def set_disp_params(s, brightness=90, timeout=-1):
    """set display settings (brightness/timeout)"""
    if not s.is_open: print('no serial connection'); return
    if (brightness < 0) or (brightness > 100):
        print('please choose a brightness in 0..100')
        return
    if (timeout < -1) or (timeout > 480):
        print('please choose a timeout between 0..480 or -1')
    disp_brightness_hex = int_to_hexstr(brightness)
    disp_timeout_hex = int_to_hexstr(timeout)
    command = commands["set_kcube_params"] + ' 01 00' + disp_brightness_hex + disp_timeout_hex + ' 00 00 00 00'
    sendcommand(s, command)
    set_brightness, set_timeout = get_disp_params(s)

# %% ---------------------------------------------------------------------------
# CHANNEL OUTPUT CONTROL FUNCTIONS
def get_chan_status(s, channel=1):
    """"get hardware channel status"""
    if not s.is_open: print('no serial connection'); return
    chan_command = commands["req_HWchan_status"][0:7] + str(channel) + commands["req_HWchan_status"][8:]
    sendcommand(s, chan_command)
    reply = recvreply(s)
    channel_status = decode_reply(reply)
    if channel_status == '1':
        print('channel ', channel, ' enabled')
    elif channel_status == '2':  # x02 means channel is disabled
        print('channel ', channel, ' disabled')
    else:
        print('no reply')


def en_chan(s, channel=1):
    """"enable hardware channel"""
    if not s.is_open: print('no serial connection'); return
    chan_command = commands["enable_HWchan"][0:7] + str(channel) + commands["enable_HWchan"][8:]
    sendcommand(s, chan_command)
    get_chan_status(s, channel)


def dis_chan(s, channel=1):
    """disable hardware channel"""
    if not s.is_open: print('no serial connection'); return
    chan_command = commands["disable_HWchan"][0:7] + str(channel) + commands["disable_HWchan"][8:]
    sendcommand(s, chan_command)
    get_chan_status(s, channel)

# %% ---------------------------------------------------------------------------
# CONTROL SETTINGS
def set_voltage(s, voltage : float, channel=1):
    """"set output voltage (0..25)"""
    if not s.is_open: print('no serial connection'); return
    # check if voltage is within the limits, otherwise set it to min/max
    if (voltage < 0) or (voltage > 25):
        print('invalid voltage: ', str(voltage))
        print('please set the voltage in V (0..25V)')
        return
    
    voltage_hex = int_to_hexstr(voltage*1000)  # input in mV
    command = commands["set_voltage"] + ' 50 01 0' + str(channel) + ' 00 ' + voltage_hex
    sendcommand(s, command)
    if FLAG_DEBUG: print('voltage set')


def get_voltage(s, channel=1):
    """"get output voltage"""
    if not s.is_open: print('no serial connection'); return
    chan_command = commands["req_voltage"][0:10] + str(channel) + commands["req_voltage"][11:]
    sendcommand(s, chan_command)
    reply = recvreply(s)
    message, message_params = decode_reply(reply)
    # voltage just encoded in last two byte
    voltage = hexstr_to_int(message_params[-5:]) / 1000  # convert to V
    print('set voltage:', voltage, 'V')
    return voltage


def set_freq(s, frequency : int, channel=1):
    """"set output frequency (500..10000 Hz)"""
    if not s.is_open: print('no serial connection'); return
    # check if frequency is within the limits
    if (frequency < 500) or (frequency > 10000):
        print('invalid frequency: ', str(frequency))
        print('please set the frequency in Hz (500..10000)')
        return
    
    freq_hex = int_to_hexstr(frequency)  # input in Hz
    command = commands["set_frequency"] + ' 50 01 0' + str(channel) + ' 00 ' + freq_hex
    sendcommand(s, command)
    if FLAG_DEBUG: print('frequency set')


def get_freq(s, channel=1):
    """"get output frequency"""
    if not s.is_open: print('no serial connection'); return
    chan_command = commands["req_frequency"][0:10] + str(channel) + commands["req_frequency"][11:]
    sendcommand(s, chan_command)
    reply = recvreply(s)
    message, message_params = decode_reply(reply)
    # frequency just encoded in last two byte
    frequency = hexstr_to_int(message_params[-5:])
    print('set frequency:', frequency, 'Hz')
    return frequency


# %% ---------------------------------------------------------------------------
# ADC mode control
def get_ADCinmode(s):
    """"get ADC input mode"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands["req_ADCinmode"])
    reply = recvreply(s)
    ADCinmode = int(decode_reply(reply))
    if ADCinmode:
        print('ADC input mode enabled')
    else:
        print('ADC input mode disabled')


def en_ADCinmode(s):
    """"enable ADC input mode"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands["enable_ADCinmode"])
    sendcommand(s, commands["req_ADCinmode"])
    reply = recvreply(s)
    ADCinmode = int(decode_reply(reply))
    if ADCinmode:
        print('ADC input mode enabled')
    else:
        print('ADC input mode disabled')


def dis_ADCinmode(s):
    """disable ADC input mode"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands["disable_ADCinmode"])
    sendcommand(s, commands["req_ADCinmode"])
    reply = recvreply(s)
    ADCinmode = int(decode_reply(reply))
    if ADCinmode:
        print('ADC input mode enabled')
    else:
        print('ADC input mode disabled')


def get_ADCparams(s):
    """get ADC parameters"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands["req_ADCparams"])
    reply = recvreply(s)
    message, message_params = decode_reply(reply)
    ADC_error = hexstr_to_int(message_params[6:11], signed=True)
    ADMAX_Value = hexstr_to_int(message_params[12:17], signed=False)
    print('ADC error: ', ADC_error)
    print('ADMAX value: ', ADMAX_Value)
# EOF --------------------------------------------------------------------------