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
    "req_info":         "05 00 00 00 50 01", # get hardware info (4+84 byte)
    "req_serial":       "15 00 00 00 50 01", # get serial number (6 + 40 byte)
    "set_serial":       "14 00 28 00 d0 01", # set serial number (+4+4+8 byte)
    "enable_HWchan":    "10 02 01 01 50 01", # enable HW channel (3rd byte defines channel)
    "disable_HWchan":   "10 02 01 02 50 01", # disable HW channel (3rd byte defines channel), x02 means "off"
    "req_HWchan_status":"11 02 01 00 50 01", # get HW device channel status (3rd byte defines channel)
    "lock_wheel":       "50 02 00 01 50 01", # lock the wheel on top of the device
    "unlock_wheel":     "50 02 00 02 50 01", # unlock the wheel on top of the device
    "wheel_status":     "51 02 00 00 50 01", # request wheel lock status
    "set_voltage":      "01 20 06 00 d0 01", # set output voltage (+6 byte)
    "req_voltage":      "02 20 01 01 50 01", # get output voltage (4th part could be 01 or 02) (6+6 byte)
    "set_frequency":    "05 20 06 00 d0 01", # set frequency (+6 byte)
    "req_frequency":    "06 20 01 01 50 01", # get frequency (4th part could be 01 or 02) (6+6 byte)
    "enable_ADCinmode": "08 20 01 01 50 01", # enable analog input mode
    "disable_ADCinmode":"08 20 01 00 50 01", # disable analog input mode
    "req_ADCinmode":    "09 20 01 00 50 01", # get analog input mode (4th byte is en/dis)
    "set_trigger_mode": "0b 20 01 01 50 01", # set device trigger pin mode (4th byte sets mode)
    "req_trigger_mode": "0c 20 01 00 50 01", # get device trigger pin mode
    "req_ADCparams":    "0e 20 01 00 50 01", # get ADC parameters (6+4 bytes)
    "set_swfreq":       "10 20 04 00 d0 01", # set device switching frequency (+4 byte)
    "req_swfreq":       "11 20 01 00 50 01", # get device switching frequency (6+4 byte)
    "mode_chan_V1":     "16 20 01 01 50 01", # set channel V1 (4th byte defines output mode)
    "mode_chan_V2":     "16 20 01 02 50 01", # set channel V2 (4th byte defines output mode)
    "mode_chan_sw":     "16 20 01 03 50 01", # set switching V1-V2 (4th byte defines output mode)
    "mode_chan_off":    "16 20 01 00 50 01", # set channel off (4th byte defines output mode)
    "req_chan_mode":    "17 20 01 00 50 01", # get device channel status (3rd byte defines channel) (not working?)
    "set_LUT_value":    "20 20 06 00 d0 01", # set device LUT values (+6 byte)
    "req_LUT_value":    "21 20 01 00 50 01", # get device LUT values (6+6 byte)
    "set_LUT_params":   "23 20 1e 00 d0 01", # set LUT parameters (+30 byte)
    "req_LUT_params":   "24 20 01 00 50 01", # get LUT paramaters (6+30 byte)
    "start_LUT_output": "26 20 01 00 50 01", # start LUT output
    "stop_LUT_output":  "27 20 01 00 50 01", # stop LUT output
    "set_output_status":"28 20 0a 00 d0 01", # set device output status (+10 byte)
    "req_output_status":"29 20 01 00 50 01", # get device output status (6+10 byte)
    "enable_status_update":"40 20 01 01 50 01", # enable status update when panel is used to change parameters
    "disable_status_update":"40 20 01 00 50 01", # disable status update when panel is used to change parameters
    "req_status_update":"41 20 01 00 50 01", # get status update status
    "set_kcube_params": "80 20 0a 00 d0 01", # set display parameters (+10 byte)
    "req_kcube_params": "81 20 01 00 50 01", # get display parameters (brightness/timeout)
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


def int_to_hexstr(value_num : float, signed=True, bytenum=2):
    """ convert an integer voltage/frequency to hex strings for transmitting to KLC controller"""
    value_int = int(value_num)
    # convert to bytes (e.g. b'\xee\x76\x01\x00')
    value_bytes = value_int.to_bytes(4, byteorder='little', signed=signed)
    # convert to hex string with space at the beginning and in between
    value_hex = ''
    for n in range(bytenum):
        value_hex = value_hex + ' ' + format(value_bytes[n], '02x')
    if FLAG_DEBUG: print(value_bytes.hex(), value_hex)
    return value_hex


def hexstr_to_ascii(hexstr):
    """ convert a string of hex values to ASCII characters
    only used to read out the model name
    """
    # remove whitespace
    hexstr = "".join(hexstr.split())
    asciistr = ""
    for i in range(0, len(hexstr), 2):
        # extract two characters from hex string
        part = hexstr[i : i + 2]
        # change it into base 16 and typecast as character
        ch = chr(int(part, 16))
        asciistr += ch
    # strip zeros
    asciistr = asciistr.strip('\x00')
    return asciistr


# %% ---------------------------------------------------------------------------
# SERIAL FUNCTIONS

# create a serial connection with the recommended parameters
def openKLC(port='', SN = ''):
    """create a serial connection with the recommended parameters to either the defined port
    or to a specified serial number SN
    """
    s = serial.Serial()
    s.baudrate = 115200
    s.bytesize = serial.EIGHTBITS
    s.parity = serial.PARITY_NONE
    s.stopbits = serial.STOPBITS_ONE # number of stop bits
    s.timeout = 2
    s.rtscts = True # enable hardware (TRS/CTS) flow control
    
    if not port:
        # find available ports depending on operating system
        if sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            ports = glob.glob('/dev/serial/by-id/*Kinesis_LC_Controller*' + SN + '*')
            if not ports:
                print('selected SN', SN, 'not available')
                available_ports = glob.glob('/dev/serial/by-id/*Kinesis_LC_Controller*')
                print('available SNs:')
                print(available_ports)
        elif sys.platform.startswith('win'):
            ports = ['COM%s' % (i + 1) for i in range(256)]
        else:
            raise EnvironmentError('Unsupported platform')
    else:
        ports = [port]
    
    # try to open the ports until one works
    if not ports:
        print('no serial port selected, aborting')
        s = ''
        return s
    
    # try to open the selected port(s) until one works
    for port in ports:
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
        print('opening failed somehow')
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
    time.sleep(0.04) # has to be at least 20 ms to work
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
    
    if mID == '02 00':
        message = 'pending disconnect'
        length = 0
    elif mID == '03 20':
        message = 'output voltage'
        length = 6
    elif mID == '06 00':
        message = 'hardware info'
        length = 84
    elif mID == '07 20':
        message = 'output frequency'
        length = 6
    elif mID == '0a 20':
        message = 'analog input mode'
        analog_mode = reply[10]
        return analog_mode
    elif mID == '0d 20':
        message = 'trigger pin mode'
        trigger_pin_mode = reply[10]
        return trigger_pin_mode
    elif mID == '0f 20':
        message = 'ADC parameters'
        length = 6
    elif mID == '12 02':
        message = 'channel status'
        channel = reply[7]
        channel_status = reply[10]
        return channel_status
    elif mID == '12 20':
        message = 'switching frequency'
        length = 4
    elif mID == '16 00':
        message = 'serial number'
        length = 40
    elif mID == '18 20':
        message = 'channel mode'
        channel_mode = reply[10]
        return channel_mode
    elif mID == '22 20':
        message = 'LUT value'
        length = 6
    elif mID == '25 20':
        message = 'LUT parameters'
        length = 30
    elif mID == '30 20':
        message = 'output status'
        length = 10
    elif mID == '42 20':
        message = 'status update'
        length = 26
    elif mID == '52 02':
        message = 'wheel lock status'
        lock_status = reply[10]
        return lock_status
    elif mID== '81 00':
        message = 'rich response' + reply[17:]
        length = 64
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
    sendcommand(s, commands['identify'])


def get_serial(s):
    """get controller serial number"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['req_serial'])
    reply = recvreply(s)
    message, message_params = decode_reply(reply)
    serial_hex = message_params[12:24]  # serial number encoded in bytes 4..7 (starting at 0)
    serial_num = hexstr_to_int(serial_hex)
    if FLAG_DEBUG:
        print('serial number: ', f"{serial_num:010d}")  # give out integer value
        print('raw reply:')
        print(reply)
        # yeah, apparently the preset serial number is always 75d...
    return serial_num


def set_serial(s, new_SN : str):
    """set controller serial number"""
    # weird structure: Data0: 01 00 00 00, Data1: serial number
    if not s.is_open: print('no serial connection'); return
    # check if length is reasonable
    if len(new_SN) != 8:
        print('please select a serial number that is 8 digits long')
        return
    # check if string of decimals
    try: 
        int(new_SN)
    except ValueError: 
        print('use a serial number that only consists of decimals')
        return
    # read previous serial number
    old_SN = get_serial(s)
    print('previous SN :', old_SN)
    # convert serial number
    new_SN_hexstr = int_to_hexstr(int(new_SN), bytenum=4)
    command = f"{commands['set_serial']} 01 00 00 00{new_SN_hexstr}" + ' 00'*32
    if FLAG_DEBUG:
        print(command)
    sendcommand(s, command)
    set_SN = get_serial(s)
    print('newly set SN:', set_SN)
    print('hint: SNs are not persistent and will be reset to 75d when powering off')


def get_info(s):
    """get hardware information, see APT protocol page 46"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['req_info'])
    reply = recvreply(s)
    message, hwinfo = decode_reply(reply)
    sn = hexstr_to_int(hwinfo[0:11]) # 4 byte serial number
    model_number = hexstr_to_ascii(hwinfo[12:35]) # 8 byte alphanumeric model number
    hw_type = hexstr_to_int(hwinfo[36:41]) # 2 byte describes type of hardware
    fw_minor = hexstr_to_int(hwinfo[42:44]) # minor firmware version (1 byte)
    fw_interim = hexstr_to_int(hwinfo[45:47]) # interim firmware version (1 byte)
    fw_major = hexstr_to_int(hwinfo[48:50]) # major firmware version (1 byte)
    fw_reserved = hexstr_to_int(hwinfo[51:53]) # always 00
    hw_version = hexstr_to_int(hwinfo[-17:-12]) # 2 byte hardware version
    hw_mod_state = hexstr_to_int(hwinfo[-11:-6]) # 2 byte hardware modification state
    n_channels = hexstr_to_int(hwinfo[-5:]) # 2 byte number of channels
    print(f"serial number:\t\t{sn}\nmodel number:\t\t{model_number}\nfirmware version:\t{fw_major}.{fw_interim}.{fw_minor}")
    print(f"hardware type:\t\t{hw_type}\nhardware version:\t{hw_version}")
    print(f"modification state:\t{hw_mod_state}\nnumber of channels:\t{n_channels}")


# %% ---------------------------------------------------------------------------
# HARDWARE CHANNEL OUTPUT CONTROL FUNCTIONS
# enable/disable output, this needs to be combined with set_chan_mode
# KLCs only have one channel so the argument is somewhat superfluous

def get_hwchan_status(s, channel=1):
    """get hardware channel status"""
    if not s.is_open: print('no serial connection'); return
    if channel != 1: print('invalid channel, using channel 1'); channel = 1
    # chan_command = f"{commands['req_HWchan_status'][0:7]}{channel}{commands['req_HWchan_status'][8:]}"
    # sendcommand(s, chan_command)
    sendcommand(s, commands['req_HWchan_status'])
    reply = recvreply(s)
    channel_status = decode_reply(reply)
    if channel_status == '1':
        print('channel ', channel, ' enabled')
    elif channel_status == '2':
        print('channel ', channel, ' disabled')
    else:
        print('no reply, channel status unknown')
    return channel_status

def en_hwchan(s, channel=1):
    """enable hardware channel"""
    if not s.is_open: print('no serial connection'); return
    # chan_command = f"{commands['enable_HWchan'][0:7]}{channel}{commands['enable_HWchan'][8:]}"
    # sendcommand(s, chan_command)
    sendcommand(s, commands['enable_HWchan'])
    get_hwchan_status(s, channel)


def dis_hwchan(s, channel=1):
    """disable hardware channel"""
    if not s.is_open: print('no serial connection'); return
    # chan_command = f"{commands['disable_HWchan'][0:7]}{channel}{commands['disable_HWchan'][8:]}"
    # sendcommand(s, chan_command)
    sendcommand(s, commands['disable_HWchan'])
    get_hwchan_status(s, channel)


# %% ---------------------------------------------------------------------------
# HARDWARE WHEEL LOCK

def lock_wheel(s):
    """lock device control wheel"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['lock_wheel'])
    get_wheel_status(s)


def unlock_wheel(s):
    """unlock device control wheel"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['unlock_wheel'])
    get_wheel_status(s)


def get_wheel_status(s):
    """"get device control wheel lock status"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['wheel_status'])
    reply = recvreply(s)
    lock_status = decode_reply(reply)
    if lock_status == '1':
        print('device wheel locked')
    elif lock_status == '2':
        print('device wheel unlocked')
    else:
        print('invalid lock status')


# %% ---------------------------------------------------------------------------
# CONTROL SETTINGS

def set_voltage(s, voltage : float, channel=1):
    """set output voltage (0..25)"""
    if not s.is_open: print('no serial connection'); return
    if not channel in [1, 2]: print('invalid channel, select 1 or 2'); return
    # check if voltage is within the limits, otherwise set it to min/max
    if (voltage < 0) or (voltage > 25):
        print('invalid voltage: ', str(voltage))
        print('please set the voltage in V (0..25V)')
        return
    
    voltage_hex = int_to_hexstr(voltage*1000)  # input in mV
    command = f"{commands['set_voltage']} 00 01 0{channel} 00 {voltage_hex}"
    sendcommand(s, command)
    if FLAG_DEBUG: print('voltage set')


def get_voltage(s, channel=1):
    """get output voltage"""
    if not s.is_open: print('no serial connection'); return
    if not channel in [1, 2]: print('invalid channel, select 1 or 2'); return
    chan_command = f"{commands['req_voltage'][0:10]}{channel}{commands['req_voltage'][11:]}"
    sendcommand(s, chan_command)
    reply = recvreply(s)
    message, message_params = decode_reply(reply)
    # voltage just encoded in last two byte
    voltage = hexstr_to_int(message_params[-5:]) / 1000  # convert to V
    if FLAG_DEBUG: print('set voltage:', voltage, 'V')
    return voltage


def set_freq(s, frequency : int, channel=1):
    """set output frequency (500..10000 Hz)"""
    if not s.is_open: print('no serial connection'); return
    if not channel in [1, 2]: print('invalid channel, select 1 or 2'); return
    # check if frequency is within the limits
    if (frequency < 500) or (frequency > 10000):
        print('invalid frequency: ', str(frequency))
        print('please set the frequency in Hz (500..10000)')
        return
    
    freq_hex = int_to_hexstr(frequency)  # input in Hz
    # command = commands['set_frequency'] + ' 50 01 0' + str(channel) + ' 00 ' + freq_hex
    command = f"{commands['set_frequency']} 00 01 0{channel} 00 {freq_hex}"
    sendcommand(s, command)
    if FLAG_DEBUG: print('frequency set')


def get_freq(s, channel=1):
    """get output frequency"""
    if not s.is_open: print('no serial connection'); return
    if not channel in [1, 2]: print('invalid channel, select 1 or 2'); return
    chan_command = f"{commands['req_frequency'][0:10]}{channel}{commands['req_frequency'][11:]}"
    sendcommand(s, chan_command)
    reply = recvreply(s)
    message, message_params = decode_reply(reply)
    # frequency just encoded in last two byte
    frequency = hexstr_to_int(message_params[-5:])
    if FLAG_DEBUG: print('set frequency:', frequency, 'Hz')
    return frequency


# %% ---------------------------------------------------------------------------
# DEVICE TRIGGER PIN MODE

def get_trigger_mode(s):
    """get device trigger pin mode"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['req_trigger_mode'])
    reply = recvreply(s)
    trigger_mode = int(decode_reply(reply))
    if trigger_mode == 1:
        print('trigger pin mode: Pin1 out, Pin2 out')
    elif trigger_mode == 2:
        print('trigger pin mode: Pin1 in, Pin2 out')
    elif trigger_mode == 3:
        print('trigger pin mode: Pin1 out, Pin2 in')
    else:
        print('invalid trigger_mode')
    return trigger_mode


def set_trigger_mode(s, mode):
    """set device trigger pin mode
    mode x01: pin1 out  pin2 out
    mode x02: pin1 in   pin2 out
    mode x03: pin1 out  pin2 in
    """
    if not s.is_open: print('no serial connection'); return
    if (mode < 1) or (mode > 3):
        print('select a valid mode, meaning 1, 2 or 3')
        return
    trig_command = commands['set_trigger_mode'][0:10] + str(mode) + commands['set_trigger_mode'][11:]
    sendcommand(s, trig_command)
    get_trigger_mode(s)


# %% ---------------------------------------------------------------------------
# ADC mode control

def get_ADCinmode(s):
    """get ADC input mode"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['req_ADCinmode'])
    reply = recvreply(s)
    ADCinmode = int(decode_reply(reply))
    if ADCinmode:
        print('ADC input mode enabled')
    else:
        print('ADC input mode disabled')
    return ADCinmode


def en_ADCinmode(s):
    """enable ADC input mode"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['enable_ADCinmode'])
    get_ADCinmode(s)


def dis_ADCinmode(s):
    """disable ADC input mode"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['disable_ADCinmode'])
    get_ADCinmode(s)


def get_ADCparams(s):
    """get ADC parameters"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['req_ADCparams'])
    reply = recvreply(s)
    message, ADCparams = decode_reply(reply)
    ADC_error = hexstr_to_int(ADCparams[6:11], signed=True)
    ADMAX_Value = hexstr_to_int(ADCparams[12:17], signed=False)
    print(f"ADC error:\t{ADC_error}")
    print(f"ADMAX value:\t{ADMAX_Value}")
    return ADC_error, ADMAX_Value


# %% ---------------------------------------------------------------------------
# SWITCHING FREQUENCY SETTING
# frequency to switch between V1 and V2 if CHANENABLESTATE = x03

def set_swfreq(s, frequency):
    """set switching frequency (0.1..150 Hz)"""
    if not s.is_open: print('no serial connection'); return
    # check if switching frequency is within the limits
    if (frequency < 0.1) or (frequency > 150):
        print('invalid frequency: ', str(frequency))
        print('please set the switching frequency in Hz (0.1..150)')
        return
    
    freq_hex = int_to_hexstr(int(frequency*10))  # input is multiplied by 10!
    command = f"{commands['set_swfreq']} 01 00{freq_hex}"
    sendcommand(s, command)
    print('switching frequency set to:', get_swfreq(s), ' Hz')
    if FLAG_DEBUG: print('switching frequency set to:', freq_hex)


def get_swfreq(s):
    """get switching frequency in Hz"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['req_swfreq'])
    reply = recvreply(s)
    message, message_params = decode_reply(reply)
    # switching frequency just encoded in last two byte
    swfreq = hexstr_to_int(message_params[-5:]) / 10  # value is stored as 10xHz
    if FLAG_DEBUG: print('set switching frequency:', sfreq, 'Hz')
    return swfreq


# %% ---------------------------------------------------------------------------
# KLC CHANNEL OUTPUT CONTROL FUNCTIONS
# controls the output mode, enable and disable output with en/dis_hwchan

def get_chan_mode(s):
    """get channel mode"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['req_chan_mode'])
    reply = recvreply(s)
    channel_mode = decode_reply(reply)
    if channel_mode == '0':
        print('output disabled')
    elif channel_mode == '1':
        print('output V1 set')
    elif channel_mode == '2':
        print('output V2 set')
    elif channel_mode == '3':
        print('output V1-V2 switching set')
    else:
        print('no reply, channel status unknown')


def set_chan_mode(s, mode=1):
    """set channel with specific output mode"""
    if not s.is_open: print('no serial connection'); return
    if mode == 0:
        mode_command = commands['mode_chan_off']
    elif mode == 1:
        mode_command = commands['mode_chan_V1']
    elif mode == 2:
        mode_command = commands['mode_chan_V2']
    elif mode == 3:
        mode_command = commands['mode_chan_sw']
    else:
        print('invalid mode, choose 0, 1, 2, or 3')
        return
    sendcommand(s, mode_command)
    get_chan_mode(s)


# %% ---------------------------------------------------------------------------
# LUT OUTPUT

def set_LUT_value(s, idx : int, voltage : int):
    """set single LUT value at specific index"""
    # check if voltage is within the limits
    if (voltage < 0) or (voltage > 25):
        print('invalid voltage: ', str(voltage))
        print('please set the voltage in V (0..25V)')
        return
    # check if index is within limits
    if (idx < 0) or (idx > 511):
        print('invalid index:', idx)
        print('select index in [0..511]')
        return
    
    voltage_hex = int_to_hexstr(voltage*1000)  # input in mV
    idx_hex = int_to_hexstr(idx)
    command = f"{commands['set_LUT_value']} 01 00{idx_hex}{voltage_hex}"
    sendcommand(s, command)


def get_LUT_value(s, idx : int):
    """get single LUT value at specified index"""
    # the index is being set in the 4th byte which is only enough for 256 values
    # no idea how to request the values from 256-511
    if not s.is_open: print('no serial connection'); return
    # check if index is within limits
    if (idx < 0) or (idx > 511):
        print('invalid index:', idx)
        print('select index in [0..511]')
        return
    if idx > 255: print('sorry, havent found out how to read out addresses 256-511') ; return
    idx_hex = int_to_hexstr(idx, bytenum=1)
    command = f"{commands['req_LUT_value'][0:8]}{idx_hex} {commands['req_LUT_value'][12:]}"
    sendcommand(s, command)
    reply = recvreply(s)
    message, message_params = decode_reply(reply)
    voltage = hexstr_to_int(message_params[-5:]) / 1000 # voltage encoded in last two byte
    idx_read = hexstr_to_int(message_params[-11:-6]) # index encoded in the two before that
    print('LUT voltage at index', idx_read, ':', voltage, 'V')
    return voltage


def set_LUT_params(s, mode=1, cycle_length=0, num_cycles=0, delay_time=0, pre_cycle_rest=0):
    """set LUT parameters
    mode: 0=disabled 1=continuous, 2=fixed number of cycles
    cycle_length: 1..512 (number of samples)
    num_cycles: 1..2147483648 (4 byte) (number of cycles to output the LUT if mode=2)
    delay_time: 1..2147483648 (4 byte) (time waiting after setting each output value)
    pre_cycle_rest (delay time before starting the LUT output)
    """
    mode_hex = int_to_hexstr(mode)
    cycle_length_hex = int_to_hexstr(cycle_length)
    num_cycles_hex = int_to_hexstr(num_cycles, bytenum=4)
    delay_time_hex = int_to_hexstr(delay_time, bytenum=4)
    pre_cycle_rest_hex = int_to_hexstr(pre_cycle_rest, bytenum=4)
    # exactly following the documentation including the reserved bytes:
    command = f"{commands['set_LUT_params']} 01 00{mode_hex}{cycle_length_hex}{num_cycles_hex}{delay_time_hex}{pre_cycle_rest_hex} 0a" + " 00"*5 + " 01" + " 00"*5
    sendcommand(s, command)


def get_LUT_params(s):
    """get LUT parameters
    mode: 1=continuous, 2=fixed number of cycles
    cycle_length: 1..512 (number of samples)
    num_cycles: 1..2147483648 (4 byte) (number of cycles to output the LUT if mode=2)
    delay_time: 1..2147483648 (4 byte) (time waiting after setting each output value)
    pre_cycle_rest (delay time before starting the LUT output)
    """
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['req_LUT_params'])
    reply = recvreply(s)
    message, LUT_params = decode_reply(reply)
    mode = int(LUT_params[7])
    cycle_length = hexstr_to_int(LUT_params[12:17])
    num_cycles = hexstr_to_int(LUT_params[18:29])
    delay_time = hexstr_to_int(LUT_params[30:41])
    pre_cycle_rest = hexstr_to_int(LUT_params[42:53])
    print(f"mode:\t\t{mode}\ncycle_length:\t{cycle_length}\nnum_cycles:\t{num_cycles}") 
    print(f"delay_time:\t{delay_time}\npre_cycle_rest:\t{pre_cycle_rest}")


def start_LUT_output(s):
    """start LUT output"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['start_LUT_output'])
    get_hwchan_status(s)
    get_chan_mode(s)
    print('starting LUT output with:')
    get_LUT_params(s)


def stop_LUT_output(s):
    """stop LUT output"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['stop_LUT_output'])
    get_hwchan_status(s)
    get_chan_mode(s)


# %% ---------------------------------------------------------------------------
# CONFIGURE ALL OPERATING PARAMETERS TOGETHER

def set_output_status(s, mode=1, voltage=5, frequency=1000, frequency_flag=1):
    """set device output status:
    mode: 0=disabled, 1=V1, 2=V2, 3=switching
    frequency_flag: 0=no change, 1=frequency changed (seems to not matter really)
    """
    if not s.is_open: print('no serial connection'); return
    # check if voltage is within the limits
    if (voltage < 0) or (voltage > 25):
        print('invalid voltage: ', str(voltage))
        print('please set the voltage in V (0..25V)')
        return
    # check if frequency is within the limits
    if (frequency < 500) or (frequency > 10000):
        print('invalid frequency: ', str(frequency))
        print('please set the frequency in Hz (500..10000)')
        return
    if mode not in [0, 1, 2, 3]: print('select a valid mode: [0, 1, 2, 3]'); return
    
    mode_hex = int_to_hexstr(mode)
    voltage_hex = int_to_hexstr(voltage*1000)
    frequency_hex = int_to_hexstr(frequency)
    freq_flag_hex = int_to_hexstr(frequency_flag)
    command = f"{commands['set_output_status']} 01 00{mode_hex}{voltage_hex}{frequency_hex}{freq_flag_hex}"
    sendcommand(s, command)
    get_output_status(s)
    get_chan_mode(s)


def get_output_status(s):
    """get device output status:
    output_active: 0 or 1
    error_flag: 1=DC offset error
    """
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['req_output_status'])
    reply = recvreply(s)
    message, status = decode_reply(reply)
    output_active = int(status[7])
    voltage = hexstr_to_int(status[12:17]) / 1000
    frequency = hexstr_to_int(status[18:23])
    error_flag = int(status[25])
    print(f"active:\t\t{output_active}\nerrors:\t\t{error_flag}")
    print(f"voltage:\t{voltage} V\nfrequency:\t{frequency} Hz")


# %% ---------------------------------------------------------------------------
# CONFIGURE STATUS UPDATE

def en_status_update(s):
    """enable status update when device panel is used to change parameters"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['enable_status_update'])


def dis_status_update(s):
    """disable status update when device panel is used to change parameters"""
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['disable_status_update'])


def get_status_update(s):
    """get status update
    chan_state: 1=V1, 2=V2, 3=Switching, 0=disabled
    ADCmode: 0=Analog in disabled, 1=enabled
    trig_conf: 1=Pin1/Pin2 Out, 2=Pin1 in/Pin2 out, 3=Pin1 out/Pin2 in
    wheel_status: 0=unlocked, 1=locked
    error_flag: most likely the second digit is the only "DC offset error" flag
    """
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['req_status_update'])
    reply = recvreply(s)
    message, status = decode_reply(reply)
    chan_mode = int(status[7])
    V1 = hexstr_to_int(status[12:17]) / 1000
    f1 = hexstr_to_int(status[18:23])
    V2 = hexstr_to_int(status[24:29]) / 1000
    f2 = hexstr_to_int(status[30:35])
    f_sw = hexstr_to_int(status[36:41]) / 10
    disp_brightness = hexstr_to_int(status[42:47])
    disp_timeout = hexstr_to_int(status[48:53], signed=True)
    ADCmode = hexstr_to_int(status[54:59])
    trig_conf = hexstr_to_int(status[60:65])
    wheel_status = hexstr_to_int(status[66:71])
    error_flag = hexstr_to_int(status[72:])
    print(f"output mode:\t{chan_mode}")
    print(f"V1={V1}V f1={f1}Hz\nV2={V2}V f2={f2}\nf_sw={f_sw}")
    return chan_mode, V1, f1, V2, f2, f_sw


# %% ---------------------------------------------------------------------------
# DISPLAY PARAMETERS

def get_disp_params(s):
    """get display settings (brightness/timeout)
    timeout multiplies with roughly 30s per value
    timeout=-1 means no timeout
    """
    if not s.is_open: print('no serial connection'); return
    sendcommand(s, commands['req_kcube_params'])
    reply = recvreply(s)
    message, disp_params = decode_reply(reply)
    disp_brightness = hexstr_to_int(disp_params[6:11])
    disp_timeout = hexstr_to_int(disp_params[12:17], signed=True)
    print('display brightness: ', disp_brightness)
    if int(disp_timeout) == -1:
        print('display timeout: never')
    else:
        print('display timeout roughly: ', disp_timeout/2,  ' min')
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
    command = f"{commands['set_kcube_params']} 01 00{disp_brightness_hex}{disp_timeout_hex} 00 00 00 00"
    sendcommand(s, command)
    set_brightness, set_timeout = get_disp_params(s)


# %% ---------------------------------------------------------------------------
# EEPROM SETTINGS

def save_params(s):
    """write parameters to eeprom
    does not save the serial number, sadly
    """
    if not s.is_open: print('no serial connection'); return
    print('are you sure you want to save the current parameters to EEPROM? (y/n)')
    userinput = input()
    if userinput == 'y':
        sendcommand(s, commands['save_params'])
        print('saving parameters to EEPROM')
    else:
        print('aborted')


def restore_factory_settings(s):
    """restore factory settings"""
    if not s.is_open: print('no serial connection'); return
    print('are you sure you want to restore factory settings? (y/n)')
    userinput = input()
    if userinput == 'y':
        print('you dont want to not keep the current settings? (y/n)')
        userinput = input()
        if userinput == 'n':
            sendcommand(s, commands['restorefactset'])
            print('restoring factory settings')
            return
    print('aborted')


# EOF --------------------------------------------------------------------------