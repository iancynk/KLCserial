#!/bin/usr/python
# Python wrapper to communicate with Thorlabs KLC101 controllers
# %% ---------------------------------------------------------------------------
# imports

import serial
import time
import glob
import sys

# %% ---------------------------------------------------------------------------
# KLC class
class KLC () :
    # set this to True when you need a lot more output
    DEBUG = False
    
    def __init__ (self, port='', SN='', DEBUG=False) :
        self.ser = None
        # open serial connection
        if port:
            # with port if port is given
            self.openKLC(port=port)
        elif SN:
            # with serial number if given
            self.openKLC(SN=SN)
        else:
            # otherwise just pick the first found port
            self.openKLC()
        if self.ser: self.identify()
        # more output info
        if DEBUG: self.DEBUG = True
    
    
    def __str__(self):
        return "Is a serial instance of a KLC controller."
        self.get_info()
    
    # --------------------------------------------------------------------------
    # COMMANDS
    
    cmds = {
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
    # --------------------------------------------------------------------------
    # CONVERSION FUNCTIONS
    
    def hexstr_to_int(self, value_hex, signed=False):
        """convert voltage/frequency value from hex to a useable integer"""
        # convert hex string to bytes, e.g. ' EE 76 01 00' to b'\xee\x76\x01\x00'
        value_bytes = bytes.fromhex(value_hex)
        # convert bytes to signed integer, eg 'e8 03' becomes 1000 [mV/Hz]
        value_int = int.from_bytes(value_bytes, byteorder='little', signed=signed) 
        if self.DEBUG: print(value_bytes.hex(), value_int, )
        return value_int
    
    
    def int_to_hexstr(self, value_num:float, signed=True, bytenum=2):
        """ convert an integer voltage/frequency to hex strings for transmitting to KLC controller"""
        value_int = int(value_num)
        # convert to bytes (e.g. b'\xee\x76\x01\x00')
        value_bytes = value_int.to_bytes(4, byteorder='little', signed=signed)
        # convert to hex string with space at the beginning and in between
        value_hex = ''
        for n in range(bytenum):
            value_hex = f"{value_hex} {value_bytes[n]:02X}"
        if self.DEBUG: print(value_bytes.hex(), value_hex)
        return value_hex
    
    
    def hexstr_to_ascii(self, hexstr):
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
    
    
    # --------------------------------------------------------------------------
    # SERIAL FUNCTIONS
    
    def openKLC(self, port='', SN = ''):
        """create a serial connection with the recommended parameters to either the defined port
        or to a specified serial number SN
        """
        self.ser = serial.Serial()
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE # number of stop bits
        self.ser.timeout = 2
        self.ser.rtscts = True # enable hardware (TRS/CTS) flow control
        
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
            self.ser = None
            return
        
        # try to open the selected port(s) until one works
        for port in ports:
            try:
                print('opening port', port)
                self.ser.port = port
                self.ser.open()
                time.sleep(0.1)
                break
            except FileNotFoundError:
                print('failed at port', port)
                pass
        
        if self.DEBUG:
            if self.ser.is_open:
                print(self.ser.port, 'is open: ', self.ser.is_open)
        return
    
    
    def port_is_open(self):
        """ check whether serial port is open
        returns False when closed or not a serial connection
        """
        try: 
            if not self.ser.is_open:
                print('serial port not open')
                return False
        except AttributeError:
            print('no serial controller connected, ignoring command')
            return False
        return True
    
    
    def closeKLC(self):
        """ close serial connection"""
        if not self.port_is_open(): return
        self.ser.close()
        print('is open: ', self.ser.is_open)
    
    
    def sendcmd(self, cmd:str):
        """send a command"""
        if not self.port_is_open(): return
        splitstring = cmd.split() # separate in to list of hex values
        cmd_ints = [int(str, 16) for str in splitstring] # convert to integer
        if self.DEBUG: print('sending command: ', cmd_ints)
        self.ser.write(bytes(cmd_ints)) # send integer in binary format to stage
    
    
    def recvreply(self):
        """receive and parse reply"""
        if not self.port_is_open(): return
        time.sleep(0.04) # has to be at least 20 ms to work
        reply = ''
        while self.ser.in_waiting > 0:
            # read every single byte (converted to hex) and add whitespace
            reply += self.ser.read().hex()
            reply += ' '
        # print('reply: ', reply)
        return reply
    
    
    def decodereply(self, reply):
        """convert reply to readable info and split into individual messages"""
        # if no reply, return
        if not reply:
            msg = ''
            params = ''
            return msg, params
            
        mID = reply[0:5] # get the first two bytes as message ID
        # header = reply[0:17] # get the first 6 bytes as header
        params = ''
        if self.DEBUG: print(reply)
        
        match mID: 
            case '02 00':
                msg = 'pending disconnect'
                length = 0
            case '03 20':
                msg = 'output voltage'
                length = 6
            case '06 00':
                msg = 'hardware info'
                length = 84
            case '07 20':
                msg = 'output frequency'
                length = 6
            case '0a 20':
                msg = 'analog input mode'
                analog_mode = reply[10]
                return msg, analog_mode
            case '0d 20':
                msg = 'trigger pin mode'
                trigger_pin_mode = reply[10]
                return msg, trigger_pin_mode
            case '0f 20':
                msg = 'ADC parameters'
                length = 6
            case '12 02':
                msg = 'channel status'
                # channel = reply[7]
                channel_status = reply[10]
                return msg, channel_status
            case '12 20':
                msg = 'switching frequency'
                length = 4
            case '16 00':
                msg = 'serial number'
                length = 40
            case '18 20':
                msg = 'channel mode'
                channel_mode = reply[10]
                return msg, channel_mode
            case '22 20':
                msg = 'LUT value'
                length = 6
            case '25 20':
                msg = 'LUT parameters'
                length = 30
            case '30 20':
                msg = 'output status'
                length = 10
            case '42 20':
                msg = 'status update'
                length = 26
            case '52 02':
                msg = 'wheel lock status'
                lock_status = reply[10]
                return msg, lock_status
            case '81 00':
                msg = 'rich response' + reply[17:]
                length = 64
            case '82 20':
                msg = 'operating/display settings'
                length = 10
            case _:
                print('not a recognised msg ID:', mID)
                msg = ''
                length = 6
        
        # combine msg plus parameter (if more than 6 bytes)
        if length > 0:
            params = reply[18:18+(3*length-1)]
        else:
            params = ''
        return msg, params
    
    
    # --------------------------------------------------------------------------
    # CONTROLLER INFO/FUNCTIONS
    
    def identify(self):
        """flash display to indicate which controller is addressed"""
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['identify'])
    
    
    def get_serial(self):
        """get controller serial number"""
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['req_serial'])
        reply = self.recvreply()
        msg, params = self.decodereply(reply)
        serial_hex = params[12:24]  # serial number encoded in bytes 4..7 (starting at 0)
        serial_num = self.hexstr_to_int(serial_hex)
        if self.DEBUG:
            print('serial number: ', f"{serial_num:010d}")  # give out integer value
            print('raw reply:')
            print(reply)
            # yeah, apparently the preset serial number is always 75d...
        return serial_num
    
    
    def set_serial(self, new_SN:str = '12345678'):
        """set controller serial number"""
        # weird structure: Data0: 01 00 00 00, Data1: serial number
        if not self.port_is_open(): return
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
        old_SN = self.get_serial()
        print('previous SN :', old_SN)
        # convert serial number
        new_SN_hexstr = self.int_to_hexstr(int(new_SN), bytenum=4)
        cmd = f"{self.cmds['set_serial']} 01 00 00 00{new_SN_hexstr}" + ' 00'*32
        if self.DEBUG:
            print(cmd)
        self.sendcmd(cmd)
        set_SN = self.get_serial()
        print('newly set SN:', set_SN)
        print('hint: SNs are not persistent and will be reset to 75d when powering off')
    
    
    def get_info(self):
        """get hardware information, see APT protocol page 46"""
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['req_info'])
        reply = self.recvreply()
        msg, hwinfo = self.decodereply(reply)
        sn = self.hexstr_to_int(hwinfo[0:11]) # 4 byte serial number
        model_number = self.hexstr_to_ascii(hwinfo[12:35]) # 8 byte alphanumeric model number
        hw_type = self.hexstr_to_int(hwinfo[36:41]) # 2 byte describes type of hardware
        fw_minor = self.hexstr_to_int(hwinfo[42:44]) # minor firmware version (1 byte)
        fw_interim = self.hexstr_to_int(hwinfo[45:47]) # interim firmware version (1 byte)
        fw_major = self.hexstr_to_int(hwinfo[48:50]) # major firmware version (1 byte)
        # fw_reserved = self.hexstr_to_int(hwinfo[51:53]) # always 00
        hw_version = self.hexstr_to_int(hwinfo[-17:-12]) # 2 byte hardware version
        hw_mod_state = self.hexstr_to_int(hwinfo[-11:-6]) # 2 byte hardware modification state
        n_channels = self.hexstr_to_int(hwinfo[-5:]) # 2 byte number of channels
        print(f"serial number:\t\t{sn}\nmodel number:\t\t{model_number}\nfirmware version:\t{fw_major}.{fw_interim}.{fw_minor}")
        print(f"hardware type:\t\t{hw_type}\nhardware version:\t{hw_version}")
        print(f"modification state:\t{hw_mod_state}\nnumber of channels:\t{n_channels}")
    
    
    # --------------------------------------------------------------------------
    # HARDWARE CHANNEL OUTPUT CONTROL FUNCTIONS
    # enable/disable output, this needs to be combined with set_chan_mode
    # KLCs only have one channel so the argument is somewhat superfluous
    
    def get_hwchan_status(self, channel=1):
        """get hardware channel status"""
        if not self.port_is_open(): return
        if channel != 1: print('invalid channel, using channel 1'); channel = 1
        # chan_cmd = f"{self.cmds['req_HWchan_status'][0:7]}{channel}{self.cmds['req_HWchan_status'][8:]}"
        # self.sendcmd(chan_cmd)
        self.sendcmd(self.cmds['req_HWchan_status'])
        reply = self.recvreply()
        msg, channel_status = self.decodereply(reply)
        if channel_status == '1':
            print('channel ', channel, ' enabled')
        elif channel_status == '2':
            print('channel ', channel, ' disabled')
        else:
            print('no reply, channel status unknown')
        return channel_status
    
    
    def en_hwchan(self, channel=1):
        """enable hardware channel"""
        if not self.port_is_open(): return
        # chan_cmd = f"{self.cmds['enable_HWchan'][0:7]}{channel}{self.cmds['enable_HWchan'][8:]}"
        # self.sendcmd(chan_cmd)
        self.sendcmd(self.cmds['enable_HWchan'])
        self.get_hwchan_status(channel)
    
    
    def dis_hwchan(self, channel=1):
        """disable hardware channel"""
        if not self.port_is_open(): return
        # chan_cmd = f"{self.cmds['disable_HWchan'][0:7]}{channel}{self.cmds['disable_HWchan'][8:]}"
        # self.sendcmd(chan_cmd)
        self.sendcmd(self.cmds['disable_HWchan'])
        self.get_hwchan_status(channel)
    
    
    # --------------------------------------------------------------------------
    # HARDWARE WHEEL LOCK
    
    def lock_wheel(self):
        """lock device control wheel"""
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['lock_wheel'])
        self.get_wheel_status()
    
    
    def unlock_wheel(self):
        """unlock device control wheel"""
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['unlock_wheel'])
        self.get_wheel_status()
    
    
    def get_wheel_status(self):
        """"get device control wheel lock status"""
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['wheel_status'])
        reply = self.recvreply()
        msg, lock_status = self.decodereply(reply)
        if lock_status == '1':
            print('device wheel locked')
        elif lock_status == '2':
            print('device wheel unlocked')
        else:
            print('invalid lock status')
    
    
    # --------------------------------------------------------------------------
    # CONTROL SETTINGS
    
    def set_voltage(self, voltage:float, channel:int = 1):
        """set output voltage (0..25)"""
        if not self.port_is_open(): return
        if channel not in [1, 2]: print('invalid channel, select 1 or 2'); return
        # check if voltage is within the limits, otherwise set it to min/max
        if (voltage < 0) or (voltage > 25):
            print('invalid voltage: ', str(voltage))
            print('please set the voltage in V (0..25V)')
            return
        
        voltage_hex = self.int_to_hexstr(voltage*1000)  # input in mV
        cmd = f"{self.cmds['set_voltage']} 00 01 0{channel} 00 {voltage_hex}"
        self.sendcmd(cmd)
        if self.DEBUG: print('voltage set')
    
    
    def get_voltage(self, channel:int = 1):
        """get output voltage"""
        if not self.port_is_open(): return
        if channel not in [1, 2]: print('invalid channel, select 1 or 2'); return
        chan_cmd = f"{self.cmds['req_voltage'][0:10]}{channel}{self.cmds['req_voltage'][11:]}"
        self.sendcmd(chan_cmd)
        reply = self.recvreply()
        msg, params = self.decodereply(reply)
        # voltage just encoded in last two byte
        voltage = self.hexstr_to_int(params[-5:]) / 1000  # convert to V
        if self.DEBUG: print('set voltage:', voltage, 'V')
        return voltage
    
    
    def set_freq(self, frequency:int, channel:int = 1):
        """set output frequency (500..10000 Hz)"""
        if not self.port_is_open(): return
        if channel not in [1, 2]: print('invalid channel, select 1 or 2'); return
        # check if frequency is within the limits
        if (frequency < 500) or (frequency > 10000):
            print('invalid frequency: ', str(frequency))
            print('please set the frequency in Hz (500..10000)')
            return
        
        freq_hex = self.int_to_hexstr(frequency)  # input in Hz
        # cmd = self.cmds['set_frequency'] + ' 50 01 0' + str(channel) + ' 00 ' + freq_hex
        cmd = f"{self.cmds['set_frequency']} 00 01 0{channel} 00 {freq_hex}"
        self.sendcmd(cmd)
        if self.DEBUG: print('frequency set')
    
    
    def get_freq(self, channel:int = 1):
        """get output frequency"""
        if not self.port_is_open(): return
        if channel not in [1, 2]: print('invalid channel, select 1 or 2'); return
        chan_cmd = f"{self.cmds['req_frequency'][0:10]}{channel}{self.cmds['req_frequency'][11:]}"
        self.sendcmd(chan_cmd)
        reply = self.recvreply()
        msg, params = self.decodereply(reply)
        # frequency just encoded in last two byte
        frequency = self.hexstr_to_int(params[-5:])
        if self.DEBUG: print('set frequency:', frequency, 'Hz')
        return frequency
    
    
    # --------------------------------------------------------------------------
    # DEVICE TRIGGER PIN MODE
    
    def get_trigger_mode(self):
        """get device trigger pin mode"""
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['req_trigger_mode'])
        reply = self.recvreply()
        msg, trigger_mode = int(self.decodereply(reply))
        if trigger_mode == 1:
            print('trigger pin mode: Pin1 out, Pin2 out')
        elif trigger_mode == 2:
            print('trigger pin mode: Pin1 in, Pin2 out')
        elif trigger_mode == 3:
            print('trigger pin mode: Pin1 out, Pin2 in')
        else:
            print('invalid trigger_mode')
        return trigger_mode
    
    
    def set_trigger_mode(self, mode):
        """set device trigger pin mode
        mode x01: pin1 out  pin2 out
        mode x02: pin1 in   pin2 out
        mode x03: pin1 out  pin2 in
        """
        if not self.port_is_open(): return
        if (mode < 1) or (mode > 3):
            print('select a valid mode, meaning 1, 2 or 3')
            return
        trig_cmd = self.cmds['set_trigger_mode'][0:10] + str(mode) + self.cmds['set_trigger_mode'][11:]
        self.sendcmd(trig_cmd)
        self.get_trigger_mode()
    
    
    # --------------------------------------------------------------------------
    # ADC mode control
    
    def get_ADCinmode(self):
        """get ADC input mode"""
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['req_ADCinmode'])
        reply = self.recvreply()
        msg, ADCinmode = int(self.decodereply(reply))
        if ADCinmode:
            print('ADC input mode enabled')
        else:
            print('ADC input mode disabled')
        return ADCinmode
    
    
    def en_ADCinmode(self):
        """enable ADC input mode"""
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['enable_ADCinmode'])
        self.get_ADCinmode()
    
    
    def dis_ADCinmode(self):
        """disable ADC input mode"""
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['disable_ADCinmode'])
        self.get_ADCinmode()
    
    
    def get_ADCparams(self):
        """get ADC parameters"""
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['req_ADCparams'])
        reply = self.recvreply()
        msg, ADCparams = self.decodereply(reply)
        ADC_error = self.hexstr_to_int(ADCparams[6:11], signed=True)
        ADMAX_Value = self.hexstr_to_int(ADCparams[12:17], signed=False)
        print(f"ADC error:\t{ADC_error}")
        print(f"ADMAX value:\t{ADMAX_Value}")
        return ADC_error, ADMAX_Value
    
    
    # --------------------------------------------------------------------------
    # SWITCHING FREQUENCY SETTING
    # frequency to switch between V1 and V2 if CHANENABLESTATE = x03
    
    def set_swfreq(self, frequency):
        """set switching frequency (0.1..150 Hz)"""
        if not self.port_is_open(): return
        # check if switching frequency is within the limits
        if (frequency < 0.1) or (frequency > 150):
            print('invalid frequency: ', str(frequency))
            print('please set the switching frequency in Hz (0.1..150)')
            return
        
        freq_hex = self.int_to_hexstr(int(frequency*10))  # input is multiplied by 10!
        self.sendcmd(f"{self.cmds['set_swfreq']} 01 00{freq_hex}")
        print('switching frequency set to:', self.get_swfreq(), ' Hz')
        if self.DEBUG: print('switching frequency set to:', freq_hex)
    
    
    def get_swfreq(self):
        """get switching frequency in Hz"""
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['req_swfreq'])
        reply = self.recvreply()
        msg, params = self.decodereply(reply)
        # switching frequency just encoded in last two byte
        swfreq = self.hexstr_to_int(params[-5:]) / 10  # value is stored as 10xHz
        if self.DEBUG: print('set switching frequency:', swfreq, 'Hz')
        return swfreq
    
    
    # --------------------------------------------------------------------------
    # KLC CHANNEL OUTPUT CONTROL FUNCTIONS
    # controls the output mode, enable and disable output with en/dis_hwchan
    
    def get_chan_mode(self):
        """get channel mode"""
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['req_chan_mode'])
        reply = self.recvreply()
        msg, channel_mode = self.decodereply(reply)
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
    
    
    def set_chan_mode(self, mode=1):
        """set channel with specific output mode"""
        if not self.port_is_open(): return
        if mode == 0:
            mode_cmd = self.cmds['mode_chan_off']
        elif mode == 1:
            mode_cmd = self.cmds['mode_chan_V1']
        elif mode == 2:
            mode_cmd = self.cmds['mode_chan_V2']
        elif mode == 3:
            mode_cmd = self.cmds['mode_chan_sw']
        else:
            print('invalid mode, choose 0, 1, 2, or 3')
            return
        self.sendcmd(mode_cmd)
        self.get_chan_mode()
    
    
    # --------------------------------------------------------------------------
    # LUT OUTPUT
    
    def set_LUT_value(self, idx:int, voltage:int):
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
        
        voltage_hex = self.int_to_hexstr(voltage*1000)  # input in mV
        idx_hex = self.int_to_hexstr(idx)
        cmd = f"{self.cmds['set_LUT_value']} 01 00{idx_hex}{voltage_hex}"
        self.sendcmd(cmd)
    
    
    def get_LUT_value(self, idx:int):
        """get single LUT value at specified index
        the index is being set in the 4th byte which is only enough for 256 values
        no idea how to request the values from 256-511
        """
        if not self.port_is_open(): return
        # check if index is within limits
        if (idx < 0) or (idx > 511):
            print('invalid index:', idx)
            print('select index in [0..511]')
            return
        if idx > 255: print('sorry, havent found out how to read out addresses 256-511') ; return
        idx_hex = self.int_to_hexstr(idx, bytenum=1)
        cmd = f"{self.cmds['req_LUT_value'][0:8]}{idx_hex} {self.cmds['req_LUT_value'][12:]}"
        self.sendcmd(cmd)
        reply = self.recvreply()
        msg, params = self.decodereply(reply)
        voltage = self.hexstr_to_int(params[-5:]) / 1000 # voltage encoded in last two byte
        idx_read = self.hexstr_to_int(params[-11:-6]) # index encoded in the two before that
        print('LUT voltage at index', idx_read, ':', voltage, 'V')
        return voltage
    
    
    def set_LUT_params(self, mode=1, cycle_length=0, num_cycles=0, delay_time=0, pre_cycle_rest=0):
        """set LUT parameters
        mode: 0=disabled 1=continuous, 2=fixed number of cycles
        cycle_length: 1..512 (number of samples)
        num_cycles: 1..2147483648 (4 byte) (number of cycles to output the LUT if mode=2)
        delay_time: 1..2147483648 (4 byte) (time waiting after setting each output value)
        pre_cycle_rest (delay time before starting the LUT output)
        """
        mode_hex = self.int_to_hexstr(mode)
        cycle_length_hex = self.int_to_hexstr(cycle_length)
        num_cycles_hex = self.int_to_hexstr(num_cycles, bytenum=4)
        delay_time_hex = self.int_to_hexstr(delay_time, bytenum=4)
        pre_cycle_rest_hex = self.int_to_hexstr(pre_cycle_rest, bytenum=4)
        # exactly following the documentation including the reserved bytes:
        cmd = f"{self.cmds['set_LUT_params']} 01 00{mode_hex}{cycle_length_hex}{num_cycles_hex}{delay_time_hex}{pre_cycle_rest_hex} 0a" + " 00"*5 + " 01" + " 00"*5
        self.sendcmd(cmd)
    
    
    def get_LUT_params(self):
        """get LUT parameters
        mode: 1=continuous, 2=fixed number of cycles
        cycle_length: 1..512 (number of samples)
        num_cycles: 1..2147483648 (4 byte) (number of cycles to output the LUT if mode=2)
        delay_time: 1..2147483648 (4 byte) (time waiting after setting each output value)
        pre_cycle_rest (delay time before starting the LUT output)
        """
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['req_LUT_params'])
        reply = self.recvreply()
        msg, LUT_params = self.decodereply(reply)
        mode = int(LUT_params[7])
        cycle_length = self.hexstr_to_int(LUT_params[12:17])
        num_cycles = self.hexstr_to_int(LUT_params[18:29])
        delay_time = self.hexstr_to_int(LUT_params[30:41])
        pre_cycle_rest = self.hexstr_to_int(LUT_params[42:53])
        print(f"mode:\t\t{mode}\ncycle_length:\t{cycle_length}\nnum_cycles:\t{num_cycles}") 
        print(f"delay_time:\t{delay_time}\npre_cycle_rest:\t{pre_cycle_rest}")
    
    
    def start_LUT_output(self):
        """start LUT output"""
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['start_LUT_output'])
        self.get_hwchan_status()
        self.get_chan_mode()
        print('starting LUT output with:')
        self.get_LUT_params()
    
    
    def stop_LUT_output(self):
        """stop LUT output"""
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['stop_LUT_output'])
        self.get_hwchan_status()
        self.get_chan_mode()


    # --------------------------------------------------------------------------
    # CONFIGURE ALL OPERATING PARAMETERS TOGETHER
    
    def set_output_status(self, mode=1, voltage=5, frequency=1000, frequency_flag=1):
        """set device output status:
        mode: 0=disabled, 1=V1, 2=V2, 3=switching
        frequency_flag: 0=no change, 1=frequency changed (seems to not matter really)
        currently only sets V1 and f1, presumably one bit will change the channel
        """
        if not self.port_is_open(): return
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
        
        mode_hex = self.int_to_hexstr(mode)
        voltage_hex = self.int_to_hexstr(voltage*1000)
        frequency_hex = self.int_to_hexstr(frequency)
        freq_flag_hex = self.int_to_hexstr(frequency_flag)
        cmd = f"{self.cmds['set_output_status']} 01 00{mode_hex}{voltage_hex}{frequency_hex}{freq_flag_hex}"
        self.sendcmd(cmd)
        self.get_output_status()
        self.get_chan_mode()
    
    
    def get_output_status(self):
        """get device output status:
        output_active: 0 or 1
        error_flag: 1=DC offset error
        """
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['req_output_status'])
        reply = self.recvreply()
        msg, status = self.decodereply(reply)
        output_active = int(status[7])
        voltage = self.hexstr_to_int(status[12:17]) / 1000
        frequency = self.hexstr_to_int(status[18:23])
        error_flag = int(status[25])
        print(f"active:\t\t{output_active}\nerrors:\t\t{error_flag}")
        print(f"voltage:\t{voltage} V\nfrequency:\t{frequency} Hz")
    
    
    # --------------------------------------------------------------------------
    # CONFIGURE STATUS UPDATE
    
    def en_status_update(self):
        """enable status update when device panel is used to change parameters"""
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['enable_status_update'])
    
    
    def dis_status_update(self):
        """disable status update when device panel is used to change parameters"""
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['disable_status_update'])
    
    
    def get_status_update(self):
        """get status update
        chan_state: 1=V1, 2=V2, 3=Switching, 0=disabled
        ADCmode: 0=Analog in disabled, 1=enabled
        trig_conf: 1=Pin1/Pin2 Out, 2=Pin1 in/Pin2 out, 3=Pin1 out/Pin2 in
        wheel_status: 0=unlocked, 1=locked
        error_flag: most likely the second digit is the only "DC offset error" flag
        """
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['req_status_update'])
        reply = self.recvreply()
        msg, status = self.decodereply(reply)
        chan_mode = int(status[7])
        V1 = self.hexstr_to_int(status[12:17]) / 1000
        f1 = self.hexstr_to_int(status[18:23])
        V2 = self.hexstr_to_int(status[24:29]) / 1000
        f2 = self.hexstr_to_int(status[30:35])
        f_sw = self.hexstr_to_int(status[36:41]) / 10
        # disp_brightness = self.hexstr_to_int(status[42:47])
        # disp_timeout = self.hexstr_to_int(status[48:53], signed=True)
        # ADCmode = self.hexstr_to_int(status[54:59])
        # trig_conf = self.hexstr_to_int(status[60:65])
        # wheel_status = self.hexstr_to_int(status[66:71])
        # error_flag = self.hexstr_to_int(status[72:])
        print(f"output mode:\t{chan_mode}")
        print(f"V1={V1}V f1={f1}Hz\nV2={V2}V f2={f2}\nf_sw={f_sw}")
        return chan_mode, V1, f1, V2, f2, f_sw
    
    
    # --------------------------------------------------------------------------
    # DISPLAY PARAMETERS
    
    def get_disp_params(self):
        """get display settings (brightness/timeout)
        timeout multiplies with roughly 30s per value
        timeout=-1 means no timeout
        """
        if not self.port_is_open(): return
        self.sendcmd(self.cmds['req_kcube_params'])
        reply = self.recvreply()
        msg, disp_params = self.decodereply(reply)
        disp_brightness = self.hexstr_to_int(disp_params[6:11])
        disp_timeout = self.hexstr_to_int(disp_params[12:17], signed=True)
        print('display brightness: ', disp_brightness)
        if int(disp_timeout) == -1:
            print('display timeout: never')
        else:
            print('display timeout roughly: ', disp_timeout/2,  ' min')
        return disp_brightness, disp_timeout
    
    
    def set_disp_params(self, brightness=90, timeout=-1):
        """set display settings (brightness/timeout)
        to reset, just call without input values
        """
        if not self.port_is_open(): return
        if (brightness < 0) or (brightness > 100):
            print('please choose a brightness in 0..100')
            return
        if (timeout < -1) or (timeout > 480):
            print('please choose a timeout between 0..480 or -1')
        disp_brightness_hex = self.int_to_hexstr(brightness)
        disp_timeout_hex = self.int_to_hexstr(timeout)
        cmd = f"{self.cmds['set_kcube_params']} 01 00{disp_brightness_hex}{disp_timeout_hex} 00 00 00 00"
        self.sendcmd(cmd)
        set_brightness, set_timeout = self.get_disp_params()
    
    
    # --------------------------------------------------------------------------
    # EEPROM SETTINGS
    
    def save_params(self):
        """write parameters to eeprom
        does not save the serial number, sadly
        """
        if not self.port_is_open(): return
        print('are you sure you want to save the current parameters to EEPROM? (y/n)')
        userinput = input()
        if userinput == 'y':
            self.sendcmd(self.cmds['save_params'])
            print('saving parameters to EEPROM')
        else:
            print('aborted')
    
    
    def restore_factory_settings(self):
        """restore factory settings"""
        if not self.port_is_open(): return
        print('are you sure you want to restore factory settings? (y/n)')
        userinput = input()
        if userinput == 'y':
            print('you dont want to not keep the current settings? (y/n)')
            userinput = input()
            if userinput == 'n':
                self.sendcmd(self.cmds['restorefactset'])
                print('restoring factory settings')
                return
        print('aborted')
    

# EOF --------------------------------------------------------------------------