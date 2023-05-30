#!/bin/usr/python
# -----------------------------------------------------------------------------
# This code gives basic examples how to use the KLCserial function
# scripted by ian cynk (ian.cynk@posteo.eu) 2023
# %% --------------------------------------------------------------------------
# imports
import KLCserial as k

# %% --------------------------------------------------------------------------
# %%
# connect stage
s = k.openKLC()
# s = k.openKLC('/dev/ttyUSB0')  # connect stage with specified port

# let the display flash
k.identify(s)

# %% --------------------------------------------------------------------------
# get information

# get serial number
k.get_serial(s)

# get/set display parameters
k.get_disp_params(s)
k.set_disp_params(s, brightness=10, timeout=-1)
# brightness: 0..100, timeout: 0..480 or -1 (never)

# get set frequency
k.get_freq(s)
# k.get_freq(s, 2)  # channel 2

# get set voltage
k.get_voltage(s)
# k.get_voltage(s, 2)  # channel 2

# get channel status
k.get_chan_status(s)
# k.get_chan_status(s, 2)  # channel 2

# %% --------------------------------------------------------------------------
# set output

# set voltage to 13.37 V
k.set_voltage(s, 13.37)
# k.set_voltage(s, 13.37, 2)  # channel 2

# set frequency to 1337 Hz
k.set_freq(s, 1337)
# k.set_freq(s, 1337, 2)  # channel 2

# enable/disable output
k.en_chan(s, 1)
k.dis_chan(s, 1)

# %% --------------------------------------------------------------------------
# disconnect stage
k.disconnect(s)
k.closeKLC(s)
