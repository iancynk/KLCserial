#!/bin/usr/python
# -----------------------------------------------------------------------------
# This code gives basic examples how to use the KLCserial function
# scripted by ian cynk (ian.cynk@posteo.eu) 2023
# %% --------------------------------------------------------------------------
# imports
import KLCserial as klc

# %% --------------------------------------------------------------------------
# %%
# connect controller
c = klc.openKLC()
# SN = '39342867'
# c = klc.openKLC(SN=SN)  # connect controller with specified serial number
# c = klc.openKLC('/dev/ttyUSB0')  # connect controller with specified port

# let the display flash
klc.identify(c)

# %% --------------------------------------------------------------------------
# get information

# get serial number
klc.get_serial(c)

# get/set display parameters
klc.get_disp_params(c)
klc.set_disp_params(c, brightness=10, timeout=-1)
# brightness: 0..100, timeout: 0..480 or -1 (never)

# get set frequency
klc.get_freq(c)
# klc.get_freq(c, 2)  # channel 2

# get set voltage
klc.get_voltage(c)
# klc.get_voltage(c, 2)  # channel 2

# get channel status
klc.get_chan_status(c)
# klc.get_chan_status(c, 2)  # channel 2

# %% --------------------------------------------------------------------------
# set output

# set voltage to 13.37 V
klc.set_voltage(c, 13.37)
# klc.set_voltage(c, 13.37, 2)  # channel 2

# set frequency to 1337 Hz
klc.set_freq(c, 1337)
# klc.set_freq(c, 1337, 2)  # channel 2

# enable/disable output
klc.en_chan(c, 1)
klc.dis_chan(c, 1)

# %% --------------------------------------------------------------------------
# disconnect controller
klc.disconnect(c)
klc.closeKLC(c)
