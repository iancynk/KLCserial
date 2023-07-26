#!/bin/usr/python
# -----------------------------------------------------------------------------
# This code gives basic examples how to use the KLCserial function
# scripted by ian cynk (ian.cynk@posteo.eu) 2023
# %% --------------------------------------------------------------------------
# imports
import KLCserial as klc
import time

# %% --------------------------------------------------------------------------
# %%
# connect controller
c = klc.openKLC()
# SN = '39123456'
# c = klc.openKLC(SN=SN)  # connect controller with specified serial number
# c = klc.openKLC('/dev/ttyUSB0')  # connect controller with specified port

# let the display flash
klc.identify(c)

# %% --------------------------------------------------------------------------
# get information
klc.get_info(c)

#%%
# change serial number (not persistent)
klc.set_serial(c, SN)

#%%
klc.save_params(c)
#%%
# get/set display parameters
klc.get_disp_params(c)
klc.set_disp_params(c, brightness=90, timeout=-1)
# brightness: 0..100, timeout: 0..480 or -1 (never)
# timeout roughly multiplies by 30s

# %%
# get set frequency
klc.get_freq(c)  # preset 1
# klc.get_freq(c, 2)  # preset 2

# get set voltage
klc.get_voltage(c)  # preset 1
# klc.get_voltage(c, 2)  # preset 2

# get channel status
klc.get_hwchan_status(c)
klc.get_chan_mode(c)

# %% --------------------------------------------------------------------------
# set output

# set voltage to 13.37 V
klc.set_voltage(c, 13.37)  # preset 1
# klc.set_voltage(c, 13.37, 2)  # preset 2

# set frequency to 1337 Hz
klc.set_freq(c, 1337)  # preset 1
# klc.set_freq(c, 1337, 2)  # preset 2

# set switching frequency to 21 Hz
klc.set_swfreq(c, 21)

# enable output
klc.en_hwchan(c)

# change output mode
klc.set_chan_mode(c, mode=1)  # preset 1
klc.set_chan_mode(c, mode=2)  # preset 2
klc.set_chan_mode(c, mode=3)  # switching between presets at switching frequency

# enable/disable output
klc.dis_hwchan(c)

# %% --------------------------------------------------------------------------
# disconnect controller
klc.closeKLC(c)
