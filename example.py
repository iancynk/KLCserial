#!/bin/usr/python
# -----------------------------------------------------------------------------
# This code gives basic examples how to use the KLCserial function
# scripted by ian cynk (ian.cynk@posteo.eu) 2023
# %% --------------------------------------------------------------------------
# imports
from KLCserial import KLC
import time

# %% --------------------------------------------------------------------------
# connect controller

c = KLC()
SN = '39123456'
# c = KLC(SN=SN)  # connect controller with specified serial number
# c = KLC(port='/dev/ttyUSB0')  # connect controller with specified port

# let the display flash
c.identify()

# %% --------------------------------------------------------------------------
# get information
c.get_info()

#%%
# change serial number (not persistent)
c.set_serial(SN)

#%%
# get/set display parameters
c.get_disp_params()
c.set_disp_params(brightness=90, timeout=-1)
# brightness: 0..100, timeout: 0..480 or -1 (never)
# timeout roughly multiplies by 30s

# %%
# get set frequency
c.get_freq()  # preset 1
# c.get_freq(2)  # preset 2

# get set voltage
c.get_voltage()  # preset 1
# c.get_voltage(2)  # preset 2

# get channel status
c.get_hwchan_status()
c.get_chan_mode()

# %% --------------------------------------------------------------------------
# set output

# set voltage to 13.37 V
c.set_voltage(13.37)  # preset 1
# c.set_voltage(13.37, 2)  # preset 2

# set frequency to 1337 Hz
c.set_freq(1337)  # preset 1
# c.set_freq(1337, 2)  # preset 2

# set switching frequency to 21 Hz
c.set_swfreq(21)

# enable output
c.en_hwchan()

# change output mode
c.set_chan_mode(mode=1)  # preset 1
c.set_chan_mode(mode=2)  # preset 2
c.set_chan_mode(mode=3)  # switching between presets at switching frequency

# enable/disable output
c.dis_hwchan()

# %%
# save parameters (voltage/frequency)
c.save_params()
# %%
# restore factory settings
c.restore_factory_settings()

# %% --------------------------------------------------------------------------
# disconnect controller
c.closeKLC()
