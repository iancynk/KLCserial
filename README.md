# KLCserial
Functions and examples to control Thorlabs KLC controllers (KLC101) through serial commands with Python.

The official software can be found on the [Supplier Download Site](https://www.thorlabs.de/software_pages/ViewSoftwarePage.cfm?Code=KLC101). 
Documentation is also available through the same website. The serial commands have been partially assembled from the [Thorlabs Motion Controllers Host-Controller Communications Protocol](https://www.thorlabs.com/Software/Motion%20Control/APT_Communications_Protocol.pdf).

There is no specific documentation for the KLC101 so I assembled commands from other stages.

## Requirements
Needs `serial` and `pyserial` (both of them are needed for this to work somehow), best installed through:
```
pip install serial pyserial
```


## Usage

Simply run
```
pip install KLCserial
```
Then in your Python script invoke the functions by
```
from KLCserial import KLC
klc = KLC()
```
With `klc` you can then call all functions and methods of the controllers.

Look at [example.py](example.py). There are a few more commands implemented than shown in the example but they appear not to be too helpful for most applications.


## Serial ports
By default the `openKLC()`-function will use all the ports found under `/dev/serial/by-id/*Kinesis_LC_Controller*` (Windows: COM[0-255], *not tested*) and try to connect through serial.

You need write-access to the tty-port. Either run as the admin or change the permissions or the ownership on the serial port, preferably with a udev rule, e.g. something like
`/etc/udev/rules.d/80-usb-serial.rules`
```
SUBSYSTEM=="tty", MODE="0666"
```
(After changing run as root: `udevadm control --reload-rules && udevadm trigger`)

## License
This project is licensed under the MIT license.

## Acknowledgement
You're welcome!

Comments and suggestions highly welcome.