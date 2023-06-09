# KLCserial
Functions and examples to control Thorlabs KLC controllers through serial commands with Python.

The official software can be found on the [Supplier Download Site](https://www.thorlabs.de/software_pages/ViewSoftwarePage.cfm?Code=KLC101)

Documentation is also available through the same website. The serial commands have been partially assembled from the [Thorlabs Motion Controllers Host-Controller Communications Protocol](https://www.thorlabs.com/Software/Motion%20Control/APT_Communications_Protocol.pdf).

There is no specific documentation for the KLC101 so I assembled commands from other stages.

## Requirements
Needs `serial` and `pyserial` (both of them are needed for this to work somehow), best installed through:
```
pip install serial pyserial
```


## Usage
Download `KLCserial.py` and put it in your working directory. 

Then look at [example.py](example.py). There are a few more commands implemented than shown in the example but they appear not to be too helpful for most applications.

This is under development but I hope it will be helpful for fellow users. Some command are not implemented yet, specifically:

* OUTPUTLUT
* OUTPUTLUTPARAMS
* START/STOPLUTOUTPUT
* STATUSUPDATE
* KCUBSWFREQ

Comments and suggestions highly welcome.

## Serial ports
By default the `openKLC()`-function will use all the ports found under `/dev/serial/by-id/*Kinesis_LC_Controller*` (Windows: COM[0-255]) and try to connect through serial.

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
