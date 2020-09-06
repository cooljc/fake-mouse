# fake-mouse
This is a very simple program using the great Lightweight USB Framework 
for AVRs (http://www.lufa-lib.org) to emulate a USB mouse that moves several
points every 20 seconds. The purpose is to prevent a screen saver kicking in
when you do not have access to settings to prevent it (corporate IT!).

The device being targeted is the Arduino Pro Micro. You can change this
in the src/Makefile. Further work might be needed to get LED etc working.

----
# Prerequisites
To build this you will need GNU make, AVR gcc, avr-libc and avrdude.

## Ubuntu
To get the required packages on an Ubuntu system use the following:
```
sudo apt-get install gcc-avr avr-libc avrdude
```

Your user will also need to be a member of the dialout group to be able to 
flash the AVR using avrdude. You can add yourself to this group with the 
following command. Remember to logout/login for group changes to take effect.
```
sudo usermod -a -G dialout [username]
```

## Other systems
I am a Linux user and have not tried building this on MAC or Windows.
So your on your own here sorry.

----
# Building the source
To build the source change into the src directory and run make
```
cd src
make
```

You can install/falsh the firmware to your device using the following command:
```
make avrdude
```

If flashing fails please try exporting the following environment variable:
```
export AVRDUDE_PORT = /dev/ttyACM0
```

