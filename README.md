# Open-Nuki-Fob
Open Source Nuki Fob based on the the NRF52832 SoC

Video of the fob in action: [Link](https://www.youtube.com/watch?v=eYI9Il37lno)

## Prequisits
The gcc-arm compilers for the NRF528322, openocd (the latest release with NRF52 support) and the nRF5 SDK 17.x.x.

Edit the Makefile and set
- SDK_ROOT: the directory of the NRF SDK
- OPENOCD_HOME: the base directory of openocd which contains the tcl files
- OPENOCD: the openocd binoary
- NRFUTIL: the nrfutil binary to generate the bootloader settings
- MERGEHEX: the mergehex binary to create a firmware file containing the fob code, bootloader and softdevice

## Build
1. Use "make" to build the fob firmware hex file.
2. Use "make flash" to flash the softdevice & firmware.

## Usage

### Pairing
To pair the fob to a lock, bring the lock into pairing mode by holding the button on the Nuki for 5 seconds. The lock is in pairing mode if the ring light is permanently glowing. Now hold the button on your fob down for 5 seconds. The LED should light permanently and after about 10 seconds the pairing should be finished. Check the Nuki app to see if the fob is showing up in the users list as "Open Nuki Fob XXXXXXXX".

### Unlocking
Pressing the button once will cause the fob to send a "Fob Action 1" command, double press will send "Fob Action 2" and triple press sends "Fob Action 3", all of which can be specified in the Nuki app. The LED on the fob will blink and it will try to connect to the Nuki lock. Upon successful connection, the fob will send the command. Afterwards, the fob will enter sleep mode. If the fob was unable to send the command within 30 seconds, it will also enter sleep mode. Low battery will be indicated by 3 short LED flashes instead of one;

### DFU Mode
To enter DFU mode, press the button on the fob 5 times and hold it down on the 5th press. The LED should indicate DFU mode by repeatedly flashing the LED in a long, long, short short pattern. The Fob can be updated with the nRF Toolbox on iOS and Android or nRF Connect on Windows.

## Hardware
I used a ST Link v2 to flash the firmware to the NRF52832. For the fob, I use cheap NRF52832 based BLE Beacons by Radioland China, which already come with a battery slot, button, led and keychain case. The build directory contains a prebuilt hex file that you can flash on the afformentioned Beacon.