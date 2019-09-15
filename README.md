# Open-Nuki-Fob
Open Source Nuki Fob based on the the NRF51822 SoC

Special thanks to the Nordic repo of [@pcbreflux](https://github.com/pcbreflux/nordic/), which this project is based on.


## Prequisits
The gcc-arm compilers for the NRF51822, openocd and the nRF5 SDK 12.3.0.

Set SDK_PATH in Makefile.common to your local nRF5 SDK directory.

## Build
1. Use "make all" to build the fob firmware hex file.
2. Use "make flash_s130" to flash the required softdevice.
3. Use "make flash" to flash the fob firmware.

## Usage

### Pairing
To pair the fob to a lock, bring the lock into pairing mode by holding the button on the Nuki for 5 seconds. The lock is in pairing mode if the ring light is permanently glowing. Now hold the button on your fob down for 10 seconds. The LED should light permanently and after about 30-40 seconds the pairing should be finished. Check the Nuki app to see if the fob is showing up in the users list as "Open Nuki Fob XXXXXXXX".

### Unlocking
Pressing the button once will cause the fob to send a "Fob Action 1" command, double press will send "Fob Action 2" and triple press sends "Fob Action 3", all of which can be specified in the Nuki app. The LED on the fob will blink and it will try to connect to the Nuki lock. Upon successful connection, the fob will send the command. Afterwards, the fob will enter sleep mode. If the fob was unable to send the command within 90 seconds, it will also enter sleep mode.

## Hardware
I used a ST Link v2 to flash the firmware to the NRF51822. For the fob, I use cheap NRF51822 based BLE Beacons by Shenzhen Radioland Technology, which already come with a battery slot, button, led and keychain case. The build directory contains a prebuilt hex file that you can flash on the afformentioned Beacon.


## TODO

The Nuki specific routines could theoretically be used on other hardware (e.g. the ESP32) after a bit of cleanup.
