# ESP USB Audio Example

This is an example for the ESP32-S3-BOX / ESP32-S3-BOX-3 to use their
microphones and speaker as a USB audio headset.

## Cloning

Since this repo contains a submodule, you need to make sure you clone it
recursively, e.g. with:

``` sh
git clone --recurse-submodules git@github.com:finger563/esp-usb-audio-example
```

Alternatively, you can always ensure the submodules are up to date after cloning
(or if you forgot to clone recursively) by running:

``` sh
git submodule update --init --recursive
```

## Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Output


## Debugging

### MacOS

``` shell
system_profiler SPUSBDataType
```


``` shell
ioreg -p IOUSB -w0 -l
```
