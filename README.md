# ESP USB Audio Example

This is an example for the ESP32-S3-BOX / ESP32-S3-BOX-3 to use their
microphones and speaker as a USB audio headset.

![CleanShot 2024-01-08 at 09 04 49](https://github.com/finger563/esp-usb-audio-example/assets/213467/81bbf450-f731-4dbf-bebc-ef8b0a1d6af2)
![CleanShot 2024-01-08 at 09 05 02](https://github.com/finger563/esp-usb-audio-example/assets/213467/e3d4e1c5-d714-44b2-abe9-7d833dc5ead9)

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

![CleanShot 2024-01-08 at 09 04 09](https://github.com/finger563/esp-usb-audio-example/assets/213467/404aaa16-4d92-4f99-ae2a-265623eb9d7c)

## Debugging

### MacOS

``` shell
system_profiler SPUSBDataType
```


``` shell
ioreg -p IOUSB -w0 -l
```
