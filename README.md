# Edge Impulse Firmware for Sony's Spresense

Edge Impulse enables developers to create the next generation of intelligent device solutions with embedded machine learning. This repository contains the Edge Impulse firmware for Sony's Spresense development board. This device supports all of Edge Impulse's device features, including ingestion, remote management and inferencing.

> **Note:** Do you just want to use this development board to collect data with Edge Impulse? No need to build this firmware. View the [getting started guide](https://docs.edgeimpulse.com/docs/sony-spresense) for a pre-built firmware image and flashing instructions. Or, you can use the [data forwarder](https://docs.edgeimpulse.com/docs/cli-data-forwarder) to capture data from any sensor.

## Requirements

### Hardware

* [Sony's Spresense Main board](https://developer.sony.com/develop/spresense/buy-now).

#### Optional add-ons:

* [Spresense Extension board](https://eu.mouser.com/ProductDetail/Sony-Spresense/CXD5602PWBEXT1_FG_875607608_P?qs=sGAEpiMZZMu3sxpa5v1qrpe%2F9%2FddSq0jgeEkn3phnwE=) - to connect external sensors.
* [CXD5602PWBCAM1 camera add-on](https://nl.mouser.com/ProductDetail/Sony-Spresense/CXD5602PWBCAM1_FG_875607605_P?qs=sGAEpiMZZMu3sxpa5v1qrpe%2F9%2FddSq0jwhSPmsfM3%252Bc%3D) - if you want to add sight to your Spresense.
* [Spresense Sensor EVK-70 add-on with an accelerometer](https://www.chip1stop.com/USA/en/view/dispDetail/DispDetail?partId=ROHM-0170579&cid=c1s_sony_spresense_SPRESENSE-SENSOR-EVK-701) - if you plan on building models that depend on motion.
* [Spresense Wi-Fi / iS110B module](https://www.chip1stop.com/product/detail?partId=IDYC-0000001&cid=c1s_sony_spresense_wifi) - for wireless connectivity.

### Software

* [Edge Impulse CLI](https://docs.edgeimpulse.com/docs/cli-installation).  
* [GNU Make](https://www.gnu.org/software/make/).  
* [GNU ARM Embedded Toolchain 9-2019-q4-major](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) - make sure `arm-none-eabi-gcc` is in your PATH.
* [Python 3](https://www.python.org/download/releases/3.0/).

## Building and flashing the application

1. Run `make` from the root folder to build the project:
    ```
    $ make -j
    ```
1. Connect the board to your computer using USB.  
1. Flash the board:  
    ```
    $ make flash
    ```

### Build and flash with Docker

1. Build the Docker image:
    ```
    $ docker build -t spresense-build .
    ```
1. Build the application by running the container as follows:  
    **Windows**
    ```
    $ docker run --rm -it -v "%cd%":/app spresense-build /bin/bash -c "make -j"
    ```
    **Linux, macOS**
    ```
    $ docker run --rm -it -v $PWD:/app:delegated spresense-build /bin/bash -c "make -j"
    ```
1. Connect the board to your computer using USB.  
1. Flash the board:
    ```
    $ make flash
    ```
    Or if you don't have `make` installed:
    ```
    $ tools/flash_writer.py -s -d -b 115200 -n build/firmware.spk
    ```

## Connecting to the board

### Edge Impulse Studio

You can use the `edge-impulse-daemon` to connect with Edge Impulse studio. Documentation can be found [here](https://docs.edgeimpulse.com/docs/cli-daemon).

### Running your impulse

The `edge-impulse-run-impulse` tool is used to start and run the impulse on your board. Documentation can be found [here](https://docs.edgeimpulse.com/docs/cli-run-impulse).

## WARNING

The nuttx stdint.h defines int32 as unsigned long, whereas the stdlib.h that ships with ARM GCC defines int32 as unsigned int.  These are the same size (https://developer.arm.com/documentation/dui0472/k/C-and-C---Implementation-Details/Basic-data-types-in-ARM-C-and-C--), so from a stack perspective, it doesn't matter, but a C++ linker will treat a different in int32 as a function overload (so you'll get a missing function error from the linker if you're not careful)

### Notes
- The video driver doesn't like to change resolutions after being opened.  This prevents lazy initialization and using the same handle for the entire life of program.
 - Thus, we open and close on entering and exiting snapshot stream, and on each snapshot for ingestion or inference

## Tips

### Override the ARM GNU Toolchain

If you have multiple toolchains installed, you can override the compiler via:

```
$ CROSS_COMPILE=~/toolchains/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi- make -j
```
