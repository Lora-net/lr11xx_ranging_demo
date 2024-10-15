# LR11XX ranging demonstration code

## Description

The sample code is used to perform a sub-GHz or 2.4G band based on Round-Trip Time of Flight (ranging) measurement between two devices : a ranging manager and a ranging subordinate. 

This sample runs in two phases: initialization with LoRa type and ranging process with RTToF type. At initialization phase, the manager and subordinate will exchange some information, such as number of frequency hopping, with the unique address by using LoRa mode. After successful initialization, they will do `RANGING_HOPPING_CHANNELS_MAX` times ranging process with a fixed frequency table for hopping by using RTToF mode. Then, the ranging results with JSON format will be printed on the manager terminal.


This example can only be used with LR1110 and LR1120(as they have RTToF feature).

## Getting started

### Configure

Before starting to build the example, check the parameters in both the common and the example-specific configuration file. 

The common parameters can be found in [lr11xx/common/apps_configuration.h](lr11xx/common/apps_configuration.h), refer to [lr11xx/common/README.md](lr11xx/common/README.md) for more details. If you want to set the frequency for LoRa, modify the macro definition - `RF_FREQ_IN_HZ`. But if you want to set the ranging channels for frequency hopping, modify the constant array - `ranging_hopping_channels_array[]` in the [app_ranging_hopping.c](lr11xx/apps/ranging_demo/ranging_hopping_frequency/app_ranging_hopping.c) file. Note also that macro `PACKET_TYPE`,  FSK and Sigfox related configuration items have no effect on this sample code. 

The example specific configuration file can be found in [lr11xx/apps/ranging_demo/main_ranging_demo.h](lr11xx/apps/ranging_demo/main_ranging_demo.h). There is a macro parameter that should be configured before starting this project.

- `RANGING_DEVICE_MODE`: Determine whether the device operates as a ranging manager (sends RTToF requests) or as a ranging subordinate (processes RTToF requests and answers by sending RTToF responses).

#### Notes

- Spreading factors from SF5 to SF10 are recommended values.

- Bandwidth shall be chosen among `LR11XX_RADIO_LORA_BW_125`, `LR11XX_RADIO_LORA_BW_250` and `LR11XX_RADIO_LORA_BW_500` for both sub-GHz and 2.4G bands.

- The preamble length, `LORA_PREAMBLE_LENGTH`, should be set as 12, because it is related to the timing of ranging process.

- Please set the IQ value - `LORA_IQ`, as the standard mode - `LR11XX_RADIO_LORA_IQ_STANDARD`. Because all the calibration values are based on the standard mode. It might reduce the ranging accuracy if used the inverted mode.

### Build

#### Keil MDK ARM

This example is delivered with a Keil project file - see `lr11xx/apps/ranging_demo/MDK-ARM/ranging_demo.uvprojx`.

To build a project:

1. Launch Keil IDE
2. Open the project file
3. Select the target
4. Compile

This project has different targets ([Keil manual](https://www.keil.com/support/man/docs/uv4/uv4_ca_projtargfilegr.htm)), each one allowing to choose the shield the example is compiled for.

The name of the targets is taken from the column `shield` of the supported shields table available in the chip family READMEs:
- for LR11XX:  [here](lr11xx/README.md#supported-shields).

#### GNU Arm embedded toolchain

This example is built from its subfolder in the `lr11xx/apps/ranging_demo/makefile`.

Build settings, compile time and configuration options are specified in the project's Makefile.

The output files of the build process are stored in the `build` folder with firmware binary file having the same name as the project with a .bin extension.

Here are the parameters available at compile time:

| Parameter    | Description                              | Default value            |
| ------------ | ---------------------------------------- | ------------------------ |
| RADIO_SHIELD | Shield for which the example is compiled | **lr11xx**: LR1110MB1DIS |

For instance, to build the project with LR1110MB1GJS shield simply run make as follows

```shell
$ cd $SDK_FOLDER/lr11xx/apps/ranging_demo/makefile
$ make RADIO_SHIELD=LR1110MB1GJS
```

##### Command line configuration

Additional configuration flags can be passed from command line to compiler with `EXTRAFLAGS` argument.
This is dedicated to define macros that can be defined like the following:

```bash
$ make EXTRAFLAGS='-D<MACRO>=<VALUE>'
```

Where `<MACRO>` is the macro name to set and `<VALUE>` is the value to set for this macro.
Not all macro can be redefined through this way. Refer to the README of examples for the list of macro that can be redefined.

Note that when using the configuration on command line, `make` cannot detect a change in configuration on next build.
Therefore `make clean` must be invoked before calling a new `make` with a different configuration.

### Load

After a project is built, it can be loaded onto a device.

There are multiple ways to do it, among which:

* Drag and drop the binary file to the USB drive listed by our OS - usually shows up as `NODE_L476RG`.
* Load it through the Keil IDE.

### View debug output

On the NUCLEO-L476RG development board, the firmware prints debug information to the UART that is connected via the ST-LINK to the host computer. The configuration is 921600/8-N-1:

* On Linux, this device usually shows up as `/dev/ttyACM0`.
* On Windows, the port can be obtained from the device manager.

For instance, using stty on Linux with a device available in `/dev/ttyACM0`:

```shell
$ stty -echo raw speed 921600 < /dev/ttyACM0 && cat /dev/ttyACM0
```

## Supported toolchains

This example can be compiled with the following toolchains:

* [Keil MDK ARM](https://www2.keil.com/mdk5) - Keil project file available in `lr11xx/apps/ranging_demo/MDK-ARM/`.
* [GNU Arm Embedded toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm) - makefile available in `lr11xx/apps/ranging_demo/makefile`.

## Remarks

1. In order to print JSON format for ranging result, the application code needs to be defined the macro `PERF_TEST_ENABLED` (defined by default) to use the `HAL_PERF_TEST_TRACE_PRINTF()` as the print function. At the same time, functions such as `HAL_DBG_TRACE_INFO()`, `HAL_DBG_TRACE_WARNING()`, etc., are useless.

2. You can use a display to show some information when you test. You need to uncomment the macro - `RANGING_DISPLAY_FOR_TEST` in the file - `lr11xx/apps/ranging_demo/main_ranging_demo.h` if you want to. And by using display mode, you can also use the user (blue) button to go into relative range mode. You can get the relative distance based on the location where you push the user button. If you want to come back to normal mode, push the reset (black) button.
