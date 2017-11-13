## Announcements ##
**API CHANGES!** Before you start to write an app check the new API!
**Known bugs!** Check the [CHANGES.md](CHANGES.md)
**All tests are done only the G variant of the sensor** Please, check your HW variant.

## Overview ##

Movesense sensor is a programmable multi-purpose device comprising accelerometer, gyroscope, magnetometer and thermometer together with optional heartrate/IBI and intelligent gear ID APIs. The APIs follow the well known REST principle and can be used both internally by the partner app or externally via the iOS/Android libraries. API specification ([link to folder](https://bitbucket.org/suunto/movesense-device-lib/src/master/MovesenseCoreLib/resources/movesense-api/)), is based on Swagger 2.0 syntax.

You can order the Movesense sensor and Movesense Developer Kit on our [shop](https://www.movesense.com/shop/).

Check also the mobile (Android and iOS) part for the mobile applications development.
[Movesense mobile lib](https://bitbucket.org/suunto/movesense-mobile-lib)
Note: If you do not have access, please contact us. 


## API ##
Resource | Description|Implemented
---------|------------|--------------
/Comm/Ble|API for managing BLE | YES
/Component/Eeprom|API for writing and reading the EEPROM memory/ies. | YES
/Info|API for accessing generic device information.| YES
/Meas/Acc|API enabling subscribing linear acceleration data (based on accelerometer).| YES
/Meas/ECG|API for the electrocardiography measurement.| YES
/Meas/Gyro|API enabling subscribing angular velocity data (based on gyroscope).| YES
/Meas/HR|API enabling subscribing heart rate data.| YES
/Meas/Magn|API enabling subscribing magnetic field data (based on magnetometer).| YES
/Meas/Temp|API enabling reading or subscribing temperature data (based on thermometer).| YES
/Mem/DataLogger|Generic logger capable of logging max. 8 different resources| YES
/Mem/Logbook|Generic Logbook from where the logged data can be read| YES
/Misc/Gear| API to get the globally unique Movesense ID associated with the physical gear | YES
/System/Energy|API for reading the battery state.| YES
/System/Mode|API for controlling the main system state (e.g. factory sleep).| YES
/System/Settings| Settings API for a Movesense device. | YES
/Time|API for accessing different time related services.| YES
/Ui/Ind|API for controlling the LED.| YES

## Documentation ##
In this document you can find only the most important informations. Please check the [documentation](https://bitbucket.org/suunto/movesense-docs/wiki/Home)

## Setup the enviroment ##

### Windows ###
Install tools:
* [GNU Toolchain for ARM Embedded 6-2017-q2](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)
  * Remember to add it to the system path by the checkbox on the last step of installation.
* [Ninja build tool](https://ninja-build.org/)
  * Download the file
    * Put ninja.exe file to C:/bin
    * Add C:/bin to the system env. PATH variable
* [cmake >=3.6](https://cmake.org/download/)
  * Download cmake-3.8.2-win64-x64.zip
  * Extract the file
  * Add bin dir (with cmake) to the system env. PATH variable
* [nrfutil package & python 2.7 (for OTA firmware update package creation)](https://github.com/NordicSemiconductor/pc-nrfutil)
  * Nordic provides precompiled Windows executable
* [Visual Studio Redistributable 2015](https://www.microsoft.com/en-us/download/details.aspx?id=48145)

Optionally: 

* For flashing using JIG:
  * Nordic Semiconductor's [nRF5x-Command-Line-Tools-XXX](https://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF52-DK)
* In case you encounter `ImportError: No module named yaml`, try to install package `pyyaml` by pip

### OSX ###
Movesense development is supported on OSX - detailed installation instructions will be added here shortly.

### Linux ###
Movesense development is supported on Linux - detailed installation instructions will be added here shortly.

## Example App Build Flow ##

```
#!python
> git clone git@bitbucket.org:suunto/movesense-device-lib.git
> cd movesense-device-lib
> mkdir myBuild
> cd myBuild

> cmake -G Ninja -DMOVESENSE_CORE_LIBRARY=../MovesenseCoreLib/ -DCMAKE_TOOLCHAIN_FILE=../MovesenseCoreLib/toolchain/gcc-nrf52.cmake ../samples/hello_world_app
or
> cmake -G Ninja -DMOVESENSE_CORE_LIBRARY=../MovesenseCoreLib/ -DCMAKE_TOOLCHAIN_FILE=../MovesenseCoreLib/toolchain/gcc-nrf52.cmake ../samples/accelerometer_app
or
> cmake -G Ninja -DMOVESENSE_CORE_LIBRARY=../MovesenseCoreLib/ -DCMAKE_TOOLCHAIN_FILE=../MovesenseCoreLib/toolchain/gcc-nrf52.cmake ../samples/blinky_app
or release version
> cmake -G Ninja -DMOVESENSE_CORE_LIBRARY=../MovesenseCoreLib/ -DCMAKE_TOOLCHAIN_FILE=../MovesenseCoreLib/toolchain/gcc-nrf52.cmake -DCMAKE_BUILD_TYPE=Release ../samples/hello_world_app

> ninja
```
*Note: As default, the SERIAL interface can be enabled (Check App.cpp file). If flashed/used without the JIG, remember to disable it to avoid excessive power consumption ([link to instructions](https://bitbucket.org/suunto/movesense-device-lib/src/master/MovesenseCoreLib/documentation/PowerOptimization.md?at=master&fileviewer=file-view-default))*

### Updating sensor "over the air" ###

If you want to create OTA firmware update package run command
```
ninja dfupkg
```

which places the OTA package with name *movesense_dfu.zip* in the build folder.

### Updating sensor with programming jig ###

if you are using a *Movesense programming JIG* you can flash the resulting .hex-file (and the nRF52 SoftDevice) with:
```
ninja flash
```
or you can use any other nRF52 compatible programming software such as *nRFGo Studio*.

### Remobving manufacturing/settings data from the sensor ###
This command additionaly erase the whole memory. You should use it only if you want to add own Manufacturing/calibration data.
Please, contact us first.
```
ninja flash_all
```

## What's New ##
For information about releases, see the [CHANGES.md](CHANGES.md) file in repository.


## Frequently Asked Questions ##
Check [FAQ.md](FAQ.md) also some answers you can find on the stackoverflow.
Please check http://stackoverflow.com/search?q=movesense

If your problem is not addressed there, please [post](http://stackoverflow.com/questions/ask) a new question, and tag it with 'movesense' (i.e. include [tag:movesense] in the question body).

## Bug Reports ##

Please report bugs by [raising](https://bitbucket.org/suunto/movesense-device-lib/issues/new) an Issue via Bitbucket.

## Contributions ##
Your input is appreciated and we encourage you to post your contributions as pull requests to this repository.

## License ##

See the LICENSE file for more info.