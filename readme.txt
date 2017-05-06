Description:

This is a self extracting archive file, to be run on an x86 Linux machine. 
Once executed, following acceptance of the legal disclaimer, it extracts the 
following content to the specified directory:

- Documents/Z-Stack Linux Gateway BeagleBone QuickStart Guide.pdf
  Quick Start Guide for Installing/Running ZStack Gateway Binaries on 
  Beaglebone + CC2531 USB Dongle

- Documents/Z-Stack Linux Gateway User Guide.pdf
  Installation and User's guide for the TI Gateway Reference Design and 
  sample application.
  
- Documents/Z-Stack Linux Gateway User Guide BeagleBone Addendum.pdf
  Installation and User's guide addendum for Beaglebone
  
- Documents/Z-Stack Linux Gateway - API document v1.1.pdf
  Z-Stack Gateway Reference Design API document
    
- Documents/sample_app_ota_readme.txt
  Readme for OTA

- Firmware/CC2531-GW-ZNP_{revision}.hex
  Firmware image to be programmed into the CC2531 USB dongle ZigBee using a CC 
  Debugger.  
  
- Firmware/SampleSwitchRouter_OTA.hex
  Firmware image for a sample OTA Download client. For more details, see
  Documents/sample_app_ota_readme.txt

- Source
  Complete source of the sample application. Can be built by running 
  Source/build_sample_app
  
  NOTE: A pre-compiled sample app binary is included in the package. To rebuild 
  the sample app, please install the Linaro tool chain from the following link:
  http://software-dl.ti.com/sitara_linux/esd/AM335xSDK/latest/index_FDS.html
  and modify your "PATH" environment variable to include:
  "<LINARO INSTALL LOCATION>/bin". 
  
  The included pre-compiled binary was built with the GCC 4.7 2013.03 version
  of the Linaro tool chain.
 
- Proto_files
  Protobuf definition files, that define the APIs for the gateway server, 
  the network manager server and the OTA server.

- Precompiled_arm/z-stack_linux_gateway_arm_binaries_<revision>.tar
  Executables pre-compiled for arm-linaro linux, including the Z-Stack Linux
  Gateway subsystem servers and the sample application. This file 
  needs to be copied to the target platform, and extracted there.
  
Once extracted, the following content is available:

* servers
  This directory contains executables of all the Gateway Reference Design 
  subsystem servers (NPI, Z-Stack Server, NetworkManager, OTA and Gateway), 
  configuration files and scripts to start the Gateway Reference Design 
  subsystem and the sample application, as well as misc OTA related files.

* app/main.bin
  Executable of the sample application that demonstrates usage of the Gateway
  Reference Design APIs.

* protobuf/
  Directory that contains protobuf library that is required for the operation
  of the Gateway Reference Design subsystem servers, as well as for the sample
  application.

* tools/sbl_tool.bin
  A serial bootloader tool for reprogramming a new firmware image into the
  flash memory of the CC2531 ZigBee SOC.

* tools/gw_soc_fw_version_query.bin
  Tool to query the version of CC2531 ZigBee SOC firmware

* tools/bbb_usbreset.bin 
  Tool to hardware reset CC2531 USB Dongle plugged into Beaglebone Black.

* misc/CC2531-GW-ZNP_{revision}.bin
  A boot-loadable CC2531 Zigbee SOC firmware image to demonstrate reprogramming
  using the serial bootloader tool. Feature-wise, it has the same ZigBee 
  Network Processor content as the the hex with the same name.

* misc/ota_sample_images
  Directory that contains sample OTA images to be downloaded to a sample OTA 
  client. For more details, see Documents/sample_app_ota_readme.txt

================================================================================

v1.0.1 Release Notes:

* There are small changes to the Z-Stack Linux Gateway API (relative to the 
  1.0.0 release).  For previous users, if you are not using the provided 
  pre-compiled protobuf data accessor **-c.h files for the various servers, 
  you must re-generate them using the included .proto files.
