The Z-Stack Gateway supports OTA Download, acting as a server for updating
the firmware of remote devices.									

In order to experiment with the OTA Download feature using the provided
sample application, you will need the following files:

  1.  Hex image, for programming a sample OTA client (based on SmartRF +
  CC2530 hardware): 
  
      SampleSwitchRouter_OTA.hex

  This file is extracted by the Z-Stack Gateway installer, under Firmware
  directory.

  This hex file can be used to program a SmartRF + CC2530 board to act as a 
  Sample Switch Router device that also contains OTA Client functionality. 
  Having OTA client functionality implies that it can communicate with an 
  OTA Server and accept images for upgrade.

  2.  Sample Upgrade Image Files for updating the target device (that is
  initially programmed with SampleSwitchRouter_OTA.hex):

      5678-1234-0000AAAA.zigbee  
      5678-1234-0000BBBB.zigbee 

  These files are located inside the 'precompiled' tar file, under directory
  misc/ota_sample_images.
  
  Two zigbee Upgrade Images that can be used to upgrade the Sample Switch OTA 
  Client via the SampleApp Demo.
  A successful upgrade with image 5678-1234-0000AAAA.zigbee will convert the 
  device to an End Device.
  A successful upgrade with image 5678-1234-0000BBBB.zigbee will convert the 
  device into a Router.
  In both cases the image version will appear on the SmartRF display after the 
  upgrade finishes.

 
Using the flash programmer
--------------------------
1. Connect the flash programmer to your host windows machines using the 
   USB connecter.
2. In the first dropdown box (What do you want to Program) select: 
   Program CCxxxx SoC or MSP430
3. Open the "System-on-chip" tab, and switch on the SmartRF board. 
   You should see an entry corresponding to your device that indicates 
   "CC2530" as the chiptype.
4. Go to the "Flash Image" entry and browse to the SampleSwitchRouter_OTA.hex 
   image in this folder.
5. Click the radio button "Retain IEEE address when reprogramming the chip"
6. Under Actions, pick "Erase, Program and Verify". 
7. Click "Perform Actions". Once completed, switch the board off.

Joining the network
-------------------
The SampleSwitch OTA Client will not automatically join an open network when 
switched on. 
The joystick on the bottom left corner needs to be pushed towards the right for 
a few seconds after the "Permit Join" on the network is enabled.
Once the device successfully joins the network, you should see it listed on the 
SampleApp UI with 2 endpoints:- 
0xE is the OTA endpoint 
0x8 is the Switch endpoint

Please refer to the User Guide for details on how to perform an OTA upgrade on 
this device.
