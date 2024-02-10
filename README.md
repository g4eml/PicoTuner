# PicoTuner
## Description

This design is for a Raspberry Pi Pico based USB interface for the BATC Minitiouner V3 Digital TV Receiver.
It is intended to replace the previously used FTDI FT2232H module which is expensive and sometimes difficult to obtain. 
The Raspberry Pi Pico is low cost, readily available and can also handle both Transport streams from the NIM module. 

## Requirements

This code will work on the original Raspberry Pi Pico, the Pico W and probably on other RP2040 based boards.
Whilst being designed for the new BATC V3 Minitiouner project it is also possible to retrofit a Pico to the V2 Minitiouner.
The advantage of doing this is to provide access to the second Transport Stream from the NIM. This will allow two signals to be received at the same time. 

## Programming or updating the Raspberry Pi Pico (quick method)

This can be done either before or after the Pico has been fitted to the minitiouner, it makes no difference. Updating to a new firmware version is done the same way.   

1. Locate the latest compiled firmware file 'PicoTuner_vxxx.uf2' which will be found here https://github.com/g4eml/PicoTuner/releases and save it to your desktop. 

2. Hold down the BOOTSEL button on the Pico while connecting it to your PC using its micro USB port. The Pico should appear as a USB disk drive on your PC.

3. Copy the .uf2 file onto the USB drive. The pico will recognise the file and immediately update its firmware, reboot and the PC should recognise a new USB device. 

## Building your own version of the firmware (longer method and not normally required)
The Raspberry Pi Pico is programmed using the Arduino IDE with the Earl F. Philhower, III  RP2040 core. 

#### Installing the Arduino IDE

1. Download and Install the Arduino IDE 2.3.0 from here https://www.arduino.cc/en/software

2. Open the Arduino IDE and go to File/Preferences.

3. in the dialog enter the following URL in the 'Additional Boards Manager URLs' field: https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json

4. Hit OK to close the Dialog.

5. Go to Tools->Board->Board Manager in the IDE.

6. Type “pico” in the search box.

7. Locate the entry for 'Raspberry Pi Pico/RP2040 by Earle F. Philhower, III' and click 'Install'
 
#### Downloading the Software.

1. Download the latest released source code .zip file from https://github.com/g4eml/PicoTuner/releases

2. Save it to a convenient location and then unzip it. 

#### Programming the Pico

1. Open the Arduino IDE and click File/Open

2. Navigate to the File PicoTuner/Picotuner.ino (downloaded in the previous step) and click Open.

3. Select Tools and make the following settings.


   Board: "Raspberry Pi Pico"

   Debug Level: "None"

   Debug Port: "Disabled"

   C++ Exceptions: "Disabled"
   
   Flash Size: "2Mb (no FS)"
   
   CPU Speed: "133MHz"

   IP/Bluetooth Stack: "IPV4 Only"
   
   Optimise: "Small (-Os)(standard)"
   
   RTTI: "Disabled"
   
   Stack Protection: "Disabled"

   Upload Method: "Default (UF2)"
   
   USB Stack: "No USB"          *** Yes this is correct, we don't need any additional USB support, it is all handled by the program. 
   
   
5. Connect the Rasperry Pi Pico to the USB port while holding down the BOOTSEL button. 

6. Click Sketch/Upload.

The Sketch should compile and upload automatically to the Pico. If the upload fails you may need to disconnect the Pico and then hold down the BOOTSEL button while reconnecting. 

