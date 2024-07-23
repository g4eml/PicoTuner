# PicoTuner
## Description

This design is for a Raspberry Pi Pico based USB interface for the BATC Minitiouner V3 Digital TV Receiver.
It is intended to replace the previously used FTDI FT2232H module which is expensive and sometimes difficult to obtain. 
The Raspberry Pi Pico is low cost, readily available and can also handle both Transport streams from the NIM module. 

## Requirements

This code will work on the original Raspberry Pi Pico, the Pico W and probably on other RP2040 based boards.
Whilst being designed for the new BATC V3 Minitiouner project it is also possible to retrofit a Pico to the V2 Minitiouner.
The advantage of doing this is to provide access to the second Transport Stream from the NIM. This will allow two signals to be received at the same time.

## Limitations

The Pico hardware can only support USB 1.1 Full Speed mode which is up to 12 Mb/s. The FTDI module can support High speed mode which is much faster. For this reason the Pico is only able to support the low Sample rate DVBS transmissions commonly used for amateur radio. It is not able to receive the much higher SRs used by commercial TV.  The upper speed limit has not yet been fully defined but it is perfectly capable of receiving the QO-100 (SR1500) Beacon twice, using both TS. 

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

## Firmware description

The firmware is based on the Raspberry Pi foundation example code for a low level USB device. It adds additional changes to the device descriptors so that the device configuration is very close to that of the previoulsy used FTDI2232 module.

Control of the GPIO pins for I2C, for device control and for reading the two TS streams is done using PIO state machines which are defined in the .pio.h files. 
Note that these files are auto generated and should not be manually edited. If you need to change the PIO machines then you will need to edit the .pio files and recompile them. 
The .pio files are not visible in the Arduino IDE, you should use another text editor to work on them. When finished you should run the 'build_pio.bat' batch job to create the pio.h files. 

Both cores of the RP2040 chip are used. Core0 does most of the work including running the interrupt driven USB device. Core1 handles the task of handling the DMA, copying and reformatting of the TS data.
Note that both cores are fairly time critical. Adding debugging print statements will slow them down and cause errors with the USB device. 

## USB device description

The resulting USB device has been made to look very similar to the FTDI FT2232H and presents two interfaces.  This is so that any existing software can easily be adapted to work with the new device. 
It is however far from a full emulation of the FT2232H. Only those features need by the Minitiuoner are implemented. Some FTDI commands are acknowledged but do nothing. 

The device VID is 0x2E8A (Raspberry Pi Foundation)

The device PID is 0xBA2C 

The endpoints are as follows:-

On Interface 0:-

0x00 and 0x80 as output and input for device control. 

0x81 and 0x02 for I2C and GPIO control. Very loosly emulating the FTDI MPSSE device

On Interface 1:-

0x83 and 0x84 as Bulk transfer inputs for streaming TS2 and TS1. Sent 512 bytes at a time. Note that there is a 2 byte status word at the beginning of each 512 byte transfer which will nedd to be removed to reform the original TS. 

## USB Drivers

No special drivers are needed for Ryde or Portsdown. If you are using the Picotuner with Windows then you will need to load the WinUSB drivers using the Zadig program which can be downloaded from https://github.com/pbatard/libwdi/releases/tag/v1.5.0

Using Zadig you can now install the WinUSB driver for both of the USB interfaces presented by the Pico.

![image-5](https://github.com/user-attachments/assets/94cb2451-55ff-4494-bf5a-d9fc8738184e)
![image-6](https://github.com/user-attachments/assets/61956c72-46fd-413a-b2a9-1ac4dca4316c)

Note that you must load the driver for Interface 0 and Interface 1  as shown above. 

Once the correct drivers have been loaded the picotuner should appear under 'Universal Serial Bus Devices' in windows hardware manager. 

![PicoTuner in Device Manager](https://github.com/user-attachments/assets/24cf0ec4-5501-42fe-8544-fb8073c3bcf9)


Alternatively Tom AR6TG has written a utility which should automate the driver installation, 

https://www.dropbox.com/scl/fi/3ziiiq71hretd2yzaou8f/picotuner_driver_test_app.zip?rlkey=gl4xsxddxprxfvjjydebvez5y&dl=0

# Adaptor PCB for Minitiouner

![Adaptor_Small](https://github.com/g4eml/PicoTuner/assets/1881884/8f318989-f6f2-4fd1-a1c9-3b0cc474d794)

Whilst the PicoTuner was designed for use in a new tuner design it can also be retrofitted to the BATC V2 Minitiouner and similar designs providing their FTDI modules are fitted in sockets. This involves removing the FTDI module, replacing it with an adaptor pcb and adding 5 wires to connect signals that are not present on the FTDI sockets. The advantage of doing this upgrade is to provide access to the second TS. When supported by the host software this then allows the reception of two signals at the same time. To save space and to try to make the module mechanically compatible surface mount techniques are used. However due to the large size of the components involved it is still easily assembled by hand.  

## What is needed to make the adaptor

![Parts Small](https://github.com/g4eml/PicoTuner/assets/1881884/f39932b6-50eb-4cb2-a415-d2c30107807e)

PCB :- These will be available from the BATC shop, or you can get your own made using one of the Chinese PCB manufacturers. The Gerber files are include in the Adaptor Folder.

Raspberry Pi Pico :- Surface mount version without pin headers.<br />
The Pi Hut Part Number:- SC0915    Mouser Part Number :- 358-SC0915

2 x 26 way Dual row 2.54mm surface mount pin headers.<br /> 
Harwin Part Number:- M20-8761346   Mouser Part Number:- 855-M20-8761346   Farnell Part Number :- 3756209

1 x 5 way Single row 2.54mm through hole pin header.<br /> 
Harwin Part Number:- M20-9990545   Mouser Part Number:- 855-M20-9990545   Farnell Part Number :- 1022252

## Assembly instructions

Start with fitting the first of the 26 Way Dual row pin headers. Apply solder to one pad of the PCB and then tack the connector in place. Ensure that all of the pins are centrally aligned on the pads. Once you are happy with the position apply solder to all af the pads to fix the connector in position. Fit the second connector in the same way, checking the the spacing is correct to fit into the socket on the minitiouner before final soldering. If you align both connectors accurately on the pads this will be correct but you may wish to use a scrap of veroboard or someting similar to check the alignment. Once you are happy solder the second connector into place.

Lay the pico board directly onto the PCB and align the edges with the pads. Apply solder to the castelated edge of pin 1 to form a fillet of solder. Check the alignment of the board and adjust as necessary. When happy apply solder to all castellated holes. If possible leave the driled holes free of solder. This will simplify the fitting of the Ethernet module at a later date. 

Fit and solder the 5 way pin header. 

![Underside Small](https://github.com/g4eml/PicoTuner/assets/1881884/49900da7-42b1-4a40-9b7e-1b92a5d29567)
![Topside Small](https://github.com/g4eml/PicoTuner/assets/1881884/d9e220fb-2e78-4aca-b498-b67add5d8090)

## Fitting to the BATC V2 Minitiouner

Remove the FTDI module from its socket. Plug the Pico adaptor PCB in its place. The USB connector should be very close to the same position and should still be accessable through the rear panel. 

Connect pin 5 of J1 on the adaptor module to pin 9 of U7 (74HC10) on the minitiouner board. The 74HC10 is no longer used, so it can be removed or left in place, it makes no difference. You can solder the wire directly to the pin of the chip, remove the chip and plug the wire into the socket or use a pin header. 

Connect pin 4 of J1 to pin 10 of U7 in the same way. 

![74HC10 Small](https://github.com/g4eml/PicoTuner/assets/1881884/7278e593-fe17-4b4f-95c3-7d5bc239c34a)

The final three wires need to be connected between J1 and the 'TS1' connector on the minitiouner PCB. These wires are only required if you wish to use the second TS. 

Connect J1 Pin 1 to D7

Connect J1 pin 2 to CLK

Connect J1 pin 3 to VA

![TS1 Small](https://github.com/g4eml/PicoTuner/assets/1881884/7707e6ab-416c-4767-a013-00b3b92feabe)
![Fitting Small](https://github.com/g4eml/PicoTuner/assets/1881884/6bb9ed64-c67d-4774-8cab-f369e46045bd)

## Programming the Pico

Program the pico as described [above](#programming-or-updating-the-raspberry-pi-pico-quick-method). 

















