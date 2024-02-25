#!/usr/bin/env python3

# forces the Picotuner into BOOTSEL mode ready for firmware update. 

# sudo pip3 install pyusb

import usb.core
import usb.util

# find the PicoTuner from its VID and PID
dev = usb.core.find(idVendor=0x2E8A, idProduct=0xBA2C)

# was it found?
if dev is None:
    print("\nError:- PicoTuner not found\n")
    exit()

# send the reset command to EP2
dev.write(0x02,chr(0xBB)) 

print("\nPicoTuner reset to BOOTSEL mode\nYou can now copy the .uf2 file to the USB drive\n") 
