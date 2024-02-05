/**
 * PicoTuner. 
 * BATC Minitiouner V3 interface. 
 * Colin Durbridge G4EML January 2024.
 *
 * Provides a partial emulation of the FTDI FT2232 Dual port fifo chip. 
 * Only those functions used by the Minitiouner are emulated.
 *
 * Written for the Arduino IDE using the RP2040 support developed by 
 * Earle F. Philhower, III.
 * See https://arduino-pico.readthedocs.io/en/latest/ for details. 
 * 
 * USB device is based on the Dev_Lowlevel example code provided by 
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 * SPDX-License-Identifier: BSD-3-Claus
 * 
 * 
 */


//Define the USB VID/PID values. 2E8A:BA2C uses the Raspberry Pi VID and a random PID. 
//Original FTDI chip uses 0403:6010
#define USBVID 0x2E8A
#define USBPID 0xBA2C

//BCDDEVICE is the device revision number. We can probably use this to identify Minitiouner or Knucker 
//0101 = Minitiouner Parallel TS      0102 = Minitiouner Serial TS      0201 = Knucker. 
#define BCDDEVICE 0x0102       

//IO pin definitions.  Each group of pins must remain on sequential pins in the same order. The PIO hardware can only access sequential pins.  

#define DEBUG1 0          //Debug pin for Scope or terminal
#define DEBUG2 1          //Debug pin for scope or terminal

#define AC0 2         //NIM RESET LOW
#define AC1 3         //T2 SYNC
#define AC2 4         //J8-6
#define AC3 5         //J8-5
#define AC4 6         //J8-4  LNB Bias Enable 
#define AC5 7         //J8-3 
#define AC6 8         //J8-2 
#define AC7 9         //J8-1  LNB BIAS Voltage Select


#define TS1DAT 10       //TS1 Data Bit (TS1D7)
#define TS1CLK 11       //TS1CLK
#define TS1VALID 12     //TS1VALID

#define TS2DAT 13     //TS2 Data bit (TS2D7)
#define TS2CLK 14     //TS2CLK 
#define TS2VALID 15   //TS2VALID

#define SPARE16 16    //Reserved for Ethernet Hat
#define SPARE17 17    //Reserved for Ethernet Hat
#define SPARE18 18    //Reserved for Ethernet Hat
#define SPARE19 19    //Reserved for Ethernet Hat               
#define SPARE20 20    //Reserved for Ethernet Hat                          
#define SPARE21 21    //Reserved for Ethernet Hat
#define SPARE22 22    //Reserved for Ethernet Hat

#define LED 25         //Onboard LED

#define SDA 26         //AD1 SDA
#define SCL 27         //AD0 SCL

#define SPARE28 28


#include <hardware/pio.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>
#include "dev_lowlevel.h"

#include "Pio_Parallel_2.pio.h"
#include "Pio_Parallel_8.pio.h"
#include "Pio_TS_S.pio.h"


//PIO to use
PIO pioa = pio0;
PIO piob = pio1;
// variable for the start location of the State Machine programs
uint offset_2;
uint offset_8;
uint offset_TS;
//variables for each state machine
uint sm_2;            //State machine for 3 bit port
uint sm_8;            //State machine for 8 bit port
uint sm_TS;           //TS Input 

//setup() initialises Core 1 of the RP2040.
void setup() 
{

   pinMode(DEBUG1,OUTPUT);
   pinMode(DEBUG2,OUTPUT);
   pinMode(LED,OUTPUT);

   //initialise the PIO state machines

   // add the 3 bit port program and return its start location 
   offset_2 = pio_add_program(pioa, &Pio_Parallel_2_program);
   // Find an unused State machine
   sm_2 = pio_claim_unused_sm(pioa, true);
   //initialse the PIO SM using the helper function defined in the .pio file
   Pio_Parallel_2_init(pioa, sm_2, offset_2, SDA);

   // add the 8 bit port program and return its start location 
   offset_8 = pio_add_program(pioa, &Pio_Parallel_8_program);
   // Find an unused State machine
   sm_8 = pio_claim_unused_sm(pioa, true);
   //initialse the PIO SM using the helper function defined in the .pio file
   Pio_Parallel_8_init(pioa, sm_8, offset_8, AC0);

  // add the Serial TS input program and return its start location 
   offset_TS = pio_add_program(piob, &Pio_TS_S_program);
   // Find an unused State machine
   sm_TS = pio_claim_unused_sm(piob, true);
   //initialse the PIO SM using the helper function defined in the .pio file
   Pio_TS_S_init(piob, sm_TS, offset_TS, TS2DAT);

    irq_set_exclusive_handler(5,isr_usbctrl);

// reset all of the  buffers. 
    clearBuffers();

    for(int i=0;i<TSBUFNUM;i++)
      {
        TSBuf[i][0] = 0xAA;                     //first two bytes of each 512 byte buffer are the FTDI status bits. 
        TSBuf[i][1] = 0xAA;                     //We don't know what they do so just using 0xAA as a nominal value.
      }

    usb_device_init();

    // Wait until configured
    while (!configured) {
        tight_loop_contents();
    }

    // Get ready to rx MPSSE commands from host
    
    usb_start_transfer(usb_get_endpoint_configuration(EP2_OUT_ADDR), NULL, 64);

}

//loop() uses Core 1 of the RP2040
//
void loop() 
{
  if(commandsAvailable() > 0)
    {
      processCommands();
    } 

   if((TSBufsAvailable() >= 1 )&&(!TSTransferInProgress))       //wait till we have a 512 byte transfer like the FTDI chip does. 
  {
    sendTS(TSNORMAL);
  }

  if((millis() > EP83Timeout) && (!TSTransferInProgress))       //send a status packet every 16ms and reset the TS state machine if we have not sent anything recently. 
  {
    sendTS(TSSTATUS);
    pio_sm_restart(piob, sm_TS);
  }

}

//setup1() initialises Core 2 of the RP2040. 
//Nothing to do here. 
void setup1()
{

}

//loop1() uses Core 2 of the TRP2040
//Core 2 just services the TS data from the PIO device and saves it to the TS buffers.

void loop1()
{
  uint TScount;
  uint32_t wor;
  int bytestobufend;
  static int testbyte=0;

 TScount = pio_sm_get_rx_fifo_level(piob, sm_TS);
                                                           
  if(TScount > 0)
    {
       for(int i=0; i < TScount; i++)
         {
            wor = pio_sm_get_blocking(piob, sm_TS);           //Get the next available 4 bytes of TS   
            for(int b=0;b<4;b++)                              //and copy them to the buffer. 
              {
                TSBuf[TSBufInNumber][TSBufInPointer++] = wor >> 24;
                wor=wor << 8;
                if(TSBufInPointer >= TSBUFSIZE)               //if this buffer is full
                  {
                    TSBufInPointer = 2;                       //move on to the next beffur
                    TSBufInNumber++;
                    if(TSBufInNumber >= TSBUFNUM) TSBufInNumber = 0;
                  }
             }
         }     
    }
}                                                                                                 

void clearBuffers(void)
{
    commandBufInPointer = 0;
    commandBufOutPointer = 0;
    TSBufInPointer = 2;
    TSBufOutPointer = 0;
    TSBufInNumber = 0;
    TSBufOutNumber = 0;
    resultBuf[0] = 0;               //It seems the FTDI Device adds a two byte header to all results. 
    resultBuf[1] = 0;               //We don't know what this should be but zeros seems to work with Longmynd
    resultBufCount = 2;             //
}

//process all available MPSSE commands. 
//Only a few are relevent to the minitiouner. The rest are ignored. 
void processCommands(void)
{
  int command;
  int param1;
  int param2;
  int param3;

  while(commandsAvailable() > 0)
  {
    command = getNextCommand();

    switch (command)
      {
        case 0xAA:                      //Special Bad Command for synchronising. Send response FA AA immediately.
          resultBuf[resultBufCount++] = 0xFA;
          resultBuf[resultBufCount++] = 0xAA;
          sendResult();
          break;

        case 0x80:                    //Set GPIO Low port 
          param1 = getNextCommand();  //pin values to set
          param2 = getNextCommand();  //pin direction 1=OUTPUT

          setGPIO(0,param1,param2);
          break;

        case 0x82:                    //Set GPIO High port 
          param1 = getNextCommand();  //pin values to set
          param2 = getNextCommand();  //pin direction 1=OUTPUT
          setGPIO(1,param1,param2);
          break;

        case 0x11:                    //Shift out 1 byte MSB first on negative edge of clock. 
          param1=getNextCommand();    //Length High
          param2=getNextCommand();    //Length Low. We only handle one byte at a time so we can ignore the length. 
          param3=getNextCommand();    //Byte
          shiftByteOut(param3);
          break;

        case 0x25:                    //Shift in 1 byte MSB first on negative edge of clock. (Datasheet says this should be 0x20 but Longmynd uses 0x25)
          param1=getNextCommand();    //Length High
          param2=getNextCommand();    //Length Low. We only handle one byte at a time so we can ignore the length. 
          shiftBitsIn(8);
          break;

        case 0x27:                    //Shift in 1 bit on negative edge of clock. (Datasheet says this should be 0x22 but Longmynd uses 0x27)
          param1=getNextCommand();    //number of bits. We only nedd to handle one bit for ACK so we can ignore this. 
          shiftBitsIn(1);
          break;

        case 0x87:                     //send result. Immediately send the current result buffer to host.
          sendResult();
          break;

        //single byte commands that we recognise but ignore because they are FTDI specific. 
        case 0x8A:                    //Disable clock/5
        case 0x97:                    //Disable adaptive clock
        case 0x85:                    //No Loopback
        case 0x8D:                    //Disable 32 phase clock        
          break;

        //Three byte commands that we recognise but ignore because they are FTDI specific. 
        case 0x86:                    //Set Clock Divide
          param1=getNextCommand();
          param2=getNextCommand();
          break;

        default:
          break;
      }

  }
}



//Using PIO state machines set the gpio port to value, set pin directions to direction, 1 = output. port 0=AD, 1=AC
 void setGPIO(int port, int value, int direction)
 {
  int invdir;
  int invval;
  switch(port)
    {
      case 0:
        invdir = ((direction & 0x01)<< 1) + ((direction & 0x02) >> 1);             //Swap the two bits of both Direction and Value so that SDA and SCL match the RP2040 Hardware pins. 
        invval = ((value & 0x01)<< 1) + ((value &0x02) >> 1);                     //This allows a possible future update to use the hardware I2C instead of bitbanging. 
        pio_sm_put_blocking(pioa, sm_2, (invdir << 2) + invval);
        break;

      case 1:
        pio_sm_put_blocking(pioa, sm_8, (direction << 8) + value );
        break;
    }

 }




//shift in count bits MSB first.
void shiftBitsIn(int count)
{
  int val = 0;

  for(int c= count ; c > 0 ; c--)
    {
      pio_sm_put_blocking(pioa, sm_2, (0x02 <<2 ) + 0x02);            //set SCL High with SDA HI Z
      delayMicroseconds(2);
      val=(val<<1) + (digitalRead(SDA) ? 1:0);                       //shift in the data bit
      pio_sm_put_blocking(pioa, sm_2, (0x02 <<2 ) + 0x00);            //set SCL Low with SDA HI Z
      delayMicroseconds(2);
    }

  resultBuf[resultBufCount++] = val;
}

//shift out one byte of data MSB first using the 3 bit State machine
void shiftByteOut(int byte)
{
  for(int c= 8 ; c > 0 ; c--)
    {
      pio_sm_put_blocking(pioa, sm_2, (0x03 <<2 ) + ((byte & 0x80) ? 1:0));            //set SDA with SCL Low
      delayMicroseconds(2);
      pio_sm_put_blocking(pioa, sm_2, (0x03 <<2 ) + ((byte & 0x80) ? 3:2));            //set SDA with SCL High
      delayMicroseconds(2);
      pio_sm_put_blocking(pioa, sm_2, (0x03 <<2 ) + ((byte & 0x80) ? 1:0));            //set SDA with SCL LOW
      byte=byte<<1;
    }


}

