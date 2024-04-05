/**
 * PicoTuner. 
 * BATC PicoTuner interface. 
 * Colin Durbridge G4EML January 2024.
 *
 * Provides a partial emulation of the FTDI FT2232 Dual port fifo chip. 
 * Only those functions used by the software are emulated.
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
// work around for Errata E15 in RP2040 datasheet. USB controller can lock up if a large bulk transfer is started near the end of a USB frame. 
// This fix limits tranfers to the first 80% of the frame time 

#define FIX_ERRATA_E15


//Version Number BCD   vvrr
#define VERSIONMAJOR 0
#define VERSIONMINOR 11                         
//Define the USB VID/PID values. 2E8A:BA2C uses the Raspberry Pi VID and a random PID. 
//Original FTDI chip uses 0403:6010
#define USBVID 0x2E8A
#define USBPID 0xBA2C

//BCDDEVICE is the device revision number. 
//0101 =Parallel TS      0102 =  Serial TS   
#define BCDDEVICE 0x0102       

//IO pin definitions.  Each group of pins must remain on sequential pins in the same order. The PIO hardware can only access sequential pins.  

#define DEBUG1 0          //Debug pin for Scope or terminal. Also used for Front Panel LEDs
#define DEBUG2 1          //Debug pin for scope or terminal. Also used for Front Panel LEDs

#define AC0 2         //NIM RESET LOW
#define AC1 3         //
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

#define ETHSPRX 16    //Reserved for Ethernet Hat SPIO RX
#define ETHSPCS 17    //Reserved for Ethernet Hat SPIO CS Low
#define ETHSPCK 18    //Reserved for Ethernet Hat SPIO CLK               
#define ETHSPTX 19    //Reserved for Ethernet Hat SPIO TX
#define ETHRSTN 20    //Reserved for Ethernet Hat Reset Low                       
#define ETHINTN 21    //Reserved for Ethernet Hat Interrupt Low

#define ENA3V3 22    //Enable 3V3 Output. Set high when USB is fully initialised. 

#define LED 25         //Onboard LED

#define SDA 26         //AD1 SDA
#define SCL 27         //AD0 SCL

#define SPARE28 28


#include <hardware/pio.h>
#include "hardware/dma.h"
#include "hardware/irq.h"
#include <stdint.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>
#include "dev_lowlevel.h"

#include "Pio_Parallel_2.pio.h"
#include "Pio_Parallel_8.pio.h"
#include "Pio_TS_S.pio.h"

// variable for the start location of the State Machine programs
uint offset_2;
uint offset_8;
uint offset_TS1;
uint offset_TS2;

//variables for each state machine
PIO pioa = pio0;
uint sm_2 = 0;            //State machine for 3 bit port use state machine 0 on pio0
uint sm_TS1 = 3;          //TS1 Input Use State machine 3 on pio0
PIO piob = pio1;
uint sm_8 = 0;            //State machine for 8 bit port use state machine 0 on pio
uint sm_TS2 = 3;           //TS2 Input Use State machine 3 on pio1

#define TSPACKETLENW 47                               //47 32bit words equals one 188 byte TS packet. 
uint32_t TS2DMA[TSPACKETLENW];                        //DMA buffer for TS2 Packet              
uint32_t TS1DMA[TSPACKETLENW];                        //DMA buffer for TS1 Packet

uint DMA2Chan;
uint DMA1Chan;

bool TS2DMAAvailable = false;
bool TS1DMAAvailable = false;

int TS1ActiveTimeout = 0;
int TS2ActiveTimeout = 0;
unsigned long lastmillis =0;
#define TSTIMEOUT  20                //20 ms timeout for LEDs

//setup() initialises Core 0 of the RP2040.
//Core 0 does most of the work. 
void setup() 
{
   pinMode(DEBUG1,OUTPUT);
   pinMode(DEBUG2,OUTPUT);
   pinMode(LED,OUTPUT);

   //initialise pins for future Ethernet module
   pinMode(ETHSPTX,INPUT_PULLUP);
   pinMode(ETHSPRX,INPUT_PULLUP);
   pinMode(ETHSPCK,INPUT_PULLUP);
   pinMode(ETHSPCS,INPUT_PULLUP);
   pinMode(ETHINTN,INPUT_PULLUP);
   pinMode(ETHRSTN,OUTPUT);
   digitalWrite(ETHRSTN,LOW);

   //Configure the Power Control Pin
   
   pinMode(ENA3V3,OUTPUT);
   digitalWrite(ENA3V3,LOW);

   //initialise the PIO state machines

   // add the 3 bit port program and return its start location 
   offset_2 = pio_add_program(pioa, &Pio_Parallel_2_program);
   //initialse the PIO SM using the helper function defined in the .pio file
   Pio_Parallel_2_init(pioa, sm_2, offset_2, SDA);

   // add the 8 bit port program and return its start location 
   offset_8 = pio_add_program(piob, &Pio_Parallel_8_program);
   //initialse the PIO SM using the helper function defined in the .pio file
   Pio_Parallel_8_init(piob, sm_8, offset_8, AC0);


  // add the Serial input program and return its start location
   offset_TS1 = pio_add_program(pioa, &Pio_TS_S_program);
  //initialse the PIO SM using the helper function defined in the .pio file
    Pio_TS_S_init(pioa, sm_TS1, offset_TS1, TS1DAT);

  // add the Serial input program and return its start location
   offset_TS2 = pio_add_program(piob, &Pio_TS_S_program);
   //initialse the PIO SM using the helper function defined in the .pio file
   Pio_TS_S_init(piob, sm_TS2, offset_TS2, TS2DAT);



    irq_set_exclusive_handler(5,isr_usbctrl);

   // reset all of the  buffers. 
    clearBuffers(0);

    for(int i=0;i<TSBUFNUM;i++)
      {
        TS2Buf[i][0] = 0xAA;                     //first two bytes of each 512 byte buffer are the FTDI status bits. 
        TS2Buf[i][1] = 0xAA;                     //We don't know what they do so just using 0xAA as a nominal value.
        TS1Buf[i][0] = 0xAA;                     //first two bytes of each 512 byte buffer are the FTDI status bits. 
        TS1Buf[i][1] = 0xAA;                     //We don't know what they do so just using 0xAA as a nominal value.   
      }


    usb_device_init();

    // Wait until configured
    while (!configured) {
        tight_loop_contents();
    }

    digitalWrite(ENA3V3, HIGH);

    // Get ready to rx MPSSE commands from host
    
    usb_start_transfer(usb_get_endpoint_configuration(EP2_OUT_ADDR), NULL, 64);

}

//loop() uses Core 0 of the RP2040
//Core 0 does most of the work.
void loop() 
{

#ifdef FIX_ERRATA_E15

    digitalWrite(LED , (lastUSBSOF & 0x40000) == 0);                 //flash the LED about 2 times per second clocked by the USB SOF interrupt 

   if(TS1Deferred > 0)                                  //if we have a deferred transfer waiting
     {
      sendTS1NI(TS1Deferred - 1);                        //try to send it
     }

   if(TS2Deferred > 0)                                  //if we have a deferred transfer waiting
     {
      sendTS2NI(TS2Deferred - 1);                        //try to send it
     }

  if(resultDeferred > 0)                                  //if we have a deferred transfer waiting
     {
      sendResult();                                       //try to send it
     }

#else

     digitalWrite(LED , (millis() & 0x100) ==0);                 //flash the LED about 2 times per second based on millis()

#endif



  if(commandsAvailable() > 0)
    {
      processCommands();
    } 


   if((TS2BufsAvailable() >= 1 )&&(!TS2TransferInProgress))       //wait till we have a 512 byte transfer like the FTDI chip does. 
  {
    sendTS2NI(TSNORMAL);
  }

  if((millis() > EP83Timeout)&&(!TS2TransferInProgress))       //send a status packet every 16ms and reset the TS state machine if we have not sent anything recently. 
  {
    sendTS2NI(TSSTATUS);
    dma_channel_set_irq0_enabled(DMA2Chan, false);
    dma_channel_abort(DMA2Chan);
    dma_channel_acknowledge_irq0(DMA2Chan);
    pio_sm_restart(piob, sm_TS2);
    dma_channel_set_irq0_enabled(DMA2Chan, true);
    dma_hw->ints0 = 1u << DMA2Chan;
    dma_channel_set_write_addr(DMA2Chan, TS2DMA, true);
    clearBuffers(2);
  }

   if((TS1BufsAvailable() >= 1 )&&(!TS1TransferInProgress))       //wait till we have a 512 byte transfer like the FTDI chip does. 
  {
    sendTS1NI(TSNORMAL);
  }

  if((millis() > EP84Timeout)&&(!TS1TransferInProgress))       //send a status packet every 16ms and reset the TS state machine if we have not sent anything recently. 
  {
    sendTS1NI(TSSTATUS);
    dma_channel_set_irq1_enabled(DMA1Chan, false);
    dma_channel_abort(DMA1Chan);
    dma_channel_acknowledge_irq1(DMA1Chan);
    pio_sm_restart(pioa, sm_TS1);
    dma_channel_set_irq1_enabled(DMA1Chan, true);
    dma_hw->ints1 = 1u << DMA1Chan;
    dma_channel_set_write_addr(DMA1Chan, TS1DMA, true);
    clearBuffers(1);
  }

  if(millis() > lastmillis)             //every millisecond
  {
    if(TS1ActiveTimeout > 0) TS1ActiveTimeout--;
    if(TS2ActiveTimeout > 0) TS2ActiveTimeout--;
    lastmillis = millis();
  }

  if(TS1ActiveTimeout > 0)
    {
      digitalWrite(DEBUG1,1);
    }
    else
    {
      digitalWrite(DEBUG1,0);
    }

  if(TS2ActiveTimeout > 0)
    {
      digitalWrite(DEBUG2,1);
    }
    else
    {
      digitalWrite(DEBUG2,0);
    }

}

//setup1() initialises Core 1 of the RP2040.
//Core 1 does the high priority work   
void setup1()
{
  //Setup DMA for TS2

  DMA2Chan = dma_claim_unused_channel(true);
 
  dma_channel_config c2 = dma_channel_get_default_config(DMA2Chan);

  channel_config_set_transfer_data_size(&c2, DMA_SIZE_32);
  channel_config_set_read_increment(&c2, false);
  channel_config_set_write_increment(&c2, true);
  channel_config_set_dreq(&c2, DREQ_PIO1_RX3);

  dma_channel_configure(
        DMA2Chan,               // Channel to be configured
        &c2,                    // The configuration we just created
        TS2DMA,                 // The initial write address the TS buffer
        &piob->rxf[sm_TS2],      // Source pointer
        TSPACKETLENW,            // Number of transfers
        false                    // do not start immediately.
    );
        
    dma_channel_set_irq0_enabled(DMA2Chan, true);      // Tell the DMA to raise IRQ line 0 when the channel finishes a block

 //Setup DMA for TS1

  DMA1Chan = dma_claim_unused_channel(true);
 
  dma_channel_config c1 = dma_channel_get_default_config(DMA1Chan);

  channel_config_set_transfer_data_size(&c1, DMA_SIZE_32);
  channel_config_set_read_increment(&c1, false);
  channel_config_set_write_increment(&c1, true);
   channel_config_set_dreq(&c1, DREQ_PIO0_RX3);

  dma_channel_configure(
        DMA1Chan,               // Channel to be configured
        &c1,                    // The configuration we just created
        TS1DMA,                 // The initial write address the TS buffer
        &pioa->rxf[sm_TS1],     // Source pointer
        TSPACKETLENW,           // Number of transfers
        false                    // do not start immediately.
    );
        
    dma_channel_set_irq1_enabled(DMA1Chan, true);      // Tell the DMA to raise IRQ line 1 when the channel finishes a block 

  //start the DMA transfers
    irq_set_exclusive_handler(DMA_IRQ_0, DMA2_handler);       // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_set_enabled(DMA_IRQ_0, true);

    DMA2_handler();        //manually call the handler once to start the transfers

    irq_set_exclusive_handler(DMA_IRQ_1, DMA1_handler);       // Configure the processor to run dma_handler() when DMA IRQ 1 is asserted
    irq_set_enabled(DMA_IRQ_1, true);

   DMA1_handler();        //manually call the handler once to start the transfers

}


//loop1() uses Core 1 of the RP2040
//Core 1 does the high priority work. Handles the DMA and re-formats the TS data from the PIO device and saves it to the TS buffers.
void loop1()
{
  union W
  {
    uint32_t w;                //map two variables to the same memory location
    uint8_t b[4];              //this allows access by 32bit word or by 4 8 bit bytes. 
  } wor;
                                                        
  if(TS2DMAAvailable)                     //have we received a new TS2 Packet?
    {
      TS2ActiveTimeout = TSTIMEOUT;
       for(int i=0; i < 47; i++)
         {
            wor.w = TS2DMA[i];                 //Get the next available 4 bytes of the TS packet  
            for(int b=3;b>=0;b--)                              //and copy them as bytes to the USB buffer. 
              {
                TS2Buf[TS2BufInNumber][TS2BufInPointer++] = wor.b[b];
                if(TS2BufInPointer >= TSBUFSIZE)               //if this buffer is full
                  {
                    TS2BufInPointer = 2;                       //move on to the next beffur
                    TS2BufInNumber++;
                    if(TS2BufInNumber >= TSBUFNUM) TS2BufInNumber = 0;
                  }
             }
         }
      TS2DMAAvailable = false;   
    }

  if(TS1DMAAvailable)                     //have we received a new TS1 Packet?
    {
      TS1ActiveTimeout = TSTIMEOUT;
       for(int i=0; i < 47; i++)
         {
            wor.w= TS1DMA[i];                 //Get the next available 4 bytes of the TS packet  
            for(int b=3;b>=0;b--)                              //and copy them to the buffer. 
              {
                TS1Buf[TS1BufInNumber][TS1BufInPointer++] = wor.b[b];
                if(TS1BufInPointer >= TSBUFSIZE)               //if this buffer is full
                  {
                    TS1BufInPointer = 2;                       //move on to the next beffur
                    TS1BufInNumber++;
                    if(TS1BufInNumber >= TSBUFNUM) TS1BufInNumber = 0;
                  }
             }
         }
      TS1DMAAvailable = false;   
    }

}                                                                                                 

//interrupt called when DMA for TS2 has completed. 
void DMA2_handler()
{
  // Clear the interrupt request.
    dma_hw->ints0 = 1u << DMA2Chan;
    // Give the channel a new write address, and re-trigger it
    dma_channel_set_write_addr(DMA2Chan, TS2DMA, true);
    TS2DMAAvailable = true;        //flag the data is avaiable. 
}

//interrupt called when DMA for TS1 has completed. 
void DMA1_handler()
{
  // Clear the interrupt request.
    dma_hw->ints1 = 1u << DMA1Chan;
    // Give the channel a new write address, and re-trigger it
    dma_channel_set_write_addr(DMA1Chan, TS1DMA, true);
    TS1DMAAvailable = true;        //flag the data is avaiable. 
}


void sendTS2NI(int mode)
{
  noInterrupts();
  sendTS2(mode);
  interrupts();
}

void sendTS1NI(int mode)
{
  noInterrupts();
  sendTS1(mode);
  interrupts();
}

void clearBuffers(int sel)
{
  if((sel == 2) || (sel == 0))
  {
    TS2BufInPointer = 2;
    TS2BufOutPointer = 0;
    TS2BufInNumber = 0;
    TS2BufOutNumber = 0;
  }


   if((sel == 1) || (sel == 0))
   {
    TS1BufInPointer = 2;
    TS1BufOutPointer = 0;
    TS1BufInNumber = 0;
    TS1BufOutNumber = 0;
   }


   if(sel == 0)
   {
    commandBufInPointer = 0;
    commandBufOutPointer = 0;
    resultBuf[0] = 0;               //It seems the FTDI Device adds a two byte header to all results. 
    resultBuf[1] = 0;               //We don't know what this should be but zeros seems to work with Longmynd
    resultBufCount = 2;             //
   }
}

//process all available MPSSE commands. 
//Only a few are relevent to the PicoTuner. The rest are ignored. 
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

        // New command to return firmware version
        case 0xBA:
          resultBuf[resultBufCount++] = VERSIONMAJOR;
          resultBuf[resultBufCount++] = VERSIONMINOR;
          sendResult();
          break; 

        // New command to force Rp2040 into BOOTSEL mode for software update
        case 0xBB:
            reset_usb_boot(0,0);
          break; 
      
        //Unrecognised commands  all return Bad Command response
        default :
          resultBuf[resultBufCount++] = 0xFA;
          resultBuf[resultBufCount++] = command;
          sendResult();
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
        pio_sm_put_blocking(piob, sm_8, (direction << 8) + value );
        break;
    }

 }




//shift in count bits MSB first.
void shiftBitsIn(int count)
{
  int val = 0;
  noInterrupts();
  for(int c= count ; c > 0 ; c--)
    {
      pio_sm_put_blocking(pioa, sm_2, (0x02 <<2 ) + 0x02);            //set SCL High with SDA HI Z
      delayMicroseconds(1);
      val=(val<<1) + (digitalRead(SDA) ? 1:0);                       //shift in the data bit
      pio_sm_put_blocking(pioa, sm_2, (0x02 <<2 ) + 0x00);            //set SCL Low with SDA HI Z
      delayMicroseconds(1);
    }
   interrupts();
  resultBuf[resultBufCount++] = val;
}

//shift out one byte of data MSB first using the 3 bit State machine
void shiftByteOut(int byte)
{
  noInterrupts();
  for(int c= 8 ; c > 0 ; c--)
    {
      pio_sm_put_blocking(pioa, sm_2, (0x03 <<2 ) + ((byte & 0x80) ? 1:0));            //set SDA with SCL Low
      delayMicroseconds(1);
      pio_sm_put_blocking(pioa, sm_2, (0x03 <<2 ) + ((byte & 0x80) ? 3:2));            //set SDA with SCL High
      delayMicroseconds(1);
      pio_sm_put_blocking(pioa, sm_2, (0x03 <<2 ) + ((byte & 0x80) ? 1:0));            //set SDA with SCL LOW
      byte=byte<<1;
    }
   interrupts();

}

