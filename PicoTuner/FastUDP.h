// custom drivers for the Wiznet 5100S chip for fast UUDP transfers. 
// Colin Durbridge G4EML

#include <W5100lwIP.h>          //we will use the library functions for the W5100S chip initially to handle the DHCP setup. Then switch to the faster UDP code when connected. 

#define WIZCHIPCS 17
#define WIZCHIPRS 20

Wiznet5100lwIP eth(WIZCHIPCS); // Parameter is the Chip Select pin  

bool EthernetConnected = false;

bool UDPTransferInProgress[3] = {false,false,false};

unsigned int ControlPort = 7300;     //UDP control port to listen on.
unsigned int TS1Port = 7301;         //UDP port to send TS1 to
unsigned int TS2Port = 7302;         //UDP Port to send TS2 to
unsigned int ControlDestPort = 7300; //UDP remote Port. 


IPAddress ControlIP(192,168,1,2);     //Default IP Destination for Control Replies       
IPAddress TS1IP(192,168,1,2);         //Default IP Destination for TS1           
IPAddress TS2IP(192,168,1,2);         //Default IP Destination for TS1   

// Local IP settings will be set by DHCP on first connection 
IPAddress MyIP;
IPAddress MyGate;
IPAddress MyNet;
uint8_t  MyMac[6];

void FastUDPInit(void);
void FastUDPSend(uint8_t socket , IPAddress DestIP , unsigned int SourcePort , unsigned int DestPort , uint8_t * Buffer, uint16_t len);
void wizchip_set_reg8(uint16_t address, uint8_t val);
void wizchip_set_reg16(uint16_t address, uint16_t val);
void wizchip_set_IP(uint16_t address, uint32_t val);
void wizchip_set_MAC(uint16_t address, uint8_t * val);
void wizchip_write_buf(uint16_t address, uint8_t* pBuf, uint16_t len);
uint8_t wizchip_read_reg8(uint16_t address);
uint16_t wizchip_read_reg16(uint16_t address);
uint16_t FastUDPAvailable(uint8_t socket);
uint16_t FastUDPRead(uint8_t socket, IPAddress * ip, uint16_t * port, uint8_t * buf);
void wizchip_read_buf(uint16_t address, uint8_t* pBuf, uint16_t len);
bool FastConnected(void);


//Wiznet chip register definitions

  /** Common registers */
    enum {
        MR   = 0x0000,  ///< Mode Register address (R/W)
        GAR  = 0x0001,  ///< Gateway IP Register address (R/W)
        SUBR = 0x0005,  ///< Subnet mask Register address (R/W)
        SHAR = 0x0009,  ///< Source MAC Register address (R/W)
        SIPR = 0x000F,  ///< Source IP Register address (R/W)
        IR   = 0x0015,  ///< Interrupt Register (R/W)
        IMR  = 0x0016,  ///< Socket Interrupt Mask Register (R/W)
        RTR  = 0x0017,  ///< Timeout register address (1 is 100us) (R/W)
        RCR  = 0x0019,  ///< Retry count register (R/W)
        RMSR = 0x001A,  ///< Receive Memory Size
        TMSR = 0x001B,  ///< Transmit Memory Size
        PHYSR0 = 0x003C, ///< Phy Status register
    };

    /** Socket registers */
    enum {
        Sn_MR     = 0x0400,  ///< Socket Mode register(R/W)
        Sn_CR     = 0x0401,  ///< Socket command register (R/W)
        Sn_IR     = 0x0402,  ///< Socket interrupt register (R)
        Sn_SR     = 0x0403,  ///< Socket status register (R)
        Sn_PORT   = 0x0404,  ///< Source port register (R/W)
        Sn_DHAR   = 0x0406,  ///< Peer MAC register address (R/W)
        Sn_DIPR   = 0x040C,  ///< Peer IP register address (R/W)
        Sn_DPORT  = 0x0410,  ///< Peer port register address (R/W)
        Sn_MSSR   = 0x0412,  ///< Maximum Segment Size(Sn_MSSR0) register address (R/W)
        Sn_PROTO  = 0x0414,  ///< IP Protocol(PROTO) Register (R/W)
        Sn_TOS    = 0x0415,  ///< IP Type of Service(TOS) Register (R/W)
        Sn_TTL    = 0x0416,  ///< IP Time to live(TTL) Register (R/W)
        Sn_TX_FSR = 0x0420,  ///< Transmit free memory size register (R)
        Sn_TX_RD  = 0x0422,  ///< Transmit memory read pointer register address (R)
        Sn_TX_WR  = 0x0424,  ///< Transmit memory write pointer register address (R/W)
        Sn_RX_RSR = 0x0426,  ///< Received data size register (R)
        Sn_RX_RD  = 0x0428,  ///< Read point of Receive memory (R/W)
        Sn_RX_WR  = 0x042A,  ///< Write point of Receive memory (R)
        Sn_MR2  = 0x042F,
    };

void switchToFastUDP(void)
{
    EthernetConnected = true;                     
    MyIP = eth.localIP();
    MyGate = eth.gatewayIP();
    MyNet = eth.subnetMask();
    eth.macAddress(MyMac);
    lwipPollingPeriod(10000);
    eth.end();
    FastUDPInit();
}

bool FastConnected(void)
{
   if(wizchip_read_reg8(PHYSR0) & 0x01)    //test for link up
   {
      return true;
   } 
   else
   {
      return false;
   }
}

//switch the Wiznet chip into 4 socket mode
void FastUDPInit(void)
{
    digitalWrite(WIZCHIPRS,LOW);
    delay(1);
    digitalWrite(WIZCHIPRS,HIGH);
    delay(70);

    wizchip_set_IP(SIPR , MyIP );         //Restore My IP Address
    wizchip_set_IP(SUBR , MyNet );        //Restore My Network Mask
    wizchip_set_IP(GAR , MyGate );        //Restore My Gateway Address
    wizchip_set_MAC(SHAR, MyMac);         //Restore My MAC
    wizchip_set_reg8(TMSR , 0x55);        //set TX buffers to 2K per socket
    wizchip_set_reg8(RMSR , 0x55);        //set TX buffers to 2K per socket
    uint8_t ready[] = "READY";
    FastUDPSend( 0, ControlIP , ControlPort , ControlDestPort, ready, 5);    //send a packet to initialise the socket
}


void FastUDPSend(uint8_t socket , IPAddress DestIP , unsigned int SourcePort , unsigned int DestPort , uint8_t * Buffer, uint16_t len)
{
   wizchip_set_reg8(Sn_MR + socket*0x100 , 0x02);                 // set socket to UDP mode
   wizchip_set_reg16(Sn_PORT + socket*0x100 , SourcePort);        // set Source Port
   wizchip_set_reg8(Sn_CR + socket*0x100 , 0x01);                 // set Open Command
   while(wizchip_read_reg8(Sn_CR + socket *0x100) != 0);          // wait for the command to complete  
   wizchip_set_IP(Sn_DIPR + socket*0x100 , DestIP);               // set Dest IP address
   wizchip_set_reg16(Sn_DPORT + socket*0x100 , DestPort);         // set destination port
   wizchip_set_reg16(Sn_TX_WR + socket*0x100 , 0);                // set TX Write pointer to start of buffer 
   wizchip_set_reg16(Sn_TX_RD + socket*0x100 , 0);                // set TX Write pointer to start of buffer 

   uint16_t add = 0x4000 + socket * 0x800;                  //calculate the Tx buffer address
   
   wizchip_write_buf(add , Buffer , len);                   //write the data to the Tx buffer
   wizchip_set_reg16(Sn_TX_WR + socket*0x100 , len);        // set TX Write
   wizchip_set_reg8(Sn_CR + socket*0x100 , 0x20);           // set send Command
   while(wizchip_read_reg8(Sn_CR + socket *0x100) != 0);    //wait for the command to complete
   UDPTransferInProgress[socket] = true;
}

bool FastUDPTransferBusy(uint8_t socket)
{
  if(UDPTransferInProgress[socket])
  {
    if((wizchip_read_reg8(Sn_IR + socket *0x100) & 0x18) != 0)      //test the transfer complete and timeout flag bits
     {
       wizchip_set_reg8(Sn_IR + socket*0x100 , 0x18);           // clear the flag bits
       UDPTransferInProgress[socket] = false;
     }
  }

  return UDPTransferInProgress[socket];
}

uint16_t FastUDPAvailable(uint8_t socket)
{
   uint16_t ret = wizchip_read_reg16(Sn_RX_RSR + socket * 0x100);
 
   if(ret > 8)              //allow for the 8 byte UDP header
   {
     return ret - 8;
   }
   else
   {
    return 0;
   }

}

uint16_t FastUDPRead(uint8_t socket, IPAddress * ip, uint16_t * port, uint8_t * buf)
{
  uint8_t header[8];
  uint16_t len = wizchip_read_reg16(Sn_RX_RSR + socket * 0x100);
  if(len > 8)                                                     //allow for the 8 byte UDP header
  {
    uint16_t add = 0x6000 + socket * 0x800;                       //calculate the Rx buffer address
    wizchip_read_buf(add , header , 8);                           //read the UDP Header from the chip
    wizchip_read_buf(add + 8 , buf , len - 8);                    //read the data portion from the chip
    wizchip_set_reg16(Sn_RX_RD + socket*0x100 , 0);               //zero the read pointer
    wizchip_set_reg16(Sn_RX_WR + socket*0x100 , 0);               //and the write pointer
    wizchip_set_reg8(Sn_CR + socket*0x100 , 0x40);               // set Read Command
    while(wizchip_read_reg8(Sn_CR + socket *0x100) != 0);         //wait for the command to complete
    *port = (header[4] <<8) + header[5];
    *ip = (header[3] << 24) + (header[2] << 16) +(header[1] << 8) + header[0];
    return len - 8;
  }
  else
  {
    return 0;
  }


}

void wizchip_set_reg8(uint16_t address, uint8_t val)
{
  uint8_t header[4];
     header[0] = 0xF0;
     header[1] = (address & 0xFF00) >> 8;
     header[2] = address & 0xFF;
     header[3] = val;

    digitalWrite(WIZCHIPCS,LOW);
    spi_write_blocking(spi0, header , 4);  
    digitalWrite(WIZCHIPCS,HIGH);
}

void wizchip_set_reg16(uint16_t address, uint16_t val)
{
  uint8_t header[5];
     header[0] = 0xF0;
     header[1] = (address & 0xFF00) >> 8;
     header[2] = address & 0xFF;
     header[3] = (val >> 8) & 0xFF;
     header[4] = val & 0xFF;

    digitalWrite(WIZCHIPCS,LOW);
    spi_write_blocking(spi0, header , 5);  
    digitalWrite(WIZCHIPCS,HIGH);
}

void wizchip_set_IP(uint16_t address, uint32_t val)
{
  uint8_t header[7];
     header[0] = 0xF0;
     header[1] = (address & 0xFF00) >> 8;
     header[2] = address & 0xFF;
     header[6] = (val >>24) & 0xFF;
     header[5] = (val >>16) & 0xFF;
     header[4] = (val >>8) & 0xFF;
     header[3] = val & 0xFF;

    digitalWrite(WIZCHIPCS,LOW);
    spi_write_blocking(spi0, header , 7);  
    digitalWrite(WIZCHIPCS,HIGH);
}

void wizchip_set_MAC(uint16_t address, uint8_t * val)
{
  uint8_t header[9];
     header[0] = 0xF0;
     header[1] = (address & 0xFF00) >> 8;
     header[2] = address & 0xFF;
     header[3] = val[0];
     header[4] = val[1];
     header[5] = val[2];
     header[6] = val[3];
     header[7] = val[4];
     header[8] = val[5];

    digitalWrite(WIZCHIPCS,LOW);
    spi_write_blocking(spi0, header , 9);  
    digitalWrite(WIZCHIPCS,HIGH);
}

uint8_t wizchip_read_reg8(uint16_t address)
{
    uint8_t header[3];
     header[0] = 0x0F;
     header[1] = (address & 0xFF00) >> 8;
     header[2] = address & 0xFF;

    digitalWrite(WIZCHIPCS,LOW);
    spi_write_blocking(spi0, header , 3); 
    uint8_t ret = SPI.transfer(0); 
    digitalWrite(WIZCHIPCS,HIGH);
    return ret;
}

uint16_t wizchip_read_reg16(uint16_t address)
{
    uint8_t header[3];
     header[0] = 0x0F;
     header[1] = (address & 0xFF00) >> 8;
     header[2] = address & 0xFF;

    digitalWrite(WIZCHIPCS,LOW);
    spi_write_blocking(spi0, header , 3); 
    uint16_t ret = ((SPI.transfer(0)) << 8) + SPI.transfer(0); 
    digitalWrite(WIZCHIPCS,HIGH);
    return ret;
}

void wizchip_write_buf(uint16_t address, uint8_t* pBuf, uint16_t len) 
{
  uint8_t header[3];
     header[0] = 0xF0;
     header[1] = (address & 0xFF00) >> 8;
     header[2] = address & 0xFF;

    digitalWrite(WIZCHIPCS,LOW);
    spi_write_blocking(spi0, header , 3);  
    spi_write_blocking(spi0, pBuf , len);  
    digitalWrite(WIZCHIPCS,HIGH);
}

void wizchip_read_buf(uint16_t address, uint8_t* pBuf, uint16_t len) 
{
  uint8_t header[3];
     header[0] = 0x0F;
     header[1] = (address & 0xFF00) >> 8;
     header[2] = address & 0xFF;

    digitalWrite(WIZCHIPCS,LOW);
    spi_write_blocking(spi0, header , 3);  
    spi_read_blocking(spi0, 0, pBuf , len);  
    digitalWrite(WIZCHIPCS,HIGH);
}

