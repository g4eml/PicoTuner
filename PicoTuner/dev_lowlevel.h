/**
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


// Include descriptor struct definitions
#include "usb_common.h"
// USB register definitions from pico-sdk
#include "hardware/regs/usb.h"
// USB hardware struct definitions from pico-sdk
#include "hardware/structs/usb.h"
// For interrupt enable and numbers
#include "hardware/irq.h"
// For resetting the USB controller
#include "hardware/resets.h"

// Device descriptors
#include "dev_lowlevel_descriptors.h"

#define usb_hw_set ((usb_hw_t *)hw_set_alias_untyped(usb_hw))
#define usb_hw_clear ((usb_hw_t *)hw_clear_alias_untyped(usb_hw))

// Function prototypes for our device specific endpoint handlers defined
// later on
void ep0_in_handler(uint8_t *buf, uint16_t len);
void ep0_out_handler(uint8_t *buf, uint16_t len);
void ep81_in_handler(uint8_t *buf, uint16_t len);
void ep2_out_handler(uint8_t *buf, uint16_t len);
void ep83_in_handler(uint8_t *buf, uint16_t len);
void ep84_in_handler(uint8_t *buf, uint16_t len);

void sendTS2(bool sendheader);
void sendTS1(bool sendheader);

void clearBuffers(void);

unsigned long EP83Timeout;
#define EP83TO 16       //timeout value in ms for EP83 packets. Will send a zero lenght packet if this timeout has expired. 
unsigned long EP84Timeout;
#define EP84TO 16       //timeout value in ms for EP83 packets. Will send a zero lenght packet if this timeout has expired. 

// Global device address
static bool should_set_address = false;
static uint8_t dev_addr = 0;
static volatile bool configured = false;

// Global data buffer for EP0
static uint8_t ep0_buf[64];

//Global buffer for MPSSE commands from Host
static uint8_t commandBuf[1024];               
static uint16_t commandBufInPointer;
static uint16_t commandBufOutPointer;

//Global buffer for MPSSE results to Host
static uint8_t resultBuf[64];
static uint16_t resultBufCount;

//Global circular buffer for TS data to Host
#define TSBUFSIZE 512  
#define TSBUFNUM 20                             //This is the buffer size used by Longmynd It is Unlikely that we ever get near this value          
static uint8_t TS2Buf[TSBUFNUM][TSBUFSIZE];                //20 512 byte buffers. 
static uint16_t TS2BufInPointer;
static uint16_t TS2BufOutPointer;
static int TS2BufInNumber;
static int TS2BufOutNumber;
static uint8_t TS1Buf[TSBUFNUM][TSBUFSIZE];                //20 512 byte buffers. 
static uint16_t TS1BufInPointer;
static uint16_t TS1BufOutPointer;
static int TS1BufInNumber;
static int TS1BufOutNumber;
#define TSZLP 0
#define TSSTATUS 1
#define TSNORMAL 2


volatile bool TS2TransferInProgress = false;
volatile bool TS1TransferInProgress = false;
volatile bool TS2shortPacketSent = false;
volatile bool TS1shortPacketSent = false;

// Struct defining the device configuration
static struct usb_device_configuration dev_config = {
        .device_descriptor = &device_descriptor,
        .interface_descriptor1 = &interface_descriptor1,
        .interface_descriptor2 = &interface_descriptor2,
        .config_descriptor = &config_descriptor,
        .lang_descriptor = lang_descriptor,
        .descriptor_strings = descriptor_strings,
        .endpoints = {
                {
                        .descriptor = &ep0_out,
                        .handler = &ep0_out_handler,
                        .endpoint_control = NULL, // NA for EP0
                        .buffer_control = &usb_dpram->ep_buf_ctrl[0].out,
                        // EP0 in and out share a data buffer
                        .data_buffer = &usb_dpram->ep0_buf_a[0],
                },
                {
                        .descriptor = &ep0_in,
                        .handler = &ep0_in_handler,
                        .endpoint_control = NULL, // NA for EP0,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[0].in,
                        // EP0 in and out share a data buffer
                        .data_buffer = &usb_dpram->ep0_buf_a[0],
                },
                {
                        .descriptor = &ep81_in,
                        .handler = &ep81_in_handler,
                        // EP81 starts at offset 0 for endpoint control
                        .endpoint_control = &usb_dpram->ep_ctrl[0].in,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[1].in,
                        // First free EPX buffer
                        .data_buffer = &usb_dpram->epx_data[0 * 64],
                },
                {
                        .descriptor = &ep2_out,
                        .handler = &ep2_out_handler,
                        .endpoint_control = &usb_dpram->ep_ctrl[1].out,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[2].out,
                        // Second free EPX buffer
                        .data_buffer = &usb_dpram->epx_data[1 * 64],
                },
                {
                        .descriptor = &ep83_in,
                        .handler = &ep83_in_handler,
                        .endpoint_control = &usb_dpram->ep_ctrl[2].in,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[3].in,
                        // third free EPX buffer
                        .data_buffer = &usb_dpram->epx_data[2 * 64],
                },
                {
                        .descriptor = &ep84_in,
                        .handler = &ep84_in_handler,
                        .endpoint_control = &usb_dpram->ep_ctrl[3].in,
                        .buffer_control = &usb_dpram->ep_buf_ctrl[4].in,
                        // fourth free EPX buffer
                        .data_buffer = &usb_dpram->epx_data[3 * 64],
                }
        }
};

/**
 * @brief Given an endpoint address, return the usb_endpoint_configuration of that endpoint. Returns NULL
 * if an endpoint of that address is not found.
 *
 * @param addr
 * @return struct usb_endpoint_configuration*
 */
struct usb_endpoint_configuration *usb_get_endpoint_configuration(uint8_t addr) {
    struct usb_endpoint_configuration *endpoints = dev_config.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor && (endpoints[i].descriptor->bEndpointAddress == addr)) {
            return &endpoints[i];
        }
    }
    return NULL;
}

/**
 * @brief Given a C string, fill the EP0 data buf with a USB string descriptor for that string.
 *
 * @param C string you would like to send to the USB host
 * @return the length of the string descriptor in EP0 buf
 */
uint8_t usb_prepare_string_descriptor(const unsigned char *str) {
    // 2 for bLength + bDescriptorType + strlen * 2 because string is unicode. i.e. other byte will be 0
    uint8_t bLength = 2 + (strlen((const char *)str) * 2);
    static const uint8_t bDescriptorType = 0x03;

    volatile uint8_t *buf = &ep0_buf[0];
    *buf++ = bLength;
    *buf++ = bDescriptorType;

    uint8_t c;

    do {
        c = *str++;
        *buf++ = c;
        *buf++ = 0;
    } while (c != '\0');

    return bLength;
}

/**
 * @brief Take a buffer pointer located in the USB RAM and return as an offset of the RAM.
 *
 * @param buf
 * @return uint32_t
 */
static inline uint32_t usb_buffer_offset(volatile uint8_t *buf) {
    return (uint32_t) buf ^ (uint32_t) usb_dpram;
}

/**
 * @brief Set up the endpoint control register for an endpoint (if applicable. Not valid for EP0).
 *
 * @param ep
 */
void usb_setup_endpoint(const struct usb_endpoint_configuration *ep) {
    // EP0 doesn't have one so return if that is the case
    if (!ep->endpoint_control) {
        return;
    }

    // Get the data buffer as an offset of the USB controller's DPRAM
    uint32_t dpram_offset = usb_buffer_offset(ep->data_buffer);
    uint32_t reg = EP_CTRL_ENABLE_BITS
                   | EP_CTRL_INTERRUPT_PER_BUFFER
                   | (ep->descriptor->bmAttributes << EP_CTRL_BUFFER_TYPE_LSB)
                   | dpram_offset;
    *ep->endpoint_control = reg;
}

/**
 * @brief Set up the endpoint control register for each endpoint.
 *
 */
void usb_setup_endpoints() {
    const struct usb_endpoint_configuration *endpoints = dev_config.endpoints;
    for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
        if (endpoints[i].descriptor && endpoints[i].handler) {
            usb_setup_endpoint(&endpoints[i]);
        }
    }
}

/**
 * @brief Set up the USB controller in device mode, clearing any previous state.
 *
 */
void usb_device_init() {
    // Reset usb controller
    reset_block(RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

    // Clear any previous state in dpram just in case
    memset(usb_dpram, 0, sizeof(*usb_dpram)); // <1>

    // Enable USB interrupt at processor
    irq_set_enabled(USBCTRL_IRQ, true);

    // Mux the controller to the onboard usb phy
    usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS | USB_USB_MUXING_SOFTCON_BITS;

    // Force VBUS detect so the device thinks it is plugged into a host
    usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;

    // Enable the USB controller in device mode.
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;

    // Enable an interrupt per EP0 transaction
    usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS; // <2>

    // Enable interrupts for when a buffer is done, when the bus is reset,
    // and when a setup packet is received
    usb_hw->inte = USB_INTS_BUFF_STATUS_BITS |
                   USB_INTS_BUS_RESET_BITS |
                   USB_INTS_SETUP_REQ_BITS;

    // Set up endpoints (endpoint control registers)
    // described by device configuration
    usb_setup_endpoints();

    // Present full speed device by enabling pull up on DP
    usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
}

/**
 * @brief Given an endpoint configuration, returns true if the endpoint
 * is transmitting data to the host (i.e. is an IN endpoint)
 *
 * @param ep, the endpoint configuration
 * @return true
 * @return false
 */
static inline bool ep_is_tx(struct usb_endpoint_configuration *ep) {
    return ep->descriptor->bEndpointAddress & USB_DIR_IN;
}

/**
 * @brief Starts a transfer on a given endpoint.
 *
 * @param ep, the endpoint configuration.
 * @param buf, the data buffer to send. Only applicable if the endpoint is TX
 * @param len, the length of the data in buf (this example limits max len to one packet - 64 bytes)
 */
void usb_start_transfer(struct usb_endpoint_configuration *ep, uint8_t *buf, uint16_t len) {

  noInterrupts();
    // We are asserting that the length is <= 64 bytes for simplicity of the example.
    // For multi packet transfers see the tinyusb port.
    assert(len <= 64);

    // Prepare buffer control register value
    uint32_t val = len;

    if (ep_is_tx(ep)) {
        // Need to copy the data from the user buffer to the usb memory
        memcpy((void *) ep->data_buffer, (void *) buf, len);
        // Mark as full
        val |= USB_BUF_CTRL_FULL;
    }

    // Set pid and flip for next transfer
    val |= ep->next_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
    ep->next_pid ^= 1u;

    *ep->buffer_control = val;
    asm volatile("nop \n nop \n nop \n nop");                  //Tiny delay to ensure Controller has received the data                                   
    *ep->buffer_control = val | USB_BUF_CTRL_AVAIL;           //set the available bit as the last thing. (See RP2040 datasheet 4.1.2.5.1. for reason.)

    interrupts();
}

/**
 * @brief Send device descriptor to host
 *
 */
void usb_handle_device_descriptor(volatile struct usb_setup_packet *pkt) {
    const struct usb_device_descriptor *d = dev_config.device_descriptor;
    // EP0 in
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_IN_ADDR);
    // Always respond with pid 1
    ep->next_pid = 1;
    usb_start_transfer(ep, (uint8_t *) d, MIN(sizeof(struct usb_device_descriptor), pkt->wLength));
}

/**
 * @brief Send the configuration descriptor (and potentially the configuration and endpoint descriptors) to the host.
 *
 * @param pkt, the setup packet received from the host.
 */
void usb_handle_config_descriptor(volatile struct usb_setup_packet *pkt) {
    uint8_t *buf = &ep0_buf[0];

    // First request will want just the config descriptor
    const struct usb_configuration_descriptor *d = dev_config.config_descriptor;
    memcpy((void *) buf, d, sizeof(struct usb_configuration_descriptor));
    buf += sizeof(struct usb_configuration_descriptor);

    // If we more than just the config descriptor copy it all
    if (pkt->wLength >= d->wTotalLength) {
        memcpy((void *) buf, dev_config.interface_descriptor1, sizeof(struct usb_interface_descriptor));
        buf += sizeof(struct usb_interface_descriptor);
        const struct usb_endpoint_configuration *ep = dev_config.endpoints;

        // Copy 2 endpoint descriptors starting from EP81
        for (uint i = 2; i < 4; i++) {
            if (ep[i].descriptor) {
                memcpy((void *) buf, ep[i].descriptor, sizeof(struct usb_endpoint_descriptor));
                buf += sizeof(struct usb_endpoint_descriptor);
            }
        }
       memcpy((void *) buf, dev_config.interface_descriptor2, sizeof(struct usb_interface_descriptor));
        buf += sizeof(struct usb_interface_descriptor);

        // Copy 2 endpoint descriptors starting from EP83
        for (uint i = 4; i < 6; i++) {
            if (ep[i].descriptor) {
                memcpy((void *) buf, ep[i].descriptor, sizeof(struct usb_endpoint_descriptor));
                buf += sizeof(struct usb_endpoint_descriptor);
            }
        }
    }

    // Send data
    // Get len by working out end of buffer subtract start of buffer
    uint32_t len = (uint32_t) buf - (uint32_t) &ep0_buf[0];
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), &ep0_buf[0], MIN(len, pkt->wLength));
}

/**
 * @brief Handle a BUS RESET from the host by setting the device address back to 0.
 *
 */
void usb_bus_reset(void) {
    // Set address back to 0
    dev_addr = 0;
    should_set_address = false;
    usb_hw->dev_addr_ctrl = 0;
    configured = false;
}

/**
 * @brief Send the requested string descriptor to the host.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_handle_string_descriptor(volatile struct usb_setup_packet *pkt) {
    uint8_t i = pkt->wValue & 0xff;
    uint8_t len = 0;

    if (i == 0) {
        len = 4;
        memcpy(&ep0_buf[0], dev_config.lang_descriptor, len);
    } else {
        // Prepare fills in ep0_buf
        len = usb_prepare_string_descriptor(dev_config.descriptor_strings[i - 1]);
    }

    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), &ep0_buf[0], MIN(len, pkt->wLength));
}

/**
 * @brief Sends a zero length status packet back to the host.
 */
void usb_acknowledge_out_request(void) {
    usb_start_transfer(usb_get_endpoint_configuration(EP0_IN_ADDR), NULL, 0);
}

/**
 * @brief Handles a SET_ADDR request from the host. The actual setting of the device address in
 * hardware is done in ep0_in_handler. This is because we have to acknowledge the request first
 * as a device with address zero.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_set_device_address(volatile struct usb_setup_packet *pkt) {
    // Set address is a bit of a strange case because we have to send a 0 length status packet first with
    // address 0
    dev_addr = (pkt->wValue & 0xff);
    // Will set address in the callback phase
    should_set_address = true;
    usb_acknowledge_out_request();
}

/**
 * @brief Handles a SET_CONFIGRUATION request from the host. Assumes one configuration so simply
 * sends a zero length status packet back to the host.
 *
 * @param pkt, the setup packet from the host.
 */
void usb_set_device_configuration(volatile struct usb_setup_packet *pkt) {
    // Only one configuration so just acknowledge the request
    usb_acknowledge_out_request();
    configured = true;
}

/**
 * @brief Respond to a setup packet from the host.
 *
 */
void usb_handle_setup_packet(void) {
    volatile struct usb_setup_packet *pkt = (volatile struct usb_setup_packet *) &usb_dpram->setup_packet;
    uint8_t req_direction = pkt->bmRequestType;
    uint8_t req = pkt->bRequest;

    // Reset PID to 1 for EP0 IN
    usb_get_endpoint_configuration(EP0_IN_ADDR)->next_pid = 1u;

    if (req_direction == USB_DIR_OUT) 
    {
        if (req == USB_REQUEST_SET_ADDRESS) 
        {
            usb_set_device_address(pkt);
        } else if (req == USB_REQUEST_SET_CONFIGURATION) 
        {
            usb_set_device_configuration(pkt);
        } else 
        {
            usb_acknowledge_out_request();
        }
    } else if (req_direction == FTDI_DEVICE_OUT_REQTYPE)
    {
        clearBuffers();
        usb_acknowledge_out_request();
    
    } else if (req_direction == USB_DIR_IN) 
    {
        if (req == USB_REQUEST_GET_DESCRIPTOR) {
            uint16_t descriptor_type = pkt->wValue >> 8;

            switch (descriptor_type) {
                case USB_DT_DEVICE:
                    usb_handle_device_descriptor(pkt);
                    break;

                case USB_DT_CONFIG:
                    usb_handle_config_descriptor(pkt);
                    break;

                case USB_DT_STRING:
                    usb_handle_string_descriptor(pkt);;
                    break;

                default:
                      break;
            }
        } 
    }
}

/**
 * @brief Notify an endpoint that a transfer has completed.
 *
 * @param ep, the endpoint to notify.
 */
static void usb_handle_ep_buff_done(struct usb_endpoint_configuration *ep) {
    uint32_t buffer_control = *ep->buffer_control;
    // Get the transfer length for this endpoint
    uint16_t len = buffer_control & USB_BUF_CTRL_LEN_MASK;

    // Call that endpoints buffer done handler
    ep->handler((uint8_t *) ep->data_buffer, len);
}

/**
 * @brief Find the endpoint configuration for a specified endpoint number and
 * direction and notify it that a transfer has completed.
 *
 * @param ep_num
 * @param in
 */
static void usb_handle_buff_done(uint ep_num, bool in) {
    uint8_t ep_addr = ep_num | (in ? USB_DIR_IN : 0);
    for (uint i = 0; i < USB_NUM_ENDPOINTS; i++) {
        struct usb_endpoint_configuration *ep = &dev_config.endpoints[i];
        if (ep->descriptor && ep->handler) {
            if (ep->descriptor->bEndpointAddress == ep_addr) {
                usb_handle_ep_buff_done(ep);
                return;
            }
        }
    }
}

/**
 * @brief Handle a "buffer status" irq. This means that one or more
 * buffers have been sent / received. Notify each endpoint where this
 * is the case.
 */
static void usb_handle_buff_status() {
    uint32_t buffers = usb_hw->buf_status;
    uint32_t remaining_buffers = buffers;

    uint bit = 1u;
    for (uint i = 0; remaining_buffers && i < USB_NUM_ENDPOINTS * 2; i++) {
        if (remaining_buffers & bit) {
            // clear this in advance
            usb_hw_clear->buf_status = bit;
            // IN transfer for even i, OUT transfer for odd i
            usb_handle_buff_done(i >> 1u, !(i & 1u));
            remaining_buffers &= ~bit;
        }
        bit <<= 1u;
    }
}

/**
 * @brief USB interrupt handler
 *
 */
/// \tag::isr_setup_packet[]
void isr_usbctrl(void) {
    // USB interrupt handler
    uint32_t status = usb_hw->ints;
    uint32_t handled = 0;
    // Setup packet received
    if (status & USB_INTS_SETUP_REQ_BITS) {
        handled |= USB_INTS_SETUP_REQ_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
        usb_handle_setup_packet();
    }
/// \end::isr_setup_packet[]

    // Buffer status, one or more buffers have completed
    if (status & USB_INTS_BUFF_STATUS_BITS) {
        handled |= USB_INTS_BUFF_STATUS_BITS;
        usb_handle_buff_status();
    }

    // Bus is reset
    if (status & USB_INTS_BUS_RESET_BITS) {
        handled |= USB_INTS_BUS_RESET_BITS;
        usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
        usb_bus_reset();
    }

    if (status ^ handled) {
        panic("Unhandled IRQ 0x%x\n", (uint) (status ^ handled));
    }

}

/**
 * @brief EP0 in transfer complete. Either finish the SET_ADDRESS process, or receive a zero
 * length status packet from the host.
 *
 * @param buf the data that was sent
 * @param len the length that was sent
 */
void ep0_in_handler(uint8_t *buf, uint16_t len) {
    if (should_set_address) {
        // Set actual device address in hardware
        usb_hw->dev_addr_ctrl = dev_addr;
        should_set_address = false;
    } else {
        // Receive a zero length status packet from the host on EP0 OUT
        struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_OUT_ADDR);
        usb_start_transfer(ep, NULL, 0);
    }
}

void ep0_out_handler(uint8_t *buf, uint16_t len) 
{
      struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP0_OUT_ADDR);
      usb_start_transfer(ep, NULL, 0);
}

void sendResult(void)
{
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP81_IN_ADDR);
    usb_start_transfer(ep, resultBuf, resultBufCount);
}
// Device specific functions
//EP81 Send MPSSE results to Host. This function is called when the transfer has completed
void ep81_in_handler(uint8_t *buf, uint16_t len) 
{
        resultBufCount = 2;                   //reset the buffer allowing for the two byte header. 
}

//EP2 Receives MPSSE commands from Host and adds then to the circular command buffer
void ep2_out_handler(uint8_t *buf, uint16_t len) 
{
      for(int c = 0; c < len ; c++)
      {
        commandBuf[commandBufInPointer++] = buf[c];
        if(commandBufInPointer == 1024 ) commandBufInPointer = 0;
      }

     // Get ready to rx again from host
    usb_start_transfer(usb_get_endpoint_configuration(EP2_OUT_ADDR), NULL, 64);
}

int commandsAvailable(void)
{
  if(commandBufInPointer >= commandBufOutPointer)
  {
    return commandBufInPointer - commandBufOutPointer;
  }
  else
  {
    return commandBufInPointer + (1024 - commandBufOutPointer);
  }
}

int getNextCommand(void)
{
  int r = commandBuf[commandBufOutPointer++];
  if(commandBufOutPointer == 1024) commandBufOutPointer = 0;
  return r;
}

//returns the number of 512 byte TS2 buffers waiting to be sent. 
int TS2BufsAvailable(void)
{
    if(TS2BufInNumber >= TS2BufOutNumber)
    {
       return TS2BufInNumber - TS2BufOutNumber;
    }
    else
    {
      return TS2BufInNumber + (TSBUFNUM - TS2BufOutNumber);
    }

}

//returns the number of 512 byte TS1 buffers waiting to be sent. 
int TS1BufsAvailable(void)
{
    if(TS1BufInNumber >= TS1BufOutNumber)
    {
       return TS1BufInNumber - TS1BufOutNumber;
    }
    else
    {
      return TS1BufInNumber + (TSBUFNUM - TS1BufOutNumber);
    }

}

//sends the next 64 byte packet of the TS2 buffer
void sendTS2(int mode)
{
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP83_IN_ADDR);
    uint8_t sbuf[64];

    switch(mode) 
    {
      case 0:                                                            //send zero length packet
         usb_start_transfer(ep, sbuf, 0);                                //start the zero length transfer to signal buffer end
         TS2TransferInProgress = true;
         TS2shortPacketSent = true;
      break;

      case 1:                                                            //send the 2 byte status packet
        sbuf[0] =0x55;                                                   //just using 0x55 as a nominal value. 
        sbuf[1] =0x55;                                                   // send the two status bytes bytes
        usb_start_transfer(ep, sbuf, 2);                                 //start the transfer of the status bytes
        TS2TransferInProgress = true;
        TS2shortPacketSent = true;
      break;

      case 2:                                                            //send the full 64 byte packet
        memcpy(sbuf, &TS2Buf[TS2BufOutNumber][TS2BufOutPointer], 64);       // send 64 bytes of the buffer
        usb_start_transfer(ep, sbuf, 64);                                //start the transfer
        TS2TransferInProgress = true;
        TS2shortPacketSent = false;
        TS2BufOutPointer = TS2BufOutPointer + 64;
        if(TS2BufOutPointer >= TSBUFSIZE) 
          {
            TS2BufOutPointer = 0;
            TS2BufOutNumber++;
            if(TS2BufOutNumber >= TSBUFNUM) TS2BufOutNumber = 0;
          }
      break;
    }
    digitalWrite(LED,1);
    EP83Timeout = millis() + EP83TO;
}

//sends the next 64 byte packet of the TS1 buffer
void sendTS1(int mode)
{
    struct usb_endpoint_configuration *ep = usb_get_endpoint_configuration(EP84_IN_ADDR);
    uint8_t sbuf[64];

    switch(mode) 
    {
      case 0:                                                            //send zero length packet
         usb_start_transfer(ep, sbuf, 0);                                //start the zero length transfer to signal buffer end
         TS1TransferInProgress = true;
         TS1shortPacketSent = true;
      break;

      case 1:                                                            //send the 2 byte status packet
        sbuf[0] =0x55;                                                   //just using 0x55 as a nominal value. 
        sbuf[1] =0x55;                                                   // send the two status bytes bytes
        usb_start_transfer(ep, sbuf, 2);                                 //start the transfer of the status bytes
        TS1TransferInProgress = true;
        TS1shortPacketSent = true;
      break;

      case 2:                                                            //send the full 64 byte packet
        memcpy(sbuf, &TS1Buf[TS1BufOutNumber][TS1BufOutPointer], 64);       // send 64 bytes of the buffer
        usb_start_transfer(ep, sbuf, 64);                                //start the transfer
        TS1TransferInProgress = true;
        TS1shortPacketSent = false;
        TS1BufOutPointer = TS1BufOutPointer + 64;
        if(TS1BufOutPointer >= TSBUFSIZE) 
          {
            TS1BufOutPointer = 0;
            TS1BufOutNumber++;
            if(TS1BufOutNumber >= TSBUFNUM) TS1BufOutNumber = 0;
          }
      break;
    }

    EP84Timeout = millis() + EP84TO;
}

//EP83 Send TS2 Data To Host This function gets called when the transfer has completed. 
void ep83_in_handler(uint8_t *buf, uint16_t len) 
{
     digitalWrite(LED,0);
     if(TS2BufsAvailable() > 0)               //if we still have data avialable then continue the bulk transfer or send a ZLP if necessary
     {
      sendTS2(TSNORMAL);                      //send the next 64 bytes
     }
     else if(TS2shortPacketSent)                //we have already sent the short packet so we are finished
     {
         TS2TransferInProgress = false;       //nothing left to send so, terminate the bulk transfer. 
         TS2shortPacketSent = false;
     }
     else
     {
      sendTS2(TSZLP);                         //send a Zero Length packet to indicate the transfer has finished. 
     }
}

//EP84 Send TS1 Data To Host This function gets called when the transfer has completed. 
void ep84_in_handler(uint8_t *buf, uint16_t len) 
{
     if(TS1BufsAvailable() > 0)               //if we still have data avialable then continue the bulk transfer or send a ZLP if necessary
     {
      sendTS1(TSNORMAL);                      //send the next 64 bytes
     }
     else if(TS1shortPacketSent)                //we have already sent the short packet so we are finished
     {
         TS1TransferInProgress = false;       //nothing left to send so, terminate the bulk transfer. 
         TS1shortPacketSent = false;
     }
     else
     {
      sendTS1(TSZLP);                         //send a Zero Length packet to indicate the transfer has finished. 
     }

}
