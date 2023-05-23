/*
NMEA2000_esp32.cpp

Copyright (c) 2015-2020 Timo Lappalainen, Kave Oy, www.kave.fi

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Inherited NMEA2000 object for ESP32 modules. See also NMEA2000 library.

Thanks to Thomas Barth, barth-dev.de, who has written ESP32 CAN code. To avoid extra
libraries, I implemented his code directly to the NMEA2000_esp32 to avoid extra
can.h library, which may cause even naming problem.
*/

#include "esp_idf_version.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
// enable support for ESP-IDF v5.0+
#include "esp_private/periph_ctrl.h"
#else
#include "driver/periph_ctrl.h"
#endif

#include "rom/gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/dport_reg.h"
#include "soc/dport_access.h"
#include "NMEA2000_esp32.h"

#if !defined(round)
#include <math.h>
#endif

bool tNMEA2000_esp32::CanInUse=false;
tNMEA2000_esp32 *pNMEA2000_esp32=0;

void ESP32Can1Interrupt(void *);

//*****************************************************************************
tNMEA2000_esp32::tNMEA2000_esp32(gpio_num_t _TxPin,  gpio_num_t _RxPin) :
    tNMEA2000(), IsOpen(false),
                                                                         speed(CAN_SPEED_250KBPS), TxPin(_TxPin), RxPin(_RxPin),
    RxQueue(NULL), TxQueue(NULL) {
}

//*****************************************************************************
bool tNMEA2000_esp32::CANSendFrame(unsigned long id, unsigned char len, const unsigned char *buf, bool /*wait_sent*/) {
  if ( uxQueueSpacesAvailable(TxQueue)==0 ) return false; // can not send to queue

  tCANFrame frame;
  frame.id=id;
  frame.len=len>8?8:len;
  memcpy(frame.buf,buf,len);

  xQueueSendToBack(TxQueue,&frame,0);  // Add frame to queue
  if ( MODULE_CAN->SR.B.TBS==0 ) return true; // Currently sending, ISR takes care of sending

  if ( MODULE_CAN->SR.B.TBS==1 ) { // Check again and restart send, if is not going on
    xQueueReceive(TxQueue,&frame,0);
    CAN_send_frame(frame);
  }

  return true;
}

//*****************************************************************************
void tNMEA2000_esp32::InitCANFrameBuffers() {
    if (MaxCANReceiveFrames<10 ) MaxCANReceiveFrames=50; // ESP32 has plenty of RAM
    if (MaxCANSendFrames<10 ) MaxCANSendFrames=40;
    uint16_t CANGlobalBufSize=MaxCANSendFrames-4;
    MaxCANSendFrames=4;  // we do not need much libary internal buffer since driver has them.
    RxQueue=xQueueCreate(MaxCANReceiveFrames,sizeof(tCANFrame));
    TxQueue=xQueueCreate(CANGlobalBufSize,sizeof(tCANFrame));

  tNMEA2000::InitCANFrameBuffers(); // call main initialization
}

//*****************************************************************************
bool tNMEA2000_esp32::CANOpen() {
    if (IsOpen) return true;

    if (CanInUse) return false; // currently prevent accidental second instance. Maybe possible in future.

    pNMEA2000_esp32=this;
    IsOpen=true;
  CAN_init();

    CanInUse=IsOpen;

  return IsOpen;
}

//*****************************************************************************
bool tNMEA2000_esp32::CANGetFrame(unsigned long &id, unsigned char &len, unsigned char *buf) {
  bool HasFrame=false;
  tCANFrame frame;

    //receive next CAN frame from queue
    if ( xQueueReceive(RxQueue,&frame, 0)==pdTRUE ) {
      HasFrame=true;
      id=frame.id;
      len=frame.len;
      memcpy(buf,frame.buf,frame.len);
  }

  return HasFrame;
}

//*****************************************************************************
void tNMEA2000_esp32::CAN_init() {

	//Time quantum
  double __tq;

  // A soft reset of the ESP32 leaves it's CAN/TWAI controller in an undefined state so a reset is needed.
  // Reset CAN/TWAI controller to same state as it would be in after a power down reset.
  periph_module_reset(PERIPH_TWAI_MODULE);

  // enable module
  DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_TWAI_CLK_EN);
  DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_TWAI_RST);

  // configure RX pin
  gpio_set_direction(RxPin, GPIO_MODE_INPUT);
  gpio_matrix_in(RxPin, TWAI_RX_IDX, 0);
  gpio_pad_select_gpio(RxPin);

    //set to PELICAN mode
	MODULE_CAN->CDR.B.CAN_M=0x1;

	//synchronization jump width is the same for all baud rates
	MODULE_CAN->BTR0.B.SJW		=0x1;

	//TSEG2 is the same for all baud rates
	MODULE_CAN->BTR1.B.TSEG2	=0x1;

	//select time quantum and set TSEG1
	switch (speed) {
  case CAN_SPEED_1000KBPS:
			MODULE_CAN->BTR1.B.TSEG1	=0x4;
    __tq = 0.125;
    break;

  case CAN_SPEED_800KBPS:
			MODULE_CAN->BTR1.B.TSEG1	=0x6;
    __tq = 0.125;
    break;
  default:
    MODULE_CAN->BTR1.B.TSEG1	=0xc;
    __tq = ((float)1000 / (float)speed) / 16;
  }

	//set baud rate prescaler
	MODULE_CAN->BTR0.B.BRP=(uint8_t)round((((APB_CLK_FREQ * __tq) / 2) - 1)/1000000)-1;

  /* Set sampling
   * 1 -> triple; the bus is sampled three times; recommended for low/medium speed buses     (class A and B) where filtering spikes on the bus line is beneficial
   * 0 -> single; the bus is sampled once; recommended for high speed buses (SAE class C)*/
    MODULE_CAN->BTR1.B.SAM	=0x1;

    //enable all interrupts
    MODULE_CAN->IER.U = 0xef;  // bit 0x10 contains Baud Rate Prescaler Divider (BRP_DIV) bit

    //no acceptance filtering, as we want to fetch all messages
  MODULE_CAN->MBX_CTRL.ACC.CODE[0] = 0;
  MODULE_CAN->MBX_CTRL.ACC.CODE[1] = 0;
  MODULE_CAN->MBX_CTRL.ACC.CODE[2] = 0;
  MODULE_CAN->MBX_CTRL.ACC.CODE[3] = 0;
  MODULE_CAN->MBX_CTRL.ACC.MASK[0] = 0xff;
  MODULE_CAN->MBX_CTRL.ACC.MASK[1] = 0xff;
  MODULE_CAN->MBX_CTRL.ACC.MASK[2] = 0xff;
  MODULE_CAN->MBX_CTRL.ACC.MASK[3] = 0xff;

    //set to normal mode
    MODULE_CAN->OCR.B.OCMODE=__CAN_OC_NOM;

    //clear error counters
  MODULE_CAN->TXERR.U = 0;
  MODULE_CAN->RXERR.U = 0;
  (void)MODULE_CAN->ECC;

    //clear interrupt flags
  (void)MODULE_CAN->IR.U;

  //install CAN ISR
  esp_intr_alloc(ETS_TWAI_INTR_SOURCE, 0, ESP32Can1Interrupt, NULL, NULL);

  //configure TX pin
  // We do late configure, since some initialization above caused CAN Tx flash
  // shortly causing one error frame on startup. By setting CAN pin here
  // it works right.
  gpio_set_direction(TxPin, GPIO_MODE_OUTPUT);
  gpio_matrix_out(TxPin, TWAI_TX_IDX, 0, 0);
  gpio_pad_select_gpio(TxPin);

    //Showtime. Release Reset Mode.
  MODULE_CAN->MOD.B.RM = 0;
}

//*****************************************************************************
void tNMEA2000_esp32::CAN_read_frame() {
  tCANFrame frame;
  CAN_FIR_t FIR;

	//get FIR
	FIR.U=MODULE_CAN->MBX_CTRL.FCTRL.FIR.U;
  frame.len=FIR.B.DLC>8?8:FIR.B.DLC;

  // Handle only extended frames
  if (FIR.B.FF==CAN_frame_ext) {  //extended frame
    //Get Message ID
    frame.id = _CAN_GET_EXT_ID;

    //deep copy data bytes
    for( size_t i=0; i<frame.len; i++ ) {
      frame.buf[i]=MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.data[i];
    }

    //send frame to input queue
    xQueueSendToBackFromISR(RxQueue,&frame,0);
  }

  //Let the hardware know the frame has been read.
  MODULE_CAN->CMR.B.RRB=1;
}

//*****************************************************************************
void tNMEA2000_esp32::CAN_send_frame(tCANFrame &frame) {
  CAN_FIR_t FIR;

  FIR.U=0;
  FIR.B.DLC=frame.len>8?8:frame.len;
  FIR.B.FF=CAN_frame_ext;

	//copy frame information record
	MODULE_CAN->MBX_CTRL.FCTRL.FIR.U=FIR.U;

  //Write message ID
  _CAN_SET_EXT_ID(frame.id);

  // Copy the frame data to the hardware
  for ( size_t i=0; i<frame.len; i++) {
    MODULE_CAN->MBX_CTRL.FCTRL.TX_RX.EXT.data[i]=frame.buf[i];
  }

  // Transmit frame
  MODULE_CAN->CMR.B.TR=1;
}

//*****************************************************************************
void tNMEA2000_esp32::InterruptHandler() {
	//Interrupt flag buffer
  uint32_t interrupt;

  // Read interrupt status and clear flags
  interrupt = (MODULE_CAN->IR.U & 0xff);

  // Handle TX complete interrupt
    if ((interrupt & __CAN_IRQ_TX) != 0) {
    tCANFrame frame;
      if ( (xQueueReceiveFromISR(TxQueue,&frame,NULL)==pdTRUE) ) {
      CAN_send_frame(frame);
    }
  }

  // Handle RX frame available interrupt
    if ((interrupt & __CAN_IRQ_RX) != 0) {
    CAN_read_frame();
  }

  // Handle error interrupts.
    if ((interrupt & (__CAN_IRQ_ERR						//0x4
                      | __CAN_IRQ_DATA_OVERRUN			//0x8
                      | __CAN_IRQ_WAKEUP				//0x10
                      | __CAN_IRQ_ERR_PASSIVE			//0x20
                      | __CAN_IRQ_ARB_LOST				//0x40
                      | __CAN_IRQ_BUS_ERR				//0x80
        )) != 0) {
    /*handler*/
  }
}

//*****************************************************************************
void ESP32Can1Interrupt(void *) {
  pNMEA2000_esp32->InterruptHandler();
}
