/*****************************************************************************
*
*  spi.c - CC3000 Host Driver Implementation.
*  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
*
* Adapted for use with the Arduino/AVR by KTOWN (Kevin Townsend) 
* & Limor Fried for Adafruit Industries
* This library works with the Adafruit CC3000 breakout 
* ----> https://www.adafruit.com/products/1469
* Adafruit invests time and resources providing this open source code,
* please support Adafruit and open-source hardware by purchasing
* products from Adafruit!
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*****************************************************************************/
#include "cmsis.h"
#include "ccspi.h"
#include "cc3000_host_driver/hci.h"
#include "cc3000_host_driver/netapp.h"
#include "cc3000_host_driver/evnt_handler.h"
#include "cc3000_host_driver/cc3000_common.h"

extern uint8_t g_csPin, g_irqPin, g_vbatPin, g_IRQnum, g_SPIspeed;

#define READ                            (3)
#define WRITE                           (1)
#define HI(value)                       (((value) & 0xFF00) >> 8)
#define LO(value)                       ((value) & 0x00FF)
#define HEADERS_SIZE_EVNT               (SPI_HEADER_SIZE + 5)
#define SPI_HEADER_SIZE                 (5)

#define eSPI_STATE_POWERUP              (0)
#define eSPI_STATE_INITIALIZED          (1)
#define eSPI_STATE_IDLE                 (2)
#define eSPI_STATE_WRITE_IRQ            (3)
#define eSPI_STATE_WRITE_FIRST_PORTION  (4)
#define eSPI_STATE_WRITE_EOT            (5)
#define eSPI_STATE_READ_IRQ             (6)
#define eSPI_STATE_READ_FIRST_PORTION   (7)
#define eSPI_STATE_READ_EOT             (8)

#define WL_MOD_ENABLE_SHIFT             (1 << 13) // Mask to turn-on Wi-Fi module  
#define WL_MOD_ENABLE_OFF               (FPTA->PDOR &= ~WL_MOD_ENABLE_SHIFT)    
#define WL_MOD_ENABLE_ON                (FPTA->PDOR |=  WL_MOD_ENABLE_SHIFT)

// CC3000 chip select + SPI config
#define CC3000_ASSERT_CS { (FPTD->PDOR &= ~(1 << 0)); }
// CC3000 chip deselect + SPI restore
#define CC3000_DEASSERT_CS { (FPTD->PDOR |= (1 << 0)); }


/* smartconfig flags (defined in Adafruit_CC3000.cpp) */
// extern unsigned long ulSmartConfigFinished, ulCC3000DHCP;

typedef struct
{
  gcSpiHandleRx  SPIRxHandler;

  unsigned short usTxPacketLength;
  unsigned short usRxPacketLength;
  unsigned long  ulSpiState;
  unsigned char *pTxPacket;
  unsigned char *pRxPacket;

} tSpiInformation;

tSpiInformation sSpiInformation;

/* Static buffer for 5 bytes of SPI HEADER */
unsigned char tSpiReadHeader[] = {READ, 0, 0, 0, 0};

void SpiWriteDataSynchronous(unsigned char *data, unsigned short size);
void SpiWriteAsync(const unsigned char *data, unsigned short size);
void SpiPauseSpi(void);
void SpiResumeSpi(void);
void SSIContReadOperation(void);
void cc3k_int_poll(void);

// The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
// for the purpose of detection of the overrun. The location of the memory where the magic number
// resides shall never be written. In case it is written - the overrun occured and either recevie function
// or send function will stuck forever.
#define CC3000_BUFFER_MAGIC_NUMBER (0xDE)

char spi_buffer[CC3000_RX_BUFFER_SIZE];
unsigned char wlan_tx_buffer[CC3000_TX_BUFFER_SIZE];

static volatile char ccspi_is_in_irq = 0;
static volatile char ccspi_int_enabled = 0;

/* Mandatory functions are:
    - SpiOpen
    - SpiWrite
    - SpiRead
    - SpiClose
    - SpiResumeSpi
    - ReadWlanInterruptPin
    - WlanInterruptEnable
    - WlanInterruptDisable
    - WriteWlanPin
 */

void __delay_cycles(unsigned long cycles)
{
  volatile unsigned long delay;
  for (delay = 0; delay < (cycles / 4); delay++);
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SpiClose(void)
{
  printf("\tCC3000: SpiClose\r\n");
  
  if (sSpiInformation.pRxPacket)
  {
    sSpiInformation.pRxPacket = 0;
  }

  /*  Disable Interrupt in GPIOA module... */
  tSLInformation.WlanInterruptDisable();
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SpiOpen(gcSpiHandleRx pfRxHandler)
{
  printf("\tCC3000: SpiOpen\r\n");
  
  sSpiInformation.ulSpiState = eSPI_STATE_POWERUP;

  memset(spi_buffer, 0, sizeof(spi_buffer));
  memset(wlan_tx_buffer, 0, sizeof(spi_buffer));

  sSpiInformation.SPIRxHandler      = pfRxHandler;
  sSpiInformation.usTxPacketLength  = 0;
  sSpiInformation.pTxPacket         = NULL;
  sSpiInformation.pRxPacket         = (unsigned char *)spi_buffer;
  sSpiInformation.usRxPacketLength  = 0;
  
  spi_buffer[CC3000_RX_BUFFER_SIZE - 1]     = CC3000_BUFFER_MAGIC_NUMBER;
  wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] = CC3000_BUFFER_MAGIC_NUMBER;

  // /* Enable interrupt on the GPIO pin of WLAN IRQ */
  tSLInformation.WlanInterruptEnable();

  printf("\tCC3000: Finished SpiOpen\r\n");
}

/****************************************************************************/
//! SetupInterruptSystem
//! IRQ source is external pin, uses Kinetis-L NVIC Interrupt Controller
//! @param  None.
//! @return Status
/*****************************************************************************/
int SetupInterruptSystem()
{

  //  Configuring NVIC for PortD Ext interrupt requires writes to the following registers:
  //  PORTA_PCR16   (Port Control Register)
  //  NVIC_ICPR0    (NVIC Clear Pending Register)
  //  NVIC_ISER0    (NVIC Set Enable Register)
  //  NVIC_IPRx     (NVIC Interrupt Priority)

  // Initialization of Port Control registers
  // For new ("black") board set-up!!! PORTA_PCR16: ISF=0, IRQC=0x0A, MUX=1
  PORTA->PCR[16] &= ~PORT_PCR_MUX_MASK;    // PTA16 used as WL_SPI_IRQ (IRQ output from CC3000)
  PORTA->PCR[16] |= PORT_PCR_MUX(1);       // GPIO mode=alt1
  PORTA->PCR[16] |= PORT_PCR_IRQC(0x0A);   // 0x0A for interrupt on falling edge.
  PORTA->PCR[16] |= PORT_PCR_ISF_MASK;     // clear any pending interrupt.

  // NVIC_IPR31: PRI_30=0x80    Priority Reg
  NVIC_SetPriority(PORTA_IRQn, 0x1);

  return(0);
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
int init_spi(void)
{
  printf("\tCC3000: init_spi\r\n");
  
  // enable SPI0 Clock gate
  SIM->SCGC4 |= SIM_SCGC4_SPI0_MASK;
  // disable COP Watchdog
  SIM->COPC &= (~SIM_COPC_COPT_MASK);
  // enable all port Clock Gating Controls
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK|SIM_SCGC5_PORTB_MASK|SIM_SCGC5_PORTC_MASK|SIM_SCGC5_PORTD_MASK|SIM_SCGC5_PORTE_MASK;
  // disable SPI0 bus
  SPI0->C1 &= ~SPI_C1_SPE_MASK;
  // pin mux selection: Define the SPI lines:
  // SPI0_MOSI       PTD2      (J2-8)      Red
  // SPI0_MISO       PTD3      (J2-10)     Orange
  // SPI0K_SCK       PTC5      (J1-9)      Yellow
  // SPI0_PCS0       PTD0      (J2-6)      Green
  // WL_MOD_ENABLE   PTA13     (J2-2)
  // WL_SPI_IRQ      PTA16     (J1-2)
  PORTD->PCR[2] &= ~PORT_PCR_MUX_MASK;                 // SPI0_mosi (sout)
  PORTD->PCR[2] |= PORT_PCR_MUX(2)|PORT_PCR_DSE_MASK;  // PTD2 used as SPI0_MOSI, config pin for High Drive Strength

  PORTD->PCR[3] &= ~PORT_PCR_MUX_MASK;                 // SPI0_miso (sin)
  PORTD->PCR[3] |= PORT_PCR_MUX(2);                    // PTD3 used as SPI0_MISO

  PORTC->PCR[5] &= ~PORT_PCR_MUX_MASK;                 // SPI0_sck (sck)
  PORTC->PCR[5] |= PORT_PCR_MUX(2)|PORT_PCR_DSE_MASK;  // PTC5 = SPI0_SCK, config pin for High Drive Strength
  PORTD->PCR[0] &= ~PORT_PCR_MUX_MASK;                 // SPI0_cs  (psc0)
  PORTD->PCR[0] |= PORT_PCR_MUX(1)|PORT_PCR_DSE_MASK;  // PTD0 = User Controlled SPI0_PCS0, config pin for High Drive Strength
  FPTD->PDDR |= (1 << 0);                              // Set PTD0 to output

  // FDM: PTA16 used as WL_SPI_IRQ, IRQ output from CC3000
  // WL_SPI_IRQ interrupt is configured in SetupInterruptSystem()

  SPI0->C1 |= SPI_C1_MSTR_MASK;                      // SPI0 is spi master slave, bus clock is 12.5Mhz (0.08us)
  SPI0->BR  = 0x21;                                  // baud rate period = 1us (1Mbps)
  SPI0->C1 |= SPI_C1_SSOE_MASK;                      // auto slave-select OE

  SPI0->C2 &= (~SPI_C2_MODFEN_MASK);                 // disable SS function in MODFEN (for manual SS control)

  SPI0->C1 |=   SPI_C1_CPHA_MASK;                    // First edge on SPSCK at start of first data transfer cycle
  SPI0->C1 &= (~SPI_C1_CPOL_MASK);                   // Configures SPI clock as active-high

  SPI0->C1 &= (~SPI_C1_LSBFE_MASK);                  // SPI serial data transfers start with MSB
  SPI0->C1 |= SPI_C1_SPE_MASK;                       // enable SPI0

  SetupInterruptSystem();

  printf("\tCC3000: Finished init_spi\r\n");
  
  return(ESUCCESS);
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
long SpiFirstWrite(unsigned char *ucBuf, unsigned short usLength)
{
  printf("\tCC3000: SpiWriteFirst\r\n");
  
  /* Workaround for the first transaction */
  CC3000_ASSERT_CS;

  /* delay (stay low) for ~50us */
  __delay_cycles(2400);

  /* SPI writes first 4 bytes of data */
  SpiWriteDataSynchronous(ucBuf, 4);

  __delay_cycles(2400);

  SpiWriteDataSynchronous(ucBuf + 4, usLength - 4);

  /* From this point on - operate in a regular manner */
  sSpiInformation.ulSpiState = eSPI_STATE_IDLE;

  CC3000_DEASSERT_CS;

  return(0);
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
long SpiWrite(unsigned char *pUserBuffer, unsigned short usLength)
{
  unsigned char ucPad = 0;

  printf("\tCC3000: SpiWrite\r\n");
  
  /* Figure out the total length of the packet in order to figure out if there is padding or not */
  if(!(usLength & 0x0001))
  {
    ucPad++;
  }

  pUserBuffer[0] = WRITE;
  pUserBuffer[1] = HI(usLength + ucPad);
  pUserBuffer[2] = LO(usLength + ucPad);
  pUserBuffer[3] = 0;
  pUserBuffer[4] = 0;

  usLength += (SPI_HEADER_SIZE + ucPad);

  /* The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
   * for the purpose of overrun detection. If the magic number is overwritten - buffer overrun
   * occurred - and we will be stuck here forever! */
  if (wlan_tx_buffer[CC3000_TX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER)
  {
    printf("\tCC3000: Error - No magic number found in SpiWrite\r\n");
    while (1);
  }

  if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
  {
    while (sSpiInformation.ulSpiState != eSPI_STATE_INITIALIZED);
  }

  if (sSpiInformation.ulSpiState == eSPI_STATE_INITIALIZED)
  {
    /* This is time for first TX/RX transactions over SPI: the IRQ is down - so need to send read buffer size command */
    SpiFirstWrite(pUserBuffer, usLength);
  }
  else
  {
    /* We need to prevent here race that can occur in case two back to back packets are sent to the
     * device, so the state will move to IDLE and once again to not IDLE due to IRQ */
    tSLInformation.WlanInterruptDisable();

    while (sSpiInformation.ulSpiState != eSPI_STATE_IDLE);

    sSpiInformation.ulSpiState = eSPI_STATE_WRITE_IRQ;
    sSpiInformation.pTxPacket = pUserBuffer;
    sSpiInformation.usTxPacketLength = usLength;

    /* Assert the CS line and wait till SSI IRQ line is active and then initialize write operation */
    CC3000_ASSERT_CS;

    /* Re-enable IRQ - if it was not disabled - this is not a problem... */
    tSLInformation.WlanInterruptEnable();

    /* Check for a missing interrupt between the CS assertion and enabling back the interrupts */
    if (tSLInformation.ReadWlanInterruptPin() == 0)
    {
      SpiWriteDataSynchronous(sSpiInformation.pTxPacket, sSpiInformation.usTxPacketLength);

      sSpiInformation.ulSpiState = eSPI_STATE_IDLE;

      CC3000_DEASSERT_CS;
    }
  }

  /* Due to the fact that we are currently implementing a blocking situation
   * here we will wait till end of transaction */
  while (eSPI_STATE_IDLE != sSpiInformation.ulSpiState);

  return(0);
}

long TXBufferIsEmpty(void)
{
   return (SPI0->S & SPI_S_SPTEF_MASK);
}

long RXBufferIsFull(void)
{
   return (SPI0->S & SPI_S_SPRF_MASK);
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SpiWriteDataSynchronous(unsigned char *data, unsigned short size)
{
  printf("\tCC3000: SpiWriteDataSynchronous Start\r\n");

  unsigned short loc;
  for (loc = 0; loc < size; loc ++) 
  {
    printf(" 0x%X", data[loc]);
    while (!(TXBufferIsEmpty()));
    SPI0->D = data[loc];
    while (!(RXBufferIsFull()));
    SPI0->D;
  }
  printf("\r\n");
  
  printf("\tCC3000: SpiWriteDataSynchronous End\r\n");
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SpiReadDataSynchronous(unsigned char *data, unsigned short size)
{
  unsigned short i = 0;
  unsigned char *data_to_send = tSpiReadHeader;
  
  printf("\tCC3000: SpiReadDataSynchronous\r\n");  
  for (i = 0; i < size; i ++)
  {
    while (!(TXBufferIsEmpty()));
    SPI0->D = data_to_send[i];
    while (!(RXBufferIsFull()));
    data[i] = SPI0->D;
    printf(" 0x%X", data[i]);
  }
  printf("\r\n");
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SpiReadHeader(void)
{
  printf("\tCC3000: SpiReadHeader\r\n");

  SpiReadDataSynchronous(sSpiInformation.pRxPacket, HEADERS_SIZE_EVNT);
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
long SpiReadDataCont(void)
{
  long data_to_recv;
  unsigned char *evnt_buff, type;

  printf("\tCC3000: SpiReadDataCont\r\n");

  /* Determine what type of packet we have */
  evnt_buff =  sSpiInformation.pRxPacket;
  data_to_recv = 0;
  STREAM_TO_UINT8((uint8_t *)(evnt_buff + SPI_HEADER_SIZE), HCI_PACKET_TYPE_OFFSET, type);

  switch(type)
  {
    case HCI_TYPE_DATA:
      {
        /* We need to read the rest of data.. */
        STREAM_TO_UINT16((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_DATA_LENGTH_OFFSET, data_to_recv);
        if (!((HEADERS_SIZE_EVNT + data_to_recv) & 1))
        {
          data_to_recv++;
        }

        if (data_to_recv)
        {
          SpiReadDataSynchronous(evnt_buff + HEADERS_SIZE_EVNT, data_to_recv);
        }
        break;
      }
    case HCI_TYPE_EVNT:
      {
        /* Calculate the rest length of the data */
        STREAM_TO_UINT8((char *)(evnt_buff + SPI_HEADER_SIZE), HCI_EVENT_LENGTH_OFFSET, data_to_recv);
        data_to_recv -= 1;

        /* Add padding byte if needed */
        if ((HEADERS_SIZE_EVNT + data_to_recv) & 1)
        {
          data_to_recv++;
        }

        if (data_to_recv)
        {
          SpiReadDataSynchronous(evnt_buff + HEADERS_SIZE_EVNT, data_to_recv);
        }

        sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
        break;
      }
  }

  return (0);
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SpiPauseSpi(void)
{
  printf("\tCC3000: SpiPauseSpi\r\n");

  ccspi_int_enabled = 0;
  tSLInformation.WlanInterruptDisable();
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SpiResumeSpi(void)
{
  printf("\tCC3000: SpiResumeSpi\r\n");

  ccspi_int_enabled = 1;
  tSLInformation.WlanInterruptEnable();
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SpiTriggerRxProcessing(void)
{
  printf("\tCC3000: SpiTriggerRxProcessing\r\n");

  /* Trigger Rx processing */
  SpiPauseSpi();
  CC3000_DEASSERT_CS;

  /* The magic number that resides at the end of the TX/RX buffer (1 byte after the allocated size)
   * for the purpose of detection of the overrun. If the magic number is overriten - buffer overrun
   * occurred - and we will stuck here forever! */
  if (sSpiInformation.pRxPacket[CC3000_RX_BUFFER_SIZE - 1] != CC3000_BUFFER_MAGIC_NUMBER)
  {
    /* You've got problems if you're here! */
    printf("\tCC3000: ERROR - magic number missing!\r\n");
    while (1);
  }

  sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
  sSpiInformation.SPIRxHandler(sSpiInformation.pRxPacket + SPI_HEADER_SIZE);
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void SSIContReadOperation(void)
{
  printf("\tCC3000: SpiContReadOperation\r\n");
  
  /* The header was read - continue with  the payload read */
  if (!SpiReadDataCont())
  {
    /* All the data was read - finalize handling by switching to teh task
     *  and calling from task Event Handler */
    printf("\tSpiTriggerRxProcessing\r\n");
    SpiTriggerRxProcessing();
  }
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void WriteWlanPin( unsigned char val )
{  
  printf("\tCC3000: WriteWlanPin - %d\r\n", val);
  __delay_cycles(2400);
  if (val)
  {
    WL_MOD_ENABLE_ON;
  }
  else
  {
    WL_MOD_ENABLE_OFF;
  }
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
long ReadWlanInterruptPin(void)
{
  uint32_t r = (FPTA->PDIR & (1<<16));
  printf("\tCC3000: ReadWlanInterruptPin - %d\r\n", r);

  return (r);
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void WlanInterruptEnable()
{
  printf("\tCC3000: WlanInterruptEnable.\r\n");
  ccspi_int_enabled = 1;
  NVIC_EnableIRQ(PORTA_IRQn);
}

/**************************************************************************/
/*!

 */
/**************************************************************************/
void WlanInterruptDisable()
{
  printf("\tCC3000: WlanInterruptDisable\r\n");
  ccspi_int_enabled = 0;
  NVIC_DisableIRQ(PORTA_IRQn);
}

//*****************************************************************************
//
//! sendDriverPatch
//!
//! @param  pointer to the length
//!
//! @return none
//!
//! @brief  The function returns a pointer to the driver patch:
//!         since there is no patch in the host - it returns 0
//
//*****************************************************************************
char *sendDriverPatch(unsigned long *Length) {
  *Length = 0;
  return NULL;
}

//*****************************************************************************
//
//! sendBootLoaderPatch
//!
//! @param  pointer to the length
//!
//! @return none
//!
//! @brief  The function returns a pointer to the boot loader patch:
//!         since there is no patch in the host - it returns 0
//
//*****************************************************************************
char *sendBootLoaderPatch(unsigned long *Length) {
  *Length = 0;
  return NULL;
}

//*****************************************************************************
//
//! sendWLFWPatch
//!
//! @param  pointer to the length
//!
//! @return none
//!
//! @brief  The function returns a pointer to the FW patch:
//!         since there is no patch in the host - it returns 0
//
//*****************************************************************************
char *sendWLFWPatch(unsigned long *Length) {
  *Length = 0;
  return NULL;
}


/**************************************************************************/
/*!

 */
/**************************************************************************/

void SPI_IRQ(void)
{
  ccspi_is_in_irq = 1;

  printf("\tCC3000: Entering SPI_IRQ\r\n");

   // Clear pending interrupt
   PORTA->PCR[16] |= PORT_PCR_ISF_MASK;
   PORTA->ISFR  |= (1<<16);  
    
  if (sSpiInformation.ulSpiState == eSPI_STATE_POWERUP)
  {
    /* IRQ line was low ... perform a callback on the HCI Layer */
    sSpiInformation.ulSpiState = eSPI_STATE_INITIALIZED;
  }
  else if (sSpiInformation.ulSpiState == eSPI_STATE_IDLE)
  {
    sSpiInformation.ulSpiState = eSPI_STATE_READ_IRQ;    
   
    /* IRQ line goes down - start reception */
    CC3000_ASSERT_CS;

    // Wait for TX/RX Compete which will come as DMA interrupt
    SpiReadHeader();
    sSpiInformation.ulSpiState = eSPI_STATE_READ_EOT;
    
    SSIContReadOperation();
  }
  else if (sSpiInformation.ulSpiState == eSPI_STATE_WRITE_IRQ)
  {
    SpiWriteDataSynchronous(sSpiInformation.pTxPacket, sSpiInformation.usTxPacketLength);
    sSpiInformation.ulSpiState = eSPI_STATE_IDLE;
    CC3000_DEASSERT_CS;
  }

  printf("\tCC3000: Leaving SPI_IRQ\r\n");

  ccspi_is_in_irq = 0;
  return;
}

//*****************************************************************************
//
//!  cc3k_int_poll
//!
//!  \brief               checks if the interrupt pin is low
//!                       just in case the hardware missed a falling edge
//!                       function is in ccspi.cpp
//
//*****************************************************************************

void cc3k_int_poll()
{
  // if (digitalRead(g_irqPin) == LOW && ccspi_is_in_irq == 0 && ccspi_int_enabled != 0) {
  //   SPI_IRQ();
  // }
}
