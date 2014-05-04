#include "mbed.h"

#include "board.h"

#include "ccspi.h"

#include "cc3000_host_driver/cc3000_common.h"
#include "cc3000_host_driver/evnt_handler.h"
#include "cc3000_host_driver/hci.h"
#include "cc3000_host_driver/netapp.h"
#include "cc3000_host_driver/nvmem.h"
#include "cc3000_host_driver/security.h"
#include "cc3000_host_driver/socket.h"
#include "cc3000_host_driver/wlan.h"

// DigitalOut myled(LED1);

void CC3000_UsynchCallback(long lEventType, char * data, unsigned char length);

int main() {
  printf("Hello, CC3000!\r\n"); 

  RGB_LEDS_init();
  USR_LEDS_init();
  USR_GPIO_init();

  NVIC_SetPriority(SPI0_IRQn, 0x0);     // Wi-Fi SPI interrupt must be higher priority than SysTick
  NVIC_SetPriority(PORTA_IRQn, 0x1);

  /* Initialise the module */
  printf("Initializing...\r\n");

  init_spi();
  
  printf("init\r\n");
  wlan_init(CC3000_UsynchCallback, sendWLFWPatch, sendDriverPatch, 
    sendBootLoaderPatch, ReadWlanInterruptPin, WlanInterruptEnable, 
    WlanInterruptDisable, WriteWlanPin);
  printf("start\r\n");

  wlan_start(0);
  
  printf("ioctl\n\r");

  while(1) {
    // myled = 1;
    // wait(0.2);
    // myled = 0;
    // wait(0.2);
  }
}

void CC3000_UsynchCallback(long lEventType, char * data, unsigned char length)
{
  if (lEventType == HCI_EVNT_WLAN_ASYNC_SIMPLE_CONFIG_DONE)
  {
    // cc3000Bitset.set(CC3000BitSet::IsSmartConfigFinished);
  }

  if (lEventType == HCI_EVNT_WLAN_UNSOL_CONNECT)
  {
    // cc3000Bitset.set(CC3000BitSet::IsConnected);
  }

  if (lEventType == HCI_EVNT_WLAN_UNSOL_DISCONNECT)
  {
    // cc3000Bitset.reset(CC3000BitSet::IsConnected | CC3000BitSet::HasDHCP);
  }
  
  if (lEventType == HCI_EVNT_WLAN_UNSOL_DHCP)
  {
    // cc3000Bitset.set(CC3000BitSet::HasDHCP);
  }

  if (lEventType == HCI_EVENT_CC3000_CAN_SHUT_DOWN)
  {
    // cc3000Bitset.set(CC3000BitSet::OkToShutDown);
  }

  if (lEventType == HCI_EVNT_WLAN_ASYNC_PING_REPORT)
  {
    //PRINT_F("CC3000: Ping report\n\r");
    // pingReportnum++;
    // memcpy(&pingReport, data, length);
  }

  if (lEventType == HCI_EVNT_BSD_TCP_CLOSE_WAIT) {
    // uint8_t socketnum;
    // socketnum = data[0];
    // //PRINT_F("TCP Close wait #"); printDec(socketnum);
    // if (socketnum < MAX_SOCKETS)
    //   closed_sockets[socketnum] = true;
  }
}
