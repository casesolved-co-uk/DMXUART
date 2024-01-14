// For testing DMXUART using two UARTs in loopback
// Output via Artnet over WiFi on universe 9

#include <DMXUART.h>
#include <espArtNetRDM.h>
#include "esp8266_peri.h"

#define BREAK_DETECT (USIS(0) & (1 << UIBD))
#define INT_STATUS USIS(0)

// remove standard Serial and Serial1
#define NO_GLOBAL_INSTANCES

#define ARTNET_OEM 0x0123    // Artnet OEM Code
#define ESTA_MAN 0x08DD      // ESTA Manufacturer Code
#define ART_FIRM_VERSION 0x0001   // Firmware given over Artnet (2 bytes)

#define DMXA_TXPIN  1
#define DMXA_DIRPIN 5
#define DMXB_TXPIN  2
#define DMXB_DIRPIN 16

esp8266ArtNetRDM artRDM;
DMXUART *DMXA;
uint8_t RXbuf[dmx_channels + 1] = {0}; // add 1 to reassemble the start byte
DMXUART *DMXB;
uint8_t TXbuf[dmx_channels] = {0};

IPAddress ip(2,0,0,1);
IPAddress subnet(255,0,0,0);
IPAddress broadcast;
uint8_t MAC_array[6];
char hotspotPass[] = "deadbeef";
char hotspotSSID[20];

uint8_t artGroup;
uint8_t artPort;


// call first
void setup_WiFi() {
  sprintf(hotspotSSID, "DMXUART_%05u", (ESP.getChipId() & 0xFFFF)); // 32-bit -> 5 digits or 99999 or <17-bits
  broadcast = (uint32_t) ip | ~((uint32_t) subnet);

  WiFi.persistent(false);    // prevent excessive writing to flash
  WiFi.hostname(hotspotSSID);
  // Must give MacOS a gateway (not 0.0.0.0) otherwise the wifi symbol keeps searching & doesn't look connected
  WiFi.softAPConfig(ip, ip, subnet);
  WiFi.softAP(hotspotSSID, hotspotPass);
  WiFi.macAddress(MAC_array);
}

void setup_ArtNet() {
  // init(IPAddress ip, IPAddress subnet, bool dhcp, const char* shortname, const char* longname, uint16_t oem, uint16_t esta, uint8_t* mac)
  artRDM.init(ip, subnet, true, hotspotSSID, hotspotSSID, ARTNET_OEM, ESTA_MAN, MAC_array);
  artRDM.setFirmwareVersion(ART_FIRM_VERSION);
  // addGroup(byte net, byte subnet)
  artGroup = artRDM.addGroup(0, 0); // higher bytes of the artnet address
  // addPort(byte group, byte port, byte universe, uint8_t type, bool mergehtp, byte* buf)
  artRDM.addPort(artGroup, 0, 9, DMX_IN);
}

void setup_DMX() {
  // DMXUART(int uart_nr, uint8_t* buf, int8_t tx_pin, int8_t dir_pin, bool invert, bool tx_mode)
  DMXA = new DMXUART(0, RXbuf, DMXA_TXPIN, DMXA_DIRPIN, true, false);
  DMXB = new DMXUART(1, TXbuf, DMXB_TXPIN, DMXB_DIRPIN, true, true);
}

void setup_data() {
  for (uint32_t i = 0; i < dmx_channels; i++) {
    TXbuf[i] = (uint8_t) i & 0xFF;
  }
}

void send_artnet(uint8_t* data, uint16_t chans) {
  artRDM.sendDMX(artGroup, artPort, broadcast, data, chans);
}


void setup() {
  setup_WiFi();
  setup_ArtNet();
  setup_data();
  setup_DMX();
  // wait for client to connect
  while(wifi_softap_get_station_num() == 0) {
    delay(5000);
  }
}

//void loop() {
//  send_artnet(TXbuf, 256);
//  delay(1000);
//}


void rx() {
  int result;
  int rx_start;

  result = DMXA->read(&rx_start);

  if (result > 0) {
    // For some reason our artnet library drops packets
    // You can tell from the sequence numbers which have large gaps in
    // We'll assume our DMX driver is okay then!
    send_artnet(RXbuf, result);
    // send the start byte afterwards
    RXbuf[0] = rx_start >=0 ? rx_start : -rx_start;
    RXbuf[1] = rx_start >=0 ? 0 : 1;
    // send_artnet(RXbuf, 2);
  } else {
    RXbuf[0] = (uint8_t) result + 5;
    send_artnet(RXbuf, 2);
  }
}

void loop() {
  uint8_t tx_start = 204; // RDM value = 204, for testing only

  DMXB->write(dmx_channels, tx_start); // our artnet library only supports even length frames

  delay(10);

  rx();
//  delay(1);
//  rx();
//  delay(2);
//  rx(); // returns zero as it should

  //send_artnet(TXbuf, dmx_channels); // debug

  for (uint32_t i = 0; i < dmx_channels; i++) {
    TXbuf[i] += 1;
  }

  delay(5);
}
