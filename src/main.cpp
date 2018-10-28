//#define USE_LCD
#define USE_COAP

#include <Arduino.h>
#include <avr/wdt.h>

#include <Automaton.h>

#ifdef USE_LCD
#include <menu.hpp>
#include <menuIO/chainStream.h>
#include <menuIO/U8x8Out.h>
#endif

#ifdef USE_COAP
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <ArduinoJson.h>

#include <coap.h>

#endif


#include "Atm_volume_sensor.hpp"

#ifdef USE_COAP
// Ethernet setup
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x40 };

// UDP and CoAP class
EthernetUDP Udp;
Coap coap(Udp);

#endif

#ifdef USE_LCD

// pin definitions
#define OLED_CS_PIN 13
#define OLED_DC_PIN 12
#define OLED_RST_PIN 11
#define OLED_MOSI_PIN 10
#define OLED_CLK_PIN 9

#define MAX_DEPTH 2

#endif

#define WATER_IN_RELAY_PIN 2
#define WATER_OUT_RELAY_PIN 3

#define BUTTON_PIN 8

Atm_volume_sensor volume_sensor;
Atm_led water_in_relay, water_out_relay;
Atm_controller filling_controller, transferring_controller;
Atm_encoder rotary;
Atm_button button;
Atm_bit filling, transferring;

// Global variables
int fill_target = 0;
int tx_amount = 0;

// Average buffer for sensor
uint16_t avgbuffer[16];

const int max_volume = 9000; // dL

enum error_no {X};

#ifdef USE_COAP

// CoAP server endpoint URL
callback callback_status(CoapPacket &packet, IPAddress ip, int port) {
  StaticJsonBuffer<200> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();
  root[String("volume")] = volume_sensor.state();
  root[String("filling")] = filling.state();
  root[String("filling_target")] = fill_target;
  root[String("transferring")] = transferring.state();
  root[String("transferring_amount")] = tx_amount;

  String answer_json;
  root.printTo(answer_json);

  coap.sendResponse(ip, port, packet.messageid, (char*)answer_json.c_str(), answer_json.length(), COAP_CONTENT, COAP_APPLICATION_JSON ,NULL, 0);
}

// CoAP server endpoint URL
callback callback_fill(CoapPacket &packet, IPAddress ip, int port) {

  char p[packet.payloadlen + 1];
  memcpy(p, packet.payload, packet.payloadlen);
  p[packet.payloadlen] = NULL;

  String message(p);

  int fill_to = message.toInt();
  if (fill_to > 0 and fill_to < max_volume) {
    fill_target = fill_to;
    filling.on();
    coap.sendResponse(ip, port, packet.messageid, NULL, 0, COAP_VALID, COAP_APPLICATION_JSON, NULL, 0);
  } else {
    fill_target = 0;
    filling.off();
    coap.sendResponse(ip, port, packet.messageid, NULL, 0, COAP_NOT_ACCEPTABLE, COAP_APPLICATION_JSON, NULL, 0);
  }
}


callback callback_transfer(CoapPacket &packet, IPAddress ip, int port) {

  char p[packet.payloadlen + 1];
  memcpy(p, packet.payload, packet.payloadlen);
  p[packet.payloadlen] = NULL;

  String message(p);

  int transfer_amount = message.toInt();
  if (transfer_amount > 0 and transfer_amount < volume_sensor.state()) {
    tx_amount = transfer_amount;
    transferring.on();
    coap.sendResponse(ip, port, packet.messageid, NULL, 0, COAP_VALID, COAP_APPLICATION_JSON, NULL, 0);
  } else {
    tx_amount = transfer_amount;
    transferring.off();
    coap.sendResponse(ip, port, packet.messageid, NULL, 0, COAP_NOT_ACCEPTABLE, COAP_APPLICATION_JSON, NULL, 0);
  }
}

#endif

#ifdef USE_LCD
// Display
U8X8_SSD1306_128X32_UNIVISION_4W_SW_SPI u8x8(OLED_CLK_PIN, OLED_MOSI_PIN, OLED_CS_PIN, OLED_DC_PIN, OLED_RST_PIN);

Menu::panel panels[] MEMMODE={{0,0,40,2}};
Menu::navNode* nodes[sizeof(panels)/sizeof(Menu::panel)];//navNodes to store navigation status
Menu::panelsList pList(panels, nodes, 1);//a list of panels and nodes

menuIn* inputsList[]={};
chainStream<0> in(inputsList);

MENU_OUTPUTS(out, MAX_DEPTH,
  U8X8_OUT(u8x8, {0, 0, 16, 4}),
  NONE
);

// Menu action handlers
void doFillTank(eventMask);
void doDrawError(eventMask);

MENU(fillMenu, "REMPLIR...", doNothing, noEvent, wrapStyle,
  FIELD(fill_target, "JUSQU'A: ", "L", 0, 900, 100, 10, doNothing, noEvent, noStyle),
  OP("GO!", doFillTank, enterEvent),
  EXIT("ANNULER")
);

MENU(txMenu, "TRANSFERER...", doNothing, noEvent, wrapStyle,
  FIELD(tx_amount, "QUANTITE: ", "L", 0, 900, 10, 1, doNothing, noEvent, noStyle),
  OP("GO!", doNothing, enterEvent),
  EXIT("ANNULER")
);

MENU(mainMenu, "HLT", doNothing, noEvent, wrapStyle,
  SUBMENU(fillMenu),
  SUBMENU(txMenu),
  EXIT("[QUITTER]")
);

NAVROOT(nav, mainMenu, MAX_DEPTH, in, out); //the navigation root object

// Notify we need to update display
void request_update_display(int idx, int v, int up) {
  nav.idleChanged = true;
}

result draw_filling(menuOut& o, idleEvent event) {
  char line1[32]; //, line2[20];

  sprintf(line1, "  %d/%dL", (int)(volume_sensor.state()/10.0), fill_target);
  u8x8.drawString(0, 0, "REMPLISSAGE...");
  u8x8.drawString(0, 3, line1);

  return proceed;
}

result draw_error(menuOut& o, idleEvent event) {
  u8x8.clear();
  u8x8.drawString(0, 1, "ERREUR:");

  return proceed;
}

void doFillTank(eventMask e) {
  filling.off();

  if (fill_target > max_volume) {
    nav.idleOn(draw_error);
  } else {
    filling.on();
    nav.idleOn(draw_filling);
  }
}

result draw_volume_big(menuOut& o, idleEvent event) {
  char line1[32]; //, line2[20];

  //float pressure = max(map(voltage*100, 100, 500, 0, 3500), 0);
  //float water_height = pressure / 10 / 9.80665; // column heigh
  //float volume = water_height * PI * (60*60); // H * PI * r^2

  int volume = volume_sensor.state();

  if (volume < 0) {
    u8x8.drawString(0, 0, "Erreur de sonde");
    return proceed;
  }

  //sprintf(line2, "Voltage: %s", String(voltage).c_str());
  sprintf(line1, "V: %3d.%02dL", volume/10, volume%10);

  u8x8.clear();
  u8x8.drawString(0, 0, line1);
  //u8x8.drawString(0, 1, line2);
  //u8x8.refreshDisplay();

  return proceed;
}

#endif // USE_LCD

void setup() {
  wdt_disable();

  // Start serial communication and set baud rate = 9600
  Serial.begin(9600);

#ifdef USE_LCD
  // OLED Display
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_saikyosansbold8_u);
#endif

#ifdef USE_COAP
  // Start the Ethernet connection and the server
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    // try to congifure using IP address instead of DHCP:
    // Ethernet.begin(mac, ip);
  }

  // CoAP callbacks
  coap.server(callback_status, "status");
  coap.server(callback_fill, "fill");
  coap.server(callback_transfer, "transfer");

  coap.start();

#endif // USE_COAP

  // Sensor reading
  volume_sensor.begin(10)
    .average(avgbuffer, sizeof(avgbuffer))
#ifdef USE_LCD
    .onChange(request_update_display)
#endif
    ;

  // Flags
  filling.begin()
    .onChange(true, water_in_relay, water_in_relay.EVT_ON)
    .onChange(true, transferring, transferring.EVT_OFF)
    .onChange(false, water_in_relay, water_in_relay.EVT_OFF)
    .off();

  transferring.begin()
    .onChange(true, water_out_relay, water_out_relay.EVT_ON)
    .onChange(true, filling, filling.EVT_OFF)
    .onChange(false, water_out_relay, water_out_relay.EVT_OFF)
    .off();

  // Controllers
  filling_controller.begin()
    .IF(volume_sensor, '+', (fill_target*10)) // fill target reached
    .OR(volume_sensor, '+', max_volume) // tank is full
    .onChange(true, filling, filling.EVT_OFF);

  transferring_controller.begin()
    .IF(volume_sensor, '<', 10) // empty tank
    .onChange(true, transferring, transferring.EVT_OFF);


  // Water in/out pump relay
  water_in_relay.begin(WATER_IN_RELAY_PIN, false).off();
  water_out_relay.begin(WATER_OUT_RELAY_PIN, true).off();

#ifdef USE_LCD
  // Trigger button
  button.begin(BUTTON_PIN)
    .debounce(100)
    .onPress( [] (int idx, int v, int up) {
      switch ( v ) {
        case 1: // short press
          nav.doNav(navCmd(enterCmd));
          return;
        case 2: // longpress
          nav.doNav(navCmd(escCmd));
          return;
        }
    })
    .longPress(2, 1000);

  rotary.begin(7, 6, 10)
    .onChange( [] ( int idx, int v, int up ) {
        nav.doNav(navCmd(up == 1 ? upCmd : downCmd));
      });


  nav.idleTask = draw_volume_big;
  nav.idleOn(draw_volume_big);

#endif // USE_LCD

  //  Re-enable watchdog
  delay(1000L);
  wdt_enable(WDTO_4S);
}


void loop() {
#ifdef USE_LCD
  nav.doOutput();
#endif // USE_LCD

#ifdef USE_COAP
  coap.loop();
#endif // USE_COAP

  // main logic
  automaton.run();

  // Reset watchdog
  wdt_reset();
}
