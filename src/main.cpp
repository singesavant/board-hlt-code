#include <Arduino.h>
#include <Automaton.h>

#include <menu.h>
#include <menuIO/chainStream.h>
#include <menuIO/U8x8Out.h>

#include "Atm_volume_sensor.hpp"

using namespace Menu;

// pin definitions
#define OLED_CS_PIN 13
#define OLED_DC_PIN 12
#define OLED_RST_PIN 11
#define OLED_MOSI_PIN 10
#define OLED_CLK_PIN 9

#define WATER_IN_RELAY_PIN 2
#define WATER_OUT_RELAY_PIN 3

#define MAX_DEPTH 2

Atm_volume_sensor volume_sensor;
Atm_led water_in_relay, water_out_relay;
Atm_controller filling_controller, overfilling_controller;
Atm_encoder rotary;
Atm_button button;
Atm_bit filling, transferring;

// Global variables
int fill_target = 0;
int tx_amount = 0;

const int max_volume = 9000; // dL

enum error_no {X};


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

uint16_t avgbuffer[16];

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

void setup() {
  // Start serial communication and set baud rate = 9600
  Serial.begin(9600);

  // OLED Display
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_saikyosansbold8_u);

  // Sensor reading
  volume_sensor.begin(10)
    //.average(avgbuffer, sizeof(avgbuffer))
    .onChange(request_update_display);

  // Flags
  filling.begin()
    .onChange(true, water_in_relay, water_in_relay.EVT_ON)
    .onChange(true, transferring, transferring.EVT_OFF)
    .onChange(false, water_in_relay, water_in_relay.EVT_OFF)
    .off();

  transferring.begin().
    .onChange(true, water_out_relay, water_out_relay.EVT_ON)
    .onChange(true, filling, filling.EVT_OFF)
    .onChange(false, water_out_relay, water_out_relay.EVT_OFF)
    .off();

  // Controllers
  filling_controller.begin()
    .IF(volume_sensor, '+', fill_target) // fill target reached
    .OR(volume_sensor, '+', max_volume) // tank is full
    .onChange(true, filling, filling.EVT_OFF);

  // Water in/out pump relay
  water_in_relay.begin(WATER_IN_RELAY_PIN, false).off();
  water_out_relay.begin(WATER_OUT_RELAY_PIN, true).off();

  // Add a comparator to make sure we don't overfill or try to transfer from an empty tank.

  // Trigger button
  button.begin(8)
    .debounce(100)
    .onPress( [] (int idx, int v, int up) {
      switch ( v ) {
        case 1:
          nav.doNav(navCmd(enterCmd));
          return;
        case 2:
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
}


void loop() {
  nav.doOutput();
  automaton.run();

/*
    // Start I2C Transmission
    Wire.beginTransmission(ADCAddr);
    // Select data register
    Wire.write(0x00);
    // Stop I2C Transmission
    Wire.endTransmission();

    // Request 2 bytes of data
    Wire.requestFrom(ADCAddr, 2);

    // Read 2 bytes of data
    // raw_adc msb, raw_adc lsb
    if (Wire.available() == 2)
    {
      data[0] = Wire.read();
      data[1] = Wire.read();
    }

    // Convert the data to 12-bits
    int raw_adc = (data[0] & 0x0F) * 256 + data[1];
    if (raw_adc > 2047)
    {
      raw_adc -= 4095;
    }



    // Output data to serial monitor
    //Serial.print("Digital value of Analog input : ");
    //Serial.println(raw_adc);
    delay(300);
    */

}
