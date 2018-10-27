#pragma once

#include <Automaton.h>
#include <Wire.h>
#include "ADS1115.h"

class Atm_volume_sensor : public Machine {
public:
  enum { IDLE, SAMPLE, SEND };            // STATES
  enum { EVT_TRIGGER, EVT_TIMER, ELSE };  // EVENTS

  Atm_volume_sensor( void ) : Machine(){};
  Atm_volume_sensor& begin(int samplerate = 50 );
  Atm_volume_sensor& average( uint16_t* v, uint16_t size );
  int state( void );
  Atm_volume_sensor& range( int toLow, int toHigh );
  Atm_volume_sensor& onChange( Machine& machine, int event = 0 );
  Atm_volume_sensor& onChange( atm_cb_push_t callback, int idx = 0 );
  Atm_volume_sensor& set( int value );

 private:
  enum { ENT_SAMPLE, ENT_SEND };  // ACTIONS
  short pin;
  atm_timer_millis timer;
  int v_sample, v_threshold, v_previous;
  atm_connector onchange;
  uint16_t* avg_buf;
  uint16_t avg_buf_size;
  uint16_t avg_buf_head;
  uint32_t avg_buf_total;
  int toLow, toHigh;

  ADS1115 ads;

  int avg();
  int sample();
  virtual int read_sample();
  int event( int id );
  void action( int id );

};
