#include "Atm_volume_sensor.hpp"

Atm_volume_sensor& Atm_volume_sensor::begin(int samplerate /* = 50 */) {
    const static state_t state_table[] PROGMEM = {
      /*              ON_ENTER    ON_LOOP  ON_EXIT  EVT_TRIGGER  EVT_TIMER   ELSE */
      /* IDLE   */          -1,        -1,      -1,          -1,   SAMPLE,    -1,
      /* SAMPLE */  ENT_SAMPLE,        -1,      -1,        SEND,       -1,  IDLE,
      /* SEND   */    ENT_SEND,        -1,      -1,          -1,       -1,  IDLE,
    };
    // clang-format on
    Machine::begin(state_table, ELSE);

    // The address can be changed making the option of connecting multiple devices
    ads.getAddr_ADS1115(ADS1115_DEFAULT_ADDRESS);   // 0x48, 1001 000 (ADDR = GND)
    // ads.getAddr_ADS1115(ADS1115_VDD_ADDRESS);    // 0x49, 1001 001 (ADDR = VDD)
    // ads.getAddr_ADS1115(ADS1115_SDA_ADDRESS);    // 0x4A, 1001 010 (ADDR = SDA)
    // ads.getAddr_ADS1115(ADS1115_SCL_ADDRESS);    // 0x4B, 1001 011 (ADDR = SCL)

    // The ADC gain (PGA), Device operating mode, Data rate
    // can be changed via the following functions

    ads.setGain(GAIN_TWO);          // 2x gain   +/- 2.048V  1 bit = 0.0625mV (default)
    //ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV
    // ads.setGain(GAIN_ONE);       // 1x gain   +/- 4.096V  1 bit = 0.125mV
    // ads.setGain(GAIN_FOUR);      // 4x gain   +/- 1.024V  1 bit = 0.03125mV
    // ads.setGain(GAIN_EIGHT);     // 8x gain   +/- 0.512V  1 bit = 0.015625mV
    // ads.setGain(GAIN_SIXTEEN);   // 16x gain  +/- 0.256V  1 bit = 0.0078125mV

    ads.setMode(MODE_CONTIN);       // Continuous conversion mode
    // ads.setMode(MODE_SINGLE);    // Power-down single-shot mode (default)

    // ads.setRate(RATE_128);          // 128SPS (default)
    // ads.setRate(RATE_8);         // 8SPS
    // ads.setRate(RATE_16);        // 16SPS
    ads.setRate(RATE_32);        // 32SPS
    // ads.setRate(RATE_64);        // 64SPS
    // ads.setRate(RATE_250);       // 250SPS
    // ads.setRate(RATE_475);       // 475SPS
    // ads.setRate(RATE_860);       // 860SPS

    ads.setOSMode(OSMODE_SINGLE);   // Set to start a single-conversion

    ads.begin();

    timer.set(samplerate);

    return *this;
  };


  int Atm_volume_sensor::event( int id ) {
  switch ( id ) {
    case EVT_TIMER:
      return timer.expired( this );
    case EVT_TRIGGER:
      return v_previous != v_sample;
  }
  return 0;
}

void Atm_volume_sensor::action( int id ) {
  switch ( id ) {
    case ENT_SAMPLE:
      v_previous = v_sample;
      v_sample = sample();
      return;
    case ENT_SEND:
      v_sample = sample();
      onchange.push( v_sample, v_sample > v_previous );
      return;
  }
}

Atm_volume_sensor& Atm_volume_sensor::range( int toLow, int toHigh ) {
  this->toLow = toLow;
  this->toHigh = toHigh;
  return *this;
}

Atm_volume_sensor& Atm_volume_sensor::set( int value ) {  // Dummy method
  return *this;
}

Atm_volume_sensor& Atm_volume_sensor::onChange( Machine& machine, int event /* = 0 */ ) {
  this->onchange.set( &machine, event );
  return *this;
}

Atm_volume_sensor& Atm_volume_sensor::onChange( atm_cb_push_t callback, int idx /* = 0 */ ) {
  onchange.set( callback, idx );
  return *this;
}

int Atm_volume_sensor::read_sample() {
  byte error;
  int8_t address;

  /*
  float pressure = map(voltage*100, 100, 500, 0, 3500);
  float water_height = pressure / 10 / 9.80665; // column heigh
  float volume = water_height * PI * (60*60); // H * PI * r^2
  */

  address = ads.ads_i2cAddress;
  // The i2c_scanner uses the return value of
  // the Write.endTransmisstion to see if
  // a device did acknowledge to the address.
  Wire.beginTransmission(address);
  error = Wire.endTransmission();
  if (error == 0)
  {
      int16_t adc0;

      //Serial.println("Getting Single-Ended Readings from AIN0..3");
      //Serial.println(" ");
      adc0 = ads.Measure_SingleEnded(0);
      //Serial.print("Digital Value of Analog Input at Channel 1: ");
      //Serial.println(adc0);
      // float mACurrent = adc0 * 0.000628;
      //Serial.print("Current Loop Input at Channel 1: ");
      //Serial.println(mACurrent, 3);

      // 6369 : 4mA
      // 6760 : atmo
      const float tank_radius = 4.5; // dm ... 93cm diam
      // const int ma_at_cylinder_bottom = 8140; // = 32 liters are contained in bottom part, not linear
      const int volume_offset = 810; // Account for non linear first 32 liters.

      double mv = adc0 * 0.0625;

      long int pascal = map(mv, 428, 2048, 0, 35000); // Map 428-2048mV to 0-35kpa
      double water_height = pascal / 9.80665; // column heigh in mm
      int volume = round((water_height * PI * powf(tank_radius, 2)) / 10.0) - volume_offset; // H * PI * r^2 ; in cl

      return volume;
  }
  else
  {
      //Serial.println("ADS1115 Disconnected!");
      return 0;
  }
}

int Atm_volume_sensor::avg() {
  uint16_t v = read_sample();
  avg_buf_total = avg_buf_total + v - avg_buf[avg_buf_head];
  avg_buf[avg_buf_head] = v;
  if ( avg_buf_head + 1 >= avg_buf_size ) {
    avg_buf_head = 0;
  } else {
    avg_buf_head++;
  }
  return avg_buf_total / avg_buf_size;
}

int Atm_volume_sensor::sample() {
  int v = avg_buf_size > 0 ? avg() : read_sample();
  return v;
}

int Atm_volume_sensor::state( void ) {
  return sample();
}

Atm_volume_sensor& Atm_volume_sensor::average( uint16_t* v, uint16_t size ) {
  avg_buf = v;
  avg_buf_size = size / sizeof( uint16_t );
  avg_buf_head = 0;
  avg_buf_total = 0;
  for ( uint16_t i = 0; i < avg_buf_size; i++ ) {
    avg_buf[i] = read_sample();
    avg_buf_total += avg_buf[i];
  }
  return *this;
}
