#include <avr/wdt.h>

// #include "PinChangeInterrupt.h"

#include "AceBMS.h"
#include "AceDump.h"
#include "AcePump.h"
#include "TinBus.h"
#include "ntc.h"

#define VMAX (27000)
#define VMIN (26000)
#define VSET (26800)

#define OFF_TIME (180000) // 180000  x 10 ms = 30 minutes
#define OFF_START (1000) // 100  x 10 ms = 10 seconds

// ATMEL ATMEGA8 & 168 / ARDUINO
//
//                  +-\/-+
//            PC6  1|    |28  PC5 (AI 5)
//      (D 0) PD0  2|    |27  PC4 (AI 4)
//      (D 1) PD1  3|    |26  PC3 (AI 3)
//      (D 2) PD2  4|    |25  PC2 (AI 2)
// PWM+ (D 3) PD3  5|    |24  PC1 (AI 1)
//      (D 4) PD4  6|    |23  PC0 (AI 0)
//            VCC  7|    |22  GND
//            GND  8|    |21  AREF
//            PB6  9|    |20  AVCC
//            PB7 10|    |19  PB5 (D 13)
// PWM+ (D 5) PD5 11|    |18  PB4 (D 12)
// PWM+ (D 6) PD6 12|    |17  PB3 (D 11) PWM
//      (D 7) PD7 13|    |16  PB2 (D 10) PWM
//      (D 8) PB0 14|    |15  PB1 (D 9) PWM
//                  +----+

#define PIN_LED_AMBER (9)
#define PIN_AUX_POWER (8)

#define PIN_INVERTER_SSR (6)
#define PIN_PUMP_SSR (7)

#define PIN_POWER_PULSE (3)

#define TEMP_SLAB_TOP (0)
#define TEMP_SLAB_BOT (1)
#define TEMP_TANK_TOP (2)
#define TEMP_TANK_BOT (3)

#define PIN_TEMP_SLAB_TOP (A0)
#define PIN_TEMP_SLAB_BOT (A1)
#define PIN_TEMP_TANK_TOP (A2)
#define PIN_TEMP_TANK_BOT (A3)

#define ADC_COUNT (4)

static uint8_t ADCIndex = 0;
static uint16_t ADCFilter[ADC_COUNT] = {0};
static int16_t temperature[ADC_COUNT] = {0};
static uint8_t ADCPins[ADC_COUNT] = {PIN_TEMP_SLAB_TOP, PIN_TEMP_SLAB_BOT,
                                     PIN_TEMP_TANK_TOP, PIN_TEMP_TANK_BOT};

#define MICROS_IN_HOUR (3600000000L)
#define MICROS_IN_100MS (100000L)

#define PULSES_PER_WH (1L)

#define PIN_RX_INTERRUPT (2)
void busCallback(unsigned char *data, unsigned char length);
TinBus tinBus(Serial, ACEBMS_BAUD, PIN_RX_INTERRUPT, busCallback);

static unsigned long lastBMSUpdate = 0;
static uint16_t batmv = 0;
static uint16_t vindv = 0;
static uint16_t setmv = VSET;
static uint16_t ssrOffTimer = OFF_START;
static uint16_t power = 0;
static uint16_t energy = 0;
static unsigned long pulseCount = 0;

static int16_t amberTimer = 0;

static volatile unsigned long powerMicros = 0;
void powerInterrupt(void) {powerMicros = micros() | 1L; }

void setup() {
  wdt_disable();

  delay(2000);
  wdt_enable(WDTO_250MS);

  attachInterrupt(digitalPinToInterrupt(PIN_POWER_PULSE), powerInterrupt,
                  RISING);

  digitalWrite(PIN_LED_AMBER, LOW);
  pinMode(PIN_LED_AMBER, OUTPUT);

  digitalWrite(PIN_AUX_POWER, HIGH);
  pinMode(PIN_AUX_POWER, OUTPUT);

  digitalWrite(PIN_INVERTER_SSR, LOW);
  pinMode(PIN_INVERTER_SSR, OUTPUT);

  digitalWrite(PIN_PUMP_SSR, LOW);
  pinMode(PIN_PUMP_SSR, OUTPUT);

  tinBus.begin();
  noInterrupts();
  powerMicros = 0;
  interrupts();
  lastBMSUpdate = micros();
}

void update_adc(void) {
  if (++ADCIndex >= ADC_COUNT)
    ADCIndex = 0;
  uint16_t filter = ADCFilter[ADCIndex];
  filter -= (filter >> 4);
  filter += analogRead(ADCPins[ADCIndex]);
  ADCFilter[ADCIndex] = filter;
  temperature[ADCIndex] = ntc_getDeciCelcius(filter >> 4);
}

void update_power(unsigned long usNow) {
  static unsigned long pulseLastTime = 0;
  static unsigned long pulseDelta = 0;

  noInterrupts();
  unsigned long pulseTime = powerMicros;
  interrupts();

  energy = SIG_DIVU16BY10(pulseCount / PULSES_PER_WH); // divide by 10

  if (pulseTime != pulseLastTime) { // check for energy pulse
    if (pulseLastTime != 0) {
      unsigned long delta = pulseTime - pulseLastTime;
      if ((delta > MICROS_IN_100MS) &&
          (delta < MICROS_IN_HOUR)) { // 60 minutes max
        pulseCount++;
        pulseDelta = delta;
        power = (MICROS_IN_HOUR / PULSES_PER_WH) / delta;
        pulseLastTime = pulseTime;
      }
    } else {
      pulseLastTime = pulseTime;
    }

  } else {
    if ((pulseLastTime != 0) && (pulseDelta != 0)) {
      unsigned long delta = usNow - pulseLastTime;
      if (delta > pulseDelta) {
        if (delta < MICROS_IN_HOUR) { // 60 minutes max
          power = (MICROS_IN_HOUR / PULSES_PER_WH) / delta;
        } else {
          power = 0;
          pulseLastTime = 0;
        }
      }
    }
  }
}

void update_10ms(unsigned long time) {

  update_adc(); // update solar input voltage
  update_power(time);

  if ((time - lastBMSUpdate > 1000000L) && (lastBMSUpdate < time)) {
    ssrOffTimer = OFF_TIME;
  }
  if (batmv > setmv) {
    ssrOffTimer = OFF_TIME;
  }
  if (ssrOffTimer) {
    ssrOffTimer--;
    digitalWrite(PIN_INVERTER_SSR, LOW);
  } else {
    digitalWrite(PIN_INVERTER_SSR, HIGH);
  }

  if(temperature[TEMP_TANK_TOP] > 350){
    digitalWrite(PIN_PUMP_SSR, HIGH);
  }
  if(temperature[TEMP_TANK_TOP] < 300){
    digitalWrite(PIN_PUMP_SSR, LOW);
  }

  if (amberTimer > 0) {
    amberTimer -= 10;
    digitalWrite(PIN_LED_AMBER, HIGH);
  } else {
    digitalWrite(PIN_LED_AMBER, LOW);
  }
}

void loop() {
  static unsigned long time = 0;

  tinBus.update();

  unsigned long now = micros();
  if (now - time >= 10000L) {
    time = now;
    update_10ms(now);
    wdt_reset();
  }
}

void busCallback(unsigned char *data, unsigned char length) {
  msg_t *msg = (msg_t *)data;
  uint16_t value;
  if (sig_decode(msg, ACEBMS_VBAT, &value) != FMT_NULL) {
    batmv = value * 10;
    lastBMSUpdate = micros();
  }
  if (sig_decode(msg, ACEBMS_RQST, &value) != FMT_NULL) {
    if ((value & 0x0003) == 0)
      amberTimer = 50; // show rx data
    uint8_t frameSequence = value;
    if ((frameSequence & 0x0F) == (SIG_MSG_ID(ACEPUMP_STATUS) & 0x0F)) {
      msg_t txMsg;
      sig_encode(&txMsg, ACEPUMP_VSET, SIG_DIVU16BY10(setmv));
      sig_encode(&txMsg, ACEPUMP_PPV, power);
      sig_encode(&txMsg, ACEPUMP_PUMP, digitalRead(PIN_PUMP_SSR));
      sig_encode(&txMsg, ACEPUMP_INV, digitalRead(PIN_INVERTER_SSR));
      uint8_t size = sig_encode(&txMsg, ACEPUMP_EPV, energy);
      tinBus.write((uint8_t *)&txMsg, size, MEDIUM_PRIORITY);
    }
    if (frameSequence == SIG_MSG_ID(ACEPUMP_TEMPS)) {
      msg_t txMsg;
      sig_encode(&txMsg, ACEPUMP_SLAB_TOP, temperature[TEMP_SLAB_TOP]);
      sig_encode(&txMsg, ACEPUMP_SLAB_BOT, temperature[TEMP_SLAB_BOT]);
      sig_encode(&txMsg, ACEPUMP_TANK_TOP, temperature[TEMP_TANK_TOP]);
      uint8_t size =
          sig_encode(&txMsg, ACEPUMP_TANK_BOT, temperature[TEMP_TANK_BOT]);
      tinBus.write((uint8_t *)&txMsg, size, MEDIUM_PRIORITY);
    }
  }
  if (sig_decode(msg, ACEPUMP_VSET, &value) != FMT_NULL) {
    if ((value <= VMAX) && (value >= VMIN)) {
      setmv = value;
      // tinBus.write(frame);  // acknowledgement
    }
  }
}
