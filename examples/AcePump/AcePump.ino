#include <avr/wdt.h>

// #include "PinChangeInterrupt.h"

#include "AceBMS.h"
#include "AceDump.h"
#include "AceGrid.h"
#include "TinBus.h"

#define VMAX (27000)
#define VMIN (26000)
#define VSET (26800)

#define OFF_TIME (60000) // 6000  x 10 ms = 10 minutes
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
//
#define LED_AMBER (9)

#define INVERTER_SSR (6)
#define PUMP_SSR (7)
#define PULSE_DETECT (3)

#define TEMP_SLAB_TOP (A0)
#define TEMP_SLAB_BOT (A1)
#define TEMP_PUMP_OUT (A2)
#define TEMP_PUMP_RET (A3)

#define TEMP_CHANNELS (4)

static int16_t tempValue[TEMP_CHANNELS] = {0};
static uint8_t tempChannel[TEMP_CHANNELS] = {TEMP_SLAB_TOP, TEMP_SLAB_BOT,
                                             TEMP_PUMP_OUT, TEMP_PUMP_RET};

#define MICROS_IN_HOUR (3600000000L)
#define MICROS_IN_10MS (10000L)

#define PULSES_PER_WH (1L)

#define kRxInterruptPin (2)
void busCallback(unsigned char *data, unsigned char length);
TinBus tinBus(Serial, ACEBMS_BAUD, kRxInterruptPin, busCallback);

static unsigned long lastBMSUpdate = 0;
static uint16_t batmv = 0;
static uint16_t vindv = 0;
static uint16_t setmv = VSET;
static uint16_t ssrOffTimer = OFF_START;
static uint16_t power = 0;
static uint16_t energy = 0;
static unsigned long pulseCount = 0;

static int16_t redTimer = 0;
static int16_t greenTimer = 0;
static int16_t amberTimer = 0;

#define kPowerInterruptPin (5)
static volatile unsigned long powerMicros = 0;
void powerInterrupt(void) { powerMicros = micros() | 1L; }

void setup() {
  wdt_disable();

  delay(2000);
  wdt_enable(WDTO_250MS);

  attachInterrupt(digitalPinToInterrupt(kPowerInterruptPin), powerInterrupt,
                  FALLING);

  digitalWrite(LED_AMBER, LOW);
  pinMode(LED_AMBER, OUTPUT);

  digitalWrite(INVERTER_SSR, LOW);
  pinMode(INVERTER_SSR, OUTPUT);

  digitalWrite(PUMP_SSR, LOW);
  pinMode(PUMP_SSR, OUTPUT);

  tinBus.begin();
  noInterrupts();
  powerMicros = 0;
  interrupts();
  lastBMSUpdate = micros();
}

void update_adc(void) {
  static uint16_t adc_filter = 0;
  uint16_t adc = analogRead(VIN_SENSE);
  if (adc > 800)
    adc = 800; // limit adc to 80 V
  adc_filter -= (adc_filter >> 4);
  adc_filter += adc;
  vindv = adc_filter >> 4; // convert to 0.1 mv steps
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
      if ((delta > MICROS_IN_10MS) &&
          (delta < MICROS_IN_HOUR)) { // 60 minutes max
        pulseCount++;
        pulseDelta = delta;
        power = (MICROS_IN_HOUR / PULSES_PER_WH) / delta;
      }
    }
    pulseLastTime = pulseTime;
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

void update_leds(void) {
  // if (redTimer > 0) {
  //   redTimer -= 10;
  //   digitalWrite(LED_RED, HIGH);
  // } else {
  //   digitalWrite(LED_RED, LOW);
  // }

  // if (greenTimer > 0) {
  //   greenTimer -= 10;
  //   digitalWrite(LED_GREEN, HIGH);
  // } else {
  //   digitalWrite(LED_GREEN, LOW);
  // }

  if (amberTimer > 0) {
    amberTimer -= 10;
    digitalWrite(LED_AMBER, HIGH);
  } else {
    digitalWrite(LED_AMBER, LOW);
  }
}

void update_10ms(unsigned long time) {
  static unsigned char seconds = 0;

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
    digitalWrite(INVERTER_SSR, LOW);
  } else {
    digitalWrite(INVERTER_SSR, HIGH);
  }

  if (seconds) {
    seconds--;
    if ((seconds == 50) && (ssrOffTimer == 0))
      greenTimer = 50; // show ssr is on
  } else {
    seconds = 99;
    redTimer = 50; // show device is alive
  }

  update_leds();
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
    // if (frameSequence == (SIG_MSG_ID(ACEGRID_STATUS) & 0xFF)) {
    if ((frameSequence & 0x0F) == (SIG_MSG_ID(ACEGRID_STATUS) & 0x0F)) {
      // if ((frameSequence & 0x03) == 0x01) {
      msg_t txMsg;
      sig_encode(&txMsg, ACEGRID_VPV, vindv);
      sig_encode(&txMsg, ACEGRID_PPV, power);
      uint8_t size = sig_encode(&txMsg, ACEGRID_EPV, energy);
      tinBus.write((uint8_t *)&txMsg, size, MEDIUM_PRIORITY);
    }
  }
  if (sig_decode(msg, ACEGRID_VSET, &value) != FMT_NULL) {
    if ((value <= VMAX) && (value >= VMIN)) {
      setmv = value;
      // tinBus.write(frame);  // acknowledgement
    }
  }
}
