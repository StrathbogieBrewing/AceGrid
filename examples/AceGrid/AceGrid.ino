#include <avr/wdt.h>

#include "PinChangeInterrupt.h"

#include "AceBMS.h"
#include "AceBus.h"
#include "AceDump.h"
#include "AceGrid.h"

#define VMAX (28000)
#define VMIN (27000)
#define VSET (27500)

#define OFF_TIME (60000) // 6000  x 10 ms = 10 minutes
#define OFF_START (1000) // 100  x 10 ms = 10 seconds

#define LED_RED (3)
#define LED_GREEN (4)
#define LED_AMBER (5)

#define ATEN_0 (16)
#define ATEN_1 (17)

#define PWM_CHARGE (6)

#define SSR_DRIVE (9)
#define PULSE_DETECT (8)

#define VIN_SENSE (A1)

#define kRxInterruptPin (2)
void aceCallback(tinframe_t *frame);
AceBus aceBus(Serial, kRxInterruptPin, aceCallback);

static unsigned long lastBMSUpdate = 0;
static uint16_t batmv = 0;
static uint16_t vindv = 0;
static uint16_t setmv = VSET;
static uint16_t ssrOffTimer = 0;
static uint16_t power = 0;

static int16_t redTimer = 0;
static int16_t greenTimer = 0;
static int16_t amberTimer = 0;

static volatile unsigned long pulseMicros = 0;
void zcdTriggered(void) { pulseMicros = micros(); }

void setup() {
  wdt_disable();
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PULSE_DETECT),
                           zcdTriggered, FALLING);
  delay(2000);
  wdt_enable(WDTO_250MS);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_AMBER, OUTPUT);

  pinMode(PWM_CHARGE, OUTPUT);
  analogWrite(PWM_CHARGE, 128);

  digitalWrite(SSR_DRIVE, LOW);
  pinMode(SSR_DRIVE, OUTPUT);

  digitalWrite(ATEN_0, HIGH); // for 0.1 mv / bit voltage sensing
  pinMode(ATEN_0, OUTPUT);
  digitalWrite(ATEN_1, HIGH);
  pinMode(ATEN_1, OUTPUT);

  aceBus.begin();
  noInterrupts();
  pulseMicros = 0;
  interrupts();
  lastBMSUpdate = micros();
  ssrOffTimer = OFF_START;
}

void update_adc(void) {
  static uint16_t adc_filter = 0;
  uint16_t adc = analogRead(VIN_SENSE); // use local vbat for control
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
  unsigned long pulseTime = pulseMicros;
  interrupts();

  if (pulseTime != pulseLastTime) { // check for energy pulse
    if (pulseLastTime != 0) {
      unsigned long delta = pulseTime - pulseLastTime;
      if ((delta > 10000L) && (delta < 600000000L)) { // 10 min (600 sec) max
        pulseDelta = delta;
        power = 3600000000L / delta;
      }
    }
    pulseLastTime = pulseTime;
  } else {
    if ((pulseLastTime != 0) && (pulseDelta != 0)){
      unsigned long delta = usNow - pulseLastTime;
      if ((delta > pulseDelta) &&
          (delta < 600000000L)) { // 10 min (600 sec) max
        power = 3600000000L / delta;
      }
    }
  }
}

void update_leds(void) {
  if (redTimer > 0) {
    redTimer -= 10;
    digitalWrite(LED_RED, HIGH);
  } else {
    digitalWrite(LED_RED, LOW);
  }

  if (greenTimer > 0) {
    greenTimer -= 10;
    digitalWrite(LED_GREEN, HIGH);
  } else {
    digitalWrite(LED_GREEN, LOW);
  }

  if (amberTimer > 0) {
    amberTimer -= 10;
    digitalWrite(LED_AMBER, HIGH);
  } else {
    digitalWrite(LED_AMBER, LOW);
  }
}

void update_10ms(unsigned long time) {
  static unsigned char seconds = 0;

  update_adc(); // update batterry voltage
  update_power(time);

  if ((time - lastBMSUpdate > 1000000L) && (lastBMSUpdate < time))
    ssrOffTimer = OFF_TIME;

  if (batmv > setmv)
    ssrOffTimer = OFF_TIME;

  if (ssrOffTimer) {
    ssrOffTimer--;
    digitalWrite(SSR_DRIVE, LOW);
  } else {
    digitalWrite(SSR_DRIVE, HIGH);
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

  wdt_reset();
  aceBus.update();

  unsigned long now = micros();
  if (now >= time + 10000L) {
    update_10ms(now);
    time = now;
  }
}

void aceCallback(tinframe_t *frame) {
  msg_t *msg = (msg_t *)(frame->data);
  int16_t value;
  if (sig_decode(msg, ACEBMS_VBAT, &value) != FMT_NULL) {
    batmv = value;
    lastBMSUpdate = micros();
  }
  if (sig_decode(msg, ACEBMS_RQST, &value) != FMT_NULL) {
    if ((value & 0x0003) == 0)
      amberTimer = 50; // show rx data
    uint8_t frameSequence = value;
    // if (frameSequence == (SIG_MSG_ID(ACEGRID_STATUS) & 0xFF)) {
    if ((frameSequence & 0x03) == 0x01) {
      tinframe_t txFrame;
      msg_t *txMsg = (msg_t *)txFrame.data;
      sig_encode(txMsg, ACEGRID_VPV, vindv);
      sig_encode(txMsg, ACEGRID_PPV, power);
      aceBus.write(&txFrame);
    }
  }
  if (sig_decode(msg, ACEGRID_VLIMIT, &value) != FMT_NULL) {
    if ((value <= VMAX) && (value >= VMIN)) {
      setmv = value;
    }
  }
}
