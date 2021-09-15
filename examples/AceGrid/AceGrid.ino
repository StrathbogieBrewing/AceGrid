#include <avr/wdt.h>

#include "PinChangeInterrupt.h"

#include "AceBMS.h"
#include "AceBus.h"
#include "AceDump.h"
#include "AceGrid.h"

#define VMAX (28000)
#define VMIN (27000)
#define VSET (27500)

#define OFF_TIME (6000) // 6000  x 100 ms = 10 minutes
#define OFF_START (100) // 100  x 100 ms = 10 seconds

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
static uint16_t offTimer = 0;
static uint16_t powerPulseCount = 0;

static volatile unsigned char pulseCount = 0;
void zcdTriggered(void) { pulseCount++; }

void setup() {
  wdt_disable();
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

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PULSE_DETECT),
                           zcdTriggered, FALLING);
  aceBus.begin();
  lastBMSUpdate = micros();
  offTimer = OFF_START;
}

void update_adc(void) {
  static uint16_t adc_filter = 0;
  uint16_t adc = analogRead(VIN_SENSE); // use local vbat for control
  if (adc > 800)
    adc = 800; // limit adc to 80 V
  adc_filter -= (adc_filter >> 4);
  adc_filter += adc;
  vindv = (adc_filter >> 1) * 2; // convert to mv
}

void update_100ms(unsigned long time) {
  static unsigned char pulseLastCount = 0;

  update_adc();                                 // update batterry voltage
  digitalWrite(LED_RED, !digitalRead(LED_RED)); // show alive

  if (lastBMSUpdate + 1000000L < time)
    offTimer = OFF_TIME;

  if (batmv > setmv)
    offTimer = OFF_TIME;

  if (offTimer) {
    offTimer--;
    digitalWrite(SSR_DRIVE, LOW);
    digitalWrite(LED_GREEN, LOW);
  } else {
    digitalWrite(SSR_DRIVE, HIGH);
    digitalWrite(LED_GREEN, HIGH);
  }

  if (pulseLastCount != pulseCount) { // check for energy pulse
    pulseLastCount = pulseCount;
    powerPulseCount++;
  }
}

void loop() {
  wdt_reset();
  aceBus.update();

  static unsigned long time = 0;
  unsigned long now = micros();
  if (now >= time + 100000L) {
    update_100ms(now);
    time = now;
  }
}

void aceCallback(tinframe_t *frame) {
  msg_t *msg = (msg_t *)(frame->data);
  int16_t value;
  if (sig_decode(msg, ACEBMS_VBAT, &value) != FMT_NULL) {
    batmv = value;
    digitalWrite(LED_AMBER, !digitalRead(LED_AMBER)); // show rx data
    lastBMSUpdate = micros();
  }
  if (sig_decode(msg, ACEBMS_RQST, &value) != FMT_NULL) {
    uint8_t frameSequence = value;
    if (frameSequence == (SIG_MSG_ID(ACEGRID_STATUS) & 0xFF)) {
      tinframe_t txFrame;
      msg_t *txMsg = (msg_t *)txFrame.data;
      sig_encode(txMsg, ACEGRID_VPV, vindv);
      sig_encode(txMsg, ACEGRID_PPV, powerPulseCount);
      powerPulseCount = 0;
      aceBus.write(&txFrame);
    }
  }
  if (sig_decode(msg, ACEGRID_VLIMIT, &value) != FMT_NULL) {
    if ((value <= VMAX) && (value >= VMIN)) {
      setmv = value;
    }
  }
}
