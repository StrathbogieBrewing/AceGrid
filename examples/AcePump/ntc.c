#include "ntc.h"

#ifndef __AVR__
#define PROGMEM
#define getWord(base, offset) (base[offset])
#include <stdio.h>
#else
#include <avr/pgmspace.h>
#define getWord(base, offset) ((int16_t)pgm_read_word(base + offset))
#endif

// stainless steel cased NTC thermistor from Honeywell with 27K drive resistor
// and 2 x 470R series resistors - table is 0 to 100 C in 5 Celcius steps
#define TEMP_TABLE                                                             \
  {                                                                            \
    282, 338, 398, 459, 519, 577, 631, 680, 725, 764, 798, 827, 852, 873, 891, \
        906, 919, 929, 938, 945, 952                                           \
  }

static const int16_t PROGMEM TEMP_LOOKUP_TABLE[] = TEMP_TABLE;
#define TEMP_LOOKUP_TABLE_SIZE (sizeof(TEMP_LOOKUP_TABLE) / sizeof(int16_t))
#define LOOKUP_TABLE(index) (getWord(TEMP_LOOKUP_TABLE, index))

// int main() {
//   uint16_t adc = 0;
//   while (adc < 1024) {
//     int16_t t = ntc_getDeciCelcius(adc);
//     printf("%d\t%d\n", adc, t);
// 		adc++;
//   }
//   return 0;
// }

int16_t ntc_getDeciCelcius(uint16_t rawADC) {
  uint8_t i = 0;
  while (rawADC > LOOKUP_TABLE(i)) {
    if (++i >= TEMP_LOOKUP_TABLE_SIZE) {
      return 9999;
    }
  }
  if (i == 0)
    return -9999;

  int16_t temp = i * 50;
  int16_t delta = (LOOKUP_TABLE(i) - LOOKUP_TABLE(i - 1));
  int16_t x = (LOOKUP_TABLE(i) - rawADC) * 50;
  while (x >= delta) {
    x -= delta;
    temp--;
  }

  return temp;
}
