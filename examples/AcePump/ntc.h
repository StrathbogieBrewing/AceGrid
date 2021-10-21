#ifndef NTC_H_
#define NTC_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif

int16_t ntc_getDeciCelcius(uint16_t rawADC);

#ifdef __cplusplus
} // extern "C"
#endif


#endif // #ifndef NTC_H_
