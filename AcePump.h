#ifndef ACEPUMP_H
#define ACEPUMP_H

#include "sig.h"

#define ACEPUMP_STATUS (0x08 | SIG_SIZE6)
#define ACEPUMP_TEMPS (0x09 | SIG_SIZE8)
#define ACEPUMP_CONFIG (0x19 | SIG_SIZE2)

#define ACEPUMP_PPV (ACEPUMP_STATUS | SIG_WORD | SIG_OFF0 | SIG_UNIT | SIG_UINT)
#define ACEPUMP_EPV (ACEPUMP_STATUS | SIG_WORD | SIG_OFF2 | SIG_CENT | SIG_UINT)
#define ACEPUMP_VSET                                                         \
  (ACEPUMP_STATUS | SIG_WORD | SIG_OFF4 | SIG_CENT | SIG_UINT  | SIG_RW)


#define ACEPUMP_SLAB_TOP (ACEPUMP_TEMPS | SIG_WORD | SIG_OFF0 | SIG_DECI)
#define ACEPUMP_SLAB_BOT (ACEPUMP_TEMPS | SIG_WORD | SIG_OFF2 | SIG_DECI)
#define ACEPUMP_TANK_TOP (ACEPUMP_TEMPS | SIG_WORD | SIG_OFF4 | SIG_DECI)
#define ACEPUMP_TANK_BOT (ACEPUMP_TEMPS | SIG_WORD | SIG_OFF6 | SIG_DECI)

#define ACEPUMP_SETV                                                         \
  (ACEPUMP_CONFIG | SIG_WORD | SIG_OFF0 | SIG_CENT | SIG_UINT  | SIG_RW)

#define ACEPUMP_NAMES                                                          \
  {"gPpv", ACEPUMP_PPV}, { "gVs", ACEPUMP_VSET }, { "gkwh", ACEPUMP_EPV }, \
  {"pSlabTop", ACEPUMP_SLAB_TOP}, {"pSlabBot", ACEPUMP_SLAB_BOT}, \
  {"pTankTop", ACEPUMP_TANK_TOP}, {"pTankBot", ACEPUMP_TANK_TOP}, \

// {"grid/Vgt", ACEPUMP_VPV}, { "grid/ppv", ACEPUMP_PPV }, { "Vgd", ACEPUMP_VSET
// }

#endif // ACEPUMP_H
