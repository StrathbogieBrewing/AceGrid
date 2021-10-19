#ifndef ACEPUMP_H
#define ACEPUMP_H

#include "sig.h"

#define ACEPUMP_STATUS (0x80 | SIG_SIZE6)
#define ACEPUMP_CONFIG (0x80 | SIG_SIZE2)

#define ACEPUMP_VPV (ACEPUMP_STATUS | SIG_WORD | SIG_OFF0 | SIG_DECI | SIG_UINT)
#define ACEPUMP_PPV (ACEPUMP_STATUS | SIG_WORD | SIG_OFF2 | SIG_UNIT | SIG_UINT)
#define ACEPUMP_EPV (ACEPUMP_STATUS | SIG_WORD | SIG_OFF4 | SIG_CENT | SIG_UINT)

#define ACEPUMP_VSET                                                         \
  (ACEPUMP_CONFIG | SIG_WORD | SIG_OFF0 | SIG_CENT | SIG_UINT  | SIG_RW)

#define ACEPUMP_NAMES                                                          \
  {"gPpv", ACEPUMP_PPV}, { "gVs", ACEPUMP_VSET }, { "gkwh", ACEPUMP_EPV }

// {"grid/Vgt", ACEPUMP_VPV}, { "grid/ppv", ACEPUMP_PPV }, { "Vgd", ACEPUMP_VSET
// }

#endif // ACEPUMP_H
