#ifndef ACEGRID_H
#define ACEGRID_H

#include "sig.h"

#define ACEGRID_STATUS (0x70 | SIG_SIZE6)
#define ACEGRID_CONFIG (0x70 | SIG_SIZE2)

#define ACEGRID_VPV (ACEGRID_STATUS | SIG_WORD | SIG_OFF0 | SIG_DECI | SIG_UINT)
#define ACEGRID_PPV (ACEGRID_STATUS | SIG_WORD | SIG_OFF2 | SIG_UNIT | SIG_UINT)
#define ACEGRID_EPV (ACEGRID_STATUS | SIG_WORD | SIG_OFF4 | SIG_CENT | SIG_UINT)

#define ACEGRID_VSET                                                         \
  (ACEGRID_CONFIG | SIG_WORD | SIG_OFF0 | SIG_CENT | SIG_UINT  | SIG_RW)

#define ACEGRID_NAMES                                                          \
  {"grid/ppv", ACEGRID_PPV}, { "grid/vset", ACEGRID_VSET }, { "grid/kwh", ACEGRID_EPV }

// {"grid/Vgt", ACEGRID_VPV}, { "grid/ppv", ACEGRID_PPV }, { "Vgd", ACEGRID_VSET
// }

#endif // ACEGRID_H
