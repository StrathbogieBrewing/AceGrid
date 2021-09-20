#ifndef ACEGRID_H
#define ACEGRID_H

#include "sig.h"

#define ACEGRID_STATUS (SIG_PRIORITY_MEDIUM | 0x70)
#define ACEGRID_CONFIG (SIG_PRIORITY_MEDIUM | 0x80)

#define ACEGRID_VPV (ACEGRID_STATUS | SIG_WORD | SIG_OFF0 | SIG_DECI | SIG_UINT)
#define ACEGRID_PPV (ACEGRID_STATUS | SIG_WORD | SIG_OFF2 | SIG_MILL | SIG_UINT)
#define ACEGRID_VSET                                                           \
  (ACEGRID_STATUS | SIG_WORD | SIG_OFF4 | SIG_MILL | SIG_UINT)

#define ACEGRID_VLIMIT                                                         \
  (ACEGRID_CONFIG | SIG_WORD | SIG_OFF0 | SIG_MILL | SIG_UINT)

#define ACEGRID_NAMES                                                          \
  {"Vgt", ACEGRID_VPV}, { "Pgt", ACEGRID_PPV }, { "Vgd", ACEGRID_VSET }

#endif // ACEGRID_H
