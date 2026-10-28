#ifndef PUMPKIN_CPU_LOAD_H
#define PUMPKIN_CPU_LOAD_H

#include <stdbool.h>
#include <stdint.h>

/* Called by displayTask; load is per-mille averaged over approximately one second. */
bool CpuLoad_Read(uint16_t *load_permille);
bool CpuLoad_GetCached(uint16_t *load_permille);

#endif
