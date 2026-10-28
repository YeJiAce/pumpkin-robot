#ifndef PUMPKIN_USER_KEY_H
#define PUMPKIN_USER_KEY_H

#include <stdbool.h>
#include <stdint.h>

void UserKey_Init(uint32_t now_ms);
/* Call every 10 ms. One event per debounced press; release required to rearm. */
bool UserKey_Pressed(uint32_t now_ms);

#endif
