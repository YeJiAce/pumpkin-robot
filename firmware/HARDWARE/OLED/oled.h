#ifndef PUMPKIN_OLED_H
#define PUMPKIN_OLED_H

/* C12H bundled 128x64 OLED; 6x12 ASCII, maximum 21 characters per line.
   GPIO setup belongs to CubeMX. Only displayTask may access this driver. */
void Oled_Init(void);
void Oled_ShowLines(const char *first, const char *second, const char *third);

#endif
