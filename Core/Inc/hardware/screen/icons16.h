#ifndef ICONS_16_H_INCLUDED
#define ICONS_16_H_INCLUDED
#include <stdint.h>

#define ICONS_NB_CHARS  32U

typedef enum{
    ARROWS_VERTICAL = 0,
    ARROWS_HORIZONTAL,
    NB_ICONS,
}icon_e;

extern const uint8_t icons_16pt[NB_ICONS][ICONS_NB_CHARS];

#endif