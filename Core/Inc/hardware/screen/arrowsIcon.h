#ifndef ARROWSICON_H_INCLUDED
#define ARROWSICON_H_INCLUDED
#include <stdint.h>

#define ARROWSICON_NB_BYTES    (UINT8_MAX + 1)  ///< Total number of bytes occupied by the arrows icon
#define ARROWSICON_WIDTH       32U              ///< Pixel width of the icon

extern const uint8_t arrowsIconLight_32px[ARROWSICON_NB_BYTES];
extern const uint8_t arrowsIconDark_32px[ARROWSICON_NB_BYTES];
extern const uint8_t arrowsIconDarkTrimmed_32px[ARROWSICON_NB_BYTES];

#endif