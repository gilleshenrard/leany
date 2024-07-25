#ifndef ARROWSICON_H_INCLUDED
#define ARROWSICON_H_INCLUDED
#include <stdint.h>

enum {
    MAX_DATA_SIZE          = 1024U,            ///< Maximum SSD1306 data size (128 * 64 pixels / 8 pixels per byte)
    ARROWSICON_NB_BYTES    = (UINT8_MAX + 1),  ///< Total number of bytes occupied by the arrows icon
    ARROWSICON_WIDTH       = 32U,              ///< Pixel width of the icon
    REFERENCETYPE_NB_BYTES = 7U,
};

extern const uint8_t baseScreen[MAX_DATA_SIZE];
extern const uint8_t relativeReferentialIcon[REFERENCETYPE_NB_BYTES];
extern const uint8_t absoluteReferentialIcon[REFERENCETYPE_NB_BYTES];
extern const uint8_t holdIcon[REFERENCETYPE_NB_BYTES];

#endif
