#ifndef INC_HARDWARE_SCREEN_ICONS_H_
#define INC_HARDWARE_SCREEN_ICONS_H_
#include <stdint.h>

enum {
    VERDANA_NB_ROWS    = 37U,
    VERDANA_NB_COLUMNS = 30U,
};

typedef enum {
    BRIGHT_GRAY_BIGENDIAN   = 0x7DEFU,  ///< #EEEEEE in RGB565 with MSB and LSB switched
    DARK_CHARCOAL_BIGENDIAN = 0xA631U,  ///< #333333 in RGB565 with MSB and LSB switched
} colours_e;

typedef enum {
    VERDANA_0 = 0,
    VERDANA_1,
    VERDANA_2,
    VERDANA_3,
    VERDANA_4,
    VERDANA_5,
    VERDANA_6,
    VERDANA_7,
    VERDANA_8,
    VERDANA_9,
    VERDANA_PLUS,
    VERDANA_MIN,
    VERDANA_DOT,
    NB_CHARACTERS,
} verdanaCharacter_e;

/**
 * @brief Pixel type definition
 */
typedef uint16_t pixel_t;

void uncompressCharacter(pixel_t buffer[], verdanaCharacter_e character);

#endif
