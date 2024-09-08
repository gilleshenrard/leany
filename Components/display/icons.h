#ifndef INC_HARDWARE_SCREEN_ICONS_H_
#define INC_HARDWARE_SCREEN_ICONS_H_
#include <stdint.h>
#include "ST7735_initialisation.h"

enum {
    VERDANA_NB_ROWS    = 49U,
    VERDANA_NB_COLUMNS = 39U,
};

typedef enum {
    BRIGHT_GRAY   = 0xEF7DU,  ///< #EEEEEE
    DARK_CHARCOAL = 0x31A6U,  ///< #333333
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
    VERDANA_COMMA,
    VERDANA_DEG,
    NB_CHARACTERS,
} verdanaCharacter_e;

/**
 * @brief Pixel type definition
 */
typedef uint16_t pixel_t;

void uncompressIconLine(registerValue_t* buffer, verdanaCharacter_e character, uint8_t line);

#endif
