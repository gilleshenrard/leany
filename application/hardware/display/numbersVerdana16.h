#ifndef NUMBERS_VERDANA_16_H_INCLUDED
#define NUMBERS_VERDANA_16_H_INCLUDED
#include <stdint.h>

#define VERDANA_NB_BYTES_CHAR 28U  ///< Number of bytes per character
#define VERDANA_CHAR_WIDTH    14U  ///< Width of a character in pixels
#define VERDANA_NB_PAGES      2U   ///< Number of SSD pages used by a character

/**
 * @brief Enumeration of all the characters available in Verdana 16
 */
typedef enum {
    INDEX_0 = 0,
    INDEX_1,
    INDEX_2,
    INDEX_3,
    INDEX_4,
    INDEX_5,
    INDEX_6,
    INDEX_7,
    INDEX_8,
    INDEX_9,
    INDEX_PLUS,
    INDEX_MINUS,
    INDEX_DOT,
    INDEX_DEG,
    NB_NUMBERS
} numbers_e;

extern const uint8_t verdana_16ptNumbers[NB_NUMBERS][VERDANA_NB_PAGES][VERDANA_CHAR_WIDTH];

#endif
