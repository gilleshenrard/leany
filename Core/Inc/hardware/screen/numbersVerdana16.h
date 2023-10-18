#ifndef NUMBERS_VERDANA_16_H_INCLUDED
#define NUMBERS_VERDANA_16_H_INCLUDED
#include <stdint.h>

#define VERDANA_NB_BYTES_CHAR	22U	///< Number of bytes per character
#define VERDANA_CHAR_WIDTH		11U	///< Width of a character in pixels

/**
 * @brief Enumeration of all the characters available in Verdana 16
 */
typedef enum{
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
	INDEX_DOT,
	INDEX_DASH,
	INDEX_DEG,
	NB_NUMBERS
}numbers_e;

extern const uint8_t verdana_16ptNumbers[NB_NUMBERS][VERDANA_NB_BYTES_CHAR];

#endif
