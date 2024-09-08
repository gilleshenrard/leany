#ifndef INC_HARDWARE_SCREEN_ST7735S_INITIALISATION_H_
#define INC_HARDWARE_SCREEN_ST7735S_INITIALISATION_H_
#include <stdint.h>
#include "ST7735_registers.h"

#define NULL (void*)0  ///< NULL address

enum {
    ST7735_NB_COMMANDS    = 16U,  ///< Number of commands to send during initialisation
    ST7735_STRUCT_PADDING = 16U,  ///< Configuration command structure memory alignment size
};

/**
 * @brief Enumeration of the available display orientations
 */
typedef enum {
    PORTRAIT = 0,   ///< Portrait
    PORTRAIT_180,   ///< Portrait, rotated 180°
    LANDSCAPE,      ///< Landscape
    LANDSCAPE_180,  ///< Landscape, rotated 180°
    NB_ORIENTATION  ///< Number of available orientations
} orientation_e;

/**
 * @brief Type associated with a register value
 */
typedef uint8_t registerValue_t;

/**
 * @brief Structure describing a command to send during configuration
 */
typedef struct {
    ST7735register_e       registerNumber;  ///< Number of the register to send
    uint8_t                nbParameters;    ///< Number of parameters sent after the register number
    const registerValue_t* parameters;      ///< Array of parameters
} __attribute__((aligned(ST7735_STRUCT_PADDING))) st7735_command_t;

extern const st7735_command_t st7735configurationScript[ST7735_NB_COMMANDS];
extern const registerValue_t  orientations[NB_ORIENTATION];

#endif
