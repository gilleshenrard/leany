#ifndef LSM6DSO_CONFIGURATION_SCRIPT_H_INCLUDED
#define LSM6DSO_CONFIGURATION_SCRIPT_H_INCLUDED
#include "LSM6DSO_registers.h"
#include "stdint.h"

#define NB_INIT_REG 5U

/**
 * @brief Structure representing a value to write at a specific register
 */
typedef struct {
    LSM6DSOregister_e registerID;  ///< Register ID to which write the value
    uint8_t           value;       ///< Value to write
} registerValue_t;

extern const registerValue_t initialisationArray
    [NB_INIT_REG];  ///< Array of values to write sequentially to specific registers at initialisation
#endif
