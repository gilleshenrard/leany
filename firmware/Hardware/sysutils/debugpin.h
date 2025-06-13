#ifndef HARDWARE_SYSUTILS_DEBUGPIN
#define HARDWARE_SYSUTILS_DEBUGPIN
#include "main.h"

static inline void setDebugPin(void) {
    LL_GPIO_SetOutputPin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin);
}

static inline void resetDebugPin(void) {
    LL_GPIO_ResetOutputPin(DEBUG_OUT_GPIO_Port, DEBUG_OUT_Pin);
}

#endif
