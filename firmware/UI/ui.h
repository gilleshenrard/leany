#ifndef UI_UI_H
#define UI_UI_H
#include "errorstack.h"

enum {
    MESSAGE_ALIGNMENT = 8U,  ///< Optimised memory alignment for the displayMessage structure
};

/**
 * Enumeration of all the types of messages handled by the UI
 */
typedef enum {
    MSG_HOLD = 0,  ///< Hold function toggled
    MSG_ZERO,      ///< Zeroing function triggered
    MSG_PWROFF,    ///< Screen off requested
    NB_MESSAGES    ///< Maximum messages enum value
} messageID_e;

/**
 * Structure defining a display message received from a queue
 */
typedef struct {
    messageID_e ID;     ///< Message type
    int16_t     value;  ///< Value linked to the message
} __attribute__((aligned(MESSAGE_ALIGNMENT))) displayMessage_t;

errorCode_u createUItask(void);
uint8_t     sendDisplayMessage(const displayMessage_t* message);

#endif  // UI_UI_H
