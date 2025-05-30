#ifndef UI_UI_H
#define UI_UI_H
#include "errorstack.h"

enum {
    MESSAGE_ALIGNMENT = 8U,
};

typedef enum {
    MSG_HOLD = 0,
    MSG_ZERO,
    MSG_PWROFF,
    NB_MESSAGES
} messageID_e;

typedef struct {
    messageID_e ID;
    int16_t     value;
} __attribute__((aligned(MESSAGE_ALIGNMENT))) displayMessage_t;

errorCode_u createUItask(void);
uint8_t     sendDisplayMessage(const displayMessage_t* message);

#endif  // UI_UI_H
