#ifndef USERDEFINED_SYSTICK_H_INCLUDED
#define USERDEFINED_SYSTICK_H_INCLUDED
#include <stdint.h>
#include <stdlib.h>

typedef uint32_t systick_t;

extern volatile systick_t sysTick_ms;  ///< Number of milliseconds elapsed since system boot

/**
 * @brief Increment system ticks count
 * @warning This function must only be used in the Systick interrupt handler and nowhere else
 */
static inline void incrementSysTick(void) {
    sysTick_ms++;
}

/**
 * @brief Get the system ticks count
 * 
 * @return Number of milliseconds elapsed since system boot
 */
static inline systick_t getSystick(void) {
    return (sysTick_ms);
}

/**
 * @brief Check if a time span has elapsed between now and the last saved tick value
 * 
 * @param previousTick_ms Last saved tick value in milliseconds
 * @param timespan_ms Time span in milliseconds
 * @retval 0 Time span not elapsed
 * @retval 1 Time span elapsed
 */
static inline uint8_t isTimeElapsed(systick_t previousTick_ms, size_t timespan_ms) {
    return ((sysTick_ms - previousTick_ms) >= (systick_t)timespan_ms);
}

#endif
