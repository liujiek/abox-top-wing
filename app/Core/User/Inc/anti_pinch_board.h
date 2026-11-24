#ifndef ANTI_PINCH_BOARD_H
#define ANTI_PINCH_BOARD_H

#include <stdint.h>

/* Call once after MX_GPIO_Init. PA7 is active high via the optocoupler,
 * with no internal pull; PA0 enables DC power. */
void AntiPinchBoard_Init(void);
/* Controller-gated motion range; disabled input produces no alarm or event. */
void AntiPinchBoard_SetScopeEnabled(uint8_t enabled);
/* RS dispatch and moving feedback hold the lamp; the first stop starts its tail. */
void AntiPinchBoard_UpdateResetLamp(uint32_t now, uint8_t moving);
/* Single caller: TIM1's 1 ms callback. Safe before Init (does nothing). */
void AntiPinchBoard_Tick(uint32_t now);
/* Main-loop event transfer: contact edges are independent of the alarm tail. */
uint8_t AntiPinchBoard_TakeTrigger(void);
uint8_t AntiPinchBoard_IsPressed(void);
uint8_t AntiPinchBoard_ContactActive(void);

#endif
