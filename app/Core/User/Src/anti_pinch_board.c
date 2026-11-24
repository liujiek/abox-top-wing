#include "anti_pinch_board.h"

#include "anti_pinch_alarm.h"
#include "main.h"
#include "ultrasonic_board.h"

static AntiPinchAlarm alarm;
static volatile uint8_t ready;
static volatile uint8_t motion_pending, contact_pressed, scope_enabled;
static uint8_t output_active;

#define RESET_LAMP_TIMEOUT_MS 3000
static volatile enum { RESET_LAMP_OFF, RESET_LAMP_MOVING, RESET_LAMP_TAIL } reset_lamp;
static volatile uint32_t reset_lamp_tick;

void AntiPinchBoard_Init(void)
{
    ready = 0U;
    motion_pending = contact_pressed = 0U;
    scope_enabled = 0U;
    output_active = 0U;
    reset_lamp = RESET_LAMP_OFF;
    /* MX_GPIO_Init owns pin configuration. PA0 enables the board's PVIN
     * high-side switch; it does not supply 12 V directly from the MCU. */
#ifdef ABOX_MCU_GD32F105
    gpio_bit_reset(GPIOA, 1U << 0);
#else
    HAL_GPIO_WritePin(ANTI_PINCH_OUTPUT_GPIO_Port, ANTI_PINCH_OUTPUT_Pin, GPIO_PIN_RESET);
#endif
    AntiPinchAlarm_Init(&alarm);
    ready = 1U;
}

void AntiPinchBoard_SetScopeEnabled(uint8_t enabled)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    scope_enabled = enabled ? 1U : 0U;
    __set_PRIMASK(irq);
}

void AntiPinchBoard_UpdateResetLamp(uint32_t now, uint8_t moving)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (moving)
    {
        reset_lamp = RESET_LAMP_MOVING;
        reset_lamp_tick = now;
    }
    else if (reset_lamp == RESET_LAMP_MOVING)
    {
        reset_lamp = RESET_LAMP_TAIL;
        reset_lamp_tick = now;
    }
    __set_PRIMASK(irq);
}

uint8_t AntiPinchBoard_ContactActive(void)
{
    if (!ready) return 1U;
#ifdef ABOX_MCU_GD32F105
    /* Connector B8 maps to PA4 on PCB V1. */
    return gpio_input_bit_get(GPIOA, 1U << 4) != RESET;
#else
    return HAL_GPIO_ReadPin(ANTI_PINCH_CONTACT_GPIO_Port, ANTI_PINCH_CONTACT_Pin)
              == GPIO_PIN_SET;
#endif
}

void AntiPinchBoard_Tick(uint32_t now)
{
    uint8_t pressed, was_pressed, alarm_active;
    if (!ready) return;
    pressed = AntiPinchBoard_ContactActive();
    was_pressed = alarm.stable_pressed;
    alarm_active = AntiPinchAlarm_Update(&alarm, now, pressed && scope_enabled);
    contact_pressed = alarm.stable_pressed;
    if (!was_pressed && contact_pressed) motion_pending = 1U;
    /* One writer owns PA0. Each alarm source can independently hold it on. */
    alarm_active = (uint8_t)(alarm_active | UltrasonicBoard_IsAlarm(now));
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    /* CAN may have refreshed the lamp after this timer tick was sampled. */
    if (reset_lamp != RESET_LAMP_OFF && (int32_t)(now - reset_lamp_tick) >= RESET_LAMP_TIMEOUT_MS)
        reset_lamp = RESET_LAMP_OFF;
    alarm_active |= reset_lamp != RESET_LAMP_OFF;
    __set_PRIMASK(irq);
    if (alarm_active != output_active)
    {
        output_active = alarm_active;
#ifdef ABOX_MCU_GD32F105
        if (alarm_active) gpio_bit_set(GPIOA, 1U << 0); else gpio_bit_reset(GPIOA, 1U << 0);
#else
        HAL_GPIO_WritePin(ANTI_PINCH_OUTPUT_GPIO_Port, ANTI_PINCH_OUTPUT_Pin,
                          alarm_active ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
    }
}

uint8_t AntiPinchBoard_TakeTrigger(void)
{
    uint32_t irq = __get_PRIMASK();
    uint8_t pending;
    __disable_irq();
    pending = motion_pending;
    motion_pending = 0U;
    __set_PRIMASK(irq);
    return pending;
}

uint8_t AntiPinchBoard_IsPressed(void) { return contact_pressed; }
