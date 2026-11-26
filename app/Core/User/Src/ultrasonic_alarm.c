#include "ultrasonic_alarm.h"
#include <string.h>

void UltrasonicAlarm_Init(UltrasonicAlarm* state) { memset(state, 0, sizeof(*state)); }

uint8_t UltrasonicAlarm_Update(UltrasonicAlarm* state, uint32_t now)
{
    if (!state->seen) return 0U;
    /* User-selected policy: clear frames or silence keep the tail for 2 s.
     * This does not establish that the module is online or the path is clear. */
    if (state->active &&
        ((uint32_t)(now - state->last_rx) >= ULTRASONIC_SILENCE_MS ||
         (state->clearing && (uint32_t)(now - state->clear_since) >= ULTRASONIC_CLEAR_MS)))
    {
        state->active = 0U;
        state->clearing = 0U;
    }
    return state->active;
}

static uint8_t distance_mask(const uint8_t data[8], uint8_t max_distance_cm)
{
    uint8_t mask = 0U;
    /* The installed module reports no valid target as either 0x00 or 0xFF.
     * Targets in its blind zone are intentionally treated as no obstacle. */
    for (uint8_t i = 0U; i < 8U; ++i)
        if (data[i] > 0U && data[i] <= max_distance_cm)
            mask |= (uint8_t)(1U << i);
    return mask;
}

static void observe(UltrasonicAlarm* state, uint32_t now, uint8_t raw_mask, uint8_t confirmed_mask)
{
    (void)UltrasonicAlarm_Update(state, now);
    state->seen = 1U;
    state->last_rx = now;
    state->obstacle_mask = raw_mask;
    if (confirmed_mask)
    {
        state->active = 1U;
        state->clearing = 0U;
    }
    else if (state->active && !state->clearing)
    {
        state->clear_since = now;
        state->clearing = 1U;
    }
}

uint8_t UltrasonicAlarm_OnFrame(UltrasonicAlarm* state, uint32_t now,
    uint32_t id, uint8_t extended, uint8_t remote, uint8_t length, const uint8_t* data)
{
    if (id != WL300D_CAN_ID || !extended || remote || length != 8U || !data) return 0U;
    uint8_t mask = distance_mask(data, ULTRASONIC_ALARM_MAX_CM);
    uint8_t confirmed = 0U;
    if (state->seen && (uint32_t)(now - state->last_rx) <= ULTRASONIC_FRESH_MS)
        confirmed = mask & state->obstacle_mask;
    observe(state, now, mask, confirmed);
    return 1U;
}

void UltrasonicAlarm_ObserveResetPath(UltrasonicAlarm* state, uint32_t now, const uint8_t data[8])
{
    uint8_t mask = distance_mask(data, WL300D_MAX_DISTANCE_CM);
    observe(state, now, mask, mask);
}
