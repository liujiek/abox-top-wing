#include <assert.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>

#include "app_log.h"
#include "wing_controller.h"
#include "wing_power.h"

CAN_HandleTypeDef hcan1 = { 1U };
CAN_HandleTypeDef hcan2 = { 2U };

static uint32_t g_tick;
static uint8_t g_tx[512][8];
static uint16_t g_tx_count;
static uint8_t g_rx[8];
static uint8_t g_have_rx;
static uint8_t g_fail_tx;
static HAL_CAN_StateTypeDef g_can_state = HAL_CAN_STATE_LISTENING;
static unsigned g_can_init_calls;
HAL_CAN_StateTypeDef HAL_CAN_GetState(const CAN_HandleTypeDef* hcan)
{ assert(hcan == &hcan2); return g_can_state; }
static uint8_t g_power_output;
void HAL_GPIO_WritePin(void *port, uint16_t pin, uint8_t value)
{
    assert(port == (void *)1 && pin == ((1U << 1) | (1U << 2)));
    g_power_output = value;
}
extern uint8_t test_raw_contact, test_sonar_stale, test_marker_write_fail;
extern int test_reset_marker;
extern uint8_t test_ultrasonic_blocked;
static unsigned reset_errors;
extern uint8_t test_ultrasonic_suppressed;
extern uint8_t test_anti_pinch_scope;
extern uint32_t test_reset_lamp_tick;
extern uint8_t test_reset_lamp_moving;
extern unsigned test_ultrasonic_aborts;
extern unsigned test_ultrasonic_prestart_warnings;
extern unsigned test_ultrasonic_hold_clears;
extern uint32_t test_ultrasonic_prestart_tick;

uint32_t HAL_GetTick(void) { return g_tick; }
void AppLog_Printf(AppLogLevel_t level, const char* tag, const char* fmt, ...)
{ (void)fmt; if (level == APP_LOG_LEVEL_ERROR && !strcmp(tag, "AUTO_RS")) ++reset_errors; }

int HAL_CAN_AddTxMessage(CAN_HandleTypeDef* hcan, CAN_TxHeaderTypeDef* header,
                         uint8_t data[8], uint32_t* mailbox)
{
    (void)hcan;
    if (g_fail_tx) return HAL_ERROR;
    assert(header->ExtId == WING_CAN_TX_ID && header->IDE == CAN_ID_EXT && header->DLC == 8U);
    assert(g_tx_count < 512U);
    memcpy(g_tx[g_tx_count++], data, 8U);
    if (mailbox) *mailbox = 0U;
    return HAL_OK;
}

int HAL_CAN_GetRxMessage(CAN_HandleTypeDef* hcan, uint32_t fifo,
                         CAN_RxHeaderTypeDef* header, uint8_t data[8])
{
    (void)hcan; (void)fifo;
    if (!g_have_rx) return HAL_ERROR;
    header->ExtId = WING_CAN_RX_ID;
    header->IDE = CAN_ID_EXT;
    header->DLC = 8U;
    memcpy(data, g_rx, 8U);
    g_have_rx = 0U;
    return HAL_OK;
}

int HAL_CAN_ConfigFilter(CAN_HandleTypeDef* hcan, CAN_FilterTypeDef* filter)
{ (void)hcan; (void)filter; ++g_can_init_calls; return HAL_OK; }
int HAL_CAN_Start(CAN_HandleTypeDef* hcan) { (void)hcan; ++g_can_init_calls; return HAL_OK; }
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef* hcan, uint32_t notification)
{ (void)hcan; (void)notification; ++g_can_init_calls; return HAL_OK; }

static void reset_controller(void)
{
    test_raw_contact = test_sonar_stale = test_marker_write_fail = 0U;
    test_reset_marker = 0;
    test_reset_lamp_tick = test_reset_lamp_moving = 0U;
    reset_errors = 0U;
    test_ultrasonic_blocked = 0U;
    test_ultrasonic_aborts = 0U;
    test_ultrasonic_prestart_warnings = 0U;
    test_ultrasonic_hold_clears = 0U;
    test_ultrasonic_prestart_tick = 0U;
    g_tick = 0U;
    g_tx_count = 0U;
    g_have_rx = 0U;
    g_fail_tx = 0U;
    WingPower_Init();
    g_can_state = HAL_CAN_STATE_LISTENING;
    g_can_init_calls = 0U;
    assert(WingController_Init() && g_can_init_calls == 0U);
}

static void receive(const uint8_t data[8], uint32_t tick)
{
    memcpy(g_rx, data, 8U);
    g_have_rx = 1U;
    g_tick = tick;
    WingController_OnCanRx(&hcan2);
    assert(!g_have_rx);
}

static void status_with_fault(uint16_t position, uint8_t moving, uint8_t fault, uint32_t tick)
{
    uint8_t data[8] = { 'R', 'D', (uint8_t)(position >> 8), (uint8_t)position,
                        fault, moving, 0U, 0U };
    receive(data, tick);
}

static void status(uint16_t position, uint8_t moving, uint32_t tick)
{ status_with_fault(position, moving, 0U, tick); }

static void limits(uint16_t top, uint16_t bottom, uint32_t tick)
{
    uint8_t data[8] = { 'R', 'L', (uint8_t)(top >> 8), (uint8_t)top,
                        (uint8_t)(bottom >> 8), (uint8_t)bottom, 0U, 0U };
    receive(data, tick);
}

static void ack(WingAction action, uint32_t tick)
{
    uint8_t data[8] = { 0U };
    if (action == WING_ACTION_ASCEND) { data[0] = 'U'; data[1] = 'P'; }
    else if (action == WING_ACTION_DESCEND) { data[0] = 'D'; data[1] = 'N'; }
    else { data[0] = 'S'; data[1] = 'T'; }
    receive(data, tick);
}

static uint16_t count_command(char first, char second)
{
    uint16_t i, count = 0U;
    for (i = 0U; i < g_tx_count; ++i)
        if (g_tx[i][0] == (uint8_t)first && g_tx[i][1] == (uint8_t)second) count++;
    return count;
}

static WingControlResult take_result(void)
{
    WingControlResult result;
    assert(WingController_TakeResult(&result));
    return result;
}

static void drain_reports(void)
{
    while (WingController_TakeReportReason() != WING_REPORT_NONE) { }
}

static void start_running_with_limits(WingAction action, const char* rid, uint16_t position,
                                      uint16_t top, uint16_t bottom)
{
    WingControlResult result;
    g_tick = 0U;
    assert(WingController_Request(action, rid) == 0U);
    assert(count_command('R', 'D') == 1U);
    status(position, 0U, 1U);
    WingController_Task(250U);
    assert(count_command('R', 'D') == 1U);
    WingController_Task(251U);
    assert(count_command('R', 'D') == 2U);
    status(position, 0U, 252U);
    assert(count_command('R', 'L') == 1U);
    limits(top, bottom, 253U);
    assert(count_command(action == WING_ACTION_ASCEND ? 'U' : 'D',
                         action == WING_ACTION_ASCEND ? 'P' : 'N') == 1U);
    ack(action, 254U);
    assert(count_command('R', 'D') == 3U);
    status(position, 1U, 255U);
    result = take_result();
    assert(result.code == 200U && result.action == action && !strcmp(result.request_id, rid));
    assert(WingController_GetSnapshot()->can_session == WING_CAN_SESSION_REMOTE_RUNNING);
}

static void start_running(WingAction action, const char* rid, uint16_t position)
{
    start_running_with_limits(action, rid, position, 100U, 10U);
}

static void test_idle_silent_and_correlation(void)
{
    const WingSnapshot* snapshot;
    reset_controller();
    WingController_Task(100U);
    WingController_Task(300000U);
    assert(g_tx_count == 0U);
    status(30U, 1U, 300001U);
    limits(100U, 10U, 300002U);
    assert(g_tx_count == 0U);
    snapshot = WingController_GetSnapshot();
    assert(snapshot->motion_state == WING_MOTION_UNKNOWN);
    assert(!snapshot->status_fresh && !snapshot->controller_online_known);
    assert(!snapshot->have_position && !snapshot->have_limits);
    assert(WingController_Request(WING_ACTION_STOP, "idle-stop") == 0U);
    assert(count_command('S', 'T') == 1U);
    ack(WING_ACTION_STOP, 300003U);
    status(30U, 0U, 300004U);
    WingControlResult result = take_result();
    assert(result.code == 200U && result.stop_confirmed);
}

static void test_preflight_local_motion_rejected(void)
{
    WingControlResult result;
    reset_controller();
    assert(WingController_Request(WING_ACTION_ASCEND, "local") == 0U);
    status(20U, 1U, 1U);
    result = take_result();
    assert(result.code == 4003U);
    assert(count_command('S', 'T') == 0U && count_command('U', 'P') == 0U);
    assert(WingController_GetSnapshot()->control_owner == WING_CONTROL_OWNER_LOCAL_SUSPECTED);
    assert(WingController_GetSnapshot()->can_session == WING_CAN_SESSION_IDLE_SILENT);
}

static void test_preflight_timeouts_and_limits(void)
{
    WingControlResult result;
    reset_controller();
    assert(WingController_Request(WING_ACTION_ASCEND, "no-rd") == 0U);
    WingController_Task(300U);
    result = take_result();
    assert(result.code == 5002U && count_command('U', 'P') == 0U);

    reset_controller();
    assert(WingController_Request(WING_ACTION_ASCEND, "bad-limits") == 0U);
    status(20U, 0U, 1U);
    WingController_Task(251U);
    status(20U, 0U, 252U);
    limits(10U, 20U, 253U);
    result = take_result();
    assert(result.code == 5003U && count_command('U', 'P') == 0U);
}

static void test_endpoint_single_stop(void)
{
    reset_controller();
    start_running(WING_ACTION_ASCEND, "endpoint", 50U);
    WingController_Task(355U);
    status(98U, 1U, 356U);
    assert(count_command('S', 'T') == 1U);
    status(99U, 1U, 357U);
    WingController_Task(500U);
    assert(count_command('S', 'T') == 1U);
    ack(WING_ACTION_STOP, 501U);
    status(100U, 0U, 502U);
    assert(WingController_GetSnapshot()->can_session == WING_CAN_SESSION_IDLE_SILENT);
    assert(!WingController_GetSnapshot()->status_fresh);
}

static void test_running_feedback_uses_periodic_mqtt_policy(void)
{
    reset_controller();
    start_running(WING_ACTION_ASCEND, "periodic", 50U);
    drain_reports();
    WingController_Task(355U);
    status(55U, 1U, 356U);
    assert(WingController_TakeReportReason() == WING_REPORT_NONE);
    assert(!strcmp(WingReportReason_Name(WING_REPORT_PERIODIC), "periodic"));
}

static void test_conflict_and_unexpected_stop(void)
{
    reset_controller();
    start_running(WING_ACTION_ASCEND, "reverse", 50U);
    WingController_Task(355U); status(60U, 1U, 356U);
    WingController_Task(456U); status(55U, 1U, 457U);
    assert(count_command('S', 'T') == 0U);
    WingController_Task(557U); status(54U, 1U, 558U);
    assert(count_command('S', 'T') == 1U);
    assert(WingController_GetSnapshot()->control_owner == WING_CONTROL_OWNER_LOCAL_SUSPECTED);

    reset_controller();
    start_running(WING_ACTION_DESCEND, "released", 50U);
    WingController_Task(355U); status(45U, 0U, 356U);
    assert(count_command('S', 'T') == 1U);
}

static void test_stop_preemption_and_safety(void)
{
    WingControlResult result;
    reset_controller();
    assert(WingController_Request(WING_ACTION_ASCEND, "preempted") == 0U);
    status(20U, 0U, 1U);
    assert(WingController_Request(WING_ACTION_STOP, "stop-now") == 0U);
    assert(count_command('S', 'T') == 1U);
    result = take_result();
    assert(result.code == 5003U && result.action == WING_ACTION_ASCEND);
    ack(WING_ACTION_STOP, 2U);
    status(20U, 0U, 3U);
    result = take_result();
    assert(result.code == 200U && result.action == WING_ACTION_STOP);

    reset_controller();
    start_running(WING_ACTION_ASCEND, "feedback-timeout", 50U);
    WingController_Task(756U);
    assert(count_command('S', 'T') == 1U);
    assert(WingController_GetSnapshot()->safety_reason == WING_SAFETY_FEEDBACK_TIMEOUT);
}

static void test_direction_change_fault_and_max_runtime(void)
{
    uint32_t tick;
    reset_controller();
    start_running(WING_ACTION_ASCEND, "switch-up", 50U);
    assert(WingController_Request(WING_ACTION_DESCEND, "switch-down-early") == 4003U);
    assert(WingController_Request(WING_ACTION_STOP, "switch-stop") == 0U);
    ack(WING_ACTION_STOP, 256U);
    status(50U, 0U, 257U);
    (void)take_result();
    g_tick = 258U;
    assert(WingController_Request(WING_ACTION_DESCEND, "switch-down") == 0U);
    assert(count_command('R', 'D') == 5U);

    reset_controller();
    start_running(WING_ACTION_ASCEND, "fault", 50U);
    WingController_Task(355U);
    status_with_fault(55U, 1U, 1U, 356U);
    assert(count_command('S', 'T') == 1U);
    assert(WingController_GetSnapshot()->safety_reason == WING_SAFETY_CONTROLLER_FAULT);

    reset_controller();
    start_running(WING_ACTION_ASCEND, "max-runtime", 50U);
    for (tick = 355U; tick < 120255U; tick += 400U)
    {
        WingController_Task(tick);
        status(50U, 1U, tick + 1U);
    }
    WingController_Task(120255U);
    assert(count_command('S', 'T') == 1U);
    assert(WingController_GetSnapshot()->safety_reason == WING_SAFETY_MAX_RUNTIME);
}

static void move_ack(uint16_t target, uint8_t rejected, uint32_t tick)
{
    uint8_t data[8] = { 'M', 'O', (uint8_t)(target >> 8), (uint8_t)target, 0, 0, 0, rejected };
    receive(data, tick);
}

static void pinch_start(uint16_t start, uint16_t top)
{
    start_running_with_limits(WING_ACTION_DESCEND, "pinch-motion", start, top, 0U);
    WingController_AntiPinch(1U, 1U);
    assert(count_command('S', 'T') == 1U && count_command('M', 'O') == 0U);
    ack(WING_ACTION_STOP, 256U);
    status(start, 0U, 257U);
    assert(count_command('R', 'L') == 2U);
    limits(top, 0U, 258U);
}

static uint16_t last_move_target(void)
{
    for (uint16_t i = g_tx_count; i > 0; --i)
        if (g_tx[i - 1][0] == 'M' && g_tx[i - 1][1] == 'O')
            return (uint16_t)((g_tx[i - 1][2] << 8) | g_tx[i - 1][3]);
    assert(0); return 0;
}

static void test_pinch_bounded_move_and_hold(void)
{
    reset_controller();
    pinch_start(50U, 100U);
    assert(count_command('M', 'O') == 1U && last_move_target() == 70U);
    assert(WingController_Request(WING_ACTION_DESCEND, "blocked") == 4003U);
    unsigned rd_count = count_command('R', 'D');
    move_ack(69U, 0U, 259U); /* Wrong target echo cannot advance the state. */
    assert(count_command('R', 'D') == rd_count);
    move_ack(70U, 0U, 260U);
    status(55U, 1U, 261U);
    WingController_Task(362U); status(69U, 1U, 363U);
    assert(count_command('S', 'T') == 1U);
    WingController_Task(464U); status(70U, 0U, 465U);
    assert(count_command('S', 'T') == 2U);
    ack(WING_ACTION_STOP, 466U); status(70U, 0U, 467U);
    for (unsigned i = 0; i < 30; ++i) {
        WingController_AntiPinch(1U, 0U);
        WingController_Task(1000U + i * 1000U);
    }
    assert(count_command('M', 'O') == 1U);
    assert(WingController_Request(WING_ACTION_ASCEND, "held") == 4003U);
    WingController_AntiPinch(0U, 0U);
    assert(WingController_Request(WING_ACTION_ASCEND, "released") == 0U);
    reset_controller();
    start_running_with_limits(WING_ACTION_DESCEND, "pinch-rearm", 50U, 100U, 0U);
    WingController_AntiPinch(1U, 1U);
    assert(count_command('S', 'T') == 1U); /* A genuinely new press can rearm. */

    reset_controller(); pinch_start(95U, 100U);
    assert(last_move_target() == 100U); /* Never command a target above top. */
    reset_controller(); pinch_start(100U, 100U);
    assert(count_command('M', 'O') == 0U);

    /* Full-stroke protection still confirms stop and clips the upward target. */
    const uint16_t cases[][3] = {
        {150U, 250U, 170U},
        {248U, 250U, 250U},
        {65530U, 65535U, 65535U}
    };
    for (unsigned i = 0U; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        uint16_t start = cases[i][0], top = cases[i][1], target = cases[i][2];
        reset_controller();
        pinch_start(start, top);
        assert(count_command('M', 'O') == 1U && last_move_target() == target);
        move_ack(target, 0U, 259U);
        status((uint16_t)(start + 1U), 1U, 260U);
        WingController_Task(361U);
        status(target, 0U, 362U);
        assert(count_command('S', 'T') == 2U);
        ack(WING_ACTION_STOP, 363U);
        status(target, 0U, 364U);
        assert(WingController_GetSnapshot()->can_session == WING_CAN_SESSION_IDLE_SILENT);
        assert(WingController_Request(WING_ACTION_DESCEND, "full-stroke-held") == 4003U);
        WingController_AntiPinch(0U, 0U);
        assert(!test_anti_pinch_scope);
        assert(WingController_Request(WING_ACTION_DESCEND, "full-stroke-released") == 0U);
    }

    reset_controller(); pinch_start(65535U, 65535U);
    assert(count_command('M', 'O') == 0U);
}

static void test_pinch_stop_confirmation_and_failures(void)
{
    reset_controller();
    start_running(WING_ACTION_DESCEND, "pinch-stop", 50U);
    unsigned rl_count = count_command('R', 'L');
    WingController_AntiPinch(1U, 1U);
    status(50U, 0U, 256U); /* No ST ACK: uncorrelated stopped report is ignored. */
    assert(count_command('R', 'L') == rl_count);
    ack(WING_ACTION_STOP, 257U);
    status_with_fault(50U, 1U, 1U, 258U);
    assert(count_command('R', 'L') == rl_count && count_command('M', 'O') == 0U);
    WingController_Task(1255U);
    assert(WingController_GetSnapshot()->can_session == WING_CAN_SESSION_FAULT);
    WingController_AntiPinch(0U, 0U);
    assert(WingController_Request(WING_ACTION_DESCEND, "unsafe") == 4003U);
    assert(WingController_Request(WING_ACTION_STOP, "recover") == 0U);
    ack(WING_ACTION_STOP, 1256U); status(50U, 0U, 1257U);
    assert(count_command('M', 'O') == 0U); /* Explicit recovery never resumes MO. */
    WingController_AntiPinch(0U, 0U);
    assert(WingController_Request(WING_ACTION_ASCEND, "recovered") == 0U);

    reset_controller();
    start_running(WING_ACTION_DESCEND, "pinch-timeout", 50U);
    WingController_AntiPinch(1U, 1U);
    ack(WING_ACTION_STOP, 256U); status(50U, 0U, 257U);
    WingController_Task(559U); /* Limit timeout: no movement. */
    assert(count_command('M', 'O') == 0U);
    assert(WingController_GetSnapshot()->can_session == WING_CAN_SESSION_FAULT);

    reset_controller(); pinch_start(50U, 100U);
    WingController_Task(1258U); /* MO might have executed despite lost ACK. */
    assert(count_command('S', 'T') == 2U);
    reset_controller(); pinch_start(50U, 100U);
    move_ack(70U, 1U, 259U); /* Rejection also ends with a confirmed stop. */
    assert(count_command('S', 'T') == 2U);

    reset_controller(); pinch_start(50U, 100U);
    move_ack(70U, 0U, 259U); status(55U, 1U, 260U);
    WingController_Task(761U); /* Feedback lost during backoff. */
    assert(count_command('S', 'T') == 2U);

    reset_controller();
    start_running(WING_ACTION_DESCEND, "pinch-tx", 50U);
    g_fail_tx = 1U;
    WingController_AntiPinch(1U, 1U);
    assert(WingController_GetSnapshot()->can_session == WING_CAN_SESSION_FAULT);
    assert(count_command('M', 'O') == 0U);
}

static void test_pinch_move_abort_paths(void)
{
    uint32_t now;
    reset_controller(); pinch_start(50U, 100U);
    move_ack(70U, 0U, 259U);
    assert(!WingController_GetSnapshot()->status_fresh); /* ACK is not feedback. */
    WingController_Task(760U);
    assert(count_command('S', 'T') == 2U);

    reset_controller(); pinch_start(50U, 100U);
    move_ack(70U, 0U, 259U); status(55U, 1U, 260U);
    WingController_Task(360U); status(50U, 1U, 361U);
    assert(count_command('S', 'T') == 2U); /* Unexpected downward motion. */

    reset_controller(); pinch_start(50U, 100U);
    move_ack(70U, 0U, 259U); status_with_fault(55U, 1U, 2U, 260U);
    assert(count_command('S', 'T') == 2U);

    reset_controller(); pinch_start(50U, 100U);
    move_ack(70U, 0U, 259U); status(50U, 0U, 260U);
    for (now = 360U; now <= 1360U; now += 100U) {
        WingController_Task(now); status(50U, 0U, now + 1U);
    }
    assert(count_command('S', 'T') == 2U); /* Never moves: abort after start grace. */

    reset_controller(); pinch_start(50U, 100U);
    move_ack(70U, 0U, 259U); status(55U, 1U, 260U);
    for (now = 360U; now < 10259U; now += 100U) {
        WingController_Task(now); status(55U, 1U, now + 1U);
    }
    WingController_Task(10259U);
    assert(count_command('S', 'T') == 2U); /* Continuous feedback cannot bypass runtime bound. */

    reset_controller();
    start_running(WING_ACTION_DESCEND, "pinch-bad-limits", 50U);
    WingController_AntiPinch(1U, 1U);
    ack(WING_ACTION_STOP, 256U); status(50U, 0U, 257U);
    limits(10U, 20U, 258U); /* Malformed limits never produce MO. */
    assert(count_command('M', 'O') == 0U);
    assert(WingController_GetSnapshot()->can_session == WING_CAN_SESSION_FAULT);
}

static void test_lost_start_ack_and_unused_bus(void)
{
    reset_controller();
    assert(WingController_Request(WING_ACTION_ASCEND, "lost-start") == 0U);
    status(50U, 0U, 1U); WingController_Task(251U);
    status(50U, 0U, 252U); limits(100U, 0U, 253U);
    assert(count_command('U', 'P') == 1U);
    WingController_Task(1253U);
    assert(count_command('S', 'T') == 1U);
    assert(WingController_Request(WING_ACTION_DESCEND, "unconfirmed") == 4003U);

    reset_controller();
    memset(g_rx, 0, sizeof(g_rx)); g_have_rx = 1U;
    WingController_OnCanRx(&hcan1);
    assert(!g_have_rx && !g_tx_count); /* Unselected FIFO is consumed, not left pending. */
}

static void test_pinch_preemption_and_operator_stop(void)
{
    reset_controller();
    start_running(WING_ACTION_DESCEND, "down", 50U);
    WingController_AntiPinch(1U, 1U);
    assert(count_command('S', 'T') == 1U);
    assert(count_command('M', 'O') == 0U);
    ack(WING_ACTION_STOP, 256U); status(45U, 0U, 257U);
    limits(100U, 0U, 258U);
    assert(last_move_target() == 65U); /* Measure from confirmed stop, not trigger. */
    assert(WingController_Request(WING_ACTION_STOP, "operator") == 0U);
    assert(count_command('S', 'T') == 2U);
    move_ack(65U, 0U, 259U); /* Late MO ACK cannot restart a cancelled move. */
    ack(WING_ACTION_STOP, 260U); status(45U, 0U, 261U);
    assert(count_command('M', 'O') == 1U);

    reset_controller();
    start_running(WING_ACTION_DESCEND, "pinch-stop-cancel", 50U);
    WingController_AntiPinch(1U, 1U);
    assert(WingController_Request(WING_ACTION_STOP, "cancel-before-reverse") == 5003U);
    ack(WING_ACTION_STOP, 256U); status(50U, 0U, 257U);
    assert(count_command('R', 'L') == 1U && count_command('M', 'O') == 0U);
}

static void test_ultrasonic_stop_and_no_resume(void)
{
    for (WingAction dir=WING_ACTION_ASCEND; dir<=WING_ACTION_DESCEND; ++dir)
    {
        reset_controller(); start_running(dir,"sonar-move",80U);
        test_ultrasonic_blocked=1U;
        WingController_UltrasonicStop();
        assert(count_command('S','T')==1 && test_ultrasonic_aborts==1);
        assert(WingController_Request(WING_ACTION_ASCEND,"blocked")==4003);
        status(80,0,256); /* Stopped feedback cannot replace ST ACK. */
        assert(WingController_GetSnapshot()->can_session==WING_CAN_SESSION_STOPPING);
        ack(WING_ACTION_STOP,257);status(80,0,258);
        assert(WingController_GetSnapshot()->can_session==WING_CAN_SESSION_IDLE_SILENT);
        unsigned before=g_tx_count;
        for(unsigned n=0;n<10;++n) WingController_Task(300+n*100);
        assert(g_tx_count==before && WingController_Request(dir,"still-blocked")==4003);
        test_ultrasonic_blocked=0;
        WingController_Task(1500);
        assert(g_tx_count==before); /* Clearing does not resume or poll. */
        assert(WingController_Request(dir,"fresh-operator-request")==0);
        assert(g_tx_count==before+1 && g_tx[before][0]=='R');
    }
}

static void test_ultrasonic_idle_is_alarm_only(void)
{
    reset_controller();
    assert(test_ultrasonic_suppressed);
    test_ultrasonic_blocked = 1U;
    WingController_UltrasonicStop();
    assert(count_command('S', 'T') == 0U && test_ultrasonic_aborts == 0U);
    assert(WingController_Request(WING_ACTION_ASCEND, "remote-blocked") == 4003U);
    assert(test_ultrasonic_prestart_warnings == 1U && test_ultrasonic_prestart_tick == 0U);

    reset_controller();
    start_running(WING_ACTION_ASCEND, "remote-sonar", 80U);
    assert(!test_ultrasonic_suppressed);
    test_ultrasonic_blocked = 1U;
    WingController_UltrasonicStop();
    assert(count_command('S', 'T') == 1U && test_ultrasonic_aborts == 1U);
}

static void test_ultrasonic_preempts_can_callback_and_pinch(void)
{
    reset_controller();
    assert(WingController_Request(WING_ACTION_ASCEND,"preflight")==0);
    status(50,0,1);WingController_Task(251);status(50,0,252);
    test_ultrasonic_blocked=1;
    limits(100,0,253); /* IRQ catches the alarm before main consumes its edge. */
    assert(count_command('U','P')==0 && count_command('S','T')==1);
    WingController_UltrasonicStop();
    assert(count_command('S','T')==1); /* Do not restart/duplicate a pending stop. */
    test_ultrasonic_blocked=0;
    ack(WING_ACTION_STOP,254);status(50,0,255);
    limits(100,0,256);ack(WING_ACTION_ASCEND,257); /* Late responses cannot restart. */
    assert(count_command('U','P')==0);

    reset_controller();pinch_start(50,100);
    assert(count_command('M','O')==1);
    test_ultrasonic_blocked=1;WingController_UltrasonicStop();
    assert(count_command('S','T')==1 && !test_ultrasonic_aborts);
    move_ack(70,0,259);status(55,1,260);
    WingController_Task(360);status(60,1,361);
    assert(count_command('S','T')==1 && last_move_target()==70);
    WingController_Task(462);status(70,0,463);
    assert(count_command('S','T')==2);
    ack(WING_ACTION_STOP,464);status(70,0,465);
    assert(count_command('M','O')==1 && !test_ultrasonic_aborts);
    assert(WingController_Request(WING_ACTION_ASCEND,"sonar-held")==4003);
    test_ultrasonic_blocked=0;WingController_AntiPinch(1,0);
    assert(WingController_Request(WING_ACTION_ASCEND,"contact-held")==4003);
    WingController_AntiPinch(0,0);
    assert(WingController_Request(WING_ACTION_ASCEND,"new-after-release")==0);

    reset_controller();start_running(WING_ACTION_DESCEND,"sonar-pinch",80);
    test_ultrasonic_blocked=1;
    WingController_UltrasonicStop();WingController_AntiPinch(1,1);
    ack(WING_ACTION_STOP,256);status(80,0,257);
    assert(count_command('M','O')==0 && count_command('R','L')==1);
    reset_controller();start_running(WING_ACTION_DESCEND,"sonar-timeout",80);
    test_ultrasonic_blocked=1;
    WingController_UltrasonicStop();WingController_AntiPinch(1,1);
    WingController_Task(1255);
    assert(WingController_GetSnapshot()->can_session==WING_CAN_SESSION_FAULT);
}

static void test_ultrasonic_preserves_early_pinch_cycle_and_limits(void)
{
    reset_controller();
    start_running_with_limits(WING_ACTION_DESCEND, "pinch-sonar-early", 95U, 100U, 0U);
    WingController_AntiPinch(1,1);
    test_ultrasonic_blocked=1;WingController_UltrasonicStop();
    ack(WING_ACTION_STOP,256);status(95,0,257);
    limits(100,0,258);
    assert(last_move_target()==100 && !test_ultrasonic_aborts);
    move_ack(100,0,259);status(100,0,260);
    ack(WING_ACTION_STOP,261);status(100,0,262);
    assert(count_command('M','O')==1 && count_command('S','T')==2);
    assert(WingController_Request(WING_ACTION_DESCEND,"blocked-after-backoff")==4003);
}

static void test_ultrasonic_stop_failure_stays_locked(void)
{
    reset_controller();start_running(WING_ACTION_DESCEND,"down",80);
    test_ultrasonic_blocked=1;WingController_UltrasonicStop();
    test_ultrasonic_blocked=0;WingController_Task(1255);
    assert(WingController_GetSnapshot()->can_session==WING_CAN_SESSION_FAULT);
    assert(WingController_Request(WING_ACTION_ASCEND,"after-failed-stop")==4003);
    assert(WingController_Request(WING_ACTION_STOP,"recover")==0);
    ack(WING_ACTION_STOP,1256);status(80,0,1257);
    assert(WingController_Request(WING_ACTION_ASCEND,"after-confirmed-stop")==0);
}

static void test_anti_pinch_motion_scope(void)
{
    reset_controller();
    WingController_AntiPinch(1U, 1U);
    assert(!test_anti_pinch_scope && count_command('S', 'T') == 0U);
    assert(WingController_Request(WING_ACTION_ASCEND, "idle-not-blocked") == 0U);

    const uint16_t positions[] = {0U, 1U, 50U, 100U, 101U, 200U, 248U, 250U, 65535U};
    for (unsigned i = 0U; i < sizeof(positions) / sizeof(positions[0]); ++i)
    {
        reset_controller();
        start_running_with_limits(WING_ACTION_ASCEND, "up-no-pinch", positions[i], 65535U, 0U);
        assert(!test_anti_pinch_scope);
        WingController_AntiPinch(1U, 1U);
        assert(count_command('S', 'T') == 0U);

        reset_controller();
        start_running_with_limits(WING_ACTION_DESCEND, "full-stroke-pinch", positions[i], 65535U, 0U);
        assert(test_anti_pinch_scope);
        WingController_AntiPinch(1U, 1U);
        assert(count_command('S', 'T') == 1U && count_command('M', 'O') == 0U);
    }
}

static void test_ultrasonic_near_retraction_suppression(void)
{
    static const struct {
        WingAction action;
        uint16_t position_mm;
        uint8_t suppressed;
    } cases[] = {
        { WING_ACTION_ASCEND, 4U, 0U },
        { WING_ACTION_ASCEND, 5U, 1U },
        { WING_ACTION_ASCEND, 70U, 1U },
        { WING_ACTION_ASCEND, 71U, 0U },
        { WING_ACTION_DESCEND, 0U, 1U },
        { WING_ACTION_DESCEND, 4U, 1U },
        { WING_ACTION_DESCEND, 70U, 1U },
        { WING_ACTION_DESCEND, 71U, 0U },
    };

    for (unsigned i = 0U; i < sizeof(cases) / sizeof(cases[0]); ++i)
    {
        reset_controller();
        start_running_with_limits(cases[i].action, "sonar-boundary",
                                  cases[i].position_mm, 100U, 0U);
        assert(test_ultrasonic_suppressed == cases[i].suppressed);
        test_ultrasonic_blocked = 1U;
        if (cases[i].suppressed)
            WingController_UltrasonicStop();
        WingController_Task(300U);
        assert(count_command('S', 'T') == (cases[i].suppressed ? 0U : 1U));
        assert(test_ultrasonic_aborts == (cases[i].suppressed ? 0U : 1U));
    }

    reset_controller();
    start_running_with_limits(WING_ACTION_ASCEND, "idle-check", 20U, 100U, 0U);
    WingController_Task(355U);
    status(20U, 1U, 356U);
    assert(test_ultrasonic_suppressed);
    test_ultrasonic_blocked = 1U;
    assert(WingController_Request(WING_ACTION_STOP, "stop-mask") == 0U);
    ack(WING_ACTION_STOP, 357U);
    status(20U, 0U, 358U);
    assert(test_ultrasonic_suppressed);
    assert(WingController_IsUltrasonicBlocked());
    assert(WingController_Request(WING_ACTION_ASCEND, "idle-blocked") == 4003U);
}

static void idle_poll(uint32_t tick, uint8_t pending)
{
    g_tick = tick;
    WingController_Task(tick);
    WingController_AutoResetTask(pending);
}

/* Read real wire-format RL and two RD samples, spaced by the quiet interval. */
static void idle_samples(uint32_t tick, uint16_t bottom, uint16_t top,
                         uint16_t first, uint16_t second)
{
    idle_poll(tick, 0U);
    limits(top, bottom, tick + 1U);
    status(first, 0U, tick + 2U);
    idle_poll(tick + 252U, 0U);
    status(second, 0U, tick + 253U);
    idle_poll(tick + 254U, 0U);
}

static void idle_attempt(uint32_t tick, uint16_t position)
{
    idle_samples(tick, 1U, 250U, position, position);
    status(position, 0U, tick + 255U);
    idle_poll(tick + 256U, 0U);
}

static void test_idle_range_recovery(void)
{
    const uint8_t rs_ack[8] = {'R', 'S'};
    const uint16_t positions[] = {0, 1, 3, 4, 250, 251};
    for (unsigned i = 0; i < sizeof(positions) / sizeof(positions[0]); ++i)
    {
        reset_controller();
        idle_attempt(3000U, positions[i]);
        unsigned outside = positions[i] == 0 || positions[i] > 250;
        assert(count_command('R','S') == outside && test_reset_marker == (int)outside);
    }
    /* This controller's limits are 10..100, so 9 is also outside. */
    reset_controller();
    idle_samples(3000, 10, 100, 9, 9);
    status(9,0,3255); idle_poll(3256,0);
    assert(count_command('R','S') == 1 && test_anti_pinch_scope);
    receive(rs_ack,3257); status(9,1,3258);
    assert(WingController_Request(WING_ACTION_ASCEND, "busy") == 4003);
    for (unsigned t = 3358; t <= 3661; t += 101)
    {
        idle_poll(t,0); status(10,0,t+1);
    }
    assert(count_command('S','T') == 1 && test_reset_marker == 1);
    ack(WING_ACTION_STOP,3663); status(10,0,3664);
    assert(reset_errors == 0);
    idle_samples(6700, 10, 100, 10, 10);
    assert(test_reset_marker == 0);
    idle_attempt(10000,251);
    assert(count_command('R','S') == 2);
}

static uint16_t check_power_off_position(uint16_t top, uint16_t bottom, uint16_t position,
                                         uint8_t moving, uint8_t fault)
{
    reset_controller();
    assert(WingController_CheckPowerOff("off") == 0U);
    status(position, 0U, 1U);
    WingController_Task(251U);
    status(position, 0U, 252U);
    limits(top, bottom, 253U);
    status_with_fault(position, moving, fault, 254U);
    assert(WingController_GetSnapshot()->bottom_limit_mm == bottom);
    return take_result().code;
}

static void test_retracted_boundaries(void)
{
    const uint8_t rs_ack[8] = {'R', 'S'};
    for (uint16_t bottom = 0; bottom <= 1; ++bottom)
    {
        for (uint16_t position = 0; position <= 4; ++position)
            assert(check_power_off_position(250, bottom, position, 0, 0) ==
                   (position <= 3 ? 200U : 4003U));
        assert(check_power_off_position(250, bottom, 0, 1, 0) == 4003U);
        assert(check_power_off_position(250, bottom, 0, 0, 14) == 5005U);

        reset_controller();
        idle_samples(3000, bottom, 250, 0, 0);
        assert(!count_command('R','S'));
        status(0,0,3255); idle_poll(3256,0);
        assert(count_command('R','S') == 1 && test_reset_marker);

        reset_controller();
        start_running_with_limits(WING_ACTION_DESCEND, "down", 5, 250, bottom);
        WingController_Task(356); status(4,1,357);
        assert(!count_command('S','T') && WingController_GetSnapshot()->at_endpoint == WING_ENDPOINT_NONE);
        WingController_Task(458); status(3,1,459);
        assert(count_command('S','T') == 1 && WingController_GetSnapshot()->at_endpoint == WING_ENDPOINT_BOTTOM);
    }
    assert(check_power_off_position(100, 10, 9, 0, 0) == 4003U);
    assert(check_power_off_position(100, 10, 12, 0, 0) == 200U);

    reset_controller();
    idle_attempt(3000, 0);
    assert(count_command('R','S') == 1);
    receive(rs_ack,3257); status(0,1,3258);
    for (unsigned t = 3358; t <= 3661; t += 101)
    {
        idle_poll(t,0); status(0,0,t+1);
    }
    assert(count_command('S','T') == 1 && test_reset_marker);
    ack(WING_ACTION_STOP,3663); status(0,0,3664);
    assert(!reset_errors);
    idle_attempt(6700,0);
    assert(test_reset_marker && count_command('R','S') == 1);
    assert(WingController_Init());
    idle_attempt(10000,0);
    assert(test_reset_marker && count_command('R','S') == 1);
    idle_attempt(13500,1);
    assert(!test_reset_marker);
    idle_attempt(17000,0);
    assert(count_command('R','S') == 2);
}

static void test_idle_reset_interlocks(void)
{
    reset_controller();
    idle_poll(3000,1); assert(g_tx_count == 0);
    idle_samples(6000,1,250,251,250);
    assert(!test_reset_marker && !count_command('R','S'));
    idle_poll(9300,0); limits(250,1,9301); status(251,1,9302);
    idle_poll(9600,0);
    assert(!count_command('R','S') && !count_command('S','T'));
    idle_poll(13000,0); limits(250,1,13001); status(251,0,13002);
    idle_poll(13252,0); idle_poll(13553,0); status(251,0,13554);
    assert(!test_reset_marker && !count_command('R','S'));
    test_raw_contact = 1;
    idle_attempt(17000,251);
    test_raw_contact = 0; test_sonar_stale = 1;
    idle_attempt(20500,251);
    test_sonar_stale = 0; test_ultrasonic_blocked = 1;
    idle_attempt(24000,251);
    assert(!test_reset_marker && !count_command('R','S'));
    test_ultrasonic_blocked = 0;
    idle_samples(27500,1,250,251,251);
    idle_poll(27755,1); /* An earlier queued request wins even after marking. */
    assert(!count_command('R','S'));
    idle_attempt(31000,251);
    assert(count_command('R','S') == 1);

    reset_controller();
    start_running_with_limits(WING_ACTION_ASCEND,"remote",30,250,1);
    g_tick = 4000U;
    WingController_AutoResetTask(0U);
    assert(count_command('R','L') == 1 && !count_command('R','S'));
    reset_controller();
    pinch_start(50,100);
    g_tick = 4000U;
    WingController_AutoResetTask(0U);
    assert(count_command('M','O') == 1 && !count_command('R','S'));
}

static void test_sonar_during_auto_reset(void)
{
    const uint8_t rs_ack[8] = { 'R', 'S' };
    const uint8_t rs_rejected[8] = { 'R', 'S', 0, 0, 0, 0, 0, 1 };

    reset_controller();
    idle_samples(3000, 1, 250, 251, 251);
    assert(test_reset_lamp_tick == 0);
    status(251,0,3255); idle_poll(3256,0);
    assert(count_command('R','S') == 1 && WingController_AutoResetActive());
    assert(test_reset_lamp_moving && test_reset_lamp_tick == 3256 && test_ultrasonic_suppressed);
    test_ultrasonic_blocked = 1;
    WingController_UltrasonicStop();
    idle_poll(3257, 0);
    assert(!count_command('S','T'));
    receive(rs_ack,3258);
    status(0,1,3259);
    test_sonar_stale = 1;
    idle_poll(3360,0); status(0,1,3361);
    assert(test_reset_lamp_moving && test_reset_lamp_tick == 3361);
    assert(!count_command('S','T') && test_anti_pinch_scope);
    for (unsigned tick = 3462; tick <= 3765; tick += 101)
    {
        idle_poll(tick,0);
        status(0,0,tick+1);
        assert(!test_reset_lamp_moving && test_reset_lamp_tick == tick + 1);
    }
    assert(count_command('S','T') == 1 && WingController_AutoResetActive());
    ack(WING_ACTION_STOP,3767);
    WingController_UltrasonicStop();
    status(0,1,3768);
    assert(test_reset_lamp_moving && test_reset_lamp_tick == 3768);
    assert(WingController_AutoResetActive() && count_command('S','T') == 1);
    idle_poll(3870,0); status(0,0,3871);
    assert(!test_reset_lamp_moving && test_reset_lamp_tick == 3871);
    assert(!WingController_AutoResetActive() && !reset_errors);
    assert(WingController_Request(WING_ACTION_ASCEND,"obstacle-after-reset") == 4003U);
    test_ultrasonic_blocked = 0;
    idle_attempt(7000,1);
    assert(!test_reset_marker);
    idle_attempt(10500,251); /* Stale sonar blocks the next RS. */
    assert(count_command('R','S') == 1);
    test_sonar_stale = 0;
    idle_attempt(14000,251);
    assert(count_command('R','S') == 2);

    test_ultrasonic_blocked = 1;
    receive(rs_rejected,14257);
    assert(!test_reset_lamp_moving && test_reset_lamp_tick == 14257);
    assert(!WingController_AutoResetActive() && reset_errors);
    assert(WingController_Request(WING_ACTION_ASCEND,"obstacle-after-reject") == 4003U);

    reset_controller();
    idle_attempt(3000,251);
    receive(rs_ack,3257);
    test_ultrasonic_blocked = 1;
    WingController_AntiPinch(1,1);
    assert(count_command('S','T') == 1 && !count_command('M','O'));
    idle_poll(4258,0); /* Missing ST feedback still enters the fault state. */
    assert(!WingController_AutoResetActive() && reset_errors);
    assert(WingController_GetSnapshot()->can_session == WING_CAN_SESSION_FAULT);
    assert(!test_reset_lamp_moving && test_reset_lamp_tick == 4258);

    reset_controller();
    idle_samples(3000,1,250,0,0);
    status(0,0,3255); g_fail_tx = 1U; idle_poll(3256,0);
    assert(!count_command('R','S') && !test_reset_lamp_tick);
}

static void test_idle_reset_failure_latch(void)
{
    reset_controller();
    test_marker_write_fail = 1;
    idle_attempt(3000,251);
    assert(reset_errors && !count_command('R','S'));
    test_marker_write_fail = 0;
    idle_attempt(6500,251);
    assert(count_command('R','S') == 1);
    idle_poll(7800,0); /* RS ACK never arrived, request a confirmed stop. */
    assert(count_command('S','T') == 1 && test_reset_marker);
    ack(WING_ACTION_STOP,7801); status(251,0,7802);
    idle_attempt(11000,251);
    assert(count_command('R','S') == 1);
    assert(WingController_Init()); /* RAM lost; persistent marker retained. */
    idle_attempt(15000,251);
    assert(count_command('R','S') == 1 && test_reset_marker);
    idle_samples(18500,1,250,1,2); /* Moving between samples cannot rearm. */
    assert(test_reset_marker);
    idle_samples(22000,1,250,250,250);
    assert(!test_reset_marker);
    idle_attempt(25500,251);
    assert(count_command('R','S') == 2);
    WingController_AntiPinch(1U,1U);
    assert(count_command('S','T') == 2 && !count_command('M','O'));
}

static void test_reset_still_outside(void)
{
    const uint8_t rs_ack[8] = {'R','S'};
    reset_controller();
    idle_attempt(3000,251);
    receive(rs_ack,3257); status(251,1,3258);
    for (unsigned t = 3358; t <= 4368; t += 101)
    {
        idle_poll(t,0); status(251,0,t+1);
    }
    assert(count_command('S','T') == 1 && reset_errors && test_reset_marker);
    ack(WING_ACTION_STOP,4370); status(251,0,4371);
    idle_attempt(7600,251);
    assert(count_command('R','S') == 1);
    unsigned errors = reset_errors;
    WingController_LogAutoResetAlarm();
    assert(reset_errors == errors + 1);
    idle_samples(11000,1,250,250,250);
    errors = reset_errors;
    WingController_LogAutoResetAlarm();
    assert(!test_reset_marker && reset_errors == errors);

    reset_controller();
    idle_poll(3000,0); limits(250,250,3001);
    assert(!count_command('R','D'));
    idle_poll(6500,0); limits(250,1,6501); status_with_fault(0,0,14,6502);
    idle_poll(6752,0); status(0,0,6753); idle_poll(6754,0);
    assert(!test_reset_marker && !count_command('R','S'));
}

int main(void)
{
    test_reset_still_outside();
    test_idle_range_recovery();
    test_retracted_boundaries();
    test_idle_reset_interlocks();
    test_sonar_during_auto_reset();
    test_idle_reset_failure_latch();
    reset_controller();
    g_can_state = HAL_CAN_STATE_RESET;
    assert(!WingController_Init());
    assert(g_can_init_calls == 0U);
    assert(WingController_Request(WING_ACTION_ASCEND, "not-ready") == 5001U);
    assert(WingController_Request(WING_ACTION_STOP, "stop-not-ready") == 5001U);
    assert(WingController_CheckPowerOff("power-not-ready") == 5001U);
    assert(g_tx_count == 0U);
    g_can_state = HAL_CAN_STATE_LISTENING;
    assert(WingController_Init());
    test_idle_silent_and_correlation();
    test_preflight_local_motion_rejected();
    test_preflight_timeouts_and_limits();
    test_running_feedback_uses_periodic_mqtt_policy();
    test_endpoint_single_stop();
    test_conflict_and_unexpected_stop();
    test_stop_preemption_and_safety();
    test_direction_change_fault_and_max_runtime();
    test_pinch_bounded_move_and_hold();
    test_pinch_stop_confirmation_and_failures();
    test_pinch_preemption_and_operator_stop();
    test_pinch_move_abort_paths();
    test_lost_start_ack_and_unused_bus();
    test_ultrasonic_stop_and_no_resume();
    test_ultrasonic_idle_is_alarm_only();
    test_ultrasonic_preempts_can_callback_and_pinch();
    test_ultrasonic_stop_failure_stays_locked();
    test_ultrasonic_preserves_early_pinch_cycle_and_limits();
    test_anti_pinch_motion_scope();
    test_ultrasonic_near_retraction_suppression();
    return 0;
}
