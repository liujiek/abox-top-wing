#include "wing_controller.h"

#include <string.h>

#include "app_log.h"
#include "anti_pinch_board.h"
#include "main.h"
#include "wing_protocol.h"
#include "wing_power.h"
#include "ota_info.h"
#include "ultrasonic_board.h"

#define RESULT_QUEUE_DEPTH 4U
#define REPORT_QUEUE_DEPTH 8U

typedef enum
{
    SESSION_IDLE = 0,
    SESSION_WAIT_RD1,
    SESSION_QUIET_GUARD,
    SESSION_WAIT_RD2,
    SESSION_WAIT_RL,
    SESSION_WAIT_START_ACK,
    SESSION_WAIT_START_STATUS,
    SESSION_RUNNING,
    SESSION_WAIT_STOP_ACK,
    SESSION_WAIT_STOP_STATUS,
    SESSION_FAULT,
    SESSION_WAIT_POWER_STATUS,
    SESSION_AUTO_RESET
} SessionState;

typedef enum { QUERY_NONE = 0, QUERY_RD, QUERY_RL } ExpectedQuery;
typedef enum { STOP_USER = 0, STOP_ENDPOINT, STOP_SAFETY, STOP_CONFLICT } StopCause;

typedef struct
{
    SessionState state;
    ExpectedQuery expected_query;
    WingAction action;
    StopCause stop_cause;
    uint32_t phase_tick;
    uint32_t query_tick;
    uint32_t last_poll_tick;
    uint32_t stop_tick;
    uint16_t best_position_mm;
    uint8_t conflict_samples;
    uint8_t direction_result_sent;
    uint8_t stop_command_sent;
    uint8_t power_off_check;
    char direction_request_id[65];
    char stop_request_id[65];
} WingSession;

static WingState g_state;
static uint8_t g_can_ready;
static WingSession g_session;
static WingControlResult g_results[RESULT_QUEUE_DEPTH];
static uint8_t g_result_head;
static uint8_t g_result_count;
static WingReport g_reports[REPORT_QUEUE_DEPTH];
static uint8_t g_report_head;
static uint8_t g_report_count;
static WingSnapshot g_confirmed_stop;
static uint8_t g_have_confirmed_stop;
typedef enum { PINCH_NONE, PINCH_STOPPING, PINCH_LIMITS, PINCH_MOVE_ACK,
               PINCH_MOVING, PINCH_FINAL_STOP, PINCH_HOLD, PINCH_FAILED } PinchPhase;
static struct {
    PinchPhase phase;
    uint32_t tick, feedback_tick;
    uint16_t start_mm, target_mm, best_mm;
} g_pinch;
static uint8_t g_pinch_pressed;

typedef enum {
    AUTO_IDLE, AUTO_LIMITS, AUTO_FIRST, AUTO_QUIET, AUTO_SECOND,
    AUTO_DECIDE, AUTO_FINAL_CHECK, AUTO_SEND, AUTO_ACK, AUTO_TRACK
} AutoPhase;

static struct {
    AutoPhase phase;
    const char* alarm;
    uint32_t tick, started, stopped_since;
    uint16_t bottom, top, position;
    WingLowerTravel lower;
    uint8_t saw_motion, settling, warned, reserved_unsent;
} auto_reset;

static void end_probe(uint32_t now)
{
    if (auto_reset.phase >= AUTO_ACK) AntiPinchBoard_UpdateResetLamp(now, 0U);
    auto_reset.phase = AUTO_IDLE;
    auto_reset.tick = now;
}

uint8_t WingController_AutoResetActive(void)
{
    return auto_reset.phase >= AUTO_ACK;
}

static uint8_t reset_path_clear(uint32_t now)
{
    /* Require fresh, clear sonar until RS is dispatched. Power and contact
     * protection remain mandatory throughout the reset. */
    return WingPower_IsOn() && !AntiPinchBoard_ContactActive() &&
           (WingController_AutoResetActive() || UltrasonicBoard_PathConfirmedClear(now));
}

/* A confirmed idle zero triggers recovery, even when RL allows zero. */
static uint16_t auto_reset_minimum_mm(void)
{
    return auto_reset.bottom ? auto_reset.bottom : 1U;
}

static uint8_t at_reset_endpoint(uint16_t position)
{
    return position >= auto_reset.lower.minimum_mm && position <= auto_reset.lower.endpoint_max_mm &&
           position <= auto_reset.top;
}

void WingController_LogAutoResetAlarm(void)
{
    const char* alarm = auto_reset.alarm;
    if (alarm) APP_LOGE("AUTO_RS", "unresolved: %s", alarm);
}

static void push_report(WingReportReason reason);

static void reset_alarm(const char* reason)
{
    auto_reset.alarm = reason;
    APP_LOGE("AUTO_RS", "%s pos=%u range=%u..%u", reason, (unsigned)auto_reset.position,
             (unsigned)auto_reset_minimum_mm(), (unsigned)auto_reset.top);
    push_report(WING_REPORT_STATE_CHANGED);
}


static void stop_failed(uint16_t code);
static uint8_t enforce_ultrasonic_stop(uint32_t now);
static uint8_t ultrasonic_near_retraction_suppressed(void)
{
    const WingSnapshot* snapshot = &g_state.snapshot;
    return snapshot->have_position &&
           snapshot->position_mm <= WING_ULTRASONIC_SUPPRESS_MAX_MM &&
           (snapshot->active_action == WING_ACTION_DESCEND ||
            (snapshot->active_action == WING_ACTION_ASCEND &&
             snapshot->position_mm >= WING_ULTRASONIC_ASCEND_SUPPRESS_MIN_MM));
}

static void sync_ultrasonic_suppression(void)
{
    uint8_t near = ultrasonic_near_retraction_suppressed();
    if (near) UltrasonicBoard_ClearAlarmHold();
    UltrasonicBoard_SetAlarmSuppressed((uint8_t)(WingController_AutoResetActive() || g_session.state == SESSION_IDLE ||
                                                g_session.power_off_check || near));
}

static uint8_t pinch_backoff_active(void)
{
    return g_pinch.phase == PINCH_STOPPING || g_pinch.phase == PINCH_LIMITS ||
           g_pinch.phase == PINCH_MOVE_ACK || g_pinch.phase == PINCH_MOVING ||
           g_pinch.phase == PINCH_FINAL_STOP;
}

static uint8_t anti_pinch_scope_eligible(void)
{
    const WingSnapshot* snapshot = &g_state.snapshot;
    return g_session.state == SESSION_AUTO_RESET || (snapshot->have_position &&
           snapshot->motion_state == WING_MOTION_DESCENDING);
}

static void sync_anti_pinch_scope(void)
{
    AntiPinchBoard_SetScopeEnabled(
        (uint8_t)(anti_pinch_scope_eligible() || g_pinch.phase != PINCH_NONE));
}

static CAN_HandleTypeDef* selected_can(void)
{
#if TOP_WING_CAN_BUS == 1
    return &hcan1;
#else
    return &hcan2;
#endif
}

static void copy_id(char dst[65], const char* src)
{
    if (!src) { dst[0] = '\0'; return; }
    strncpy(dst, src, 64U);
    dst[64] = '\0';
}

static void push_report(WingReportReason reason)
{
    uint8_t tail;
    if (reason == WING_REPORT_NONE) return;
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (g_report_count &&
        g_reports[(uint8_t)((g_report_head + g_report_count - 1U) % REPORT_QUEUE_DEPTH)].reason == reason)
    {
        tail = (uint8_t)((g_report_head + g_report_count - 1U) % REPORT_QUEUE_DEPTH);
        goto capture;
    }
    if (g_report_count >= REPORT_QUEUE_DEPTH)
    {
        g_report_head = (uint8_t)((g_report_head + 1U) % REPORT_QUEUE_DEPTH);
        g_report_count--;
    }
    tail = (uint8_t)((g_report_head + g_report_count) % REPORT_QUEUE_DEPTH);
    g_report_count++;
capture:
    g_reports[tail].reason = reason;
    g_reports[tail].snapshot = g_state.snapshot;
    g_reports[tail].tick = HAL_GetTick();
    copy_id(g_reports[tail].request_id, g_session.stop_request_id[0] ?
            g_session.stop_request_id : g_session.direction_request_id);
    __set_PRIMASK(irq);
}

static void push_result(const char* request_id, WingAction action, uint16_t code)
{
    uint8_t tail;
    if (!request_id || !request_id[0]) return;
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (g_result_count >= RESULT_QUEUE_DEPTH)
    {
        g_result_head = (uint8_t)((g_result_head + 1U) % RESULT_QUEUE_DEPTH);
        g_result_count--;
    }
    tail = (uint8_t)((g_result_head + g_result_count) % RESULT_QUEUE_DEPTH);
    memset(&g_results[tail], 0, sizeof(g_results[tail]));
    copy_id(g_results[tail].request_id, request_id);
    g_results[tail].action = action;
    g_results[tail].code = code;
    g_results[tail].accepted = code == 200U ? 1U : 0U;
    g_results[tail].snapshot = g_state.snapshot;
    g_results[tail].observed_tick = HAL_GetTick();
    g_results[tail].stop_confirmed = action == WING_ACTION_STOP && code == 200U &&
        g_state.snapshot.status_fresh && !g_state.raw_moving && !g_state.snapshot.fault_code;
    g_result_count++;
    __set_PRIMASK(irq);
}

static uint8_t send_frame(const uint8_t data[8])
{
    CAN_TxHeaderTypeDef header;
    uint32_t mailbox;
    if (!g_can_ready) return 0U;
    memset(&header, 0, sizeof(header));
    header.ExtId = WING_CAN_TX_ID;
    header.IDE = CAN_ID_EXT;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8U;
    header.TransmitGlobalTime = DISABLE;
    return HAL_CAN_AddTxMessage(selected_can(), &header, (uint8_t*)data, &mailbox) == HAL_OK ? 1U : 0U;
}

static uint8_t send_action(WingAction action, const char* request_id)
{
    uint8_t data[8];
    uint8_t ok;
    if (!WingProtocol_EncodeAction(action, data)) return 0U;
    ok = send_frame(data);
    if (ok)
        APP_LOGI("CAN", "tx rid=%s act=%s id=%08lX", request_id && request_id[0] ? request_id : "internal",
                 WingAction_Name(action), (unsigned long)WING_CAN_TX_ID);
    else
        APP_LOGE("CAN", "tx fail rid=%s act=%s", request_id && request_id[0] ? request_id : "internal",
                 WingAction_Name(action));
    return ok;
}

static uint8_t send_query(ExpectedQuery query, uint32_t now)
{
    uint8_t data[8];
    if (query == QUERY_RD) WingProtocol_EncodeStatusQuery(data);
    else if (query == QUERY_RL) WingProtocol_EncodeLimitQuery(data);
    else return 0U;
    if (!send_frame(data)) return 0U;
    g_session.expected_query = query;
    g_session.query_tick = now;
    g_session.last_poll_tick = now;
    APP_LOGI("CAN", "tx query=%s rid=%s", query == QUERY_RD ? "RD" : "RL",
             g_session.direction_request_id[0] ? g_session.direction_request_id : g_session.stop_request_id);
    return 1U;
}

static void clear_session(void)
{
    memset(&g_session, 0, sizeof(g_session));
    g_session.state = SESSION_IDLE;
}

static void finish_idle(WingControlOwner owner)
{
    char request_id[65];
    copy_id(request_id, g_session.stop_request_id[0] ? g_session.stop_request_id : g_session.direction_request_id);
    clear_session();
    WingState_SetIdleSilent(&g_state, owner);
    sync_ultrasonic_suppression();
    sync_anti_pinch_scope();
    push_report(WING_REPORT_STATE_CHANGED);
    /* Preserve the observation at stop confirmation, before idle-silent
     * deliberately invalidates live feedback. This does not change control. */
    {
        WingReport* report = &g_reports[(g_report_head + g_report_count - 1U) % REPORT_QUEUE_DEPTH];
        copy_id(report->request_id, request_id);
        if (g_have_confirmed_stop) report->snapshot = g_confirmed_stop;
        g_have_confirmed_stop = 0U;
    }
}

static void fail_direction(uint16_t code, WingControlOwner owner)
{
    push_result(g_session.direction_request_id, g_session.action, code);
    APP_LOGW("WING", "preflight/start failed rid=%s act=%s code=%u",
             g_session.direction_request_id, WingAction_Name(g_session.action), (unsigned)code);
    finish_idle(owner);
}

static uint8_t begin_stop(StopCause cause, const char* stop_request_id, uint32_t now)
{
    const char* tx_id = stop_request_id;
    uint8_t aborted = 1U;
    if (g_session.stop_command_sent) return 1U;
    if (g_session.state == SESSION_AUTO_RESET)
    {
        if (auto_reset.phase == AUTO_ACK)
            aborted = HAL_CAN_AbortTxRequest(selected_can(), CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2) == HAL_OK;
        if (cause != STOP_ENDPOINT) reset_alarm("interrupted/unconfirmed");
    }
    if (!g_session.direction_result_sent && g_session.direction_request_id[0])
    {
        push_result(g_session.direction_request_id, g_session.action, 5003U);
        g_session.direction_result_sent = 1U;
    }
    g_session.expected_query = QUERY_NONE;
    g_session.stop_cause = cause;
    copy_id(g_session.stop_request_id, stop_request_id);
    if (!tx_id || !tx_id[0]) tx_id = g_session.direction_request_id;
    if (!send_action(WING_ACTION_STOP, tx_id) || !aborted)
    {
        stop_failed(5001U);
        return 0U;
    }
    g_session.stop_command_sent = 1U;
    g_session.stop_tick = now;
    g_session.phase_tick = now;
    g_session.state = SESSION_WAIT_STOP_ACK;
    WingState_SetSession(&g_state, WING_CAN_SESSION_STOPPING, WING_PHASE_STOPPING,
                         cause == STOP_CONFLICT ? WING_CONTROL_OWNER_LOCAL_SUSPECTED : WING_CONTROL_OWNER_REMOTE);
    if (cause == STOP_ENDPOINT) push_report(WING_REPORT_ENDPOINT_STOP);
    else if (cause == STOP_CONFLICT) push_report(WING_REPORT_CONTROL_CONFLICT);
    else if (cause == STOP_SAFETY) push_report(WING_REPORT_SAFETY_STOP);
    return 1U;
}

static void stop_failed(uint16_t code)
{
    if (auto_reset.phase >= AUTO_ACK)
    {
        reset_alarm("stop unconfirmed");
        end_probe(HAL_GetTick());
    }
    push_result(g_session.stop_request_id, WING_ACTION_STOP, code);
    if (g_pinch.phase != PINCH_NONE) g_pinch.phase = PINCH_FAILED;
    clear_session();
    g_session.state = SESSION_FAULT;
    WingState_SetSession(&g_state, WING_CAN_SESSION_FAULT, WING_PHASE_STOPPING,
                         WING_CONTROL_OWNER_NONE);
    push_report(WING_REPORT_STATE_CHANGED);
}

uint8_t WingController_IsUltrasonicBlocked(void)
{
    return UltrasonicBoard_ObstacleActive(HAL_GetTick());
}

uint8_t WingController_RejectRemoteStartForUltrasonic(void)
{
    uint32_t irq = __get_PRIMASK();
    uint8_t rejected = 0U;
    __disable_irq();
    if (g_session.state == SESSION_IDLE && WingController_IsUltrasonicBlocked())
    {
        UltrasonicBoard_StartPrestartWarning(HAL_GetTick());
        rejected = 1U;
    }
    __set_PRIMASK(irq);
    if (rejected) APP_LOGW("SONAR", "prestart obstacle: 2s warning, movement rejected");
    return rejected;
}

static void ultrasonic_stop(uint32_t now)
{
    if (WingController_AutoResetActive()) return;
    if (g_session.state == SESSION_IDLE ||
        ultrasonic_near_retraction_suppressed())
    {
        sync_ultrasonic_suppression();
        return;
    }
    /* An already-started pinch protection cycle has priority over sonar.
     * Keep its bounded target and its final stop confirmation unchanged. */
    if (pinch_backoff_active())
    {
        APP_LOGI("SONAR", "obstacle: preserve active pinch backoff");
        return;
    }
    if (g_session.state == SESSION_WAIT_STOP_ACK || g_session.state == SESSION_WAIT_STOP_STATUS)
        return;
    int aborted = HAL_CAN_AbortTxRequest(selected_can(),
        CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2) == HAL_OK;
    g_session.stop_command_sent = 0U;
    APP_LOGW("SONAR", "obstacle: cancel motion and confirm stop");
    (void)begin_stop(STOP_SAFETY, "", now);
    if (!aborted) stop_failed(5001U);
}

void WingController_UltrasonicStop(void)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    ultrasonic_stop(HAL_GetTick());
    __set_PRIMASK(irq);
}

static uint8_t enforce_ultrasonic_stop(uint32_t now)
{
    if (g_session.state >= SESSION_WAIT_RD1 && g_session.state <= SESSION_RUNNING &&
        !pinch_backoff_active() && !ultrasonic_near_retraction_suppressed() &&
        WingController_IsUltrasonicBlocked())
    {
        ultrasonic_stop(now);
        return 1U;
    }
    return 0U;
}

static void finish_stop(void)
{
    PinchPhase pinch_phase = g_pinch.phase;
    uint16_t stopped_mm = g_state.snapshot.position_mm;
    WingControlOwner owner = g_session.stop_cause == STOP_CONFLICT ?
                             WING_CONTROL_OWNER_LOCAL_SUSPECTED : WING_CONTROL_OWNER_NONE;
    if (g_state.snapshot.fault_code) { stop_failed(5005U); return; }
    if (auto_reset.phase >= AUTO_ACK)
    {
        if (g_session.stop_cause == STOP_ENDPOINT)
        {
            if (at_reset_endpoint(stopped_mm) && stopped_mm == auto_reset.position)
                APP_LOGI("AUTO_RS", "confirmed stopped at endpoint pos=%u", (unsigned)stopped_mm);
            else reset_alarm("final position unconfirmed");
        }
        end_probe(HAL_GetTick());
    }
    if (g_session.stop_request_id[0])
        push_result(g_session.stop_request_id, WING_ACTION_STOP, 200U);
    WingState_ConfirmStop(&g_state, g_session.stop_request_id, g_session.stop_request_id[0] ? 1U : 0U);
    g_have_confirmed_stop = g_state.snapshot.status_fresh && !g_state.snapshot.fault_code;
    g_confirmed_stop = g_state.snapshot;
    finish_idle(owner);
    if (pinch_phase == PINCH_STOPPING)
    {
        g_pinch.start_mm = stopped_mm;
        g_pinch.phase = PINCH_LIMITS;
        g_pinch.tick = HAL_GetTick();
        WingState_InvalidateLimits(&g_state);
        WingState_SetSession(&g_state, WING_CAN_SESSION_PREFLIGHT, WING_PHASE_CHECKING_LIMITS,
                             WING_CONTROL_OWNER_NONE);
        if (!send_query(QUERY_RL, g_pinch.tick)) stop_failed(5001U);
    }
    else if (pinch_phase == PINCH_FINAL_STOP) g_pinch.phase = PINCH_HOLD;
}

static void pinch_stop(uint32_t now)
{
    g_pinch.phase = PINCH_FINAL_STOP;
    g_session.stop_command_sent = 0U;
    (void)begin_stop(STOP_SAFETY, "", now);
}

/* Consume only the internal position-move phases. Stop uses the ordinary
 * ST ACK + fresh RD stopped confirmation path, never an ACK alone. */
static uint8_t pinch_receive(const WingCanMessage* msg, uint32_t now)
{
    if (g_pinch.phase == PINCH_LIMITS)
    {
        uint32_t target;
        uint8_t data[8];
        if (msg->kind != WING_FRAME_LIMITS || g_session.expected_query != QUERY_RL) return 1U;
        g_session.expected_query = QUERY_NONE;
        if (!WingState_ApplyLimits(&g_state, msg->top_limit_mm, msg->bottom_limit_mm) ||
            g_pinch.start_mm < msg->bottom_limit_mm)
        { stop_failed(5003U); return 1U; }
        target = (uint32_t)g_pinch.start_mm + WING_ANTI_PINCH_BACKOFF_MM;
        if (target > msg->top_limit_mm) target = msg->top_limit_mm;
        if (target <= g_pinch.start_mm)
        {
            g_pinch.phase = PINCH_HOLD;
            finish_idle(WING_CONTROL_OWNER_NONE);
            APP_LOGI("PINCH", "already at upper limit; no upward move");
            return 1U;
        }
        g_pinch.target_mm = (uint16_t)target;
        g_pinch.best_mm = g_pinch.start_mm;
        WingProtocol_EncodeMoveTo(g_pinch.target_mm, data);
        g_pinch.phase = PINCH_MOVE_ACK;
        g_pinch.tick = now;
        if (!send_frame(data)) { pinch_stop(now); return 1U; }
        WingState_SetSession(&g_state, WING_CAN_SESSION_PREFLIGHT, WING_PHASE_STARTING,
                             WING_CONTROL_OWNER_NONE);
        APP_LOGI("PINCH", "MO start=%u target=%u top=%u", (unsigned)g_pinch.start_mm,
                 (unsigned)g_pinch.target_mm, (unsigned)msg->top_limit_mm);
        return 1U;
    }
    if (g_pinch.phase == PINCH_MOVE_ACK)
    {
        if (msg->kind != WING_FRAME_MOVE_ACK || msg->position_mm != g_pinch.target_mm) return 1U;
        if (!msg->accepted) { pinch_stop(now); return 1U; }
        g_pinch.phase = PINCH_MOVING;
        g_pinch.tick = now;
        WingState_AcceptDirection(&g_state, WING_ACTION_ASCEND, "", now);
        g_state.snapshot.control_owner = WING_CONTROL_OWNER_NONE;
        /* An ACK starts a deadline; it is not fresh position feedback. */
        g_pinch.feedback_tick = now;
        if (!send_query(QUERY_RD, now)) pinch_stop(now);
        return 1U;
    }
    if (g_pinch.phase == PINCH_MOVING)
    {
        if (msg->kind != WING_FRAME_STATUS || !msg->accepted || g_session.expected_query != QUERY_RD) return 1U;
        g_session.expected_query = QUERY_NONE;
        WingState_ApplyFeedback(&g_state, now, msg->position_mm, msg->fault_code, msg->moving);
        g_pinch.feedback_tick = now;
        if (msg->fault_code || msg->position_mm >= g_pinch.target_mm ||
            (uint32_t)msg->position_mm + WING_DIRECTION_CONFLICT_MM < g_pinch.best_mm ||
            (!msg->moving && now - g_pinch.tick >= WING_COMMAND_TIMEOUT_MS))
        {
            APP_LOGI("PINCH", "ending move pos=%u target=%u moving=%u fault=%u",
                     (unsigned)msg->position_mm, (unsigned)g_pinch.target_mm,
                     (unsigned)msg->moving, (unsigned)msg->fault_code);
            pinch_stop(now);
        }
        else if (msg->position_mm > g_pinch.best_mm) g_pinch.best_mm = msg->position_mm;
        return 1U;
    }
    return g_pinch.phase == PINCH_FAILED || g_pinch.phase == PINCH_HOLD;
}

static uint8_t pinch_task(uint32_t now)
{
    if (g_pinch.phase == PINCH_LIMITS && now - g_pinch.tick >= WING_QUERY_TIMEOUT_MS)
        stop_failed(5003U);
    else if (g_pinch.phase == PINCH_MOVE_ACK && now - g_pinch.tick >= WING_COMMAND_TIMEOUT_MS)
        pinch_stop(now);
    else if (g_pinch.phase == PINCH_MOVING)
    {
        if (WingState_NeedsSafetyStop(&g_state) ||
            now - g_pinch.feedback_tick > WING_ACTIVE_FEEDBACK_TIMEOUT_MS ||
            now - g_pinch.tick >= WING_ANTI_PINCH_MOVE_TIMEOUT_MS)
            pinch_stop(now);
        else if (g_session.expected_query == QUERY_NONE && now - g_session.last_poll_tick >= WING_ACTIVE_STATUS_POLL_MS)
        {
            if (!send_query(QUERY_RD, now)) pinch_stop(now);
        }
    }
    return g_pinch.phase == PINCH_LIMITS || g_pinch.phase == PINCH_MOVE_ACK ||
           g_pinch.phase == PINCH_MOVING || g_pinch.phase == PINCH_FAILED || g_pinch.phase == PINCH_HOLD;
}

static void assess_running_status(uint32_t now)
{
    uint16_t pos = g_state.snapshot.position_mm;
    uint8_t reversed = 0U;
    if (g_state.snapshot.fault_code)
    {
        WingState_LatchSafety(&g_state, WING_SAFETY_CONTROLLER_FAULT);
        (void)begin_stop(STOP_SAFETY, "", now);
        return;
    }
    if (WingState_EndpointReached(&g_state, g_session.action))
    {
        (void)begin_stop(STOP_ENDPOINT, "", now);
        return;
    }
    if (!g_state.raw_moving)
    {
        (void)begin_stop(STOP_CONFLICT, "", now);
        return;
    }
    if (g_session.action == WING_ACTION_ASCEND)
    {
        if (pos > g_session.best_position_mm) g_session.best_position_mm = pos;
        else if ((uint32_t)pos + WING_DIRECTION_CONFLICT_MM < g_session.best_position_mm) reversed = 1U;
    }
    else
    {
        if (pos < g_session.best_position_mm) g_session.best_position_mm = pos;
        else if ((uint32_t)pos > (uint32_t)g_session.best_position_mm + WING_DIRECTION_CONFLICT_MM) reversed = 1U;
    }
    if (reversed)
    {
        if (g_session.conflict_samples < 255U) g_session.conflict_samples++;
        if (g_session.conflict_samples >= WING_DIRECTION_CONFLICT_SAMPLES)
            (void)begin_stop(STOP_CONFLICT, "", now);
    }
    else g_session.conflict_samples = 0U;
}

uint8_t WingController_Init(void)
{
    memset(&auto_reset, 0, sizeof(auto_reset));
    end_probe(HAL_GetTick());
    memset(&g_pinch, 0, sizeof(g_pinch));
    g_pinch_pressed = 0U;
    g_have_confirmed_stop = 0U;
    g_can_ready = 0U;
    WingState_Init(&g_state);
    clear_session();
    sync_ultrasonic_suppression();
    sync_anti_pinch_scope();
    g_result_head = g_result_count = 0U;
    g_report_head = g_report_count = 0U;
    /* MX_CANx_Init owns filter, start and RX interrupt setup and checks each
     * HAL result. Starting an already listening STM32 CAN returns HAL_ERROR. */
    if (HAL_CAN_GetState(selected_can()) != HAL_CAN_STATE_LISTENING)
    {
        APP_LOGE("WING", "CAN%u is not listening", (unsigned)TOP_WING_CAN_BUS);
        return 0U;
    }
    g_can_ready = 1U;
    APP_LOGI("WING", "CAN%u idle=limit-monitor tx=0x%08lX rx=0x%08lX",
             (unsigned)TOP_WING_CAN_BUS, (unsigned long)WING_CAN_TX_ID,
             (unsigned long)WING_CAN_RX_ID);
    return 1U;
}

static uint16_t controller_request(WingAction action, const char* request_id, uint8_t untracked_stop)
{
    uint32_t now = HAL_GetTick();
    uint16_t code;
    if (!request_id || (!request_id[0] && !(untracked_stop && action == WING_ACTION_STOP)) ||
        action == WING_ACTION_NONE) return 4001U;
    if (!WingPower_IsOn()) return 5002U;
    if (!g_can_ready) return 5001U;

    /* An operator request supersedes read-only background observations. */
    if (auto_reset.phase < AUTO_ACK) end_probe(now);
    if (action == WING_ACTION_STOP)
    {
        if (g_session.power_off_check) fail_direction(4003U, WING_CONTROL_OWNER_NONE);
        /* An explicit operator stop cancels any automatic upward continuation. */
        if (g_pinch.phase != PINCH_NONE) g_pinch.phase = PINCH_FINAL_STOP;
        /* Idle also needs feedback confirmation for an explicit stop. */
        if (g_session.state == SESSION_WAIT_STOP_ACK || g_session.state == SESSION_WAIT_STOP_STATUS)
            return 5003U;
        return begin_stop(STOP_USER, request_id, now) ? 0U : 5001U;
    }

    if (g_pinch.phase != PINCH_NONE || g_pinch_pressed) return 4003U;
    code = WingState_AdmissionCode(&g_state, action);
    if (code) return code;
    if (WingController_RejectRemoteStartForUltrasonic()) return 4003U;
    clear_session();
    g_session.state = SESSION_WAIT_RD1;
    g_session.action = action;
    g_session.phase_tick = now;
    copy_id(g_session.direction_request_id, request_id);
    WingState_InvalidateLimits(&g_state);
    WingState_SetSession(&g_state, WING_CAN_SESSION_PREFLIGHT, WING_PHASE_CHECKING_MOTION,
                         WING_CONTROL_OWNER_NONE);
    push_report(WING_REPORT_STATE_CHANGED);
    if (!send_query(QUERY_RD, now))
    {
        finish_idle(WING_CONTROL_OWNER_NONE);
        return 5001U;
    }
    return 0U;
}

uint16_t WingController_Request(WingAction action, const char* request_id)
{
    uint32_t irq = __get_PRIMASK();
    uint16_t code;
    __disable_irq();
    code = controller_request(action, request_id, 0U);
    __set_PRIMASK(irq);
    return code;
}

uint16_t WingController_RequestUntrackedStop(void)
{
    uint32_t irq = __get_PRIMASK();
    uint16_t code;
    __disable_irq();
    code = controller_request(WING_ACTION_STOP, "", 1U);
    __set_PRIMASK(irq);
    return code;
}

uint16_t WingController_CheckPowerOff(const char* request_id)
{
    uint32_t irq = __get_PRIMASK();
    uint16_t code = 0U;
    uint32_t now = HAL_GetTick();
    __disable_irq();
    if (auto_reset.phase < AUTO_ACK) end_probe(now);
    if (!request_id || !request_id[0]) code = 4001U;
    else if (!WingPower_IsOn()) code = 5002U;
    else if (!g_can_ready) code = 5001U;
    else if (g_session.state != SESSION_IDLE || g_pinch.phase != PINCH_NONE || g_pinch_pressed) code = 4003U;
    else
    {
        clear_session();
        g_session.state = SESSION_WAIT_RD1;
        g_session.power_off_check = 1U;
        g_session.phase_tick = now;
        copy_id(g_session.direction_request_id, request_id);
        WingState_InvalidateLimits(&g_state);
        WingState_SetSession(&g_state, WING_CAN_SESSION_PREFLIGHT, WING_PHASE_CHECKING_MOTION,
                             WING_CONTROL_OWNER_NONE);
        if (!send_query(QUERY_RD, now))
        {
            finish_idle(WING_CONTROL_OWNER_NONE);
            code = 5001U;
        }
    }
    __set_PRIMASK(irq);
    return code;
}

void WingController_AntiPinch(uint8_t pressed, uint8_t trigger)
{
    uint32_t irq = __get_PRIMASK();
    uint8_t eligible;
    __disable_irq();
    eligible = anti_pinch_scope_eligible();
    if (g_pinch.phase != PINCH_NONE) g_pinch_pressed = pressed ? 1U : 0U;
    else g_pinch_pressed = pressed && eligible ? 1U : 0U;
    if (g_session.state == SESSION_AUTO_RESET && pressed && trigger)
    {
        /* The drive controls RS direction internally. Stop without starting MO. */
        g_pinch.phase = PINCH_FINAL_STOP;
        (void)begin_stop(STOP_SAFETY, "", HAL_GetTick());
        sync_anti_pinch_scope();
        __set_PRIMASK(irq);
        return;
    }
    if (!WingController_AutoResetActive() && WingController_IsUltrasonicBlocked())
    {
        if (pressed && trigger && eligible && !pinch_backoff_active())
            g_pinch.phase = PINCH_HOLD;
        sync_anti_pinch_scope();
        __set_PRIMASK(irq);
        return;
    }
    if (g_pinch.phase == PINCH_HOLD && !g_pinch_pressed) g_pinch.phase = PINCH_NONE;
    if (pressed && trigger && eligible && g_pinch.phase == PINCH_NONE)
    {
        g_pinch.phase = PINCH_STOPPING;
        APP_LOGW("PINCH", "contact: stop, confirm, bounded 20mm backoff");
        (void)begin_stop(STOP_SAFETY, "", HAL_GetTick());
    }
    sync_anti_pinch_scope();
    __set_PRIMASK(irq);
}

static void receive_ack(const WingCanMessage* message, uint32_t now)
{
    if (message->kind == WING_FRAME_ACK)
    {
        if (g_session.state == SESSION_WAIT_START_ACK && message->action == g_session.action)
        {
            if (!message->accepted) { fail_direction(5004U, WING_CONTROL_OWNER_NONE); return; }
            g_session.state = SESSION_WAIT_START_STATUS;
            g_session.phase_tick = now;
            if (!send_query(QUERY_RD, now))
                (void)begin_stop(STOP_SAFETY, "", now);
            return;
        }
        if (g_session.state == SESSION_WAIT_STOP_ACK && message->action == WING_ACTION_STOP)
        {
            if (!message->accepted)
            {
                stop_failed(5004U);
                return;
            }
            g_session.state = SESSION_WAIT_STOP_STATUS;
            g_session.phase_tick = now;
            g_session.expected_query = QUERY_NONE;
            (void)send_query(QUERY_RD, now);
        }
        return;
    }

}

static void receive_limits(const WingCanMessage* message, uint32_t now)
{
    if (message->kind == WING_FRAME_LIMITS)
    {
        if (g_session.expected_query != QUERY_RL || g_session.state != SESSION_WAIT_RL) return;
        g_session.expected_query = QUERY_NONE;
        if (!WingState_ApplyLimits(&g_state, message->top_limit_mm, message->bottom_limit_mm))
        {
            fail_direction(5003U, WING_CONTROL_OWNER_NONE);
            return;
        }
        if (g_session.power_off_check)
        {
            if (message->top_limit_mm <= message->bottom_limit_mm)
            {
                fail_direction(5003U, WING_CONTROL_OWNER_NONE);
                return;
            }
            g_session.state = SESSION_WAIT_POWER_STATUS;
            g_session.phase_tick = now;
            if (!send_query(QUERY_RD, now)) fail_direction(5001U, WING_CONTROL_OWNER_NONE);
            return;
        }
        WingState_SetSession(&g_state, WING_CAN_SESSION_PREFLIGHT, WING_PHASE_STARTING,
                             WING_CONTROL_OWNER_REMOTE);
        g_session.state = SESSION_WAIT_START_ACK;
        g_session.phase_tick = now;
        if (!send_action(g_session.action, g_session.direction_request_id))
            fail_direction(5001U, WING_CONTROL_OWNER_NONE);
        else push_report(WING_REPORT_STATE_CHANGED);
        return;
    }

}

static void receive_status(const WingCanMessage* message, uint32_t now)
{
    if (message->kind != WING_FRAME_STATUS || g_session.expected_query != QUERY_RD) return;
    if (!message->accepted) return;
    g_session.expected_query = QUERY_NONE;
    WingState_ApplyFeedback(&g_state, now, message->position_mm, message->fault_code, message->moving);
    sync_ultrasonic_suppression();
    sync_anti_pinch_scope();

    if (g_session.state == SESSION_WAIT_POWER_STATUS)
    {
        WingLowerTravel lower = WingState_LowerTravel(g_state.snapshot.top_limit_mm, g_state.snapshot.bottom_limit_mm);
        uint16_t code = message->fault_code ? 5005U :
            (message->moving || message->position_mm < lower.minimum_mm ||
             message->position_mm > lower.endpoint_max_mm) ? 4003U : 200U;
        push_result(g_session.direction_request_id, WING_ACTION_NONE, code);
        finish_idle(WING_CONTROL_OWNER_NONE);
        return;
    }

    if (g_session.state == SESSION_WAIT_RD1 || g_session.state == SESSION_WAIT_RD2)
    {
        uint8_t second = g_session.state == SESSION_WAIT_RD2 ? 1U : 0U;
        if (message->fault_code) { fail_direction(5005U, WING_CONTROL_OWNER_NONE); return; }
        if (message->moving) { fail_direction(4003U, WING_CONTROL_OWNER_LOCAL_SUSPECTED); return; }
        if (!second)
        {
            g_session.state = SESSION_QUIET_GUARD;
            g_session.phase_tick = now;
            WingState_SetSession(&g_state, WING_CAN_SESSION_PREFLIGHT, WING_PHASE_QUIET_GUARD,
                                 WING_CONTROL_OWNER_NONE);
            push_report(WING_REPORT_STATE_CHANGED);
        }
        else
        {
            g_session.state = SESSION_WAIT_RL;
            g_session.phase_tick = now;
            WingState_SetSession(&g_state, WING_CAN_SESSION_PREFLIGHT, WING_PHASE_CHECKING_LIMITS,
                                 WING_CONTROL_OWNER_NONE);
            if (!send_query(QUERY_RL, now)) fail_direction(5001U, WING_CONTROL_OWNER_NONE);
            else push_report(WING_REPORT_STATE_CHANGED);
        }
        return;
    }

    if (g_session.state == SESSION_WAIT_START_STATUS)
    {
        if (message->fault_code)
        {
            if (!g_session.direction_result_sent)
            {
                push_result(g_session.direction_request_id, g_session.action, 5005U);
                g_session.direction_result_sent = 1U;
            }
            WingState_LatchSafety(&g_state, WING_SAFETY_CONTROLLER_FAULT);
            (void)begin_stop(STOP_SAFETY, "", now);
            return;
        }
        if (!message->moving)
        {
            (void)begin_stop(STOP_CONFLICT, "", now);
            return;
        }
        WingState_AcceptDirection(&g_state, g_session.action, g_session.direction_request_id, now);
        sync_ultrasonic_suppression();
        sync_anti_pinch_scope();
        g_session.state = SESSION_RUNNING;
        g_session.phase_tick = now;
        g_session.last_poll_tick = now;
        g_session.best_position_mm = message->position_mm;
        g_session.direction_result_sent = 1U;
        push_result(g_session.direction_request_id, g_session.action, 200U);
        push_report(WING_REPORT_STATE_CHANGED);
        return;
    }

    if (g_session.state == SESSION_RUNNING)
    {
        assess_running_status(now);
        return;
    }

    if (g_session.state == SESSION_WAIT_STOP_STATUS)
    {
        if (WingController_AutoResetActive()) AntiPinchBoard_UpdateResetLamp(now, message->moving);
        if (message->fault_code) { stop_failed(5005U); return; }
        if (!message->moving) finish_stop();
        return;
    }
}

/* The idle probe owns no motion session. Requests cancel it before starting a
 * normal session. Only the main loop can persist the fence and transmit RS. */
static uint8_t auto_query(AutoPhase phase, uint32_t now)
{
    uint8_t data[8];
    if (phase == AUTO_LIMITS) WingProtocol_EncodeLimitQuery(data);
    else WingProtocol_EncodeStatusQuery(data);
    if (!send_frame(data)) return 0U;
    auto_reset.phase = phase;
    auto_reset.tick = now;
    return 1U;
}

static void monitor_reset_status(const WingCanMessage* msg, uint32_t now)
{
    if (msg->kind != WING_FRAME_STATUS || g_session.expected_query != QUERY_RD) return;
    if (!msg->accepted || now - g_session.query_tick >= WING_QUERY_TIMEOUT_MS ||
        now - auto_reset.tick > WING_ACTIVE_FEEDBACK_TIMEOUT_MS ||
        now - auto_reset.started >= WING_MAX_RUNTIME_MS ||
        (!auto_reset.saw_motion && now - auto_reset.started >= WING_COMMAND_TIMEOUT_MS))
    { (void)begin_stop(STOP_SAFETY, "", now); return; }
    g_session.expected_query = QUERY_NONE;
    auto_reset.tick = now;
    WingState_ApplyFeedback(&g_state, now, msg->position_mm, msg->fault_code, msg->moving);
    AntiPinchBoard_UpdateResetLamp(now, msg->moving);
    if (msg->fault_code || !reset_path_clear(now))
    { (void)begin_stop(STOP_SAFETY, "", now); return; }
    if (msg->moving)
    {
        if (!auto_reset.saw_motion)
        {
            WingState_SetSession(&g_state, WING_CAN_SESSION_REMOTE_RUNNING, WING_PHASE_RUNNING, WING_CONTROL_OWNER_NONE);
            push_report(WING_REPORT_STATE_CHANGED);
        }
        auto_reset.saw_motion = 1U;
        auto_reset.settling = 0U;
    }
    else if (auto_reset.saw_motion)
    {
        if (!auto_reset.settling || msg->position_mm != auto_reset.position)
        {
            auto_reset.settling = 1U;
            auto_reset.stopped_since = now;
        }
        else if (now - auto_reset.stopped_since >= WING_PREFLIGHT_QUIET_MS && at_reset_endpoint(msg->position_mm))
            (void)begin_stop(STOP_ENDPOINT, "", now);
        else if (now - auto_reset.stopped_since >= WING_COMMAND_TIMEOUT_MS)
            (void)begin_stop(STOP_CONFLICT, "", now);
    }
    auto_reset.position = msg->position_mm;
}

static void auto_receive(const WingCanMessage* msg, uint32_t now)
{
    AutoPhase phase = auto_reset.phase;
    if (phase == AUTO_ACK)
    {
        if (msg->kind != WING_FRAME_RESET_ACK) return;
        if (now - auto_reset.started >= WING_COMMAND_TIMEOUT_MS)
        { (void)begin_stop(STOP_SAFETY, "", now); return; }
        if (!msg->accepted)
        {
            reset_alarm("drive rejected reset");
            end_probe(now);
            finish_idle(WING_CONTROL_OWNER_NONE);
            return;
        }
        auto_reset.phase = AUTO_TRACK;
        auto_reset.tick = now;
        if (!reset_path_clear(now) || !send_query(QUERY_RD, now))
            (void)begin_stop(STOP_SAFETY, "", now);
        return;
    }
    if (phase == AUTO_TRACK) { monitor_reset_status(msg, now); return; }
    if ((phase == AUTO_QUIET || phase == AUTO_DECIDE || phase == AUTO_SEND) &&
        msg->kind == WING_FRAME_STATUS && (!msg->accepted || msg->moving || msg->fault_code ||
                                           msg->position_mm != auto_reset.position))
    { end_probe(now); return; }
    if (phase == AUTO_LIMITS && msg->kind == WING_FRAME_LIMITS)
    {
        if (now - auto_reset.tick >= WING_QUERY_TIMEOUT_MS || msg->top_limit_mm <= msg->bottom_limit_mm)
        { end_probe(now); return; }
        auto_reset.top = msg->top_limit_mm;
        auto_reset.bottom = msg->bottom_limit_mm;
        auto_reset.lower = WingState_LowerTravel(auto_reset.top, auto_reset.bottom);
        if (!auto_query(AUTO_FIRST, now)) end_probe(now);
        return;
    }
    if (msg->kind != WING_FRAME_STATUS ||
        (phase != AUTO_FIRST && phase != AUTO_SECOND && phase != AUTO_FINAL_CHECK)) return;
    if (now - auto_reset.tick >= WING_QUERY_TIMEOUT_MS || !msg->accepted || msg->fault_code || msg->moving ||
        (phase != AUTO_FIRST && msg->position_mm != auto_reset.position))
    { end_probe(now); return; }
    auto_reset.position = msg->position_mm;
    auto_reset.tick = now;
    auto_reset.phase = phase == AUTO_FIRST ? AUTO_QUIET : (phase == AUTO_SECOND ? AUTO_DECIDE : AUTO_SEND);
}

static void auto_dispatch(uint32_t now)
{
    uint8_t frame[8];
    WingProtocol_EncodeReset(frame);
    auto_reset.reserved_unsent = 0U;
    WingState_ApplyLimits(&g_state, auto_reset.top, auto_reset.bottom);
    WingState_ApplyFeedback(&g_state, now, auto_reset.position, 0U, 0U);
    if (!send_frame(frame))
    {
        reset_alarm("reset transmission failed");
        end_probe(now);
        finish_idle(WING_CONTROL_OWNER_NONE);
        return;
    }
    g_session.state = SESSION_AUTO_RESET;
    auto_reset.phase = AUTO_ACK;
    auto_reset.started = now;
    auto_reset.saw_motion = auto_reset.settling = 0U;
    WingState_SetSession(&g_state, WING_CAN_SESSION_PREFLIGHT, WING_PHASE_STARTING, WING_CONTROL_OWNER_NONE);
    sync_anti_pinch_scope();
    sync_ultrasonic_suppression();
    AntiPinchBoard_UpdateResetLamp(now, 1U);
    APP_LOGW("AUTO_RS", "reset once pos=%u range=%u..%u", (unsigned)auto_reset.position,
             (unsigned)auto_reset_minimum_mm(), (unsigned)auto_reset.top);
}

/* Main loop, after incoming control commands have been serviced. Flash
 * transactions run with interrupts enabled. Recheck position afterwards. */
void WingController_AutoResetTask(uint8_t control_pending)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    uint32_t now = HAL_GetTick();
    if (g_session.state == SESSION_AUTO_RESET)
    {
        if (!reset_path_clear(now) || now - auto_reset.started >= WING_MAX_RUNTIME_MS ||
            (auto_reset.phase == AUTO_TRACK && now - auto_reset.tick > WING_ACTIVE_FEEDBACK_TIMEOUT_MS) ||
            (!auto_reset.saw_motion && now - auto_reset.started >= WING_COMMAND_TIMEOUT_MS) ||
            (g_session.expected_query == QUERY_RD && now - g_session.query_tick >= WING_QUERY_TIMEOUT_MS))
            (void)begin_stop(STOP_SAFETY, "", now);
        else if (auto_reset.phase == AUTO_TRACK && g_session.expected_query == QUERY_NONE &&
                 now - g_session.last_poll_tick >= WING_ACTIVE_STATUS_POLL_MS && !send_query(QUERY_RD, now))
            (void)begin_stop(STOP_SAFETY, "", now);
        goto done;
    }
    if (auto_reset.phase >= AUTO_ACK) goto done; /* ST confirmation owns CAN. */
    if (!g_can_ready || control_pending || g_session.state != SESSION_IDLE || g_pinch.phase != PINCH_NONE ||
        g_pinch_pressed || !WingPower_IsOn())
    { end_probe(now); goto done; }
    switch (auto_reset.phase)
    {
        case AUTO_IDLE:
            if (now - auto_reset.tick >= WING_IDLE_LIMIT_POLL_MS && !auto_query(AUTO_LIMITS, now)) end_probe(now);
            break;
        case AUTO_QUIET:
            if (now - auto_reset.tick >= WING_COMMAND_TIMEOUT_MS) end_probe(now);
            else if (now - auto_reset.tick >= WING_PREFLIGHT_QUIET_MS && !auto_query(AUTO_SECOND, now)) end_probe(now);
            break;
        case AUTO_DECIDE:
        {
            uint8_t outside = auto_reset.position < auto_reset_minimum_mm() || auto_reset.position > auto_reset.top;
            if (now - auto_reset.tick >= WING_QUERY_TIMEOUT_MS || (outside && !reset_path_clear(now)))
            { end_probe(now); break; }
            __set_PRIMASK(irq);
            int latched = OTA_Info_AutoResetLatched();
            int claim = outside && latched == 0;
            uint8_t ok = latched >= 0;
            if (claim || (!outside && latched > 0)) ok = OTA_Info_SetAutoResetLatched(outside);
            __disable_irq();
            now = HAL_GetTick();
            if (claim && ok) auto_reset.reserved_unsent = 1U;
            if (!outside && ok) auto_reset.reserved_unsent = 0U;
            if (auto_reset.phase != AUTO_DECIDE) break;
            if (!ok || (outside && latched > 0 && !auto_reset.reserved_unsent))
            {
                if (!auto_reset.warned)
                {
                    reset_alarm(!ok ? "one-shot storage unavailable" : "still outside; reset inhibited");
                    auto_reset.warned = 1U;
                }
            }
            else if (!outside) { auto_reset.warned = 0U; auto_reset.alarm = NULL; }
            if (outside && ok && auto_reset.reserved_unsent && reset_path_clear(now) && auto_query(AUTO_FINAL_CHECK, now)) break;
            end_probe(now);
            break;
        }
        case AUTO_SEND:
            if (now - auto_reset.tick < WING_QUERY_TIMEOUT_MS && reset_path_clear(now)) auto_dispatch(now);
            else end_probe(now);
            break;
        default:
            if (now - auto_reset.tick >= WING_QUERY_TIMEOUT_MS) end_probe(now);
            break;
    }
done:
    __set_PRIMASK(irq);
}

void WingController_OnCanRx(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef header = {0};
    uint8_t data[8];
    WingCanMessage msg;
    uint32_t now;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) return;
    if (hcan != selected_can()) return; /* Drain even the unused bus FIFO. */
    if (header.IDE != CAN_ID_EXT || header.RTR != CAN_RTR_DATA ||
        !WingProtocol_Decode(header.ExtId, (uint8_t)header.DLC, data, &msg)) return;
    now = HAL_GetTick();
    if ((g_session.state == SESSION_IDLE && auto_reset.phase != AUTO_IDLE) ||
        g_session.state == SESSION_AUTO_RESET)
    {
        auto_receive(&msg, now);
        return;
    }
    if (!g_session.power_off_check && enforce_ultrasonic_stop(now)) return;
    if (g_session.state != SESSION_WAIT_STOP_ACK && g_session.state != SESSION_WAIT_STOP_STATUS &&
        pinch_receive(&msg, now)) return;

    if (auto_reset.phase >= AUTO_ACK && g_session.stop_command_sent &&
        now - g_session.stop_tick >= WING_COMMAND_TIMEOUT_MS)
    { stop_failed(5003U); return; }
    if (msg.kind == WING_FRAME_ACK) receive_ack(&msg, now);
    else if (msg.kind == WING_FRAME_LIMITS) receive_limits(&msg, now);
    else receive_status(&msg, now);
}

static void controller_task(uint32_t now)
{
    WingState_Tick(&g_state, now);
    sync_ultrasonic_suppression();
    sync_anti_pinch_scope();
    if (!g_session.power_off_check && enforce_ultrasonic_stop(now)) return;
    if (g_session.state != SESSION_WAIT_STOP_ACK && g_session.state != SESSION_WAIT_STOP_STATUS &&
        pinch_task(now)) return;

    if (g_session.state == SESSION_IDLE) return;

    if (g_session.state == SESSION_QUIET_GUARD &&
        (now - g_session.phase_tick) >= WING_PREFLIGHT_QUIET_MS)
    {
        g_session.state = SESSION_WAIT_RD2;
        g_session.phase_tick = now;
        WingState_SetSession(&g_state, WING_CAN_SESSION_PREFLIGHT, WING_PHASE_CHECKING_MOTION,
                             WING_CONTROL_OWNER_NONE);
        if (!send_query(QUERY_RD, now)) fail_direction(5001U, WING_CONTROL_OWNER_NONE);
        else push_report(WING_REPORT_STATE_CHANGED);
        return;
    }

    if ((g_session.state == SESSION_WAIT_RD1 || g_session.state == SESSION_WAIT_RD2 ||
         g_session.state == SESSION_WAIT_RL || g_session.state == SESSION_WAIT_POWER_STATUS) && g_session.expected_query != QUERY_NONE &&
        (now - g_session.query_tick) >= WING_QUERY_TIMEOUT_MS)
    {
        fail_direction(g_session.expected_query == QUERY_RD ? 5002U : 5003U,
                       WING_CONTROL_OWNER_NONE);
        return;
    }

    if ((g_session.state == SESSION_WAIT_START_ACK || g_session.state == SESSION_WAIT_START_STATUS) &&
        (now - g_session.phase_tick) >= WING_COMMAND_TIMEOUT_MS)
    {
        (void)begin_stop(STOP_SAFETY, "", now);
        return;
    }

    if (g_session.state == SESSION_RUNNING)
    {
        if (WingState_NeedsSafetyStop(&g_state))
        {
            (void)begin_stop(STOP_SAFETY, "", now);
            return;
        }
        if (g_session.expected_query == QUERY_NONE &&
            (now - g_session.last_poll_tick) >= WING_ACTIVE_STATUS_POLL_MS)
            (void)send_query(QUERY_RD, now);
        return;
    }

    if (g_session.state == SESSION_WAIT_STOP_STATUS &&
        g_session.expected_query == QUERY_NONE &&
        (now - g_session.last_poll_tick) >= WING_ACTIVE_STATUS_POLL_MS)
        (void)send_query(QUERY_RD, now);

    if ((g_session.state == SESSION_WAIT_STOP_ACK || g_session.state == SESSION_WAIT_STOP_STATUS) &&
        (now - g_session.stop_tick) >= WING_COMMAND_TIMEOUT_MS)
    {
        stop_failed(5003U);
    }
}

void WingController_Task(uint32_t now)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    controller_task(now);
    __set_PRIMASK(irq);
}

const WingSnapshot* WingController_GetSnapshot(void) { return &g_state.snapshot; }
void WingController_CopySnapshot(WingSnapshot* snapshot)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (snapshot) *snapshot = g_state.snapshot;
    __set_PRIMASK(irq);
}

uint8_t WingController_TakeResult(WingControlResult* result)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (!result || !g_result_count) { __set_PRIMASK(irq); return 0U; }
    *result = g_results[g_result_head];
    g_result_head = (uint8_t)((g_result_head + 1U) % RESULT_QUEUE_DEPTH);
    g_result_count--;
    __set_PRIMASK(irq);
    return 1U;
}

WingReportReason WingController_TakeReportReason(void)
{
    WingReportReason reason;
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (!g_report_count) { __set_PRIMASK(irq); return WING_REPORT_NONE; }
    reason = g_reports[g_report_head].reason;
    g_report_head = (uint8_t)((g_report_head + 1U) % REPORT_QUEUE_DEPTH);
    g_report_count--;
    __set_PRIMASK(irq);
    return reason;
}

uint8_t WingController_TakeReport(WingReport* report)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (!report || !g_report_count) { __set_PRIMASK(irq); return 0U; }
    *report = g_reports[g_report_head];
    g_report_head = (uint8_t)((g_report_head + 1U) % REPORT_QUEUE_DEPTH);
    g_report_count--;
    __set_PRIMASK(irq);
    return 1U;
}

const char* WingReportReason_Name(WingReportReason reason)
{
    if (reason == WING_REPORT_PERIODIC) return "periodic";
    if (reason == WING_REPORT_STATE_CHANGED) return "state_changed";
    if (reason == WING_REPORT_CONTROLLER_OFFLINE) return "controller_offline";
    if (reason == WING_REPORT_CONTROLLER_ONLINE) return "controller_online";
    if (reason == WING_REPORT_FAULT_RAISED) return "fault_raised";
    if (reason == WING_REPORT_FAULT_CLEARED) return "fault_cleared";
    if (reason == WING_REPORT_SAFETY_STOP) return "safety_stop";
    if (reason == WING_REPORT_ENDPOINT_STOP) return "endpoint_stop";
    if (reason == WING_REPORT_CONTROL_CONFLICT) return "control_conflict";
    return "unknown";
}
