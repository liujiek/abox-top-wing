#include <assert.h>
#include <string.h>
#include "main.h"
#include "ultrasonic_board.h"
#include "ultrasonic_alarm.h"
CAN_HandleTypeDef hcan1={1},hcan2={2};
static uint32_t tick;
static unsigned count, consumed;
static CAN_RxHeaderTypeDef headers[6];
static uint8_t payloads[6][8];
uint32_t HAL_GetTick(void) {return tick;}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef* h, uint32_t fifo)
{assert(h==&hcan1 && fifo==CAN_RX_FIFO0);return count-consumed;}
int HAL_CAN_GetRxMessage(CAN_HandleTypeDef* h,uint32_t fifo,CAN_RxHeaderTypeDef* head,uint8_t* d)
{assert(h==&hcan1 && fifo==CAN_RX_FIFO0);*head=headers[consumed];memcpy(d,payloads[consumed++],8);return HAL_OK;}
int main(void)
{
    assert(!UltrasonicBoard_IsAlarm(0));
    UltrasonicBoard_Init();
    assert(!UltrasonicBoard_PathConfirmedClear(0));
    count=6;
    for(unsigned i=0;i<count;++i)
    {headers[i]=(CAN_RxHeaderTypeDef){.ExtId=WL300D_CAN_ID,.IDE=CAN_ID_EXT,.DLC=8};memset(payloads[i],0xFF,8);}
    headers[0].ExtId=1; headers[1].IDE=0; headers[2].RTR=CAN_RTR_REMOTE;headers[3].DLC=7;
    for(unsigned i=0;i<4;++i)payloads[i][0]=20;
    memset(payloads[4],0xFF,8);
    UltrasonicBoard_OnCanRx(&hcan2);assert(!consumed); /* Preserve CAN2. */
    UltrasonicBoard_OnCanRx(&hcan1);assert(consumed==count && !UltrasonicBoard_IsAlarm(0));
    assert(UltrasonicBoard_PathConfirmedClear(0));
    assert(!UltrasonicBoard_PathConfirmedClear(ULTRASONIC_FRESH_MS + 1U));
    UltrasonicDiagnostics diag;
    UltrasonicBoard_CopyDiagnostics(&diag);
    assert(diag.received==6 && diag.accepted==2 && diag.seen && !diag.active);
    assert(diag.last_id==WL300D_CAN_ID && diag.last_dlc==8 && diag.last_extended);
    assert(!diag.last_remote && diag.last_valid_data[7]==0xFF);
    consumed=0;count=1;headers[0]=headers[4];memset(payloads[0],0xFF,8);payloads[0][7]=30;tick=50;
    UltrasonicBoard_OnCanRx(&hcan1);
    assert(!UltrasonicBoard_PathConfirmedClear(tick) && !UltrasonicBoard_IsAlarm(tick));
    consumed=0;tick=100;
    UltrasonicBoard_OnCanRx(&hcan1);assert(UltrasonicBoard_IsAlarm(101));
    UltrasonicBoard_CopyDiagnostics(&diag);
    assert(!UltrasonicBoard_PathConfirmedClear(101));
    assert(diag.received==8 && diag.accepted==4 && diag.mask==128 && diag.active);
    assert(diag.last_valid_tick==100 && diag.last_valid_data[7]==30);
    consumed=0;headers[0].IDE=0;headers[0].StdId=0x321;tick=200;
    UltrasonicBoard_OnCanRx(&hcan1);
    UltrasonicBoard_CopyDiagnostics(&diag);
    assert(diag.received==9 && diag.accepted==4 && diag.last_id==0x321 && !diag.last_extended);
    assert(diag.last_valid_tick==100 && diag.last_valid_data[7]==30);
    assert(UltrasonicBoard_IsAlarm(2099));
    assert(!UltrasonicBoard_IsAlarm(2100)); /* Silence clears after exactly two seconds. */
    assert(UltrasonicBoard_MotionBlocked(2100)); /* A short alarm cannot lose its stop event. */
    assert(UltrasonicBoard_TakeTrigger());
    assert(!UltrasonicBoard_TakeTrigger() && !UltrasonicBoard_MotionBlocked(2100));

    UltrasonicBoard_Init();
    consumed=0;count=1;headers[0]=(CAN_RxHeaderTypeDef){
        .ExtId=WL300D_CAN_ID,.IDE=CAN_ID_EXT,.DLC=8};
    memset(payloads[0],0xFF,8);payloads[0][0]=20;tick=1950;
    UltrasonicBoard_OnCanRx(&hcan1);
    consumed=0;tick=2000;
    UltrasonicBoard_OnCanRx(&hcan1);
    assert(UltrasonicBoard_MotionBlocked(2000));
    UltrasonicBoard_ClearAlarmHold();
    UltrasonicBoard_SetAlarmSuppressed(1);
    UltrasonicBoard_CopyDiagnostics(&diag);
    assert(diag.active && diag.suppressed);
    assert(!UltrasonicBoard_IsAlarm(2000));
    assert(!UltrasonicBoard_MotionBlocked(2000));
    assert(UltrasonicBoard_ObstacleActive(2000));
    assert(!UltrasonicBoard_TakeTrigger());
    UltrasonicBoard_SetAlarmSuppressed(0);
    assert(UltrasonicBoard_IsAlarm(2000));
    assert(UltrasonicBoard_MotionBlocked(2000));
    consumed=0;memset(payloads[0],0x00,8);tick=2100;
    UltrasonicBoard_OnCanRx(&hcan1);
    assert(UltrasonicBoard_IsAlarm(2100));
    assert(UltrasonicBoard_IsAlarm(4099));
    assert(!UltrasonicBoard_IsAlarm(4100));
    assert(!UltrasonicBoard_MotionBlocked(4100));

    UltrasonicBoard_Init();
    consumed=0;count=1;headers[0]=(CAN_RxHeaderTypeDef){
        .ExtId=WL300D_CAN_ID,.IDE=CAN_ID_EXT,.DLC=8};
    memset(payloads[0],0xFF,8);payloads[0][0]=20;tick=2950;
    UltrasonicBoard_SetAlarmSuppressed(1);
    UltrasonicBoard_OnCanRx(&hcan1);
    UltrasonicBoard_StartPrestartWarning(tick);
    assert(!UltrasonicBoard_ObstacleActive(tick) && !UltrasonicBoard_IsAlarm(tick));
    consumed=0;tick=3000;
    UltrasonicBoard_OnCanRx(&hcan1);
    UltrasonicBoard_StartPrestartWarning(3000);
    UltrasonicBoard_CopyDiagnostics(&diag);
    assert(diag.prestart_warning);
    UltrasonicBoard_ClearAlarmHold(); /* Stop during the initial two-second prompt. */
    consumed=0;tick=3001;
    UltrasonicBoard_OnCanRx(&hcan1);
    assert(!UltrasonicBoard_IsAlarm(tick) && UltrasonicBoard_ObstacleActive(tick));
    UltrasonicBoard_StartPrestartWarning(tick); /* A fresh refused start warns again. */
    UltrasonicBoard_StartPrestartWarning(3999); /* Repeated request must not extend. */
    assert(UltrasonicBoard_IsAlarm(3999));
    for(uint32_t t=3100;t<=5200;t+=100)
    {
        consumed=0;tick=t;
        UltrasonicBoard_OnCanRx(&hcan1);
        assert(UltrasonicBoard_IsAlarm(t));
    }
    consumed=0;memset(payloads[0],0x00,8);tick=5300;
    UltrasonicBoard_OnCanRx(&hcan1);
    for(uint32_t t=5400;t<=7299;t+=100)
    {
        consumed=0;tick=t;
        UltrasonicBoard_OnCanRx(&hcan1);
        assert(UltrasonicBoard_IsAlarm(t));
    }
    assert(UltrasonicBoard_IsAlarm(7299));
    assert(!UltrasonicBoard_IsAlarm(7300));
    assert(!UltrasonicBoard_MotionBlocked(7300));
    assert(!UltrasonicBoard_TakeTrigger());

    UltrasonicBoard_Init();
    consumed=0;count=1;tick=5950;
    memset(payloads[0],0xFF,8);payloads[0][0]=20;
    UltrasonicBoard_OnCanRx(&hcan1);
    consumed=0;tick=6000;
    UltrasonicBoard_OnCanRx(&hcan1);
    assert(UltrasonicBoard_IsAlarm(6000));
    UltrasonicBoard_SetAlarmSuppressed(1); /* Idle transition preserves remote alarm. */
    assert(UltrasonicBoard_IsAlarm(6000));
    for(uint32_t t=6100;t<=10000;t+=100)
    {
        consumed=0;tick=t;
        UltrasonicBoard_OnCanRx(&hcan1);
        assert(UltrasonicBoard_IsAlarm(t));
    }
    consumed=0;memset(payloads[0],0x00,8);tick=10100;
    UltrasonicBoard_OnCanRx(&hcan1);
    assert(UltrasonicBoard_IsAlarm(12099));
    assert(!UltrasonicBoard_IsAlarm(12100));

    UltrasonicBoard_Init();
    consumed=0; tick=13000;
    memset(payloads[0],151,8);
    UltrasonicBoard_OnCanRx(&hcan1);
    UltrasonicBoard_StartPrestartWarning(tick);
    assert(!UltrasonicBoard_IsAlarm(tick) && !UltrasonicBoard_ObstacleActive(tick));
    assert(!UltrasonicBoard_MotionBlocked(tick) && !UltrasonicBoard_TakeTrigger());
    assert(!UltrasonicBoard_PathConfirmedClear(tick)); /* Full-range gate for autonomous reset. */
    UltrasonicBoard_CopyDiagnostics(&diag);
    assert(diag.mask == 0 && diag.last_valid_data[0] == 151);
    consumed=0; tick++; payloads[0][7]=150;
    UltrasonicBoard_OnCanRx(&hcan1);
    assert(!UltrasonicBoard_MotionBlocked(tick) && !UltrasonicBoard_TakeTrigger());
    consumed=0; tick+=50;
    UltrasonicBoard_OnCanRx(&hcan1);
    assert(UltrasonicBoard_IsAlarm(tick) && UltrasonicBoard_ObstacleActive(tick));
    assert(UltrasonicBoard_MotionBlocked(tick) && UltrasonicBoard_TakeTrigger());
    return 0;
}
