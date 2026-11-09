/* CAN2 work protocol. Legacy counter names are retained for UART diagnostics. */
#include "can_test.h"
#include "can.h"
#include "work_can.h"

volatile uint32_t CAN_Test_TxCount, CAN_Test_RxCount, CAN_Test_ErrorCount;
volatile uint32_t CAN_Test_LastHalError;
volatile uint8_t CAN_Test_State;
static WorkCanPacket request, reply;
static uint8_t ready, have_reply, dirty, tx_pending;
static uint32_t last_tx, last_rx, mailbox, pending_since;

static void record_error(uint8_t state)
{
    CAN_Test_State = state;
    CAN_Test_ErrorCount++;
    CAN_Test_LastHalError = HAL_CAN_GetError(&hcan2);
}

void CAN_Test_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    ready = have_reply = tx_pending = 0;
    dirty = 1;
    last_tx = last_rx = 0;
    CAN_Test_TxCount = CAN_Test_RxCount = CAN_Test_ErrorCount = 0;
    CAN_Test_LastHalError = 0;
    CAN_Test_State = 0;
    request.state = WORK_HELLO;
    request.task = 0;
    request.region = request.result = 0;
    filter.FilterBank = 14;
    filter.SlaveStartFilterBank = 14;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.FilterIdHigh = WORK_CAN_REPLY_ID << 5;
    filter.FilterMaskIdHigh = 0x7FFu << 5;
    filter.FilterMaskIdLow = 6u; /* Match standard data frames only. */
    if (HAL_CAN_ConfigFilter(&hcan2, &filter) != HAL_OK ||
        HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        record_error(2);
        return;
    }
    ready = 1;
}

void Work_CAN_SetRequest(uint8_t state, uint32_t task, uint8_t region, uint8_t result)
{
    if (request.state == state && request.task == task &&
        request.region == region && request.result == result) return;
    request.state = state;
    request.task = task;
    request.region = region;
    request.result = result;
    dirty = 1;
    /* Remove an obsolete queued state before publishing its successor. */
    if (tx_pending)
    {
        (void)HAL_CAN_AbortTxRequest(&hcan2, mailbox);
        tx_pending = 0;
    }
}

uint8_t Work_CAN_Online(void)
{
    return ready && have_reply &&
        (uint32_t)(HAL_GetTick() - last_rx) < WORK_CAN_TIMEOUT_MS;
}

uint8_t Work_CAN_GetReply(uint32_t task)
{
    if (!Work_CAN_Online()) return 0;
    if (reply.state == WORK_FAULT) return WORK_FAULT;
    return reply.task == task ? reply.state : 0;
}

uint8_t Work_CAN_Result(void) { return reply.result; }
uint8_t Work_CAN_Moving(void)
{
    return Work_CAN_Online() && (reply.region & WORK_CAN_MOVING_FLAG) != 0;
}

void CAN_Test_Task(void)
{
    CAN_RxHeaderTypeDef rx;
    CAN_TxHeaderTypeDef tx = {0};
    WorkCanPacket packet;
    uint8_t data[8], count = 0;
    uint32_t now = HAL_GetTick();
    if (!ready) return;
    while (count++ < 8 && HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0))
    {
        if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx, data) != HAL_OK)
        {
            record_error(4);
            break;
        }
        if (rx.IDE != CAN_ID_STD || rx.StdId != WORK_CAN_REPLY_ID ||
            rx.RTR != CAN_RTR_DATA || rx.DLC != 8 || !work_can_decode(data, &packet))
            continue;
        if (packet.task != request.task && packet.state != WORK_FAULT) continue;
        if (packet.state != WORK_SYNC && packet.state != WORK_READY &&
            packet.state != WORK_SLOW_ACK && packet.state != WORK_STOPPING &&
            packet.state != WORK_STOPPED && packet.state != WORK_GO_ACK &&
            packet.state != WORK_FAULT) continue;
        reply = packet;
        have_reply = 1;
        last_rx = now;
        CAN_Test_RxCount++;
        CAN_Test_State = 1;
    }
    if (have_reply && !Work_CAN_Online()) CAN_Test_State = 5;
    if (tx_pending)
    {
        if (!HAL_CAN_IsTxMessagePending(&hcan2, mailbox)) tx_pending = 0;
        else if ((uint32_t)(now - pending_since) >= WORK_CAN_PERIOD_MS)
        {
            (void)HAL_CAN_AbortTxRequest(&hcan2, mailbox);
            tx_pending = 0;
            record_error(3);
        }
    }
    if (tx_pending || (!dirty && (uint32_t)(now - last_tx) < WORK_CAN_PERIOD_MS)) return;
    if (!HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)) return;
    work_can_encode(data, request);
    tx.StdId = WORK_CAN_REQUEST_ID;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 8;
    tx.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(&hcan2, &tx, data, &mailbox) != HAL_OK)
    {
        record_error(3);
        return;
    }
    CAN_Test_TxCount++;
    last_tx = pending_since = now;
    tx_pending = 1;
    dirty = 0;
}
