#include "work_control.h"
#include "work_can.h"
#include "vision_motor.h"
#include <string.h>

volatile uint8_t Vision_WorkState, Vision_WorkResult;
volatile uint32_t Vision_WorkTask;
static uint8_t syncing, paired, attempts[5], stable_region, stable_frames;
static uint8_t empty_frames, departed_frames, moving_timing, missing_timing;
static uint8_t empty_timing, saw_movement;
static uint32_t state_since, tree_since, stable_since, empty_since, frame_floor;
static uint32_t check_since, check_delay, moving_since, missing_since, last_frame;

static void change_state(uint8_t state, uint32_t now)
{
    Vision_WorkState = state;
    state_since = now;
    empty_frames = empty_timing = departed_frames = 0;
}

static void fail(uint8_t reason, uint32_t now)
{
    Vision_WorkResult = reason;
    change_state(VISION_WORK_FAULT, now);
}

static void publish(void)
{
    uint8_t state;
    switch (Vision_WorkState)
    {
    case VISION_WORK_BOOT: state = syncing && Vision_Online ? WORK_READY : WORK_HELLO; break;
    case VISION_WORK_IDLE: state = WORK_READY; break;
    case VISION_WORK_APPROACH: state = WORK_SLOW; break;
    case VISION_WORK_WAIT_STOP: state = WORK_WAIT; break;
    case VISION_WORK_CHECK:
    case VISION_WORK_ACTING: state = WORK_WORKING; break;
    case VISION_WORK_RELEASE:
    case VISION_WORK_LEAVING: state = WORK_GO; break;
    default: state = WORK_FAULT; break;
    }
    Work_CAN_SetRequest(state, Vision_WorkTask, Vision_Region, Vision_WorkResult);
}

void Work_Control_Init(void)
{
    Vision_WorkState = VISION_WORK_BOOT;
    Vision_WorkResult = WORK_RESULT_OK;
    Vision_WorkTask = 0;
    syncing = paired = stable_region = stable_frames = 0;
    empty_frames = empty_timing = departed_frames = 0;
    moving_timing = missing_timing = saw_movement = 0;
    state_since = tree_since = stable_since = empty_since = frame_floor = 0;
    check_since = check_delay = moving_since = missing_since = last_frame = 0;
    memset(attempts, 0, sizeof(attempts));
}

static void begin_check(uint32_t now, uint32_t frame_count, uint8_t returned)
{
    change_state(VISION_WORK_CHECK, now);
    frame_floor = frame_count;
    check_since = now;
    check_delay = WORK_FRESH_VISION_MS;
    if (returned)
    {
        check_since = Vision_Motor_ReturnedAt();
        if (WORK_SETTLE_MS > check_delay) check_delay = WORK_SETTLE_MS;
    }
    stable_region = stable_frames = 0;
    missing_timing = 0;
}

static void update_target(uint32_t now, uint8_t new_frame, uint8_t tree_valid, int32_t offset)
{
    if (!tree_valid || offset < -WORK_SLOW_DX || offset > WORK_SLOW_DX || !Vision_Region)
    {
        stable_region = stable_frames = 0;
        return;
    }
    if (!new_frame) return;
    if (stable_region != Vision_Region)
    {
        stable_region = Vision_Region;
        stable_since = now;
        stable_frames = 1;
    }
    else if (stable_frames < 255) stable_frames++;
}

static void process_check(uint32_t now, uint32_t frame_count,
                          uint8_t new_frame, uint8_t tree_valid, int32_t offset)
{
    if (!tree_valid)
    {
        empty_timing = empty_frames = 0;
        if (!missing_timing) { missing_timing = 1; missing_since = now; }
        if ((uint32_t)(now - missing_since) >= WORK_VISION_WAIT_MS)
            fail(WORK_RESULT_VISION, now);
        return;
    }
    missing_timing = 0;
    if (offset < -WORK_STOP_DX || offset > WORK_STOP_DX)
    {
        fail(WORK_RESULT_POSITION, now);
        return;
    }
    if ((uint32_t)(now - tree_since) >= WORK_TREE_TIMEOUT_MS)
    {
        Vision_WorkResult = WORK_RESULT_TREE_TIMEOUT;
        change_state(VISION_WORK_RELEASE, now);
        return;
    }
    if ((uint32_t)(now - check_since) < check_delay)
    {
        frame_floor = frame_count;
        return;
    }
    if (!new_frame || frame_count == frame_floor) return;
    if (Vision_Region)
    {
        empty_timing = empty_frames = 0;
        if (stable_frames < 3 || (uint32_t)(now - stable_since) < WORK_TARGET_STABLE_MS) return;
        if (attempts[Vision_Region] >= WORK_MAX_ATTEMPTS_PER_REGION)
        {
            Vision_WorkResult = WORK_RESULT_ATTEMPTS;
            change_state(VISION_WORK_RELEASE, now);
            return;
        }
        if (Vision_Motor_Start(Vision_Region))
        {
            attempts[Vision_Region]++;
            change_state(VISION_WORK_ACTING, now);
        }
        return;
    }
    if (!empty_timing) { empty_timing = 1; empty_since = now; }
    if (empty_frames < 255) empty_frames++;
    if (empty_frames >= WORK_NO_TARGET_FRAMES &&
        (uint32_t)(now - empty_since) >= WORK_NO_TARGET_MS)
        change_state(VISION_WORK_RELEASE, now);
}

static void process_departure(uint32_t now, uint8_t new_frame, uint8_t tree_valid, int32_t offset)
{
    if (!Vision_Online) { fail(WORK_RESULT_VISION, now); return; }
    if (Work_CAN_Moving())
    {
        if (!moving_timing) { moving_timing = 1; moving_since = now; }
        if ((uint32_t)(now - moving_since) >= WORK_LEAVE_MIN_MOVING_MS) saw_movement = 1;
    }
    else moving_timing = 0;
    if (new_frame && tree_valid)
    {
        missing_timing = 0;
        if (offset * WORK_LEAVE_DIRECTION >= WORK_LEAVE_DX)
        {
            if (departed_frames < 255) departed_frames++;
        }
        else departed_frames = 0;
        if (departed_frames >= 3 && saw_movement) change_state(VISION_WORK_IDLE, now);
    }
    else if (new_frame && !tree_valid)
    {
        departed_frames = 0;
        if (!missing_timing) { missing_timing = 1; missing_since = now; }
        if (saw_movement && (uint32_t)(now - missing_since) >= WORK_LEAVE_MISSING_MS)
            change_state(VISION_WORK_IDLE, now);
    }
}

void Work_Control_Task(uint32_t now, uint32_t frame_count)
{
    int32_t offset = Vision_TreeDX - WORK_DX_CENTER;
    uint8_t tree_valid = Vision_Online && Vision_TreeDX != 9999;
    uint8_t new_frame = frame_count != last_frame;
    uint8_t reply = Work_CAN_GetReply(Vision_WorkTask);
    last_frame = frame_count;
    update_target(now, new_frame, tree_valid, offset);
    if (paired && Vision_WorkState != VISION_WORK_FAULT)
    {
        if (!Work_CAN_Online()) fail(WORK_RESULT_LINK, now);
        else if (reply == WORK_FAULT) fail(Work_CAN_Result(), now);
        else if ((Vision_WorkState == VISION_WORK_CHECK || Vision_WorkState == VISION_WORK_ACTING) &&
                 reply != WORK_STOPPED) fail(WORK_RESULT_PROTOCOL, now);
    }
    switch (Vision_WorkState)
    {
    case VISION_WORK_BOOT:
        if (reply == WORK_SYNC) syncing = 1;
        else if (reply == WORK_READY && syncing)
        {
            paired = 1;
            change_state(VISION_WORK_IDLE, now);
        }
        break;
    case VISION_WORK_IDLE:
        if (!Vision_Online)
        {
            fail(WORK_RESULT_VISION, now);
            break;
        }
        if (new_frame && tree_valid && offset >= -WORK_SLOW_DX && offset <= WORK_SLOW_DX)
        {
            if (Vision_WorkTask == UINT32_MAX) { fail(WORK_RESULT_PROTOCOL, now); break; }
            Vision_WorkTask++;
            Vision_WorkResult = WORK_RESULT_OK;
            memset(attempts, 0, sizeof(attempts));
            change_state(VISION_WORK_APPROACH, now);
        }
        break;
    case VISION_WORK_APPROACH:
        if (!tree_valid)
        {
            tree_since = now;
            change_state(VISION_WORK_WAIT_STOP, now);
            break;
        }
        if (reply != WORK_SLOW_ACK) break;
        if (Vision_Region && offset * WORK_LEAVE_DIRECTION > WORK_STOP_DX)
            fail(WORK_RESULT_POSITION, now);
        else if (offset >= -WORK_STOP_DX && offset <= WORK_STOP_DX &&
                 stable_frames >= 3 && (uint32_t)(now - stable_since) >= WORK_TARGET_STABLE_MS)
        {
            tree_since = now;
            change_state(VISION_WORK_WAIT_STOP, now);
        }
        else if (new_frame)
        {
            if (!Vision_Region && offset * WORK_LEAVE_DIRECTION >= WORK_LEAVE_DX)
            {
                if (++departed_frames >= 3)
                {
                    Vision_WorkResult = WORK_RESULT_SKIPPED;
                    change_state(VISION_WORK_RELEASE, now);
                }
            }
            else departed_frames = 0;
        }
        break;
    case VISION_WORK_WAIT_STOP:
        if (reply == WORK_STOPPED) begin_check(now, frame_count, 0);
        else if ((uint32_t)(now - state_since) >= WORK_STOP_TIMEOUT_MS)
            fail(WORK_RESULT_STOP_TIMEOUT, now);
        break;
    case VISION_WORK_CHECK:
        process_check(now, frame_count, new_frame, tree_valid, offset);
        break;
    case VISION_WORK_ACTING:
        if (!Vision_MotorPhase) begin_check(now, frame_count, 1);
        break;
    case VISION_WORK_RELEASE:
        if (reply == WORK_GO_ACK)
        {
            moving_timing = missing_timing = saw_movement = 0;
            change_state(VISION_WORK_LEAVING, now);
        }
        break;
    case VISION_WORK_LEAVING:
        process_departure(now, new_frame, tree_valid, offset);
        break;
    default:
        /* Faults latch. An active action still completes its timed return. */
        break;
    }
    publish();
}
