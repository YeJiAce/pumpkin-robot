/** V1 session, command application results and timestamped feedback scheduling. */
#include "ros_link.h"
#include "motor_app.h"
#include "imu_app.h"
#include "serial_dma.h"
#include "cpu_load.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

#define REPLY_CAPACITY 8U

typedef struct { WireFrame frame; bool pending; } FeedbackSlot;
static WireFrame replies[REPLY_CAPACITY];
static uint8_t reply_head, reply_count;
static FeedbackSlot feedback[3];
static uint8_t feedback_next;
static uint16_t tx_sequence, rx_sequence;
static uint32_t session, ticket_counter;
static bool established;
static bool control_pending, control_hello, control_silent;
static uint32_t control_ticket;
static uint16_t control_sequence;
static uint8_t control_type;
static uint32_t imu_at, wheels_at, status_at;
static uint32_t mode_generation;

static void SyncMode(const MotorAppStatus *motor)
{
    if (motor->mode_generation == mode_generation)
    {
        return;
    }
    mode_generation = motor->mode_generation;
    session = 0U;
    established = false;
    control_pending = false;
    reply_count = 0U;
    rx_sequence = 0U;
    memset(feedback, 0, sizeof(feedback));
}

static uint16_t SaturateU16(uint32_t value)
{
    return value > UINT16_MAX ? UINT16_MAX : (uint16_t)value;
}

static WireFrame MakeFrame(uint8_t type, uint16_t length, uint32_t stamp)
{
    WireFrame frame = {.type = type, .length = length, .time_ms = stamp};
    Wire_WriteU32(frame.payload, session);
    return frame;
}

static void Reply(const WireFrame *frame)
{
    /* Under overload keep the newest replies; motion safety does not depend on ACK delivery. */
    if (reply_count == REPLY_CAPACITY)
    {
        reply_head = (reply_head + 1U) % REPLY_CAPACITY;
        --reply_count;
    }
    replies[(reply_head + reply_count) % REPLY_CAPACITY] = *frame;
    ++reply_count;
}

static void Ack(uint16_t sequence, uint8_t type, uint8_t result)
{
    WireFrame frame = MakeFrame(WIRE_ACK, 8U, HAL_GetTick());
    Wire_WriteU16(frame.payload + 4, sequence);
    frame.payload[6] = type;
    frame.payload[7] = result;
    Reply(&frame);
}

static void HelloReply(void)
{
    WireFrame frame = MakeFrame(WIRE_HELLO_REPLY, 12U, HAL_GetTick());
    Wire_WriteU32(frame.payload + 4, 0x00010001U); /* Firmware 1.0.1, protocol V1. */
    Wire_WriteU32(frame.payload + 8, 0x0000001FU); /* bit4: fixed-reference spin translation. */
    Reply(&frame);
}

static bool PostControl(MotorRemoteAction action, const WireFrame *frame, bool hello, bool silent)
{
    MotorRemoteRequest request = {.action = action, .sequence = frame->sequence,
                                 .mode_generation = mode_generation};
    if (++ticket_counter == 0U)
    {
        ++ticket_counter;
    }
    request.ticket = ticket_counter;
    if (action == MOTOR_REMOTE_VELOCITY || action == MOTOR_REMOTE_SPIN)
    {
        for (unsigned int i = 0U; i < 3U; ++i)
        {
            request.values[i] = Wire_ReadI16(frame->payload + 4 + i * 2U);
        }
    }
    else if (action == MOTOR_REMOTE_HEADING)
    {
        request.values[0] = Wire_ReadI16(frame->payload + 4);
    }
    if (!MotorApp_PostRemote(&request))
    {
        return false;
    }
    control_pending = true;
    control_hello = hello;
    control_silent = silent;
    control_ticket = request.ticket;
    control_sequence = frame->sequence;
    control_type = frame->type;
    return true;
}

static uint16_t ExpectedLength(uint8_t type)
{
    switch (type)
    {
        case WIRE_HELLO: case WIRE_STOP: case WIRE_IMU_CAL: return 4U;
        case WIRE_ENABLE: return 5U;
        case WIRE_TIME_SYNC: return 8U;
        case WIRE_CMD_VEL: case WIRE_CMD_SPIN: return 10U;
        case WIRE_HEADING: return 6U;
        default: return 0U;
    }
}

void RosLink_Receive(const WireFrame *frame)
{
    MotorAppStatus motor;
    MotorApp_GetStatus(&motor);
    SyncMode(&motor);
    if (!motor.remote_mode)
    {
        Ack(frame->sequence, frame->type, 8U);
        return;
    }
    if (frame->type == WIRE_HELLO)
    {
        if (frame->length != 4U || frame->time_ms != 0U || Wire_ReadU32(frame->payload) == 0U)
        {
            Ack(frame->sequence, frame->type, 1U);
            return;
        }
        uint32_t incoming = Wire_ReadU32(frame->payload);
        if (incoming == session && established)
        {
            HelloReply();
            return;
        }
        if (incoming == session && control_pending && control_hello)
        {
            return;
        }
        if (!PostControl(MOTOR_REMOTE_RESET, frame, true, false))
        {
            Ack(frame->sequence, frame->type, 4U);
            return;
        }
        session = incoming;
        established = false;
        rx_sequence = frame->sequence;
        reply_count = 0U;
        memset(feedback, 0, sizeof(feedback));
        return;
    }
    uint16_t expected = ExpectedLength(frame->type);
    if (expected == 0U)
    {
        Ack(frame->sequence, frame->type, 7U);
        return;
    }
    if (frame->length != expected || frame->time_ms != 0U)
    {
        Ack(frame->sequence, frame->type, 1U);
        return;
    }
    if (session == 0U || Wire_ReadU32(frame->payload) != session)
    {
        Ack(frame->sequence, frame->type, 5U);
        return;
    }
    if (!established)
    {
        Ack(frame->sequence, frame->type, 4U);
        return;
    }
    uint16_t difference = (uint16_t)(frame->sequence - rx_sequence);
    bool fresh = difference != 0U && difference < 32768U;
    if (!fresh && frame->type != WIRE_STOP)
    {
        Ack(frame->sequence, frame->type, 6U);
        return;
    }
    if (fresh)
    {
        rx_sequence = frame->sequence;
    }
    if (frame->type == WIRE_TIME_SYNC)
    {
        WireFrame response = MakeFrame(WIRE_TIME_REPLY, 16U, HAL_GetTick());
        memcpy(response.payload + 4, frame->payload + 4, 4U);
        Wire_WriteU32(response.payload + 8, HAL_GetTick());
        Reply(&response);
        return;
    }
    MotorRemoteAction action;
    switch (frame->type)
    {
        case WIRE_ENABLE:
            if (frame->payload[4] > 1U)
            {
                Ack(frame->sequence, frame->type, 1U);
                return;
            }
            action = frame->payload[4] ? MOTOR_REMOTE_ARM : MOTOR_REMOTE_DISARM;
            break;
        case WIRE_STOP: action = MOTOR_REMOTE_STOP; break;
        case WIRE_CMD_VEL: action = MOTOR_REMOTE_VELOCITY; break;
        case WIRE_CMD_SPIN: action = MOTOR_REMOTE_SPIN; break;
        case WIRE_HEADING: action = MOTOR_REMOTE_HEADING; break;
        default: action = MOTOR_REMOTE_CALIBRATE; break;
    }
    bool priority = action == MOTOR_REMOTE_STOP || action == MOTOR_REMOTE_DISARM;
    if (control_pending && !priority)
    {
        Ack(frame->sequence, frame->type, 4U);
        return;
    }
    if (control_pending && !control_silent && !control_hello)
    {
        Ack(control_sequence, control_type, 4U);
    }
    if (!PostControl(action, frame, false, false))
    {
        Ack(frame->sequence, frame->type, 4U);
    }
}

void RosLink_TransportFault(void)
{
    if (session != 0U)
    {
        WireFrame frame = {.type = WIRE_STOP};
        (void)PostControl(MOTOR_REMOTE_STOP, &frame, false, true);
    }
}

static void StatusFrame(const MotorAppStatus *motor, const ImuAppStatus *imu,
                         const WireParser *parser, uint32_t drops)
{
    WireFrame *frame = &feedback[0].frame;
    *frame = MakeFrame(WIRE_STATUS, 24U, HAL_GetTick());
    uint16_t flags = (motor->remote_armed ? 1U : 0U) | (motor->running ? 2U : 0U) |
                     (motor->heading_hold ? 4U : 0U) | (imu->state == IMU_READY ? 8U : 0U) |
                     (imu->state == IMU_CALIBRATING ? 16U : 0U) | (motor->spin_mode ? 32U : 0U) |
                     (motor->remote_mode ? 64U : 0U);
    uint16_t faults = (imu->state == IMU_OFFLINE ? 1U : 0U) |
                      (motor->fault == 1U && imu->state != IMU_OFFLINE ? 2U : 0U) |
                      (motor->fault == 2U ? 4U : 0U) | (drops != 0U ? 8U : 0U) |
                      (motor->fault == 3U ? 16U : 0U);
    Wire_WriteU16(frame->payload + 4, flags);
    Wire_WriteU16(frame->payload + 6, faults);
    Wire_WriteU16(frame->payload + 8, motor->last_applied_cmd_seq);
    Wire_WriteU16(frame->payload + 10, motor->command_age_ms);
    uint16_t cpu = 0U;
    (void)CpuLoad_GetCached(&cpu);
    Wire_WriteU16(frame->payload + 12, cpu);
    Wire_WriteU16(frame->payload + 14, SaturateU16(HAL_GetTick() - imu->timestamp_ms));
    Wire_WriteU16(frame->payload + 16, SaturateU16((uint32_t)xPortGetFreeHeapSize()));
    Wire_WriteU16(frame->payload + 18, SaturateU16(parser->crc_errors));
    Wire_WriteU16(frame->payload + 20, SaturateU16(parser->drops + drops));
    frame->payload[22] = motor->stop_reason;
    frame->payload[23] = !motor->running ? 0U : motor->heading_move ? 2U :
                         motor->spin_mode ? 4U : motor->remote_mode ? 1U : 3U;
    feedback[0].pending = true;
}

static void WheelsFrame(const MotorAppStatus *motor)
{
    WireFrame *frame = &feedback[1].frame;
    *frame = MakeFrame(WIRE_WHEELS, 40U, motor->timestamp_ms);
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        Wire_WriteU32(frame->payload + 4 + 4U * i, motor->encoder_count[i]);
        Wire_WriteU16(frame->payload + 16 + 2U * i, (uint16_t)motor->wheel_target_x10[i]);
        Wire_WriteU16(frame->payload + 22 + 2U * i, (uint16_t)motor->rpm_x10[i]);
        Wire_WriteU16(frame->payload + 28 + 2U * i, (uint16_t)motor->wheel_pwm_permille[i]);
        Wire_WriteU16(frame->payload + 34 + 2U * i, (uint16_t)motor->body_measured[i]);
    }
    feedback[1].pending = true;
}

static int32_t MicroUnits(float value)
{
    return isfinite(value) ? (int32_t)lroundf(value * 1000000.0f) : 0;
}

static void ImuFrame(const ImuAppStatus *imu)
{
    WireFrame *frame = &feedback[2].frame;
    *frame = MakeFrame(WIRE_IMU, 36U, imu->timestamp_ms);
    bool valid = imu->valid && (HAL_GetTick() - imu->timestamp_ms <= 50U);
    uint16_t flags = (valid ? 1U : 0U) | (imu->state == IMU_READY ? 2U : 0U) |
                     (valid && imu->state == IMU_READY ? 4U : 0U);
    Wire_WriteU16(frame->payload + 4, flags);
    Wire_WriteU16(frame->payload + 6, SaturateU16(imu->sample_ms));
    for (unsigned int i = 0U; i < 3U; ++i)
    {
        Wire_WriteU32(frame->payload + 8 + 4U * i, (uint32_t)MicroUnits(imu->accel[i]));
        Wire_WriteU32(frame->payload + 20 + 4U * i, (uint32_t)MicroUnits(imu->gyro[i]));
    }
    Wire_WriteU32(frame->payload + 32, (uint32_t)MicroUnits(imu->yaw_rad));
    feedback[2].pending = true;
}

static void Transmit(void)
{
    WireFrame *frame = NULL;
    uint8_t slot = 0U;
    if (reply_count != 0U)
    {
        frame = &replies[reply_head];
    }
    else
    {
        for (uint8_t i = 0U; i < 3U; ++i)
        {
            slot = (feedback_next + i) % 3U;
            if (feedback[slot].pending)
            {
                frame = &feedback[slot].frame;
                break;
            }
        }
    }
    if (frame == NULL)
    {
        return;
    }
    frame->sequence = tx_sequence;
    if (frame->type == WIRE_TIME_REPLY)
    {
        frame->time_ms = HAL_GetTick();
        Wire_WriteU32(frame->payload + 12, frame->time_ms);
    }
    uint8_t bytes[WIRE_MAX_FRAME];
    size_t length = Wire_Encode(bytes, frame);
    if (SerialDma_TryWrite(bytes, length))
    {
        ++tx_sequence;
        if (reply_count != 0U)
        {
            reply_head = (reply_head + 1U) % REPLY_CAPACITY;
            --reply_count;
        }
        else
        {
            feedback[slot].pending = false;
            feedback_next = (slot + 1U) % 3U;
        }
    }
}

void RosLink_Poll(const WireParser *parser, uint32_t drops)
{
    MotorAppStatus motor;
    ImuAppStatus imu;
    MotorApp_GetStatus(&motor);
    SyncMode(&motor);
    ImuApp_GetStatus(&imu);
    uint32_t now = HAL_GetTick();
    if (control_pending && motor.remote_ticket == control_ticket)
    {
        if (control_hello && motor.remote_result == 0U)
        {
            established = true;
            HelloReply();
            imu_at = wheels_at = status_at = now;
        }
        else if (!control_silent && !control_hello &&
                 ((control_type != WIRE_CMD_VEL && control_type != WIRE_CMD_SPIN) || motor.remote_result != 0U))
        {
            Ack(control_sequence, control_type, motor.remote_result);
        }
        control_pending = false;
    }
    if (established)
    {
        if (now - imu_at >= 10U)
        {
            imu_at = now;
            ImuFrame(&imu);
        }
        if (now - wheels_at >= 20U)
        {
            wheels_at = now;
            WheelsFrame(&motor);
        }
        if (now - status_at >= 200U)
        {
            status_at = now;
            StatusFrame(&motor, &imu, parser, drops);
        }
    }
    Transmit();
}
