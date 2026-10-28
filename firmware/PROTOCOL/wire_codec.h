#ifndef PUMPKIN_WIRE_CODEC_H
#define PUMPKIN_WIRE_CODEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define WIRE_VERSION 1U
#define WIRE_MAX_PAYLOAD 64U
#define WIRE_OVERHEAD 14U
#define WIRE_MAX_FRAME (WIRE_MAX_PAYLOAD + WIRE_OVERHEAD)

enum
{
    WIRE_HELLO = 0x01, WIRE_ENABLE = 0x02, WIRE_STOP = 0x03, WIRE_TIME_SYNC = 0x04,
    WIRE_CMD_VEL = 0x10, WIRE_HEADING = 0x11, WIRE_IMU_CAL = 0x12, WIRE_CMD_SPIN = 0x13,
    WIRE_STATUS = 0x20, WIRE_WHEELS = 0x21, WIRE_IMU = 0x22,
    WIRE_ACK = 0x80, WIRE_HELLO_REPLY = 0x81, WIRE_TIME_REPLY = 0x82
};

typedef struct
{
    uint8_t type;
    uint16_t sequence;
    uint16_t length;
    uint32_t time_ms;
    uint8_t payload[WIRE_MAX_PAYLOAD];
} WireFrame;

typedef struct
{
    uint8_t bytes[WIRE_MAX_FRAME];
    uint16_t used;
    uint32_t last_byte_ms;
    uint32_t crc_errors;
    uint32_t drops;
} WireParser;

uint16_t Wire_Crc16(const uint8_t *bytes, size_t length);
uint16_t Wire_ReadU16(const uint8_t *bytes);
int16_t Wire_ReadI16(const uint8_t *bytes);
uint32_t Wire_ReadU32(const uint8_t *bytes);
void Wire_WriteU16(uint8_t *bytes, uint16_t value);
void Wire_WriteU32(uint8_t *bytes, uint32_t value);
size_t Wire_Encode(uint8_t *bytes, const WireFrame *frame);
void WireParser_Push(WireParser *parser, uint8_t byte, uint32_t now_ms);
bool WireParser_Next(WireParser *parser, uint32_t now_ms, WireFrame *frame);

#endif
