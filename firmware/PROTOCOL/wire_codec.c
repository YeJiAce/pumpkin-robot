/** Portable V1 byte codec: no packed structs, alignment assumptions or HAL dependencies. */
#include "wire_codec.h"
#include <string.h>

uint16_t Wire_ReadU16(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

int16_t Wire_ReadI16(const uint8_t *bytes)
{
    uint16_t value = Wire_ReadU16(bytes);
    return (int16_t)(value < 32768U ? (int32_t)value : (int32_t)value - 65536);
}

uint32_t Wire_ReadU32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
           ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

void Wire_WriteU16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8);
}

void Wire_WriteU32(uint8_t *bytes, uint32_t value)
{
    for (unsigned int i = 0U; i < 4U; ++i)
    {
        bytes[i] = (uint8_t)(value >> (8U * i));
    }
}

uint16_t Wire_Crc16(const uint8_t *bytes, size_t length)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0U; i < length; ++i)
    {
        crc ^= (uint16_t)((uint16_t)bytes[i] << 8);
        for (unsigned int bit = 0U; bit < 8U; ++bit)
        {
            crc = (uint16_t)((crc & 0x8000U) != 0U ? (crc << 1) ^ 0x1021U : crc << 1);
        }
    }
    return crc;
}

size_t Wire_Encode(uint8_t *bytes, const WireFrame *frame)
{
    if (frame->length > WIRE_MAX_PAYLOAD)
    {
        return 0U;
    }
    bytes[0] = 0xAAU;
    bytes[1] = 0x55U;
    bytes[2] = WIRE_VERSION;
    bytes[3] = frame->type;
    Wire_WriteU16(bytes + 4, frame->sequence);
    Wire_WriteU16(bytes + 6, frame->length);
    Wire_WriteU32(bytes + 8, frame->time_ms);
    memcpy(bytes + 12, frame->payload, frame->length);
    Wire_WriteU16(bytes + 12 + frame->length, Wire_Crc16(bytes + 2, 10U + frame->length));
    return WIRE_OVERHEAD + frame->length;
}

static void Discard(WireParser *parser, uint16_t count)
{
    parser->used -= count;
    memmove(parser->bytes, parser->bytes + count, parser->used);
}

void WireParser_Push(WireParser *parser, uint8_t byte, uint32_t now_ms)
{
    if (parser->used == WIRE_MAX_FRAME)
    {
        Discard(parser, 1U);
        ++parser->drops;
    }
    parser->bytes[parser->used++] = byte;
    parser->last_byte_ms = now_ms;
}

bool WireParser_Next(WireParser *parser, uint32_t now_ms, WireFrame *frame)
{
    for (;;)
    {
        if (parser->used < 2U)
        {
            return false;
        }
        if ((parser->bytes[0] != 0xAAU) || (parser->bytes[1] != 0x55U))
        {
            Discard(parser, 1U);
            continue;
        }
        bool timeout = (now_ms - parser->last_byte_ms) >= 50U;
        if (parser->used < 12U)
        {
            if (timeout)
            {
                Discard(parser, 1U);
                ++parser->drops;
                continue;
            }
            return false;
        }
        uint16_t length = Wire_ReadU16(parser->bytes + 6);
        if ((parser->bytes[2] != WIRE_VERSION) || (length > WIRE_MAX_PAYLOAD))
        {
            Discard(parser, 1U);
            ++parser->drops;
            continue;
        }
        uint16_t total = WIRE_OVERHEAD + length;
        if (parser->used < total)
        {
            if (timeout)
            {
                Discard(parser, 1U);
                ++parser->drops;
                continue;
            }
            return false;
        }
        if (Wire_Crc16(parser->bytes + 2, 10U + length) != Wire_ReadU16(parser->bytes + 12 + length))
        {
            Discard(parser, 1U);
            ++parser->crc_errors;
            continue;
        }
        frame->type = parser->bytes[3];
        frame->sequence = Wire_ReadU16(parser->bytes + 4);
        frame->length = length;
        frame->time_ms = Wire_ReadU32(parser->bytes + 8);
        memcpy(frame->payload, parser->bytes + 12, length);
        Discard(parser, total);
        return true;
    }
}
