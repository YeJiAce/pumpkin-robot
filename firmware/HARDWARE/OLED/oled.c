/** HAL adaptation of WHEELTEC C12H OLED serial interface and initialization. */
#include "oled.h"
#include "oled_font.h"
#include "main.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

static uint8_t frame[8][128];

static void WriteByte(uint8_t value, bool data)
{
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, data ? GPIO_PIN_SET : GPIO_PIN_RESET);
    for (unsigned int bit = 0U; bit < 8U; ++bit)
    {
        HAL_GPIO_WritePin(OLED_SCLK_GPIO_Port, OLED_SCLK_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(OLED_SDIN_GPIO_Port, OLED_SDIN_Pin,
                          (value & 0x80U) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(OLED_SCLK_GPIO_Port, OLED_SCLK_Pin, GPIO_PIN_SET);
        value <<= 1;
    }
    HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_SET);
}

static void SetPage(uint8_t page)
{
    WriteByte((uint8_t)(0xB0U + page), false);
    WriteByte(0x00U, false);
    WriteByte(0x10U, false);
}

void Oled_Init(void)
{
    /* Same controller settings as the supplied example; no GPIO/debug reconfiguration. */
    static const uint8_t commands[] =
    {
        0xAE, 0xD5, 0x50, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x02, 0xA1, 0xC0, 0xDA, 0x12,
        0x81, 0xEF, 0xD9, 0xF1, 0xDB, 0x30, 0xA4, 0xA6
    };
    HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_RESET);
    osDelay(100U);
    HAL_GPIO_WritePin(OLED_RST_GPIO_Port, OLED_RST_Pin, GPIO_PIN_SET);
    osDelay(10U);
    for (unsigned int i = 0U; i < sizeof(commands); ++i)
    {
        WriteByte(commands[i], false);
    }
    for (uint8_t page = 0U; page < 8U; ++page)
    {
        SetPage(page);
        for (unsigned int x = 0U; x < 128U; ++x)
        {
            WriteByte(0U, true);
        }
    }
    WriteByte(0xAFU, false);
}

static void DrawLine(uint8_t y, const char *text)
{
    for (unsigned int character = 0U; (character < 21U) && (text[character] != '\0'); ++character)
    {
        uint8_t code = (uint8_t)text[character];
        if ((code < 32U) || (code > 126U))
        {
            code = '?';
        }
        for (unsigned int column = 0U; column < 6U; ++column)
        {
            for (unsigned int row = 0U; row < 12U; ++row)
            {
                uint8_t glyph = oled_asc2_1206[code - 32U][column * 2U + row / 8U];
                if ((glyph & (0x80U >> (row % 8U))) != 0U)
                {
                    unsigned int pixel_y = y + row;
                    /* Match the supplied driver's logical-to-panel orientation. */
                    frame[7U - pixel_y / 8U][character * 6U + column] |=
                        (uint8_t)(1U << (7U - pixel_y % 8U));
                }
            }
        }
    }
}

void Oled_ShowLines(const char *first, const char *second, const char *third)
{
    memset(frame, 0, sizeof(frame));
    DrawLine(0U, first);
    DrawLine(16U, second);
    DrawLine(32U, third);
    for (uint8_t page = 2U; page < 8U; ++page)
    {
        SetPage(page);
        for (unsigned int x = 0U; x < 128U; ++x)
        {
            WriteByte(frame[page][x], true);
        }
    }
}
