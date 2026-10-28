#include "user_key.h"

#include "main.h"

#define DEBOUNCE_MS 30U

static bool candidate;
static bool stable;
static bool armed;
static uint32_t changed_at;

static bool IsDown(void)
{
    return HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == GPIO_PIN_RESET;
}

void UserKey_Init(uint32_t now_ms)
{
    candidate = IsDown();
    stable = candidate;
    armed = false; /* Also require a stable release when booting with key held. */
    changed_at = now_ms;
}

bool UserKey_Pressed(uint32_t now_ms)
{
    bool down = IsDown();
    if (down != candidate)
    {
        candidate = down;
        changed_at = now_ms;
    }

    if ((now_ms - changed_at) < DEBOUNCE_MS)
    {
        return false;
    }
    stable = candidate;
    if (!stable)
    {
        armed = true;
    }
    else if (armed)
    {
        armed = false;
        return true;
    }
    return false;
}
