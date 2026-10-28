/** FreeRTOS runtime counter and non-idle task utilization. */
#include "cpu_load.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define STATS_TASK_CAPACITY 12U
#define COUNTER_HZ 100000U

static uint32_t counter;
static uint32_t previous_cycles;
static uint32_t remainder_cycles;
static uint32_t cycles_per_unit;
static volatile uint16_t cached_load;
static volatile bool cached_valid;

bool CpuLoad_GetCached(uint16_t *load_permille)
{
    *load_permille = cached_load;
    return cached_valid;
}

/* Strong implementations replace CubeMX's weak runtime-stat hooks. */
void configureTimerForRunTimeStats(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    previous_cycles = 0U;
    remainder_cycles = 0U;
    counter = 0U;
    cycles_per_unit = SystemCoreClock / COUNTER_HZ;
}

unsigned long getRunTimeCounterValue(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint32_t now = DWT->CYCCNT;
    /* Unsigned subtraction extends CYCCNT across its ~60-second wrap.
       FreeRTOS calls this on context switches, at least once each tick here. */
    uint64_t cycles = (uint64_t)(uint32_t)(now - previous_cycles) + remainder_cycles;
    previous_cycles = now;
    if (cycles_per_unit != 0U)
    {
        counter += (uint32_t)(cycles / cycles_per_unit);
        remainder_cycles = (uint32_t)(cycles % cycles_per_unit);
    }
    uint32_t value = counter;
    __set_PRIMASK(mask);
    return value;
}

bool CpuLoad_Read(uint16_t *load_permille)
{
    static TaskStatus_t tasks[STATS_TASK_CAPACITY];
    static uint32_t previous_total;
    static uint32_t previous_idle;
    static uint32_t sampled_at;
    static uint16_t load;
    static bool baseline;
    static bool valid;

    uint32_t now = HAL_GetTick();
    if (!baseline || ((now - sampled_at) >= 1000U))
    {
        uint32_t total = 0U;
        UBaseType_t count = uxTaskGetSystemState(tasks, STATS_TASK_CAPACITY, &total);
        bool found = false;
        uint32_t idle = 0U;
        for (UBaseType_t i = 0U; i < count; ++i)
        {
            /* This project's FreeRTOS 10.3.1 kernel creates its idle task as "IDLE". */
            if (strcmp(tasks[i].pcTaskName, "IDLE") == 0)
            {
                idle = tasks[i].ulRunTimeCounter;
                found = true;
                break;
            }
        }
        if (!found)
        {
            *load_permille = 0U;
            return false;
        }
        uint32_t elapsed = total - previous_total;
        uint32_t idle_elapsed = idle - previous_idle;
        if (baseline && (elapsed != 0U) && (idle_elapsed <= elapsed))
        {
            load = (uint16_t)(1000U - (uint64_t)idle_elapsed * 1000U / elapsed);
            valid = true;
            cached_load = load;
            cached_valid = true;
        }
        previous_total = total;
        previous_idle = idle;
        sampled_at = now;
        baseline = true;
    }
    *load_permille = load;
    return valid;
}
