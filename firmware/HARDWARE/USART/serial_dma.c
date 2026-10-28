/** USART1 transport: circular RX DMA -> software FIFO, normal TX DMA. */
#include "serial_dma.h"

#include "cmsis_os.h"
#include "usart.h"
#include <string.h>

#define RX_DMA_SIZE       256U
#define RX_FIFO_SIZE      1024U
#define TX_BUFFER_SIZE    192U
#define TX_TIMEOUT_MS     100U

static uint8_t rx_dma[RX_DMA_SIZE];
static uint8_t rx_fifo[RX_FIFO_SIZE];
static uint8_t tx_buffer[TX_BUFFER_SIZE];
static uint16_t rx_position;
static volatile uint16_t fifo_head;
static volatile uint16_t fifo_tail;
static volatile bool rx_restart;
static volatile bool rx_fault;
static bool tx_active;
static uint32_t tx_started_at;

bool SerialDma_Init(void)
{
    rx_position = 0U;
    fifo_head = 0U;
    fifo_tail = 0U;
    rx_restart = false;
    rx_fault = false;
    return HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma, RX_DMA_SIZE) == HAL_OK;
}

void SerialDma_Service(void)
{
    if (tx_active)
    {
        if (huart1.gState == HAL_UART_STATE_READY)
        {
            tx_active = false;
        }
        else if ((HAL_GetTick() - tx_started_at) >= TX_TIMEOUT_MS)
        {
            (void)HAL_UART_AbortTransmit(&huart1);
            tx_active = false;
        }
    }
    if (rx_restart)
    {
        /* HAL error processing may stop RX DMA. Recovery belongs in task context. */
        if (HAL_UART_AbortReceive(&huart1) == HAL_OK)
        {
            rx_position = 0U;
            rx_restart = false;
            if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma, RX_DMA_SIZE) != HAL_OK)
            {
                rx_restart = true;
            }
        }
    }
}

bool SerialDma_ReadByte(uint8_t *byte)
{
    if (fifo_tail == fifo_head)
    {
        return false;
    }

    *byte = rx_fifo[fifo_tail];
    __DMB();
    fifo_tail = (uint16_t)((fifo_tail + 1U) % RX_FIFO_SIZE);
    return true;
}

bool SerialDma_TakeRxFault(void)
{
    bool fault;
    uint32_t mask = __get_PRIMASK();

    __disable_irq();
    fault = rx_fault;
    if (fault)
    {
        /* Discard fragments after loss; protocol parser searches for the next frame. */
        fifo_tail = fifo_head;
        rx_fault = false;
    }
    __set_PRIMASK(mask);
    return fault;
}

bool SerialDma_Write(const char *text)
{
    size_t length = strlen(text);
    uint32_t start;

    if ((length == 0U) || (length > sizeof(tx_buffer)) ||
        (huart1.gState != HAL_UART_STATE_READY))
    {
        return false;
    }

    /* Owned copy remains valid until DMA and the final UART TC interrupt finish. */
    memcpy(tx_buffer, text, length);
    if (HAL_UART_Transmit_DMA(&huart1, tx_buffer, (uint16_t)length) != HAL_OK)
    {
        return false;
    }

    start = HAL_GetTick();
    while (huart1.gState != HAL_UART_STATE_READY)
    {
        if ((HAL_GetTick() - start) >= TX_TIMEOUT_MS)
        {
            (void)HAL_UART_AbortTransmit(&huart1);
            return false;
        }
        osDelay(1U);
    }
    return true;
}

bool SerialDma_TryWrite(const uint8_t *bytes, size_t length)
{
    if (tx_active || (huart1.gState != HAL_UART_STATE_READY) ||
        (length == 0U) || (length > sizeof(tx_buffer)))
    {
        return false;
    }
    memcpy(tx_buffer, bytes, length);
    if (HAL_UART_Transmit_DMA(&huart1, tx_buffer, (uint16_t)length) != HAL_OK)
    {
        return false;
    }
    tx_started_at = HAL_GetTick();
    tx_active = true;
    return true;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    if ((uart != &huart1) || rx_restart || (size > RX_DMA_SIZE))
    {
        return;
    }

    /* HT, TC and IDLE all report positions; retain SIZE at TC to avoid duplicates. */
    uint16_t position = rx_position;
    while (position != size)
    {
        if (position == RX_DMA_SIZE)
        {
            position = 0U;
            if (position == size)
            {
                break;
            }
        }

        uint16_t next = (uint16_t)((fifo_head + 1U) % RX_FIFO_SIZE);
        if (next == fifo_tail)
        {
            rx_fault = true;
        }
        else
        {
            rx_fifo[fifo_head] = rx_dma[position];
            __DMB();
            fifo_head = next;
        }
        ++position;
    }
    rx_position = size;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1)
    {
        rx_fault = true;
        rx_restart = true;
    }
}
