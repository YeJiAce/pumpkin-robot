/** Binary-only UART transport loop. Text diagnostics are intentionally not mixed in. */
#include "comm_app.h"
#include "serial_dma.h"
#include "wire_codec.h"
#include "ros_link.h"
#include "cmsis_os.h"
#include "main.h"

void CommApp_Run(void)
{
    static WireParser parser;
    static WireFrame received;
    uint32_t transport_drops = 0U;
    if (!SerialDma_Init())
    {
        Error_Handler();
    }
    for (;;)
    {
        SerialDma_Service();
        if (SerialDma_TakeRxFault())
        {
            parser.used = 0U;
            ++transport_drops;
            RosLink_TransportFault();
        }
        for (unsigned int i = 0U; i < 512U; ++i)
        {
            uint8_t byte;
            if (!SerialDma_ReadByte(&byte))
            {
                break;
            }
            WireParser_Push(&parser, byte, HAL_GetTick());
            while (WireParser_Next(&parser, HAL_GetTick(), &received))
            {
                RosLink_Receive(&received);
            }
        }
        while (WireParser_Next(&parser, HAL_GetTick(), &received))
        {
            RosLink_Receive(&received);
        }
        RosLink_Poll(&parser, transport_drops);
        osDelay(1U);
    }
}
