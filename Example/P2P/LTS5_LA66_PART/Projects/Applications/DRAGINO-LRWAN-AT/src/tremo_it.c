#include "tremo_lpuart.h"
#include "tremo_it.h"
#include "tremo_gpio.h"

#include <stdio.h>
#include "tremo_uart.h"
#include "log.h"

// extern uint8_t esp_pc5_flag;

extern uint8_t uart_rx_data_from_esp32[32];
extern uint8_t uart_esp32_rx_data_len_index;
extern uint8_t uart_all_data_rx_done_flag;

extern void RadioOnDioIrq(void);
extern void RtcOnIrq(void);
extern void linkwan_serial_input(uint8_t cmd);
extern void dma0_IRQHandler(void);
extern void dma1_IRQHandler(void);
/**
 * @brief  This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void)
{

    /* Go to infinite loop when Hard Fault exception occurs */
    while (1) { }
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1) { }
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1) { }
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1) { }
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void)
{
}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler(void)
{
}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void)
{
}

/**
 * @brief  This function handles PWR Handler.
 * @param  None
 * @retval None
 */
void PWR_IRQHandler()
{
}

/******************************************************************************/
/*                 Tremo Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_cm4.S).                                               */
/******************************************************************************/

/**
 * @brief  This function handles PPP interrupt request.
 * @param  None
 * @retval None
 */
void LORA_IRQHandler()
{
    RadioOnDioIrq();
}

void RTC_IRQHandler(void)
{
    RtcOnIrq();
}

void UART0_IRQHandler(void)
{
}

void LPUART_IRQHandler(void)
{
    if (lpuart_get_rx_status(LPUART, LPUART_SR0_RX_DONE_STATE)) {
        uint8_t rx_data_temp = lpuart_receive_data(LPUART);
        lpuart_clear_rx_status(LPUART, LPUART_SR0_RX_DONE_STATE);
        linkwan_serial_input(rx_data_temp);
    }
}

void esp32_rx_handle_func(uint8_t rx_data) {
    if(rx_data == 0x00) {
        if(uart_esp32_rx_data_len_index == 0) {
            LOG_PRINTF(LL_DEBUG, "Uart2 receive abnormal data: (%d)\r\n", rx_data);
            return;
        }
    }
    if(uart_all_data_rx_done_flag == 1) {
        return;
    }
	if( (rx_data == '\n') && (uart_rx_data_from_esp32[uart_esp32_rx_data_len_index - 1] == '\r') && (uart_esp32_rx_data_len_index >= 1)) {
        uart_rx_data_from_esp32[uart_esp32_rx_data_len_index - 1] = '\0';
	    uart_esp32_rx_data_len_index += 1;

		uart_all_data_rx_done_flag = 1;
		return;
	}
	uart_rx_data_from_esp32[uart_esp32_rx_data_len_index] = rx_data;
	uart_esp32_rx_data_len_index += 1;
}

void UART2_IRQHandler(void) {
    if(uart_get_interrupt_status(UART2, UART_INTERRUPT_RX_DONE) == SET)
    {
        
        uint8_t rx_data_temp = uart_receive_data(UART2);
        LOG_PRINTF(LL_DEBUG, "Uart2 receive (%02X)\r\n", rx_data_temp);
        esp32_rx_handle_func(rx_data_temp);
        uart_clear_interrupt(UART2, UART_INTERRUPT_RX_DONE);
    }
}

/**
 * @brief  This function handles dma0 Handler.
 * @param  None
 * @retval None
 */
void DMA0_IRQHandler(void)
{
    dma0_IRQHandler();
}

/**
 * @brief  This function handles dma1 Handler.
 * @param  None
 * @retval None
 */
void DMA1_IRQHandler(void)
{
    dma1_IRQHandler();
}

/******************************************************************************/
/*                 Tremo Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_cm4.S).                                               */
/******************************************************************************/
void GPIO_IRQHandler(void)
{
    // if(gpio_get_interrupt_status(GPIOC, GPIO_PIN_5) == SET) {
        // gpio_clear_interrupt(GPIOC, GPIO_PIN_5);
        // esp_pc5_flag = 1;
    // }
}
