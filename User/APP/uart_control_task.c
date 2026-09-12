/**
	*********************************************************************
	* @file      uart_control_task.c/h
	* @brief     UART control task: processes UART commands and applies them to chassis
	*            Similar to PS2 control but receives commands via UART1
	* @note      Runs at 100Hz to continuously update chassis setpoints from UART
	* @history
	*
	@verbatim
	==============================================================================

	==============================================================================
	@endverbatim
	*********************************************************************
	*/

#include "uart_control_task.h"
#include "app_uart.h"
#include "chassisR_task.h"
#include "cmsis_os.h"

extern UartCmd_t uart_cmd;
extern chassis_t chassis_move;

void UART_Control_Task(void const * argument)
{
    // UART control mode completely removed as requested
    osDelay(10); // 10ms = 100Hz dummy loop to prevent task exit
}
