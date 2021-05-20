#include "main.h"
#include "string.h"

void print (char * data)
{
	HAL_UART_Transmit(&huart6, (uint8_t*) data, strlen(data), 1000);
}
