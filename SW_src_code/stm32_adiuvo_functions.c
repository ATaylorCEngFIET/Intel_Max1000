#include "main.h"
#include "string.h"

extern UART_HandleTypeDef huart6;

void print (char * data)
{
	HAL_UART_Transmit(&huart6, (uint8_t*) data, strlen(data), 1000);
}
