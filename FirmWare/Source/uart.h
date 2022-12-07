
#ifndef __UART_H
#define __UART_H

#include <stdint.h>
#include <stdbool.h>

#include "main.h"

//Идентификаторы скорости обмена
typedef enum {
    UART_SPEED_600,                         //идентификатор скорости обмена 600 бит/сек
    UART_SPEED_1200,                        //идентификатор скорости обмена 1200 бит/сек
    UART_SPEED_2400,                        //идентификатор скорости обмена 2400 бит/сек
    UART_SPEED_4800,                        //идентификатор скорости обмена 4800 бит/сек
    UART_SPEED_9600,                        //идентификатор скорости обмена 9600 бит/сек
    UART_SPEED_14400,                       //идентификатор скорости обмена 14400 бит/сек
    UART_SPEED_19200,                       //идентификатор скорости обмена 19200 бит/сек
    UART_SPEED_28800,                       //идентификатор скорости обмена 28800 бит/сек
    UART_SPEED_38400,                       //идентификатор скорости обмена 38400 бит/сек
    UART_SPEED_56000,                       //идентификатор скорости обмена 56000 бит/сек
    UART_SPEED_57600,                       //идентификатор скорости обмена 57600 бит/сек
    UART_SPEED_115200                       //идентификатор скорости обмена 115200 бит/сек
 } UARTSpeed;

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void UartInit( void );
void UartSendStr( char *str );
void UartRecvComplt( void );
void UartSendComplt( void );
char *UartBuffer( void );
uint32_t UartGetSpeed( UARTSpeed speed );
uint16_t ModbusTimeOut( UARTSpeed speed );
ErrorStatus CheckBaudRate( uint32_t baud, UARTSpeed *speed );

#endif
