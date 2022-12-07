
#ifndef __RS485_H
#define __RS485_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

//*************************************************************************************************
// Функции управления/статуса/состояния
//*************************************************************************************************
void Rs485Init( void );
void Rs485Callback( void );
void RS485RecvComplt( void );
void RS485SendComplt( void );
uint8_t *RS485SendBuff( void );
void ClearSend( void );

#endif
