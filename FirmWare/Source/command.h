
#ifndef __COMMAND_H_
#define __COMMAND_H_

#include <stdio.h>
#include <stdint.h>

//тип вывода адреса при выводе HEX дампа
typedef enum {
    HEX_16BIT_ADDR,                             //16 бит
    HEX_32BIT_ADDR                              //32 бита
 } HEX_TYPE_ADDR;

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void CommandInit( void );
void DataHexDump( uint8_t *data, HEX_TYPE_ADDR type_addr, uint32_t addr, char *buff );

#endif
