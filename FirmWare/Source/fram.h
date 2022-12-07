
#ifndef __FRAM_H
#define __FRAM_H

#include "config.h"
#include "water.h"

//#define DEBUG_FRAM                                      //вывод отладочной информации

#define FRAM_SIZE           2048                        //размер FRAM памяти (байт)

#define FRAM_ADDR_DATA      0x0000                      //адрес хранения текущих параметров расхода воды
#define FRAM_ADDR_LOG       0x0020                      //адрес хранения событий и интервальных данных расхода воды

#define FRAM_BLOCK_SIZE     32                          //размер логического блока данных (байт)

#define FRAM_BLOCKS         (FRAM_SIZE/FRAM_BLOCK_SIZE) //кол-во доступных блоков в FRAM, в т.ч. для текущих данных

//Результат выполнения операций с FRAM памятью
typedef enum {
    FRAM_OK,                                            //данные прочитаны/записаны успешно
    FRAM_ERROR,                                         //данные не прочитаны, ошибка FRAM
    FRAM_BUSY,                                          //I2C интерфейс занят
    FRAM_TIMEOUT,                                       //вышло время ожидания
    FRAM_ERROR_CRC,                                     //данные прочитаны, CRC не совпала
    FRAM_ERROR_PARAM                                    //ошибка в параметрах
} FramStatus;

//тип статуса после чтения/записи (только в режиме инициализации контроллера)
typedef enum {
    FRAM_ERR_READ,                                      //ошибки чтения
    FRAM_ERR_WRITE                                      //ошибки записи
} FramErrorType;

//тип записываемых данных
typedef enum {
    CURRENT_DATA,                                       //текущие данные расхода воды 
    WATER_DATA_LOG                                      //аварийные события, журнал текущих данных
} TypeData;

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void FramInit( void );
void FramCheck( void );
void FramClear( void );
void FramTest( void );
void FramHexDump( uint8_t blocks );
char *FramErrorDesc( FramStatus error );
FramStatus FramError( FramErrorType type );
FramStatus FramReadData( uint16_t addr, uint8_t *ptr_data, uint16_t len );
FramStatus FramSaveData( TypeData type, uint8_t *ptr_data, uint16_t len );

#endif

