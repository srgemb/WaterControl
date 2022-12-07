
#ifndef __XTIME_H
#define __XTIME_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32f1xx.h"

//тип выводимых данных для DateTimeStr()
typedef enum {
    MASK_DATE = 1,                          //вывод даты
    MASK_TIME,                              //вывод времени
    MASK_DATE_TIME                          //вывод дата + время
} DataTimeMask;

#pragma pack( push, 1 )                     //выравнивание структуры по границе 1 байта

//Структура для хранения дата/время
typedef struct {
    uint8_t     day;                        //день
    uint8_t     month;                      //месяц
    uint16_t    year;                       //год
    uint8_t     hour;                       //часы
    uint8_t     min;                        //минуты
    uint8_t	    sec;                        //секунды
} DATE_TIME;

#pragma pack( pop )

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void GetTimeDate( DATE_TIME *ptr );
ErrorStatus SetTimeDate( DATE_TIME *ptr );
uint8_t DayOfWeek( uint8_t day, uint8_t month, uint16_t year );
ErrorStatus TimeSet( char *time );
ErrorStatus DateSet( char *date );
char *DateTimeStr( char *buff, DataTimeMask mask );

#endif
