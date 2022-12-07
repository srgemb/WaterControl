
#ifndef __WATER_H
#define __WATER_H

#include <stdint.h>
#include <stdbool.h>

#include "water.h"
#include "valve.h"

//Источник давления
typedef enum {
    WATER_COLD,                             //холодная вода
    WATER_HOT                               //горячая вода
 } Water;

//Тип учета расхода воды
typedef enum {
    COUNT_COLD,                             //холодная вода
    COUNT_HOT,                              //горячая вода
    COUNT_FILTER                            //питьевая вода
 } CountType;

//Состояние датчика утечки
typedef enum {
    LEAK_NO,                                //утечки нет
    LEAK_YES                                //есть утечка
 } LeakStat;

//Контроль напряжения 12VDC
typedef enum {
    DC12V_ERROR,                            //напряжение отсутствует
    DC12V_OK                                //напряжение есть
 } DC12VStat;

//Тип датчика утечки
typedef enum {
    LEAK_ALL,                               //датчик 1+2
    LEAK1,                                  //датчик 1
    LEAK2                                   //датчик 2
 } LeakType;

//Тип данных
typedef enum {
    EVENT_DATA,                             //интервальные данные
    EVENT_ALARM                             //событие утечки
 } EventType;

#pragma pack( push, 1 )

//структура для хранения текущих значений счетчиков расхода воды
typedef struct {
    uint32_t    count_cold;                 //значения счетчика холодной воды
    uint32_t    count_hot;                  //значения счетчика горячей воды
    uint32_t    count_filter;               //значения счетчика питьевой воды
    uint16_t    reserv;                     //резерв
    uint16_t    next_addr;                  //адрес в FRAM для записи следующего события в журнал
    //источник сброса и дата/время включения контроллера
    uint8_t     res_src;                    //источник сброса
    uint8_t     day;                        //день
    uint8_t     month;                      //месяц
    uint16_t    year;                       //год
    uint8_t     hour;                       //часы
    uint8_t     min;                        //минуты
    uint8_t	    sec;                        //секунды
 } CURR_DATA;

//структура для хранения интервальных значений: счетчиков, давления, датчиков утечки
typedef struct {
    //дата/время
    uint8_t	    sec;                        //секунды
    uint8_t     min;                        //минуты
    uint8_t     hour;                       //часы
    uint8_t     day;                        //день
    uint8_t     month;                      //месяц
    uint16_t    year;                       //год
    uint8_t	    unused;                     //выравнивание до 8 байт
    //состояния электроприводов
    ValveStat   stat_valve_cold  : 2;       //статус электропривода холодный воды    
    ValveError  error_valve_cold : 2;       //код ошибки электропривода холодный воды
    ValveStat   stat_valve_hot : 2;         //статус электропривода горячей воды     
    ValveError  error_valve_hot : 2;        //код ошибки электропривода горячей воды 
    //значения учета
    uint32_t    count_cold;                 //значения счетчика холодной воды
    uint32_t    count_hot;                  //значения счетчика горячей воды
    uint32_t    count_filter;               //значения счетчика питьевой воды
    uint16_t    pressr_cold;                //давление холодной воды
    uint16_t    pressr_hot;                 //давление горячей воды
    LeakStat    leak1 : 1;                  //состояние датчика утечки #1
    LeakStat    leak2 : 1;                  //состояние датчика утечки #2
    unsigned    reserv : 4;                 //резерв
    EventType   type_event : 1;             //признак данных: данные/событие
    DC12VStat   dc12_chk : 1;               //контроль напряжения 12VDС для питания датчиков утечки
 } WATER_LOG;

#pragma pack( pop )

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void WaterInit( void );
LeakStat LeakStatus( LeakType type );
DC12VStat DC12VStatus( void );

#endif 
