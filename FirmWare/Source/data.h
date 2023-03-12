
#ifndef __DATA_H
#define __DATA_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "water.h"
#include "xtime.h"
#include "valve.h"
#include "fram.h"

//Тип передаваемых данных, CAN шина
typedef enum {
    DATA_LEAKS_VALVE,                       //состояния датчиков утечки и состояния электроприводов 
    DATA_COLD,                              //текущие данные по холодной воде
    DATA_HOT,                               //текущие данные по горячей воде
    DATA_FILTER,                            //текущие данные по питьевой воде
    DATA_DATE,                              //текущие данные RTC
    DATA_LOG_COLD,                          //данные из журнала событий за указанную дату по холодной воде
    DATA_LOG_HOT,                           //данные из журнала событий за указанную дату по горячей воде
    DATA_LOG_FILTER                         //данные из журнала событий за указанную дату по питьевой воде
 } DataType;

//ID пакетов передаваемых/принимаемых через радио модуль
typedef enum {
    //исходящие ответа
    ZB_PACK_STATE = 1,                      //данные состояния контроллера
    ZB_PACK_DATA,                           //текущие данные расхода/давления/утечки воды
    ZB_PACK_WLOG,                           //журнальные данные расхода/давления/утечки воды
    ZB_PACK_VALVE,                          //состояние электроприводов
    ZB_PACK_LEAKS,                          //состояние датчикоы утечки воды
    //входящие запросы  
    ZB_PACK_SYNC_DTIME,                     //синхронизация даты/времени
    ZB_PACK_REQ_STATE,                      //запрос текущего состояния контроллера
    ZB_PACK_REQ_VALVE,                      //состояние электроприводов подачи воды
    ZB_PACK_REQ_DATA,                       //запрос журнальных/текущих данных расхода/давления/утечки воды
    ZB_PACK_CTRL_VALVE,                     //управление электроприводами подачи воды
    ZB_PACK_ACK                             //подтверждение получение пакета с журнальными данными
 } ZBTypePack;

#pragma pack( push, 1 )

//Передача по CAN шине, данные передаются по запросу
//структура для передачи данных счетчиков расхода и давления воды, состояния датчиков утечки
typedef struct {
    ValveStat       stat_valve  : 2;        //статус электропривода
    ValveError      error_valve : 2;        //код ошибки электропривода
    LeakStat        leak1 : 1;              //состояние датчика утечки #1
    LeakStat        leak2 : 1;              //состояние датчика утечки #2
    TypeData        type_data : 1;          //тип данных текущие/суточные
    DC12VStat       dc12_chk : 1;           //контроль напряжения 12VDc для питания датчиков утечки
    uint32_t        count;                  //значение расхода воды (литры)
    uint16_t        pressure;               //значение давления воды (атм)
 } DATA_COUNT;

//Передача по CAN шине, в случае срабатывания датчиков утечки
//структура для передачи данных состояния датчиков утечки и состояния электроприводов 
typedef struct {
    VALVE_STAT_ERR  valve_stat;             //состояния электроприводов
    LeakStat        leak1 : 1;              //состояние датчика утечки #1
    LeakStat        leak2 : 1;              //состояние датчика утечки #2
    unsigned        reserv : 5;             //выравнивание до 1 байта
    DC12VStat       dc12_chk : 1;           //контроль напряжения 12VDc для питания датчиков утечки
 } DATA_LEAK;

//Структура данных для запроса архивных событий
typedef struct {
    uint8_t         day;                    //день
    uint8_t         month;                  //месяц
    uint16_t        year;                   //год
} LOG_REQ;

//Структура для передачи по MODBUS значений дата-время внутренних часов
typedef struct {
    uint8_t         month;                  //месяц
    uint8_t         day;                    //день
    uint16_t        year;                   //год
    uint8_t         min;                    //минуты
    uint8_t         hour;                   //часы
 } MBUS_DTIME;

//Передача по CAN шине, информация события: расход и давления воды,
//состояние электропривода, для холодной и горячей воды
typedef struct {
    ValveStat       stat_valve  : 2;        //статус электропривода
    ValveError      error_valve : 2;        //код ошибки электропривода
    unsigned        reserv : 6;             //выравнивание до 1 байта
    uint32_t        count;                  //значение расхода воды (литры)
    uint16_t        pressure;               //значение давления воды (атм)
 } CAN_DATA1;

//Передача по CAN шине, информация события: расход питьевой воды, состояние датчиков утечки
typedef struct {
    LeakStat        leak1 : 1;              //состояние датчика утечки #1
    LeakStat        leak2 : 1;              //состояние датчика утечки #2
    unsigned        reserv : 5;             //выравнивание до 1 байта
    DC12VStat       dc12_chk : 1;           //контроль напряжения 12VDc для питания датчиков утечки
    uint32_t        count;                  //значение расхода воды (литры)
 } DATA_LOG3;

//*************************************************************************************************
// Исходящие пакеты для радио модуля
//*************************************************************************************************

//Текущая дата/время, источник сброса, даты/время включения контроллера
typedef struct {
    ZBTypePack      type_pack;              //тип пакета
    uint16_t        dev_numb;               //номер уст-ва в сети
    uint16_t        addr_dev;               //адрес уст-ва в сети
    DATE_TIME       rtc;                    //текущее даты/время
    DATE_TIME       start;                  //дата/время включения контроллера
    uint8_t         res_src;                //источник сброса
    uint16_t        crc;                    //контрольная сумма
 } PACK_STATE;

//Текущие/журнальные данные расхода/давления/утечки воды/состояния электроприводов
typedef struct {
    ZBTypePack      type_pack;              //тип пакета
    uint16_t        dev_numb;               //номер уст-ва в сети
    uint16_t        addr_dev;               //адрес уст-ва в сети
    DATE_TIME       date_time;              //дата/время события
    uint32_t        count_cold;             //значения счетчика холодной воды
    uint32_t        count_hot;              //значения счетчика горячей воды
    uint32_t        count_filter;           //значения счетчика питьевой воды
    uint16_t        pressr_cold;            //давление холодной воды
    uint16_t        pressr_hot;             //давление горячей воды
    LeakStat        leak1 : 1;              //состояние датчика утечки #1
    LeakStat        leak2 : 1;              //состояние датчика утечки #2
    unsigned        reserv : 4;             //выравнивание до 1 байта
    EventType       type_event : 1;         //признак данных: данные/событие
    DC12VStat       dc12_chk : 1;           //контроль напряжения 12VDc для питания датчиков утечки
    VALVE_STAT_ERR  valve_stat;             //состояния электроприводов
    uint16_t        crc;                    //контрольная сумма
 } PACK_DATA;

//Состояния электроприводов
typedef struct {
    ZBTypePack      type_pack;              //тип пакета
    uint16_t        numb_dev;               //номер уст-ва в сети
    uint16_t        addr_dev;               //адрес уст-ва в сети
    VALVE_STAT_ERR  valve_stat;             //состояния электроприводов
    uint16_t        crc;                    //контрольная сумма
 } PACK_VALVE;

//Состояния датчиков утечки
typedef struct {
    ZBTypePack      type_pack;              //тип пакета
    uint16_t        numb_dev;               //номер уст-ва в сети
    uint16_t        addr_dev;               //адрес уст-ва в сети
    LeakStat        leak1 : 1;              //состояние датчика утечки #1
    LeakStat        leak2 : 1;              //состояние датчика утечки #2
    unsigned        reserv : 5;             //выравнивание до 1 байта
    DC12VStat       dc12_chk : 1;           //контроль напряжения 12VDc для питания датчиков утечки
    uint16_t        crc;                    //контрольная сумма
 } PACK_LEAKS;

//*************************************************************************************************
// Входящие пакеты от радио модуля
// для корректного значения net_addr необходимо выполнить перестановку байт: __REVSH( net_addr )
//*************************************************************************************************

//Cинхронизация даты/времени
typedef struct {
    ZBTypePack      type_pack;              //тип пакета
    DATE_TIME       date_time;              //дата/время
    uint16_t        crc;                    //контрольная сумма
 } ZB_PACK_RTC; 

//Запрос данных ZB_PACK_REQ_STATE, ZB_PACK_REQ_DATA
typedef struct {
    ZBTypePack      type_pack;              //тип пакета
    uint16_t        dev_numb;               //номер уст-ва
    uint16_t        dev_addr;               //адрес уст-ва в сети
    uint8_t         count_log;              //кол-во записей из журнала
    uint16_t        crc;                    //контрольная сумма
    uint16_t        gate_addr;              //адрес отправителя
 } ZB_PACK_REQ;

//Команды управления электроприводами
typedef struct {
    ZBTypePack      type_pack;              //тип пакета
    uint16_t        dev_numb;               //номер уст-ва в сети
    uint16_t        dev_addr;               //адрес уст-ва в сети
    ValveCtrlMode   cold;                   //команда управления электропривода горячей воды
    ValveCtrlMode   hot;                    //команды управления электропривода холодной воды
    uint16_t        crc;                    //контрольная сумма
    uint16_t        gate_addr;              //адрес отправителя
 } ZB_PACK_CTRL;

//Подтверждение получение данных PACK_DATA 
typedef struct {
    ZBTypePack      type_pack;              //тип пакета
    uint16_t        dev_numb;               //номер уст-ва в сети
    uint16_t        dev_addr;               //адрес уст-ва в сети
    uint16_t        crc;                    //контрольная сумма
    uint16_t        gate_addr;              //адрес отправителя
 } ZB_PACK_ACK_DATA;

#pragma pack( pop )

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
uint8_t *GetDataCan1( uint8_t *size );
uint8_t *GetDataCan2( DataType type, uint8_t *size );
uint8_t *GetDataMbus( uint16_t reg_id, uint16_t reg_cnt, uint8_t *bytes );
uint8_t *GetDataLog( DataType type, WATER_LOG *wtr_log, uint8_t *size );

DATE_TIME *GetAddrDtime( void );
uint8_t *CreatePack( ZBTypePack type, uint8_t *len, uint16_t addr );
ErrorStatus CheckPack( uint8_t *data, uint8_t len );

#endif 
