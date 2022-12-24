
//*************************************************************************************************
//
// Формирование данных для передачи по CAN/MODBUS/ZIGBEE
// 
//*************************************************************************************************

#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "cmsis_os2.h"

#include "data.h"
#include "water.h"
#include "fram.h"
#include "sort.h"
#include "events.h"
#include "config.h"
#include "crc16.h"
#include "xtime.h"
#include "zigbee.h"
#include "modbus_reg.h"

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern CONFIG config;
extern VALVE valve_data;
extern CURR_DATA curr_data;
extern uint16_t pressure_cold, pressure_hot;
extern ZB_CONFIG zb_cfg;

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static char str[60];
static uint8_t cnt_last;

static DATE_TIME    data_rtc;
static MBUS_DTIME   mbus_dtime;
static DATA_LEAK    data_leak;
static DATA_COUNT   data_cold, data_hot, data_filter;
static uint8_t      data_modbus[26];

static PACK_STATE   pack_state;
static PACK_DATA    pack_data;
static PACK_VALVE   pack_valve;
static PACK_LEAKS   pack_leaks;

static ZB_PACK_RTC  zb_pack_rtc;
static ZB_PACK_REQ  zb_pack_req;
static ZB_PACK_CTRL zb_pack_ctrl;

static WATER_LOG    wtr_log;

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static uint8_t CreateData( uint8_t cnt_log );

//*************************************************************************************************
// Возвращает указатель на структуру DATA_LEAK - состоянию датчиков утечки и 
// электроприводов для передачи по CAN шине
//-------------------------------------------------------------------------------------------------
// uint8_t *size - возвращает размер данных в структуре
// return        - указатель на структуру DATA_LEAK
//*************************************************************************************************
uint8_t *GetDataCan1( uint8_t *size ) {

    *size = sizeof( data_leak );
    memset( (uint8_t *)&data_leak, 0x00, sizeof( data_leak ) );
    data_leak.stat_valve_cold = ValveGetStatus( VALVE_COLD );
    data_leak.error_valve_cold = valve_data.error_cold;
    data_leak.stat_valve_hot = ValveGetStatus( VALVE_HOT );
    data_leak.error_valve_hot = valve_data.error_cold;
    data_leak.leak1 = LeakStatus( LEAK1 );
    data_leak.leak2 = LeakStatus( LEAK2 );
    data_leak.dc12_chk = DC12VStatus();
    return (uint8_t *)&data_leak;
 }

//*************************************************************************************************
// Возвращает указатель на структуру DATA_COUNT - текущие данные по расходу воды, состояние 
// датчиков утечки, состояние электроприводов для передачи данных по CAN шине
//-------------------------------------------------------------------------------------------------
// DataType type - тип показаний: холодная/горячая/питьевая вода
// uint8_t *size - возвращает размер данных в структуре DATA_COUNT
// return        - указатель на структуру DATA_COUNT
//*************************************************************************************************
uint8_t *GetDataCan2( DataType type, uint8_t *size ) {

    if ( type == DATA_COLD ) {
        //данные по холодной воде
        *size = sizeof( data_cold );
        memset( (uint8_t *)&data_cold, 0x00, sizeof( data_cold ) );
        data_cold.stat_valve = ValveGetStatus( VALVE_COLD );
        data_cold.error_valve = valve_data.error_cold;
        data_cold.leak1 = LeakStatus( LEAK1 );
        data_cold.leak2 = LeakStatus( LEAK2 );
        data_cold.dc12_chk = DC12VStatus();
        data_cold.count = curr_data.count_cold;
        data_cold.pressure = pressure_cold;
        data_cold.type_data = CURRENT_DATA;
        return (uint8_t *)&data_cold;
       }
    if ( type == DATA_HOT ) {
        //данные по горячей воде
        *size = sizeof( data_hot );
        memset( (uint8_t *)&data_hot, 0x00, sizeof( data_hot ) );
        data_hot.stat_valve = ValveGetStatus( VALVE_HOT );
        data_hot.error_valve = valve_data.error_hot;
        data_hot.leak1 = LeakStatus( LEAK1 );
        data_hot.leak2 = LeakStatus( LEAK2 );
        data_hot.dc12_chk = DC12VStatus();
        data_hot.count = curr_data.count_hot;
        data_hot.pressure = pressure_hot;
        data_hot.type_data = CURRENT_DATA;
        return (uint8_t *)&data_hot;
       }
    if ( type == DATA_FILTER ) {
        //данные по питьевой воде
        *size = sizeof( data_filter );
        memset( (uint8_t *)&data_filter, 0x00, sizeof( data_filter ) );
        data_filter.stat_valve = ValveGetStatus( VALVE_COLD );
        data_filter.error_valve = valve_data.error_cold;
        data_filter.leak1 = LeakStatus( LEAK1 );
        data_filter.leak2 = LeakStatus( LEAK2 );
        data_filter.dc12_chk = DC12VStatus();
        data_filter.count = curr_data.count_filter;
        data_filter.pressure = pressure_cold;
        data_filter.type_data = CURRENT_DATA;
        return (uint8_t *)&data_filter;
       }
    if ( type == DATA_DATE ) {
        //данные часов и календаря
        *size = sizeof( data_rtc );
        memset( (uint8_t *)&data_rtc, 0x00, sizeof( data_rtc ) );
        GetTimeDate( &data_rtc );
        return (uint8_t *)&data_rtc;
       }
    *size = 0;
    return NULL;
 }

//*************************************************************************************************
// Возвращает указатель на структуру DATA_COUNT - данные по расходу воды, состояние датчиков 
// утечки, состояние электроприводов, за указанную дату события, для передачи данных по CAN шине 
//-------------------------------------------------------------------------------------------------
// DataType type      - тип показаний: холодная/горячая/питьевая вода 
// WATER_LOG *wtr_log - указатель на структуру с журнальными данными
// uint8_t *size      - возвращает размер данных в структуре DATA_COUNT
// return != NULL     - указатель на структуру DATA_COUNT с данными в зависимости от параметра DataType
//        == NULL     - Запрашиваемые данные не сформированы
//*************************************************************************************************
uint8_t *GetDataLog( DataType type, WATER_LOG *wtr_log, uint8_t *size ) {

    if ( type == DATA_LOG_COLD ) {
        //данные по холодной воды
        *size = sizeof( data_cold );
        memset( (uint8_t *)&data_cold, 0x00, sizeof( data_cold ) );
        data_cold.stat_valve = wtr_log->stat_valve_cold;
        data_cold.error_valve = wtr_log->error_valve_cold;
        data_cold.leak1 = wtr_log->leak1;
        data_cold.leak2 = wtr_log->leak2;
        data_cold.dc12_chk = wtr_log->dc12_chk;
        data_cold.count = wtr_log->count_cold;
        data_cold.pressure = wtr_log->pressr_cold;
        data_cold.type_data = WATER_DATA_LOG;
        return (uint8_t *)&data_cold;
       }
    if ( type == DATA_LOG_HOT ) {
        //данные по горячей воде
        *size = sizeof( data_hot );
        memset( (uint8_t *)&data_hot, 0x00, sizeof( data_hot ) );
        data_hot.stat_valve = wtr_log->stat_valve_hot;
        data_hot.error_valve = wtr_log->error_valve_hot;
        data_hot.leak1 = wtr_log->leak1;
        data_hot.leak2 = wtr_log->leak2;
        data_hot.dc12_chk = wtr_log->dc12_chk;
        data_hot.count = wtr_log->count_hot;
        data_hot.pressure = wtr_log->pressr_hot;
        data_hot.type_data = WATER_DATA_LOG;
        return (uint8_t *)&data_hot;
       }
    if ( type == DATA_LOG_FILTER ) {
        //данные по питьевой воде
        *size = sizeof( data_filter );
        memset( (uint8_t *)&data_filter, 0x00, sizeof( data_filter ) );
        data_filter.stat_valve = wtr_log->stat_valve_cold;
        data_filter.error_valve = wtr_log->error_valve_cold;
        data_filter.leak1 = wtr_log->leak1;
        data_filter.leak2 = wtr_log->leak2;
        data_filter.dc12_chk = wtr_log->dc12_chk;
        data_filter.count = wtr_log->count_filter;
        data_filter.pressure = wtr_log->pressr_cold;
        data_filter.type_data = WATER_DATA_LOG;
        return (uint8_t *)&data_filter;
       }
    *size = 0;
    return NULL;
 }

//*************************************************************************************************
// Функция формирует набор данных в зависимости от номера регистра для передачи по MODBUS
//-------------------------------------------------------------------------------------------------
// uint16_t reg_id  - адрес регистра
// uint16_t reg_cnt - кол-во регистров
// uint8_t *bytes   - указатель на переменную в которой размещается кол-во сформированных байт данных
// return           - указатель на блок данных
//*************************************************************************************************
uint8_t *GetDataMbus( uint16_t reg_id, uint16_t reg_cnt, uint8_t *bytes ) {

    uint16_t data16;
    uint32_t data32;
    uint8_t cnt_byte = 0;
    
    if ( reg_cnt && reg_id == MBUS_REG_CTRL ) {
        //состояния датчиков утечки и состояния электроприводов
        data_leak.stat_valve_cold = ValveGetStatus( VALVE_COLD );
        data_leak.error_valve_cold = valve_data.error_cold;
        data_leak.stat_valve_hot = ValveGetStatus( VALVE_HOT );
        data_leak.error_valve_hot = valve_data.error_cold;
        data_leak.leak1 = LeakStatus( LEAK1 );
        data_leak.leak2 = LeakStatus( LEAK2 );
        data_leak.dc12_chk = DC12VStatus();
        memcpy( data_modbus, (uint8_t *)&data_leak, sizeof( data_leak ) );
        cnt_byte += sizeof( data_leak );
        //переход на следующий регистр
        if ( reg_cnt ) {
            reg_cnt--;
            reg_id += 1;
           }
       }
    if ( reg_cnt && reg_id == MBUS_REG_WTR_COLD ) {
        //данные по холодной воде
        data32 = curr_data.count_cold;
        memcpy( data_modbus + cnt_byte, (uint8_t *)&data32, sizeof( data32 ) );
        cnt_byte += sizeof( data32 );
        //переход на следующий регистр
        if ( reg_cnt ) {
            reg_cnt -= 2;
            reg_id += 2;
           }
       }
    if ( reg_cnt && reg_id == MBUS_REG_COLD_PRESR ) {
        //давление холодной выды
        data16 = pressure_cold;
        memcpy( data_modbus + cnt_byte, (uint8_t *)&data16, sizeof( data16 ) );
        cnt_byte += sizeof( data16 );
        //переход на следующий регистр
        if ( reg_cnt ) {
            reg_cnt--;
            reg_id += 1;
           }
       }
    if ( reg_cnt && reg_id == MBUS_REG_WTR_HOT ) {
        //данные по горячей воде
        data32 = curr_data.count_hot;
        memcpy( data_modbus + cnt_byte, (uint8_t *)&data32, sizeof( data32 ) );
        cnt_byte += sizeof( data32 );
        //переход на следующий регистр
        if ( reg_cnt ) {
            reg_cnt -= 2;
            reg_id += 2;
           }
       }
    if ( reg_cnt && reg_id == MBUS_REG_HOT_PRESR ) {
        //давление горячей воды
        data16 = pressure_hot;
        memcpy( data_modbus + cnt_byte, (uint8_t *)&data16, sizeof( data16 ) );
        cnt_byte += sizeof( data16 );
        //переход на следующий регистр
        if ( reg_cnt ) {
            reg_cnt--;
            reg_id += 1;
           }
       }
    if ( reg_cnt && reg_id == MBUS_REG_WTR_FILTER ) {
        //данные по питьевой воде
        data32 = curr_data.count_filter;
        memcpy( data_modbus + cnt_byte, (uint8_t *)&data32, sizeof( data32 ) );
        cnt_byte += sizeof( data32 );
        //переход на следующий регистр
        if ( reg_cnt ) {
            reg_cnt -= 2;
            reg_id += 2;
           }
       }
    if ( reg_cnt && reg_id == MBUS_REG_DAYMON ) {
        //дата время
        GetTimeDate( &data_rtc );
        mbus_dtime.day = data_rtc.day;
        mbus_dtime.month = data_rtc.month;
        mbus_dtime.year = data_rtc.year;
        mbus_dtime.hour = data_rtc.hour;
        mbus_dtime.min = data_rtc.min;
        memcpy( data_modbus + cnt_byte, (uint8_t *)&mbus_dtime, sizeof( mbus_dtime ) );
        cnt_byte += sizeof( mbus_dtime );
       }
    *bytes = cnt_byte;
    return data_modbus;
 }

//*************************************************************************************************
// Формирует пакет данных для отправки по ZigBee
//-------------------------------------------------------------------------------------------------
// ZBTypePack type - тип формируемого пакета
// uint8_t *len    - указатель на переменную для размещения размера сформированного пакета
// return == NULL  - тип пакета не указан
//        != NULL  - указатель на пакет данных
//*************************************************************************************************
uint8_t *CreatePack( ZBTypePack type, uint8_t *len ) {

    *len = 0;
    if ( type == ZB_PACK_STATE ) {
        //данные состояния контроллера
        pack_state.type_pack = ZB_PACK_STATE;                               //тип пакета
        pack_state.dev_numb = config.dev_numb;                              //номер уст-ва
        pack_state.addr_dev = __REVSH( *((uint16_t *)&zb_cfg.short_addr) ); //адрес уст-ва в сети
        GetTimeDate( &pack_state.rtc );     
        //даты/время включения контроллера      
        pack_state.start.day = curr_data.day;                               //день
        pack_state.start.month = curr_data.month;                           //месяц
        pack_state.start.year = curr_data.year;                             //год
        pack_state.start.hour = curr_data.hour;                             //часы
        pack_state.start.min = curr_data.min;                               //минуты
        pack_state.start.sec = curr_data.sec;                               //секунды
        pack_state.res_src = curr_data.res_src;                             //источник сброса
        //контрольная сумма
        pack_state.crc = CalcCRC16( (uint8_t *)&pack_state, sizeof( pack_state ) - sizeof( pack_state.crc ) );
        *len = sizeof( pack_state );
        return (uint8_t *)&pack_state;
       }
    if ( type == ZB_PACK_DATA ) {
        //текущие данные расхода/давления/утечки воды
        pack_data.type_pack = type;                                         //тип пакета
        pack_data.dev_numb = config.dev_numb;                               //номер уст-ва
        pack_data.addr_dev = __REVSH( *((uint16_t *)&zb_cfg.short_addr) );  //адрес уст-ва в сети
        GetTimeDate( &pack_data.date_time );
        //значения учета
        pack_data.count_cold = curr_data.count_cold;                        //значения счетчика холодной воды
        pack_data.count_hot = curr_data.count_hot;                          //значения счетчика горячей воды
        pack_data.count_filter = curr_data.count_filter;                    //значения счетчика питьевой воды
        pack_data.pressr_cold = pressure_cold;                              //давление холодной воды
        pack_data.pressr_hot = pressure_hot;                                //давление горячей воды
        //состояние датчиков учета
        pack_data.leak1 = LeakStatus( LEAK1 );                              //состояние датчика утечки #1
        pack_data.leak2 = LeakStatus( LEAK2 );                              //состояние датчика утечки #2
        pack_data.reserv = 0;                                               //резерв
        pack_data.type_event = EVENT_DATA;                                  //признак данных: данные/событие
        pack_data.dc12_chk = DC12VStatus();                                 //контроль напряжения 12VDc для питания датчиков утечки
        //состояния электроприводов кранов      
        pack_data.stat_valve_cold = ValveGetStatus( VALVE_COLD );           //статус крана холодный воды    
        pack_data.error_valve_cold = valve_data.error_cold;                 //код ошибки крана холодный воды
        pack_data.stat_valve_hot = ValveGetStatus( VALVE_HOT );             //статус крана горячей воды     
        pack_data.error_valve_hot = valve_data.error_hot;                   //код ошибки крана горячей воды 
        //контрольная сумма
        pack_data.crc = CalcCRC16( (uint8_t *)&pack_data, sizeof( pack_data ) - sizeof( pack_state.crc ) );
        *len = sizeof( pack_data );
        return (uint8_t *)&pack_data;
       }
    if ( type == ZB_PACK_WLOG ) {
        //журнальные данные расхода/давления/утечки воды
        pack_data.type_pack = type;                                         //тип пакета
        pack_data.dev_numb = config.dev_numb;                               //номер уст-ва
        pack_data.addr_dev = __REVSH( *((uint16_t *)&zb_cfg.short_addr) );  //адрес уст-ва в сети
        //дата время события
        pack_data.date_time.sec = wtr_log.sec;                              //секунды
        pack_data.date_time.min = wtr_log.min;                              //минуты
        pack_data.date_time.hour = wtr_log.hour;                            //часы
        pack_data.date_time.day = wtr_log.day;                              //день
        pack_data.date_time.month = wtr_log.month;                          //месяц
        pack_data.date_time.year = wtr_log.year;                            //год
        //значения учета        
        pack_data.count_cold = wtr_log.count_cold;                          //значения счетчика холодной воды
        pack_data.count_hot = wtr_log.count_hot;                            //значения счетчика горячей воды
        pack_data.count_filter = wtr_log.count_filter;                      //значения счетчика питьевой воды
        pack_data.pressr_cold = wtr_log.pressr_cold;                        //давление холодной воды
        pack_data.pressr_hot = wtr_log.pressr_hot;                          //давление горячей воды
        //состояние датчиков учета
        pack_data.leak1 = wtr_log.leak1;                                    //состояние датчика утечки #1
        pack_data.leak2 = wtr_log.leak2;                                    //состояние датчика утечки #2
        pack_data.reserv = 0;                                               //резерв
        pack_data.type_event = wtr_log.type_event;                          //признак данных: данные/событие
        pack_data.dc12_chk = wtr_log.dc12_chk;                              //контроль напряжения 12VDc для питания датчиков утечки
        //состояния электроприводов      
        pack_data.stat_valve_cold = wtr_log.stat_valve_cold;                //статус крана холодный воды    
        pack_data.error_valve_cold = wtr_log.error_valve_cold;              //код ошибки крана холодный воды
        pack_data.stat_valve_hot = wtr_log.stat_valve_hot;                  //статус крана горячей воды     
        pack_data.error_valve_hot = wtr_log.error_valve_hot;                //код ошибки крана горячей воды 
        //контрольная сумма
        pack_data.crc = CalcCRC16( (uint8_t *)&pack_data, sizeof( pack_data ) - sizeof( pack_state.crc ) );
        *len = sizeof( pack_data );
        return (uint8_t *)&pack_data;
       }
    if ( type == ZB_PACK_VALVE ) {
        //состояние электроприводов управления кранами подачи воды
        pack_valve.type_pack = type;                                        //тип пакета
        pack_valve.numb_dev = config.dev_numb;                              //номер уст-ва
        pack_valve.addr_dev = __REVSH( *((uint16_t *)&zb_cfg.short_addr) ); //адрес уст-ва в сети
        //состояния электроприводов      
        pack_data.stat_valve_cold = ValveGetStatus( VALVE_COLD );           //статус крана холодный воды    
        pack_data.error_valve_cold = valve_data.error_cold;                 //код ошибки крана холодный воды
        pack_data.stat_valve_hot = ValveGetStatus( VALVE_HOT );             //статус крана горячей воды     
        pack_data.error_valve_hot = valve_data.error_hot;                   //код ошибки крана горячей воды 
        //контрольная сумма
        pack_valve.crc = CalcCRC16( (uint8_t *)&pack_valve, sizeof( pack_valve ) - sizeof( pack_valve.crc ) );
        *len = sizeof( pack_valve );
        return (uint8_t *)&pack_valve;
       }
    if ( type == ZB_PACK_LEAKS ) {
        //состояние датчиков утечки воды
        pack_leaks.type_pack = type;                                        //тип пакета
        pack_leaks.numb_dev = config.dev_numb;                              //номер уст-ва
        pack_leaks.addr_dev = __REVSH( *((uint16_t *)&zb_cfg.short_addr) ); //адрес уст-ва в сети
        //состояния электроприводов      
        pack_leaks.leak1 = LeakStatus( LEAK1 );                             //состояние датчика утечки #1
        pack_leaks.leak2 = LeakStatus( LEAK2 );                             //состояние датчика утечки #2
        pack_leaks.reserv = 0;                                              //резерв
        pack_leaks.dc12_chk = DC12VStatus();                                //контроль напряжения 12VDc для питания датчиков утечки
        //контрольная сумма
        pack_leaks.crc = CalcCRC16( (uint8_t *)&pack_leaks, sizeof( pack_leaks ) - sizeof( pack_leaks.crc ) );
        *len = sizeof( pack_leaks );
        return (uint8_t *)&pack_leaks;
       }
    return NULL;
 }

//*************************************************************************************************
// Проверка принятого пакета на соответствие: типа пакета/размера/контрольная сумма
//-------------------------------------------------------------------------------------------------
// uint8_t *data    - указатель на буфер принятого пакета
// uint8_t len      - размер принятого пакета
// return = SUCCESS - пакет данных идентифицирован и проверен
//        = ERROR   - пакет данных не идентифицирован
//*************************************************************************************************
ErrorStatus CheckPack( uint8_t *data, uint8_t len ) {

    char *ptr;
    uint16_t crc, addr_dev;
    ZBTypePack type;
    
    type = (ZBTypePack)*data;
    osEventFlagsSet( led_event, EVN_LED_ZB_ACTIVE );
    addr_dev = __REVSH( *((uint16_t *)&zb_cfg.short_addr) );
    if ( ( type == ZB_PACK_REQ_STATE || type == ZB_PACK_REQ_DATA ) && len == sizeof( ZB_PACK_REQ ) ) {
        //запрос текущего состояния контроллера
        //запрос текущих данных расхода/давления/утечки воды
        //запрос журнальных данных расхода/давления/утечки воды
        memcpy( (uint8_t *)&zb_pack_req, data, sizeof( ZB_PACK_REQ ) );
        //КС считаем без полученной КС и net_addr (gate_addr не входит в подсчет КС)
        crc = CalcCRC16( (uint8_t *)&zb_pack_req, sizeof( zb_pack_req ) - ( sizeof( uint16_t ) * 2 ) );
        if ( zb_pack_req.crc != crc ) {
            ZBIncError( ZB_ERROR_CRC );
            return ERROR;
           }
        if ( zb_pack_req.dev_numb != config.dev_numb ) {
            ZBIncError( ZB_ERROR_NUMB );
            return ERROR;
           }
        if ( zb_pack_req.dev_addr != addr_dev ) {
            ZBIncError( ZB_ERROR_NUMB );
            return ERROR;
           }
        if ( type == ZB_PACK_REQ_STATE )
            osEventFlagsSet( zb_ctrl, EVN_ZC_SEND_STATE ); //текущего состояния контроллера
        if ( type == ZB_PACK_REQ_DATA && !zb_pack_req.count_log )
            osEventFlagsSet( zb_ctrl, EVN_ZC_SEND_DATA ); //текущих данных расхода/давления/утечки воды
        if ( type == ZB_PACK_REQ_DATA && zb_pack_req.count_log ) {
            if ( CreateData( zb_pack_req.count_log ) )
                osEventFlagsSet( zb_ctrl, EVN_ZC_SEND_WLOG ); //журнальные данные расхода/давления/утечки воды
           }
        return SUCCESS;
       }
    if ( type == ZB_PACK_SYNC_DTIME && len == sizeof( ZB_PACK_RTC ) ) {
        //синхронизация даты/времени
        memcpy( (uint8_t *)&zb_pack_rtc, data, sizeof( ZB_PACK_RTC ) );
        //КС считаем без полученной КС и net_addr (gate_addr не входит в подсчет КС)
        crc = CalcCRC16( (uint8_t *)&zb_pack_rtc, sizeof( zb_pack_rtc ) - ( sizeof( uint16_t ) ) );
        if ( zb_pack_rtc.crc != crc ) {
            ZBIncError( ZB_ERROR_CRC );
            return ERROR;
           }
        osEventFlagsSet( zb_ctrl, EVN_ZC_SYNC_DTIME );
        return SUCCESS;
       }
    if ( type == ZB_PACK_CTRL_VALVE && len == sizeof( ZB_PACK_CTRL ) ) {
        //управление кранами подачи воды
        memcpy( (uint8_t *)&zb_pack_ctrl, data, sizeof( ZB_PACK_CTRL ) );
        //КС считаем без полученной КС и net_addr (gate_addr не входит в подсчет КС)
        crc = CalcCRC16( (uint8_t *)&zb_pack_ctrl, sizeof( zb_pack_ctrl ) - ( sizeof( uint16_t ) * 2 ) );
        if ( zb_pack_ctrl.crc != crc ) {
            ZBIncError( ZB_ERROR_CRC );
            return ERROR;
           }
        if ( zb_pack_ctrl.dev_numb != config.dev_numb ) {
            ZBIncError( ZB_ERROR_NUMB );
            return ERROR;
           }
        if ( zb_pack_ctrl.dev_addr != addr_dev ) {
            ZBIncError( ZB_ERROR_NUMB );
            return ERROR;
           }
        //управление электроприводами, отправка подтверждения выполняется из 
        //задачи TaskValve() после завершения работы электроприводов
        if ( zb_pack_ctrl.cold == VALVE_CTRL_OPEN )
            osEventFlagsSet( valve_event, EVN_VALVE_COLD_OPN );
        if ( zb_pack_ctrl.cold == VALVE_CTRL_CLOSE )
            osEventFlagsSet( valve_event, EVN_VALVE_COLD_CLS );
        if ( zb_pack_ctrl.hot == VALVE_CTRL_OPEN )
            osEventFlagsSet( valve_event, EVN_VALVE_HOT_OPN );
        if ( zb_pack_ctrl.hot == VALVE_CTRL_CLOSE )
            osEventFlagsSet( valve_event, EVN_VALVE_HOT_CLS );
        if ( zb_pack_ctrl.cold || zb_pack_ctrl.hot ) {
            ptr = str;
            ptr += sprintf( ptr, "Valve control " );
            ptr += sprintf( ptr, "COLD = %s ", ValveCtrlDesc( zb_pack_ctrl.cold ) );
            ptr += sprintf( ptr, "HOT = %s\r\n", ValveCtrlDesc( zb_pack_ctrl.hot ) );
            UartSendStr( str );
           }
        return SUCCESS;
       }
    return ERROR;
 }

//*************************************************************************************************
// Функция возвращает указатель на структуру DATE_TIME содержащую дату/время 
// от координатора для синхронизации внутренних часов
// После первого чтения обнуляется поле zb_pack_rtc.crc как признак устаревания данных 
//-------------------------------------------------------------------------------------------------
// return - указатель на структуру DATE_TIME
// return = NULL - данные уже прочитаны
//*************************************************************************************************
DATE_TIME *GetAddrDtime( void ) {

    if ( zb_pack_rtc.crc ) {
        zb_pack_rtc.crc = 0;
        return &zb_pack_rtc.date_time;
       }
    else return NULL;
 }
 
//*************************************************************************************************
// Подготовка данных из журнала событий (построение таблицы индексов по убыванию дата-время события)
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_log - кол-во записей запрашиваемых из журнала
//*************************************************************************************************
static uint8_t CreateData( uint8_t cnt_log ) {

    uint16_t addr, rec, cnt;

    cnt = MakeSort();
    if ( !cnt ) {
        UartSendStr( "Records not found.\r\n" );
        return 0;
       }
    if ( cnt_log > cnt )
        return 0; //нельзя запрашивать записей больше чем есть в журнале
    sprintf( str, "Records uploaded: %u\r\n", cnt );
    UartSendStr( str );
    cnt_last = cnt_log; //кол-во запрашиваемых записей
    for ( rec = 0; rec < cnt && cnt_log; rec++ ) {
        //выборка адреса данных из FRAM по индексу
        addr = GetAddrSort( rec );
        if ( !addr )
            continue;
        //чтение данных из FRAM
        memset( (uint8_t *)&wtr_log, 0x00, sizeof( wtr_log ) );
        if ( FramReadData( addr, (uint8_t *)&wtr_log, sizeof( wtr_log ) ) != FRAM_OK )
            continue;
        cnt_log--;
        sprintf( str, "LOG: %s #%02u (0x%04X): %02u.%02u.%04u %02u:%02u:%02u\r\n", 
                 wtr_log.type_event == EVENT_DATA ? "Event" : "ALARM", rec+1, addr, 
                 wtr_log.day, wtr_log.month, wtr_log.year, wtr_log.hour, wtr_log.min, wtr_log.sec );
        UartSendStr( str );
       }
    return cnt;
 }

//*************************************************************************************************
// Функция заполняет структуру WATER_LOG данными из журнала событий по указанному индексу.
// Данные необходимо предварительно подготовить (отсортировать) через вызов CreateData()
//-------------------------------------------------------------------------------------------------
// uint8_t rec      - номер записи (индекса) в журнале 0 ... N для получения данных
// return = SUCCESS - данные доступны в wtr_log
//          ERROR   - все данные уже прочитаны
//*************************************************************************************************
ErrorStatus GetAddrLog( uint8_t rec ) {

    uint16_t addr;

    if ( !cnt_last )
        return ERROR; //все данные уже прочитаны
    //адрес данных в журнале
    addr = GetAddrSort( rec );
    //чтение данных из FRAM
    memset( (uint8_t *)&wtr_log, 0x00, sizeof( wtr_log ) );
    if ( FramReadData( addr, (uint8_t *)&wtr_log, sizeof( wtr_log ) ) != FRAM_OK ) {
        cnt_last = 0;
        return ERROR;
       }
    cnt_last--;
    return SUCCESS;
 }
