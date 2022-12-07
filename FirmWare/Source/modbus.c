
//*************************************************************************************************
//
// Управление протоколом MODBUS (slave)
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "main.h"
#include "data.h"
#include "rs485.h"
#include "modbus.h"
#include "events.h"
#include "message.h"
#include "parse.h"
#include "crc16.h"
#include "config.h"
#include "xtime.h"

#include "modbus_reg.h"
#include "modbus_def.h"

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern CONFIG config;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart3;

extern const uint8_t func_access[];
extern const RegsRead regs_read[];
extern const RegsRead regs_write[];
extern const ValueValid val_valid[];

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
//расшифровка ошибок протокола MODBUS
static char * const error_descr[] = {
    "OK",                                           //MBUS_REQUEST_OK
    "Function code not supported",                  //MBUS_ERROR_FUNC
    "Data address not available",                   //MBUS_ERROR_ADDR
    "Invalid value in data field",                  //MBUS_ERROR_DATA
    "Unrecoverable error",                          //MBUS_ERROR_DEV
    "It takes time to process the request",         //MBUS_ERROR_ACKWAIT
    "Busy processing command",                      //MBUS_ERROR_BUSY
    "The slave cannot execute the function",        //MBUS_ERROR_NOACK
    "Memory Parity error",                          //MBUS_ERROR_MEMCRC
    "Received packet checksum error",               //MBUS_REQST_CRC
    "The request is addressed to another device",   //MBUS_REQST_NOT_FOR_DEV
    "Errors in function call parameters"            //MBUS_ERROR_PARAM
 };

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
#ifdef DEBUG_MODBUS
static char str[120];
#endif
static uint32_t recv_total, error_cnt[SIZE_ARRAY( error_descr )]; //счетчики ошибок протокола

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static ModbusRequst TypeRequst( uint8_t func_id );
static ErrorStatus ChkFuncValid( uint8_t func_id, const uint8_t *list );
static ErrorStatus ChkRegValid( MBUS_REQ *reqst, const RegsRead *list );
static ErrorStatus ChkRegValue( MBUS_REQ *reqst, const ValueValid *list );
static ErrorStatus RegWrite( MBUS_REQ *reqst );
static ModbusAddrReg ModBusAddr( uint8_t func );

//*************************************************************************************************
// Инициализация протокола
//*************************************************************************************************
void ModBusInit( void ) {

    ModBusErrClr();
 }

//*************************************************************************************************
// Проверка запроса, если запрос проверен данные запроса размещаюся в структуре MBUS_REQ
//-------------------------------------------------------------------------------------------------
// uint8_t *data        - адрес буфера с принятыми данными
// uint8_t len          - размер фрейма (байт)
// MBUS_REQ *request    - укахатель на структуру для размещения данных запроса
// return = ModBusError - результат проверки принятого фрейма
//*************************************************************************************************
ModBusError CheckRequest( uint8_t *data, uint8_t len, MBUS_REQ *request ) {

    ModbusRequst type_req;
    MBUS_REQ_REG mbus_req;
    MBUS_WRT_REGS mbus_wrtn;
    uint8_t func, idx, cnt_word;
    uint16_t crc_calc, crc_data, *ptr_uint16;

    //проверка параметров
    if ( data == NULL || !len )
        return MBUS_ERROR_PARAM;
    //проверим КС всего фрейма
    crc_calc = CalcCRC16( data, len - sizeof( uint16_t ) );
    crc_data = *((uint16_t *)( data + len - sizeof( uint16_t ) ));
    if ( crc_calc != crc_data )
        return MBUS_REQST_CRC; //КС не совпали
    recv_total++;
    //проверка адреса получателя
    if ( *( data + MB_REQUEST_DEV ) != config.modbus_id )
        return MBUS_REQST_NOT_FOR_DEV;
    //проверка доступности функции
    func = *( data + MB_REQUEST_FUNC );
    if ( ChkFuncValid( func, func_access ) == ERROR )
        return MBUS_ERROR_FUNC;
    //определяем тип запроса
    type_req = TypeRequst( func );
    if ( type_req == MBUS_REQST_UNKNOW )
        return MBUS_ERROR_PARAM;
    if ( type_req == MBUS_REQST_READ ) {
        //чтение одного/нескольких регистров
        //заполнение промежуточной структуры MBUS_REQ_REG данными запроса
        memcpy( (uint8_t *)&mbus_req, data, sizeof( mbus_req ) );
        //заполнение общей структуры MBUS_REQ данными запроса
        request->dev_addr = mbus_req.dev_addr;
        request->function = mbus_req.function;
        request->reg_addr = __REVSH( mbus_req.reg_addr );
        request->reg_cnt = __REVSH( mbus_req.reg_cntval );
        request->ptr_data = NULL;
        //проверка первого регистра
        if ( ChkRegValid( request, &regs_read[0] ) == ERROR )
            return MBUS_ERROR_ADDR;
       }
    if ( type_req == MBUS_REQST_WRITE1 ) {
        //запись одного регистра
        memcpy( (uint8_t *)&mbus_req, data, sizeof( mbus_req ) );
        //заполнение общей структуры MBUS_REQ данными запроса
        request->dev_addr = mbus_req.dev_addr;
        request->function = mbus_req.function;
        request->reg_addr = __REVSH( mbus_req.reg_addr );
        request->reg_cnt = 1;
        request->ptr_data = data + MB_REQUEST_DATA1;
        //меняем байты местами для регистра данных
        ptr_uint16 = (uint16_t *)( data + MB_REQUEST_DATA1 );
        *ptr_uint16 = __REVSH( *ptr_uint16 );
        //проверка адреса регистра
        if ( ChkRegValid( request, &regs_write[0] ) == ERROR )
            return MBUS_ERROR_ADDR;
        //проверка значения регистра
        if ( ChkRegValue( request, &val_valid[0] ) == ERROR )
            return MBUS_ERROR_DATA;
        //выполняем запись ...
        if ( RegWrite( request ) == ERROR )
            return MBUS_ERROR_DATA;
       }
    if ( type_req == MBUS_REQST_WRITEN ) {
        //запись в несколько регистров
        memcpy( (uint8_t *)&mbus_wrtn, data, sizeof( mbus_wrtn ) );
        //заполнение общей структуры MBUS_REQ данными запроса
        request->dev_addr = mbus_wrtn.dev_addr;
        request->function = mbus_wrtn.function;
        request->reg_addr = __REVSH( mbus_wrtn.reg_addr );
        request->reg_cnt = __REVSH( mbus_wrtn.reg_cnt );
        request->ptr_data = data + MB_REQUEST_DATAN;
        //проверка необходимости перестановки байт
        if ( ModBusAddr( request->function ) == MBUS_REG_16BIT ) {
            //для 16-битных регистров - перестановка байтов
            cnt_word = mbus_wrtn.byte_cnt / sizeof( uint16_t );
            ptr_uint16 = (uint16_t *)( data + MB_REQUEST_DATAN );
            for ( idx = 0; idx < cnt_word; idx++, ptr_uint16++ )
                *ptr_uint16 = __REVSH( *ptr_uint16 );
           }
        //проверка первого регистра
        if ( ChkRegValid( request, &regs_write[0] ) == ERROR )
            return MBUS_ERROR_ADDR;
        //проверка значения регистра
        if ( ChkRegValue( request, &val_valid[0] ) == ERROR )
            return MBUS_ERROR_DATA;
        //выполняем запись ...
        if ( RegWrite( request ) == ERROR )
            return MBUS_ERROR_DATA;
       }
    #ifdef DEBUG_MODBUS
    sprintf( str, "DEV: 0x%02X FUCT: 0x%02X ADDR: 0x%04X-0x%04X CNT: %d\r\n", request->dev_addr, request->function, request->reg_addr, 
            request->reg_addr + request->reg_cnt - 1, request->reg_cnt );
    UartSendStr( str );
    if ( request->ptr_data != NULL ) {
        UartSendStr( "DATA: " );
        ptr_uint16 = (uint16_t *)request->ptr_data;
        for ( idx = 0; idx < request->reg_cnt; idx++, ptr_uint16++ ) {
            sprintf( str, "0x%04X ", *ptr_uint16 );
            UartSendStr( str );
           }
        UartSendStr( "\r\n" );
       }
    #endif
    return MBUS_REQUEST_OK;
 }

//*************************************************************************************************
// Формирование фрейма ответа протокола MODBUS
//-------------------------------------------------------------------------------------------------
// MBUS_DATA *reqst  - указатель на структуры с параметрами запроса
// ModBusError error - код ошибки проверки запроса
// return            - размер фрейма в байтах для передачи
//*************************************************************************************************
uint8_t CreateFrame( MBUS_REQ *reqst, ModBusError error ) {

    uint16_t *ptr16, crc;
    uint8_t idx, data_len, *pdata, *sdata;
    MBUS_ERROR err_reg;
    MBUS_REQ_REG mbus_req;

    sdata = RS485SendBuff();
    if ( error > MBUS_REQUEST_OK && error < MBUS_REQST_CRC ) {
        //фрейм ответа с ошибкой
        err_reg.dev_addr = reqst->dev_addr;
        err_reg.function = reqst->function | FUNC_ANSWER_ERROR;
        err_reg.error = error;
        err_reg.crc = CalcCRC16( (uint8_t *)&err_reg, sizeof( err_reg ) - sizeof( uint16_t ) );
        ClearSend();
        memcpy( sdata, (uint8_t *)&err_reg, sizeof( err_reg ) );
        return sizeof( err_reg );
       }
    //только чтение регистра(ов)
    if ( TypeRequst( reqst->function ) == MBUS_REQST_READ ) {
        ClearSend();
        *( sdata + MB_ANSWER_DEV ) = reqst->dev_addr;
        *( sdata + MB_ANSWER_FUNC ) = reqst->function;
        //формируем набор данных
        pdata = GetDataMbus( reqst->reg_addr, reqst->reg_cnt, &data_len );
        if ( !data_len )
            return 0;
        *( sdata + MB_ANSWER_CNT ) = data_len;
        memcpy( sdata + MB_ANSWER_DATA, pdata, data_len );
        if ( ModBusAddr( reqst->function ) == MBUS_REG_16BIT ) {
            //поменяем байты местами для переменных uint16_t, т.к. сначала передаем старший байт
            //в текущей модели LITTLE-ENDIAN младший байт хранится первым
            for ( idx = 0; idx < data_len; idx += sizeof( uint16_t ) ) {
                ptr16 = (uint16_t *)&sdata[idx + MB_ANSWER_DATA];
                *ptr16 = __REVSH( *ptr16 );
               }
           }
        //расчет контрольной суммы данных по уже переставленным байтам
        //контрольная сумма передается в фрейме младшим байтом вперед
        crc = CalcCRC16( RS485SendBuff(), data_len + MB_ANSWER_HEAD );
        memcpy( sdata + data_len + MB_ANSWER_HEAD, (uint8_t *)&crc, sizeof( crc ) );
        return data_len + MB_ANSWER_HEAD + sizeof( crc );
       }
    //ответ на запись значения(й) в один(несколько) регистр(ов) хранения
    if ( TypeRequst( reqst->function ) == MBUS_REQST_WRITE1 || TypeRequst( reqst->function ) == MBUS_REQST_WRITEN ) {
        mbus_req.dev_addr = reqst->dev_addr;
        mbus_req.function = reqst->function;
        mbus_req.reg_addr = __REVSH( reqst->reg_addr );
        if ( TypeRequst( reqst->function ) == MBUS_REQST_WRITEN )
            mbus_req.reg_cntval = __REVSH( reqst->reg_cnt );
        else {
            //при записи в один регистр возвращаем записанное значение
            ptr16 = (uint16_t *)reqst->ptr_data;
            mbus_req.reg_cntval = __REVSH( *ptr16 );
           }
        mbus_req.crc = CalcCRC16( (uint8_t *)&mbus_req, sizeof( mbus_req ) - sizeof( mbus_req.crc ) );
        //запись данных для отправки
        ClearSend();
        memcpy( sdata, &mbus_req, sizeof( mbus_req ) );
        return sizeof( mbus_req );
       }
    return 0;
 }

//*************************************************************************************************
// Функция проверяет код функции на наличии в списке поддерживаемых функций
//-------------------------------------------------------------------------------------------------
// uint8_t func_id     - код функции
// const uint8_t *list - указатель на список поддерживаемых функций
// return = SUCCESS    - код функции есть в списке
//        = ERROR      - код функции не поддерживается
//*************************************************************************************************
static ErrorStatus ChkFuncValid( uint8_t func_id, const uint8_t *list ) {

    uint8_t i;
    
    for ( i = 0; ; i++ ) {
        if ( *( list + i ) == FUNC_END )
            break;
        if ( *( list + i ) == func_id )
            return SUCCESS;
       }
    return ERROR;
 }

//*************************************************************************************************
// Функция проверяет номер регистра в списке доступных и кол-во регистров для чтения/записи
//-------------------------------------------------------------------------------------------------
// MBUS_REQ *reqst      - Указатель на структуру с данными запроса MODBUS 
// const RegsRead *list - Указатель на массив набора регистров
// return = SUCCESS     - регистр есть в списке
//        = ERROR       - регистра нет в списке
//*************************************************************************************************
static ErrorStatus ChkRegValid( MBUS_REQ *reqst, const RegsRead *list ) {

    uint8_t i, r;
    ErrorStatus check = ERROR;
    
    //проверка доступности регистра
    for ( i = 0; ; i++ ) {
        //проверка на признак окончания набора данных
        if ( list[i].id_reg == REG_END )
            break;
        //поиск по номеру регистра
        if ( list[i].id_reg == reqst->reg_addr ) {
            //проверка кол-ва регистров
            for ( r = 0; r < sizeof( list[i].cnt_reg ); r++ )
                if ( list[i].cnt_reg[r] == reqst->reg_cnt )
                    return SUCCESS;
           }
       }
    return check;
 }
 
//*************************************************************************************************
// Функция проверяет значения регистра(ов) на допустимые диапазоны 
//-------------------------------------------------------------------------------------------------
// MBUS_REQ *reqst        - Указатель на структуру с данными запроса MODBUS
// const ValueValid *list - Указатель на массив допустимых значений регистров
// return = SUCCESS       - значение находиться в допустимом диапазоне
//        = ERROR         - значение за пределами допустимого диапазона
//*************************************************************************************************
static ErrorStatus ChkRegValue( MBUS_REQ *reqst, const ValueValid *list ) {

    uint16_t i, reg, cnt, *ptr16, value;
    ErrorStatus check = ERROR;
    
    reg = reqst->reg_addr;
    cnt = reqst->reg_cnt;
    ptr16 = (uint16_t *)reqst->ptr_data;
    //проверка доступности регистра
    for ( i = 0; ; i++ ) {
        //проверка на признак окончания набора данных
        if ( list[i].id_reg == REG_END )
            break;
        //поиск по номеру регистра
        if ( list[i].id_reg == reg ) {
            //регистр найден
            cnt--;
            value = *ptr16;
            if ( value >= list[i].min_value && value <= list[i].max_value ) {
                //значения регистра находятся в допустимом диапазоне
                check = SUCCESS;
                if ( cnt ) {
                    //переход на следующий регистр если cnt_reg > 1
                    reg++;
                    ptr16++;
                   }
               }
            else return ERROR;
           }
       }
    return check;
 }

//*************************************************************************************************
// Запись значения(й) в регистр(ы)
//-------------------------------------------------------------------------------------------------
// MBUS_REQ *reqst  - Указатель на структуру с данными запроса MODBUS
// return = SUCCESS - значение записано
//        = ERROR   - ошибка записи значения
//*************************************************************************************************
static ErrorStatus RegWrite( MBUS_REQ *reqst ) {

    uint16_t write, *ptr16;
    MBUS_DTIME dtime;
    DATE_TIME  rtc;
    
    ptr16 = (uint16_t *)reqst->ptr_data;
    write = *ptr16;
    if ( reqst == NULL )
        return ERROR;
    if ( reqst->reg_addr == MBUS_REG_CTRL ) {
        //проверка на закрытие горячей и холодной воды
        if ( write == MBUS_CMD_ALL_CLOSE ) {
            osEventFlagsSet( valve_event, EVN_VALVE_COLD_CLS | EVN_VALVE_HOT_CLS | EVN_VALVE_PREV_CHECK );
            return SUCCESS; //высокий приоритет, дальше команды не проверяем
           }
        if ( write == MBUS_CMD_ALL_OPEN ) {
            osEventFlagsSet( valve_event, EVN_VALVE_COLD_OPN | EVN_VALVE_HOT_OPN | EVN_VALVE_PREV_CHECK );
            return SUCCESS; //высокий приоритет, дальше команды не проверяем
           }
        //проверка на взаимоисключающие команды
        if ( write & MBUS_CMD_COLD_OPEN && write & MBUS_CMD_COLD_CLOSE )
            return ERROR;
        if ( write & MBUS_CMD_HOT_OPEN && write & MBUS_CMD_HOT_CLOSE )
            return ERROR;
        //выполнение команды
        if ( write & MBUS_CMD_COLD_OPEN )
            osEventFlagsSet( valve_event, EVN_VALVE_COLD_OPN | EVN_VALVE_PREV_CHECK );
        if ( write & MBUS_CMD_COLD_CLOSE )
            osEventFlagsSet( valve_event, EVN_VALVE_COLD_CLS | EVN_VALVE_PREV_CHECK );
        if ( write & MBUS_CMD_HOT_OPEN )
            osEventFlagsSet( valve_event, EVN_VALVE_HOT_OPN | EVN_VALVE_PREV_CHECK );
        if ( write & MBUS_CMD_HOT_CLOSE )
            osEventFlagsSet( valve_event, EVN_VALVE_HOT_CLS | EVN_VALVE_PREV_CHECK );
        return SUCCESS;
       }
    if ( reqst->reg_addr == MBUS_REG_DAYMON && reqst->reg_cnt == 3 ) {
        //установка даты - времени
        memcpy( (uint8_t *)&dtime, reqst->ptr_data, sizeof( dtime ) );
        rtc.day = dtime.day; 
        rtc.month = dtime.month;
        rtc.year = dtime.year;
        rtc.hour = dtime.hour;
        rtc.min = dtime.min;
        rtc.sec = 0; 
        return SetTimeDate( &rtc );
       }
    return ERROR;
 }

//*************************************************************************************************
// По коду функции возвращет тип адресации регистров 8/16-битная
//-------------------------------------------------------------------------------------------------
// uint8_t func         - код функции
// return ModbusAddrReg - тип адресации регистров
//*************************************************************************************************
static ModbusAddrReg ModBusAddr( uint8_t func ) {

    //функции с 8-битной адресаций
    if ( func == FUNC_RD_COIL_STAT || func == FUNC_RD_DISC_INP || func == FUNC_WR_SING_COIL || \
         func == FUNC_WR_MULT_COIL || func == FUNC_RD_EXCP_STAT )
        return MBUS_REG_8BIT;
    //функции с 16-битной адресаций
    if ( func == FUNC_RD_HOLD_REG || func == FUNC_RD_INP_REG || func == FUNC_WR_SING_REG || \
         func == FUNC_WR_MULT_REG || func == FUNC_WR_MASK_REG || func == FUNC_RD_FIFO_QUE || \
         func == FUNC_RD_FILE_REC || func == FUNC_WR_FILE_REC || func == FUNC_RD_EVENT_CNT || \
         func == FUNC_RD_DIAGNOSTIC || func == FUNC_RD_EVENT_LOG )
        return MBUS_REG_16BIT;
    return MBUS_REG_OTHER;
 }

//*************************************************************************************************
// Функция возвращает тип запроса master -> slave
//-------------------------------------------------------------------------------------------------
// uint8_t func_id     - код функции
// return ModbusRequst - тип запроса
//*************************************************************************************************
static ModbusRequst TypeRequst( uint8_t func_id ) {

    if ( func_id == FUNC_RD_COIL_STAT || func_id == FUNC_RD_DISC_INP || func_id == FUNC_RD_HOLD_REG || \
         func_id == FUNC_RD_INP_REG || func_id == FUNC_RD_EXCP_STAT || func_id == FUNC_RD_DIAGNOSTIC )
        return MBUS_REQST_READ;
    if ( func_id == FUNC_WR_SING_COIL || func_id == FUNC_WR_SING_REG || func_id == FUNC_WR_MASK_REG )
        return MBUS_REQST_WRITE1;
    if ( func_id == FUNC_WR_MULT_COIL || func_id == FUNC_WR_MULT_REG || func_id == FUNC_WR_FILE_REC )
        return MBUS_REQST_WRITEN;
    return MBUS_REQST_UNKNOW;
 }

//*************************************************************************************************
// Обнуляет счетчики ошибок протокола MODBUS
//*************************************************************************************************
void ModBusErrClr( void ) {

    recv_total = 0;
    memset( (uint8_t *)&error_cnt, 0x00, sizeof( error_cnt ) );
 }

//*************************************************************************************************
// Инкремент счетчиков ошибок
//-------------------------------------------------------------------------------------------------
// ModBusError error - индекс кода ошибки
//*************************************************************************************************
void IncError( ModBusError error ) {

    if ( error < SIZE_ARRAY( error_cnt ) )
        error_cnt[error]++;
 }

//*************************************************************************************************
// Возвращает указатель на строку расшифровки результата выполнения запроса по протоколу MODBUS
//-------------------------------------------------------------------------------------------------
// ModBusError error - код ошибки обработки принятого ответа от уст-ва
// return            - расшифровка кода ошибки
//*************************************************************************************************
char *ModBusErrDesc( ModBusError error ) {

    if ( error >= SIZE_ARRAY( error_descr ) )
        return NULL;
    return error_descr[error];
 }

//*************************************************************************************************
// Возвращает значение счетчика ошибок MODBUS
//-------------------------------------------------------------------------------------------------
// ModBusError error - индекс счетчика ошибок, для error = MBUS_REQUEST_OK возвращается кол-во счетчиков
// return            - значение счетчика ошибок
//*************************************************************************************************
uint32_t ModBusErrCnt( ModBusError error ) {

    if ( error == MBUS_REQUEST_OK )
        return SIZE_ARRAY( error_cnt );
    if ( error < SIZE_ARRAY( error_cnt ) )
        return error_cnt[error];
    return 0;
 }

//*************************************************************************************************
// Возвращает расшифровку и значения счетчиков ошибок MODBUS
//-------------------------------------------------------------------------------------------------
// ModBusError error - индекс счетчика ошибок, для error = MBUS_REQUEST_OK 
//                     возвращается общее кол-во принятых фреймов
// char *str         - указатель для размещения результата
// return            - значение счетчика ошибок
//*************************************************************************************************
char *ModBusErrCntDesc( ModBusError error, char *str ) {

    char *ptr;
    
    if ( error >= SIZE_ARRAY( error_cnt ) )
        return NULL;
    ptr = str;
    if ( error == MBUS_REQUEST_OK ) {
        ptr += sprintf( ptr, "Total packages recv" );
        ptr += AddDot( str, 45, 0 );
        ptr += sprintf( ptr, "%6u", recv_total );
        return str;
       }
    ptr += sprintf( ptr, "%s", ModBusErrDesc( error ) );
    //дополним расшифровку ошибки справа знаком "." до 45 символов
    ptr += AddDot( str, 45, 0 );
    ptr += sprintf( ptr, "%6u ", error_cnt[error] );
    return str;
 }
