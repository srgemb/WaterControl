
//*************************************************************************************************
//
// Управление внешней FRAM памятью
// 
//*************************************************************************************************

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "fram.h"
#include "crc16.h"
#include "uart.h"
#include "message.h"
#include "events.h"
#include "xtime.h"
#include "command.h"

//#define DEBUG_FRAM                                      //вывод отладочной информации

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define FRAM_ID_ADDR            0xA0            //slave адрес FRAM памяти

#define FRAM_TIMEOUT            100             //время ожидания выполнения операции чтения/записи
                                                //в FRAM без использования DMA-IT

//Индексы ошибок обмена данными по I2C
typedef enum {
    ERROR_I2C_OK,
    ERROR_I2C_BERR,
    ERROR_I2C_ARLO,
    ERROR_I2C_AF,
    ERROR_I2C_OVR,
    ERROR_I2C_DMA_TRANFER,
    ERROR_I2C_TIMEOUT,
    ERROR_I2C_SIZE,
    ERROR_I2C_DMA_PARAM,
    ERROR_I2C_START
 } ERROR_I2C;

//Расшифровка ошибок функций обмена данными по I2C
static char * const error_dma[] = {
    "OK",
    "BERR error ",
    "ARLO error ",
    "AF error ",
    "OVR error ",
    "DMA transfer error ",
    "Timeout Error ",
    "Size Management error ",
    "DMA Parameter Error ",
    "Wrong start Error "
 };

//Расшифровка ошибок при вызове функций чтения/записи по I2C
static char * const error_fram[] = {
    "OK",
    "Error",
    "Busy",
    "Timeout",
    "CRC",
    "Parameter"
 };

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern CONFIG config;
extern CURR_DATA curr_data;
extern WATER_LOG water_log;
extern I2C_HandleTypeDef hi2c1;

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osEventFlagsId_t fram_event = NULL;

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static FramStatus fram_error_rd, fram_error_wr;
static char buffer1[160], buffer2[32];
static osMutexId_t fram_mutex = NULL;
static osSemaphoreId_t sem_save = NULL, sem_read = NULL;

#pragma pack( push, 1 )

//структура для чтения/записи блока данных в FRAM
typedef struct {
    uint8_t  data[FRAM_BLOCK_SIZE - sizeof( uint16_t )];
    uint16_t crc;
} FRAM_DATA; 

#pragma pack( pop )

static FRAM_DATA fram_read, fram_save;     //буфер хранения одного блока данных FRAM

static const osSemaphoreAttr_t sem_attr = { .name = "FramSem" };
static const osMutexAttr_t mutex_attr = { .name = "FramMut", .attr_bits = osMutexPrioInherit };
 
//*************************************************************************************************
// Прототипы локальные функций
//*************************************************************************************************
static FramStatus FRAMSave( uint16_t mem_addr, uint8_t *ptr_data, uint16_t len );
static FramStatus FRAMRead( uint16_t mem_addr, uint8_t *ptr_data, uint16_t len );

//*************************************************************************************************
// Инициализация объектов RTOS, чтение текущих параметров
//*************************************************************************************************
void FramInit( void ) {

    uint16_t crc;
    DATE_TIME dtime;
    
    //семафор ожидания завершения цикла DMA
    sem_read = osSemaphoreNew( 1, 0, &sem_attr );
    sem_save = osSemaphoreNew( 1, 0, &sem_attr );
    //мьютекс блокировки работы с FRAM
    fram_mutex = osMutexNew( &mutex_attr );
    //читаем блок из FRAM без IT/DMA
    fram_error_rd = (FramStatus)HAL_I2C_Mem_Read( &hi2c1, FRAM_ID_ADDR, FRAM_ADDR_DATA, I2C_MEMADD_SIZE_16BIT, (uint8_t *)&fram_read, sizeof( fram_read ), FRAM_TIMEOUT );
    if ( fram_error_rd != FRAM_OK )
        memset( (uint8_t *)&curr_data, 0x00, sizeof( curr_data ) ); //ошибка чтения данных
    else {
        //проверка CRC прочитанного блока
        crc = CalcCRC16( (uint8_t *)&fram_read, sizeof( fram_read.data ) );
        if ( crc != fram_read.crc ) {
            //КС не совпала, сформируем блок с нулевыми данными
            fram_error_rd = FRAM_ERROR_CRC;
            memset( (uint8_t *)&curr_data, 0x00, sizeof( curr_data ) );
            curr_data.next_addr = FRAM_ADDR_LOG;
           }
       }
    if ( fram_error_rd == FRAM_OK )
        memcpy( (uint8_t *)&curr_data, (uint8_t *)&fram_read, sizeof( curr_data ) ); //прочитанный блок сохраним в WATER
    //обновим источник сброса и дата/время включения контроллера
    GetTimeDate( &dtime );
    curr_data.res_src = ResetSrc(); //источник сброса
    curr_data.day = dtime.day;      //день
    curr_data.month = dtime.month;  //месяц
    curr_data.year = dtime.year;    //год
    curr_data.hour = dtime.hour;    //часы
    curr_data.min = dtime.min;      //минуты
    curr_data.sec = dtime.sec;      //секунды
    //сохраним блок с новыми данными в FRAM
    memset( (uint8_t *)&fram_save, 0x00, sizeof( fram_save ) );
    memcpy( (uint8_t *)&fram_save, (uint8_t *)&curr_data, sizeof( curr_data ) );
    fram_save.crc = CalcCRC16( (uint8_t *)&fram_save, sizeof( fram_save.data ) );
    fram_error_wr = (FramStatus)HAL_I2C_Mem_Write( &hi2c1, FRAM_ID_ADDR, FRAM_ADDR_DATA, I2C_MEMADD_SIZE_16BIT, (uint8_t *)&fram_save, sizeof( fram_save ), FRAM_TIMEOUT );
 }
 
//*************************************************************************************************
// Чтение блока данных из FRAM памяти
//-------------------------------------------------------------------------------------------------
// uint16_t addr     - адрес в FRAM памяти
// uint8_t *ptr_data - указатель на размещения прочитанного блока данных
// uint16_t len      - размер записываемого блока данных
// return FramStatus - результат чтения данных
//*************************************************************************************************
FramStatus FramReadData( uint16_t addr, uint8_t *ptr_data, uint16_t len ) {

    uint16_t crc;
    FramStatus status;
    
    if ( len > FRAM_BLOCK_SIZE )
        return FRAM_ERROR_PARAM;
    osMutexAcquire( fram_mutex, osWaitForever ); //устанавливаем блокировку
    //очищаем буфер данных
    memset( (uint8_t *)&fram_read, 0x00, sizeof( fram_read ) );
    //чтение блок данных
    status = FRAMRead( addr, (uint8_t *)&fram_read, sizeof( fram_read ) );
    if ( status != FRAM_OK ) {
        osMutexRelease( fram_mutex ); //снимаем блокировку
        return status;
       }
    //проверка CRC прочитанного блока
    crc = CalcCRC16( (uint8_t *)&fram_read, sizeof( fram_read.data ) );
    if ( crc != fram_read.crc ) {
        //ошибка контрольной суммы
        osMutexRelease( fram_mutex ); //снимаем блокировку
        return FRAM_ERROR_CRC;
       }
    //копируем прочитанные данные в буфер назначения
    memcpy( ptr_data, (uint8_t *)&fram_read, len );
    osMutexRelease( fram_mutex ); //снимаем блокировку
    return FRAM_OK;
 }

//*************************************************************************************************
// Запись блока данных в FRAM память, перед записью вычисляется КС блока данных
// Для данных типа WATER_DATA_LOG выполняется расчет следующего адреса для размещения 
// следующего блока данных
//-------------------------------------------------------------------------------------------------
// FramData type - тип блока данных
// uint8_t *ptr  - указатель на записываемый блок данных
// uint16_t len  - размер записываемого блока данных
//*************************************************************************************************
FramStatus FramSaveData( TypeData type, uint8_t *ptr_data, uint16_t len ) {

    FramStatus status;
    uint16_t addr = 0;
    
    if ( len > sizeof( fram_save.data ) || ptr_data == NULL )
        return FRAM_ERROR_PARAM;
    //устанавливаем блокировку
    osMutexAcquire( fram_mutex, osWaitForever );
    //адрес размещения блока данных в памяти
    if ( type == CURRENT_DATA )
        addr = FRAM_ADDR_DATA;
    else addr = curr_data.next_addr;
    #ifdef DEBUG_FRAM
    sprintf( buffer1, "Record current data at: 0x%04X ", addr );
    UartSendStr( buffer1 );
    #endif
    //очищаем буфер данных
    memset( (uint8_t *)&fram_save, 0x00, sizeof( fram_save ) );
    //копируем блок даннных в промежуточный буфер
    memcpy( (uint8_t *)&fram_save, ptr_data, len );
    //расчет КС блока данных
    fram_save.crc = CalcCRC16( (uint8_t *)&fram_save, sizeof( fram_save.data ) );
    //запись блока
    status = FRAMSave( addr, (uint8_t *)&fram_save, sizeof( fram_save ) );
    if ( status != FRAM_OK ) {
        //выходим при ошибке
        osMutexRelease( fram_mutex ); //снимаем блокировку
        sprintf( buffer1, "Error write to FRAM: 0x%04X %s\r\n", addr, FramErrorDesc( status ) );
        UartSendStr( buffer1 );
        return status;                  
       }
    if ( type == WATER_DATA_LOG ) {
        curr_data.next_addr += sizeof( fram_save );
        if ( curr_data.next_addr >= FRAM_SIZE )
            curr_data.next_addr = FRAM_ADDR_LOG;
       }
    osMutexRelease( fram_mutex ); //снимаем блокировку
    return FRAM_OK;
 }

//*************************************************************************************************
// Очистка FRAM памяти только в области хранения данных логирования
//*************************************************************************************************
void FramClear( void ) {

    uint16_t mem_addr;
    FramStatus status;

    //устанавливаем блокировку FRAM
    osMutexAcquire( fram_mutex, osWaitForever );
    memset( (uint8_t *)&fram_save, 0x00, sizeof( fram_save ) );
    for ( mem_addr = FRAM_ADDR_LOG; mem_addr < FRAM_SIZE; mem_addr += sizeof( fram_save ) ) {
        status = FRAMSave( mem_addr, (uint8_t *)&fram_save, sizeof( fram_save ) );
        if ( status != FRAM_OK ) {
            sprintf( buffer1, "Error write to FRAM: 0x%04X %s\r\n", mem_addr, FramErrorDesc( status ) );
            UartSendStr( buffer1 );
           }
       }
    //снимаем блокировку FRAM
    osMutexRelease( fram_mutex );
 }

//*************************************************************************************************
// Вывод HEX дампа FRAM памяти
// uint8_t blocks - кол-во блоков 16x16 для вывода
//*************************************************************************************************
void FramHexDump( uint8_t blocks ) {

    uint16_t addr;
    FramStatus status;
    uint8_t data_hex[16], block, cnt_blk;

    //устанавливаем блокировку FRAM
    osMutexAcquire( fram_mutex, osWaitForever );
    if ( blocks )
        cnt_blk = blocks; //указано кол-во блоков
    else cnt_blk = FRAM_SIZE/256;
    //чтение - вывод данных
    for ( addr = 0, block = 0; addr < FRAM_SIZE && cnt_blk; addr += sizeof( data_hex ) ) {
        memset( (uint8_t *)&data_hex, 0x00, sizeof( data_hex ) );
        status = (FramStatus)FRAMRead( addr, data_hex, sizeof( data_hex ) );
        if ( status != FRAM_OK ) {
            sprintf( buffer1, "Error read from FRAM: 0x%04X %s\r\n", addr, FramErrorDesc( status ) );
            UartSendStr( buffer1 );
           }
        //вывод строки HEX дампа
        DataHexDump( data_hex, HEX_16BIT_ADDR, addr, buffer1 );
        UartSendStr( buffer1 );
        if ( block++ >= 15 ) {
            cnt_blk--;
            block = 0; //выделение следующего блока 16*16
            UartSendStr( (char *)msg_crlr );
           }
       }
    //снимаем блокировку FRAM
    osMutexRelease( fram_mutex );
 }

//*************************************************************************************************
// Проверка контрольных сумм всех блоках в FRAM
//*************************************************************************************************
void FramCheck( void ) {

    uint16_t addr;
    FramStatus status;

    //проверка интервальных данных
    UartSendStr( "Checking FRAM data ...\r\n" );
    for ( addr = FRAM_ADDR_DATA; addr < FRAM_SIZE; addr += sizeof( fram_read ) ) {
        //чтение блока данных, проверка
        status = FramReadData( addr, (uint8_t *)&fram_read, sizeof( fram_read ) );
        if ( status == FRAM_OK )
            sprintf( buffer1, "Address: 0x%04X ... OK\r\n", addr );
        else {
            if ( status == FRAM_ERROR_CRC && ( fram_read.crc == 0x0000 || fram_read.crc == 0xFFFF ) )
                sprintf( buffer1, "Address: 0x%04X ... Block free\r\n", addr );
            else sprintf( buffer1, "Address: 0x%04X ... %s\r\n", addr, FramErrorDesc( status ) );
           }
        UartSendStr( buffer1 );
       }
 } 

//*************************************************************************************************
// Полный тест FRAM памяти
// Проверка выполняется блоками по 16 байт
//*************************************************************************************************
void FramTest( void ) {

    uint16_t addr, error_cnt;
    FramStatus status;
    static uint8_t i, save[16], read[16];
    uint8_t value[] = { 0xFF, 0x55, 0xAA, 0x00 }; //значения для тестирования
    
    //устанавливаем блокировку FRAM
    osMutexAcquire( fram_mutex, osWaitForever );
    for ( i = 0; i < sizeof( value ); i++ ) {
        memset( save, value[i], sizeof( save ) );
        sprintf( buffer1, "Write: 0x%02X  ", value[i] );
        UartSendStr( buffer1 );
        //запись одного значения по всем адресам
        for ( addr = 0; addr < FRAM_SIZE; addr += sizeof( save ) ) {
            status = FRAMSave( addr, (uint8_t *)&save, sizeof( save ) );
            if ( status != FRAM_OK ) {
                //сообщим об ошибке
                sprintf( buffer1, "Error save: 0x%04X\r\n", addr );
                UartSendStr( buffer1 );
               }
           }
        //чтение - проверка
        error_cnt = 0;
        for ( addr = 0; addr < FRAM_SIZE; addr += sizeof( save ) ) {
            memset( read, 0x00, sizeof( read ) );
            status = FRAMRead( addr, (uint8_t *)&read, sizeof( read ) );
            if ( status != FRAM_OK ) {
                //сообщим об ошибке
                sprintf( buffer1, "Error read: 0x%04X\r\n", addr );
                UartSendStr( buffer1 );
               }
            //сравнение записанных/прочитанных данных
            if ( memcmp( (uint8_t *)&save, (uint8_t *)&read, sizeof( save ) ) != 0 ) {
                error_cnt++;
                sprintf( buffer1, "Error compare: 0x%04X\r\n", addr );
                UartSendStr( buffer1 );
               }
           }
        if ( !error_cnt )
            UartSendStr( "Compare: OK\r\n" );
       }
    UartSendStr( (char *)msg_crlr );
    //снимаем блокировку FRAM
    osMutexRelease( fram_mutex );
 }

//*************************************************************************************************
// Чтение блока данных из FRAM памяти с использованием DMA
//-------------------------------------------------------------------------------------------------
// uint16_t mem_addr - адрес чтения из FRAM
// uint8_t *ptr_dta  - указатель на адрес размещения прочитанного блока данных
// uint16_t len      - размер читаемого блока данных
// return FramStatus - результат выполнения
//*************************************************************************************************
static FramStatus FRAMRead( uint16_t mem_addr, uint8_t *ptr_data, uint16_t len ) {

    FramStatus status;
    
    status = (FramStatus)HAL_I2C_Mem_Read_DMA( &hi2c1, FRAM_ID_ADDR, mem_addr, I2C_MEMADD_SIZE_16BIT, ptr_data, len );
    if ( status == FRAM_OK )
        osSemaphoreAcquire( sem_read, osWaitForever ); //ждем завершения чтения
    return status;
 }

//*************************************************************************************************
// Запись данных в FRAM память с использованием DMA
//-------------------------------------------------------------------------------------------------
// uint16_t mem_addr - адрес записи в FRAM
// uint8_t *ptr_dta  - указатель на адрес размещения прочитанного блока данных
// uint16_t len      - размер записываемого блока данных
// return FramStatus - результат выполнения
//*************************************************************************************************
static FramStatus FRAMSave( uint16_t mem_addr, uint8_t *ptr_data, uint16_t len ) {

    FramStatus status;
    
    status = (FramStatus)HAL_I2C_Mem_Write_DMA( &hi2c1, FRAM_ID_ADDR, mem_addr, I2C_MEMADD_SIZE_16BIT, ptr_data, len );
    if ( status == FRAM_OK )
        osSemaphoreAcquire( sem_save, osWaitForever ); //ждем завершения записи
    return status;
 }

//*************************************************************************************************
// CallBack функция завершения записи данных в FRAM через DMA
//*************************************************************************************************
void HAL_I2C_MemTxCpltCallback( I2C_HandleTypeDef *hi2c ) {

    osSemaphoreRelease( sem_save );
 }

//*************************************************************************************************
// CallBack функция завершения чтения данных из FRAM через DMA
//*************************************************************************************************
void HAL_I2C_MemRxCpltCallback( I2C_HandleTypeDef *hi2c ) {
    
    osSemaphoreRelease( sem_read );
 }

//*************************************************************************************************
// CallBack функция ошибки обмена по I2C
//*************************************************************************************************
void HAL_I2C_ErrorCallback( I2C_HandleTypeDef *hi2c ) {

    uint32_t code;
    
    code = hi2c->ErrorCode;
    memset( buffer1, 0x00, sizeof( buffer1 ) );
    strcat( buffer1, "ErrorCallback: " );
    if ( code & HAL_I2C_ERROR_NONE )
        strcat( buffer1, error_dma[ERROR_I2C_OK] );
    if ( code & HAL_I2C_ERROR_BERR )
        strcat( buffer1, error_dma[ERROR_I2C_BERR] );
    if ( code & HAL_I2C_ERROR_ARLO )
        strcat( buffer1, error_dma[ERROR_I2C_ARLO] );
    if ( code & HAL_I2C_ERROR_AF )
        strcat( buffer1, error_dma[ERROR_I2C_AF] );
    if ( code & HAL_I2C_ERROR_OVR )
        strcat( buffer1, error_dma[ERROR_I2C_OVR] );
    if ( code & HAL_I2C_ERROR_DMA )
        strcat( buffer1, error_dma[ERROR_I2C_DMA_TRANFER] );
    if ( code & HAL_I2C_ERROR_TIMEOUT )
        strcat( buffer1, error_dma[ERROR_I2C_TIMEOUT] );
    if ( code & HAL_I2C_ERROR_SIZE )
        strcat( buffer1, error_dma[ERROR_I2C_SIZE] );
    if ( code & HAL_I2C_ERROR_DMA_PARAM )
        strcat( buffer1, error_dma[ERROR_I2C_DMA_PARAM] );
    if ( code & HAL_I2C_WRONG_START )
        strcat( buffer1, error_dma[ERROR_I2C_START] );
    strcat( buffer1, "\r\n" );
    UartSendStr( buffer1 );
 }

//*************************************************************************************************
// Функция возвращает код ошибки при чтения/записи по I2C при инициализации контроллера
//-------------------------------------------------------------------------------------------------
// FramErrorType type - тип ошибки (чтения/запись)
// return FramStatus  - код ошибки
//*************************************************************************************************
FramStatus FramError( FramErrorType type ) {

    if ( type == FRAM_ERR_READ )
        return fram_error_rd;
    else return fram_error_wr;
 }

//*************************************************************************************************
// Функция возвращает расшифровку кода ошибки при вызове функций чтения/записи по I2C
//-------------------------------------------------------------------------------------------------
// FramStatus error - код ошибки
// return           - указатель на строку с расшифровкой результата   
//*************************************************************************************************
char *FramErrorDesc( FramStatus error ) {

    if ( error == FRAM_OK )
        sprintf( buffer2, "%s", error_fram[error] );
    else sprintf( buffer2, "Error: %s", error_fram[error] );
    return buffer2;
 }
