
//*************************************************************************************************
//
// Обработка команд полученных по UART
//
//*************************************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "cmsis_os.h"

#include "config.h"
#include "command.h"
#include "modbus.h"
#include "events.h"
#include "can.h"
#include "fram.h"
#include "water.h"
#include "uart.h"
#include "xtime.h"
#include "zigbee.h"
#include "parse.h"
#include "sort.h"
#include "message.h"
#include "version.h"

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern CONFIG config;
extern CURR_DATA curr_data;
extern WATER_LOG water_log;
extern uint16_t pressure_cold, pressure_hot;

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osEventFlagsId_t cmnd_event = NULL;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
//структура хранения и выполнения команд
typedef struct {
    char    name_cmd[20];                              //имя команды
    void    (*func)( uint8_t cnt_par, char *param );   //указатель на функцию выполнения
} CMD;

//расшифровка статуса задач
#ifdef DEBUG_TARGET
static char * const state_name[] = {
    "Inactive",
    "Ready",
    "Running",
    "Blocked",
    "Terminated",
    "Error"
 };
#endif

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t task_attr = {
    .name = "Command", 
    .stack_size = 832,
    .priority = osPriorityNormal
 };

static const osEventFlagsAttr_t evn_attr = { .name = "CmndEvents" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void TaskCommand( void *argument );
static void ExecCommand( char *buff );
static void WaterLog( uint8_t cnt_view );
static ErrorStatus StrHexToBin( char *str, uint8_t *hex, uint8_t size );
static ErrorStatus HexToBin( char *ptr, uint8_t *bin );

#ifdef DEBUG_TARGET
static char *TaskStateDesc( osThreadState_t state );
#endif
static char *VersionRtos( uint32_t version, char *str );

static void CmndDate( uint8_t cnt_par, char *param );
static void CmndTime( uint8_t cnt_par, char *param );
static void CmndDateTime( uint8_t cnt_par, char *param );
static void CmndHelp( uint8_t cnt_par, char *param );
static void CmndWater( uint8_t cnt_par, char *param );
static void CmndValve( uint8_t cnt_par, char *param );
static void CmndFram( uint8_t cnt_par, char *param );
static void CmndStat( uint8_t cnt_par, char *param );
static void CmndConfig( uint8_t cnt_par, char *param );
static void CmndZigBee( uint8_t cnt_par, char *param );
static void CmndVersion( uint8_t cnt_par, char *param );
#ifdef DEBUG_TARGET
static void CmndLog( uint8_t cnt_par, char *param );
static void CmndTask( uint8_t cnt_par, char *param );
static void CmndFlash( uint8_t cnt_par, char *param );
static void CmndReset( uint8_t cnt_par, char *param );
#endif

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static char buffer[128];

static const char *help = {
    "date [dd.mm.yy]                 - Display/set date.\r\n"
    "time [hh:mm[:ss]]               - Display/set time.\r\n"
    "dtime                           - Display date and time.\r\n"
    "valve [cold/hot opn/cls]        - Status, valve drive control.\r\n"
    "stat                            - Statistics.\r\n"
    "fram [N][clr/chk/test]          - FRAM HEX dump, clear, testing.\r\n"
    #ifdef DEBUG_TARGET              
    "task                            - List task statuses, time statistics.\r\n"
    "flash                           - FLASH config HEX dump.\r\n"
    #endif                           
    "zb [res/init/net/save/cfg/chk]  - ZigBee module control.\r\n"
    "water [cold/hot/filter/log [N]] - Water flow status, setting initial values.\r\n"
    "config                          - Display of configuration parameters.\r\n"
    "config save                     - Save configuration settings.\r\n"
    "config {cold/hot/filter} xxxxx  - Setting incremental values for water meters.\r\n"
    "config uart xxxxx               - Setting the speed baud (600 - 115200).\r\n"
    "config modbus speed xxxxx       - Setting the speed baud (600 - 115200).\r\n"
    "config modbus id 0x01 - 0xF8    - Setting the device ID on the modbus (HEX format without 0x).\r\n"
    "config can id 0xXXXXXXXX        - Setting the Device ID on the CAN Bus (HEX format without 0x).\r\n"
    "config can addr xxxxx           - Setting the width of the CAN bus identifier (11/29 bits).\r\n"
    "config can speed xxxxx          - Set the CAN bus speed 10,20,50,125,250,500 (kbit/s).\r\n"
    "config pres_max xxxxx           - Set the maximum pressure for the sensor.\r\n"
    "config pres_omin xxxxx          - Setting the minimum output voltage of the pressure sensor.\r\n"
    "config pres_omax xxxxx          - Setting the maximum output voltage of the pressure sensor.\r\n"
    "config panid 0x0000 - 0xFFFE    - Network PANID (HEX format without 0x).\r\n"
    "config netgrp 1-99              - Network group number.\r\n"
    "config netkey XXXX....          - Network key (HEX format without 0x).\r\n"
    "config devnumb 0x0001 - 0xFFFF  - Device number on the network (HEX format without 0x).\r\n"
    "config gate 0x0000- 0xFFF8      - Gateway address (HEX format without 0x).\r\n"
    "version                         - Displays the version number and date.\r\n"
    #ifdef DEBUG_TARGET              
    "reset                           - Reset controller.\r\n"
    #endif                           
    "?                               - Help.\r\n"
  };

//Перечень доступных команд
//*********************************************
//     Имя команды      Функция вызова
//*********************************************
static const CMD cmd[] = {
    { "time",           CmndTime },
    { "date",           CmndDate },
    { "dtime",          CmndDateTime },
    { "water",          CmndWater },
    { "valve",          CmndValve },
    { "stat",           CmndStat },
    { "fram",           CmndFram },
    { "config",         CmndConfig },
    { "zb",             CmndZigBee },
    { "version",        CmndVersion },
    #ifdef DEBUG_TARGET
    { "log",            CmndLog },
    { "task",           CmndTask },
    { "flash",          CmndFlash },
    { "reset",          CmndReset },
    #endif
    { "?",              CmndHelp }
 };

//*************************************************************************************************
// Инициализация задачи
//*************************************************************************************************
void CommandInit( void ) {

    //очередь событий
    cmnd_event = osEventFlagsNew( &evn_attr );
    //создаем задачу обработки команд
    osThreadNew( TaskCommand, NULL, &task_attr );
}

//*************************************************************************************************
// Задача обработки очереди сообщений на выполнение команд полученных по UART1
//*************************************************************************************************
static void TaskCommand( void *argument ) {

    int32_t event;

    for ( ;; ) {
        event = osEventFlagsWait( cmnd_event, EVN_CMND_MASK, osFlagsWaitAny, osWaitForever );
        if ( event & EVN_CMND_EXEC ) {
            //выполнение команды
            ExecCommand( UartBuffer() );
            //разрешаем прием по UART следующей команды
            osEventFlagsSet( uart_event, EVN_UART_START );
           }
        if ( event & EVN_CMND_PROMPT )
            UartSendStr( (char *)msg_prompt );
       }
 }

//*************************************************************************************************
// Обработка команд полученных по UART
//-------------------------------------------------------------------------------------------------
// char *buff - указатель на буфер с командой
//*************************************************************************************************
static void ExecCommand( char *buff ) {

    uint8_t i, cnt_par;

    //разбор параметров команды
    cnt_par = ParseCommand( buff );
    //проверка и выполнение команды
    for ( i = 0; i < SIZE_ARRAY( cmd ); i++ ) {
        if ( strcasecmp( (const char *)&cmd[i].name_cmd, GetParamVal( IND_PAR_CMND ) ) )
            continue;
        UartSendStr( (char *)msg_crlr );
        cmd[i].func( cnt_par, GetParamList() ); //выполнение команды
        UartSendStr( (char *)msg_prompt );
        break;
       }
    if ( i == SIZE_ARRAY( cmd ) ) {
        UartSendStr( (char *)msg_no_command );
        UartSendStr( (char *)msg_prompt );
       }
 }

//*********************************************************************************************
// Вывод статистики по задачам
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров включая команду
// char *param     - указатель на список параметров
//*********************************************************************************************
#ifdef DEBUG_TARGET
static void CmndTask( uint8_t cnt_par, char *param ) {

    uint8_t i;
    const char *name;
    osThreadState_t state;
    osPriority_t priority;
    osThreadId_t th_id[20];
    uint32_t cnt_task, stack_space, stack_size; 
    
    //вывод шапки параметров
    UartSendStr( "\r\n   Name thread     Priority  State      Stack Unused\r\n" );
    UartSendStr( (char *)msg_str_delim );
    //заполним весь массив th_id значением NULL
    memset( th_id, 0x00, sizeof( th_id ) );
    cnt_task = osThreadGetCount();
    cnt_task = osThreadEnumerate( &th_id[0], sizeof( th_id )/sizeof( th_id[0] ) );
    for ( i = 0; i < cnt_task; i++ ) {
        state = osThreadGetState( th_id[i] );
        priority = osThreadGetPriority( th_id[i] );
        //https://github.com/ARM-software/CMSIS-FreeRTOS/issues/14
        //https://github.com/ARM-software/CMSIS-FreeRTOS/blob/dd7793adcbea0c3c0f3524f86b031ab88b9e2193/DoxyGen/General/src/cmsis_freertos.txt#L299
        //osThreadGetStackSize is not implemented.
        stack_size = osThreadGetStackSize( th_id[i] );
        stack_space = osThreadGetStackSpace( th_id[i] );
        name = osThreadGetName( th_id[i] );
        if ( name != NULL && strlen( name ) )
            sprintf( buffer, "%2u %-16s    %2u    %-10s %5u %5u\r\n", i + 1, name, priority, TaskStateDesc( state ), stack_size, stack_space );
        else sprintf( buffer, "%2u ID = %-11u    %2u    %-10s %5u %5u\r\n", i + 1, (uint32_t)th_id[i], priority, TaskStateDesc( state ), stack_size, stack_space );
        UartSendStr( buffer );
       }
    UartSendStr( (char *)msg_str_delim );
    sprintf( buffer, "Free heap size: %u of %u bytes.\r\n", xPortGetFreeHeapSize(), configTOTAL_HEAP_SIZE );
    UartSendStr( buffer );
 }
#endif

//*************************************************************************************************
// Вывод параметров настроек контроллера, установка параметров
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndConfig( uint8_t cnt_par, char *param ) {

    char *ptr;
    uint8_t error, ind, bin[sizeof( config.net_key )];
    CANSpeed can_speed;
    UARTSpeed uart_speed;
    bool change = false;
    union {
        float    val_float;
        uint8_t  val_uint8;
        uint16_t val_uint16;
        uint32_t val_uint32;
       } value;

    //установка скорости UART порта отладки
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "uart" ) ) {
        value.val_uint32 = atol( GetParamVal( IND_PARAM2 ) );
        //проверка на допустимые значения скорости UART порта
        if ( CheckBaudRate( value.val_uint32, &uart_speed ) == SUCCESS ) {
            change = true;
            config.debug_speed = uart_speed;
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //установка значения инкремента для счетчика горячей воды
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "cold" ) ) {
        value.val_uint32 = atol( GetParamVal( IND_PARAM2 ) );
        //проверка на максимальное значение инкремента приращения показаний счетчика
        if ( value.val_uint32 <= 1000 ) {
            change = true;
            config.inc_cnt_cold = value.val_uint32;
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //установка значения инкремента для счетчика холодный воды
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "hot" ) ) {
        value.val_uint32 = atol( GetParamVal( IND_PARAM2 ) );
        //проверка на максимальное значение инкремента приращения показаний счетчика
        if ( value.val_uint32 <= 1000 ) {
            change = true;
            config.inc_cnt_hot = value.val_uint32;
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //установка значения инкремента для счетчика питьевой воды
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "filter" ) ) {
        value.val_uint32 = atol( GetParamVal( IND_PARAM2 ) );
        //проверка на максимальное значение инкремента приращения показаний счетчика
        if ( value.val_uint32 <= 1000 ) {
            change = true;
            config.inc_cnt_filter = value.val_uint32;
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //установка скорости RS-485 порта (MODBUS)
    if ( cnt_par == 4 && !strcasecmp( GetParamVal( IND_PARAM1 ), "modbus" ) && !strcasecmp( GetParamVal( IND_PARAM2 ), "speed" ) ) {
        value.val_uint32 = atol( GetParamVal( IND_PARAM3 ) );
        //проверка на допустимое значение скорости UART порта
        if ( CheckBaudRate( value.val_uint32, &uart_speed ) == SUCCESS ) {
            change = true;
            config.modbus_speed = uart_speed;
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //установка адреса ведомого уст-ва MODBUS
    if ( cnt_par == 4 && !strcasecmp( GetParamVal( IND_PARAM1 ), "modbus" ) && !strcasecmp( GetParamVal( IND_PARAM2 ), "id" ) ) {
        if ( StrHexToBin( GetParamVal( IND_PARAM3 ), (uint8_t *)&value.val_uint8, sizeof( value.val_uint8 ) ) == SUCCESS ) {
            //проверка на допустимые значения для ID адреса уст-ва
            if ( value.val_uint8 > 0 && value.val_uint8 < 248  ) {
                change = true;
                config.modbus_id = value.val_uint8;
               }
            else UartSendStr( (char *)msg_err_param );
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //установка идентификатора уст-ва CAN шины
    if ( cnt_par == 4 && !strcasecmp( GetParamVal( IND_PARAM1 ), "can" ) && !strcasecmp( GetParamVal( IND_PARAM2 ), "id" ) ) {
        if ( StrHexToBin( GetParamVal( IND_PARAM3 ), (uint8_t *)&value.val_uint32, sizeof( value.val_uint32 ) ) == SUCCESS ) {
            //после обработки строки в StrHexToBin() данные в uint32_t будут в модели big endian (от старшего к младшему)
            value.val_uint32 = __REV( value.val_uint32 );
            //проверка на допустимые значения для ID адреса уст-ва
            if ( ( config.can_addr == CAN_ADDRESS_11_BIT && value.val_uint32 <= 0x7FF ) || ( config.can_addr == CAN_ADDRESS_29_BIT && value.val_uint32 <= 0x1FFFFFFF ) ) {
                change = true;
                config.can_id = value.val_uint32;
               }
            else UartSendStr( (char *)msg_err_param );
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //установка размера адресации уст-ва CAN шины (11/29 бит CAN 2.0 A/B)
    if ( cnt_par == 4 && !strcasecmp( GetParamVal( IND_PARAM1 ), "can" ) && !strcasecmp( GetParamVal( IND_PARAM2 ), "addr" ) ) {
        value.val_uint32 = atol( GetParamVal( IND_PARAM3 ) );
        //проверка на допустимое значение разрядности ID шины
        if ( value.val_uint8 == 11 || value.val_uint8 == 29 ) {
            change = true;
            config.can_addr = ( value.val_uint8 == 11 ? CAN_ADDRESS_11_BIT : CAN_ADDRESS_29_BIT );
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //установка скорости обмена по CAN шине
    if ( cnt_par == 4 && !strcasecmp( GetParamVal( IND_PARAM1 ), "can" ) && !strcasecmp( GetParamVal( IND_PARAM2 ), "speed" ) ) {
        value.val_uint32 = atol( GetParamVal( IND_PARAM3 ) );
        //проверка на допустимые значения скорости CAN шины
        if ( CheckCanSpeed( value.val_uint32, &can_speed ) == SUCCESS ) {
            change = true;
            config.can_speed = can_speed;
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //установка максимального давления измеряемого датчиком давления
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "pres_max" ) ) {
        value.val_float = atof( GetParamVal( IND_PARAM2 ) );
        //проверка на допустимое значения максимального датчика давления
        if ( value.val_float <= 15 ) {
            change = true;
            config.pressure_max = value.val_float;
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //установка минимального выходного напряжения датчика давления
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "pres_omin" ) ) {
        value.val_float = atof( GetParamVal( IND_PARAM2 ) );
        //проверка на допустимое значение минимального выходного напряжения датчика давления
        if ( value.val_float < config.press_out_max ) {
            change = true;
            config.press_out_min = value.val_float;
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //установка максимального выходного напряжения датчика давления
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "pres_omax" ) ) {
        value.val_float = atof( GetParamVal( IND_PARAM2 ) );
        //проверка на допустимое значение максимального выходного напряжения датчика давления
        if ( value.val_float && value.val_float > config.press_out_min ) {
            change = true;
            config.press_out_max = value.val_float;
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //установка адреса сети в канале
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "panid" ) ) {
        if ( StrHexToBin( GetParamVal( IND_PARAM2 ), (uint8_t *)&value.val_uint16, sizeof( value.val_uint16 ) ) == SUCCESS ) {
            if ( value.val_uint16 <= MAX_NETWORK_PANID ) {
                change = true;
                memcpy( (uint8_t *)&config.net_pan_id, (uint8_t *)&value.val_uint16, sizeof( config.net_pan_id ) );
                config.net_pan_id = __REVSH( config.net_pan_id );
               }
            else UartSendStr( (char *)msg_err_param );
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //установка ключа сети
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "netkey" ) ) {
        if ( StrHexToBin( GetParamVal( IND_PARAM2 ), (uint8_t *)&bin, sizeof( bin ) ) == SUCCESS ) {
            change = true;
            memcpy( config.net_key, bin, sizeof( config.net_key ) );
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //номер устройства в сети 1 - 65535
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "devnumb" ) ) {
        value.val_uint16 = atoi( GetParamVal( IND_PARAM2 ) );
        if ( value.val_uint16 && value.val_uint16 <= MAX_DEVICE_NUMB ) {
            change = true;
            config.dev_numb = value.val_uint16;
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //установка номера группы
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "netgrp" ) ) {
        value.val_uint8 = atoi( GetParamVal( IND_PARAM2 ) );
        if ( value.val_uint8 <= MAX_NETWORK_GROUP ) {
            change = true;
            config.net_group = value.val_uint8;
           }
       }
    //установка адреса шлюза с сети
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "gate" ) ) {
        if ( StrHexToBin( GetParamVal( IND_PARAM2 ), (uint8_t *)&value.val_uint16, sizeof( value.val_uint16 ) ) == SUCCESS ) {
            if ( value.val_uint16 <= MAX_NETWORK_ADDR ) {
                change = true;
                memcpy( (uint8_t *)&config.addr_gate, (uint8_t *)&value.val_uint16, sizeof( config.addr_gate ) );
                config.addr_gate = __REVSH( config.addr_gate );
               }
            else UartSendStr( (char *)msg_err_param );
           }
        else UartSendStr( (char *)msg_err_param );
       }
    //сохранение параметров
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "save" ) ) {
        UartSendStr( (char *)msg_save );
        error = ConfigSave();
        if ( error != HAL_OK ) 
            UartSendStr( ConfigError( error ) );
        else UartSendStr( (char *)msg_ok );
        return;
       }

    sprintf( buffer, "Reading parameters from flash memory: %s\r\n", FlashReadStat() );
    UartSendStr( buffer );
    UartSendStr( (char *)msg_str_delim );
    //вывод значений параметров
    sprintf( buffer, "UART speed: ......................... %u\r\n", UartGetSpeed( (UARTSpeed)config.debug_speed ) );
    UartSendStr( buffer );
    UartSendStr( (char *)msg_str_delim );
    sprintf( buffer, "Cold water meter increment: ......... %u liters/imp\r\n", config.inc_cnt_cold );
    UartSendStr( buffer );
    sprintf( buffer, "Hot water meter increment: .......... %u liters/imp\r\n", config.inc_cnt_hot );
    UartSendStr( buffer );
    sprintf( buffer, "Increment of drinking water meter: .. %u liters/imp\r\n", config.inc_cnt_filter );
    UartSendStr( buffer );
    //параметры датчика давления
    UartSendStr( (char *)msg_str_delim );
    sprintf( buffer, "Maximum measured value of\r\n the pressure sensor: ............... %2.2f atm\r\n", config.pressure_max );
    UartSendStr( buffer );
    sprintf( buffer, "Minimum voltage at\r\n the pressure sensor output: ........ %2.2f\r\n", config.press_out_min );
    UartSendStr( buffer );
    sprintf( buffer, "Maximum voltage at\r\n the pressure sensor output: ........ %2.2f\r\n", config.press_out_max );
    UartSendStr( buffer );
    //параметры радио CAN шины
    UartSendStr( (char *)msg_str_delim );
    if ( config.can_addr == CAN_ADDRESS_11_BIT )
        sprintf( buffer, "CAN identifier: ..................... 0x%03X\r\n", config.can_id );
    else sprintf( buffer, "CAN identifier: ..................... 0x%08X\r\n", config.can_id );
    UartSendStr( buffer );
    sprintf( buffer, "CAN identifier bit length: .......... %u\r\n", config.can_addr == CAN_ADDRESS_11_BIT ? 11 : 29  );
    UartSendStr( buffer );
    sprintf( buffer, "CAN speed: .......................... %u kbit/s\r\n", CanGetParam( (CANSpeed)config.can_speed, CAN_PARAM_SPEED ) );
    UartSendStr( buffer );
    UartSendStr( (char *)msg_str_delim );
    sprintf( buffer, "MODBUS device address: .............. 0x%02X\r\n", config.modbus_id );
    UartSendStr( buffer );
    sprintf( buffer, "MODBUS speed: ....................... %u\r\n", UartGetSpeed( (UARTSpeed)config.modbus_speed ) );
    UartSendStr( buffer );
    //параметры радио модуля (сети)
    UartSendStr( (char *)msg_str_delim );
    sprintf( buffer, "Network PANID: ...................... 0x%04X\r\n", config.net_pan_id );
    UartSendStr( buffer );
    sprintf( buffer, "Network group number: ............... %u\r\n", config.net_group );
    UartSendStr( buffer );
    ptr = buffer;
    ptr += sprintf( ptr, "Network key: ........................ " );
    for ( ind = 0; ind < sizeof( config.net_key ); ind++ )
        ptr += sprintf( ptr, "%02X", config.net_key[ind] );
    ptr += sprintf( ptr, "\r\n" );
    UartSendStr( buffer );
    sprintf( buffer, "Device number on the network: ....... %05u\r\n", config.dev_numb );
    UartSendStr( buffer );
    sprintf( buffer, "Gateway address: .................... 0x%04X\r\n", config.addr_gate );
    UartSendStr( buffer );
    if ( change == true ) {
        //сохранение параметров
        UartSendStr( (char *)msg_save );
        error = ConfigSave();
        if ( error != HAL_OK ) 
            UartSendStr( ConfigError( error ) );
        else UartSendStr( (char *)msg_ok );
       }
 }

//*************************************************************************************************
// Вывод состояния расхода воды, установка начального значения
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndWater( uint8_t cnt_par, char *param ) {

    FramStatus status;
    bool change = false; //признак новых данных
    
    //выборка параметров текущего расхода воды
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "cold" ) ) {
        //установка значения текущего расхода холодной воды
        change = true;
        curr_data.count_cold = atol( GetParamVal( IND_PARAM2 ) );
       }
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "hot" ) ) {
        //установка значения текущего расхода горячей воды
        change = true;
        curr_data.count_hot = atol( GetParamVal( IND_PARAM2 ) );
       }
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "filter" ) ) {
        //установка значения текущего расхода питьевой воды
        change = true;
        curr_data.count_filter = atol( GetParamVal( IND_PARAM2 ) );
       }
    if ( ( cnt_par == 2 || cnt_par == 3 ) && !strcasecmp( GetParamVal( IND_PARAM1 ), "log" ) ) {
        //вывод событий из архива
        WaterLog( atoi( GetParamVal( IND_PARAM2 ) ) );
        return;
       }
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "addr" ) && atol( GetParamVal( IND_PARAM2 ) ) == 0 ) {
        //установка значения адреса следующего блока для записи события
        change = true;
        curr_data.next_addr = FRAM_ADDR_LOG;
       }
    //вывод текущих значений 
    sprintf( buffer, "Cold water meter values: ...... %u.%03u\r\n", curr_data.count_cold/1000, curr_data.count_cold%1000 );
    UartSendStr( buffer );
    sprintf( buffer, "Hot water meter values: ....... %u.%03u\r\n", curr_data.count_hot/1000, curr_data.count_hot%1000 );
    UartSendStr( buffer );
    sprintf( buffer, "Drinking water meter values: .. %u.%03u\r\n", curr_data.count_filter/1000, curr_data.count_filter%1000 );
    UartSendStr( buffer );
    sprintf( buffer, "Cold water pressure: .......... %u.%u atm\r\n", pressure_cold/100, pressure_cold%100 );
    UartSendStr( buffer );
    sprintf( buffer, "Hot water pressure: ........... %u.%u atm\r\n", pressure_hot/100, pressure_hot%100 );
    UartSendStr( buffer );
    sprintf( buffer, "Leakage sensor power check: ... %s\r\n", DC12VStatus() == DC12V_OK ? "OK" : "ALARM" );
    UartSendStr( buffer );
    sprintf( buffer, "Leak sensor status #1: ........ %s\r\n", LeakStatus( LEAK1 ) == LEAK_NO ? "OK" : "WATER LEAK" );
    UartSendStr( buffer );
    sprintf( buffer, "Leak sensor status #2: ........ %s\r\n", LeakStatus( LEAK2 ) == LEAK_NO ? "OK" : "WATER LEAK" );
    UartSendStr( buffer );
    sprintf( buffer, "Address of the next log entry:  0x%04X\r\n", curr_data.next_addr );
    UartSendStr( buffer );
    if ( change == true ) {
        //сохранение данных
        status = FramSaveData( CURRENT_DATA, (uint8_t *)&curr_data, sizeof( curr_data ) );
        UartSendStr( FramErrorDesc( status ) );
       }
 }

//*************************************************************************************************
// Вывод состояния/управление электроприводов
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndValve( uint8_t cnt_par, char *param ) {

    int opn, cls;
    
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "cold" ) ) {
        //управление электроприводом холодной воды
        opn = strcasecmp( GetParamVal( IND_PARAM2 ), "opn" );
        cls = strcasecmp( GetParamVal( IND_PARAM2 ), "cls" );
        if ( !opn && cls )
            osEventFlagsSet( valve_event, EVN_VALVE_COLD_OPN );
        if ( opn && !cls )
            osEventFlagsSet( valve_event, EVN_VALVE_COLD_CLS );
        return;
       }
    if ( cnt_par == 3 && !strcasecmp( GetParamVal( IND_PARAM1 ), "hot" ) ) {
        //управление электроприводом горячей воды
        opn = strcasecmp( GetParamVal( IND_PARAM2 ), "opn" );
        cls = strcasecmp( GetParamVal( IND_PARAM2 ), "cls" );
        if ( !opn && cls )
            osEventFlagsSet( valve_event, EVN_VALVE_HOT_OPN );
        if ( opn && !cls )
            osEventFlagsSet( valve_event, EVN_VALVE_HOT_CLS );
        return;
       }
    //вывод текущих состояний
    sprintf( buffer, "Cold water tap drive status: .. %s\r\n", ValveStatusDesc( VALVE_COLD ) );
    UartSendStr( buffer );
    sprintf( buffer, "Cold water valve error: ....... %s\r\n", ValveErrorDesc( VALVE_COLD ) );
    UartSendStr( buffer );
    sprintf( buffer, "Hot water tap drive status: ... %s\r\n", ValveStatusDesc( VALVE_HOT ) );
    UartSendStr( buffer );
    sprintf( buffer, "Hot water valve error: ........ %s\r\n", ValveErrorDesc( VALVE_HOT ) );
    UartSendStr( buffer );
 }

//*************************************************************************************************
// Вывод номера и даты версий: FirmWare, RTOS, HAL
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndVersion( uint8_t cnt_par, char *param ) {

    char val[32];
    osVersion_t osv;
    char infobuf[40];
    osStatus_t status;

    sprintf( buffer, "FirmWare version: .... %s\r\n", FWVersion( GetFwVersion() ) );
    UartSendStr( buffer );
    sprintf( buffer, "FirmWare date build: . %s\r\n", FWDate( GetFwDate() ) );
    UartSendStr( buffer );
    sprintf( buffer, "FirmWare time build: . %s\r\n", FWTime( GetFwTime() ) );
    UartSendStr( buffer );
    UartSendStr( (char *)msg_crlr );
    sprintf( buffer, "The HAL revision: .... %s\r\n", FWVersion( HAL_GetHalVersion() ) );
    UartSendStr( buffer );
    status = osKernelGetInfo( &osv, infobuf, sizeof( infobuf ) );
    if ( status == osOK ) {
        sprintf( buffer, "Kernel Information: .. %s\r\n", infobuf );
        UartSendStr( buffer );
        sprintf( buffer, "Kernel Version: ...... %s\r\n", VersionRtos( osv.kernel, val ) );
        UartSendStr( buffer );
        sprintf( buffer, "Kernel API Version: .. %s\r\n", VersionRtos( osv.api, val ) );
        UartSendStr( buffer );
       }
}

//*************************************************************************************************
// Вывод/установка текущего значения дата/время
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndDate( uint8_t cnt_par, char *param ) {

    if ( cnt_par == 2 ) {
        if ( DateSet( GetParamVal( IND_PARAM1 ) ) == ERROR ) {
            UartSendStr( (char *)msg_err_param );
            return;
           }
       }
    DateTimeStr( buffer, MASK_DATE );
    UartSendStr( buffer );
}

//*************************************************************************************************
// Вывод/установка текущего значения времени
// Установка времени, входной формат HH:MI:SS
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndTime( uint8_t cnt_par, char *param ) {

    if ( cnt_par == 2 ) {
        if ( TimeSet( GetParamVal( IND_PARAM1 ) ) == ERROR ) {
            UartSendStr( (char *)msg_err_param );
            return;
           }
       }
    DateTimeStr( buffer, MASK_TIME );
    UartSendStr( buffer );
 }

//*************************************************************************************************
// Вывод текущего значения дата/время
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndDateTime( uint8_t cnt_par, char *param ) {

    DateTimeStr( buffer, MASK_DATE_TIME );
    UartSendStr( buffer );
}

//*************************************************************************************************
// Управление FRAM памятью
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndFram( uint8_t cnt_par, char *param ) {

    uint8_t cnt;
    FramStatus status;
    
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "clr" ) ) {
        //очистка FRAM памяти
        FramClear();
        UartSendStr( (char *)msg_ok );
        //сброс адреса хранения следующей записи журнала
        curr_data.next_addr = FRAM_ADDR_LOG;
        //сохранение данных
        status = FramSaveData( CURRENT_DATA, (uint8_t *)&curr_data, sizeof( curr_data ) );
        UartSendStr( FramErrorDesc( status ) );
        return;
       }
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "chk" ) ) {
        //проверка данных в FRAM памяти
        FramCheck();
        UartSendStr( (char *)msg_ok );
        return;
       }
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "test" ) ) {
        //тест FRAM памяти
        FramTest();
        UartSendStr( (char *)msg_ok );
        return;
       }
    if ( cnt_par == 1 || cnt_par == 2 ) {
        //вывод дампа FRAM памяти
        cnt = atoi( GetParamVal( IND_PARAM1 ) );
        FramHexDump( cnt );
        UartSendStr( (char *)msg_ok );
        return;
       }
    UartSendStr( (char *)msg_err_param );
 }

//*************************************************************************************************
// Вывод статистики обмена данными CAN/MODBUS/ZIGBEE
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndStat( uint8_t cnt_par, char *param ) {

    char str[120];
    uint8_t i, cnt;

    //источник перезапуска контроллера
    sprintf( str, "Source reset: %s\r\n", ResetSrcDesc() );
    UartSendStr( str );
    //дата/время включения контроллера
    sprintf( str, "Date/time of activation: %02u.%02u.%04u  %02u:%02u:%02u\r\n\r\n", 
             curr_data.day, curr_data.month, curr_data.year, curr_data.hour, curr_data.min, curr_data.sec );
    UartSendStr( str );
    //статистика протокола MODBUS
    UartSendStr( "Modbus statistics ...\r\n" );
    UartSendStr( (char *)msg_str_delim );
    cnt = ModBusErrCnt( MBUS_REQUEST_OK );
    for ( i = 0; i < cnt; i++ ) {
        sprintf( buffer, "%s\r\n", ModBusErrCntDesc( (ModBusError)i, str ) );
        UartSendStr( buffer );
       }
    //статистика протокола CAN
    UartSendStr( "\r\nCAN statistics ...\r\n" );
    UartSendStr( (char *)msg_str_delim );
    cnt = CANErrCnt( CAN_ERR_OK );
    for ( i = 0; i < cnt; i++ ) {
        sprintf( buffer, "%s\r\n", CANErrCntDesc( (CANError)i, str ) );
        UartSendStr( buffer );
       }
    //статистика протокола ZigBee
    UartSendStr( "\r\nZigBee statistics ...\r\n" );
    UartSendStr( (char *)msg_str_delim );
    cnt = ZBErrCnt( ZB_ERROR_OK );
    for ( i = 0; i < cnt; i++ ) {
        sprintf( buffer, "%s\r\n", ZBErrCntDesc( (ZBErrorState)i, str ) );
        UartSendStr( buffer );
       }
 }

//*************************************************************************************************
// Вывод дампа FLASH памяти (хранение параметров)
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
#ifdef DEBUG_TARGET
static void CmndFlash( uint8_t cnt_par, char *param ) {
    
    uint8_t dw, dwi, dw_cnt, data_hex[16];
    uint32_t *source_addr, *dest_addr;

    source_addr = (uint32_t *)FLASH_DATA_ADDRESS;
    dw_cnt = sizeof( FLASH_DATA )/sizeof( uint32_t );
    for ( dw = 0; dw < dw_cnt; dw += 4 ) {
        dest_addr = (uint32_t *)&data_hex;
        //читаем значение только как WORD (по 4 байта)
        for ( dwi = 0; dwi < 4; dwi++, dest_addr++, source_addr++ )
            *dest_addr = *(__IO uint32_t *)source_addr;
        //вывод дампа
        DataHexDump( data_hex, HEX_32BIT_ADDR, (uint32_t)source_addr - 16, buffer );
        UartSendStr( buffer );
       }
    UartSendStr( (char *)msg_crlr );
 }
#endif

//*************************************************************************************************
// Вывод протокола аварийных событий и логирования данных.
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_view - кол-во событий для вывода
//*************************************************************************************************
static void WaterLog( uint8_t cnt_view ) {

    WATER_LOG wtr_log;
    uint16_t addr, rec, cnt;

    cnt = MakeSort( cnt_view );
    if ( !cnt ) {
        UartSendStr( "Records not found.\r\n" );
        return;
       }
    sprintf( buffer, "Records uploaded: %u\r\n\r\n", cnt );
    UartSendStr( buffer );
    if ( !cnt_view )
        cnt_view = cnt;
    for ( rec = 0; rec < cnt && cnt_view; rec++ ) {
        //выборка адреса данных из FRAM по индексу
        addr = GetAddrSort( rec );
        if ( !addr )
            continue;
        //чтение данных из FRAM
        memset( (uint8_t *)&wtr_log, 0x00, sizeof( wtr_log ) );
        if ( FramReadData( addr, (uint8_t *)&wtr_log, sizeof( wtr_log ) ) != FRAM_OK )
            continue;
        cnt_view--;
        //дата время
        sprintf( buffer, "%s #%02u (0x%04X): %02u.%02u.%04u %02u:%02u:%02u\r\n", wtr_log.type_event == EVENT_DATA ? "Event" : "ALARM",
                 rec + 1, addr, wtr_log.day, wtr_log.month, wtr_log.year, wtr_log.hour, wtr_log.min, wtr_log.sec );
        UartSendStr( buffer );
        UartSendStr( (char *)msg_str_delim );
        //расход воды по счетчикам, давление воды
        sprintf( buffer, "Cold:  ........... %5.3f  Pressure: ... %2.1f\r\n", ((float)wtr_log.count_cold)/1000, ((float)wtr_log.pressr_cold)/100 );
        UartSendStr( buffer );
        sprintf( buffer, "Hot: ............. %5.3f  Pressure: ... %2.1f\r\n", ((float)wtr_log.count_hot)/1000, ((float)wtr_log.pressr_hot)/100 );
        UartSendStr( buffer );
        sprintf( buffer, "Filter: .......... %5.3f\r\n", ((float)wtr_log.count_filter)/1000 );
        UartSendStr( buffer );
        //состояние электроприводов
        sprintf( buffer, "Valve cold: ...... %s %s\r\n", ValveStatusDesc( VALVE_COLD ), ValveErrorDesc( VALVE_COLD ) );
        UartSendStr( buffer );
        sprintf( buffer, "Valve hot: ....... %s %s\r\n", ValveStatusDesc( VALVE_HOT ), ValveErrorDesc( VALVE_HOT ) );
        UartSendStr( buffer );
        //состояние датчиков утечки
        sprintf( buffer, "DC12V: ........... %s\r\n", wtr_log.dc12_chk == DC12V_OK ? "OK " : "ALARM" );
        UartSendStr( buffer );
        sprintf( buffer, "Leak sensor #1: .. %s\r\n", wtr_log.leak1 == LEAK_NO ? "OK " : "ALARM" );
        UartSendStr( buffer );
        sprintf( buffer, "Leak sensor #2: .. %s\r\n", wtr_log.leak2 == LEAK_NO ? "OK " : "ALARM" );
        UartSendStr( buffer );
        UartSendStr( (char *)msg_crlr );
       }
 }

//*************************************************************************************************
// Управление радио модулем ZigBee
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndZigBee( uint8_t cnt_par, char *param ) {

    static ZBErrorState state;
    
    if ( cnt_par == 1 ) {
        UartSendStr( (char *)msg_err_param );
        return;
       }
    //аппаратный перезапуск модуля
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "res" ) )
        state = ZBControl( ZB_DEV_RESET );
    //программный перезапуск модуля
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "init" ) )
        state = ZBControl( ZB_DEV_INIT );
    //переподключение к сети
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "net" ) )
        state = ZBControl( ZB_NET_RESTART );
    //запись конфигурации в радио модуль
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "save" ) )
        state = ZBControl( ZB_SAVE_CONFIG );
    //проверка конфигурации радио модуля
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "chk" ) )
        ZBCheckConfig();
    //чтение и вывод конфигурации
    if ( cnt_par == 2 && !strcasecmp( GetParamVal( IND_PARAM1 ), "cfg" ) ) {
        state = ZBControl( ZB_READ_CONFIG );
        if ( state == ZB_ERROR_OK )
            ZBConfig(); //вывод параметров конфигурации
       }
    sprintf( buffer, "%s\r\n", ZBErrDesc( state ) );
    UartSendStr( buffer );
 }

//*************************************************************************************************
// Перезапуск контроллера
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
#ifdef DEBUG_TARGET
static void CmndReset( uint8_t cnt_par, char *param ) {

    NVIC_SystemReset();
}
#endif

//*************************************************************************************************
// Тестовая имитация срабатывания датчиков утечки
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
#ifdef DEBUG_TARGET
static void CmndLog( uint8_t cnt_par, char *param ) {

    if ( atoi( GetParamVal( IND_PARAM1 ) ) == 1 )
        osEventFlagsSet( water_event, EVN_WTR_LEAK1 );
    if ( atoi( GetParamVal( IND_PARAM1 ) ) == 2 )
        osEventFlagsSet( water_event, EVN_WTR_LEAK2 );
 }
#endif

//*************************************************************************************************
// Вывод подсказки по доступным командам
//-------------------------------------------------------------------------------------------------
// uint8_t cnt_par - кол-во параметров
// char *param     - указатель на список параметров
//*************************************************************************************************
static void CmndHelp( uint8_t cnt_par, char *param ) {

    UartSendStr( (char *)help );
 }

//*************************************************************************************************
// Расшифровка версии RTOS в текстовый буфер
//-------------------------------------------------------------------------------------------------
// uint32_t version - значение версии в формате: mmnnnrrrr dec
// char *str        - указатель на строку для размещения результата
// return           - указатель на строку с расшифровкой в формате major.minor.rev
//*************************************************************************************************
static char *VersionRtos( uint32_t version, char *str ) {

    uint32_t major, minor, rev;
    
    rev = version%10000;
    version /= 10000;
    minor = version%1000;
    major = version/1000;
    sprintf( str, "%u.%u.%u", major, minor, rev );
    return str; 
 }

//*************************************************************************************************
// Возвращает расшифровку статуса задачи
//-------------------------------------------------------------------------------------------------
// osThreadState_t state - код статуса
//*************************************************************************************************
#ifdef DEBUG_TARGET
static char *TaskStateDesc( osThreadState_t state ) {

    if ( state == osThreadInactive )
        return state_name[0];
    if ( state == osThreadReady )
        return state_name[1];
    if ( state == osThreadRunning )
        return state_name[2];
    if ( state == osThreadBlocked )
        return state_name[3];
    if ( state == osThreadTerminated )
        return state_name[4];
    if ( state == osThreadError )
        return state_name[5];
    return NULL;
 }
#endif

//*************************************************************************************************
// Вывод одной строки дампа данных в HEX формате 
// 0x0000: 00 00 00 00 00 00 00 00  00 00 00 00 00 00 00 00  ................
//-------------------------------------------------------------------------------------------------
// uint8_t *data           - указатель на массив данных 
// HEX_TYPE_ADDR type_addr - разрядность выводимого адреса 16/32 бита
// uint16_t addr           - адрес для вывода 
// char *buff              - указатель на буфер для размещения результата
//*************************************************************************************************
void DataHexDump( uint8_t *data, HEX_TYPE_ADDR type_addr, uint32_t addr, char *buff ) {

    char *ptr, ch;
    uint8_t offset;

    ptr = buff;
    //вывод адреса строки
    if ( type_addr == HEX_16BIT_ADDR )
        ptr += sprintf( ptr, "0x%04X: ", addr );
    else ptr += sprintf( ptr, "0x%08X: ", addr );
    //вывод HEX данных
    for ( offset = 0; offset < 16; offset++ ) {
        ch = *( data + offset );
        //выводим HEX коды
        if ( ( offset & 0x07 ) == 7 )
            ptr += sprintf( ptr, "%02X  ", ch );
        else ptr += sprintf( ptr, "%02X ", ch );
       }
    //выводим символы
    for ( offset = 0; offset < 16; offset++ ) {
        ch = *( data + offset );
        if ( ch >= 32 && ch < 127 )
            ptr += sprintf( ptr, "%c", ch );
        else ptr += sprintf( ptr, "." );
       }
    ptr += sprintf( ptr, "\r\n" );
 }

//*************************************************************************************************
// Преобразует строку с данными в формате HEX в формат BIN
//-------------------------------------------------------------------------------------------------
// char *str        - указатель на исходную строку с HEX значениями
// uint8_t *hex     - указатель для массив для размещения результата
// uint8_t size     - размер переменной для размещения результата
// return = SUCCESS - преобразование выполнено без ошибок
//        = ERROR   - преобразование не выполнено, есть не допустимые символы
//*************************************************************************************************
static ErrorStatus StrHexToBin( char *str, uint8_t *hex, uint8_t size ) {

    uint8_t high, low, len;
    
    len = strlen( str );
    //проверка на длину строки и кратность длины строки = 2 
    if ( ( len/2 ) > size || ( len & 0x01 ) )
        return ERROR;
    while ( *str ) {
        high = low = 0;
        if ( HexToBin( str, &high ) == ERROR )
            return ERROR;
        if ( *++str ) {
            //не конец строки
            if ( HexToBin( str, &low ) == ERROR )
                return ERROR;
            str++; //переход на следующие два байта
           }
        high <<= 4;
        high |= low;
        *hex++ = high;
       }
    return SUCCESS;
 }

//*************************************************************************************************
// Преобразует символ в формате HEX в формат BIN
//-------------------------------------------------------------------------------------------------
// char *ptr        - указатель на HEX символ
// uint8_t *bin     - указатель на переменную для размещения результата
// return = SUCCESS - преобразование выполнено без ошибок
//        = ERROR   - преобразование не выполнено, есть не допустимые символы
//*************************************************************************************************
static ErrorStatus HexToBin( char *ptr, uint8_t *bin ) {

    *bin = 0;
    if ( *ptr >= '0' && *ptr <= '9' ) {
        *bin = *ptr - '0';
        return SUCCESS;
       }
    if ( *ptr >= 'a' && *ptr <= 'f' ) {
        *bin = *ptr - 87;
        return SUCCESS;
       }
    if ( *ptr >= 'A' && *ptr <= 'F' ) {
        *bin = *ptr - 55;
        return SUCCESS;
       }
    return ERROR;
 }
