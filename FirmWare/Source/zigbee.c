
//*************************************************************************************************
//
// Управление обменом c модулем ZigBee
// 
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "main.h"
#include "data.h"
#include "valve.h"
#include "uart.h"
#include "crc16.h"
#include "events.h"
#include "message.h"
#include "parse.h"
#include "config.h"
#include "xtime.h"
#include "zigbee.h"

#define DEBUG_ZIGBEE                        //вывод принятых/отправленных пакетов в HEX формате

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern CONFIG config;
extern CURR_DATA curr_data;
extern VALVE  valve_data;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;
extern uint16_t pressure_cold, pressure_hot;

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
ZB_CONFIG zb_cfg;
osEventFlagsId_t zb_flow = NULL, zb_ctrl = NULL;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define BUFFER_CMD              80          //размер буфера для команд управления
#define BUFFER_SIZE             80          //размер буферов приема/передачи

#define ZB_CMD_SEND_DATA        0xFC        //код команды передачи данных
#define ZB_SYS_CONFIG           0xFB        //ответ чтения конфигурации

#define OFFSET_DATA_SIZE        1           //смещение для размещения размера пакета
#define ZB_ADDR_SIZE            2           //размер адреса шлюза (координатора)
#define ZB_MODE_SIZE            2           //кол-во байт определяюшие тип передачи пакета

#define TIME_DELAY_RESET        100         //задержка восстановления сигнала сброса (msec)
#define TIME_DELAY_CHECK        700         //задержка проверки включения ZigBee модуля (msec)

#define TIME_WAIT_ACK           5000        //время ожидания ответа подтверждения получения данных(msec)
#define TIME_NOWAIT_ACK         0           //без ожидания подтверждения получения данных

#define OFFSET_CFG_DATA         3           //смещения для размещения параметров
                                            //конфигурации ZigBee модуля

//Структура для команд управления ZigBee модулем
typedef struct {
    ZBCmnd       id_command;                //ID команды
    uint8_t      code_command[4];           //коды команды
    uint16_t     time_answer;               //время ожидаемого ответа (msec)
    ZBErrorState (*func_exec)( void *ptr ); //указатель на функцию исполнения команды
 } ZB_COMMAND;

//Структура стандартных ответов ZigBee модуля
typedef struct {
    ZBAnswer     id_answer;                 //ID ответа
    uint8_t      code_answer[2];            //коды ответа
 } ZB_ANSWER;

//Расшифровка результата выполнения команд
static char * const error_descr[] = {
    "OK",                                   //ошибок нет
    "Error in command data",                //ошибка в данных команды
    "UART transmission error",              //ошибка передачи данных
    "Module no running",                    //модуль не включен
    "No network",                           //нет сети
    "Command undefined",                    //команда не определена
    "Timed out for response",               //вышло время ожидания ответа на команду
    "Data error",                           //ошибка в данных при вызове функции
    "No confirmation of receipt of data",   //нет подтверждения от координатора получения пакета данных
    "Checksum error",                       //ошибка контрольной суммы в полученном пакете данных
    "Device number error",                  //ошибка в номере устройства (номер в настройках контроллера 
                                            //не соответствует номеру полученному в пакете данных)
    "Device address error"                  //ошибка в адресе (адрес, присвоенный при подключении к 
                                            //сети не соответствует адресу полученному в пакете)
 };

static char * const dev_type[] = {
    "Coordinator", "Router", "Terminal"
 };

static char * const nwk_state[] = {
    "No network", "Network exists"
 };

#if defined( DEBUG_ZIGBEE ) && defined( DEBUG_TARGET )
static char * const desc_answer[] = {
    "ANSWER_ERROR", 
    "ANSWER_BLD_NET", 
    "ANSWER_JOIN_NET", 
    "ANSWER_NO_NET", 
    "ANSWER_SET_CONFIG", 
    "ANSWER_RESTART",
    "ANSWER_CFG_FACTORY",
    "ANSWER_NET_RESTART",
    "ANSWER_UNDEF"
 };
#endif

static char * const txpower[] = { "-3/16/20", "-1.5/17/22", "0/19/24", "2.5/20/26", "4.5/20/27" };

//*************************************************************************************************
// Прототипы локальных функций вызываемые по ссылке
//*************************************************************************************************
static ZBErrorState SetCfg( void *ptr );
static ZBErrorState Command( void *ptr );

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static char str[100];
#if defined( DEBUG_ZIGBEE ) && defined( DEBUG_TARGET )
static char str2[180];
#endif
static bool time_out = false;

static ZBCmnd last_cmnd = ZB_NO_COMMAND;
static osTimerId_t timer_res, timer_ans, timer_chk;
static osMutexId_t zb_mutex = NULL;
static osSemaphoreId_t sem_send = NULL, sem_ans = NULL, sem_res = NULL;

static uint16_t recv_ind = 0;
static uint32_t send_cnt = 0, recv_cnt = 0;
static uint32_t error_cnt[SIZE_ARRAY( error_descr )]; //счетчики ошибок
static uint8_t recv, buff_data[BUFFER_CMD]; 
static uint8_t recv_buff[BUFFER_SIZE], send_buff[BUFFER_SIZE];

//Набор команд управления модулем ZigBee
static ZB_COMMAND zb_cmd[] = {
    //код команды, коды команды, время ожидания ответа, функция вызова
    { ZB_READ_CONFIG,   { 0xFE, 0x01, 0xFE, 0xFF }, 100, Command }, //чтение конфигурации
    { ZB_SAVE_CONFIG,   { 0xFD, 0x2E, 0xFE, 0xFF }, 200, SetCfg  }, //запись конфигурации
    { ZB_DEV_INIT,      { 0xFD, 0x01, 0x12, 0xFF }, 100, Command }, //перезапуск модуля
    { ZB_DEV_FACTORY,   { 0xFD, 0x01, 0x13, 0xFF }, 100, Command }, //установка заводских настроек
    { ZB_NET_RESTART,   { 0xFD, 0x01, 0x14, 0xFF }, 100, Command }  //переподключение к сети
   };

//Набор ответов модуля ZigBee (для HEX режима обмена данными)
static ZB_ANSWER zb_answr[] = {
    ZB_ANSWER_ERROR,         { 0xF7, 0xFF },    //ошибка команды
    ZB_ANSWER_BLD_NET,       { 0xFF, 0xFF },    //Координатор запросил информацию при установлении сети
    ZB_ANSWER_JOIN_NET,      { 0xFF, 0xAA },    //выполнено  подключение к сети
    ZB_ANSWER_NO_NET,        { 0xFF, 0x00 },    //сеть потеряна
    ZB_ANSWER_SET_CONFIG,    { 0xFA, 0xFE },    //запись конфигурации выполнена
    ZB_ANSWER_RESTART,       { 0xFA, 0x12 },    //модуль ZigBee перезапущен
    ZB_ANSWER_CFG_FACTORY,   { 0xFA, 0x13 },    //сброс настроек выполнен
    ZB_ANSWER_NET_RESTART,   { 0xFA, 0x14 }     //модуль переподключен к сети
 };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void TaskZBFlow( void *pvParameters );
static void TaskZBCtrl( void *pvParameters );
static void Timer1Callback( void *arg );
static void Timer2Callback( void *arg );
static void Timer3Callback( void *arg );
static ZBErrorState SendData( ZBCmnd cmnd, uint8_t *data, uint8_t len, uint16_t timeout );
static ErrorStatus DevStatus( ZBDevState type );
static ErrorStatus CheckSysAnswer( uint8_t *answer, uint8_t len );
static ZBAnswer CheckAnswer( uint8_t *answer, uint8_t len );
static ZBErrorState CheckRecv( ZBCmnd cmnd, uint8_t *answer, uint8_t len );
static void ClearRecv( void );
static void ErrorClr( void );

static char *DevType( ZBDevType dev );
static char *NwkState( ZBNetState state );
static char *TxPower( ZBTxPower id_pwr );

#if defined( DEBUG_ZIGBEE ) && defined( DEBUG_TARGET )
static void SendDebug( char type, uint8_t *data, uint8_t len );
static char *AnswDesc( ZBAnswer id_answ );
#endif

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t task1_attr = {
    .name = "ZBFlow", 
    .stack_size = 704,
    .priority = osPriorityNormal
 };

static const osThreadAttr_t task2_attr = {
    .name = "ZBCtrl", 
    .stack_size = 704,
    .priority = osPriorityNormal
 };

static const osSemaphoreAttr_t sem1_attr = { .name = "ZBSemSend" };
static const osSemaphoreAttr_t sem2_attr = { .name = "ZBSemAns" };
static const osSemaphoreAttr_t sem3_attr = { .name = "ZBSemRes" };
static const osEventFlagsAttr_t evn1_attr = { .name = "ZBEvents1" };
static const osEventFlagsAttr_t evn2_attr = { .name = "ZBEvents2" };
static const osTimerAttr_t timer1_attr = { .name = "ZBTimer1" };
static const osTimerAttr_t timer2_attr = { .name = "ZBTimer2" };
static const osTimerAttr_t timer3_attr = { .name = "ZBTimer3" };
static const osMutexAttr_t mutex_attr = { .name = "ZBBee", .attr_bits = osMutexPrioInherit };

//*************************************************************************************************
// Инициализация задачи и очереди событий управления модулем ZigBee
//*************************************************************************************************
void ZBInit( void ) {

    ErrorClr();
    //очередь событий
    zb_flow = osEventFlagsNew( &evn1_attr );
    zb_ctrl = osEventFlagsNew( &evn2_attr );
    //таймер интервалов
    timer_res = osTimerNew( Timer1Callback, osTimerOnce, NULL, &timer1_attr );
    timer_ans = osTimerNew( Timer2Callback, osTimerOnce, NULL, &timer2_attr );
    timer_chk = osTimerNew( Timer3Callback, osTimerOnce, NULL, &timer3_attr );
    //семафоры блокировки
    sem_send = osSemaphoreNew( 1, 0, &sem1_attr );
    sem_ans = osSemaphoreNew( 1, 0, &sem2_attr );
    sem_res = osSemaphoreNew( 1, 0, &sem3_attr );
    //мьютех ожидания завершения цикла работы
    zb_mutex = osMutexNew( &mutex_attr );
    //создаем задачу
    osThreadNew( TaskZBFlow, NULL, &task1_attr );
    osThreadNew( TaskZBCtrl, NULL, &task2_attr );
    //т.к. при выполнении HAL_TIM_Base_Start_IT() почти сразу формируется прерывание
    //вызов HAL_TIM_Base_Start_IT() выполняем только один раз, дальнейшее управление
    //TIMER2 выполняется через __HAL_TIM_ENABLE()/__HAL_TIM_DISABLE()
    HAL_TIM_Base_Start_IT( &htim3 );
    //запуск приема
    ClearRecv();
    HAL_UART_Receive_IT( &huart2, (uint8_t *)&recv, sizeof( recv ) );
 }

//*************************************************************************************************
// Задача обработки событий управления модулем ZigBee
//*************************************************************************************************
static void TaskZBCtrl( void *pvParameters ) {

    char *ptr;
    int32_t event;
    uint8_t *data, len, rec;
    ZBErrorState state;
    DATE_TIME *ptr_dtime;
    ErrorStatus stat, rec_rdy;

    //проверка конфигурации модуля ZigBee с задержкой после включения
    if ( !osTimerIsRunning( timer_chk ) )
        osTimerStart( timer_chk, TIME_DELAY_CHECK );
    for ( ;; ) {
        event = osEventFlagsWait( zb_ctrl, EVN_ZC_MASK, osFlagsWaitAny, osWaitForever );
        if ( event & EVN_ZC_CONFIG_CHECK ) {
            //проверка конфигурации при включении
            ZBCheckConfig();
            //инициализация компонентов завершена, вывод приглашения в консоль
            osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
           }
        if ( event & EVN_ZC_NET_LOST ) {
            UartSendStr( "Network lost.\r\n" );
            //обновим параметры (прочитаем) ZigBee модуля
            ZBControl( ZB_READ_CONFIG );
            osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
           }
        if ( event & EVN_ZC_NET_RESTORE ) {
            UartSendStr( "Network restored.\r\n" );
            //обновим параметры (прочитаем) ZigBee модуля
            ZBControl( ZB_READ_CONFIG );
            //после подключения к сети сообщим координатору состояние контроллера
            osEventFlagsSet( zb_ctrl, EVN_ZC_SEND_STATE );
           }
        if ( event & EVN_ZC_SEND_STATE || event & EVN_ZC_IM_HERE ) {
            //данные состояния контроллера
            data = CreatePack( ZB_PACK_STATE, &len );
            if ( data != NULL ) {
                state = ZBSendPack( data, len, TIME_NOWAIT_ACK );
                sprintf( str, "Send state: %s\r\n", ZBErrDesc( state ) );
                UartSendStr( str );
                osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
               }
           }
        if ( event & EVN_ZC_SEND_DATA ) {
            //текущие данные расхода/давления/утечки воды
            data = CreatePack( ZB_PACK_DATA, &len );
            if ( data != NULL ) {
                state = ZBSendPack( data, len, TIME_NOWAIT_ACK );
                sprintf( str, "Send data: %s\r\n", ZBErrDesc( state ) );
                UartSendStr( str );
                osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
               }
           }
        if ( event & EVN_ZC_SEND_WLOG ) {
            //журнальные данные расхода/давления/утечки воды
            while ( true ) {
                //запрос данных
                rec_rdy = GetAddrLog( rec );
                if ( rec_rdy == ERROR )
                    break;
                //формируем пакет
                data = CreatePack( ZB_PACK_WLOG, &len );
                if ( data != NULL ) {
                    state = ZBSendPack( data, len, TIME_WAIT_ACK );
                    sprintf( str, "Send log data: %s\r\n", ZBErrDesc( state ) );
                    UartSendStr( str );
                    osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
                    if ( state != ZB_ERROR_OK )
                        break;
                   }
                else break;
               }
           }
        if ( event & EVN_ZC_SEND_VALVE ) {
            //состояние электроприводов
            data = CreatePack( ZB_PACK_VALVE, &len );
            if ( data != NULL ) {
                state = ZBSendPack( data, len, TIME_NOWAIT_ACK );
                sprintf( str, "Send valve status: %s\r\n", ZBErrDesc( state ) );
                UartSendStr( str );
                osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
               }
           }
        if ( event & EVN_ZC_SEND_LEAKS ) {
            //состояние электроприводов
            data = CreatePack( ZB_PACK_LEAKS, &len );
            if ( data != NULL ) {
                state = ZBSendPack( data, len, TIME_WAIT_ACK );
                sprintf( str, "Send status leaks: %s\r\n", ZBErrDesc( state ) );
                UartSendStr( str );
                osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
               }
           }
        if ( event & EVN_ZC_SYNC_DTIME ) {
            //синхронизация даты/времени
            ptr_dtime = GetAddrDtime();
            if ( ptr_dtime != NULL )
                stat = SetTimeDate( ptr_dtime );
                ptr = str;
                ptr += sprintf( ptr, "Date/Time synchronization: " );
                ptr += sprintf( ptr, "%02d.%02d.%04d ", ptr_dtime->day, ptr_dtime->month, ptr_dtime->year );
                ptr += sprintf( ptr, "%02d:%02d:%02d ", ptr_dtime->hour, ptr_dtime->min, ptr_dtime->sec );
                ptr += sprintf( ptr, " %s\r\n", stat == SUCCESS ? "OK" : "Error" );
                UartSendStr( str );
                osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
           }
       }
 }

//*************************************************************************************************
// CallBack функция таймера, задержка проверки параметров ZigBee модуля
//*************************************************************************************************
static void Timer3Callback( void *arg ) {

    osEventFlagsSet( zb_ctrl, EVN_ZC_CONFIG_CHECK );
 }

//*************************************************************************************************
// Задача обработки событий управления обменом данными UART <-> ZigBee модуля
//*************************************************************************************************
static void TaskZBFlow( void *pvParameters ) {

    int32_t event;

    for ( ;; ) {
        event = osEventFlagsWait( zb_flow, EVN_ZB_MASK, osFlagsWaitAny, osWaitForever );
        if ( event & EVN_ZB_RECV_CHECK ) {
            //получен ответ/данные от ZigBee модуля, проверка наличия ответов:
            //ZB_ANSWER_JOIN_NET, ZB_ANSWER_BLD_NET, ZB_ANSWER_NO_NET
            if ( CheckSysAnswer( recv_buff, recv_ind ) == SUCCESS ) {
                ClearRecv();
                #if defined( DEBUG_ZIGBEE ) && defined( DEBUG_TARGET )
                UartSendStr( "Answer SYS OK\r\n" );
                osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
                #endif
               }
            else {
                if ( last_cmnd == ZB_NO_COMMAND ) {
                    //проверка на запросы: ZB_PACK_TIMEDATE, ZB_PACK_REQ_STATE, ZB_PACK_REQ_DATA, ZB_PACK_WCTRL
                    if ( CheckPack( recv_buff, recv_ind ) == SUCCESS ) {
                        //отправка на обработку в TaskZBCtrl()
                        recv_cnt++;
                        #if defined( DEBUG_ZIGBEE ) && defined( DEBUG_TARGET )
                        UartSendStr( "CheckPack Success\r\n" );
                        osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
                        #endif
                       }
                    #if defined( DEBUG_ZIGBEE ) && defined( DEBUG_TARGET )
                    else {
                        UartSendStr( "CheckPack Error\r\n" );
                        osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
                       }
                    #endif
                    ClearRecv();
                   }
                //переход на обработку ответа в SendData()/CheckRecv()
                else {
                    osSemaphoreRelease( sem_ans ); 
                    #if defined( DEBUG_ZIGBEE ) && defined( DEBUG_TARGET )
                    UartSendStr( "SemaphoreRelease SEM_ANS +\r\n" );
                    osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
                    #endif
                   }
               }
           }
        if ( event & EVN_ZB_RECV_TIMEOUT ) {
            //время ожидания ответа от ZigBee модуля закончилось, ответ не получен
            time_out = true;
            osSemaphoreRelease( sem_ans );
           }
        if ( event & EVN_ZB_SEND_COMPLT )
            osSemaphoreRelease( sem_send ); //завершение передачи из UART2
       }
 }

//*************************************************************************************************
// CallBack функция таймера, восстановление сигнала сброса
//*************************************************************************************************
static void Timer1Callback( void *arg ) {

    //восстановление сигнала сброса ZigBee модуля
    HAL_GPIO_WritePin( ZB_RES_GPIO_Port, ZB_RES_Pin, GPIO_PIN_SET );
    osSemaphoreRelease( sem_res );
 }

//*************************************************************************************************
// CallBack функция таймера, ожидания ответа ZigBee модуля
//*************************************************************************************************
static void Timer2Callback( void *arg ) {

    //вышло время ожидания ответа ZigBee модуля
    osEventFlagsSet( zb_flow, EVN_ZB_RECV_TIMEOUT );
 }

//*************************************************************************************************
// CallBack функция TIMER3 - пауза между пакетами
//*************************************************************************************************
void ZBCallBack( void ) {

    //выключаем таймер
    __HAL_TIM_DISABLE( &htim3 );
    //сообщим в задачу для дальнейшей обработки принятых данных
    if ( recv_ind )
        osEventFlagsSet( zb_flow, EVN_ZB_RECV_CHECK );
 }

//*************************************************************************************************
// CallBack функция при приеме байта по UART2
//*************************************************************************************************
void ZBRecvComplt( void ) {

    //прием одного байта
    if ( recv_ind < sizeof( recv_buff ) )
        recv_buff[recv_ind++] = recv;
    else ClearRecv(); //переполнение буфера
    //продолжаем прием
    HAL_UART_Receive_IT( &huart2, (uint8_t *)&recv, sizeof( recv ) );
    //если таймер выключен - стартуем один раз
    //при приеме каждого следующего байта - только сброс счетчика
    if ( !( htim3.Instance->CR1 & TIM_CR1_CEN ) )
        __HAL_TIM_ENABLE( &htim3 );
    else __HAL_TIM_SetCounter( &htim3, 0 );
}

//*************************************************************************************************
// CallBack функция, вызывается при завершении передачи из UART2
//*************************************************************************************************
void ZBSendComplt( void ) {

    //osSemaphoreRelease( sem_send );
    osEventFlagsSet( zb_flow, EVN_ZB_SEND_COMPLT );
 }

//*************************************************************************************************
// Управление ZigBee модулем, кроме команды ZB_SEND_DATA, данные передаются через ZBSendPack()
//-------------------------------------------------------------------------------------------------
// ZBCmnd command      - код команды
// return ZBErrorState - результат выполнения
//*************************************************************************************************
ZBErrorState ZBControl( ZBCmnd command ) {

    uint8_t ind;
    ZBErrorState state;
    
    //проверка включенного ZigBee модуля
    if ( DevStatus( ZB_STATUS_RUN ) == ERROR )
        return ZB_ERROR_RUN;
    //блокировка доступа к ZigBee модулю
    osMutexAcquire( zb_mutex, osWaitForever );
    if ( command == ZB_DEV_RESET ) {
        //формируем сигнал сброса ZigBee модуля
        HAL_GPIO_WritePin( ZB_RES_GPIO_Port, ZB_RES_Pin, GPIO_PIN_RESET );
        //задержка на восстановления сигнала на TIME_DELAY_RESET msec
        osTimerStart( timer_res, TIME_DELAY_RESET );
        //ждем завершения сброса ZigBee модуля
        osSemaphoreAcquire( sem_res, osWaitForever );
        //снимаем блокировку доступа к ZigBee модулю
        osMutexRelease( zb_mutex );
        return ZB_ERROR_OK;
       }
    //проверка и выполнение команды
    for ( ind = 0; ind < SIZE_ARRAY( zb_cmd ); ind++ ) {
        if ( zb_cmd[ind].id_command != command )
            continue;
        //выполнение команды через вызов функций: Command(), SetCfg()
        state = zb_cmd[ind].func_exec( &zb_cmd[ind] );
        break;
       }
    //снимаем блокировку доступа к радию модулю
    osMutexRelease( zb_mutex );
    if ( ind == SIZE_ARRAY( zb_cmd ) )
        return ZB_ERROR_CMD;
    return state;
 }

//*************************************************************************************************
// Формирует команды для управления ZigBee модулем (программный перезапуск, переподключение к сети 
// чтение параметров ) вызывается по ссылке
//-------------------------------------------------------------------------------------------------
// uint8_t *ptr        - указатель на ZB_COMMAND
// return ZBErrorState - результат выполнения
//*************************************************************************************************
static ZBErrorState Command( void *ptr ) {

    ZB_COMMAND *zc_ptr;
    ZBErrorState state;

    zc_ptr = (ZB_COMMAND *)ptr;
    memset( buff_data, 0x00, sizeof( buff_data ) );
    memcpy( buff_data, zc_ptr->code_command, sizeof( zc_ptr->code_command ) );
    //передача данных
    state = SendData( zc_ptr->id_command, buff_data, sizeof( zc_ptr->code_command ), zc_ptr->time_answer );
    ZBIncError( state );
    last_cmnd = ZB_NO_COMMAND;
    return state;
 }

//*************************************************************************************************
// Формирует команды записи параметров ZigBee модуля
//-------------------------------------------------------------------------------------------------
// uint8_t *ptr        - указатель на ZB_COMMAND
// return ZBErrorState - результат выполнения
//*************************************************************************************************
static ZBErrorState SetCfg( void *ptr ) {

    uint8_t *dst, *cfg_ptr;
    ZB_COMMAND *zc_ptr;
    ZBErrorState state;
    
    dst = buff_data;
    zc_ptr = (ZB_COMMAND *)ptr;
    //установка параметров, тип уст-ва
    zb_cfg.dev_type = ZB_DEV_TERMINAL;          
    //PAN ID
    cfg_ptr = (uint8_t *)&config.net_pan_id;
    zb_cfg.pan_id[1] = *cfg_ptr++;
    zb_cfg.pan_id[0] = *cfg_ptr;
    //Network group (NET No)
    zb_cfg.group = config.net_group;
    //для выполнения записи, необходимо обнулить: MAC, Network address, MAC coordinator 
    zb_cfg.short_addr[1] = 0x00;
    zb_cfg.short_addr[0] = 0x00;
    memset( zb_cfg.mac_addr, 0x00, sizeof( zb_cfg.mac_addr ) );
    zb_cfg.coor_short_addr[0] = 0x00;
    zb_cfg.coor_short_addr[1] = 0x00;
    memset( zb_cfg.coor_mac_addr, 0x00, sizeof( zb_cfg.coor_mac_addr ) );
    //ключ шифрования
    memcpy( (uint8_t *)&zb_cfg.key, config.net_key, sizeof( zb_cfg.key ) );
    //копируем всю команду
    memset( buff_data, 0x00, sizeof( buff_data ) );
    memcpy( buff_data, zc_ptr->code_command, sizeof( zc_ptr->code_command ) );
    //адрес для добавления данных с учетом смещения
    dst += OFFSET_CFG_DATA;
    //добавляем данные с учетом смещения
    memcpy( dst, (uint8_t *)&zb_cfg, sizeof( zb_cfg ) );
    //смещение для добавления кода завершения команды
    dst += sizeof( zb_cfg );
    //добавляем код завершения команды с учетом добавленных данных
    *dst = *( zc_ptr->code_command + sizeof( zc_ptr->code_command ) - 1 );
    //передача данных
    state = SendData( zc_ptr->id_command, buff_data, sizeof( zc_ptr->code_command ) + sizeof( zb_cfg ), zc_ptr->time_answer );
    ZBIncError( state );
    last_cmnd = ZB_NO_COMMAND;
    return state;
 }

//*************************************************************************************************
// Формирование и отправка пакета данных.
//-------------------------------------------------------------------------------------------------
// uint8_t *data       - указатель на передаваемые данные
// uint8_t len         - размер передаваемых данных
// uint16_t time_answ  - время ожидания ответа
// return ZBErrorState - результат передачи данных
//*************************************************************************************************
ZBErrorState ZBSendPack( uint8_t *data, uint8_t len, uint16_t time_answ ) {

    ZBErrorState state;
    uint8_t *dst, command[] = { ZB_CMD_SEND_DATA, 0x00, ZB_ONDEMAND, ZB_ONDEMAND_ADDRESS };

    //проверка: адреса получателя/размера передаваемых данных
    if ( len > ( sizeof( buff_data ) - sizeof( command ) ) ) {
        ZBIncError( ZB_ERROR_DATA );
        return ZB_ERROR_DATA;
       }
    //проверка включенного ZigBee модуля
    if ( DevStatus( ZB_STATUS_RUN ) == ERROR ) {
        ZBIncError( ZB_ERROR_RUN );
        return ZB_ERROR_RUN;
       }
    //проверка наличия сети ZigBee модуля
    //if ( DevStatus( ZB_STATUS_NET ) == ERROR )
    if ( zb_cfg.nwk_state == ZB_NETSTATE_NO ) {
        ZBIncError( ZB_ERROR_NETWORK );
        return ZB_ERROR_NETWORK;
       }
    send_cnt++; //подсчет отправленных пакетов
    //ставим блокировку доступа к ZigBee модуля
    osMutexAcquire( zb_mutex, osWaitForever );
    //подготовка пакета
    dst = buff_data;
    memset( buff_data, 0x00, sizeof( buff_data ) );
    //копируем в буфер команду
    memcpy( buff_data, command, sizeof( command ) );
    //смещение для добавления адреса получателя
    dst += sizeof( command );
    //адрес получателя из ZB_CONFIG
    *dst++ = zb_cfg.coor_short_addr[0];
    *dst++ = zb_cfg.coor_short_addr[1];
    //добавляем данные
    memcpy( dst, data, len );
    //корректируем параметр "размер блока данных"
    *( buff_data + OFFSET_DATA_SIZE ) = len + ZB_MODE_SIZE + ZB_ADDR_SIZE;
    state = SendData( ZB_SEND_DATA, buff_data, len + sizeof( command ) + ZB_ADDR_SIZE, time_answ );
    ZBIncError( state );
    last_cmnd = ZB_NO_COMMAND;
    //передача завершена, снимаем блокировку доступа
    osMutexRelease( zb_mutex );
    return state;
 }

//*************************************************************************************************
// Отправка данных/команд в ZigBee модуль
//-------------------------------------------------------------------------------------------------
// ZBCmnd cmnd         - код команды
// uint8_t *data       - указатель на передаваемые данные
// uint8_t len         - размер передаваемых данных
// uint16_t timeout    - время ожидания ответа
// return ZBErrorState - результат передачи данных
//*************************************************************************************************
static ZBErrorState SendData( ZBCmnd cmnd, uint8_t *data, uint8_t len, uint16_t timeout ) {

    ZBErrorState state;
    
    last_cmnd = cmnd;
    //проверка параметров вызова
    if ( data == NULL || len > sizeof( send_buff ) )
        return ZB_ERROR_DATA;
    //копируем в буфер данные
    memset( send_buff, 0x00, sizeof( send_buff ) );
    memcpy( send_buff, data, len );
    #if defined( DEBUG_ZIGBEE ) && defined( DEBUG_TARGET )
    SendDebug( 'S', send_buff, len );
    #endif
    ClearRecv();
    osEventFlagsSet( led_event, EVN_LED_ZB_ACTIVE );
    //передача данных
    time_out = false;
    if ( HAL_UART_Transmit_IT( &huart2, send_buff, len ) == HAL_OK )
        osSemaphoreAcquire( sem_send, osWaitForever ); //ждем завершение передачи данных
    else return ZB_ERROR_SEND;
    state = ZB_ERROR_OK;
    if ( timeout ) {
        //таймер ожидания ответа запускаем только при указанном времени ожидания
        osTimerStart( timer_ans, timeout );
        //ждем получения ответа
        osSemaphoreAcquire( sem_ans, osWaitForever );
        osTimerStop( timer_ans );
        //проверка ответа
        state = CheckRecv( cmnd, recv_buff, recv_ind );
       }
    //проверим срабатывание таймера ожидания ответа
    if ( time_out == true )
        return ZB_ERROR_TIMEOUT;
    return state;
 }

//*************************************************************************************************
// Проверка ответов ZigBee модуля для команд: ZB_READ_CONFIG, ZB_SAVE_CONFIG,
// ZB_DEV_INIT, ZB_DEV_FACTORY, ZB_NET_RESTART
//-------------------------------------------------------------------------------------------------
// ZBCmnd cmnd     - код ранее отправленной команды
// uint8_t *answer - указатель на буфер содержащий ответ ZigBee модуля
// uint8_t len     - размер ответа в байтах
// return ZBAnswer - результат проверки ответа
//*************************************************************************************************
static ZBErrorState CheckRecv( ZBCmnd cmnd, uint8_t *answer, uint8_t len ) {

    ZBAnswer answ;
    uint8_t pack_id;
    ZBErrorState state;

    //идентификация ответа
    answ = CheckAnswer( answer, len );
    pack_id = recv_buff[0]; //тип пакета (ответа)
    if ( answ == ZB_ANSWER_UNDEF )
        state = ZB_ERROR_TIMEOUT;
    if ( answ == ZB_ANSWER_ERROR )
        state = ZB_ERROR_EXEC;
    if ( cmnd == ZB_READ_CONFIG && pack_id == ZB_SYS_CONFIG ) {
        //чтение конфигурации модуля
        memcpy( (uint8_t *)&zb_cfg, recv_buff + 1, sizeof( zb_cfg ) );
        state = ZB_ERROR_OK;
       }
    if ( cmnd == ZB_SAVE_CONFIG && answ == ZB_ANSWER_SET_CONFIG )
        state = ZB_ERROR_OK; //запись конфигурации в ZigBee модуль выполнена
    if ( cmnd == ZB_DEV_INIT && answ == ZB_ANSWER_RESTART )
        state = ZB_ERROR_OK; //программный перезапуск ZigBee модуля выполнен
    if ( cmnd == ZB_DEV_FACTORY && answ == ZB_ANSWER_CFG_FACTORY )
        state = ZB_ERROR_OK; //установка заводских настроек ZigBee модуля выполнена
    if ( cmnd == ZB_NET_RESTART && pack_id == ZB_ANSWER_NET_RESTART )
        state = ZB_ERROR_OK; //переподключение к сети ZigBee модуля выполнено
    if ( cmnd == ZB_SEND_DATA && answ == ZB_ANSWER_ACK )
        state = ZB_ERROR_OK; //получено подтверждение передачи пакета
    ClearRecv();
    return state;
 }

//*************************************************************************************************
// Проверка на наличие сообщения ZB_ANSWER_JOIN_NET, ZB_ANSWER_BLD_NET, ZB_ANSWER_NO_NET -
// cообщения удаляются из приемного буфера
//-------------------------------------------------------------------------------------------------
// uint8_t *answer  - указатель на буфер содержащий ответ ZigBee модуля
// uint8_t len      - размер ответа в байтах
// return = SUCCESS - сообщения обнаружено
//        = ERROR   - сообщение не обнаружено в списке стандартных ответов (ZB_ANSWER)
//*************************************************************************************************
static ErrorStatus CheckSysAnswer( uint8_t *answer, uint8_t len ) {

    ZBAnswer answ;

    #if defined( DEBUG_ZIGBEE ) && defined( DEBUG_TARGET )
    SendDebug( 'R', answer, len );
    #endif
    //идентификация ответа
    answ = CheckAnswer( answer, len );
    if ( answ == ZB_ANSWER_UNDEF )
        return ERROR;
    //актуализация состояния подключения к сети
    if ( answ == ZB_ANSWER_JOIN_NET || answ == ZB_ANSWER_BLD_NET ) {
        if ( zb_cfg.nwk_state == ZB_NETSTATE_NO )
            osEventFlagsSet( zb_ctrl, EVN_ZC_NET_RESTORE );
        zb_cfg.nwk_state = ZB_NETSTATE_OK;
        return SUCCESS;
       }
    if ( answ == ZB_ANSWER_NO_NET ) {
        if ( zb_cfg.nwk_state == ZB_NETSTATE_OK )
            osEventFlagsSet( zb_ctrl, EVN_ZC_NET_LOST );
        zb_cfg.nwk_state = ZB_NETSTATE_NO;
        return SUCCESS;
       }
    return ERROR;
 }

//*************************************************************************************************
// Функция проверяет наличие ответов ZigBee модуля из zb_answr[] в т.ч. подтверждение ZB_PACK_ACK
//-------------------------------------------------------------------------------------------------
// uint8_t *answer - указатель на буфер с ответом
// uint8_t len     - размер ответа в байтах
// return ZBAnswer - код ответа
//*************************************************************************************************
static ZBAnswer CheckAnswer( uint8_t *answer, uint8_t len ) {

    ZBAnswer ind;
    uint16_t crc;
    ZBTypePack type;
    ZB_PACK_ACK_DATA ack;

    //первый шаг проверки - фиксированные ответа
    for ( ind = ZB_ANSWER_ERROR; ind < SIZE_ARRAY( zb_answr ); ind++ ) {
        if ( len != sizeof( zb_answr[ind].code_answer ) )
            continue; //сравнивать только ответ заданной длины
        if ( memcmp( answer, zb_answr[ind].code_answer, sizeof( zb_answr[ind].code_answer ) ) == 0 ) {
            #if defined( DEBUG_ZIGBEE ) && defined( DEBUG_TARGET )
            sprintf( str, "CheckAnswer: %s\r\n", AnswDesc( ind ) );
            UartSendStr( str );
            osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
            #endif
            return ind;
           }
       }
    //второй шаг проверки - тип ответа "подтверждение получения данных"
    type = (ZBTypePack)*answer;
    if ( type == ZB_PACK_ACK && len == sizeof( ZB_PACK_ACK_DATA ) ) {
        //пакет подтверждения
        memcpy( (uint8_t *)&ack, answer, sizeof( ack ) );
        //if ( ack.dev_numb == config.dev_numb )
        //КС считаем без полученной КС и net_addr (net_addr не входит в подсчет КС)
        crc = CalcCRC16( (uint8_t *)&ack, sizeof( ack ) - ( sizeof( uint16_t ) * 2 ) );
        if ( ack.crc != crc ) {
            #if defined( DEBUG_ZIGBEE ) && defined( DEBUG_TARGET )
            UartSendStr( "CheckAnswer ACK CRC error\r\n" );
            osEventFlagsSet( cmnd_event, EVN_CMND_PROMPT );
            #endif
            ZBIncError( ZB_ERROR_CRC );
            return ZB_ANSWER_UNDEF;
           }
        else {
            recv_cnt++;
            return ZB_ANSWER_ACK;
           }
       }
    return ZB_ANSWER_UNDEF;
 }

//*************************************************************************************************
// Чтение и установка параметров ZigBee модуля 
// Если параметры модуля не совпадают с параметрами в конфигурации - выполняется установка параметров
//*************************************************************************************************
void ZBCheckConfig( void ) {

    uint8_t *ptr;
    uint16_t cfg_id, cfg_zb;
    ZBErrorState state;
    bool change = false;
    
    //чтение конфигурации
    state = ZBControl( ZB_READ_CONFIG );
    sprintf( str, "ZB: read config ... %s\r\n", ZBErrDesc( state ) );
    UartSendStr( str );
    if ( state != ZB_ERROR_OK )
        return;
    //установка параметров ZigBee модуля
    if ( zb_cfg.dev_type != ZB_DEV_TERMINAL ) {
        change = true;
        zb_cfg.dev_type = ZB_DEV_TERMINAL;
       }
    //PAN ID
    cfg_id = __REVSH( config.net_pan_id );
    cfg_zb = *( (uint16_t *)&zb_cfg.pan_id );
    if ( cfg_id != cfg_zb ) {
        change = true;
        ptr = (uint8_t *)&config.net_pan_id;
        zb_cfg.pan_id[1] = *ptr++;
        zb_cfg.pan_id[0] = *ptr;
       }
    //Network group (NET No)
    if ( zb_cfg.group != config.net_group ) {
        change = true;
        zb_cfg.group = config.net_group;
       }
    //ключ шифрования
    if ( memcmp( zb_cfg.key, config.net_key, sizeof( zb_cfg.key ) ) != 0 ) {
        change = true;
        memcpy( zb_cfg.key, config.net_key, sizeof( zb_cfg.key ) );
       }
    if ( change == false ) {
        UartSendStr( "ZB: parameters match.\r\n" );
        return;
       }
    //запись параметров
    state = ZBControl( ZB_SAVE_CONFIG );
    sprintf( str, "ZB: save config ... %s\r\n", ZBErrDesc( state ) );
    UartSendStr( str );
    state = ZBControl( ZB_READ_CONFIG );
    sprintf( str, "ZB: read config ... %s\r\n", ZBErrDesc( state ) );
    UartSendStr( str );
    //вывод конфигурации ZigBee модуля
    ZBConfig();
 }

//*************************************************************************************************
// Обнуление приемного буфера
//*************************************************************************************************
static void ClearRecv( void ) {

    recv_ind = 0;
    memset( (uint8_t *)recv_buff, 0x00, sizeof( recv_buff ) );
 }

//*************************************************************************************************
// Возвращает статус ZigBee модуля для запрашиваемого типа состояния
//-------------------------------------------------------------------------------------------------
// ZBDevStat type  - тип состояния "запущен/в сети"
// return = SUCCESS - успешно
//          ERROR   - ошибка
//*************************************************************************************************
static ErrorStatus DevStatus( ZBDevState type ) {

    if ( type == ZB_STATUS_RUN )
        return (ErrorStatus)HAL_GPIO_ReadPin( ZB_RUN_GPIO_Port, ZB_RUN_Pin );
    if ( type == ZB_STATUS_NET )
        return (ErrorStatus)HAL_GPIO_ReadPin( ZB_NET_GPIO_Port, ZB_NET_Pin );
    return ERROR;
 }

//*************************************************************************************************
// Вывод значений параметров модуля ZigBee
//*************************************************************************************************
void ZBConfig( void ) {

    uint8_t ind;
    char *ptr, str[80];

    sprintf( str, "Device type ........................... %s\r\n", DevType( zb_cfg.dev_type ) );
    UartSendStr( str );
    sprintf( str, "Network state ......................... %s\r\n", NwkState( zb_cfg.nwk_state ) );
    UartSendStr( str );
    sprintf( str, "Network PAN_ID ........................ 0x%02X%02X\r\n", zb_cfg.pan_id[0], zb_cfg.pan_id[1] );
    UartSendStr( str );
    ptr = str;
    ptr += sprintf( ptr, "Network key ........................... " );
    for ( ind = 0; ind < sizeof( zb_cfg.key ); ind++ )
        ptr += sprintf( ptr, "%02X", zb_cfg.key[ind] );
    ptr += sprintf( ptr, "\r\n" );
    UartSendStr( str );
    sprintf( str, "Network short address ................. 0x%02X%02X\r\n", zb_cfg.short_addr[0], zb_cfg.short_addr[1] );
    UartSendStr( str );
    ptr = str;
    ptr += sprintf( ptr, "MAC address ........................... " );
    for ( ind = 0; ind < sizeof( zb_cfg.mac_addr ); ind++ )
        ptr += sprintf( ptr, "%02X ", zb_cfg.mac_addr[ind] );
    ptr += sprintf( ptr, "\r\n" );
    UartSendStr( str );
    sprintf( str, "Network short address of father node .. 0x%02X%02X\r\n", zb_cfg.coor_short_addr[0], zb_cfg.coor_short_addr[1] );
    UartSendStr( str );
    ptr = str;
    ptr += sprintf( ptr, "MAC address of father node ............ " );
    for ( ind = 0; ind < sizeof( zb_cfg.coor_mac_addr ); ind++ )
        ptr += sprintf( ptr, "%02X ", zb_cfg.coor_mac_addr[ind] );
    ptr += sprintf( ptr, "\r\n" );
    UartSendStr( str );
    sprintf( str, "Network group number .................. %u\r\n", zb_cfg.group );
    UartSendStr( str );
    sprintf( str, "Communication channel ................. %u\r\n", zb_cfg.chanel );
    UartSendStr( str );
    sprintf( str, "TX power .............................. %s dbm\r\n", TxPower( zb_cfg.txpower ) );
    UartSendStr( str );
    sprintf( str, "Sleep state ........................... %u\r\n", zb_cfg.sleep_time );
    UartSendStr( str );
 }

//*************************************************************************************************
// Вывод отладочной информации при передаче данных в модуль ZigBee
//-------------------------------------------------------------------------------------------------
// uint8_t *data    - указатель на передаваемые данные пакета
// uint8_t len_send - размер передаваемых данных
//*************************************************************************************************
#if defined( DEBUG_ZIGBEE ) && defined( DEBUG_TARGET )
static void SendDebug( char type, uint8_t *data, uint8_t len ) {

    uint8_t i;
    char *ptr;

    ptr = str2;
    if ( type == 'R' )
        ptr += sprintf( ptr, "RECV: " );
    else ptr += sprintf( ptr, "SEND: " );
    for ( i = 0; i < len; i++ )
        ptr += sprintf( ptr, "%02X ", *data++ );
    ptr += sprintf( ptr, "\r\n" );
    UartSendStr( str2 );
 }
#endif

//*************************************************************************************************
// Возвращает расшифровку и значения счетчиков ошибок
//-------------------------------------------------------------------------------------------------
// ZBErrorState err_ind - индекс счетчика ошибок, для err_ind = ZB_ERROR_OK 
//                        возвращается общее кол-во принятых пакетов
// char *str            - указатель для размещения результата
// return               - значение счетчика ошибок
//*************************************************************************************************
char *ZBErrCntDesc( ZBErrorState err_ind, char *str ) {

    char *ptr, *prev;
    
    if ( err_ind >= SIZE_ARRAY( error_cnt ) )
        return NULL;
    ptr = str;
    if ( err_ind == ZB_ERROR_OK ) {
        ptr += sprintf( ptr, "Total packages recv" );
        ptr += AddDot( str, 45, 0 );
        ptr += sprintf( ptr, "%6u\r\n", recv_cnt );
        prev = ptr;
        ptr += sprintf( ptr, "Total packages send" );
        ptr += AddDot( str, 45, (uint8_t)( prev - str ) );
        ptr += sprintf( ptr, "%6u", send_cnt );
        return str;
       }
    ptr += sprintf( ptr, "%s", ZBErrDesc( err_ind ) );
    //дополним расшифровку ошибки справа знаком "." до 45 символов
    ptr += AddDot( str, 45, 0 );
    ptr += sprintf( ptr, "%6u ", error_cnt[err_ind] );
    return str;
 }

//*************************************************************************************************
// Возвращает указатель на строку расшифровки результата выполнения запроса по протоколу 
//-------------------------------------------------------------------------------------------------
// ModBusError err_ind - код ошибки обработки принятого ответа от уст-ва
// return              - расшифровка кода ошибки
//*************************************************************************************************
char *ZBErrDesc( ZBErrorState err_ind ) {

    if ( err_ind < SIZE_ARRAY( error_descr ) )
        return error_descr[err_ind];
    return NULL;
 }

//*************************************************************************************************
// Возвращает указатель на строку с расшифровкой значения параметра: тип параметра
//-------------------------------------------------------------------------------------------------
// ZBDevType dev - идентификатор сообщения
// return = NULL - идентификтор не определен, иначе указатель на строку
//*************************************************************************************************
static char *DevType( ZBDevType dev ) {

    if ( dev < SIZE_ARRAY( dev_type ) )
        return dev_type[dev];
    else return NULL;
 }
 
//*************************************************************************************************
// Возвращает указатель на строку с расшифровкой значения параметра: статус сети
//-------------------------------------------------------------------------------------------------
// ZBDevState state - идентификатор сообщения
// return = NULL    - идентификтор не определен, иначе указатель на строку
//*************************************************************************************************
static char *NwkState( ZBNetState state ) {

    if ( state < SIZE_ARRAY( nwk_state ) )
        return nwk_state[state];
    else return NULL;
 }
 
//*************************************************************************************************
// Возвращает указатель на строку с расшифровкой значения стандартного ответа ZigBee модуля
//-------------------------------------------------------------------------------------------------
// ZBAnswer id_answ - идентификатор сообщения
// return = NULL    - идентификтор не определен, иначе указатель на строку
//*************************************************************************************************
#if defined( DEBUG_ZIGBEE ) && defined( DEBUG_TARGET )
static char *AnswDesc( ZBAnswer id_answ ) {

    if ( id_answ < SIZE_ARRAY( desc_answer ) )
        return desc_answer[id_answ];
    else return NULL;
 }
#endif
 
//*************************************************************************************************
// Возвращает указатель на строку с расшифровкой значения параметра: значение мощности
//-------------------------------------------------------------------------------------------------
// uint8_t id_mess - идентификатор сообщения
// return = NULL   - идентификтор не определен, иначе указатель на строку
//*************************************************************************************************
static char *TxPower( ZBTxPower id_pwr ) {

    if ( id_pwr < SIZE_ARRAY( txpower ) )
        return txpower[id_pwr];
    else return NULL;
 }
 
//*************************************************************************************************
// Обнуляет счетчики ошибок
//*************************************************************************************************
static void ErrorClr( void ) {

    recv_cnt = send_cnt = 0;
    memset( (uint8_t *)&error_cnt, 0x00, sizeof( error_cnt ) );
 }

//*************************************************************************************************
// Инкремент счетчиков ошибок
//-------------------------------------------------------------------------------------------------
// ZBErrorState err_ind - код ошибки
//*************************************************************************************************
void ZBIncError( ZBErrorState err_ind ) {

    if ( err_ind < SIZE_ARRAY( error_cnt ) )
        error_cnt[err_ind]++;
 }

//*************************************************************************************************
// Возвращает значение счетчика ошибок
//-------------------------------------------------------------------------------------------------
// ZBErrorState err_ind - индекс счетчика ошибок, для err_ind = ZB_ERROR_OK возвращается 
//                        кол-во счетчиков
// return               - значение счетчика ошибок
//*************************************************************************************************
uint32_t ZBErrCnt( ZBErrorState err_ind ) {

    if ( err_ind == ZB_ERROR_OK )
        return SIZE_ARRAY( error_cnt );
    if ( err_ind < SIZE_ARRAY( error_cnt ) )
        return error_cnt[err_ind];
    return 0;
 }
