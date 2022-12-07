
//*************************************************************************************************
//
// Управление обменом данными по CAN шине
// 
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "main.h"
#include "can.h"
#include "data.h"
#include "valve.h"
#include "uart.h"
#include "sort.h"
#include "events.h"
#include "message.h"
#include "parse.h"
#include "xtime.h"
#include "config.h"

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern CONFIG config;
extern CAN_HandleTypeDef hcan;
extern osEventFlagsId_t valve_event;

//*************************************************************************************************
// Локальные константы                        
//*************************************************************************************************
#define CAN_MASK_SUB_ID         0x03        //маска для выделения ID команды
                                            //инверсией CAN_MASK_SUB_ID получаем маску для 
                                            //фильтрации ID: принимаемые ID xxxx00, xxxx01, xxxx02, xxxx03 
                                            //"0" в маске означает то, что в полученном идентификаторе 
                                            //может быть любое значение, "1" - точное соответствие фильтру

//ID сообщений ответов по CAN шине
#define CAN_ANS_DATETIME        1           //Дата/время
#define CAN_ANS_COLD            2           //Показания счетчика и давления холодной воды, 
                                            //состоянии электропривода, датчиков утечки
#define CAN_ANS_HOT             3           //Показания счетчика и давления горячей воды, 
                                            //состоянии электропривода, датчиков утечки
#define CAN_ANS_FILTER          4           //Показания счетчика фильтра питьевой воды и 
                                            //давления холодной воды (датчиков утечки)

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osMessageQueueId_t recv_can = NULL, send_can = NULL;

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static char str[80];
static osSemaphoreId_t sem_wait;

//Значения параметров Prescaler, TimeSeg1, TimeSeg2 в зависимости от скорости CAN интерфейса
//PCLK1 (APB1) = 32 MHz, Sample-Point at: 87.5%
static uint32_t can_cfg[][5] = {
    //Speed_ID,         Speed,  Prescaler,  TimeSeg1,       TimeSeg2
    { CAN_SPEED_10,     10,     200,        CAN_BS1_13TQ,   CAN_BS2_2TQ },      //10 kbit/s
    { CAN_SPEED_20,     20,     100,        CAN_BS1_13TQ,   CAN_BS2_2TQ },      //20 kbit/s
    { CAN_SPEED_50,     50,     40,         CAN_BS1_13TQ,   CAN_BS2_2TQ },      //50 kbit/s
    { CAN_SPEED_125,    125,    16,         CAN_BS1_13TQ,   CAN_BS2_2TQ },      //125 kbit/s
    { CAN_SPEED_250,    250,    8,          CAN_BS1_13TQ,   CAN_BS2_2TQ },      //250 kbit/s
    { CAN_SPEED_500,    500,    4,          CAN_BS1_13TQ,   CAN_BS2_2TQ }       //500 kbit/s
 };

//Расшифровка кодов ошибок CAN шины
static char * const error_descr[] = {
    "OK",                                                   //0x00000000
    "Protocol Error Warning",                               //0x00000001
    "Error Passive",                                        //0x00000002
    "Bus-off error",                                        //0x00000004
    "Stuff error",                                          //0x00000008
    "Form error",                                           //0x00000010
    "Acknowledgment error",                                 //0x00000020
    "Bit recessive error",                                  //0x00000040
    "Bit dominant error",                                   //0x00000080
    "CRC error",                                            //0x00000100
    "Rx FIFO0 overrun error",                               //0x00000200
    "Rx FIFO1 overrun error",                               //0x00000400
    "TxMailbox 0 failure due to arbitration lost",          //0x00000800
    "TxMailbox 0 failure due to transmit error",            //0x00001000
    "TxMailbox 1 failure due to arbitration lost",          //0x00002000
    "TxMailbox 1 failure due to transmit error",            //0x00004000
    "TxMailbox 2 failure due to arbitration lost",          //0x00008000
    "TxMailbox 2 failure due to transmit error",            //0x00010000
    "Timeout error",                                        //0x00020000
    "Peripheral not initialized",                           //0x00040000
    "Peripheral not ready",                                 //0x00080000
    "Peripheral not started",                               //0x00100000
    "Parameter error"                                       //0x00200000
 };

static WATER_LOG wtr_log;
static uint32_t recv_total, send_total;
static uint32_t error_cnt[SIZE_ARRAY( error_descr )]; //счетчики ошибок протокола

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void CANErrClr( void );
static void IncError( CANError err_ind );
static void CommandExec( CtrlCommand cmnd );
static void TaskCanRecv( void *argument );
static void TaskCanSend( void *argument );

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t task1_attr = {
    .name = "CanRecv", 
    .stack_size = 384,
    .priority = osPriorityNormal
 };

static const osThreadAttr_t task2_attr = {
    .name = "CanSend", 
    .stack_size = 384,
    .priority = osPriorityNormal
 };

static const osSemaphoreAttr_t sem_attr = { .name = "CanSemaph" };
static const osMessageQueueAttr_t recv_attr = { .name = "CanMsgRecv" };
static const osMessageQueueAttr_t send_attr = { .name = "CanMsgSend" };

//*************************************************************************************************
// Инициализация фильтров, очередей, задач управления обменом по CAN шине
//*************************************************************************************************
void CanInit( void ) {
    
    uint32_t mask;
    CAN_FilterTypeDef canFilterConfig;

    CANErrClr();
    //семафор блокировки
    sem_wait = osSemaphoreNew( 1, 0, &sem_attr );
    //создаем задачи
    osThreadNew( TaskCanRecv, NULL, &task1_attr );
    osThreadNew( TaskCanSend, NULL, &task2_attr );
    //создаем очереди передачи данных
    recv_can = osMessageQueueNew( 8, sizeof( CAN_DATA ), &recv_attr );
    send_can = osMessageQueueNew( 8, sizeof( CAN_DATA ), &send_attr );
    
    //параметры фильтрации для CAN: принимаем только фреймы (+RTR) адреса в config.can_id
    canFilterConfig.FilterBank = 0;                      //установить группу фильтров 0, диапазон от 0 до 13
    canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //установить ширину группы фильтров 0 бит 32
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  //установить группу фильтров 0 в режим маски
    mask = ~CAN_MASK_SUB_ID;
    if ( config.can_addr == CAN_ADDRESS_29_BIT ) {
        //ID = 29 бит
        canFilterConfig.FilterIdHigh = (uint16_t)( config.can_id >> 13 );
        canFilterConfig.FilterIdLow = (uint16_t)( config.can_id << 3 ) | CAN_ID_EXT; 
        canFilterConfig.FilterMaskIdHigh = (uint16_t)( mask >> 13 );
        canFilterConfig.FilterMaskIdLow = (uint16_t)( mask << 3 ) | CAN_ID_EXT;
       }
    else {
        //ID = 11 бит
        canFilterConfig.FilterIdHigh = (uint16_t)( config.can_id << 5 );
        canFilterConfig.FilterIdLow = 0x0000;
        canFilterConfig.FilterMaskIdHigh = (uint16_t)( mask << 5 );
        canFilterConfig.FilterMaskIdLow = 0x0000;
       }
    canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilterConfig.FilterActivation = ENABLE;
    //Установка фильтра
    HAL_CAN_ConfigFilter( &hcan, &canFilterConfig );
    HAL_CAN_Start( &hcan );
    HAL_CAN_ActivateNotification( &hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR );
 }

//*************************************************************************************************
// Задача обработки принятых CAN сообщений
//*************************************************************************************************
static void TaskCanRecv( void *argument ) {

    bool find = false;
    LOG_REQ *log_req;
    CANCommand can_cmnd;
    osStatus_t status;
    CAN_DATA can_data;
    uint8_t *ptr, len = 0;
    uint16_t addr, rec, cnt;

    for ( ;; ) {
        status = osMessageQueueGet( recv_can, &can_data, NULL, osWaitForever );
        if ( status == osOK ) {
            can_cmnd = (CANCommand)( can_data.msg_id & CAN_MASK_SUB_ID );
            //выполнение команды управления электроприводами
            if ( can_data.rtr == CAN_RTR_DATA && can_cmnd == CAN_COMMAND_CTRL )
                CommandExec( (CtrlCommand)can_data.data[0] );
            //выполнение команды: установка дата/время
            if ( can_data.rtr == CAN_RTR_DATA && can_cmnd == CAN_COMMAND_DATETIME )
                SetTimeDate( (DATE_TIME *)&can_data.data[0] );
            //запрос текущих данных по счетчикам
            if ( can_data.rtr == CAN_RTR_REMOTE && can_cmnd == CAN_COMMAND_CTRL ) {
                //запрос данных RTC
                ptr = GetDataCan2( DATA_DATE, &len );
                if ( ptr != NULL ) {
                    can_data.data_len = len;
                    can_data.msg_id = CAN_ANS_DATETIME;
                    memcpy( can_data.data, ptr, len );
                    //передача сообщения в очередь для обработки
                    osMessageQueuePut( send_can, &can_data, 0, 0 );
                   }
                //запрос данных по холодной воде
                ptr = GetDataCan2( DATA_COLD, &len );
                if ( ptr != NULL ) {
                    can_data.data_len = len;
                    can_data.msg_id = CAN_ANS_COLD;
                    memcpy( can_data.data, ptr, len );
                    //передача сообщения в очередь для обработки
                    osMessageQueuePut( send_can, &can_data, 0, 0 );
                   }
                //запрос данных по горячей воды
                ptr = GetDataCan2( DATA_HOT, &len );
                if ( ptr != NULL ) {
                    can_data.data_len = len;
                    can_data.msg_id = CAN_ANS_HOT;
                    memcpy( can_data.data, ptr, len );
                    //передача сообщения в очередь для обработки
                    osMessageQueuePut( send_can, &can_data, 0, 0 );
                   }
                //запрос данных по питьевой воде
                ptr = GetDataCan2( DATA_FILTER, &len );
                if ( ptr != NULL ) {
                    can_data.data_len = len;
                    can_data.msg_id = CAN_ANS_FILTER;
                    memcpy( can_data.data, ptr, len );
                    //передача сообщения в очередь для обработки
                    status = osMessageQueuePut( send_can, &can_data, 0, 0 );
                   }
               }
            //запрос интервальных показаний
            if ( can_data.rtr == CAN_RTR_DATA && can_cmnd == CAN_COMMAND_LOG ) {
                log_req = (LOG_REQ *)&can_data.data[0];
                cnt = MakeSort();
                if ( cnt ) {
                    for ( rec = 0; rec < cnt; rec++ ) {
                        //сортировка данных
                        addr = GetAddrSort( rec );
                        if ( !addr )
                            continue;
                        memset( (uint8_t *)&wtr_log, 0x00, sizeof( wtr_log ) );
                        if ( FramReadData( addr, (uint8_t *)&wtr_log, sizeof( wtr_log ) ) != FRAM_OK )
                            continue;
                        if ( wtr_log.type_event != EVENT_DATA )
                            continue; //только интервальные данные
                        if ( wtr_log.day == log_req->day && wtr_log.month == log_req->month && wtr_log.year == log_req->year &&
                             !wtr_log.hour && !wtr_log.min && !wtr_log.sec ) {
                             find = true;
                             break; //найдена дата
                            }
                       } //for end
                   } //end if ( cnt )
               } //end if ( can_data.rtr ... )
            if ( find == true ) {
                //подготовка данных по холодной воде
                ptr = GetDataLog( DATA_LOG_COLD, &wtr_log, &len );
                if ( ptr != NULL ) {
                    can_data.data_len = len;
                    can_data.msg_id = CAN_ANS_COLD;
                    memcpy( can_data.data, ptr, len );
                    //передача сообщения в очередь для обработки
                    osMessageQueuePut( send_can, &can_data, 0, 0 );
                   }
                //подготовка данных по горячей воды
                ptr = GetDataLog( DATA_LOG_HOT, &wtr_log, &len );
                if ( ptr != NULL ) {
                    can_data.data_len = len;
                    can_data.msg_id = CAN_ANS_HOT;
                    memcpy( can_data.data, ptr, len );
                    //передача сообщения в очередь для обработки
                    osMessageQueuePut( send_can, &can_data, 0, 0 );
                   }
                //подготовка данных по питьевой воде
                ptr = GetDataLog( DATA_LOG_FILTER, &wtr_log, &len );
                if ( ptr != NULL ) {
                    can_data.data_len = len;
                    can_data.msg_id = CAN_ANS_FILTER;
                    memcpy( can_data.data, ptr, len );
                    //передача сообщения в очередь для обработки
                    osMessageQueuePut( send_can, &can_data, 0, 0 );
                   }
               }
           }
      }
 }

//*************************************************************************************************
// Задача передачи сообщений по CAN шине
//*************************************************************************************************
static void TaskCanSend( void *argument ) {

    osStatus_t status;
    CAN_DATA can_data;
    uint32_t freelevel, mailBoxNum = 0;
    CAN_TxHeaderTypeDef msgHeader;

    for ( ;; ) {
        status = osMessageQueueGet( send_can, &can_data, NULL, osWaitForever );
        if ( status == osOK ) {
            freelevel = HAL_CAN_GetTxMailboxesFreeLevel( &hcan );
            if ( !freelevel )
                osSemaphoreAcquire( sem_wait, osWaitForever ); //все почтовые ящики заняты, установим семафор ожидания
            //готовим данные для отправки по CAN шине
            if ( config.can_addr == CAN_ADDRESS_29_BIT ) {
                //расширенный адрес, 29 бит
                msgHeader.IDE = CAN_ID_EXT;
                msgHeader.ExtId = config.can_id | can_data.msg_id;
               }
            else {
                //стандартный адрес, 11 бит
                msgHeader.IDE = CAN_ID_STD;
                msgHeader.StdId = config.can_id | can_data.msg_id;
               }
            msgHeader.DLC = can_data.data_len; //размер блока данных
            msgHeader.TransmitGlobalTime = DISABLE;
            msgHeader.RTR = CAN_RTR_DATA;      //фрейм данных
            //передача данных
            HAL_CAN_AddTxMessage( &hcan, &msgHeader, can_data.data, &mailBoxNum );
            //ждем завершения передачи
            osSemaphoreAcquire( sem_wait, osWaitForever );
           }
       }
 }

//*************************************************************************************************
// Обработка прерывания - исходящий почтовый ящик свободен
//*************************************************************************************************
void HAL_CAN_TxMailbox0CompleteCallback( CAN_HandleTypeDef *hcan ) {

    send_total++;
    osSemaphoreRelease( sem_wait );
 }

//*************************************************************************************************
// Callback - данные приняты по CAN шине
//*************************************************************************************************
void HAL_CAN_RxFifo0MsgPendingCallback( CAN_HandleTypeDef *hcan ) {

    CAN_DATA can_data;
    uint8_t msg_data[8];
    CAN_RxHeaderTypeDef msgHeader;

    //проверим очередь сообщений FIFO0, прочитаем сообщение
    if ( HAL_CAN_GetRxMessage( hcan, CAN_RX_FIFO0, &msgHeader, msg_data ) == HAL_OK ) {
        //приняты данные
        recv_total++;
        if ( msgHeader.IDE == CAN_ID_EXT ) //тип адреса
            can_data.msg_id = msgHeader.ExtId;
        else can_data.msg_id = msgHeader.StdId;
        can_data.rtr = msgHeader.RTR;
        can_data.data_len = msgHeader.DLC;
        //копируем принятый пакет данных в структуру CAN_DATA 
        memcpy( can_data.data, msg_data, sizeof( msg_data ) );
        //передача сообщения в очередь для обработки
        osMessageQueuePut( recv_can, &can_data, 0, 0 );
       }
 }

//*************************************************************************************************
// Расшифровка ошибок CAN шины
//*************************************************************************************************
void HAL_CAN_ErrorCallback( CAN_HandleTypeDef *hcan ) {

    uint8_t ind_err;
    uint32_t mask = 0x00000001;

    if ( !hcan->ErrorCode )
        return;
    sprintf( str, "\r\nCAN error code: 0x%08X\r\n", (unsigned int)hcan->ErrorCode );
    UartSendStr( str );
    for ( ind_err = 1; ind_err < sizeof( error_cnt ); ind_err++, mask <<= 1 ) {
        if ( hcan->ErrorCode & mask )
            IncError( (CANError)ind_err );
       }
    HAL_CAN_ResetError( hcan );
    //снимаем семафор блокировки
    osSemaphoreRelease( sem_wait );
}

//*************************************************************************************************
// Возвращает значения параметров Prescaler, TimeSeg1, TimeSeg2 для указанной скорости CAN интерфейса
//-------------------------------------------------------------------------------------------------
// CANSpeed speed_id - идентификатор значения скорости
// CANParam id_param - ID параметра по которому необходимо получить значение
// return            - значение параметра для MX_CAN_Init()
//*************************************************************************************************
uint32_t CanGetParam( CANSpeed speed_id, CANParam id_param ) {

    uint8_t idx;

    for ( idx = 0; idx < SIZE_ARRAY( can_cfg ); idx++ ) {
        if ( can_cfg[idx][CAN_PARAM_SPEED_ID] == speed_id )
            return can_cfg[idx][id_param];
       }
    //по умолчанию скорость 125 kbit/s
    return can_cfg[3][id_param];
}

//*************************************************************************************************
// Проверка значения baud на допустимое значение скорости обмена.
// При успешной проверке в speed возвращается ID скорости обмена
//-------------------------------------------------------------------------------------------------
// uint32_t baud    - скорость обмена
// CANSpeed *speed  - ID скорости обмена
// return = ERROR   - не допустимое значение скорости обмена
//        = SUCCESS - в speed указан ID значения скорости обмена
//*************************************************************************************************
ErrorStatus CheckCanSpeed( uint32_t baud, CANSpeed *speed ) {

    uint8_t idx;

    *speed = CAN_SPEED_125; //значение по умолчанию
    for ( idx = 0; idx < SIZE_ARRAY( can_cfg ); idx++ ) {
        if ( can_cfg[idx][CAN_PARAM_SPEED] == baud ) {
            *speed = (CANSpeed)can_cfg[idx][CAN_PARAM_SPEED_ID];
            return SUCCESS;
           }
       }
    return ERROR;
}

//*************************************************************************************************
// Выполнение команд, управления электроприводами, полученных по CAN шине
//-------------------------------------------------------------------------------------------------
// CtrlCommand cmnd - код команды
//*************************************************************************************************
static void CommandExec( CtrlCommand cmnd ) {

    if ( cmnd == CAN_CLOSE_ALL ) {
        osEventFlagsSet( valve_event, EVN_VALVE_COLD_CLS | EVN_VALVE_PREV_CHECK );
        osEventFlagsSet( valve_event, EVN_VALVE_HOT_CLS | EVN_VALVE_PREV_CHECK );
       }
    if ( cmnd == CAN_OPEN_ALL ) {
        osEventFlagsSet( valve_event, EVN_VALVE_COLD_OPN | EVN_VALVE_PREV_CHECK );
        osEventFlagsSet( valve_event, EVN_VALVE_HOT_OPN | EVN_VALVE_PREV_CHECK );
       }
    if ( cmnd == CAN_COLD_OPEN )
        osEventFlagsSet( valve_event, EVN_VALVE_COLD_OPN | EVN_VALVE_PREV_CHECK );
    if ( cmnd == CAN_COLD_CLOSE )
        osEventFlagsSet( valve_event, EVN_VALVE_COLD_CLS | EVN_VALVE_PREV_CHECK );
    if ( cmnd == CAN_HOT_OPEN )
        osEventFlagsSet( valve_event, EVN_VALVE_HOT_OPN | EVN_VALVE_PREV_CHECK );
    if ( cmnd == CAN_HOT_CLOSE )
        osEventFlagsSet( valve_event, EVN_VALVE_HOT_CLS | EVN_VALVE_PREV_CHECK );
 }

//*************************************************************************************************
// Возвращает указатель на строку расшифровки ошибки
//-------------------------------------------------------------------------------------------------
// CANError err_ind - код ошибки обработки принятого ответа от уст-ва
// return           - расшифровка кода ошибки
//*************************************************************************************************
char *CANErrDesc( CANError err_ind ) {

    if ( err_ind >= SIZE_ARRAY( error_descr ) )
        return NULL;
    return error_descr[err_ind];
 }

//*************************************************************************************************
// Обнуляет счетчики ошибок
//*************************************************************************************************
static void CANErrClr( void ) {

    send_total = recv_total = 0;
    memset( (uint8_t *)&error_cnt, 0x00, sizeof( error_cnt ) );
 }

//*************************************************************************************************
// Возвращает значение счетчика ошибок CAN
//-------------------------------------------------------------------------------------------------
// ModBusError err_ind - индекс счетчика ошибок, для error = CAN_ERR_OK возвращается кол-во счетчиков
// return              - значение счетчика ошибок
//*************************************************************************************************
uint32_t CANErrCnt( CANError err_ind ) {

    if ( err_ind == CAN_ERR_OK )
        return SIZE_ARRAY( error_cnt );
    if ( err_ind < SIZE_ARRAY( error_cnt ) )
        return error_cnt[err_ind];
    return 0;
 }

//*************************************************************************************************
// Инкремент счетчиков ошибок
//-------------------------------------------------------------------------------------------------
// CANError error - код ошибки по которому выполняется инкремент счетчика
//*************************************************************************************************
static void IncError( CANError err_ind ) {

    if ( err_ind < SIZE_ARRAY( error_cnt ) )
        error_cnt[err_ind]++;
 }

//*************************************************************************************************
// Возвращает расшифровку и значения счетчиков ошибок CAN
//-------------------------------------------------------------------------------------------------
// ModBusError err_ind - индекс счетчика ошибок, для error = CAN_ERR_OK 
//                       возвращается общее кол-во принятых фреймов
// char *str           - указатель для размещения результата
// return              - значение счетчика ошибок
//*************************************************************************************************
char *CANErrCntDesc( CANError err_ind, char *str ) {

    char *prev, *ptr;
    
    if ( err_ind >= SIZE_ARRAY( error_descr ) )
        return NULL;
    ptr = str;
    if ( err_ind == CAN_ERR_OK ) {
        ptr += sprintf( ptr, "Total packages recv" );
        ptr += AddDot( str, 45, 0 );
        ptr += sprintf( ptr, "%6u\r\n", recv_total );
        prev = ptr;
        ptr += sprintf( ptr, "Total packages send" );
        ptr += AddDot( str, 45, (uint8_t)( prev - str ) );
        ptr += sprintf( ptr, "%6u", send_total );
        return str;
       }
    ptr += sprintf( ptr, "%s", CANErrDesc( err_ind ) );
    //дополним расшифровку ошибки справа знаком "." до 45 символов
    ptr += AddDot( str, 45, 0 );
    ptr += sprintf( ptr, "%6u ", error_cnt[err_ind] );
    return str;
 }

