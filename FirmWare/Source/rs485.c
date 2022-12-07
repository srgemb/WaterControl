
//*************************************************************************************************
//
// Управление интерфейсом RS-485
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "main.h"
#include "rs485.h"
#include "uart.h"
#include "modbus.h"
#include "events.h"

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart3;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define BUFFER_SIZE             255                 //размер приемного и передающего буфера

#define RS485_MODE_SEND         GPIO_PIN_SET
#define RS485_MODE_RECV         GPIO_PIN_RESET

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
#ifdef MODBUS_DEBUG
char str[80];
#endif
static char recv;
static uint8_t recv_ind = 0; 
static uint8_t recv_buff[BUFFER_SIZE], send_buff[BUFFER_SIZE];

static osEventFlagsId_t modbus_event = NULL;

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t task_attr = {
    .name = "Modbus", 
    .stack_size = 512,
    .priority = osPriorityNormal
 };
 
//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void ClearRecv( void );
static void TaskModbus( void *pvParameters );

//*************************************************************************************************
// Инициализация протокола
//*************************************************************************************************
void Rs485Init( void ) {

    ModBusErrClr();
    modbus_event = osEventFlagsNew( NULL );
    //создаем задачу управления обменом по MODBUS
    osThreadNew( TaskModbus, NULL, &task_attr );
    //т.к. при выполнении HAL_TIM_Base_Start_IT() почти сразу формируется прерывание
    //вызов HAL_TIM_Base_Start_IT() выполняем только один раз, дальнейшее управление
    //TIMER2 выполняется через __HAL_TIM_ENABLE()/__HAL_TIM_DISABLE()
    HAL_TIM_Base_Start_IT( &htim2 );
 }

//*************************************************************************************************
// Задача управления обменом данными
//*************************************************************************************************
static void TaskModbus( void *pvParameters ) {

    int32_t event;
    uint8_t len_send;
    ModBusError error;
    MBUS_REQ request;
    
    //запускаем прием
    osEventFlagsSet( modbus_event, EVN_MODBUS_START );
    for ( ;; ) {
        event = osEventFlagsWait( modbus_event, EVN_MODBUS_MASK, osFlagsWaitAny, osWaitForever );
        //стартуем прием данных
        if ( event & EVN_MODBUS_START ) {
            ClearRecv();
            HAL_GPIO_WritePin( RS485_SEND_GPIO_Port, RS485_SEND_Pin, RS485_MODE_RECV );
            HAL_UART_Receive_IT( &huart3, (uint8_t *)&recv, sizeof( recv ) );
           }
        //обработка принятого фрейма
        if ( event & EVN_MODBUS_RECV ) {
            error = CheckRequest( recv_buff, recv_ind, &request );
            IncError( error );
            #ifdef MODBUS_DEBUG
            sprintf( str, "MODBUS: %s\r\n", ModBusErrDesc( error ) );
            UartSendStr( str );
            #endif
            if ( error == MBUS_REQST_NOT_FOR_DEV || error == MBUS_REQST_CRC || error == MBUS_ERROR_PARAM )
                ClearRecv(); //принятый фрейм не обрабатываем
            else {
                //подготовка ответа
                len_send = CreateFrame( &request, error );
                if ( len_send ) {
                    //переход в режим передачи
                    HAL_GPIO_WritePin( RS485_SEND_GPIO_Port, RS485_SEND_Pin, RS485_MODE_SEND );
                    HAL_UART_Transmit_IT( &huart3, (uint8_t *)&send_buff, len_send );
                   }
                ClearRecv();
               }
           }
      }
 }

//*************************************************************************************************
// Функция обратного вызова при приеме байта по UART3
//*************************************************************************************************
void RS485RecvComplt( void ) {

    //прием одного байта
    if ( recv_ind < sizeof( recv_buff ) )
        recv_buff[recv_ind++] = recv;
    else ClearRecv(); //переполнение буфера
    //продолжаем прием
    HAL_UART_Receive_IT( &huart3, (uint8_t *)&recv, sizeof( recv ) );
    //если таймер выключен - стартуем один раз
    //при приеме каждого следующего байта - только сброс счетчика
    if ( !( htim2.Instance->CR1 & TIM_CR1_CEN ) )
        __HAL_TIM_ENABLE( &htim2 );
    else __HAL_TIM_SetCounter( &htim2, 0 );
 }

//*************************************************************************************************
// Функция обратного вызова от таймера TIMER2 - пауза между пакетами
//*************************************************************************************************
void Rs485Callback( void ) {

    //выключаем таймер
    __HAL_TIM_DISABLE( &htim2 );
    //сообщим в задачу для дальнейшей обработки принятого фрейма
    if ( recv_ind )
        osEventFlagsSet( modbus_event, EVN_MODBUS_RECV );
 }

//*************************************************************************************************
// Функция обратного вызова при завершении передачи фрейма по UART3
//*************************************************************************************************
void RS485SendComplt( void ) {

    //переход в режим приема
    HAL_GPIO_WritePin( RS485_SEND_GPIO_Port, RS485_SEND_Pin, RS485_MODE_RECV );
 }

//*************************************************************************************************
// Обнуление приемного буфера
//*************************************************************************************************
static void ClearRecv( void ) {

    recv_ind = 0;
    memset( (uint8_t *)recv_buff, 0x00, sizeof( recv_buff ) );
 }

//*************************************************************************************************
// Обнуление передающего буфера
//*************************************************************************************************
void ClearSend( void ) {

    memset( (uint8_t *)send_buff, 0x00, sizeof( send_buff ) );
 }

//*************************************************************************************************
// Возвращает указатель на передающий буфер
//*************************************************************************************************
uint8_t *RS485SendBuff( void ) {

    return send_buff;
 }
