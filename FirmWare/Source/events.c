
//*************************************************************************************************
//
// Callback функции обработки прерываний
// 
//*************************************************************************************************

#include <string.h>

#include "cmsis_os2.h"

#include "main.h"
#include "uart.h"
#include "events.h"
#include "xtime.h"
#include "zigbee.h"
#include "rs485.h"

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern IWDG_HandleTypeDef hiwdg;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern UART_HandleTypeDef huart1, huart2, huart3;

extern osEventFlagsId_t valve_event, water_event;

//*************************************************************************************************
// CallBack функция, обработка внешних прерываний EXTI
//*************************************************************************************************
void HAL_GPIO_EXTI_Callback( uint16_t pin ) {

    uint32_t event = 0; 
    
    //прерывания от датчиков утечки
    if ( pin == LEAK1_Pin || pin == LEAK2_Pin ) {
        osEventFlagsSet( valve_event, EVN_VALVE_COLD_CLS | EVN_VALVE_HOT_CLS );
        if ( pin == LEAK1_Pin )
            event |= EVN_WTR_LEAK1;
        if ( pin == LEAK2_Pin )
            event |= EVN_WTR_LEAK2;
        osEventFlagsSet( water_event, event );
       }
    //прерывания от счетчиков расхода воды
    if ( pin == COUNT_COLD_Pin ) 
        osEventFlagsSet( water_event, EVN_WTR_CNT_COLD );
    if ( pin == COUNT_HOT_Pin ) 
        osEventFlagsSet( water_event, EVN_WTR_CNT_HOT );
    if ( pin == COUNT_FILTER_Pin ) 
        osEventFlagsSet( water_event, EVN_WTR_CNT_FILTER );
 }

//*************************************************************************************************
// CallBack функция, вызывается при приеме байта по UART
//*************************************************************************************************
void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart ) {

    if ( huart == &huart1 )
        UartRecvComplt();
    if ( huart == &huart2 )
        ZBRecvComplt();
    if ( huart == &huart3 )
        RS485RecvComplt();
}

//*************************************************************************************************
// CallBack функция, вызывается при завершении передачи из UART
//*************************************************************************************************
void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart ) {

    if ( huart == &huart1 )
        UartSendComplt();
    if ( huart == &huart2 )
        ZBSendComplt();
    if ( huart == &huart3 )
        RS485SendComplt();
 }

//*************************************************************************************************
// CallBack функция, вызывается при завершении преобразования в АЦП по обоим каналам
//*************************************************************************************************
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

    if( hadc->Instance == ADC1 ) {
        HAL_ADC_Stop_DMA( &hadc1 );
        osEventFlagsSet( water_event, EVN_WTR_PRESSURE_END );
       }
 }

//*************************************************************************************************
// CallBack функция, секундное прерывание от RTC
//*************************************************************************************************
void HAL_RTCEx_RTCEventCallback( RTC_HandleTypeDef *hrtc ) {

    DATE_TIME date_time;
    
    GetTimeDate( &date_time );
    //отправка события "запись в журнал суточных данных"
    osEventFlagsSet( water_event, EVN_WTR_LOG );
    //отправка состояние контроллера координатору сети каждую минуту
    if ( !date_time.sec /*&& date_time.min % 2 == 0*/ )
        osEventFlagsSet( zb_ctrl, EVN_ZC_IM_HERE );
 }

//*************************************************************************************************
// CallBack функция, простой планировщика
//*************************************************************************************************
void vApplicationIdleHook( void ) {

    #ifndef DEBUG_TARGET    
    HAL_IWDG_Refresh( &hiwdg );
    #endif
 }
