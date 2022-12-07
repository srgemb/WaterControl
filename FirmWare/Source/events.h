
#ifndef __EVENTS_H
#define __EVENTS_H

#include "cmsis_os2.h"

//*************************************************************************************************
// ID объектов RTOS
//*************************************************************************************************
extern osMessageQueueId_t recv_can, send_can;
extern osEventFlagsId_t led_event, valve_event, water_event, cmnd_event;
extern osEventFlagsId_t uart_event, fram_event, zb_flow, zb_ctrl;

//*************************************************************************************************
// Флаги событий при обмене данными по UART
//*************************************************************************************************
#define EVN_UART_RECV               0x00000001  //принята строка команды
#define EVN_UART_START              0x00000002  //запуск ожидания приема данных 
                                                //после завершения выполнения команды
#define EVN_UART_NEXT               0x00000004  //передача следующего блока 

#define EVN_UART_MASK               ( EVN_UART_RECV | EVN_UART_START | EVN_UART_NEXT )

//*************************************************************************************************
// Флаги событий выполнения команд
//*************************************************************************************************
#define EVN_CMND_EXEC               0x00000001  //выполнение команды
#define EVN_CMND_PROMPT             0x00000002  //вывод символа ">" в консоль

#define EVN_CMND_MASK               ( EVN_CMND_EXEC | EVN_CMND_PROMPT )

//*************************************************************************************************
// Флаги событий от датчиков утечки и счетчиков воды
//*************************************************************************************************
#define EVN_WTR_CNT_COLD            0x00000001  //счетчик холодной воды
#define EVN_WTR_CNT_HOT             0x00000002  //счетчик горячей воды
#define EVN_WTR_CNT_FILTER          0x00000004  //счетчик питьевой воды

#define EVN_WTR_LEAK1               0x00000010  //датчик утечки воды #1
#define EVN_WTR_LEAK2               0x00000020  //датчик утечки воды #2

#define EVN_WTR_LOG                 0x00000100  //сохранение суточных данных

#define EVN_WTR_PRESSURE_START      0x00000200  //запуск измерения давления воды
#define EVN_WTR_PRESSURE_END        0x00000400  //измерение давление воды завершено

#define EVN_WTR_VALUE               0x00001000  //вывод результата чтения данных из FRAM при включении

#define EVN_WTR_DATA                0x00002000  //передача текущих данных по расходу воды

#define EVN_WTR_MASK                ( EVN_WTR_CNT_COLD | EVN_WTR_CNT_HOT | EVN_WTR_CNT_FILTER | \
                                    EVN_WTR_LEAK1 | EVN_WTR_LEAK2 | EVN_WTR_LOG | EVN_WTR_VALUE |\
                                    EVN_WTR_PRESSURE_START | EVN_WTR_PRESSURE_END | EVN_WTR_DATA )

//*************************************************************************************************
// Флаги событий управления индикацией состояния электроприводов
//*************************************************************************************************
#define EVN_LED_COLD_OPN_OFF        0x00000001  //выключение светодиода "отрытие холодной воды"
#define EVN_LED_COLD_OPN_ON         0x00000002  //включение светодиода "отрытие холодной воды"
#define EVN_LED_COLD_OPN_BLK        0x00000004  //мигание светодиода "отрытие холодной воды"

#define EVN_LED_COLD_CLS_OFF        0x00000010  //выключение светодиода "закрытие холодной воды"
#define EVN_LED_COLD_CLS_ON         0x00000020  //включение светодиода "закрытие холодной воды"
#define EVN_LED_COLD_CLS_BLK        0x00000040  //мигание светодиода "закрытие холодной воды"

#define EVN_LED_HOT_OPN_OFF         0x00000100  //выключение светодиода "отрытие горячей воды"
#define EVN_LED_HOT_OPN_ON          0x00000200  //включение светодиода "отрытие горячей воды"
#define EVN_LED_HOT_OPN_BLK         0x00000400  //мигание светодиода "отрытие горячей воды"
                                                  
#define EVN_LED_HOT_CLS_OFF         0x00001000  //выключение светодиода "закрытие горячей воды"
#define EVN_LED_HOT_CLS_ON          0x00002000  //включение светодиода "закрытие горячей воды"
#define EVN_LED_HOT_CLS_BLK         0x00004000  //мигание светодиода "закрытие горячей воды"

#define EVN_LED_ZB_ACTIVE           0x00010000  //включение индикации "обмен пакетами по ZigBee"

#define EVN_LED_MASK                ( EVN_LED_HOT_OPN_OFF | EVN_LED_HOT_OPN_ON | EVN_LED_HOT_OPN_BLK | \
                                    EVN_LED_HOT_CLS_OFF | EVN_LED_HOT_CLS_ON | EVN_LED_HOT_CLS_BLK | \
                                    EVN_LED_COLD_OPN_OFF | EVN_LED_COLD_OPN_ON | EVN_LED_COLD_OPN_BLK | \
                                    EVN_LED_COLD_CLS_OFF | EVN_LED_COLD_CLS_ON  | EVN_LED_COLD_CLS_BLK | \
                                    EVN_LED_ZB_ACTIVE )

//*************************************************************************************************
// Флаги управления электроприводами холодной/горячей воды
//*************************************************************************************************
#define EVN_VALVE_COLD_STOP         0x00000001  //выключить электропривод холодной воды
#define EVN_VALVE_COLD_OPN          0x00000002  //открыть кран подачи холодной воды
#define EVN_VALVE_COLD_CLS          0x00000004  //закрыть кран подачи холодной воды

#define EVN_VALVE_HOT_STOP          0x00000010  //выключить электропривод горячей воды
#define EVN_VALVE_HOT_OPN           0x00000020  //открыть кран подачи горячей воды
#define EVN_VALVE_HOT_CLS           0x00000040  //закрыть кран подачи горячей воды

//*************************************************************************************************
// Флаги событий при управлении электроприводами
//*************************************************************************************************
#define EVN_VALVE_COLD_CHK1         0x00000100  //проверка наличия тока в цепи электропривода холодной воды
#define EVN_VALVE_COLD_CHK2         0x00000200  //проверка отсутствия тока в цепи электропривода холодной воды
#define EVN_VALVE_HOT_CHK1          0x00000400  //проверка наличия тока в цепи электропривода горячей воды
#define EVN_VALVE_HOT_CHK2          0x00000800  //проверка отсутствия тока в цепи электропривода горячей воды

#define EVN_VALVE_COLD_PWR          0x00001000  //выключения тока в цепи электропривода холодной воды
#define EVN_VALVE_COLD_OVR          0x00002000  //превышение тока в цепи электропривода холодной воды
#define EVN_VALVE_HOT_PWR           0x00004000  //выключения тока в цепи электропривода горячей воды
#define EVN_VALVE_HOT_OVR           0x00008000  //превышение тока в цепи электропривода горячей воды

#define EVN_VALVE_COLD_KEY          0x00010000  //кнопка управления электроприводом холодной воды
#define EVN_VALVE_HOT_KEY           0x00020000  //кнопка управления электроприводом горячей воды

#define EVN_VALVE_PREV_CHECK        0x00080000  //предварительный контроль состояния электропривода 

#define EVN_VALVE_MASK              ( EVN_VALVE_HOT_STOP | EVN_VALVE_COLD_STOP | \
                                    EVN_VALVE_HOT_OPN | EVN_VALVE_HOT_CLS | \
                                    EVN_VALVE_COLD_OPN | EVN_VALVE_COLD_CLS | \
                                    EVN_VALVE_HOT_CHK1 | EVN_VALVE_HOT_CHK2 | \
                                    EVN_VALVE_COLD_CHK1 | EVN_VALVE_COLD_CHK2 | \
                                    EVN_VALVE_COLD_PWR | EVN_VALVE_COLD_OVR | \
                                    EVN_VALVE_HOT_PWR | EVN_VALVE_HOT_OVR | \
                                    EVN_VALVE_COLD_KEY | EVN_VALVE_HOT_KEY | EVN_VALVE_PREV_CHECK )

//*************************************************************************************************
// Флаги событий управления обменом с модулем ZigBee
//*************************************************************************************************
#define EVN_ZC_CONFIG_CHECK         0x00000001  //проверка настроек модуля ZigBee
#define EVN_ZC_NET_LOST             0x00000002  //сеть потеряна
#define EVN_ZC_NET_RESTORE          0x00000004  //сеть восстановлена

#define EVN_ZC_SEND_STATE           0x00000010  //передача состояния контроллера
#define EVN_ZC_SEND_DATA            0x00000020  //передача теущих данных расхода воды
#define EVN_ZC_SEND_WLOG            0x00000040  //передача журнальных данных
#define EVN_ZC_SEND_VALVE           0x00000080  //передача состояние электроприводов
#define EVN_ZC_SEND_LEAKS           0x00000100  //передача состояние датчиков утечки

#define EVN_ZC_SYNC_DTIME           0x00000200  //синхронизация даты/времени
#define EVN_ZC_IM_HERE              0x00000400  //отправка состояние контроллера координатору

#define EVN_ZC_MASK                 ( EVN_ZC_CONFIG_CHECK | EVN_ZC_NET_LOST | EVN_ZC_NET_RESTORE | \
                                    EVN_ZC_SEND_VALVE | EVN_ZC_SEND_STATE | EVN_ZC_SEND_DATA | \
                                    EVN_ZC_SEND_WLOG | EVN_ZC_SYNC_DTIME | EVN_ZC_SEND_LEAKS | EVN_ZC_IM_HERE )

#define EVN_ZB_RECV_CHECK           0x00000001  //прием пакета завершен
#define EVN_ZB_RECV_TIMEOUT         0x00000002  //вышло время ожидания ответа
#define EVN_ZB_SEND_COMPLT          0x00000004  //завершение передачи из UART2

#define EVN_ZB_MASK                 ( EVN_ZB_RECV_CHECK | EVN_ZB_RECV_TIMEOUT | EVN_ZB_SEND_COMPLT )

//*************************************************************************************************
// Cобытия обрабатываемые в задачах "Modbus"
//*************************************************************************************************
#define EVN_MODBUS_START            0x00001000  //запуск приема
#define EVN_MODBUS_RECV             0x00002000  //принят фрейм данных

#define EVN_MODBUS_MASK             ( EVN_MODBUS_START | EVN_MODBUS_RECV )

#endif
