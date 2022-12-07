
#ifndef __CAN_H
#define __CAN_H

#include <stdint.h>
#include <stdbool.h>

#include "main.h"

//Индексы кодов ошибок
typedef enum {
    CAN_ERR_OK,                             //0x00000000 OK
    CAN_ERR_WARNING,                        //0x00000001 Protocol Error Warning
    CAN_ERR_PASSIVE,                        //0x00000002 Error Passive
    CAN_ERR_BUSS_OFF,                       //0x00000004 Bus-off error
    CAN_ERR_STUFF,                          //0x00000008 Stuff error
    CAN_ERR_FORM,                           //0x00000010 Form error
    CAN_ERR_ACK,                            //0x00000020 Acknowledgment error
    CAN_ERR_BIT_RECV,                       //0x00000040 Bit recessive error
    CAN_ERR_BIT_DOM,                        //0x00000080 Bit dominant error
    CAN_ERR_CRC,                            //0x00000100 CRC error
    CAN_ERR_RX_FIFO0,                       //0x00000200 Rx FIFO0 overrun error
    CAN_ERR_RX_FIFO1,                       //0x00000400 Rx FIFO1 overrun error
    CAN_ERR_TX_MAIL0_LOST,                  //0x00000800 TxMailbox 0 transmit failure due to arbitration lost
    CAN_ERR_TX_MAIL0_ERROR,                 //0x00001000 TxMailbox 0 transmit failure due to transmit error
    CAN_ERR_TX_MAIL1_LOST,                  //0x00002000 TxMailbox 1 transmit failure due to arbitration lost
    CAN_ERR_TX_MAIL1_ERROR,                 //0x00004000 TxMailbox 1 transmit failure due to transmit error
    CAN_ERR_TX_MAIL2_LOST,                  //0x00008000 TxMailbox 2 transmit failure due to arbitration lost
    CAN_ERR_TX_MAIL2_ERROR,                 //0x00010000 TxMailbox 2 transmit failure due to transmit error
    CAN_ERR_TIMEOUT,                        //0x00020000 Timeout error
    CAN_ERR_NOT_INIT,                       //0x00040000 Peripheral not initialized
    CAN_ERR_NOT_READY,                      //0x00080000 Peripheral not ready
    CAN_ERR_NOT_STARTED,                    //0x00100000 Peripheral not started
    CAN_ERR_PARAM,                          //0x00200000 Parameter error
 } CANError;                                

//Коды параметров конфигурации CAN шины
typedef enum {
    CAN_PARAM_SPEED_ID,                     //код скорости обмена см.CANSpeed
    CAN_PARAM_SPEED,                        //скорость обмена
    CAN_PARAM_PRESCALER,                    //пред-делитель
    CAN_PARAM_TIMESEG1,                     //длительность сегмента #1 (Prop_Seg + Phase_Seg1)
    CAN_PARAM_TIMESEG2                      //длительность сегмента #2
} CANParam;

//Коды скорости обмена по CAN шине 
typedef enum {
    CAN_SPEED_10,                           //скорость обмена 10 kbit/sec
    CAN_SPEED_20,                           //скорость обмена 20 kbit/sec
    CAN_SPEED_50,                           //скорость обмена 50 kbit/sec
    CAN_SPEED_125,                          //скорость обмена 125 kbit/sec
    CAN_SPEED_250,                          //скорость обмена 250 kbit/sec
    CAN_SPEED_500                           //скорость обмена 500 kbit/sec
} CANSpeed;

//Коды разрядности ID для CAN шины
typedef enum {
    CAN_ADDRESS_11_BIT,                     //тип адресации CAN шины 11 бит
    CAN_ADDRESS_29_BIT                      //тип адресации CAN шины 29 бит
} CANAddress;

//Коды команд в составе адреса уст-ва
typedef enum {
    CAN_COMMAND_CTRL,                       //команда управления электроприводами см. CtrlCommand
    CAN_COMMAND_DATETIME,                   //установка значений дата/время
    CAN_COMMAND_LOG                         //запрос интервальных данных
} CANCommand;

//Коды команд для CAN шины: управления электроприводами 
typedef enum {
    CAN_CLOSE_ALL,                          //закрыть все
    CAN_OPEN_ALL,                           //открыть все
    CAN_COLD_OPEN,                          //открыть холодную воду
    CAN_COLD_CLOSE,                         //закрыть холодную воду
    CAN_HOT_OPEN,                           //открыть горячую воду
    CAN_HOT_CLOSE                           //закрыть горячую воду
} CtrlCommand;

#pragma pack( push, 1 )

//Структура данных для передачи/приема по CAN шине
typedef struct {
    uint32_t msg_id;
    uint8_t  rtr;
    uint8_t  data_len;
    uint8_t  data[8];
} CAN_DATA;

#pragma pack( pop )

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void CanInit( void );
uint32_t CanGetParam( CANSpeed speed_id, CANParam id_param );
ErrorStatus CheckCanSpeed( uint32_t baud, CANSpeed *speed );
char *CANErrDesc( CANError err_ind );
uint32_t CANErrCnt( CANError err_ind );
char *CANErrCntDesc( CANError err_ind, char *str );

#endif
