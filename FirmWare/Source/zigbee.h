
#ifndef __ZIGBEE_H
#define __ZIGBEE_H

#include <stdbool.h>
#include <stdio.h>

#include "water.h"
#include "valve.h"

//#define DEBUG_ZIGBEE                        //вывод принятых/отправленных пакетов в HEX формате

#define MAX_NETWORK_PANID           0xFFFE  //максимальный номер сети
#define MAX_NETWORK_ADDR            0xFFF8  //максимальный адрес уст-ва в сети
#define MAX_NETWORK_GROUP           99      //максимальный номер группы
#define MAX_DEVICE_NUMB             0xFFFF  //максимальный номер уст-ва в сети

//Зарезервированные адреса для трансляции
#define BROADCAST_ALL_DEV           0xFFFE  //передача для всех уст-в в сети
#define BROADCAST_IDLE_DEV          0xFFFD  //передача для всех простаивающих уст-в в сети (кроме режиме "сон")
#define BROADCAST_CO_ROUTER_DEV     0xFFFC  //передача для всех координаторов и роутеров 

//Команды управления ZigBee модулем
typedef enum {
    ZB_NO_COMMAND,
    ZB_DEV_RESET,                           //аппаратный перезапуск ZigBee 
    ZB_READ_CONFIG,                         //чтение конфигурации
    ZB_SAVE_CONFIG,                         //запись конфигурации
    ZB_DEV_INIT,                            //программный перезапуск ZigBee 
    ZB_DEV_FACTORY,                         //установка заводских настроек
    ZB_NET_RESTART,                         //переподключение к сети
    ZB_SEND_DATA                            //передача данных терминал -> координатор
 } ZBCmnd;

//Результат выполнения команд управления модулем ZigBee 
typedef enum {
    ZB_ERROR_OK,                            //ошибок нет
    ZB_ERROR_EXEC,                          //ошибка выполненение команды
    ZB_ERROR_SEND,                          //ошибка передачи данных
    ZB_ERROR_RUN,                           //модуль не включен
    ZB_ERROR_NETWORK,                       //нет сети
    ZB_ERROR_CMD,                           //команда не определена (нет списке доступных команд)
    ZB_ERROR_TIMEOUT,                       //вышло время ожидания ответа на команду
    ZB_ERROR_DATA,                          //ошибка в данных при вызове функции
    ZB_ERROR_ACK,                           //нет подтверждения получения пакета данных от координатора
    ZB_ERROR_CRC,                           //ошибка контрольной суммы в полученном пакете данных
    ZB_ERROR_NUMB,                          //ошибка номер уст-ва в полученном пакете не совпадает 
                                            //с номеров в настройках контроллера
    ZB_ERROR_ADDR                           //ошибка адрес уст-ва в полученном пакете не совпадает 
                                            //с адресом в настройках контроллера
 } ZBErrorState;

//Запрос состояния модуля ZigBee 
typedef enum {
    ZB_STATUS_RUN,                          //модуль включен (да/нет)
    ZB_STATUS_NET                           //модуль зарегистрирован в сети (да/нет)
 } ZBDevState;

//Коды ответов модуля ZigBee 
typedef enum {
    ZB_ANSWER_ERROR,                        //ошибка команды
    ZB_ANSWER_BLD_NET,                      //Координатор запросил информацию при установлении сети
    ZB_ANSWER_JOIN_NET,                     //выполнено подключение к сети
    ZB_ANSWER_NO_NET,                       //сеть потеряна
    ZB_ANSWER_SET_CONFIG,                   //Запись конфигурации выполнена
    ZB_ANSWER_RESTART,                      //ZigBee  перезапущен
    ZB_ANSWER_CFG_FACTORY,                  //сброс настроек выполнен
    ZB_ANSWER_NET_RESTART,                  //модуль переподключен к сети
    ZB_ANSWER_ACK,                          //получено подтверждение получение отправленных данных
    ZB_ANSWER_UNDEF                         //ответ не определен
 } ZBAnswer;

//Уровень выходной мощности без PA/c PA
typedef enum {
    ZB_TX_POWER_0,                          //-3/16/20
    ZB_TX_POWER_1,                          //-1.5/17/22
    ZB_TX_POWER_2,                          //0/19/24
    ZB_TX_POWER_3,                          //2.5/20/26
    ZB_TX_POWER_4                           //4.5/20/27
 } ZBTxPower;

//Состояние сети
typedef enum {
    ZB_NETSTATE_NO,                         //No network
    ZB_NETSTATE_OK                          //Network exists
 } ZBNetState;

//*************************************************************************************************
// Параметры для модуля ZigBee
//*************************************************************************************************
//Режим работы ZigBee модуля
typedef enum {
    ZB_DEV_COORDINATOR,                     //координатор
    ZB_DEV_ROUTER,                          //роутер
    ZB_DEV_TERMINAL                         //терминал
 } ZBDevType;

//Режим передачи данных
typedef enum {
    ZB_BROADCASTING = 1,                    //широковещательная передача данных ( 01 + TYPE + DATA )
    ZB_MULTICAST,                           //групповая передача данных ( 02 + GROUP + DATA )
    ZB_ONDEMAND                             //данные по запросу ( 03 + TYPE + ADDR + DATA )
 } ZBSendType;

//Значения параметра type для режима ZB_BROADCASTING
typedef enum {
    ZB_BROADCASTING_MODE1 = 1,              //сообщение транслируется на все устройства во всей сети
    ZB_BROADCASTING_MODE2,                  //сообщение транслируется только на устройства, на которых 
                                            //включен прием (кроме спящих устройств)
    ZB_BROADCASTING_MODE3                   //сообщение рассылается на все полнофункциональные устройства 
                                            //(маршрутизаторы и координаторы)
 } ZBBroadCasting;

//Значения параметра type для режима ZB_ONDEMAND
typedef enum {
    ZB_ONDEMAND_TRANSPARENT = 1,            //Прозрачный режим передачи
    ZB_ONDEMAND_ADDRESS,                    //передача конкретному адресату по короткому адресу
    ZB_ONDEMAND_MAC,                        //передача конкретному адресату по MAC адресу
 } ZBOnDemand;

#pragma pack( push, 1 )

//Структура содержит все параметры модуля ZigBee (для HEX режима обмена данными)
typedef struct {
    ZBDevType   dev_type;                   //device type
    ZBNetState  nwk_state;                  //network state
    uint8_t     pan_id[2];                  //network PAN_ID формат хранения big endian (от старшего к младшему)
    uint8_t     key[16];                    //network key
    uint8_t     short_addr[2];              //network short address - при чтении (заполняется при подключении к сети)
                                            //                      - для записи заполнить 0x0000
                                            //формат хранения big endian (от старшего к младшему)
    uint8_t     mac_addr[8];                //MAC address - при чтении
                                            //            - для записи заполнить 0x00 ... 0x00
    uint8_t     coor_short_addr[2];         //network short address of father node - при чтении
                                            //                                     - для записи заполнить 0x0000
                                            //формат хранения big endian (от старшего к младшему)
    uint8_t     coor_mac_addr[8];           //MAC address of father node
    uint8_t     group;                      //network group number (Net No.)
    uint8_t     chanel;                     //communication channel
    ZBTxPower   txpower;                    //tx power
    uint8_t     baud;                       //uart baud rate
    uint8_t     sleep_time;                 //sleep state
 } ZB_CONFIG;

#pragma pack( pop )

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void ZBInit( void );
void ZBConfig( void );
void ZBRecvComplt( void );
void ZBSendComplt( void );
void ZBCallBack( void );
void ZBCheckConfig( void );
void ZBIncError( ZBErrorState err_ind );
ZBErrorState ZBControl( ZBCmnd command );
ZBErrorState ZBSendPack( uint8_t *data, uint8_t len, uint16_t time_answ );
char *ZBErrCntDesc( ZBErrorState err_ind, char *str );
uint32_t ZBErrCnt( ZBErrorState err_ind );
char *ZBErrDesc( ZBErrorState error );

#endif 
