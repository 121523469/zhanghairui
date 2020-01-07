#ifndef _PRODUCT_CONFIG_H
#define _PRODUCT_CONFIG_H
#include "stdint.h"

/*moduler support define*/
#define IOT_NET_TYPE_NBIOT
//#define IOT_NET_TYPE_LORAWAN

/*sw ver define*/
#define RELEASE_SW_VER    (1)
#define MINOR_SW_VER    (0)
#define MAJOR_SW_VER    (1)
#define RESERVE_SW_VER    (0)

#define VER_MONTH       (0x01)
#define VER_DAY         (0x09)

/*device name define*/
#define DEVICE_NAME "CS-iTDTU-01"

/*device type define*/
#define DEVICE_TYPE       PROTO_TERMTYPE_DTU

/*system feature support define*/
//#define UART_RXTX_SWAP_FEATURE 

/*device protocol define*/
//#define PROTOCOL_VER_H3C
#endif
