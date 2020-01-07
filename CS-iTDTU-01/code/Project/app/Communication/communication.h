#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "stdint.h"
#include "product_protocol.h"
typedef union
{
    uint8_t  u8[4];
    uint32_t u32;
}UNU32U8_t;

#define MENU_NBIOT_OFF
/**************************************************************************************************
 * type defines
 *************************************************************************************************/

typedef int16_t (*pfComMenuIndexFunc)(void);

/** 用户配置菜单 */
typedef enum
{
    EN_USERMENU_SETDEVICEID = 0,  /**< 设置设备ID */
#ifdef IOT_NET_TYPE_NBIOT
    EN_USERMENU_SETIP,                           /**< 设值IP和PORT */
    EN_USERMENU_NBMODE,                      /*设置nbiot通信方式*/
#endif
#ifdef IOT_NET_TYPE_LORAWAN
    EN_USERMENU_SETLWMODE,
#endif
    EN_USERMENU_SETINTERVAL,                     /**< 设值定时上报间隔，报警时间间隔，采样时间间隔 */
    EN_USERMENU_SETAICH1,
    EN_USERMENU_SETAICH2,
    EN_USERMENU_SHOWPARA,                        /**< 查看当前所有参数 */
    EN_USERMENU_SERTRANS,                        /**< 透传模式 */
    EN_USERMENU_FAC,                             /**< 恢复出厂设置 */
    EN_USERMENU_EXIT,
    EN_USERMENU_MAX
}ENUserMenuID_t;
/** 管理员配置菜单 */
typedef enum
{
    EN_ADMINMENU_SETHWVERSION = EN_USERMENU_MAX,               /**< 设置硬件版本 */
    EN_ADMINMENU_SETDEVICESN,                    /**< 设置设备SN */
    EN_ADMINMENU_BACKUP,
    EN_ADMINMENU_MAX
}ENAdminMenuID_t;



/** 通道设置菜单 */
typedef enum
{
    EN_CHMENU_DISABLE = 0,                       /**< 通道禁用 */
    EN_CHMENU_ENABLE,                            /**< 通道使能 */
    EN_CHMENU_SETALAUP,                          /**< 设置报警上限值 */
    EN_CHMENU_SETALALOW,                         /**< 设置报警下限值 */
    EN_CHMENU_SETZERO,                           /**< 设置标定零点值 */
    EN_CHMENU_SETFS,                             /**< 设置标定满点值 */
    EN_CHMENU_EXIT,                              /**< 退出配置通道菜单，返回上级菜单 */
    EN_CHMENU_MAX
}ENChannelMenu_t;

/** 菜单目录简单描述和输入提示 */
typedef struct
{
    int8_t s8MenuID;                             /**< 菜单Index */
    char *pcDes;                                 /**< 简要说明 */
    char *pcHelp;                                /**< 输入提示 */
    pfComMenuIndexFunc fpMenuIndex;
}StMenuItem_t;

typedef enum
{
    EN_MENU_STATE_TIMEOUT,
    EN_MENU_STATE_ACK_OK,
}ENCOMTRANSState_t;

typedef struct
{
    volatile uint8_t u8ComEXTIFlag;
    volatile uint8_t u8ComEnterFlag;
    volatile uint8_t u8ComSerialTransFlag;
    volatile uint8_t u8ComMenuIndex;
    volatile uint8_t u8ComSecCount;
    volatile uint8_t u8ComConfigFlag;
    volatile uint8_t u8nbPSMstate;
}StComMenuItem_t;


extern void Com_process(void);
extern void Com_Init(void);
extern void ComSetConfigFlag(void);
#endif
