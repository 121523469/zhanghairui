#ifndef _LORAWAN_H_
#define _LORAWAN_H_
#include "product_config.h"
#ifdef IOT_NET_TYPE_LORAWAN
#include "stdint.h"
#include "SrvIODevice.h"
#include "SrvIODeviceItem.h"
typedef int16_t (*pfun_para1)(uint8_t* pBuff);

#define LORAWAN_POWER_CTRL_PORT    GPIOB
#define LORAWAN_POWER_CTRL_PIN     GPIO_PIN_1
#define LORAWAN_RESET_PORT         GPIOA
#define LORAWAN_RESET_PIN          GPIO_PIN_12

#define LORAWAN_VERSION_MAX        12
/**< ioctl控制命令 */
typedef enum
{
    EN_CMD_ENTER_LOWPOWER,        /**< 模块进入低功耗 */
    EN_CMD_RCV_DATA,              /**< 接收下行数据 */
    EN_CMD_GET_POWER,             /**< 获取TX发射功率 */
    EN_CMD_GET_RSSI,              /**< 获取信号强度 */
    EN_CMD_RESET_MODULER,         /**< 模块重启 */
    EN_CMD_SEARCH_NETWORK,        /**< 搜索网络 */	
    EN_CMD_GET_LPSATAE,           /**< 低功耗模式 */
    EN_CMD_GET_LORAWAN_NET_STATE,
    EN_CMD_SET_MUART_STATE, /*Set muart state*/
    EN_CMD_GET_LORAWAN_MoudleVER,
}EnIoctlCmd_t;

typedef enum
{
    EN_JOIN_FUN,
    EN_SENDDATA_FUN,
    EN_RECBUFF_FUN,
    EN_LOWPOWER_FUN,
    EN_VER_FUN,
    EN_POWER_FUN,
    EN_RESET_FUN,
    EN_ACK_FUN,
    EN_MAX_FUN
}EnFun_t;

typedef struct
{
    EnFun_t Enfun;
    pfun_para1 fp_MsgProc;
}stLoraWanReceivehandle_t;

typedef enum
{
    EN_CMD_VER,
    EN_CMD_JOIN,
    EN_CMD_LOWPOWER,
    EN_CMD_RCVDATA,
    EN_CMD_POWER,
    EN_CMD_RESET,
    EN_CMD_AT,
    EN_CMD_ACK,
}EnAtCMD_t;

typedef enum
{
    EN_JOIN_FAIL,
    EN_JOIN_BUSY,
    EN_JOIN_SUCCESS,
    EN_JOIN_BUTT
}EnJoinNetstate_t;

typedef enum
{
    EN_LORAWAN_SUCCESS,
    EN_LORAWAN_FAIL,
    EN_LORAWAN_BUSY,
    EN_LORAWAN_BUTT
}EnLoraWanState_t;

typedef enum
{
    EN_SEND_DATA_SUCCESS,
    EN_LOWPOWER_SUCCESS,
    EN_RCVDATA_SUCCESS,
    EN_VER_SUCCESS,
    EN_POWER_SUCCESS,
    EN_SEND_DATA_BUTT
}EnLoraWanReposeState_t;

typedef enum
{
    EN_AT_LW_STATE_NULL,
    EN_AT_LW_STATE_WAIT_ACK,
    EN_AT_LW_STATE_TIMEOUT,
    EN_AT_LW_STATE_ACK_OK,
    EN_AT_LW_STATE_BUSY,
    EN_AT_LW_STATE_ACK_ERR,
    EN_AT_LW_STATE_BUT
}EnLWState_t;

typedef struct
{
    EnAtCMD_t enatcmd;
    char*     LoraWanAtCmd;
    EnLWState_t EnLWState;
}stLoraWanNetwork_t;

/** IOT_Transport mode Enum */
typedef enum
{
    EN_TRANS_MODE_ABP = 0x55,
    EN_TRANS_MODE_OTAA = 0xAA,
    EN_TRANS_MODE_BUT,
}EnTransModeType_t;

typedef struct
{
    EnTransModeType_t EnTransMode;
    uint8_t Reserve[3];
}stLoraWanCfg_t;

typedef struct
{
    uint8_t u8RcvDataFlag;
    uint8_t u8LoraWanPsmState;
    stLoraWanCfg_t stLoraWanPara;
    int32_t s32TxPower;
    uint32_t u32SignalStrength;
    uint8_t u8NetState;
    uint8_t u8MuartState;
    uint8_t u8LoraVer[LORAWAN_VERSION_MAX];
}m_stLoraWanItem_t;

extern StSrvIODevice_t stLORAWAN;
extern void Lorawan_configure(stLoraWanCfg_t LoarWanPara);
extern void ModuleDownLinkMsgProc(void);

#endif

#endif