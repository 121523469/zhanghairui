#ifndef _NBIOT_H_
#define _NBIOT_H_

#include "product_config.h"

#ifdef IOT_NET_TYPE_NBIOT

#include "stdint.h"
#include "SrvIODevice.h"
#include "SrvIODeviceItem.h"

typedef uint32_t (*UartRingBufferCallback)(uint8_t* Data, uint32_t Len);

#define NBIOT_RESET_PORT        GPIOA
#define NBIOT_RESET_PIN        GPIO_PIN_12

#define NBIOT_POWER_CTRL_PORT    GPIOB
#define NBIOT_POWER_CTRL_PIN        GPIO_PIN_3

#define NBIOT_WAKEUP_PORT      GPIOC
#define NBIOT_WAKEUP_PIN      GPIO_PIN_8

#define NBIOT_PWKEN_PORT    GPIOC
#define NBIOT_PWKEN_PIN    GPIO_PIN_9

#define IMEI_LEN_MAX    (16)
#define IMSI_LEN_MAX    (16)
typedef enum 
{
    CHECK_UART,
    CLOSE_ECHO,
    CHECK_SIM_STATE,
    MODULER_ENABLE,/*moduler enabled*/
    CHECK_CSQ,
    ATTACH,    /*附着*/
    NET_REG,
    REGISTED,       /*已注册上*/
    STATUS_BUTT,
}EnNBIOTNetStatus_t;

typedef enum 
{
    SET_RECVMODE,
    TCP_CLOSE,
    TCP_SETUP,
    CHECK_IPSTATUS,
    TCP_SUCCESS,
}EnNBIOTTCPStatus_t;

typedef struct
{
    uint32_t u32IpAddr; /**< Ip address setting,FFFFFFFFʱ��ʾ���ֵ�ǰ���ò��� */
    uint16_t u16Port;   /**< Port number set,FFFFʱ��ʾ���ֵ�ǰ���ò��� */
}StNBIOTNetPara_t;

/** IOT_Transport mode Enum */
typedef enum
{
    EN_TRANS_MODE_COAP = 0x55,
    EN_TRANS_MODE_UDP  = 0xAA,
    EN_TRANS_MODE_TCP  = 0x5A,
    EN_TRANS_MODE_MQTT = 0x6A,
    EN_TRANS_MODE_ONENET = 0x7A,
}EnTransModeType_t;//EnLoraWanTransMode_t;

typedef struct
{
    uint16_t u16Port;                   /**< Port number set，FFFF时表示保持当前设置不变 */
    uint32_t u32IpAddr;                 /**< Ip address setting， FFFFFFFF时表示保持当前设置不变 */
    EnTransModeType_t EnTransMode;
    uint8_t Reserve[3];
}stNbiotCfg_t;

/**< ioctl控制命令 */
typedef enum
{
    EN_CMD_GETECL,            /**< Get CellID */
    EN_CMD_GETRSSI,              /**< Get Rssi */
    EN_CMD_REGRECVCALLBACK,      /**< 注册回调函数 */
    EN_CMD_DLMSG_PROC,       /*nbiot下行消息处理*/
    EN_CMD_PSM_STATE,/*NBIOT psm state*/	
    EN_CMD_SEARCH_NETWORK,/*搜网*/
    EN_CMD_GET_IMEI,/*qry imei*/
    EN_CMD_GET_IMSI,/*qry imsi*/
    EN_CMD_SET_MUART_STATE,
    EN_CMD_GET_NBIOT_NET_STATE,/*nbiot net state*/
    EN_CMD_WAKEUP,
    EN_CMD_ENABLE_PSM,
    EN_CMD_DISABLE_PSM,
}EnIoctlCmd_t;

typedef enum
{
    EN_CMD_AT,
    EN_CMD_ATE0,
    EN_CMD_NEONBIOTCFG,
    EN_CMD_CGATT_QRY,
    EN_CMD_CEREG_QRY,
    EN_CMD_CSQ,
    EN_CMD_TUESTATS_SET,
    EN_CMD_NCDP_SET,
    EN_CMD_NCDPOPEN,
    EN_CMD_NNMI_SET,
    EN_CMD_SET_CGATT,
    EN_CMD_SET_CGATT_DETACH,
    EN_CMD_QRY_CFUN,
    EN_CMD_SET_CFUN,
    EN_CMD_CIMI,
    EN_CMD_NMGS,
    EN_CMD_QRY_IMEI,
    EN_CMD_NVSETPM_NORMAL,
    EN_CMD_NVSETPM_LP,
    EN_CMD_PSM_SET,
    EN_CMD_EDRX_SET,
    EN_CMD_XIIC_SET,
    EN_CMD_XIIC_QRY,
    EN_CMD_RECVMODE,
    EN_CMD_UDP_CLOSE,
    EN_CMD_UDP_SETUP,
    EN_CMD_UDP_SEND,
    EN_CMD_TCPCLOSE,
    EN_CMD_TCP_SETUP,
    EN_CMD_TCP_SEND,
    EN_CMD_IPSTATUS,
    EN_CMD_MAX
}enNbiotAtCmd_t;



typedef enum
{
    EN_NBIOT_NBAND_CTCC = 5,/*中国电信NBIOT band*/
    EN_NBIOT_NBAND_CMCC = 8,/*中国移动NBIOT band*/
    EN_NBIOT_NBAND_MAX
}EnNbiotBand_t;

typedef enum
{
    EN_SOCKET_DISABLE,/*socket disable*/
    EN_SOCKET_ENABLE,/*socket enable*/
    EN_SOCKET_BUT
}EnSocketState_t;

typedef enum
{
    EN_AT_STATE_NULL,/*socket enable*/
    EN_AT_STATE_WAIT_ACK,/*socket disable*/
    EN_AT_STATE_TIMEOUT,
    EN_AT_STATE_ACK_OK,
    EN_AT_STATE_ACK_ERR,
    EN_AT_STATE_BUT
}EnAtState_t;

typedef struct 
{
    enNbiotAtCmd_t CmdIndex;
    uint8_t *CmdStr;
    EnAtState_t EnCmdState;
}StAtCmdTab_t;

#pragma pack(1)
typedef struct
{
    uint8_t u8ECL;
    uint8_t s8SNR;
    uint16_t u16PCI;
    uint32_t u32CellID;
}StNBReadECL_t;
#pragma pack()

#pragma pack(1)
/**< NBIOT   */
typedef struct
{
    volatile uint8_t u8RcvUartFlag; /*一帧接受完成标志，1--完成一帧接收*/
    volatile EnNbiotBand_t enBand; /*当前网络支持band*/
    volatile uint8_t u8CfunState;
    volatile uint8_t u8CgattState;
    volatile uint8_t u8CeregState;
    volatile int32_t s32Rssi;
    volatile StNBReadECL_t stNbiotEcl;
    volatile EnNBIOTNetStatus_t enNetState;/*入网状态*/
    volatile uint8_t StNbiotPsmState;
    uint8_t u8IpInfoStr[32];
    volatile EnSocketState_t u8SocketState;    /*socekt state*/
    UartRingBufferCallback p_UartRingBufferCallback;/*Muart interrupt call back*/
    stNbiotCfg_t stNbiotCfgPara;
    volatile uint8_t u8SocketNum;/*udp socket 序号:0--7*/
    uint8_t u8Imei[IMEI_LEN_MAX];
    uint8_t u8Imsi[IMSI_LEN_MAX];
    uint8_t u8MuartState;/*muart init state,0--enable，1--disable*/
    uint8_t u8NetState;/*net state,0--no signal，1--signal ok*/
    volatile EnNBIOTTCPStatus_t EnTCPStatus;
    volatile uint8_t u8NcdOopenState;
    volatile uint8_t u8PsmState;
}StNBIOTItem_t;
#pragma pack()

#define NBIOT_GET_RCV_FALG() m_stNBIOTItem.u8RcvUartFlag
extern StSrvIODevice_t stNBIOT;
extern void nbiot_configure(stNbiotCfg_t p_stNbiotCfg);
extern int16_t NbGetSocketState(void);

extern void ModuleDownLinkMsgProc(void);

#endif
#endif