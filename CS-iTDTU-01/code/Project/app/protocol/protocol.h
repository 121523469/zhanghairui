#include "product_protocol.h"
#ifndef PROTOCOL_VER_H3C
#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_
#include <stdint.h>

#define PROTOCO_VER    (1)
#define PROTOCOL_OK       (0)
#define PROTOCOL_ERR        (-1)
#define CMD_BUF_LEN_MAX (256)
#define VALID_CMD_HEADER    (0X55)
#define VALID_CMD_EOC    (0XAA)
#define CMD_HEADER_LEN    (2)
#define CMD_EOC_LEN    (2)

#define PRO_CMD_MAX    (16)
#define PACKET_SIZE     (1000)

typedef enum
{
    EN_PRO_FUNCCODE_NULL_0,
    EN_PRO_FUNCCODE_POWERON_REPORT = 1,     /**< 开机信息上报 */
    EN_PRO_FUNCCODE_TIMELY_REPORT = 2,      /**< 定时上报 */
    EN_PRO_FUNCCODE_CONFIG = 3,             /**< 配置命令报文 */
    EN_PRO_FUNCCODE_RESET = 4,              /**< 复位命令报文 */
    EN_PRO_FUNCCODE_REV_1 = 5,             /**< 布防撤防命令报文 */
    EN_PRO_FUNCCODE_CANCEL_ALM = 6,         /**< 撤销报警命令报文 */
    EN_PRO_FUNCCODE_READ_POWERON = 7,       /**< 读取开机信息命令报文 */
    EN_PRO_FUNCCODE_NULL_8 = 8,
    EN_PRO_FUNCCODE_RESTORE = 9,            /**< 回复出厂设置命令报文 */
    EN_PRO_FUNCCODE_CONTROL = 10,
    EN_PRO_FUNCCODE_ACK = 0xAA,             /**< 应答命令报文 */
    EN_PRO_FUNCCODE_BUT
}EnProFuncCode_t;

typedef enum
{
    EN_PRO_CMD_STATE_NULL,
    EN_PRO_CMD_STATE_WAIT_ACK,
    EN_PRO_CMD_STATE_ACK_ERR,
    EN_PRO_CMD_STATE_ACK_OK,
    EN_PRO_CMD_STATE_TIMEOUT,
}EnProCmdState_t;
typedef uint16_t (*pfProCmdHeader)(void *p_pvParam);
typedef int8_t (*pfProNfcCmdFunc)(uint32_t Param);

/** 设备状态宏定义 */
#define PROTO_TERMSTATE_BAT           (0x01u)  /**< BP： 0-电池正常，1-电池欠压 */
#define PROTO_TERMSTATE_ACK           (0x02u)  /**< ACK：0-云端应答正常，1-云端应答错误 */
#define PROTO_TERMSTATE_WM            (0x04u)  /**< WM： 0-无线模块状态正常，1-无线模块状态异常 */
#define PROTO_TERMSTATE_SF            (0x08u)  /**< SF： 0-设备撤防，1-设备布防 */
#define PROTO_TERMSTATE_AL            (0x10u)  /**< AL:     0-无报警，1-报警 */

/** 错误码宏定义 */
#define PROTO_ERRCODE_OK              (0x00u)   /**< 没有错误 */
#define PROTO_ERRCODE_INTERAL         (0x01u)   /**< 内部错误 */
#define PROTO_ERRCODE_CRC             (0x02u)   /**< CRC错误	Crc校验错误 */
#define PROTO_ERRCODE_PARAM           (0x03u)   /**< 参数错误 */
#define PROTO_ERRCODE_NUM             (0x04u)

/** 模拟通道使能定义 */
#define PROTO_ANALOGCH_VOL            (0x0001u)   /**< 0-通道禁止使能，1-通道使能 */
#define PROTO_ANALOGCH_CUR            (0x0010u)   /**< 0-通道禁止使能，1-通道使能 */
#define PROTO_ANALOGCH_PWM            (0x0100u)   /**< 0-通道禁止使能，1-通道使能 */
#define PROTO_ANALOGCH_SW             (0x1000u)   /**< 0-通道禁止使能，1-通道使能 */


/** 设备类型定义 */
#define PROTO_TERMTYPE_DTU    (0x1Bu)   /**< 车位终端 */

#pragma pack(1)
typedef struct 
{
    uint32_t Len  ;
    uint32_t OffSet;
    uint32_t CmdEndPos;
    uint8_t CmdBuf[CMD_BUF_LEN_MAX];
}StCmdBuf_t;
#pragma pack()

#pragma pack(1)
/** Frame   header  struct */
typedef struct {
    uint8_t u8ProtocolVer;        /**< Protocol    Version */
    uint8_t u8FuncCode;           /**< Function code */
    uint16_t u16DeviceId;         /**<Device ID */
    uint16_t u16MsgSn;            /**< Message    SN */
    uint16_t u16DataLength;       /**< Data length of  */
}StFrameHeader_t;
#pragma pack()

#pragma pack(1)
/** firmware version struct */
typedef struct {
    uint8_t u8Release;          /**< firmware release */
    uint8_t u8MinorVersion;     /**< firmware minor version */
    uint8_t u8MajorVerison;     /**< firmware major version */
    uint8_t u8Reserve;
}StFWVersion_t;
#pragma pack()

#pragma pack(1)
/** Power on report frame struct 开机报文结构体 */
typedef struct {
    StFrameHeader_t stFrameHeader;
    uint32_t u32DeviceSn;              /**< Device SN */
    uint8_t u8DeviceType;              /**< Device type */
    uint8_t u8HWVersion;               /**< Hardware version */
    StFWVersion_t stFWversion;         /**< Firmware version */
    uint16_t u16Reserve_0;
    uint16_t u16HeartBeatInteval; /**< Timely repeat internal */
    uint16_t u16SampleInteval;         /**< Timely Alarm internal */
    uint16_t u16Reserve_1;          /**< Timely Alarm internal */
    StPoweronFramePayload_t stPayload;    /**< 其他数据 */
    uint8_t Reserve[3];
    uint16_t u16CRC;                   /**< CRC checksum */
}StPoweronFrame_t;
#pragma pack()

#pragma pack(1)
/** 定时上报报文结构体 */
typedef struct {
    StFrameHeader_t stFrameHeader;
    uint32_t u32DeviceSn;          /**< Device SN */
    uint16_t u16DeviceState;       /**< Device state */
    uint8_t u8BatteryLeftVol;      /**< Battery left voltage*/
    uint8_t u8Reserve;             /**< reserve*/
    int32_t s32Rssi;              /**< Rssi*/
    StRegularUploadPayload_t stPayload;    /**< 其他数据 */
    uint16_t u16CRC;               /**< CRC checksum */
}StRegularUpload_t;
#pragma pack()

#pragma pack(1)
/** Ack frame 应答报文结构体 */
typedef struct {
    StFrameHeader_t stFrameHeader;
    uint8_t u8ErrCode;              /**< Error code */
    uint8_t u8ResponseFuncCode;     /**< 被应答消息功能码 */
    uint16_t u16CRC;                /**< CRC checksum */
}StACKFrame_t;
#pragma pack()

#pragma pack(1)
/** 配置命令结构体 */
typedef struct {
    StFrameHeader_t stFrameHeader;
    uint16_t u16NewDeviceId;            /**< New Device ID */
    uint16_t u16HeartBeatInteval;       /**< Timely repeat internal， FFFF时表示保持当前设置不变 */
    uint16_t u16SampleInteval;          /**< Sample internal， FFFF时表示保持当前设置不变 */
    uint32_t u32IpAddr;                 /**< Ip address setting， FFFFFFFF时表示保持当前设置不变 */
    uint16_t u16Port;                   /**< Port number set，FFFF时表示保持当前设置不变 */
    StConfigureCmdPayload_t stPayload;
    uint8_t Reserve[2];
    uint16_t u16CRC;                    /**< CRC checksum */
}StConfigureCmd_t;
#pragma pack()

#pragma pack(1)
/** 休眠模式命令结构体 */
typedef struct {
    StFrameHeader_t stFrameHeader;
    uint16_t u16CRC;               /**< CRC checksum */
}StSleepModeCmd_t;
#pragma pack()

#pragma pack(1)
/** 采集背景磁场命令结构体 */
typedef struct {
    StFrameHeader_t stFrameHeader;
    uint16_t u16CRC;               /**< CRC checksum */
}StGetBaseMagnitCmd_t;
#pragma pack()

#pragma pack(1)
/** 布防命令结构体 */
typedef struct {
    StFrameHeader_t stFrameHeader;
    uint16_t u16DefendState;         /**< Defend status  */
    uint16_t u16CRC;               /**< CRC checksum */
}StDefendCmd_t;
#pragma pack()

/** 复位命令结构体 */
#pragma pack(1)
typedef struct {
    StFrameHeader_t stFrameHeader;
    uint16_t u16CRC;               /**< CRC checksum */
}StResetCmd_t;
#pragma pack()

#pragma pack(1)
typedef struct {
    StFrameHeader_t stFrameHeader;
    uint8_t u8ControlStatus;
    uint16_t u16CRC;               /**< CRC checksum */
}StDOControlCmd_t;
#pragma pack()


#pragma pack(1)
/** restore   struct define */
typedef struct {
    StFrameHeader_t stFrameHeader;
    uint16_t u16CRC;               /**< CRC checksum */
}StRestoreCmd_t;
#pragma pack()

#pragma pack(1)
/** Cancel   Alarm   struct */
typedef struct {
    StFrameHeader_t stFrameHeader;
    uint16_t u16CRC;               /**< CRC checksum */
}StCancelAlarmCmd_t;
#pragma pack()

#pragma pack(1)
/** Read power on report cmd struct */
typedef struct {
    StFrameHeader_t stFrameHeader;
    uint16_t u16CRC;               /**< CRC checksum */
}StReadPowerOnReportCmd_t;
#pragma pack()


typedef struct
{
    uint16_t u16PacketTotal;
    uint16_t u16PacketCRC;
    uint8_t u8UpgradeFlag;
    uint8_t u8CRCCheckFlag;
    uint8_t u32Reserve[6];
}StShareInfo_t;

#pragma pack(1)
typedef struct
{
    StFrameHeader_t stFrameHeader;
    uint16_t u16PacketTotal;
    uint16_t u16PacketCRC;
    uint16_t u16CRC;
}StUpgradeCMD_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
    StFrameHeader_t stFrameHeader;
    uint16_t u16PacketNum;
    uint16_t u16Reserve;
    uint8_t u8Packet[PACKET_SIZE];
    uint16_t u16CRC;
}StUpgradeData_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
    StFrameHeader_t stFrameHeader;
    uint16_t u16PacketNum;
    uint8_t u8ErrCode;              /**< Error code */
    uint8_t u8ResponsedFuncCode;     /**< 被应答消息功能码 */
    uint16_t u16CRC;                /**< CRC checksum */
}StUpgradeDataACK_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
    StFrameHeader_t stFrameHeader;
    uint16_t u16PacketNum;
    uint8_t u8ErrCode;              /**< Error code */
    uint8_t u8ResponsedFuncCode;
    uint16_t u16CRC;                /**< CRC checksum */
}StUpgradeReqData_t;
#pragma pack()

extern StShareInfo_t stShareInfo;

extern int16_t Pro_UpgradeCmdFun(uint8_t* p_pu8RcvBuffer);
extern int16_t Pro_UpgradeDataHandle(uint8_t* p_pu8RcvBuffer);
extern int8_t Pro_Init(void);
extern uint16_t Pro_SendPowerOnReportFrame(EnProFuncCode_t enFuncCode, uint16_t u16MsgSn);
extern int16_t Pro_SendRegularUpload(void);
extern void Pro_CmdParseHandle(uint8_t* p_pu8RcvBuffer);
extern EnProCmdState_t ProGetCmdAckState(EnProFuncCode_t FuncCode);
extern int16_t Pro_SendUpgradeReqData(void);
extern int16_t Pro_SendUpgradeDataAck(void);
extern int8_t Pro_UpdateDeviceId(uint16_t u16DevId);
extern int8_t Pro_UpdateHeartBeatInteval(uint16_t u16Inteval);
extern int8_t Pro_UpdateDetLevel(uint8_t p_u8DetLev);
extern int8_t Pro_UpdateFreeAffirmThd(uint8_t p_u8FreeAffirmThd);
extern int8_t Pro_UpdateALarmAffirmThd(uint8_t p_u8AlmAffirmThd);

#ifdef IOT_NET_TYPE_NBIOT
extern int8_t Pro_UpdatePortNum(uint16_t p_u16Port);
extern int8_t Pro_UpdateIpAddr(uint32_t p_u32IpAddr);
extern int8_t Pro_UpdateNbiotMode(uint8_t p_u8Mode);
#endif
extern uint16_t CRC16(uint8_t *dataIn,uint32_t length);

#endif
#endif
