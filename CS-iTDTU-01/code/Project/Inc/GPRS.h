#ifndef __GPRS_H
#define __GPRS_H

#include "stdint.h"
#include "SrvIODevice.h"
/**< 错误类型 */
typedef enum 
{
    EN_GPRS_ERR_OK,        /**< 没有错误 */
    EN_GPRS_ERR_NULL,      /**< 空指针 */
    EN_GPRS_ERR_TIMOUT,    /**< 超时 */
    EN_GPRS_ERR_NONEP,     /**< 无参数 */
    EN_GPRS_ERR_FAIL,      /**< 发送数据失败 */
    EN_GPRS_ERR_ATE0,      /**< 关闭回显错误 */
    EN_GPRS_ERR_CPIN,      /**< 检查SIM卡状态错误 */
    EN_GPRS_ERR_CSQ,       /**< 检查信号强度错误 */
    EN_GPRS_ERR_CREG,      /**< 检查网络注册状态错误 */
    EN_GPRS_ERR_CGATT,     /**< 检查GPRS附着状态错误 */
    EN_GPRS_ERR_CSTT,      /**< 设置APN错误 */
    EN_GPRS_ERR_CIICR,     /**< 建立无线链路错误 */
    EN_GPRS_ERR_CIFSR,     /**< 获得本地IP错误 */
    EN_GPRS_ERR_CIPSTART,  /**< 建立TCP/UDP链接失败 */
    EN_GPRS_ERR_CIPSEND,   /**< 没有收到'<' */
    EN_GPRS_ERR_CIPSTATUS, /**< 建立TCP/UDP链接失败 */
    EN_GPRS_ERR_CIPSHUT,   /**< 关闭移动场景失败 */
    EN_GPRS_ERR_CIPRXGET,  /**< 关闭移动场景失败 */
	  //EN_GPRS_ERR_CIPSTART,/**< 建立TCP/UDP链接失败 */
	  //EN_GPRS_ERR_CIPSTART,/**< 建立TCP/UDP链接失败 */
}EnGPRSErrCode_t;

typedef struct
{
	uint32_t u32IpAddr;           /**< Ip address setting,FFFFFFFF时表示保持当前设置不变 */
	uint16_t u16Port;             /**< Port number set,FFFF时表示保持当前设置不变 */
}StGPRSNetPara_t;
/**< GPRS状态结构体 */
typedef struct
{
    volatile uint8_t u8JoinNetState;
    volatile uint8_t u8RcvUartFlag;
    volatile uint8_t *m_u8Gprscallback;
}StGPRSItem_t;

/**< AT command */
typedef enum
{
	  EN_GPRS_ATE0,           /**< 关闭回显 */
    EN_GPRS_CPIN,           /**< 检查SIM卡状态 */
    EN_GPRS_CSQ,            /**< 检查网络信号强度 */
    EN_GPRS_CREG,           /**< 检查网络注册状态 */
    EN_GPRS_CGATT,          /**< 检查GPRS附着状态 */
    EN_GPRS_CSTT,           /**< 设置APN */
    EN_GPRS_CIICR,          /**< 建立无线链路（GPRS或CSD）*/
    EN_GPRS_CIFSR,          /**< 获得本地IP地址 */
    EN_GPRS_CIPSTART,       /**< 建立TCP链接 */
    EN_GPRS_CIPSEND,        /**< 发送数据到服务器 */
		EN_GPRS_CIPSTATUS,      /**< 确认链路链接状态 */
		EN_GPRS_CIPSHUT,        /**< 关闭移动场景 */
		EN_GPRS_CIPRXGET,       /**< 允许手动获取数据 */
		EN_GPRS_CIPRXGET_HEX,   /**< 16进制模式读数据 */
	  EN_GPRS_CGMR,           /**< 固件版本 */
	//EN_GPRS_CIPRXGET_HEX,   /**< 信号强度 */
}EnGPRSCommand_t;
extern StSrvIODevice_t stGPRSDevice;
void GPRS_configure(void);
#endif
