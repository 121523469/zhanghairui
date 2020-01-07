#ifndef 	_IOCTLCMD_H_
#define 	_IOCTLCMD_H_
#if 0
/**< ioctl控制命令 */
typedef enum
{
	EN_CMD_GETCELLID,            /**< Get CellID */
	EN_CMD_GETRSSI,              /**< Get Rssi */
	EN_CMD_GETBAND,              /**< Get BAND */
	EN_CMD_GETVERSION,           /**< Get Version */
	EN_CMD_REGEXTICALLBACK,      /**< 注册回调函数 */
	EN_CMD_REGRECVCALLBACK,      /**< 注册回调函数 */
	EN_CMD_SETSLEEPMODULE,       /**< 睡眠模式 */
	EN_CMD_SETMANUCONFIGMODULE,  /**< 出厂配置模式 */
	EN_CMD_SERIALTRANSMODULE,    /**< 透传模式 */
	EN_CMD_PA3EXTIMODE,          /**< PA3切换为外部中断模式 */
	EN_CMD_PA3UARTRXMODE,        /**< PA3切换为UART2 RX 模式 */
	//and so on
}EnModuleCmd_t;
#endif


#endif

