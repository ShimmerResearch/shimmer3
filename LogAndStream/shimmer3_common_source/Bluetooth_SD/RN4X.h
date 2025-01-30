#ifndef RN4X_H
#define RN4X_H

#include <stdint.h>

/* Enables BLE if FW is LogAndStream and the RN4678 is detected */
#define BT_ENABLE_BLE_FOR_LOGANDSTREAM_AND_RN4678 1

/********** Defines created when testing Bluetooth driver - Start ***********/
/* This toggles the CTS pin whenever the DMA interrupt is being processed to tell the BT module to stop sending bytes while the MCU is not in a position to recieve them */
#define BT_CTS_CONTROL_ENABLED 1
/* Experimental feature to turn status strings on for the RN42 in order to make it perform the same way as the RN4678 */
#define BT_RN42_STAT_STR_ENABLED 0
/* Toggles between using DMA or a FIFO buffer based on the UART interrupt for receiving bytes from the BT module */
#define BT_DMA_USED_FOR_RX 1
/* useful just for debugging but not needed in run-time */
#define BT_CLEAR_RX_BUF_DURING_READ 0
/* Controls whether the baud rate can be changed by the sensor configuration */
#define BT_ENABLE_BAUD_RATE_CHANGE 0
/* Experimental feature to try and reawaken the RN4678 if an RTS lock is detected */
#define BT_FLUSH_TX_BUF_IF_RN4678_RTS_LOCK_DETECTED 0
/********** Defines created when testing Bluetooth driver - End ***********/

/* Custom advertising name */
#define ADVERTISING_NAME_IS_OUTPUT 0

#define ADVERTISING_NAME_OUTPUT "OUTPUT\0"
#define ADVERTISING_NAME_SHIMMER3 "Shimmer3\0"

#define BLE_ADVERTISING_NAME_SHIMMER3 "S3BLE\0"

typedef enum
{
    /* Default mode (SM,0)  In this mode, other Bluetooth devices can discover
    * and connect to the module. Outbound connections can also be initiated in
    * this mode.*/
    SLAVE_MODE                  = 0, /* Supported in RN42 and RN4678 */
    MASTER_MODE                 = 1, /* Supported in RN42 */
    TRIGGER_MODE                = 2, /* Supported in RN42 */
    /* Can be used if the target slave MAC address is stored in the Bluetooth
     * module using the "SR," command rather then being passed on the fly using
     * the "C," command */
    AUTO_CONNECT_MASTER_MODE    = 3, /* Supported in RN42 */
    AUTO_CONNECT_DTR_MODE       = 4, /* Supported in RN42 */
    AUTO_CONNECT_ANY_MODE       = 5, /* Supported in RN42 */
    /* Pairing mode (SM,6)  In this mode, the module attempts to connect to the
    * remote device whose MAC address matches the value stored in the
    * Remote Address field in the module. The command SR sets the
    * Remote Address field. */
    PAIRING_MODE                = 6 /* Supported in RN42 and RN4678 */
} btOperatingMode;

typedef enum
{
    BT_SETUP,
    SHIMMER_CMD,
    SENSOR_DATA
} btResponseType;

/* Order here needs to be maintained as it's saved to the EEPROM */
enum BT_FIRMWARE_VERSION
{
    BT_FW_VER_UNKNOWN,
    RN42_V4_77,
    RN42_V6_15,
    RN42_V6_30, // Not supported
    RN4678_V1_00_5,
    RN4678_V1_11_0,
    RN4678_V1_13_5,
    RN4678_V1_22_0,
    RN4678_V1_23_0,
    RN41_V4_77,
};

enum BT_HARDWARE_VERSION
{
    RN42 = 0U,
    RN4678 = 1U,
    RN41 = 2U,
    BT_HW_VER_UNKNOWN = 0xFF,
};

/* Order here needs to be maintained as it's saved to the EEPROM */
enum BT_BAUD_RATE
{
    BAUD_115200     = 0U,
    BAUD_1200       = 1U,   // Only supported in RN42
    BAUD_2400       = 2U,
    BAUD_4800       = 3U,
    BAUD_9600       = 4U,
    BAUD_19200      = 5U,
    BAUD_38400      = 6U,
    BAUD_57600      = 7U,
    BAUD_230400     = 8U,   // Only supported in RN42
    BAUD_460800     = 9U,   // Only supported in RN42
    BAUD_921600     = 10U,  // Only supported in RN42
    BAUD_1000000    = 11U,  // Only supported in RN4678 v1.23 (issues with v1.13.5 & v1.22)
    BAUD_NO_CHANGE_NEEDED = 0xFF,
};

typedef enum
{
    RN4678_TX_POWER_MINUS_20_DBM = 0,
    RN4678_TX_POWER_MINUS_7_DBM,
    RN4678_TX_POWER_MINUS_2_DBM,
    RN4678_TX_POWER_0_DBM,
    RN4678_TX_POWER_PLUS_2_DBM
} rn4678TxPower_et;

typedef enum
{
    RN42_TX_POWER_PRE_AUG_2012_PLUS_12_DBM = 0,
    RN42_TX_POWER_PRE_AUG_2012_PLUS_6_DBM,
    RN42_TX_POWER_PRE_AUG_2012_PLUS_2_DBM,
    RN42_TX_POWER_PRE_AUG_2012_0_DBM,
    RN42_TX_POWER_PRE_AUG_2012_MINUS_5_DBM,
    RN42_TX_POWER_PRE_AUG_2012_MINUS_10_DBM,
    RN42_TX_POWER_PRE_AUG_2012_MINUS_20_DBM,
} rn42TxPowerPreAug2012_et;

typedef enum
{
    RN42_TX_POWER_POST_AUG_2012_PLUS_16_DBM = 0,
    RN42_TX_POWER_POST_AUG_2012_PLUS_12_DBM,
    RN42_TX_POWER_POST_AUG_2012_PLUS_8_DBM,
    RN42_TX_POWER_POST_AUG_2012_PLUS_4_DBM,
    RN42_TX_POWER_POST_AUG_2012_0_DBM,
    RN42_TX_POWER_POST_AUG_2012_MINUS_4_DBM,
    RN42_TX_POWER_POST_AUG_2012_MINUS_8_DBM,
    RN42_TX_POWER_POST_AUG_2012_MINUS_12_DBM
} rn42TxPowerPostAug2012_et;

enum BT_SET_COMMAND_STAGES
{
    WAIT_FOR_BOOT,
    CMD_MODE_START,
    GET_VERSION,
    RESET_FACTORY_DEFAULTS,
    GET_OPERATING_MODE,
    SET_OPERATING_MODE,
    GET_MAC_ADDRESS,
    SET_MASTER_MAC,
    GET_AUTHENTICATION,
    SET_AUTHENTICATION,
    ENABLE_ENCRYPTION,
    GET_ADVERTISING_NAME,
    SET_ADVERTISING_NAME,
    GET_PIN,
    SET_PIN,
    SET_SVC_CLASS_REQUEST,
    SET_DEV_CLASS_REQUEST,
    SET_SVC_NAME_REQUEST,
    RN42_GET_REMOTE_CONFIGURATION_TIMER,
    RN42_SET_REMOTE_CONFIGURATION_TIMER,
    GET_INQUIRY_SCAN_WINDOW,
    SET_INQUIRY_SCAN_WINDOW,
    GET_PAGING_TIME,
    SET_PAGING_TIME,
    GET_BT_POWER_LEVEL,
    SET_BT_POWER_LEVEL,
    RN4678_GET_BT_MODE,
    RN4678_SET_BT_MODE,
    RN4678_GET_FAST_MODE,
    RN4678_SET_FAST_MODE,
    RN4678_GET_MODEL_STRING,
    RN4678_SET_MODEL_STRING,
    RN4678_GET_MANUFACTURER_STRING,
    RN4678_SET_MANUFACTURER_STRING,
    RN4678_GET_SOFTWARE_REVISION_STRING,
    RN4678_SET_SOFTWARE_REVISION_STRING,
    RN4678_GET_CONNECTION_PARAMETERS,
    RN4678_SET_CONNECTION_PARAMETERS,
    GET_STAT_STR_PREFIX_AND_SUFFIX,
    UPDATE_STAT_STR_PREFIX_AND_SUFFIX,
    UPDATE_BAUD_RATE_1,
    UPDATE_BAUD_RATE_2,
    UPDATE_BAUD_RATE_3,
    RN42_REENTER_CMD_MODE,
    REBOOT,
    RN4678_REENTER_CMD_MODE,
    RN4678_SET_BLE_LOCAL_ADV_NAME,
//    RN4678_SET_BLE_APPEARANCE,
    CMD_MODE_STOP,
    FINISH
};

// powerup state is reset == low (true); mike conrad of roving networks sez:
// wait about 1s to 2s after reset toggle
#define BT_DELAY_REBOOT_TICKS           48000000UL // 2s @24MHz

#define RNX_TYPE_EEPROM_ADDRESS         (EEPROM_MAX_SIZE_IN_BYTES - CAT24C16_PAGE_SIZE)
#define RNX_RADIO_TYPE_IDX              (0U)
#define RN4678_BAUD_RATE_IDX            (1U)

#define RN4X_AOK_RESPONSE "AOK\r\n"
#define RN4X_CMD_LEN 5U

#define RN42_CMD "CMD\r\n"
#define RN41_VERSION_RESPONSE_V4_77 "Ver 4.77 05/12/09 \r\n(c) Roving Networks\r\n"
#define RN41_VERSION_RESPONSE_LEN_V4_77 41U
#define RN42_VERSION_RESPONSE_V4_77 "Ver 4.77 RN-42 01/05/10 \r\n(c) Roving Networks\r\n"
#define RN42_VERSION_RESPONSE_LEN_V4_77 47U
#define RN42_VERSION_RESPONSE_V6_15 "Ver 6.15 04/26/2013\r\n(c) Roving Networks\r\n"
#define RN42_VERSION_RESPONSE_LEN_V6_15 42U

#define RN4678_CMD "CMD> "
#define RN4678_AOK_CMD_RESPONSE (RN4X_AOK_RESPONSE RN4678_CMD)
#define RN4678_AOK_CMD_RESPONSE_LEN (5U + RN4X_CMD_LEN)

#define RN4678_VERSION_RESPONSE_V1_00_5 ("RN4678 V1.00.5 11/15/2016 (c)Microchip Technology Inc\r\n" RN4678_CMD)
#define RN4678_VERSION_LEN_V1_00_5 (7U + 18U + 30U + RN4X_CMD_LEN)
#define RN4678_VERSION_RESPONSE_V1_11_0 ("RN4678 V1.11.00 6/1/2017 (c)Microchip Technology Inc\r\n" RN4678_CMD)
#define RN4678_VERSION_LEN_V1_11_0 (7U + 17U + 30U + RN4X_CMD_LEN)
#define RN4678_VERSION_RESPONSE_V1_13_5 ("RN4678 V1.13.5 8/29/2018 (c)Microchip Technology Inc\r\n" RN4678_CMD)
#define RN4678_VERSION_LEN_V1_13_5 (7U + 17U + 30U + RN4X_CMD_LEN)
#define RN4678_VERSION_RESPONSE_V1_22_0 ("RN4678 V1.22 12/08/2020 (c)Microchip Technology Inc   \r\n" RN4678_CMD)
#define RN4678_VERSION_LEN_V1_22_0 (7U + 16U + 33U + RN4X_CMD_LEN)
#define RN4678_VERSION_RESPONSE_V1_23_0 ("RN4678 V1.23 06/30/2021 (c)Microchip Technology Inc\r\n" RN4678_CMD)
#define RN4678_VERSION_LEN_V1_23_0 (7U + 16U + 30U + RN4X_CMD_LEN)

#define BT_VER_RESPONSE_SMALLEST RN41_VERSION_RESPONSE_LEN_V4_77
#define BT_VER_RESPONSE_LARGEST RN4678_VERSION_LEN_V1_22_0

#define RN4678_STATUS_STRING_SEPARATOR                '%'

/* +2U for the % prefix and suffix */
#define BT_STAT_STR_LEN_AUTHENTICATED  (13U + 2U)
#define BT_STAT_STR_LEN_AUTH_FAIL      (9U + 2U)
#define BT_STAT_STR_LEN_BONDED         (6U + 2U)
#define BT_STAT_STR_LEN_RN4678_CONNECT (20U + 2U) /* %CONNECT,001BDC06A3D5% for RN4678 */
#define BT_STAT_STR_LEN_RN4678_LCONNECT (23U + 2U) /* %LCONNECT,001BDC06A3D5,1% for RN4678 */
#define BT_STAT_STR_LEN_RN42_v477_CONNECT   (7U + 1U) /* %CONNECT */
#define BT_STAT_STR_LEN_RN42_v615_CONNECT   (22U + 1U) /* %CONNECT,001BDC06A3D5,0 for RN42 v6.15 */
#define BT_STAT_STR_LEN_RN4678_DISCONN (7U + 2U)
#define BT_STAT_STR_LEN_RN42_DISCONNECT (10U + 1U)
#define BT_STAT_STR_LEN_CONN_PARAM     (25U + 2U)
#define BT_STAT_STR_LEN_END_INQ        (7U + 2U)
#define BT_STAT_STR_LEN_END_SCN        (7U + 2U)
#define BT_STAT_STR_LEN_ERR_CONN       (8U + 2U)
#define BT_STAT_STR_LEN_ERR_CONN_PARAM (14U + 2U)
#define BT_STAT_STR_LEN_ERR_LSEC       (8U + 2U)
#define BT_STAT_STR_LEN_ERR_SEC        (7U + 2U)
#define BT_STAT_STR_LEN_FACTORY_RESET  (13U + 2U)
#define BT_STAT_STR_LEN_LBONDED        (7U + 2U)
#define BT_STAT_STR_LEN_LSECURED       (8U + 2U)
#define BT_STAT_STR_LEN_LSECURE_FAIL   (12U + 2U)
#define BT_STAT_STR_LEN_LSTREAM_OPEN   (12U + 2U)
#define BT_STAT_STR_LEN_MLDP_MODE      (9U + 2U)
#define BT_STAT_STR_LEN_RN4678_REBOOT  (6U + 2U) /* %REBOOT% */
#define BT_STAT_STR_LEN_RN42_REBOOT    (6U + 1U) /* %REBOOT */
#define BT_STAT_STR_LEN_RFCOMM_CLOSE   (12U + 2U)
#define BT_STAT_STR_LEN_RFCOMM_OPEN    (11U + 2U)
#define BT_STAT_STR_LEN_SECURE_FAIL    (11U + 2U)
#define BT_STAT_STR_LEN_SESSION_CLOSE  (13U + 2U)
#define BT_STAT_STR_LEN_SESSION_OPEN   (12U + 2U)
#define BT_STAT_STR_LEN_SECURED        (7U + 2U)

/* Smallest is %REBOOT */
#define BT_STAT_STR_LEN_SMALLEST        BT_STAT_STR_LEN_RN42_REBOOT
#define BT_STAT_STR_LEN_LARGEST         BT_STAT_STR_LEN_CONN_PARAM

#define BT_TX_BUF_SIZE                  256U              /* serial buffer in bytes (power 2)  */
#define BT_TX_BUF_MASK                  (BT_TX_BUF_SIZE-1UL)
#if !BT_DMA_USED_FOR_RX
#define BT_RX_BUF_SIZE                  (256UL)              /* serial buffer in bytes (power 2)   */
#define BT_RX_BUF_MASK                  (BT_RX_BUF_SIZE-1UL)  /* buffer size mask                   */
#endif
#define BLE_MTU_SIZE                    157U

#define INSTREAM_STATUS_RESPONSE_LEN    4U

#define RN42_OTHER_SETTINGS_TX_POWER    "TX Power=\0"
#define RN42_OTHER_SETTINGS_ROLE_SWCH    "RoleSwch=\0"


typedef struct{
    uint8_t data[BT_TX_BUF_SIZE];
    // tail points to the buffer index for the oldest byte that was added to it
    uint16_t rdIdx;
    // head points to the index of the next empty byte in the buffer
    uint16_t wrIdx;
} RingFifoTx_t;

#if !BT_DMA_USED_FOR_RX
typedef struct{
    uint8_t data[BT_RX_BUF_SIZE];
    // tail points to the buffer index for the oldest byte that was added to it
    uint16_t rdIdx;
    // head points to the index of the next empty byte in the buffer
    uint16_t wrIdx;
    uint8_t appendingToBuffer;
} RingFifoRx_t;
#endif

typedef enum
{
    RN4678_OP_MODE_NOT_USED,
    RN4678_OP_MODE_WRITE_FLASH,
    RN4678_OP_MODE_WRITE_EEPROM_AND_TEST,
    RN4678_OP_MODE_APPLICATION,
    RN4678_OP_MODE_DISABLE
} rn4678OperationalMode;

typedef enum
{
    RN4678_DISCONNECTED,
    RN4678_CONNECTED_CLASSIC,
    RN4678_CONNECTED_BLE
} rn4678ConnectionType;

// set a callback function cb that runs when Bt is successfully started
void BT_startDone_cb(void (*cb)(void));

// set a callback function cb that runs when baud rate is successfully changed
void BT_baudRateChange_cb(void (*cb)(void));

// set a callback function cb that runs before bytes are transmitted to the BT module from the TX Buf
void BT_setSendNextChar_cb(void (*cb)(void));

void initRN1(void);
void initRN2(void);
void initRN3(void);
void setupUART(uint8_t baudRate);
void disableUART(void);
void disableRN(void);
void writeCommandNoRsp(char *cmd);
void writeCommandNoRspWithCmdLen(char *cmd, uint8_t cmdLen);
void writeCommand(char *cmd, char *response);
void writeCommandWithCmdLen(char *cmd, uint8_t cmdLen, char *response);
void runSetCommands(void);
void runMasterCommands(void);
void runSetBaudRate(void);
void sendBaudRateUpdateToBtModule(void);
uint8_t handlePostBaudRateChange(void);

void writeCommandBufAndExpectAok(void);
void writeCommandBufAndExpectAokWithCmdLen(uint8_t cmdBufLen);
void btCmdModeStartAfterRn4xTempBaudChange(void);
void btCmdModeStart(void);
void btCmdModeStop(void);

//set new baud rate. This change is effective immediately.
//For the RN42, this change is only temporary. Reverts to previously configured rate after reset.
void BT_changeBaudRateInBtModule(uint8_t baudRate);

void setBtBaudRateToUse(uint8_t baudRate);

void BT_resetBaudRate(void);

// when received command is the same as the expected response, run this
void BT_setGoodCommand(void);
uint8_t areBtSetupCommandsRunning(void);

void sendNextCharIfNotInProgress(void);

uint8_t isBtModuleOverflowPinHigh(void);
void sendNextChar(void);

// initialize the vars and power on
void BT_init(void);

// set up the BT/UART pins
void BT_start(void);
void start2(void);
void start3(void);

// disable the BT/UART pins. if the start is in progress, stop it
void BT_disable(void);

//write data to be transmitted to the Bluetooth module
//returns 0 if fails, else 1
//will fail if the function is already being called or the buffer doesn't have the capacity to store buf argument
extern void (*BT_write)(uint8_t *buf, uint8_t len, btResponseType responseType);
void BT_write_rn42(uint8_t *buf, uint8_t len, btResponseType responseType);
void BT_write_rn4678_460800(uint8_t *buf, uint8_t len, btResponseType responseType);
void BT_write_rn4678_ble(uint8_t *buf, uint8_t len, btResponseType responseType);
void BT_write_rn4678_with_buf(uint8_t *buf, uint8_t len, btResponseType responseType, uint8_t sampleSetBufferSize);
void BT_write_rn4678_1M(uint8_t *buf, uint8_t len, btResponseType responseType);

//connect to a specific device that was previously discovered
void BT_connect(uint8_t *addr);
//after this command is called there will be no link to the connected device
void BT_disconnect(void);

//enum SLAVE_MODE, MASTER_MODE, TRIGGER_MASTER_MODE, AUTO_MASTER_MODE
void BT_setGetMacAddress(uint8_t val);
void BT_setGetVersion(uint8_t val);
void BT_setWaitForInitialBoot(uint8_t val);
#if BT_DMA_USED_FOR_RX
void BT_setWaitForReturnNewLine(uint8_t val);
#endif

void BT_setRadioMode(btOperatingMode mode);
void BT_setAutoMaster(char *master);
void BT_setDiscoverable(uint8_t disc);
void BT_setEncryption(uint8_t enc);
void BT_setAuthentication(uint8_t auth);
void BT_setAdvertisingName(char *name); // max 16 chars
void BT_useSpecificAdvertisingName(uint8_t val);
void BT_setPIN(char *name); // max 16 chars
void BT_setServiceClass(char *serviceClass); // max 4 chars (hex word)
void BT_setServiceName(char *name); // max 16 chars
void BT_setDeviceClass(char *deviceClass); // max 4 chars (hex word)
void BT_rn4xDisableRemoteConfig(uint8_t disableRemoteConfig);
void BT_setUpdateBaudDuringBoot(uint8_t val);

//save power by minimising time Inquiry/Page scanning
//module reset necessary for changes to take effect
void BT_setPagingTime(char *hexvalTime); // max 4 chars (hex word)
void BT_setInquiryScanWindow(char *hexvalTime); // max 4 chars (hex word)
void BT_setRn4678FastMode(char *hexval_time);
void BT_setRn4678BleConnectionParameters(char *hexval_time);
void BT_setRn4678BleCompleteLocalName(char *hexval_name);
char * BT_getDesiredRnTxPowerForBtVerSetCmd(void);
char * BT_getDesiredRnTxPowerForBtVerGetCmd(void);
void BT_setRn4678TxPower(rn4678TxPower_et newValue);
void BT_setRn42TxPowerPreAug2012(rn42TxPowerPreAug2012_et newValue);
void BT_setRn42TxPowerPostAug2012(rn42TxPowerPostAug2012_et newValue);
void BT_setRn4678BtMode(char *hexval_time);
void BT_setRn4xRemoteConfigurationTimer(char *hexval_time);
void BT_resetDefaults(void);

//pass in a pointer to the function that will get called when
//data arrives
//the pass in function returns a 1 if program execution should resume
//(i.e. clear LPM3 bits)
//otherwise return 0
void BT_receiveFunction(uint8_t (*receiveFuncPtr)(uint8_t data));

//this function needs to be called from within the BT_RTS ISR
//in order to inform the RN42 driver about the state change
//value needs to be 1 if interrupt was low to high else 0
void BT_rtsInterrupt(uint8_t value);

void BT_rst_MessageProgress(void); // reset messageInProgress to 0 as in the initialisation @weibo

// pass the expected response to the main function for user customized use (e.g.DMA).
uint8_t* BT_getExpResp(void);
uint8_t BT_getWaitForInitialBoot(void);
#if BT_DMA_USED_FOR_RX
void BT_setWaitForStartCmd(uint8_t val);
uint8_t BT_getWaitForStartCmd(void);
void BT_setWaitForMacAddress(uint8_t val);
uint8_t BT_getWaitForMacAddress(void);
void BT_setWaitForVersion(uint8_t val);
uint8_t BT_getWaitForVersion(void);
uint8_t BT_getWaitForReturnNewLine(void);
#endif

void clearBtTxBuf(uint8_t isCalledFromMain);
void pushByteToBtTxBuf(uint8_t b);
void pushBytesToBtTxBuf(uint8_t *buf, uint8_t len);
uint16_t getUsedSpaceInBtTxBuf(void);
uint16_t getSpaceInBtTxBuf(void);
#if BT_DMA_USED_FOR_RX
void setDmaWaitingForResponseIfStatusStrEnabled(void);
void setDmaWaitingForResponseIfStatusStrDisabled(void);
void setDmaWaitingForResponse(uint16_t numChars);
void setDmaWaitForReturnNewLine(void);
void DMA2AndCtsDisable(void);
#endif
void setIsBtClearToSend(uint8_t isBtClearToSend);
void setBtRtsInterruptState(uint8_t isEnabled);
void setBtConnectionStatusInterruptIsEnabled(uint8_t isEnabled);
void updateBtConnectionStatusInterruptDirection(void);
void setBtModuleReset(uint8_t isResetHeld);
void setBtModulePower(uint8_t isEnabled);
uint8_t isBtDeviceUnknown(void);
uint8_t isBtDeviceRn41orRN42(void);
uint8_t isBtDeviceRn41(void);
uint8_t isBtDeviceRn42(void);
uint8_t isBtDeviceRn4678(void);
uint8_t doesBtDeviceSupport1Mbps(void);
void setBtFwVersion(enum BT_FIRMWARE_VERSION btFwVerNew);
enum BT_FIRMWARE_VERSION getBtFwVersion(void);
enum BT_HARDWARE_VERSION getBtHwVersion(void);
void updateBtWriteFunctionPtr(void);
uint8_t getCurrentBtBaudRate(void);
void setBtRxFullResponsePtr(char *ptr);
uint8_t getBtClearTxBufFlag(void);
void setBtClearTxBufFlag(uint8_t val);
uint8_t areBtStatusStringsEnabled(void);
void setCommandModeActive(uint8_t state);
uint8_t isRnCommandModeActive(void);
char* getTxCmdBufPtr(void);
void clearBtCmdBuf(void);
void clearExpectedResponseBuf(void);
void setRn4678OperationalMode(rn4678OperationalMode mode);
void setRn4678OperationalModePins(rn4678OperationalMode mode);
uint8_t isRn4678ConnectionEstablished(void);
uint8_t isRn4678ConnectionBle(void);
void setRn4678ConnectionState(rn4678ConnectionType state);
void calculateClassicBtTxSampleSetBufferSize(uint8_t len, uint16_t samplingRateTicks);
uint8_t getDefaultBaudForBtVersion(void);
void setBleDeviceInformation(char *daughtCardIdStrPtr, uint8_t fwVerMajorNew, uint8_t fwVerMinorNew, uint8_t fwVerRelNew);
uint8_t parseRnGetResponse(char *cmdPtr, char *responsePtr);
void setRebootRequired(uint8_t state);
void checkAuth(char *rxBufPtr);
void checkStatusStrPrefixAndSuffix(char *rxBufPtr);
void checkOperatingMode(char *rxBufPtr);
void checkPagingTime(char *rxBufPtr);
void checkBtmode(char *rxBufPtr);
void checkFastMode(char *rxBufPtr);
void checkTransmitPower(char *rxBufPtr);
void checkBleConnectionParameters(char *rxBufPtr);
void checkInquiryScanWindow(char *rxBufPtr);
void checkManufacturerString(char *rxBufPtr);
void checkModelString(char *rxBufPtr);
void checkSoftwareRevision(char *rxBufPtr);
void checkRn4xRemoteConfigTimer(char *rxBufPtr);
void checkAdvertisingName(char *rxBufPtr);
void checkPin(char *rxBufPtr);
void string2hexString(char* input, char* output);
void bt_setMacId(uint8_t *buf);
uint8_t* getMacIdStrPtr(void);
uint8_t* getMacIdBytesPtr(void);
#if !BT_DMA_USED_FOR_RX
RingFifoRx_t *getRxFifoPtr(void);
void readByteFromBtRxBuf(uint8_t *buf);
uint16_t getNumBytesInBtRxBuf(void);
void pushByteToBtRxBufIfNotFull(uint8_t c);
void clearBtRxBuf(void);
uint8_t isBtStarting(void);
#endif

#endif //RN4X_H
