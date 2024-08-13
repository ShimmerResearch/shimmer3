/*
 * shimmer_dock_comms.h
 *
 *  Created on: 13 Aug 2024
 *      Author: MarkNolan
 */

#ifndef SHIMMER3_COMMON_SOURCE_DOCK_SHIMMER_DOCK_COMMS_H_
#define SHIMMER3_COMMON_SOURCE_DOCK_SHIMMER_DOCK_COMMS_H_

#include <stdint.h>

// UART COMMANDS
//================= WP uart 3.0: command names ==============
#define UART_STEP_WAIT4_CMD   4
#define UART_STEP_WAIT4_LEN   3
#define UART_STEP_WAIT4_DATA  2
#define UART_STEP_WAIT4_CRC   1
#define UART_STEP_WAIT4_NONE  0

#define UART_RXBUF_START      0
#define UART_RXBUF_CMD        1
#define UART_RXBUF_LEN        2
#define UART_RXBUF_COMP       3
#define UART_RXBUF_PROP       4  // data in rxbuf starts from byte 3
#define UART_RXBUF_DATA       5  // data in rxbuf starts from byte 3
#define UART_DATA_LEN_MAX     138// max case: '$' + get + length + comp_shimmer+ prop_infomem
#define UART_RSP_PACKET_SIZE  138// + info_len + info_loc*2 + 128bytes data + crc*2 = 138


#define UART_SET                    0x01
#define UART_RESPONSE               0x02
#define UART_GET                    0x03
#define UART_BAD_CMD_RESPONSE       0xfc //252
#define UART_BAD_ARG_RESPONSE       0xfd //253
#define UART_BAD_CRC_RESPONSE       0xfe //254
#define UART_ACK_RESPONSE           0xff //255
//================= WP uart 3.0: components names ==============
#define UART_COMP_SHIMMER           0x01
#define UART_COMP_BAT               0x02 // this is seen as a sensor
#define UART_COMP_DAUGHTER_CARD     0x03
#define UART_COMP_D_ACCEL           0x04
#define UART_COMP_GSR               0x05
#define UART_COMP_BT                0x0A
#define UART_COMP_TEST              0x0B
//================= WP uart 3.0: property names ==============
// component == UART_COMP_SHIMMER:
#define UART_PROP_ENABLE            0x00 // this is for all sensors
#define UART_PROP_SAMPLE_RATE       0x01
#define UART_PROP_MAC               0x02
#define UART_PROP_VER               0x03
#define UART_PROP_RWC_CFG_TIME      0x04
#define UART_PROP_CURR_LOCAL_TIME   0x05
#define UART_PROP_INFOMEM           0x06
// component == UART_COMP_BAT:
//#define UART_PROP_SAMPLE_RATE       0x01
#define UART_PROP_VALUE             0x02
//#define UART_PROP_DIVIDER           0x05
// component == UART_COMP_DAUGHTER_CARD:
#define UART_PROP_CARD_ID           0x02
#define UART_PROP_CARD_MEM          0x03
// component == UART_COMP_D_ACCEL:
//#define UART_PROP_ENABLE            0x00
//#define UART_PROP_SAMPLE_RATE       0x01
#define UART_PROP_DATA_RATE         0x02
#define UART_PROP_RANGE             0x03
#define UART_PROP_LP_MODE           0x04
#define UART_PROP_HR_MODE           0x05
#define UART_PROP_FREQ_DIVIDER      0x06
#define UART_PROP_CALIBRATION       0x07
// component == UART_COMP_GSR:
//#define UART_PROP_ENABLE            0x00
//#define UART_PROP_SAMPLE_RATE       0x01
//#define UART_PROP_RANGE             0x03
//#define UART_PROP_DIVIDER           0x05
#define UART_PROP_TEST_ALL          0x00
#define UART_PROP_TEST_LED_START    0x01
#define UART_PROP_TEST_STOP         0x02

//== new uart ends ==
#define CBUF_SIZE             27
#define CBUF_PARAM_LEN_MAX    (CBUF_SIZE-6)
// UART OLD COMMANDS
#define UART_CMD_MAC 1
#define UART_CMD_VER 2
#define UART_CMD_BAT 3
#define UART_CMD_MEM 4
#define UART_CMD_RTC 5
#define UART_CMD_RCT 6
#define UART_CMD_RDT 7
#define UART_CMD_TIM 8

void dock_uart_init(void);
void dock_uart_reset_variables(void);
uint8_t UartCallback(uint8_t data);
void UartProcessCmd(void);
void UartSendRsp(void);
uint8_t UartCheckCrc(uint8_t len);

#endif /* SHIMMER3_COMMON_SOURCE_DOCK_SHIMMER_DOCK_COMMS_H_ */
