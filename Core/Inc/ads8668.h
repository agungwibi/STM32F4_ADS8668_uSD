/*
 * ads8668.h
 *
 *  Created on: Nov 23, 2023
 *      Author: asus
 */

#ifndef INC_ADS8668_H_
#define INC_ADS8668_H_

#include "main.h"

/*** ADC address ****/
    #define NO_OP     0x00  // Continue operation in previous mode
    #define STDBY     0x82  // Device is placed into standby mode
    #define PWR_DN    0x83  // Device is powered down
    #define RST       0x85  // Program register is reset to default
    #define AUTO_RST  0xA0  // Auto mode enabled following a reset
    #define MAN_Ch_0  0xC0  // Channel 0 input is selected
    #define MAN_Ch_1  0xC4  // Channel 1 input is selected
    #define MAN_Ch_2  0xC8  // Channel 2 input is selected
    #define MAN_Ch_3  0xCC  // Channel 3 input is selected
    #define MAN_Ch_4  0xD0  // Channel 4 input is selected
    #define MAN_Ch_5  0xD4  // Channel 5 input is selected
    #define MAN_Ch_6  0xD8  // Channel 6 input is selected
    #define MAN_Ch_7  0xDC  // Channel 7 input is selected
    #define MAN_AUX   0xE0  // AUX channel input is selected

    // PROGRAM REGISTER MAP -------------------------------------------------------------------------------------------

    // AUTO SCAN SEQUENCING CONTROL
    #define AUTO_SEQ_EN   0x01  // Auto Squencing Enable: default 0xFF - bitX to enable chX
    #define CH_PWR_DN     0x02  // Channel Power Down: default 0x00 - bitX to power down chX

    // DEVICE FEATURES SELECTION CONTROL
    #define FT_SEL        0x03  // Feature Select: default 0x00
                                // bit 7-6 for daisy chain ID, bit 4 for ALARM feature, bit 2-0 SDO data format bits

    // RANGE SELECT REGISTERS
    #define RG_Ch_0       0x05   // Channel 0 Input Range: default 0x00 - bit 3-0 to select range
    #define RG_Ch_1       0x06   // Channel 1 Input Range: default 0x00 - bit 3-0 to select range
    #define RG_Ch_2       0x07   // Channel 2 Input Range: default 0x00 - bit 3-0 to select range
    #define RG_Ch_3       0x08   // Channel 3 Input Range: default 0x00 - bit 3-0 to select range
    #define RG_Ch_4       0x09   // Channel 4 Input Range: default 0x00 - bit 3-0 to select range
    #define RG_Ch_5       0x0A   // Channel 5 Input Range: default 0x00 - bit 3-0 to select range
    #define RG_Ch_6       0x0B   // Channel 6 Input Range: default 0x00 - bit 3-0 to select range
    #define RG_Ch_7       0x0C   // Channel 7 Input Range: default 0x00 - bit 3-0 to select range

    // ALARM FLAG REGISTERS (Read-only)
    #define ALARM_OVERVIEW          0x10 // ALARM Overview Tripped Flag
    #define ALARM_CH0_TRIPPED_FLAG  0x11 // ALARM Ch 0-3 Tripped-Flag
    #define ALARM_CH0_ACTIVE_FLAG   0x12 // ALARM Ch 0-3 Active-Flag
    #define ALARM_CH4_TRIPPED_FLAG  0x13 // ALARM Ch 4-7 Tripped-Flag
    #define ALARM_CH4_ACTIVE_FLAG   0x14 // ALARM Ch 4-7 Active-Flag

    // ALARM THRESHOLD REGISTERS
    #define CH0_HYST      0x15   // Ch 0 Hysteresis
    #define CH0_HT_MSB    0x16   // Ch 0 High Threshold MSB
    #define CH0_HT_LSB    0x17   // Ch 0 High Threshold LSB
    #define CH0_LT_MSB    0x18   // Ch 0 Low Threshold MSB
    #define CH0_LT_LSB    0x19   // Ch 0 Low Threshold LSB
    //... CHx register address are Ch0 + 5x

    // COMMAND READ BACK (Read-Only)
    #define CMD_READBACK  0x3F   // Command Read Back

 // SPECIFIC VALUES -------------------------------------------------------------------------------------------

    //RANGE SELECTION
    #define R0            0x00   // Input range to -2.5/+2.5         Vref   +/- 10.24V
    #define R1            0x01   // Input range to -1.25/+1.25       Vref   +/-  5.12V
    #define R2            0x02   // Input range to -0.625/+0.625     Vref   +/-  2.56V
    #define R3            0x03   // Input range to -0.3125/+0.3125   Vref   +/-  1.28V
    #define R4            0x0B   // Input range to -0.15625/+0.15625 Vref   +/-  0.64V
    #define R5            0x05   // Input range to +2.5    Vref   10.24V
    #define R6            0x06   // Input range to +1.25   Vref    5.12V
    #define R7            0x07   // Input range to +0.625  Vref    2.56V
    #define R8            0x0F   // Input range to +0.3125 Vref    1.28V

    // OPERATION MODES
    #define MODE_IDLE       0
    #define MODE_RESET      1
    #define MODE_STANDBY    2
    #define MODE_POWER_DN   3
    #define MODE_PROG       4
    #define MODE_MANUAL     5
    #define MODE_AUTO       6
    #define MODE_AUTO_RST   7

	#define ALARM_TRUE 	0x00
	#define ALARM_FALSE 0x01

void ADS8668_Reset(void);
void ADS8668_AutoRst(void);
uint8_t ADS8668_getChannelSequence() ;
uint8_t ADS8668_getChannelPowerDown();
void ADS8668_SetChanSeqPowDown(uint8_t flag);
void ADS8668_setGlobalRange(uint8_t range);
uint16_t ADS8668_NoOp(void);
float ADS8668_I2V(uint16_t x, uint8_t range) ;


#endif /* INC_ADS8668_H_ */
