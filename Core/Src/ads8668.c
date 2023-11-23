/*
 * ads8668.c
 *
 *  Created on: Nov 23, 2023
 *      Author: asus
 */

#include "ads8668.h"


uint8_t SPI_Tx[4] = {0};
uint8_t SPI_Rx[16] = {0};
uint8_t ADS8668_mode = MODE_IDLE;
uint8_t ADS8668_feature = 0;
float ADS8668_vref = 4.096; //internal Vref

extern SPI_HandleTypeDef hspi1;

/////////////////////////
//  PRIVATE FUNCTIONS  //
/////////////////////////
static void ADS8668_WriteRegister(uint8_t reg, uint8_t val)
{
	SPI_Tx[0] = (reg<<1) | 0x01;
	SPI_Tx[1] = val;
	SPI_Tx[2] = 0x00;
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, SPI_Tx, SPI_Rx, 3,1000);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

    ADS8668_mode = MODE_PROG;
}

static uint8_t ADS8668_ReadRegister(uint8_t reg) {
	SPI_Tx[0] = (reg<<1) & 0xFE;
	SPI_Tx[1] = 0x00;
	SPI_Tx[2] = 0x00;
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, SPI_Tx, SPI_Rx, 3,1000);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

    ADS8668_mode = MODE_PROG;

    return SPI_Rx[2];
}

static uint16_t ADS8668_CmdReg(uint8_t reg)
{
	uint16_t result = 0;
	HAL_StatusTypeDef _status;

	SPI_Tx[0] = reg;
	SPI_Tx[1] = 0x00;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	_status = HAL_SPI_TransmitReceive(&hspi1, SPI_Tx, SPI_Rx, 2,1000);
	SPI_Tx[0] = 0x00;
	if (ADS8668_mode > 4) {
		_status = HAL_SPI_TransmitReceive(&hspi1, SPI_Tx, SPI_Rx, 2,1000);
		result = (SPI_Rx[0]<<8) | SPI_Rx[1];
	}
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

	if (ADS8668_mode == MODE_POWER_DN) HAL_Delay(100);

	switch (reg) {
		case NO_OP:
			switch (ADS8668_mode) {
				case MODE_RESET:
					ADS8668_mode = MODE_IDLE;
					break;
				case MODE_PROG :
					ADS8668_mode = MODE_IDLE;
					break;
				case MODE_AUTO_RST:
					ADS8668_mode = MODE_AUTO;
					break;
			}
			break;
		case STDBY:
			ADS8668_mode = MODE_STANDBY;
			break;
		case PWR_DN:
			ADS8668_mode = MODE_POWER_DN;
			break;
		case RST:
			ADS8668_mode = MODE_RESET;
			break;
		case AUTO_RST:
			ADS8668_mode = MODE_AUTO_RST;
			break;
		default:
			ADS8668_mode = MODE_MANUAL;
			break;
	}
	if(_status == HAL_OK )
		return result;
	else
		return 0xFFFF;

}

/////////////////////////
//  GLOBAL FUNCTIONS    //
/////////////////////////
uint16_t ADS8668_NoOp(void)
{
	return ADS8668_CmdReg(NO_OP);
}


void ADS8668_StandBy(void)
{
	ADS8668_CmdReg(STDBY);
}

void ADS8668_PowerDown(void)
{
	ADS8668_CmdReg(PWR_DN);
}

void ADS8668_Reset(void)
{
	ADS8668_CmdReg(RST);
}

void ADS8668_AutoRst(void)
{
	ADS8668_CmdReg(AUTO_RST);
}

uint16_t ADS8668_ManualChannel(uint8_t ch)
{
	uint8_t reg;
	switch (ch) {
		case 0:  reg = MAN_Ch_0;break;
		case 1:  reg = MAN_Ch_1;break;
		case 2:  reg = MAN_Ch_2;break;
		case 3:  reg = MAN_Ch_3;break;
		case 4:  reg = MAN_Ch_4;break;
		case 5:  reg = MAN_Ch_5;break;
		case 6:  reg = MAN_Ch_6;break;
		case 7:  reg = MAN_Ch_7;break;
		case 8:  reg = MAN_AUX; break;
		default: reg = MAN_Ch_0;break;
	}
	return ADS8668_CmdReg(reg);
}

////////////////////////////////////
//  PROGRAM REGISTER FUNCTIONS    //
////////////////////////////////////

uint8_t ADS8668_GetFeatureSelect() {
    return ADS8668_ReadRegister(FT_SEL);
}

void ADS8668_SetFeatureSelect(uint8_t id, uint8_t alarm, uint8_t sdo) {
	if(alarm == ALARM_TRUE || alarm == ALARM_FALSE){
		ADS8668_feature = ((id & 0b11)<<6) | (alarm<<4) | (sdo & 0b111);
		ADS8668_WriteRegister(FT_SEL,ADS8668_feature);
	}
}

void ADS8668_setChannelSequence(uint8_t flag) {
	ADS8668_WriteRegister(AUTO_SEQ_EN,flag);
}

void ADS8668_setChannelPowerDown(uint8_t flag) {
	ADS8668_WriteRegister(CH_PWR_DN,flag);
}

uint8_t ADS8668_getChannelSequence() {
	return ADS8668_ReadRegister(AUTO_SEQ_EN);
}

uint8_t ADS8668_getChannelPowerDown() {
	return ADS8668_ReadRegister(CH_PWR_DN);
}

void ADS8668_SetChanSeqPowDown(uint8_t flag) {
	ADS8668_setChannelSequence(flag);
	ADS8668_setChannelPowerDown((uint8_t)~flag);
}

uint8_t ADS8668_GetChannelRange(uint8_t ch) {
    uint8_t reg;
    switch (ch) {
        case 0:  reg = RG_Ch_0;break;
        case 1:  reg = RG_Ch_1;break;
        case 2:  reg = RG_Ch_2;break;
        case 3:  reg = RG_Ch_3;break;
        case 4:  reg = RG_Ch_4;break;
        case 5:  reg = RG_Ch_5;break;
        case 6:  reg = RG_Ch_6;break;
        case 7:  reg = RG_Ch_7;break;
        default: reg = RG_Ch_0;break;
        }
    return ADS8668_ReadRegister(reg);
}

void ADS8668_SetChannelRange(uint8_t ch, uint8_t range) {
    uint8_t reg;
    switch (ch) {
        case 0:  reg = RG_Ch_0;break;
        case 1:  reg = RG_Ch_1;break;
        case 2:  reg = RG_Ch_2;break;
        case 3:  reg = RG_Ch_3;break;
        case 4:  reg = RG_Ch_4;break;
        case 5:  reg = RG_Ch_5;break;
        case 6:  reg = RG_Ch_6;break;
        case 7:  reg = RG_Ch_7;break;
        default: reg = RG_Ch_0;break;
        }
    ADS8668_WriteRegister(reg,range);
}

void ADS8668_setGlobalRange(uint8_t range) {
    for (uint8_t i=0;i<8;i++){
    	ADS8668_SetChannelRange(i,range);
    }
}


uint8_t ADS8668_GetId() {
    return (ADS8668_GetFeatureSelect() >> 6);
}

void ADS8668_SetId(uint8_t id) {
	ADS8668_feature = (ADS8668_feature & 0b00010111) | ((id & 0b11)<<6);
    ADS8668_WriteRegister(FT_SEL,ADS8668_feature);
}

uint8_t ADS8668_GetAlarm() {
    return (ADS8668_GetFeatureSelect() >> 4) & 1;
}

void ADS8668_SetAlarm(uint8_t alarm)
{
	if(alarm == ALARM_TRUE || alarm == ALARM_FALSE){
		ADS8668_feature = (ADS8668_feature & 0b11000111) | (alarm<<4);
		ADS8668_WriteRegister(FT_SEL,ADS8668_feature);
	}
}

uint8_t ADS8668_GetCommandReadBack() {
    return ADS8668_ReadRegister(CMD_READBACK);
}

uint8_t ADS8668_GetChannelHysteresis(uint8_t ch) {
    uint8_t reg = 5*(ch>7?7:ch) + CH0_HYST;
    return ADS8668_ReadRegister(reg);
}

uint16_t ADS8668_GetChannelLowThreshold(uint8_t ch) {
    uint8_t reg = 5*(ch>7?7:ch) + CH0_LT_MSB;
    uint8_t MSB = ADS8668_ReadRegister(reg);
    uint8_t LSB = ADS8668_ReadRegister(reg+1);
    return (MSB << 8) | LSB;
}

uint16_t ADS8668_GetChannelHighThreshold(uint8_t ch) {
    uint8_t reg = 5*(ch>7?7:ch) + CH0_HT_MSB;
    uint8_t MSB = ADS8668_ReadRegister(reg);
    uint8_t LSB = ADS8668_ReadRegister(reg+1);
    return (MSB << 8) | LSB;
}

void ADS8668_SetChannelHysteresis(uint8_t ch, uint8_t val) {
    uint8_t reg = 5*(ch>7?7:ch) + CH0_HYST;
    ADS8668_WriteRegister(reg,val);
}

void ADS8668_SetChannelLowThreshold(uint8_t ch, uint16_t val) {
    uint8_t reg = 5*(ch>7?7:ch) + CH0_LT_MSB;
    ADS8668_WriteRegister(reg,val>>8);
    ADS8668_WriteRegister(reg+1,val&255);
}

void ADS8668_SetChannelHighThreshold(uint8_t ch, uint16_t val)  {
    uint8_t reg = 5*(ch>7?7:ch) + CH0_HT_MSB;
    ADS8668_WriteRegister(reg,val>>8);
    ADS8668_WriteRegister(reg+1,val&255);
}

float ADS8668_I2V(uint16_t x, uint8_t range) {
    float out_min, out_max;
    uint16_t _tmpX=0;

    _tmpX = (x>>4);
    switch (range) {
        case R1:
            out_min = -1.25 * ADS8668_vref;
            out_max = 1.25 * ADS8668_vref;
            break;
        case R2:
            out_min = -0.625 * ADS8668_vref;
            out_max = 0.625 * ADS8668_vref;
            break;
        case R3:
            out_min = -0.3125 * ADS8668_vref;
            out_max = 0.3125 * ADS8668_vref;
            break;
        case R4:
            out_min = -0.15625 * ADS8668_vref;
            out_max = 0.15625 * ADS8668_vref;
            break;
        case R5:
            out_min = 0 * ADS8668_vref;
            out_max = 2.5 * ADS8668_vref;
            break;
        case R6:
            out_min = 0 * ADS8668_vref;
            out_max = 1.25 * ADS8668_vref;
            break;
        case R7:
            out_min = 0 * ADS8668_vref;
            out_max = 0.625 * ADS8668_vref;
            break;
        case R8:
            out_min = 0 * ADS8668_vref;
            out_max = 0.3125 * ADS8668_vref;
            break;
        default:
            out_min = -2.5 * ADS8668_vref;
            out_max = 2.5 * ADS8668_vref;
            break;

    }

    return (float)_tmpX * (out_max - out_min) / 4095. + out_min;
}

