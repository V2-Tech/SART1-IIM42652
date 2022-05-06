/*
 *		Created on: 27/07/2021
 *      Author: Valerio Mazzoni
 *      Company: OMD
 *
 *      Version: 1.0.0
 *
 *      Changelog:
 *      	- 27/07/2021 - 1.0.0: start of the source code writing.
 */
#ifndef _DAC7571_H_
#define _DAC7571_H_

//--------------------------------------
// ------------ LIBRARYS ---------------
//--------------------------------------
#include "stm32f4xx_hal.h"
#include "math.h"

//--------------------------------------
// ------------ PROTOCOLS --------------
//--------------------------------------

/* 	DAC5571 I2C FullSpeed protocol
 *
 * 	Address byte
 *
 * 	Byte 1
 * 	MSB		6		5		4		3		2		1		LSB
 *	1		0		0		1		1		0		0(A0)	0(R=0/W=1 Not supported)
 *
 * 	Control bytes
 *
 *	Byte 1 (Ctrl-MSB)
 *	MSB		6		5		4		3		2		1		LSB
 *	0		0		PD1		PD0		D11		D10		D9		D8
 *
 *	Byte 2 (Ctrl-LSB)
 *	MSB		6		5		4		3		2		1		LSB
 *	D7		D6		D5		D4		D3		D2		D1		D0
 *
 *	where:
 *
 *	PD0		PD1		Modes of Operation
 *	0		0		Normal Operation (Default usage)
 *	0		1		1kΩ to AGND, PWD
 *	1		0		100kΩ to AGND, PWD
 *	1		1		High Impedance, PWD
 *
 *	D11-D0 Desired analog output value with 4096 resolution
 *
 *	Vout = Vdd * D/4096
*/

//--------------------------------------
//---------- REGISTERS VALUES ----------
//--------------------------------------
#define DAC_ALIM_VOLTAGE 3.27			//Device supply voltage value, measured directly on PCB

#define DAC7571_BUS_ADDRESS 0x98 		//7bit-Address + 1bit-R/W

#define REG_CONFIG_OP_NORMAL 0x0000		//Normal Operation
#define REG_CONFIG_OP_1KGND = 0x1000;	//1K to AGND PWD
#define REG_CONFIG_OP_100KGND = 0x2000;	//100K to AGND, PWD
#define REG_CONFIG_OP_HIIMP = 0x3000;	//High Impedance, PWD

//--------------------------------------
// -------- CLASS VARIABLES ------------
//--------------------------------------
typedef struct {
	/* I2C */
	I2C_HandleTypeDef *m_i2cHandle;

	/* DMA */
	uint8_t m_writingAnalogVal;
	uint8_t TxDataBuffer[2];

	/* Variables */
	uint8_t m_EnableDAC;
	uint8_t m_InitDone;
	float m_Vdd;

} DAC7571_t;

//--------------------------------------
// ---------- CLASS METHODS ------------
//--------------------------------------
void DAC7571_Init(DAC7571_t *dac, I2C_HandleTypeDef *i2cHandle, float Vdd);
uint8_t DAC7571_SetOutVoltage(DAC7571_t *dac, float Vout);
void DAC7571_SetOutEnabling(DAC7571_t *dac, uint8_t EnableState);

/*
 * Polling
 */
uint8_t DAC7571_Write(DAC7571_t *dac);

/*
 * DMA
 */
uint8_t DAC7571_WriteDMA(DAC7571_t *dac);
void DAC7571_WriteDMA_Complete(DAC7571_t *dac);


//--------------------------------------
// -------- SIMULATION DATA ------------
//--------------------------------------
/*
 * Timer values for sin wave simulation
 * Output SineWave Frequency = Trigger frequency / NS
 * TriggerFrequency = 96MHz / (PSC+1)(ARR+1)
 *
 * For 10kHz sin wave:
 * TriggerFrequency = 96MHz * 4096 = 40,96MHz
 * ARR = (96MHz/40,96MHz) - 1 = 1
 *
 */
#define SinNS 128

void DAC7571_TestOutput(DAC7571_t *dac);

#endif
