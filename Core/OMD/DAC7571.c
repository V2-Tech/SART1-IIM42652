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

#include "DAC7571.h"

DAC7571_t DAC;

const uint16_t SinWave[SinNS] = {
		2048,2148,2248,2348,2447,2545,2642,2737,
		2831,2923,3013,3100,3185,3267,3346,3423,
		3495,3565,3630,3692,3750,3804,3853,3898,
		3939,3975,4007,4034,4056,4073,4085,4093,
		4095,4093,4085,4073,4056,4034,4007,3975,
		3939,3898,3853,3804,3750,3692,3630,3565,
		3495,3423,3346,3267,3185,3100,3013,2923,
		2831,2737,2642,2545,2447,2348,2248,2148,
		2048,1947,1847,1747,1648,1550,1453,1358,
		1264,1172,1082,995,910,828,749,672,
		600,530,465,403,345,291,242,197,
		156,120,88,61,39,22,10,2,
		0,2,10,22,39,61,88,120,
		156,197,242,291,345,403,465,530,
		600,672,749,828,910,995,1082,1172,
		1264,1358,1453,1550,1648,1747,1847,1947};

/*
 * Initialization of the device
 * @param	dac Pointer to a DAC7571_t structure that contains
 *			the configuration information for the specified DAC7571 device.
 * @param  	i2cHandle Pointer to a I2C_HandleTypeDef structure that contains
 * 			the configuration information for the specified I2C.
 * @param	Vdd DAC7571 Supply voltage
 * @param	timerHandle Pointer to timer handle used in test function
 */
void DAC7571_Init(DAC7571_t *dac, I2C_HandleTypeDef *i2cHandle, float Vdd)
{
	dac->m_i2cHandle = i2cHandle;
	dac->m_Vdd = Vdd;

	dac->m_InitDone = 1;
}

/*
 * Set desired output voltage value.
 * @param	dac Pointer to a DAC7571_t structure that contains
 *			the configuration information for the specified DAC7571 device.
 * @param	Vdd DAC7571 Supply voltage
 * @param	Vout Desired output voltage
 * @return	Data stored in struct buffer (1), Data out of range (0).
 */
uint8_t DAC7571_SetOutVoltage(DAC7571_t *dac, float Vout)
{
	uint16_t data_i;
	/* Check if desired output voltage is in range, if not return an error */
	if (Vout > dac->m_Vdd || Vout < 0)
	{
		return 1;
	}
	/* Convert Voltage(float) to the equivalent 12bit-scaled value */
	data_i = floor((Vout/dac->m_Vdd)*4095.0);

	/* Fill the last 4 MSB with desired DAC configuration */
	data_i = (REG_CONFIG_OP_NORMAL | data_i);

	/* Split 16bit calculated data value into two byte.
	 * HAL library can handle only byte's data.
	 */
	dac->TxDataBuffer[0] = data_i>>8;
	dac->TxDataBuffer[1] = data_i;

	return 0;
}

/*
 * Set desired output voltage value.
 * @param	dac Pointer to a DAC7571_t structure that contains
 *			the configuration information for the specified DAC7571 device.
 * @param	EnableState enable (1) or disable (0) the sending of the output buffer value to the DAC.
 * 			If you put it in disable state, output buffer will be set to zero.
 *
 */
void DAC7571_SetOutEnabling(DAC7571_t *dac, uint8_t EnableState)
{
	dac->m_EnableDAC = EnableState;
	if (EnableState == 0)
	{
		DAC7571_SetOutVoltage(dac, 0);
	}
}

uint8_t DAC7571_Write(DAC7571_t *dac)
{
	if (dac->m_EnableDAC == 1)
	{
		if (HAL_I2C_Master_Transmit(dac->m_i2cHandle, DAC7571_BUS_ADDRESS, dac->TxDataBuffer, 2, HAL_MAX_DELAY) == HAL_OK)
		{
			dac->m_writingAnalogVal = 1;
			/* Data sent correctly over I2C bus */
			return 0;
		}
		else
		{
			/* Error on sending data */
			return 1;
		}
	}
	else
	{
		return 1;
	}
}

uint8_t DAC7571_WriteDMA(DAC7571_t *dac)
{
	/*
	 * Send the data to I2C bus
	 */
	if (dac->m_EnableDAC == 1)
	{
		if (HAL_I2C_Master_Transmit_DMA(dac->m_i2cHandle, DAC7571_BUS_ADDRESS, dac->TxDataBuffer, 2) == HAL_OK)
		{
			dac->m_writingAnalogVal = 1;
			/* Data sent correctly to I2C DMA */
			return 0;
		}
		else
		{
			/* Error on sending data */
			return 1;
		}
	}
	else
	{
		return 1;
	}
}
void DAC7571_WriteDMA_Complete(DAC7571_t *dac)
{
	/*
	* I2C DMA Writing loop
	*/
	DAC7571_WriteDMA(dac);
	dac->m_writingAnalogVal = 0;
}

void DAC7571_TestOutput(DAC7571_t *dac)
{
	static uint16_t currentVal;
	static uint8_t slope;

	DAC7571_SetOutEnabling(dac, 1);

	if (currentVal < SinNS && slope == 0)
	{
		currentVal++;
	}
	if (currentVal == SinNS)
	{
		currentVal = 0;
	}

	DAC7571_SetOutVoltage(dac, DAC_ALIM_VOLTAGE*SinWave[currentVal]/4095.0);
}
