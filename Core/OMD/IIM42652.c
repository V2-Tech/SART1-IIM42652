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
#include "IIM42652.h"

extern osThreadId UsbTxTaskHandle;
extern osThreadId MainTaskHandle;
extern osThreadId SignalElabTaskHandle;
extern QueueHandle_t IMUDataOutputQueue;
extern uint16_t traceVar;

IIM42652_t IMU;
RINGBUFFER(AccXBuffer, IMU_FFT_SIZE, float32_t);
RINGBUFFER(AccYBuffer, IMU_FFT_SIZE, float32_t);
RINGBUFFER(AccZBuffer, IMU_FFT_SIZE, float32_t);
RINGBUFFER(GyroXBuffer, IMU_FFT_SIZE, float32_t);
RINGBUFFER(GyroYBuffer, IMU_FFT_SIZE, float32_t);
RINGBUFFER(GyroZBuffer, IMU_FFT_SIZE, float32_t);

RINGBUFFER(FIFOCount_buffer, 512, uint16_t);

/* -------------------------------------------------------------------
 * -------------------------------------------------------------------
 * 						IMPORTANT USAGE NOTES
 * -------------------------------------------------------------------
 * -------------------------------------------------------------------
 * The only register settings that user can modify during sensor operation are for ODR selection,
 * FSR selection, and sensor mode changes
 * (register parameters GYRO_ODR, ACCEL_ODR, GYRO_FS_SEL, ACCEL_FS_SEL, GYRO_MODE, ACCEL_MODE).
 * User must not modify any other register values during sensor operation.
 *
 * The following procedure must be used for register values modification:
 * • Turn Accel and Gyro Off
 * • Modify register values
 * • Turn Accel and/or Gyro On
 *
 */

//--------------------------------------
// ------ INITIALIZATION METHODS -------
//--------------------------------------

uint8_t IIM42652_Init(IIM42652_t *imu, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *csImuPinBank, uint16_t csImuPin, AC1_Config_t *actConfig)
{
	imu->m_spiHandle = spiHandle;
	imu->m_csImuPinBank = csImuPinBank;
	imu->m_csImuPin = csImuPin;
	imu->m_RegisterBank = 0;
	imu->bSingleShot = 0;
	imu->decimationFactor = 32;
	imu->m_DMPStarted = 0;

	imu->m_actUserConfig = actConfig;

	memset(imu->txBufDMA, 0x00, sizeof(imu->txBufDMA));

	/* Activate SPI-type communication */
	uint8_t txBuf[2] = {0x10 | 0x80, 0x00}; //Read dummy address: set the 8th-bit to 1.
	uint8_t rxBuf[2];
	HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(imu->m_spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);

	/* Check communication with the device */
	uint8_t status = HAL_OK;

	status += IIM42652_CheckCommunication(imu);

	if (status == HAL_OK)
	{
		/* Connection OK */
		/* Initialize the device registers */
		status += IIM42652_SetBasicConfig(imu);

		if (imu->m_actUserConfig->OMD_CONFIG_VBYTE != USER_DATA_VALID_VALUE)
		{
			//Initialize user config data to a valid/default state
			imu->m_actUserConfig->OMD_CONFIG_VBYTE = USER_DATA_VALID_VALUE;
			imu->m_actUserConfig->OMD_CONFIG_ACC_ODR = IIM42652_Accelerometer_8kHz;
			imu->m_actUserConfig->OMD_CONFIG_ACC_FSR = IIM42652_Accelerometer_8G;
			imu->m_actUserConfig->OMD_CONFIG_ACC_FILT_BW = IIM42652_Accelerometer_ODR_LL;
			imu->m_actUserConfig->OMD_CONFIG_GYRO_ODR = IIM42652_Gyroscope_8kHz;
			imu->m_actUserConfig->OMD_CONFIG_GYRO_FSR = IIM42652_Gyroscope_2000dps;
			imu->m_actUserConfig->OMD_CONFIG_GYRO_FILT_BW = IIM42652_Gyroscope_ODR_LL;
			imu->m_actUserConfig->OMD_CONFIG_WOM_X_TH = 128;
			imu->m_actUserConfig->OMD_CONFIG_WOM_Y_TH = 128;
			imu->m_actUserConfig->OMD_CONFIG_WOM_Z_TH = 128;
			imu->m_actUserConfig->OMD_CONFIG_WOM_INT_MODE = IMU_WOMIntMode_OR;
			imu->m_actUserConfig->OMD_CONFIG_WOM_MODE = IMU_WOMMode_Previous;
			imu->m_actUserConfig->OMD_CONFIG_SMD_MODE = IMU_SMDMode_WOM;
		}

		status += IIM42652_SetUserConfig(imu);
	}
	return status;
}

uint8_t IIM42652_InitBusGuard(IIM42652_t *imu, osSemaphoreId spiSemaphoreHandler)
{
	imu->m_spiSemaphoreHandle = spiSemaphoreHandler;
	configASSERT(imu->m_spiSemaphoreHandle);

	return HAL_OK;
}

uint8_t IIM42652_CheckCommunication(IIM42652_t *imu)
{
	uint8_t txBuf[2] = {REG_WHO_AM_I | 0x80, 0x00}; //Read operation: set the 8th-bit to 1.
	uint8_t rxBuf[2];

	HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
	uint8_t status = (HAL_SPI_TransmitReceive(imu->m_spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);

	if ((status == HAL_OK) && (rxBuf[1] == IIM42652_WHO_AM_I_ID))
	{
		/* IMU correctly recognized */
		status = HAL_OK;
	}
	else
	{
		status = HAL_ERROR;
	}

	return status;
}

uint8_t IIM42652_SetBasicConfig(IIM42652_t *imu)
{
	uint8_t status = HAL_OK;

	/* Power OFF Accel, Gyro and Temp sensors*/
	__IMU_TURN_OFF(imu);

	/* Check actual register bank selected */
	if (imu->m_RegisterBank != 0)
	{
		IIM42652_SetRegisterBank(imu, IMU_RegBank_0);
	}

	/* Soft-reset */
	status += IIM42652_SoftReset(imu);

	/* Interrupt pins configuration */
	status += IIM42652_SetIntType(imu, IMU_Int_1_2, IMU_Int_Pulsed);
	status += IIM42652_SetIntCircuit(imu, IMU_Int_1_2, IMU_Int_PushPull);
	status += IIM42652_SetIntPolarity(imu, IMU_Int_1_2, IMU_Int_ActiveHigh);

	/* FIFO configuration */
	status += IIM42652_SetFIFOMode(imu, IMU_FIFOMode_Steam_Continuos);
	status += IIM42652_SetFIFOCountMode(imu, IMU_FIFOCountMode_Records);
	status += IIM42652_SetFIFOHoldLastValidData_Enable(imu, DISABLE);
//	status += IIM42652_SetFIFOIntThreshold_Point(imu, IIM42652_FIFO_MIRRORING_SIZE / (2 * FIFO_16BYTES_PACKET_SIZE));
	status += IIM42652_SetFIFOIntThreshold_Point(imu, 125 /* packets */);
	status += IIM42652_SetFIFOIntThreshold_Enable(imu, ENABLE);
	status += IIM42652_SetFIFOStore_Accel(imu, ENABLE);
	status += IIM42652_SetFIFOStore_Gyro(imu, ENABLE);
	status += IIM42652_SetFIFOStore_Temperature(imu, ENABLE);
//	status += IIM42652_SetFIFOStore_FsyncTimestamp(imu, DISABLE);

	/* Interrupt event configuration */
	status += IIM42652_SetIntPulsDuration(imu, IMU_IntPuls_8u);
	status += IIM42652_SetIntPulsDeassert(imu, DISABLE);
	status += IIM42652_SetIntPulsAsyncReset(imu, ENABLE);

	/* INTERRUPT 1 sources configuration */
	status += IIM42652_SetIntSource(imu, IMU_Int_1, IMU_IntSource_RESET_DONE, DISABLE);
	status += IIM42652_SetIntSource(imu, IMU_Int_1, IMU_IntSource_UI_Data_RDY, DISABLE);
//	status += IIM42652_SetIntSource(imu, IMU_Int_1, IMU_IntSource_SMD, ENABLE);
	status += IIM42652_SetIntSource(imu, IMU_Int_1, IMU_IntSource_FIFO_THS, ENABLE);
//	status += IIM42652_SetIntSource(imu, IMU_Int_1, IMU_IntSource_UI_Data_RDY, ENABLE);

	/* Sensors AAF (Anti_Aliasing filter) */
	//(Default = 1163)
	//ODR 8kHz -> Set AAF BW to max value of 3979 <= Nyquist frequency + (-3dB range):
	//AAF_DELT=63, DELTSQR=3968, BITSHIFT=3
	uint16_t dummy16;
	uint8_t dummy8;
	status += IIM42652_GetAccelAafBW(imu, &dummy16);
	status += IIM42652_GetAccelFilterOrder(imu, &dummy8);
	status += IIM42652_GetGyroFilterOrder(imu, &dummy8);

	/* Timestamp */
	status += IIM42652_SetTMSFSyncEnable(imu, DISABLE);
	status += IIM42652_SetTMSDeltaMode(imu, ENABLE);

	/* APEX/DMP configuration */
	status += IIM42652_SetDMPPowerSaveMode(imu, DISABLE);
	status += IIM42652_SetDMPodr(imu, IIM42652_DMP_100Hz);

	osDelay(200);

	/* Power ON Accel, Gyro and Temp sensors in LN mode */
	__IMU_TURN_ON(imu);

	return status;
}

uint8_t IIM42652_SetUserConfig(IIM42652_t *imu)
{
	uint8_t status = HAL_OK;

	/* Power OFF Accel, Gyro and Temp sensors*/
	__IMU_TURN_OFF(imu);

	/* Sensors ODR and FSR */
	status += IIM42652_SetGyroFSR(imu, imu->m_actUserConfig->OMD_CONFIG_GYRO_FSR);
	status += IIM42652_SetGyroODR(imu, imu->m_actUserConfig->OMD_CONFIG_GYRO_ODR);
	status += IIM42652_SetAccelFSR(imu, imu->m_actUserConfig->OMD_CONFIG_ACC_FSR);
	status += IIM42652_SetAccelODR(imu, imu->m_actUserConfig->OMD_CONFIG_ACC_ODR);

	/* Sensors BW */
	status += IIM42652_SetGyroBW(imu, imu->m_actUserConfig->OMD_CONFIG_GYRO_FILT_BW);
	status += IIM42652_SetAccelBW(imu, imu->m_actUserConfig->OMD_CONFIG_ACC_FILT_BW);

	/* WOM configuration */
	status += IIM42652_SetWomXTh(imu, imu->m_actUserConfig->OMD_CONFIG_WOM_X_TH);
	status += IIM42652_SetWomYTh(imu, imu->m_actUserConfig->OMD_CONFIG_WOM_Y_TH);
	status += IIM42652_SetWomZTh(imu, imu->m_actUserConfig->OMD_CONFIG_WOM_Z_TH);
	status += IIM42652_SetIntSource(imu, IMU_Int_2, IMU_IntSource_WOM_X, ENABLE);
	status += IIM42652_SetIntSource(imu, IMU_Int_2, IMU_IntSource_WOM_Y, ENABLE);
	status += IIM42652_SetIntSource(imu, IMU_Int_2, IMU_IntSource_WOM_Z, ENABLE);

	osDelay(50);

	status += IIM42652_SetWOMIntMode(imu, imu->m_actUserConfig->OMD_CONFIG_WOM_INT_MODE);
	status += IIM42652_SetWOMMode(imu, imu->m_actUserConfig->OMD_CONFIG_WOM_MODE);
	status += IIM42652_SetSMDMode(imu, imu->m_actUserConfig->OMD_CONFIG_SMD_MODE);

	/* APEX/DMP enabling */
//	status += IIM42652_DMPStart(imu);

	/* Power ON Accel, Gyro and Temp sensors in LN mode */
	__IMU_TURN_ON(imu);

	return status;
}

//--------------------------------------
// ------- COMMUNICATION METHODS -------
//--------------------------------------
uint8_t IIM42652_ReadRegister(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, uint8_t *data)
{
	uint8_t txBuf[2] = {regAddr | 0x80, 0x00}; //Read operation: set the 8th-bit to 1.
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;

	if (imu->m_RegisterBank != regBank)
	{
		IIM42652_SetRegisterBank(imu, regBank);
	}

	if (osKernelRunning() == 1)
	{
		osSemaphoreWait(imu->m_spiSemaphoreHandle, osWaitForever);
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive(imu->m_spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY));
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
		osSemaphoreRelease(imu->m_spiSemaphoreHandle);
	}
	else
	{
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive(imu->m_spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY));
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
	}

	if (status == HAL_OK)
	{
		*data = rxBuf[1];
	}

	return status;
}

/* Read value of multiple bits in an 8-bit device register.
 * @param *imu 		Pointer to a IIM42652_t structure that contains
 *					the configuration information for the specified IMU device.
 * @param reg_addr 	Register regAddr to read from
 * @param start_bit First left bit position to read (0-7)
 * @param len 		Number of bits to read (not more than 8)
 * @param *data 	Pointer to variable where store the retrieved right-aligned read value
 * @return Status of operation (0 = success, 1 = error)
 */
uint8_t IIM42652_ReadRegisterBits(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, uint8_t start_bit, uint8_t len, uint8_t* data)
{
	uint8_t status = HAL_ERROR;
	uint8_t tempData = 0;

	if (len>8 || start_bit>7)
	{
		return HAL_ERROR;
	}

	if (IIM42652_ReadRegister(imu, regAddr, regBank, &tempData) == HAL_OK)
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		tempData &= mask; // zero all non-important bits in data
		tempData >>= (start_bit - len + 1); //shift data to zero position
		*data = tempData;
		status = HAL_OK;
	}
	else
	{
		status = HAL_ERROR;
		*data = 0;
	}

	return status;
}
uint8_t IIM42652_WriteRegister(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, uint8_t data)
{
	uint8_t txBuf[2] = {regAddr | 0x00, data}; //Write operation: set the 8th-bit to 0.
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;

	if (imu->m_RegisterBank != regBank)
	{
		IIM42652_SetRegisterBank(imu, regBank);
	}

	if (osKernelRunning() == 1)
	{
		osSemaphoreWait(imu->m_spiSemaphoreHandle, osWaitForever);
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive(imu->m_spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY));
		while(HAL_SPI_GetState(imu->m_spiHandle) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
		osSemaphoreRelease(imu->m_spiSemaphoreHandle);
	}
	else
	{
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive(imu->m_spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY));
		while(HAL_SPI_GetState(imu->m_spiHandle) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
	}

	return status;
}

/* Write multiple bits in an 8-bit device register.
 * @param *imu 		Pointer to a IIM42652_t structure that contains
 *					the configuration information for the specified IMU device.
 * @param reg_addr 	Register regAddr to write to
 * @param start_bit First left bit position to write (0-7)
 * @param len 		Number of bits to write (not more than 8)
 * @param data 		Right-aligned value to write
 * @return Status of operation (0 = success, 1 = error)
 */
uint8_t IIM42652_WriteRegisterBits(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, uint8_t start_bit, uint8_t len, uint8_t data)
{
	uint8_t txBuf[2];
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;
	uint8_t tempData = 0;

	if (len>8 || start_bit>7)
	{
		return HAL_ERROR;
	}

	if (IIM42652_ReadRegister(imu, regAddr, regBank, &tempData) == HAL_OK)
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		data <<= (start_bit - len + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		tempData &= ~(mask); // zero all important bits in existing byte
		tempData |= data; // combine data with existing byte

		txBuf[0] = regAddr;
		txBuf[1] = tempData;
		if (osKernelRunning() == 1)
		{
			osSemaphoreWait(imu->m_spiSemaphoreHandle, osWaitForever);
			HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
			status = (HAL_SPI_TransmitReceive(imu->m_spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY));
			while(HAL_SPI_GetState(imu->m_spiHandle) != HAL_SPI_STATE_READY);
			HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
			osSemaphoreRelease(imu->m_spiSemaphoreHandle);
		}
		else
		{
			HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
			status = (HAL_SPI_TransmitReceive(imu->m_spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY));
			while(HAL_SPI_GetState(imu->m_spiHandle) != HAL_SPI_STATE_READY);
			HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
		}
	}

	return status;
}

uint8_t IIM42652_ReadMultiRegisters(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, uint8_t *data, uint16_t byteQuantity)
{
	uint8_t *pTxBuf;
	uint8_t *pRxBuf;
	uint8_t status = HAL_ERROR;

	if (imu->m_RegisterBank != regBank)
	{
		IIM42652_SetRegisterBank(imu, regBank);
	}

	pTxBuf = ( uint8_t * )pvPortMalloc(sizeof(uint8_t) * (byteQuantity + 1));
	pRxBuf = ( uint8_t * )pvPortMalloc(sizeof(uint8_t) * (byteQuantity + 1));
	memset(pTxBuf, 0, byteQuantity + 1*sizeof(uint8_t));

	pTxBuf[0] = regAddr | 0x80; //Read operation: set the 8th-bit to 1.

	if (osKernelRunning() == 1)
	{
		osSemaphoreWait(imu->m_spiSemaphoreHandle, osWaitForever);
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive(imu->m_spiHandle, pTxBuf, pRxBuf, byteQuantity+1, HAL_MAX_DELAY));
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
		osSemaphoreRelease(imu->m_spiSemaphoreHandle);
	}
	else
	{
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive(imu->m_spiHandle, pTxBuf, pRxBuf, byteQuantity+1, HAL_MAX_DELAY));
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
	}

	if (status == HAL_OK)
	{
		memcpy(data, &pRxBuf[1], byteQuantity*sizeof(uint8_t));
	}

	vPortFree(pTxBuf);
	vPortFree(pRxBuf);

	return status;
}

uint8_t IIM42652_WriteMultiRegisters(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, uint8_t *data, uint16_t byteQuantity)
{
	uint8_t txBuf[byteQuantity + 1];
	uint8_t rxBuf[byteQuantity + 1];
	uint8_t status = HAL_ERROR;

	if (imu->m_RegisterBank != regBank)
	{
		IIM42652_SetRegisterBank(imu, regBank);
	}

	txBuf[0] = regAddr | 0x00; //Write operation: set the 8th-bit to 0.
	memcpy(txBuf+1, data, byteQuantity);

	if (osKernelRunning() == 1)
	{
		osSemaphoreWait(imu->m_spiSemaphoreHandle, osWaitForever);
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive(imu->m_spiHandle, txBuf, rxBuf, byteQuantity+1, HAL_MAX_DELAY));
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
		osSemaphoreRelease(imu->m_spiSemaphoreHandle);
	}
	else
	{
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive(imu->m_spiHandle, txBuf, rxBuf, byteQuantity+1, HAL_MAX_DELAY));
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
	}


	return status;
}

uint8_t IIM42652_ReadRegisterDMA(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank)
{
	uint8_t txBuf[2] = {regAddr | 0x80, 0x00}; //Read operation: set the 8th-bit to 1.

	if (imu->m_RegisterBank != regBank)
	{
		IIM42652_SetRegisterBank(imu, regBank);
	}

	if (osKernelRunning() == 1)
	{
		osSemaphoreWait(imu->m_spiSemaphoreHandle, osWaitForever);
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
		if (HAL_SPI_TransmitReceive_DMA(imu->m_spiHandle, txBuf, (uint8_t *)imu->rxBufDMA, 2) == HAL_OK)
		{
			return HAL_OK;
		}
		else
		{
			HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
			osSemaphoreRelease(imu->m_spiSemaphoreHandle);
			return HAL_ERROR;
		}
	}
	else
	{
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
		if (HAL_SPI_TransmitReceive_DMA(imu->m_spiHandle, txBuf, (uint8_t *)imu->rxBufDMA, 2) == HAL_OK)
		{
			return HAL_OK;
		}
		else
		{
			HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
			return HAL_ERROR;
		}
	}
}

uint8_t IIM42652_WriteRegisterDMA(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, uint8_t data)
{
	uint8_t txBuf[2] = {regAddr | 0x00, data}; //Write operation: set the 8th-bit to 0.

	if (imu->m_RegisterBank != regBank)
	{
		IIM42652_SetRegisterBank(imu, regBank);
	}

	if (osKernelRunning() == 1)
	{
		osSemaphoreWait(imu->m_spiSemaphoreHandle, osWaitForever);
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
		if (HAL_SPI_TransmitReceive_DMA(imu->m_spiHandle, txBuf, (uint8_t *)imu->rxBufDMA, 2) == HAL_OK)
		{
			return HAL_OK;
		}
		else
		{
			HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
			osSemaphoreRelease(imu->m_spiSemaphoreHandle);
			return HAL_ERROR;
		}
	}
	else
	{
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
		if (HAL_SPI_TransmitReceive_DMA(imu->m_spiHandle, txBuf, (uint8_t *)imu->rxBufDMA, 2) == HAL_OK)
		{
			return HAL_OK;
		}
		else
		{
			HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
			return HAL_ERROR;
		}
	}
}

uint8_t IIM42652_ReadMultiRegisters_DMA(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, volatile uint8_t *pRxBuf, uint16_t byteQuantity)
{
	uint8_t status = HAL_ERROR;
	/*
	uint8_t *pTxBuf;

	pTxBuf = ( uint8_t * )pvPortMalloc(sizeof(uint8_t) * (byteQuantity + 1));
	memset(pTxBuf, 0, byteQuantity + 1*sizeof(uint8_t));

	pTxBuf[0] = regAddr | 0x80; //Read operation: set the 8th-bit to 1.
	*/
	if (imu->m_RegisterBank != regBank)
	{
		IIM42652_SetRegisterBank(imu, regBank);
	}

	imu->txBufDMA[0] = regAddr | 0x80; //Read operation: set the 8th-bit to 1.

	if (osKernelRunning() == 1)
	{
		if( osSemaphoreWait(imu->m_spiSemaphoreHandle, osWaitForever) == osOK )
		{
			HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
			status = (HAL_SPI_TransmitReceive_DMA(imu->m_spiHandle, imu->txBufDMA, (uint8_t *)pRxBuf, byteQuantity+1));
			if (status != HAL_OK)
			{
				HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
				osSemaphoreRelease(imu->m_spiSemaphoreHandle);
			}
		}
	}
	else
	{
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive_DMA(imu->m_spiHandle, imu->txBufDMA, (uint8_t *)pRxBuf, byteQuantity+1));
	}

	//vPortFree(pTxBuf);

	return status;
}
uint8_t IIM42652_ReadMultiRegisters_IT(IIM42652_t *imu, uint8_t regAddr, uint8_t regBank, volatile uint8_t *pRxBuf, uint16_t byteQuantity)
{
	uint8_t status = HAL_ERROR;
	if (imu->m_RegisterBank != regBank)
	{
		IIM42652_SetRegisterBank(imu, regBank);
	}

	imu->txBufDMA[0] = regAddr | 0x80; //Read operation: set the 8th-bit to 1.

	if (osKernelRunning() == 1)
	{
		if( osSemaphoreWait(imu->m_spiSemaphoreHandle, osWaitForever) == osOK )
		{
			HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
			status = (HAL_SPI_TransmitReceive_IT(imu->m_spiHandle, imu->txBufDMA, (uint8_t *)pRxBuf, byteQuantity+1));
			while(HAL_SPI_GetState(imu->m_spiHandle) != HAL_SPI_STATE_READY);
		}
	}
	else
	{
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive_DMA(imu->m_spiHandle, imu->txBufDMA, (uint8_t *)pRxBuf, byteQuantity+1));
	}
}
void IIM42652_ReadRegisterDMA_Complete(IIM42652_t *imu)
{
	if (osKernelRunning() == 1)
	{
		osSemaphoreRelease(imu->m_spiSemaphoreHandle);
	}
	HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
}

void IIM42652_WriteRegisterDMA_Complete(IIM42652_t *imu)
{
	HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
}

//--------------------------------------
// ------- CONFIGURATION METHODS -------
//--------------------------------------
uint8_t __IMU_TURN_ON(IIM42652_t *imu)
{
	uint8_t status = HAL_ERROR;

	status = IIM42652_SetAccelMode(imu,IMU_AccelMode_LN);
	status = IIM42652_SetGyroMode(imu,IMU_GyroMode_LN);
	status = IIM42652_SetTempMode(imu,ENABLE);

	return status;
}
uint8_t __IMU_LP(IIM42652_t *imu)
{
	uint8_t status = HAL_ERROR;

	status = IIM42652_SetAccelMode(imu,IMU_AccelMode_LP);
	status = IIM42652_SetGyroMode(imu,IMU_GyroMode_OFF);
	status = IIM42652_SetTempMode(imu,ENABLE);

	return status;
}
uint8_t __IMU_TURN_OFF(IIM42652_t *imu)
{
	uint8_t status = HAL_ERROR;

	status = IIM42652_SetAccelMode(imu,IMU_AccelMode_OFF);
	status = IIM42652_SetGyroMode(imu,IMU_GyroMode_OFF);
	status = IIM42652_SetTempMode(imu,DISABLE);

	return status;
}
void __IMU_INT1_ON()
{
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}
void __IMU_INT1_OFF()
{
	HAL_NVIC_DisableIRQ(EXTI4_IRQn);
}
void __IMU_INT2_ON()
{
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
}
void __IMU_INT2_OFF()
{
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);
}
uint8_t IIM42652_SoftReset(IIM42652_t *imu)
{
	/* Device configuration register */
	/* DEVICE_CONFIG Register bits
	 * 	MSB		6		5		4				3		2		1		LSB
	 *	RES		RES		RES		SPI_MODE		RES		RES		RES		SOFT_RESET
	 *							0 = Mode 0 & 3							0
	 *							1 = Mode 1 & 2							1 = Activate reset (wait >1ms before ALL other operation)
	 */
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_DEVICE_CONFIG, IMU_RegBank_0, 0, 1, IIM42652_SOFTRESET);
	osDelay(2);

	return status;
}

/* REG_BANK_SEL Register bits
 * 	MSB		6		5		4		3		2:LSB
 *	RES		RES		RES		RES		RES		0-4=Bank Number
 */

uint8_t IIM42652_SetRegisterBank(IIM42652_t *imu, IIM42652_RegBankValue RegisterBankNumber)
{
	uint8_t txBuf[2] = {REG_REG_BANK_SEL | 0x80, 0x00}; //Read operation: set the 8th-bit to 1.
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;
	uint8_t tempData = 0;
	uint8_t start_bit = 2;
	uint8_t len = 3;

	if (RegisterBankNumber > 4)
	{
		return HAL_ERROR;
	}

	if (osKernelRunning() == 1)
	{
		osSemaphoreWait(imu->m_spiSemaphoreHandle, osWaitForever);
	}

	HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(imu->m_spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);

	if (status == HAL_OK)
	{
		tempData = rxBuf[1];
		memset(txBuf, 0x00, sizeof(txBuf));
		memset(rxBuf, 0x00, sizeof(rxBuf));
	}
	else
	{
		if (osKernelRunning() == 1)
		{
			osSemaphoreRelease(imu->m_spiSemaphoreHandle);
		}
		return status;
	}

	uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
	RegisterBankNumber <<= (start_bit - len + 1); // shift data into correct position
	RegisterBankNumber &= mask; // zero all non-important bits in data
	tempData &= ~(mask); // zero all important bits in existing byte
	tempData |= RegisterBankNumber; // combine data with existing byte

	txBuf[0] = REG_REG_BANK_SEL;
	txBuf[1] = tempData;

	HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(imu->m_spiHandle, txBuf, rxBuf, 2, HAL_MAX_DELAY));
	while(HAL_SPI_GetState(imu->m_spiHandle) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);

	if (status == HAL_OK)
	{
		imu->m_RegisterBank = RegisterBankNumber;
	}

	if (osKernelRunning() == 1)
	{
		osSemaphoreRelease(imu->m_spiSemaphoreHandle);
	}

	//	_DELAY_AUTO(1); !!!! Generate FreeRTOS crash if we call this function from ISR !!!
	return status;
}

/* INT_CONFIG Register bits
 * 	MSB		6		5			4				3				2			1				LSB
 *	RES		RES		INT2_MODE	INT2_CIRCUIT	INT2_POLARITY	INT1_MODE	INT1_CIRCUIT	INT1_POLARITY
 *					0 = Puls	0 = Open drain	0 = Active low  0 = Puls	0 = Open drain	0 = Active low
 *					1 = Latch	1 = Push pull	1 = Active high 1 = Latch	1 = Push pull	1 = Active high
 */
uint8_t IIM42652_SetIntType (IIM42652_t *imu, IIM42652_InterruptPinNumber InterruptPinNumber, IIM42652_InterruptType type)
{
	uint8_t status = 0;

	if (InterruptPinNumber == IMU_Int_1)
	{
		status = IIM42652_WriteRegisterBits(imu, REG_INT_CONFIG, IMU_RegBank_0, 2, 1, type);
	}
	else if (InterruptPinNumber == IMU_Int_2)
	{
		status = IIM42652_WriteRegisterBits(imu, REG_INT_CONFIG, IMU_RegBank_0, 5, 1, type);
	}
	else if (InterruptPinNumber == IMU_Int_1_2)
	{
		status = IIM42652_WriteRegisterBits(imu, REG_INT_CONFIG, IMU_RegBank_0, 2, 1, type);
		status = IIM42652_WriteRegisterBits(imu, REG_INT_CONFIG, IMU_RegBank_0, 5, 1, type);
	}
	else
	{
		status = HAL_ERROR;
	}

	osDelay(1);

	return status;
}
uint8_t IIM42652_SetIntCircuit (IIM42652_t *imu, IIM42652_InterruptPinNumber InterruptPinNumber, IIM42652_InterruptCircuit type)
{
	uint8_t status = 0;

	if (InterruptPinNumber == IMU_Int_1)
	{
		status = IIM42652_WriteRegisterBits(imu, REG_INT_CONFIG, IMU_RegBank_0, 1, 1, type);
	}
	else if (InterruptPinNumber == IMU_Int_2)
	{
		status = IIM42652_WriteRegisterBits(imu, REG_INT_CONFIG, IMU_RegBank_0, 4, 1, type);
	}
	else if (InterruptPinNumber == IMU_Int_1_2)
	{
		status = IIM42652_WriteRegisterBits(imu, REG_INT_CONFIG, IMU_RegBank_0, 1, 1, type);
		status = IIM42652_WriteRegisterBits(imu, REG_INT_CONFIG, IMU_RegBank_0, 4, 1, type);
	}
	else
	{
		status = HAL_ERROR;
	}

	osDelay(1);

	return status;
}
uint8_t IIM42652_SetIntPolarity (IIM42652_t *imu, IIM42652_InterruptPinNumber InterruptPinNumber, IIM42652_InterruptPolarity type)
{
	uint8_t status = 0;

	if (InterruptPinNumber == IMU_Int_1)
	{
		status = IIM42652_WriteRegisterBits(imu, REG_INT_CONFIG, IMU_RegBank_0, 0, 1, type);
	}
	else if (InterruptPinNumber == IMU_Int_2)
	{
		status = IIM42652_WriteRegisterBits(imu, REG_INT_CONFIG, IMU_RegBank_0, 3, 1, type);
	}
	else if (InterruptPinNumber == IMU_Int_1_2)
	{
		status = IIM42652_WriteRegisterBits(imu, REG_INT_CONFIG, IMU_RegBank_0, 0, 1, type);
		status = IIM42652_WriteRegisterBits(imu, REG_INT_CONFIG, IMU_RegBank_0, 3, 1, type);
	}
	else
	{
		status = HAL_ERROR;
	}

	osDelay(1);

	return status;
}

/* FIFO_CONFIG Register bits
* 	MSB:6					5		4		3		2		1		LSB
*	FIFO_MODE				RES		RES		RES		RES		RES		RES
*	0 = Bypass
*	1 = Stream-to-FIFO
*	2/3 = STOP-on-FULL
*/
uint8_t IIM42652_SetFIFOMode(IIM42652_t *imu, IIM42652_FIFOMode mode)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_FIFO_CONFIG, IMU_RegBank_0, 7, 2, mode);
	osDelay(1);
	return status;
}

/* INTF_CONFIG0 Register bits
* 	MSB					6							5						4					3:2		1:LSB
*	HOLD_LAST_DATA_EN	FIFO_COUNT_REC				FIFO_COUNT_ENDIAN		SENSOR_DATA_ENDIAN	RES		UI_SIFS_CFG
*						0 = COUNT report bytes		0 = Little endian       0 = Little endian 			2 = Disable SPI
*						1 = COUNT report record		1 = Big endian          1 = Big endian    			3 = Disable I2C
*/
uint8_t IIM42652_SetFIFOCountMode(IIM42652_t *imu, IIM42652_FIFOCountMode mode)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_INTF_CONFIG0, IMU_RegBank_0, 6, 1, mode);
	osDelay(1);
	return status;
}
uint8_t IIM42652_SetFIFOHoldLastValidData_Enable(IIM42652_t *imu, uint8_t EnableState)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_INTF_CONFIG0, IMU_RegBank_0, 7, 1, EnableState);
	osDelay(1);
	return status;
}
/* FIFO_CONFIG1 Register bits
* 	MSB		6			5			4			3				2			1			LSB
*	RES		PARTIAL_RD	WM_GT_TH	HIRES_EN	TMST_FSYNC_EN	TEMP_EN		GYRO_EN		ACCEL_EN
*/
uint8_t IIM42652_SetFIFOIntThreshold_Enable(IIM42652_t *imu, uint8_t EnableState)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_FIFO_CONFIG1, IMU_RegBank_0, 5, 1, EnableState);
	osDelay(1);
	return status;
}
uint8_t IIM42652_SetFIFOStore_Accel(IIM42652_t *imu, uint8_t EnableState)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_FIFO_CONFIG1, IMU_RegBank_0, 0, 1, EnableState);
	osDelay(1);
	return status;
}
uint8_t IIM42652_SetFIFOStore_Gyro(IIM42652_t *imu, uint8_t EnableState)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_FIFO_CONFIG1, IMU_RegBank_0, 1, 1, EnableState);
	osDelay(1);
	return status;
}
uint8_t IIM42652_SetFIFOStore_Temperature(IIM42652_t *imu, uint8_t EnableState)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_FIFO_CONFIG1, IMU_RegBank_0, 2, 1, EnableState);
	osDelay(1);
	return status;
}
uint8_t IIM42652_SetFIFOStore_FsyncTimestamp(IIM42652_t *imu, uint8_t EnableState)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_FIFO_CONFIG1, IMU_RegBank_0, 3, 1, EnableState);
	osDelay(1);
	return status;
}

/* FIFO_CONFIG2 Register bits
 * 	MSB:LSB
 *	FIFO_WM[7:0]
 */
/* FIFO_CONFIG3 Register bits
 * 	MSB:LSB
 *	FIFO_WM[11:8]
 */
uint8_t IIM42652_SetFIFOIntThreshold_Point(IIM42652_t *imu, uint16_t threshold)
{
	uint8_t data[2];
	data[0] = threshold & 0x00ff;
	data[1] = (threshold >> 8) & 0x00ff;
	uint8_t status = IIM42652_WriteMultiRegisters(imu, REG_FIFO_CONFIG2, IMU_RegBank_0, data, 2);
	osDelay(1);
	return status;
}

/* INT_CONFIG1 Register bits
 * 	MSB		6				5					4			3		2		1		LSB
 *	RES		TPULSE_DURATION	TDEASSERT_DISABLE	ASYNC_RESET	RES		RES		RES		RES
 */
uint8_t IIM42652_SetIntPulsDuration(IIM42652_t *imu, IIM42652_InterruptDuration duration)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_INT_CONFIG1, IMU_RegBank_0, 6, 1, duration);
	osDelay(1);
	return status;
}
uint8_t IIM42652_SetIntPulsDeassert(IIM42652_t *imu, uint8_t EnableState)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_INT_CONFIG1, IMU_RegBank_0, 5, 1, !EnableState);
	osDelay(1);
	return status;
}
uint8_t IIM42652_SetIntPulsAsyncReset(IIM42652_t *imu, uint8_t EnableState)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_INT_CONFIG1, IMU_RegBank_0, 4, 1, !EnableState);
	osDelay(1);
	return status;
}

/* INT_SOURCE0/3 Register bits
 * 	MSB		6			5			4			3			2			1			LSB
 *	RES		UI_FSYNC	PLL_RDY		RESET_DONE	UI_DRDY		FIFO_THS	FIFO_FULL	UI_AGC_RDY
 */
/* INT_SOURCE1/4 Register bits
* 	MSB		6					5		4		3		2		1		LSB
*	RES		I3C_PROTOCOL_ERROR	RES		RES		SMD		WOM_Z	WOM_Y	WOM_X
*/
uint8_t IIM42652_SetIntSource(IIM42652_t *imu, IIM42652_InterruptPinNumber InterruptPinNumber,
								IIM42652_InterruptSource source, uint8_t EnableState)
{
	uint8_t status = 0;

	if (InterruptPinNumber == IMU_Int_1)
	{
		if (source <= IMU_IntSource_UI_FSYNC)
		{
			status = IIM42652_WriteRegisterBits(imu, REG_INT_SOURCE0, IMU_RegBank_0, source, 1, EnableState);
		}
		else
		{
			status = IIM42652_WriteRegisterBits(imu, REG_INT_SOURCE1, IMU_RegBank_0, source-8, 1, EnableState);
		}
	}
	else if (InterruptPinNumber == IMU_Int_2)
	{
		if (source <= IMU_IntSource_UI_FSYNC)
		{
			status = IIM42652_WriteRegisterBits(imu, REG_INT_SOURCE3, IMU_RegBank_0, source, 1, EnableState);
		}
		else
		{
			status = IIM42652_WriteRegisterBits(imu, REG_INT_SOURCE4, IMU_RegBank_0, source-8, 1, EnableState);
		}
	}
	else if (InterruptPinNumber == IMU_Int_1_2)
	{
		if (source <= IMU_IntSource_UI_FSYNC)
		{
			status = IIM42652_WriteRegisterBits(imu, REG_INT_SOURCE0, IMU_RegBank_0, source, 1, EnableState);
			status = IIM42652_WriteRegisterBits(imu, REG_INT_SOURCE3, IMU_RegBank_0, source, 1, EnableState);
		}
		else
		{
			status = IIM42652_WriteRegisterBits(imu, REG_INT_SOURCE1, IMU_RegBank_0, source-8, 1, EnableState);
			status = IIM42652_WriteRegisterBits(imu, REG_INT_SOURCE4, IMU_RegBank_0, source-8, 1, EnableState);
		}
	}
	else
	{
		status = HAL_ERROR;
	}

	osDelay(1);

	return status;
}

/* ACCEL_WOM_X_THR Register bits
 * 	MSB:LSB
 *	WOM_X_THR Fixed range [0g : 1g] with resolution 1g/256=~3.9mg
 */
uint8_t IIM42652_SetWomXTh(IIM42652_t *imu, uint8_t threshold)
{
	uint8_t status = IIM42652_WriteRegister(imu, REG_ACCEL_WOM_X_THR, IMU_RegBank_4, threshold);

	uint8_t value;
	IIM42652_GetWomXTh(imu, &value);
	osDelay(1);
	return status;
}
uint8_t IIM42652_GetWomXTh(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_ReadRegister(imu, REG_ACCEL_WOM_X_THR, IMU_RegBank_4, value);
}
uint8_t IIM42652_SetWomYTh(IIM42652_t *imu, uint8_t threshold)
{
	uint8_t status = IIM42652_WriteRegister(imu, REG_ACCEL_WOM_Y_THR, IMU_RegBank_4, threshold);
	uint8_t value;
	IIM42652_GetWomYTh(imu, &value);
	osDelay(1);
	return status;
}
uint8_t IIM42652_GetWomYTh(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_ReadRegister(imu, REG_ACCEL_WOM_Y_THR, IMU_RegBank_4, value);
}
uint8_t IIM42652_SetWomZTh(IIM42652_t *imu, uint8_t threshold)
{
	uint8_t status = IIM42652_WriteRegister(imu, REG_ACCEL_WOM_Z_THR, IMU_RegBank_4, threshold);
	uint8_t value;
	IIM42652_GetWomZTh(imu, &value);
	osDelay(1);
	return status;
}
uint8_t IIM42652_GetWomZTh(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_ReadRegister(imu, REG_ACCEL_WOM_Z_THR, IMU_RegBank_4, value);
}

/* SMD_CONFIG Register bits
 * 	MSB		6		5		4		3				2			1:LSB
 *	RES		RES		RES		RES		WOM_INT_MODE	WOM_MODE	SMD_MODE
 */
uint8_t IIM42652_SetWOMIntMode(IIM42652_t *imu, IIM42652_WOMInterruptMode mode)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_SMD_CONFIG, IMU_RegBank_0, 3, 1, mode);
//	_DELAY_AUTO(1);
	return status;
}
uint8_t IIM42652_GetWOMIntMode(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_ReadRegisterBits(imu, REG_SMD_CONFIG, IMU_RegBank_0, 3, 1, value);
}
uint8_t IIM42652_SetWOMMode(IIM42652_t *imu, IIM42652_WOMMode mode)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_SMD_CONFIG, IMU_RegBank_0, 2, 1, mode);
//	_DELAY_AUTO(1);
	return status;
}
uint8_t IIM42652_GetWOMMode(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_ReadRegisterBits(imu, REG_SMD_CONFIG, IMU_RegBank_0, 2, 1, value);
}
uint8_t IIM42652_SetSMDMode(IIM42652_t *imu, IIM42652_SMDMode mode)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_SMD_CONFIG, IMU_RegBank_0, 1, 2, mode);
//	_DELAY_AUTO(1);
	return status;
}
uint8_t IIM42652_GetSMDMode(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_ReadRegisterBits(imu, REG_SMD_CONFIG, IMU_RegBank_0, 1, 2, value);
}

/* PWR_MGMT0 Register bits
 * 	MSB		6		5			4		3:2			1:LSB
 *	RES		RES		TEMP_DIS	IDLE	GYRO_MODE	ACCEL_MODE
 */
uint8_t IIM42652_SetAccelMode(IIM42652_t *imu, IIM42652_AccelMode mode)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_PWR_MGMT0, IMU_RegBank_0, 1, 2, mode);
	_DELAY_AUTO(1);
	return status;
}
uint8_t IIM42652_SetGyroMode(IIM42652_t *imu, IIM42652_GyroMode mode)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_PWR_MGMT0, IMU_RegBank_0, 3, 2, mode);
	_DELAY_AUTO(1);
	return status;
}
uint8_t IIM42652_SetTempMode(IIM42652_t *imu, uint8_t EnableState)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_PWR_MGMT0, IMU_RegBank_0, 5, 1, !EnableState);
	_DELAY_AUTO(1);
	return status;
}

uint8_t IIM42652_GetAccelAafBW(IIM42652_t *imu, uint16_t* value)
{
	uint8_t AAF_en_state = 0;
	uint8_t AAF_delt = 0;
	uint16_t AAF_deltSqr = 0;
	uint8_t AAF_bs = 0;
	uint8_t dataBuf[3];
	uint8_t status = IIM42652_ReadMultiRegisters(imu, REG_ACCEL_CONFIG_STATIC2, IMU_RegBank_2, dataBuf, 3);

	AAF_en_state = (dataBuf[0] & 0x01);
	AAF_en_state ^= 1U;

	AAF_delt = (dataBuf[0] >> 1) & 0x3F;

	AAF_deltSqr = dataBuf[1] | ((dataBuf[2] & 0x0F) << 8);

	AAF_bs = ((dataBuf[2] >> 4) & 0x0F);

	return status;
}
uint8_t IIM42652_GetGyroAafBW(IIM42652_t *imu, uint16_t* value)
{
	;
}

uint8_t IIM42652_SetGyroFSR(IIM42652_t *imu, IIM42652_GyroFSRValues GyroFSR)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_GYRO_CONFIG0, IMU_RegBank_0, 7, 3, GyroFSR);
	if (status == HAL_OK)
	{
		IIM42652_SetGyroScaleFactor(imu, GyroFSR);
	}
	_DELAY_AUTO(1);
	return status;
}
uint8_t IIM42652_GetGyroFSR(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_ReadRegisterBits(imu, REG_GYRO_CONFIG0, IMU_RegBank_0, 7, 3, value);
}
uint8_t IIM42652_SetAccelFSR(IIM42652_t *imu, IIM42652_AccFSRValues AccelFSR)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_ACCEL_CONFIG0, IMU_RegBank_0, 7, 3, AccelFSR);
	if (status == HAL_OK)
	{
		IIM42652_SetAccelScaleFactor(imu, AccelFSR);
	}
	_DELAY_AUTO(1);
	return status;
}
uint8_t IIM42652_GetAccelFSR(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_ReadRegisterBits(imu, REG_ACCEL_CONFIG0, IMU_RegBank_0, 7, 3, value);
}
uint8_t IIM42652_SetGyroODR(IIM42652_t *imu, IIM42652_GyroODRValues GyroODR)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_GYRO_CONFIG0, IMU_RegBank_0, 3, 4, GyroODR);
	if (GyroODR == IIM42652_Gyroscope_32kHz)
	{
		imu->decimationFactor = 2;
	}
	else if(GyroODR == IIM42652_Gyroscope_16kHz)
	{
		imu->decimationFactor = 1;
	}
	else
	{
		imu->decimationFactor = 0;
	}
	_DELAY_AUTO(1);
	return status;
}
uint8_t IIM42652_GetGyroODR(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_ReadRegisterBits(imu, REG_GYRO_CONFIG0, IMU_RegBank_0, 3, 4, value);
}
uint8_t IIM42652_SetAccelODR(IIM42652_t *imu, IIM42652_AccODRValues AccelODR)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_ACCEL_CONFIG0, IMU_RegBank_0, 3, 4, AccelODR);
	if (AccelODR == IIM42652_Accelerometer_32kHz)
	{
		imu->decimationFactor = 2;
	}
	else if(AccelODR == IIM42652_Accelerometer_16kHz)
	{
		imu->decimationFactor = 1;
	}
	else
	{
		imu->decimationFactor = 0;
	}
	_DELAY_AUTO(1);
	return status;
}
uint8_t IIM42652_GetAccelODR(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_ReadRegisterBits(imu, REG_ACCEL_CONFIG0, IMU_RegBank_0, 3, 4, value);
}

/* 	GYRO_ACCEL_CONFIG0 Register bits
* 	MSB:4				3:LSB
*	ACCEL_UI_FILT_BW	GYRO_UI_FILT_BW
*/
uint8_t IIM42652_SetAccelBW(IIM42652_t *imu, IIM42652_AccBWValues AccelBW)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_GYRO_ACCEL_CONFIG0, IMU_RegBank_0, 7, 4, AccelBW);
	_DELAY_AUTO(1);
	return status;
}
uint8_t IIM42652_GetAccelBW(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_ReadRegisterBits(imu, REG_GYRO_ACCEL_CONFIG0, IMU_RegBank_0, 7, 4, value);
}
uint8_t IIM42652_SetGyroBW(IIM42652_t *imu, IIM42652_GyroBWValues GyroBW)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_GYRO_ACCEL_CONFIG0, IMU_RegBank_0, 3, 4, GyroBW);
	_DELAY_AUTO(1);
	return status;
}
uint8_t IIM42652_GetGyroBW(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_ReadRegisterBits(imu, REG_GYRO_ACCEL_CONFIG0, IMU_RegBank_0, 3, 4, value);
}
uint8_t IIM42652_SetAccelFilterOrder(IIM42652_t *imu, IIM42652_UIFilterOrder FO)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_ACCEL_CONFIG1, IMU_RegBank_0, 4, 2, FO);
	_DELAY_AUTO(1);
	return status;
}
uint8_t IIM42652_GetAccelFilterOrder(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_ReadRegisterBits(imu, REG_ACCEL_CONFIG1, IMU_RegBank_0, 4, 2, value);
}
uint8_t IIM42652_SetGyroFilterOrder(IIM42652_t *imu, IIM42652_UIFilterOrder FO)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_GYRO_CONFIG1, IMU_RegBank_0, 3, 2, FO);
	_DELAY_AUTO(1);
	return status;
}
uint8_t IIM42652_GetGyroFilterOrder(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_ReadRegisterBits(imu, REG_GYRO_CONFIG1, IMU_RegBank_0, 3, 2, value);
}

void IIM42652_SetAccelScaleFactor(IIM42652_t *imu, IIM42652_AccFSRValues AccelFSR)
{
	switch (AccelFSR)
	{
	case IIM42652_Accelerometer_16G:
		imu->AccelScaleFactor = 2048;
		break;
	case IIM42652_Accelerometer_8G:
		imu->AccelScaleFactor = 4096;
		break;
	case IIM42652_Accelerometer_4G:
		imu->AccelScaleFactor = 8192;
		break;
	case IIM42652_Accelerometer_2G:
		imu->AccelScaleFactor = 16384;
		break;
	}
}
void IIM42652_SetGyroScaleFactor(IIM42652_t *imu, IIM42652_GyroFSRValues GyroFSR)
{
	switch (GyroFSR)
	{
	case IIM42652_Gyroscope_2000dps:
		imu->GyroScaleFactorRad = 16.4 / ((2 * M_PI)/360.0);
		imu->GyroScaleFactorDegree = 16.4;
		break;
	case IIM42652_Gyroscope_1000dps:
		imu->GyroScaleFactorRad = 32.8 / ((2 * M_PI)/360.0);
		imu->GyroScaleFactorDegree = 32.8;
		break;
	case IIM42652_Gyroscope_500dps:
		imu->GyroScaleFactorRad = 65.5 / ((2 * M_PI)/360.0);
		imu->GyroScaleFactorDegree = 65.5;
		break;
	case IIM42652_Gyroscope_250dps:
		imu->GyroScaleFactorRad = 131 / ((2 * M_PI)/360.0);
		imu->GyroScaleFactorDegree = 131;
		break;
	case IIM42652_Gyroscope_125dps:
		imu->GyroScaleFactorRad = 262 / ((2 * M_PI)/360.0);
		imu->GyroScaleFactorDegree = 262;
		break;
	case IIM42652_Gyroscope_62_5dps:
		imu->GyroScaleFactorRad = 524.3 / ((2 * M_PI)/360.0);
		imu->GyroScaleFactorDegree = 524.3;
		break;
	case IIM42652_Gyroscope_31_25dps:
		imu->GyroScaleFactorRad = 1048.6 / ((2 * M_PI)/360.0);
		imu->GyroScaleFactorDegree = 1048.6;
		break;
	case IIM42652_Gyroscope_15_625dps:
		imu->GyroScaleFactorRad = 2097.2 / ((2 * M_PI)/360.0);
		imu->GyroScaleFactorDegree = 2097.2;
		break;
	}
}

/* TMST_CONFIG Register bits
 * 	MSB:5		4							3				2				1			LSB
 *	RES			TMST_TO_REGS				TMST_RES		TMST_DELTA		TMST_FSYNC	TMST_EN
 *				0=TMST_VALUE always 0		0=1us/LSB(def)	0=Last ODR					0=Disable TMS report
 *				1=TMST_VALUE return values	1=16us/LSB		1=Difference				1=Enable TMS report(def)
 */
uint8_t IIM42652_SetTMSReportEnable(IIM42652_t *imu, uint8_t EnableState)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_TMST_CONFIG, IMU_RegBank_0, 0, 1, EnableState);
	osDelay(1);
	return status;
}
uint8_t IIM42652_SetTMSFSyncEnable(IIM42652_t *imu, uint8_t EnableState)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_TMST_CONFIG, IMU_RegBank_0, 1, 1, EnableState);
	osDelay(1);
	return status;
}
uint8_t IIM42652_SetTMSDeltaMode(IIM42652_t *imu, uint8_t EnableState)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_TMST_CONFIG, IMU_RegBank_0, 2, 1, EnableState);
	osDelay(1);
	return status;
}
uint8_t IIM42652_SetTMSHighResolution(IIM42652_t *imu, uint8_t EnableState)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_TMST_CONFIG, IMU_RegBank_0, 3, 1, EnableState);
	osDelay(1);
	return status;
}
uint8_t IIM42652_SetTMStoRegEnable(IIM42652_t *imu, uint8_t EnableState)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_TMST_CONFIG, IMU_RegBank_0, 4, 1, EnableState);
	osDelay(1);
	return status;
}

/* APEX_CONFIG0 Register bits
 * 	MSB 				6					5					4				3			2				1:LSB
 *	DMP_POWER_SAVE		TAP_ENABLE			PED_ENABLE			TILT_ENABLE		RES			FF_ENABLE 		DMP_ODR
 *	0=Disabled																								0=25Hz
 *	1=Enabled																								1=500Hz
 *																											2=50Hz (Default)
 *																											3=100Hz
 */
uint8_t IIM42652_SetDMPPowerSaveMode(IIM42652_t *imu, uint8_t EnableState)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_APEX_CONFIG0, IMU_RegBank_0, 7, 1, EnableState);
	return status;
}
uint8_t IIM42652_SetDMPodr(IIM42652_t *imu, IIM42652_DMPODRValueS DmpODR)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_APEX_CONFIG0, IMU_RegBank_0, 1, 2, DmpODR);
	return status;
}

/* SIGNAL_PATH_RESET Register bits
 * 	MSB 	6					5					4		3					2				1				LSB
 *	RES		DMP_INIT_EN			DMP_MEM_RESET_EN	RES		ABORT_AND_RESET		TMST_STROBE		FIFO_FLUSH		RES
 *			0=DMP disabled		0=							0=					0=				0=
 *			1=DMP enabled		1=DMP memory reset			1=Restart ODR and	1=				1=Flush all
 *															signal path data					FIFO data
 */
uint8_t IIM42652_SetDMPInit(IIM42652_t *imu, uint8_t EnableState)
{
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_SIGNAL_PATH_RESET, IMU_RegBank_0, 6, 1, EnableState);
	osDelay(1);
	return status;
}
uint8_t IIM42652_SetDMPMemReset(IIM42652_t *imu)
{
	const int16_t ref_timeout = 50; /*50 ms*/
	static int16_t timeout = ref_timeout;
	uint8_t state;

	/* Reset DMP internal memories */
	uint8_t status = IIM42652_WriteRegisterBits(imu, REG_SIGNAL_PATH_RESET, IMU_RegBank_0, 5, 1, ENABLE);
	osDelay(1);

//	status += IIM42652_WriteRegisterBits(imu, REG_SIGNAL_PATH_RESET, IMU_RegBank_0, 5, 1, DISABLE);
//	if (status == HAL_OK)
//	{
//		/* Make sure reset procedure has finished by reading back DMP_MEM_RESET_EN bit */
//		do
//		{
//			IIM42652_GetDMPMemReset(imu, &state);
//			osDelay(1);
//		} while(state && timeout--);
//
//		if (timeout <= 0)
//		{
//			return HAL_ERROR;
//		}
//	}
	return status;
}
uint8_t IIM42652_GetDMPMemReset(IIM42652_t *imu, uint8_t *state)
{
	return IIM42652_ReadRegisterBits(imu, REG_SIGNAL_PATH_RESET, IMU_RegBank_0, 5, 1, state);
}

uint8_t IIM42652_DMPStart(IIM42652_t *imu)
{
	uint8_t status = HAL_OK;

	/* On first enabling of DMP, reset internal state */
	if(!imu->m_DMPStarted)
	{
		status += IIM42652_SetDMPMemReset(imu);
		imu->m_DMPStarted = 1;
	}

	/* Initialize DMP */
	status += IIM42652_DMPResume(imu);

	return status;
}
uint8_t IIM42652_DMPResume(IIM42652_t *imu)
{
	const int16_t ref_timeout = 50; /*50 ms*/
	static int16_t timeout = ref_timeout;
	uint8_t state;

	uint8_t status = IIM42652_SetDMPInit(imu, ENABLE);

	/* Max accel ODR is 500Hz, and DMP INIT is updated every accel ODR */
	osDelay(2);

	if(status == HAL_OK)
	{
		/* Wait for DMP idle */
		do
		{
			IIM42652_GetDMPState(imu, &state);
			osDelay(1);
		} while(!state && timeout--);

		if (timeout <= 0)
		{
			return HAL_ERROR;
		}
	}
	return status;
}
uint8_t IIM42652_GetDMPState(IIM42652_t *imu, uint8_t *state)
{
	return IIM42652_ReadRegisterBits(imu, REG_APEX_DATA3, IMU_RegBank_0, 2, 1, state);
}

uint8_t IIM42652_FIFO_Flush(IIM42652_t *imu)
{
	return IIM42652_WriteRegisterBits(imu, REG_SIGNAL_PATH_RESET, IMU_RegBank_0, 1, 1, 1);
}

uint16_t IIM42652_GetFIFOCount(IIM42652_t *imu)
{
	uint8_t rxBuf[2];
	if (IIM42652_ReadMultiRegisters(imu, REG_FIFO_COUNTH, IMU_RegBank_0, rxBuf, 2) == HAL_OK)
	{
		return (int16_t) ((rxBuf[0] << 8) | rxBuf[1]);
	}

	return -1;
}

uint16_t IIM42652_GetFIFOCount_ISR(IIM42652_t *imu)
{
	uint8_t txBuf[3];
	uint8_t rxBuf[3];
	uint8_t status = HAL_ERROR;

	txBuf[0] = REG_FIFO_COUNTH | 0x80; //Read operation: set the 8th-bit to 1.

	if (osKernelRunning() == 1)
	{
		osSemaphoreWait(imu->m_spiSemaphoreHandle, osWaitForever);
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive(imu->m_spiHandle, txBuf, rxBuf, 2+1, HAL_MAX_DELAY));
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
		osSemaphoreRelease(imu->m_spiSemaphoreHandle);
	}
	else
	{
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive(imu->m_spiHandle, txBuf, rxBuf, 2+1, HAL_MAX_DELAY));
		HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);
	}

	if (status == HAL_OK)
	{
		return (int16_t) ((rxBuf[1] << 8) | rxBuf[2]);
	}
	else
	{
		return 0;
	}
}

uint8_t IIM42652_GetFIFOData(IIM42652_t *imu)
{
	/* Ensure data ready status bit is set and FIFO as reached at least FIFO_THS point */
	uint8_t intStatus;
	if (IIM42652_GetIntStatus(imu, &intStatus) != HAL_OK)
	{
		return HAL_ERROR;
	}

	if (BitCheckB(intStatus, IMU_IntStatus_Data_Ready) || BitCheckB(intStatus, IMU_IntStatus_FIFO_Ths) || BitCheckB(intStatus, IMU_IntStatus_FIFO_Full))
	{
		static uint16_t decimatorCounter = 0;
		fifo_header_t * header;
		uint16_t packet_count_i = 0;
		uint16_t packetCount = IIM42652_GetFIFOCount_ISR(imu);
		imu->m_FIFO_Count = packetCount;
		if (IIM42652_ReadMultiRegisters(imu, REG_FIFO_DATA, IMU_RegBank_0, (uint8_t *)imu->m_FIFOData, FIFO_16BYTES_PACKET_SIZE * packetCount) != HAL_OK)
		{
			/* sensor data is in FIFO according to FIFO_COUNT but failed to read FIFO,
			 * reset FIFO and try next chance */
			IIM42652_FIFO_Flush(imu);
			return HAL_ERROR;
		}

		if (imu->m_FIFO_Count > 0)
		{
			uint16_t fifo_idx = 0;

			for(packet_count_i = 0; packet_count_i < imu->m_FIFO_Count; packet_count_i++)
			{
				IIM42652_FIFO_Packet16_t event;

				header = (fifo_header_t *) &imu->m_FIFOData[fifo_idx];
				event.m_FIFO_Header = imu->m_FIFOData[fifo_idx];
				fifo_idx += FIFO_HEADER_SIZE;

				if (header->bits.msg_bit)
				{
					/* MSG BIT set in FIFO header,
					 * reset FIFO and try next chance */
					IIM42652_FIFO_Flush(imu);

					imu->m_FIFO_reading = 0;
					return HAL_ERROR;
				}

				if(header->bits.accel_bit)
				{
					IIM42652_Format_Data(BIG_ENDIAN_DATA, (uint8_t *)&imu->m_FIFOData[0+fifo_idx], (uint16_t *)&event.m_FIFO_Accel_X);
					IIM42652_Format_Data(BIG_ENDIAN_DATA, (uint8_t *)&imu->m_FIFOData[2+fifo_idx], (uint16_t *)&event.m_FIFO_Accel_Y);
					IIM42652_Format_Data(BIG_ENDIAN_DATA, (uint8_t *)&imu->m_FIFOData[4+fifo_idx], (uint16_t *)&event.m_FIFO_Accel_Z);

					fifo_idx += FIFO_ACCEL_DATA_SIZE;
				}

				if (header->bits.gyro_bit)
				{
					IIM42652_Format_Data(BIG_ENDIAN_DATA, (uint8_t *)&imu->m_FIFOData[0+fifo_idx], (uint16_t *)&event.m_FIFO_Gyro_X);
					IIM42652_Format_Data(BIG_ENDIAN_DATA, (uint8_t *)&imu->m_FIFOData[2+fifo_idx], (uint16_t *)&event.m_FIFO_Gyro_Y);
					IIM42652_Format_Data(BIG_ENDIAN_DATA, (uint8_t *)&imu->m_FIFOData[4+fifo_idx], (uint16_t *)&event.m_FIFO_Gyro_Z);

					fifo_idx += FIFO_GYRO_DATA_SIZE;
				}

				if ((header->bits.accel_bit) || (header->bits.gyro_bit))
				{
					event.m_FIFO_Temp = (int8_t)imu->m_FIFOData[0+fifo_idx];

					fifo_idx += FIFO_TEMP_DATA_SIZE;
				}

				if ((header->bits.timestamp_bit) || (header->bits.fsync_bit))
				{
					IIM42652_Format_Data(BIG_ENDIAN_DATA, (uint8_t *)&imu->m_FIFOData[0+fifo_idx], &event.m_FIFO_TimeStamp);

					fifo_idx += FIFO_TS_FSYNC_SIZE;
				}

				//Check valid value and add to fft buffer
				float32_t tempValue;
				if(event.m_FIFO_Accel_X != FIFO_INVALID_SAMPLE)
				{
					tempValue = IIM42652_GetAccelGravity(imu, event.m_FIFO_Accel_X);
					RINGBUFFER_PUSH(&AccXBuffer, (float32_t *)&tempValue);
				}
				if(event.m_FIFO_Accel_Y != FIFO_INVALID_SAMPLE)
				{
					tempValue = IIM42652_GetAccelGravity(imu, event.m_FIFO_Accel_Y);
					RINGBUFFER_PUSH(&AccYBuffer, (float32_t *)&tempValue);
				}
				if(event.m_FIFO_Accel_Z != FIFO_INVALID_SAMPLE)
				{
					tempValue = IIM42652_GetAccelGravity(imu, event.m_FIFO_Accel_Z);
					RINGBUFFER_PUSH(&AccZBuffer, (float32_t *)&tempValue);
				}
				if(event.m_FIFO_Gyro_X != FIFO_INVALID_SAMPLE)
				{
					tempValue = IIM42652_GetGyroRps(imu, event.m_FIFO_Gyro_X);
					RINGBUFFER_PUSH(&GyroXBuffer, (float32_t *)&tempValue);
				}
				if(event.m_FIFO_Gyro_Y != FIFO_INVALID_SAMPLE)
				{
					tempValue = IIM42652_GetGyroRps(imu, event.m_FIFO_Gyro_Y);
					RINGBUFFER_PUSH(&GyroYBuffer, (float32_t *)&tempValue);
				}
				if(event.m_FIFO_Gyro_Z != FIFO_INVALID_SAMPLE)
				{
					tempValue = IIM42652_GetGyroRps(imu, event.m_FIFO_Gyro_Z);
					RINGBUFFER_PUSH(&GyroZBuffer, (float32_t *)&tempValue);
				}

				if (imu->TelemetryON == 1)
				{
					if (decimatorCounter >= imu->decimationFactor)
					{
						IMU_Data_Buf_t imuPacketBuf, dummyPacket;
						imu_TO_buffer(imu, 1, &event, imuPacketBuf.buffer);
						if (!xQueueIsQueueFullFromISR(IMUDataOutputQueue))
						{
							xQueueSend(IMUDataOutputQueue, &imuPacketBuf.buffer, portMAX_DELAY);
							xTaskGenericNotify(UsbTxTaskHandle, AC1_Notify_NewDataAvaiable, eSetBits, NULL);
							decimatorCounter = 0;
						}
						else
						{
							xQueueReceive(IMUDataOutputQueue, &dummyPacket.buffer, portMAX_DELAY);
							xQueueSend(IMUDataOutputQueue, &imuPacketBuf.buffer, portMAX_DELAY);
							xTaskGenericNotify(UsbTxTaskHandle, AC1_Notify_NewDataAvaiable, eSetBits, NULL);
							decimatorCounter = 0;
						}
					}
					else
					{
						decimatorCounter++;
					}
				}
			}
		}
	}
}
uint8_t IIM42652_GetFIFOData_DMA(IIM42652_t *imu)
{
	imu->m_FIFO_reading = 1;
	uint16_t packetCount = IIM42652_GetFIFOCount_ISR(imu);
//	uint16_t packetCount = IIM42652_GetFIFOCount(imu);
//	uint16_t packetCount = 128;
	imu->m_FIFO_Count = packetCount;
	RINGBUFFER_PUSH(&FIFOCount_buffer, &packetCount);

	return (IIM42652_ReadMultiRegisters_DMA(imu, REG_FIFO_DATA, IMU_RegBank_0, imu->m_FIFOData, FIFO_16BYTES_PACKET_SIZE * packetCount) == HAL_OK);
}
void IIM42652_GetFIFOData_DMA_Complete(IIM42652_t *imu)
{
	fifo_header_t * header;
	uint16_t packet_count_i = 0;
	volatile static uint16_t decimatorCounter = 0;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	HAL_GPIO_WritePin(imu->m_csImuPinBank, imu->m_csImuPin, GPIO_PIN_SET);

//	/*
//	 * Clean FIFO mirror buffer
//	 */
//	// Trash the first byte
//	memcpy((uint8_t *)imu->m_FIFOData, (uint8_t *)&imu->m_FIFOData[1], imu->m_FIFO_Count * FIFO_16BYTES_PACKET_SIZE);
//	// Delete the NULL last one
//	imu->m_FIFOData[imu->m_FIFO_Count] = 0;

	/*
	 * Decrypt FIFO data and push them into ring buffer
	 */
	if (imu->m_FIFO_Count > 0)
	{
		/* First byte is a dummy byte relative to the transmitted byte response: trash it */
		uint16_t fifo_idx = 1;

		for(packet_count_i = 0; packet_count_i < imu->m_FIFO_Count; packet_count_i++)
		{
			IIM42652_FIFO_Packet16_t event;

			header = (fifo_header_t *) &imu->m_FIFOData[fifo_idx];
			event.m_FIFO_Header = imu->m_FIFOData[fifo_idx];
			fifo_idx += FIFO_HEADER_SIZE;
			if (header->bits.msg_bit)
			{
				/* MSG BIT set in FIFO header, Resetting FIFO */
				IIM42652_FIFO_Flush(imu);

				imu->m_FIFO_reading = 0;
				if (osKernelRunning() == 1)
				{
					osSemaphoreRelease(imu->m_spiSemaphoreHandle);
				}

				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				return;
			}

			if(header->bits.accel_bit)
			{
				IIM42652_Format_Data(BIG_ENDIAN_DATA, (uint8_t *)&imu->m_FIFOData[0+fifo_idx], (uint16_t *)&event.m_FIFO_Accel_X);
				IIM42652_Format_Data(BIG_ENDIAN_DATA, (uint8_t *)&imu->m_FIFOData[2+fifo_idx], (uint16_t *)&event.m_FIFO_Accel_Y);
				IIM42652_Format_Data(BIG_ENDIAN_DATA, (uint8_t *)&imu->m_FIFOData[4+fifo_idx], (uint16_t *)&event.m_FIFO_Accel_Z);

				fifo_idx += FIFO_ACCEL_DATA_SIZE;
			}

			if (header->bits.gyro_bit)
			{
				IIM42652_Format_Data(BIG_ENDIAN_DATA, (uint8_t *)&imu->m_FIFOData[0+fifo_idx], (uint16_t *)&event.m_FIFO_Gyro_X);
				IIM42652_Format_Data(BIG_ENDIAN_DATA, (uint8_t *)&imu->m_FIFOData[2+fifo_idx], (uint16_t *)&event.m_FIFO_Gyro_Y);
				IIM42652_Format_Data(BIG_ENDIAN_DATA, (uint8_t *)&imu->m_FIFOData[4+fifo_idx], (uint16_t *)&event.m_FIFO_Gyro_Z);

				fifo_idx += FIFO_GYRO_DATA_SIZE;
			}

			if ((header->bits.accel_bit) || (header->bits.gyro_bit))
			{
				event.m_FIFO_Temp = (int8_t)imu->m_FIFOData[0+fifo_idx];

				fifo_idx += FIFO_TEMP_DATA_SIZE;
			}

			if ((header->bits.timestamp_bit) || (header->bits.fsync_bit))
			{
				IIM42652_Format_Data(BIG_ENDIAN_DATA, (uint8_t *)&imu->m_FIFOData[0+fifo_idx], &event.m_FIFO_TimeStamp);

				fifo_idx += FIFO_TS_FSYNC_SIZE;
			}

			//Check valid value and add to fft buffer
			float32_t tempValue;
			if(event.m_FIFO_Accel_X != FIFO_INVALID_SAMPLE)
			{
				tempValue = IIM42652_GetAccelGravity(imu, event.m_FIFO_Accel_X);
				RINGBUFFER_PUSH(&AccXBuffer, (float32_t *)&tempValue);
			}
			if(event.m_FIFO_Accel_Y != FIFO_INVALID_SAMPLE)
			{
				tempValue = IIM42652_GetAccelGravity(imu, event.m_FIFO_Accel_Y);
				RINGBUFFER_PUSH(&AccYBuffer, (float32_t *)&tempValue);
			}
			if(event.m_FIFO_Accel_Z != FIFO_INVALID_SAMPLE)
			{
				tempValue = IIM42652_GetAccelGravity(imu, event.m_FIFO_Accel_Z);
				RINGBUFFER_PUSH(&AccZBuffer, (float32_t *)&tempValue);
			}
			if(event.m_FIFO_Gyro_X != FIFO_INVALID_SAMPLE)
			{
				tempValue = IIM42652_GetGyroRps(imu, event.m_FIFO_Gyro_X);
				RINGBUFFER_PUSH(&GyroXBuffer, (float32_t *)&tempValue);
			}
			if(event.m_FIFO_Gyro_Y != FIFO_INVALID_SAMPLE)
			{
				tempValue = IIM42652_GetGyroRps(imu, event.m_FIFO_Gyro_Y);
				RINGBUFFER_PUSH(&GyroYBuffer, (float32_t *)&tempValue);
			}
			if(event.m_FIFO_Gyro_Z != FIFO_INVALID_SAMPLE)
			{
				tempValue = IIM42652_GetGyroRps(imu, event.m_FIFO_Gyro_Z);
				RINGBUFFER_PUSH(&GyroZBuffer, (float32_t *)&tempValue);
			}

			if (imu->TelemetryON == 1)
			{

				if (decimatorCounter >= imu->decimationFactor)
				{
					IMU_Data_Buf_t imuPacketBuf, dummyPacket;
					imu_TO_buffer(imu, 1, &event, imuPacketBuf.buffer);
					if (!xQueueIsQueueFullFromISR(IMUDataOutputQueue))
					{
//						if (uxQueueMessagesWaitingFromISR(IMUDataOutputQueue) <= IMU_OUTPUT_DATA_QUEUE_SIZE - sizeof(IMU_Data_Buf_t))
//						{
//							xQueueSendFromISR(IMUDataOutputQueue, &imuPacketBuf.buffer, &xHigherPriorityTaskWoken);
//							xTaskNotifyFromISR(UsbTxTaskHandle, AC1_Notify_NewDataAvaiable, eSetBits, &xHigherPriorityTaskWoken);
//							decimatorCounter = 0;
//						}
						xQueueSendFromISR(IMUDataOutputQueue, &imuPacketBuf.buffer, &xHigherPriorityTaskWoken);
						xTaskNotifyFromISR(UsbTxTaskHandle, AC1_Notify_NewDataAvaiable, eSetBits, &xHigherPriorityTaskWoken);
						decimatorCounter = 0;
					}
					else
					{
						xQueueReceiveFromISR(IMUDataOutputQueue, &dummyPacket.buffer, &xHigherPriorityTaskWoken);
						xQueueSendFromISR(IMUDataOutputQueue, &imuPacketBuf.buffer, &xHigherPriorityTaskWoken);
						xTaskNotifyFromISR(UsbTxTaskHandle, AC1_Notify_NewDataAvaiable, eSetBits, &xHigherPriorityTaskWoken);
						decimatorCounter = 0;
					}
				}
				else
				{
					decimatorCounter++;
				}
			}
		}
	}
	imu->m_FIFO_reading = 0;
	if (osKernelRunning() == 1)
	{
		osSemaphoreRelease(imu->m_spiSemaphoreHandle);
	}
}

/* INT_STATUS Register bits
 * 	MSB		6			5			4			3			2			1			LSB
 *	RES		UI_FSYNC	PLL_RDY		RESET_DONE	DATA_RDY	FIFO_THS	FIFO_FULL	AGC_RDY
 */
uint8_t IIM42652_GetIntStatus(IIM42652_t *imu, uint8_t *value)
{
	return IIM42652_ReadRegister(imu, REG_INT_STATUS, IMU_RegBank_0, value);
}

/* INT_STATUS2 Register bits
 * 	MSB:4	3			2			1			LSB
 *	RES		SMD_INT		WOM_Z_INT 	WOM_Y_INT	WOM_X_INT
 */
uint8_t IIM42652_GetIntStatus2(IIM42652_t *imu, uint8_t *value)
{
	return IIM42652_ReadRegister(imu, REG_INT_STATUS2, IMU_RegBank_0, value);
}

/* INT_STATUS3 Register bits
 * 	MSB:6	5			4				3			2		1		LSB
 *	RES		STEP_DET	STEP_CNT_OVF 	TILT_DET 	RES		FF_DET	TAP_DET
 */
uint8_t IIM42652_GetIntStatus3(IIM42652_t *imu, uint8_t *value)
{
	return IIM42652_ReadRegister(imu, REG_INT_STATUS3, IMU_RegBank_0, value);
}

uint8_t IIM42652_ReadIMUData(IIM42652_t *imu)
{
	uint8_t rxBuf[14];
	int16_t tempVal;

	if (IIM42652_ReadMultiRegisters(imu, REG_TEMP_DATA1_UI, IMU_RegBank_0, rxBuf, 14) == HAL_OK)
	{
		/* Temperature */
		tempVal = (int16_t) ((rxBuf[0] << 8) | rxBuf[1]);
		imu->Temp_C =  IIM42652_GetTempCelsius16(tempVal);

		/* Acceleration X */
		tempVal = (int16_t) ((rxBuf[2] << 8) | rxBuf[3]);
		imu->Acc_mps2[0] = (float) tempVal / imu->AccelScaleFactor;

		/* Acceleration Y */
		tempVal = (int16_t) ((rxBuf[4] << 8) | rxBuf[5]);
		imu->Acc_mps2[1] = (float) tempVal / imu->AccelScaleFactor;

		/* Acceleration Z */
		tempVal = (int16_t) ((rxBuf[6] << 8) | rxBuf[7]);
		imu->Acc_mps2[2] = (float) tempVal / imu->AccelScaleFactor;

		/* Rotation X */
		tempVal = (int16_t) ((rxBuf[8] << 8) | rxBuf[9]);
		imu->Gyro_rps[0] = (float) tempVal / imu->GyroScaleFactorDegree;

		/* Rotation Y */
		tempVal = (int16_t) ((rxBuf[10] << 8) | rxBuf[11]);
		imu->Gyro_rps[1] = (float) tempVal / imu->GyroScaleFactorDegree;

		/* Rotation Z */
		tempVal = (int16_t) ((rxBuf[12] << 8) | rxBuf[13]);
		imu->Gyro_rps[2] = (float) tempVal / imu->GyroScaleFactorDegree;

		return HAL_OK;
	}

	return HAL_ERROR;
}

uint8_t IIM42652_ReadGyroDMA(IIM42652_t *imu)
{
	return HAL_ERROR;
}
uint8_t IIM42652_ReadAccelDMA(IIM42652_t *imu)
{
	return HAL_ERROR;
}
void IIM42652_ReadGyroDMA_Complete(IIM42652_t *imu)
{

}
void IIM42652_ReadAccelDMA_Complete(IIM42652_t *imu)
{

}

float IIM42652_GetTempCelsius16(int16_t TemperatureHalfWord)
{
	return (float)(TemperatureHalfWord / 132.48) + 25;
}
float IIM42652_GetTempCelsius8(int8_t TemperatureByte)
{
	return (float)(TemperatureByte / 2.07) + 25;
}
float IIM42652_GetAccelGravity(IIM42652_t *imu, int16_t AccelerationByte)
{
	return (float) AccelerationByte / imu->AccelScaleFactor;
}
float IIM42652_GetGyroRps(IIM42652_t *imu, int16_t RotationByte)
{
	return (float) RotationByte / imu->GyroScaleFactorDegree;
}
uint16_t IIM42652_GetTimestampUS(uint16_t FIFOTimestampHalfWord)
{
	return (uint16_t) FIFOTimestampHalfWord * 32/30;
}
void IIM42652_Format_Data(const uint8_t endian, const uint8_t *in, uint16_t *out)
{
	if(endian == BIG_ENDIAN_DATA)
		*out = (in[0] << 8) | in[1];
	else
		*out = (in[1] << 8) | in[0];
}
uint16_t IIM42652_RegToAAFBW(uint8_t AAF_BS, uint8_t AAF_DELT, uint8_t AAF_DELTSQR)
{
	;
}
void imu_TO_buffer(IIM42652_t *imu, uint8_t decimationFactor, IIM42652_FIFO_Packet16_t* imuData, uint8_t* buffer)
{
	uint16_t tempVal;
	uint8_t dataBuff[16];

	/* Accel X */
	dataBuff[0] = (imuData->m_FIFO_Accel_X >> 8) & 0x00ff;
	dataBuff[1] = imuData->m_FIFO_Accel_X & 0x00ff;
	/* Accel Y */
	dataBuff[2] = (imuData->m_FIFO_Accel_Y >> 8) & 0x00ff;
	dataBuff[3] = imuData->m_FIFO_Accel_Y & 0x00ff;
	/* Accel Z */
	dataBuff[4] = (imuData->m_FIFO_Accel_Z >> 8) & 0x00ff;
	dataBuff[5] = imuData->m_FIFO_Accel_Z & 0x00ff;
	/* Gyro X */
	dataBuff[6] = (imuData->m_FIFO_Gyro_X >> 8) & 0x00ff;
	dataBuff[7] = imuData->m_FIFO_Gyro_X & 0x00ff;
	/* Gyro Y */
	dataBuff[8] = (imuData->m_FIFO_Gyro_Y >> 8) & 0x00ff;
	dataBuff[9] = imuData->m_FIFO_Gyro_Y & 0x00ff;
	/* Gyro Z */
	dataBuff[10] = (imuData->m_FIFO_Gyro_Z >> 8) & 0x00ff;
	dataBuff[11] = imuData->m_FIFO_Gyro_Z & 0x00ff;
	/* Temperature */
	dataBuff[12] = imuData->m_FIFO_Temp;
	/* Timestamp */
	tempVal = imuData->m_FIFO_TimeStamp * decimationFactor;
	dataBuff[13] = (tempVal >> 8) & 0x00ff;
	dataBuff[14] = tempVal & 0x00ff;

	/* APEX Interrupt state */
	dataBuff[15] = imu->m_INT2State;

	memcpy(buffer, dataBuff, sizeof(dataBuff));
}

void buffer_TO_imu(IIM42652_FIFO_Packet16_t* imuData, uint8_t* buffer)
{
	/* Header */
	imuData->m_FIFO_Header = 0;
	/* Accel X */
	imuData->m_FIFO_Accel_X = *buffer;
	imuData->m_FIFO_Accel_X = (imuData->m_FIFO_Accel_X << 8) | *(buffer+1);
	/* Accel Y */
	imuData->m_FIFO_Accel_Y = *(buffer+2);
	imuData->m_FIFO_Accel_Y = (imuData->m_FIFO_Accel_Y << 8) | *(buffer+3);
	/* Accel Z */
	imuData->m_FIFO_Accel_Z = *(buffer+4);
	imuData->m_FIFO_Accel_Z = (imuData->m_FIFO_Accel_Z << 8) | *(buffer+5);
	/* Gyro X */
	imuData->m_FIFO_Gyro_X = *(buffer+6);
	imuData->m_FIFO_Gyro_X = (imuData->m_FIFO_Gyro_X << 8) | *(buffer+7);
	/* Gyro Y */
	imuData->m_FIFO_Gyro_Y = *(buffer+8);
	imuData->m_FIFO_Gyro_Y = (imuData->m_FIFO_Gyro_Y << 8) | *(buffer+9);
	/* Gyro Z */
	imuData->m_FIFO_Gyro_Z = *(buffer+10);
	imuData->m_FIFO_Gyro_Z = (imuData->m_FIFO_Gyro_Z << 8) | *(buffer+11);
	/* Temperature */
	imuData->m_FIFO_Temp = *(buffer+12);
	/* Timestamp */
	imuData->m_FIFO_TimeStamp = *(buffer+13);
	imuData->m_FIFO_TimeStamp = (imuData->m_FIFO_TimeStamp << 8) | *(buffer+14);
}

void imu_TO_vector(IIM42652_FIFO_Packet16_t* imuData, vc_vector* vector)
{
	uint8_t tempByte;

	/* Accel X */
	//MSB
	tempByte = (imuData->m_FIFO_Accel_X >> 8) & 0x00ff;
	vc_vector_push_back(vector, &tempByte);
	//LBS
	tempByte = imuData->m_FIFO_Accel_X & 0x00ff;
	vc_vector_push_back(vector, &tempByte);

	/* Accel Y */
	tempByte = (imuData->m_FIFO_Accel_Y >> 8) & 0x00ff;
	vc_vector_push_back(vector, &tempByte);
	//LBS
	tempByte = imuData->m_FIFO_Accel_Y & 0x00ff;
	vc_vector_push_back(vector, &tempByte);

	/* Accel Z */
	tempByte = (imuData->m_FIFO_Accel_Z >> 8) & 0x00ff;
	vc_vector_push_back(vector, &tempByte);
	//LBS
	tempByte = imuData->m_FIFO_Accel_Z & 0x00ff;
	vc_vector_push_back(vector, &tempByte);

	/* Gyro X */
	//MSB
	tempByte = (imuData->m_FIFO_Gyro_X >> 8) & 0x00ff;
	vc_vector_push_back(vector, &tempByte);
	//LBS
	tempByte = imuData->m_FIFO_Gyro_X & 0x00ff;
	vc_vector_push_back(vector, &tempByte);

	/* Gyro Y */
	//MSB
	tempByte = (imuData->m_FIFO_Gyro_Y >> 8) & 0x00ff;
	vc_vector_push_back(vector, &tempByte);
	//LBS
	tempByte = imuData->m_FIFO_Gyro_Y & 0x00ff;
	vc_vector_push_back(vector, &tempByte);

	/* Gyro Z */
	//MSB
	tempByte = (imuData->m_FIFO_Gyro_Z >> 8) & 0x00ff;
	vc_vector_push_back(vector, &tempByte);
	//LBS
	tempByte = imuData->m_FIFO_Gyro_Z & 0x00ff;
	vc_vector_push_back(vector, &tempByte);

	/* Temperature */
	tempByte = imuData->m_FIFO_Temp & 0x00ff;
	vc_vector_push_back(vector, &tempByte);

	/* Timestamp */
	//MSB
	tempByte = (imuData->m_FIFO_TimeStamp >> 8) & 0x00ff;
	vc_vector_push_back(vector, &tempByte);
	//LBS
	tempByte = imuData->m_FIFO_TimeStamp & 0x00ff;
	vc_vector_push_back(vector, &tempByte);
}

size_t imuUsbPacketsCreator(IIM42652_t *imu, IMU_Data_Buf_t* bufferIn, size_t bufferInLen, uint8_t* encBufferOut, size_t bufferOutLen)
{
//	uint8_t imuBuffer[sizeof(IMU_Data_Buf_t) + COMMAND_DATA_SIZE + 2*sizeof(float)];
	uint8_t imuBuffer[sizeof(IMU_Data_Buf_t) + COMMAND_DATA_SIZE];
	cobs_encode_result res;

	if (bufferOutLen < (sizeof(imuBuffer) + ENCODEING_EXTRA_SIZE))
	{
		/* Encoded data buff doesen't have enough space for START and END byte */
		return 0;
	}

	memset(imuBuffer, 0x00, sizeof(imuBuffer));

	/* Add command type byte */
	imuBuffer[0] = commandIMUdata;

	/* Add scale factors */
//	memcpy(&imuBuffer[1], &imu->AccelScaleFactor, sizeof(float));
//	memcpy(&imuBuffer[5], &imu->GyroScaleFactorDegree, sizeof(float));

	/* Add imu data */
//	memcpy(&imuBuffer[9], &bufferIn->buffer, bufferInLen);
	memcpy(&imuBuffer[1], &bufferIn->buffer, bufferInLen);

	/* Encode the packet */
	res = omdEncodeBuf(encBufferOut, bufferOutLen, imuBuffer, sizeof(imuBuffer));

	if (res.status == COBS_ENCODE_OK)
	{
		return (res.out_len + 1);
	}
	else
	{
		return 0;
	}
}
