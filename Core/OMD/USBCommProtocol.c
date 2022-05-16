/*
 * USBCommProtocol.c
 *
 *  	Created on: Oct 25, 2021
 *      Author: Valerio Mazzoni
 *      Company: OMD
 *
 *      Version: 1.0.0
 *
 *      Changelog:
 *      	- 11/02/2021 - 1.0.0: start of the source code writing.
 */

/** OMD (COBS) encode data to buffer
	@param data Pointer to input data to encode
	@param length Number of bytes to encode
	@param buffer Pointer to encoded output buffer
	@return Encoded buffer length in bytes
	@note Does not output delimiter byte
*/
#include "USBCommProtocol.h"
#include "usbd_cdc_if.h"
//--------------------------------------
// ------------- VARIABLES -------------
//--------------------------------------
OMD_Protocol_t OMDCommProtocol;

extern IIM42652_t IMU;
extern AC1_StatusWord_t AC1_StatusWord;
//--------------------------------------
// ------------- METHODS ---------------
//--------------------------------------
void OMDProtocolInit(OMD_Protocol_t *ProtocolStruct)
{
	osMutexDef(TxQueueMutex);
	ProtocolStruct->m_TxQueueMutexHandle = osMutexCreate(osMutex(TxQueueMutex));
	configASSERT(ProtocolStruct->m_TxQueueMutexHandle);

	memset(ProtocolStruct->m_encodedRxBuf,0x00,APP_RX_DATA_SIZE);
	memset(ProtocolStruct->m_encodedTxBuf,0x00,APP_TX_DATA_SIZE);

	RINGBUFFER_CLEAR(&ProtocolStruct->m_rxBuffer);
}

void GiveRxData(OMD_Protocol_t *ProtocolStruct, vc_vector* rxDataVector)
{
	vc_vector* packet = vc_vector_create(0, sizeof(uint8_t), NULL);

	// Push new data into received buffer
	MoveRxDataInBuffer(ProtocolStruct, rxDataVector, packet);

	// Packet found: decode and execute command
	while (!vc_vector_empty(packet)) {
		vc_vector* decodedData = vc_vector_create(2, sizeof(uint8_t), NULL);

		omdDecode(packet, decodedData);

		executeCommand(ProtocolStruct, decodedData);

		MoveRxDataInBuffer(ProtocolStruct, rxDataVector, packet);
	}
}

void MoveRxDataInBuffer(OMD_Protocol_t *ProtocolStruct, vc_vector* rxDataVector, vc_vector* packetVector)
{
	// Clear any existing data from packet
	vc_vector_clear(packetVector);

	// Pop bytes from front of received data vector
	while (!vc_vector_empty(rxDataVector)) {
		uint8_t byteOfData = *((uint8_t *)vc_vector_front(rxDataVector));

		vc_vector_erase(rxDataVector, 0);

		RINGBUFFER_PUSH(&ProtocolStruct->m_rxBuffer, &byteOfData);

		// Looking for 0x00 byte in received data
		if (byteOfData == 0x00)
		{
			/*
			 * Found end-of-packet!
			 * Move everything from the start to byteOfData from rxData
			 * into a new packet
			 */
			uint16_t buffSize = RINGBUFFER_SIZE(&ProtocolStruct->m_rxBuffer);

			for (uint16_t it = 0; it < buffSize; ++it)
			{
				vc_vector_push_back(packetVector, &ProtocolStruct->m_rxBuffer.buffer[it]);
			}

			RINGBUFFER_CLEAR(&ProtocolStruct->m_rxBuffer);
			return;
		}
	}
}

void executeCommand(OMD_Protocol_t *ProtocolStruct, vc_vector* command)
{
	/* Extract command */
	uint8_t commandType;
	uint16_t value16;
	uint8_t value8;

	commandType = *(uint8_t *)vc_vector_front(command);

	switch (commandType)
	{
	case commandSaveUserData:
		/* Extract value */
		value8 = *(uint8_t *)vc_vector_at(command,1);

		if (value8 >= 1)
		{
			saveUserConfig();
			// Send ACK post-execution into main task
		}
		else
		{
			ackCommand(ProtocolStruct, command, OMD_COMM_ACK_ERR);
		}
		break;
	case commandDeviceReset:
		/* Extract value */
		value8 = *(uint8_t *)vc_vector_at(command,1);

		if (value8 >= 1)
		{
			IIM42652_SoftReset(&IMU);
			// Send ACK post-execution into main task
		}
		else
		{
			ackCommand(ProtocolStruct, command, OMD_COMM_ACK_ERR);
		}

		break;
	case commandSetConfigMode:
		/* Extract value */
		value8 = *(uint8_t *)vc_vector_at(command,1);

		if (value8 > 0)
		{
			setConfigMode(ENABLE);
			// Send ACK post-execution into main task
		}
		else if(value8 == 0)
		{
			setConfigMode(DISABLE);
			// Send ACK post-execution into main task
		}
		else
		{
			ackCommand(ProtocolStruct, command, OMD_COMM_ACK_ERR);
		}
		break;
	case commandGetConfigMode:
		/* Get requested value */
		value8 = getConfigMode();
		ackCommand(ProtocolStruct, command, value8);
		break;
	case commandSetTelemetry:
		/* Extract value */
		value8 = *(uint8_t *)vc_vector_at(command,1);
		setTelemetryEn(value8);
		ackCommand(ProtocolStruct, command, OMD_COMM_ACK_OK);
		break;
	case commandGetTelemetry:
		/* Get requested value */
		value8 = getTelemetryEn();
		ackCommand(ProtocolStruct, command, value8);
		break;
	case commandSetAccODR:
		/* Extract value */
		value8 = *(uint8_t *)vc_vector_at(command,1);
		setAccelODR(&IMU, value8);
		ackCommand(ProtocolStruct, command, OMD_COMM_ACK_OK);
		break;
	case commandGetAccODR:
		/* Get requested value */
		if (getAccelODR(&IMU, &value8) == HAL_OK)
		{
			ackCommand(ProtocolStruct, command, value8);
		}
		break;
	case commandSetAccFSR:
		/* Extract value */
		value8 = *(uint8_t *)vc_vector_at(command,1);
		setAccelFSR(&IMU, value8);
		ackCommand(ProtocolStruct, command, OMD_COMM_ACK_OK);
		break;
	case commandGetAccFSR:
		/* Get requested value */
		if (getAccelFSR(&IMU, &value8) == HAL_OK)
		{
			ackCommand(ProtocolStruct, command, value8);
		}
		break;
	case commandSetGyroODR:
		/* Extract value */
		value8 = *(uint8_t *)vc_vector_at(command,1);
		setGyroODR(&IMU, value8);
		ackCommand(ProtocolStruct, command, OMD_COMM_ACK_OK);
		break;
	case commandGetGyroODR:
		/* Get requested value */
		if (getGyroODR(&IMU, &value8) == HAL_OK)
		{
			ackCommand(ProtocolStruct, command, value8);
		}
		break;
	case commandSetGyroFSR:
		/* Extract value */
		value8 = *(uint8_t *)vc_vector_at(command,1);
		setGyroFSR(&IMU, value8);
		ackCommand(ProtocolStruct, command, OMD_COMM_ACK_OK);
		break;
	case commandGetGyroFSR:
		/* Get requested value */
		if (getGyroFSR(&IMU, &value8) == HAL_OK)
		{
			ackCommand(ProtocolStruct, command, value8);
		}
		break;
	case commandSetAccBW:
		/* Extract value */
		value8 = *(uint8_t *)vc_vector_at(command,1);
		setAccelBW(&IMU, value8);
		ackCommand(ProtocolStruct, command, OMD_COMM_ACK_OK);
		break;
	case commandGetAccBW:
		/* Get requested value */
		if (getAccelBW(&IMU, &value8) == HAL_OK)
		{
			ackCommand(ProtocolStruct, command, value8);
		}
		break;
	case commandSetGyroBW:
		/* Extract value */
		value8 = *(uint8_t *)vc_vector_at(command,1);
		setGyroBW(&IMU, value8);
		ackCommand(ProtocolStruct, command, OMD_COMM_ACK_OK);
		break;
	case commandGetGyroBW:
		/* Get requested value */
		if (getGyroBW(&IMU, &value8) == HAL_OK)
		{
			ackCommand(ProtocolStruct, command, value8);
		}
		break;
	case commandSetWomXTh:
		/* Extract value */
		value8 = *(uint8_t *)vc_vector_at(command,1);
		setWomXth(&IMU, value8);
		ackCommand(ProtocolStruct, command, OMD_COMM_ACK_OK);
		break;
	case commandGetWomXTh:
		/* Get requested value */
		if (getWomXth(&IMU, &value8) == HAL_OK)
		{
			ackCommand(ProtocolStruct, command, value8);
		}
		break;
	case commandSetWomYTh:
		/* Extract value */
		value8 = *(uint8_t *)vc_vector_at(command,1);
		setWomYth(&IMU, value8);
		ackCommand(ProtocolStruct, command, OMD_COMM_ACK_OK);
		break;
	case commandGetWomYTh:
		/* Get requested value */
		if (getWomYth(&IMU, &value8) == HAL_OK)
		{
			ackCommand(ProtocolStruct, command, value8);
		}
		break;
	case commandSetWomZTh:
		/* Extract value */
		value8 = *(uint8_t *)vc_vector_at(command,1);
		setWomZth(&IMU, value8);
		ackCommand(ProtocolStruct, command, OMD_COMM_ACK_OK);
		break;
	case commandGetWomZTh:
		/* Get requested value */
		if (getWomZth(&IMU, &value8) == HAL_OK)
		{
			ackCommand(ProtocolStruct, command, value8);
		}
		break;
	case commandSetWomIntMode:
		/* Extract value */
		value8 = *(uint8_t *)vc_vector_at(command,1);
		setWomIntMode(&IMU, value8);
		ackCommand(ProtocolStruct, command, OMD_COMM_ACK_OK);
		break;
	case commandGetWomIntMode:
		/* Get requested value */
		if (getWomIntMode(&IMU, &value8) == HAL_OK)
		{
			ackCommand(ProtocolStruct, command, value8);
		}
		break;
	case commandSetWomMode:
		/* Extract value */
		value8 = *(uint8_t *)vc_vector_at(command,1);
		setWomMode(&IMU, value8);
		ackCommand(ProtocolStruct, command, OMD_COMM_ACK_OK);
		break;
	case commandGetWomMode:
		/* Get requested value */
		if (getWomMode(&IMU, &value8) == HAL_OK)
		{
			ackCommand(ProtocolStruct, command, value8);
		}
		break;
	case commandSetSmdMode:
		/* Extract value */
		value8 = *(uint8_t *)vc_vector_at(command,1);
		setSmdMode(&IMU, value8);
		ackCommand(ProtocolStruct, command, OMD_COMM_ACK_OK);
		break;
	case commandGetSmdMode:
		/* Get requested value */
		if (getSmdMode(&IMU, &value8) == HAL_OK)
		{
			ackCommand(ProtocolStruct, command, value8);
		}
		break;
	case commandPingPong:
		/* Get IMU chip ID */
		if (getPingPong(&IMU, &value8) == HAL_OK)
		{
			ackCommand(ProtocolStruct, command, value8);
		}
	default:

		break;
	}
}

void sendCommand(OMD_Protocol_t *ProtocolStruct, uint8_t command, vc_vector* value)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vc_vector* transmitData = vc_vector_create(3, sizeof(uint8_t), NULL);
	vc_vector* encodedTransmitData = vc_vector_create(3, sizeof(uint8_t), NULL);
	uint8_t commandByte = command;

	vc_vector_push_back(transmitData, &commandByte);

	for ( uint8_t i = 0; i < vc_vector_count(value); i++)
	{
		vc_vector_push_back(transmitData, (uint8_t *)vc_vector_front(value));
		vc_vector_erase(value, 0);
	}

	omdEncode(transmitData, encodedTransmitData);

	osMutexWait(ProtocolStruct->m_TxQueueMutexHandle, osWaitForever);
	for ( uint8_t i = 0; i < vc_vector_count(encodedTransmitData); i++)
	{
		xQueueGenericSend(usb_TX_Queue, (uint8_t *)vc_vector_at(encodedTransmitData,i), osWaitForever, queueSEND_TO_BACK);
	}
	osMutexRelease(ProtocolStruct->m_TxQueueMutexHandle);

	xTaskNotifyFromISR(UsbTxTaskHandle, AC1_Notify_NewDataAvaiable, eSetBits, &xHigherPriorityTaskWoken);
}
void sendCommandBuf(OMD_Protocol_t *ProtocolStruct, uint8_t command, uint8_t value)
{
	uint8_t commandBuffer[sizeof(command) + sizeof(value)];
	uint8_t encodedBuffer[sizeof(command) + sizeof(value) + ENCODEING_EXTRA_SIZE];
	cobs_encode_result res;

	memset(commandBuffer, 0x00, sizeof(commandBuffer));

	/* Add command type byte */
	commandBuffer[0] = command;

	/* Add command value byte */
	commandBuffer[1] = value;

	/* Encode the packet */
	res = omdEncodeBuf(encodedBuffer, sizeof(encodedBuffer), commandBuffer, sizeof(commandBuffer));

	if(res.status == COBS_ENCODE_OK)
	{
		osMutexWait(ProtocolStruct->m_TxQueueMutexHandle, osWaitForever);
		for ( uint8_t i = 0; i < sizeof(encodedBuffer)/sizeof(encodedBuffer[0]); i++)
		{
			xQueueGenericSend(usb_TX_Queue, &encodedBuffer[i], osWaitForever, queueSEND_TO_BACK);
		}
		osMutexRelease(ProtocolStruct->m_TxQueueMutexHandle);
		xTaskGenericNotify(UsbTxTaskHandle, AC1_Notify_NewDataAvaiable, eSetBits, NULL);
	}
}
void sendCommandFromISR(OMD_Protocol_t *ProtocolStruct, uint8_t command, vc_vector* value)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vc_vector* transmitData = vc_vector_create(3, sizeof(uint8_t), NULL);
	vc_vector* encodedTransmitData = vc_vector_create(3, sizeof(uint8_t), NULL);
	uint8_t commandByte = command;

	vc_vector_push_back(transmitData, &commandByte);

	for ( uint8_t i = 0; i < vc_vector_count(value); i++)
	{
		vc_vector_push_back(transmitData, (uint8_t *)vc_vector_front(value));
		vc_vector_erase(value, 0);
	}

	omdEncode(transmitData, encodedTransmitData);

	osMutexWait(ProtocolStruct->m_TxQueueMutexHandle, osWaitForever);
	for ( uint8_t i = 0; i < vc_vector_count(encodedTransmitData); i++)
	{
		xQueueGenericSendFromISR(usb_TX_Queue, (uint8_t *)vc_vector_at(encodedTransmitData,i), &xHigherPriorityTaskWoken, queueSEND_TO_BACK);
		xTaskNotifyFromISR(UsbTxTaskHandle, AC1_Notify_NewDataAvaiable, eSetBits, &xHigherPriorityTaskWoken);
	}
	osMutexRelease(ProtocolStruct->m_TxQueueMutexHandle);
}

void ackCommand(OMD_Protocol_t *ProtocolStruct, vc_vector* command, uint8_t result)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vc_vector* transmitData = vc_vector_create(3, sizeof(uint8_t), NULL);
	vc_vector* encodedTransmitData = vc_vector_create(3, sizeof(uint8_t), NULL);
	uint8_t ackCommand = commandAck;

	//Create ACK packet
	vc_vector_push_back(transmitData, &ackCommand);
	vc_vector_push_back(transmitData, (uint8_t *)vc_vector_front(command));
	vc_vector_push_back(transmitData, &result);

	//Clear command input vector so as to prevent bugs in case of multiple ackCommand call in the same variable scope.
	vc_vector_clear(command);

	omdEncode(transmitData, encodedTransmitData);

	//Send packet to usb tx task
	osMutexWait(ProtocolStruct->m_TxQueueMutexHandle, osWaitForever);
	for ( uint8_t i = 0; i < vc_vector_count(encodedTransmitData); i++)
	{
		xQueueGenericSend(usb_TX_Queue, (uint8_t *)vc_vector_at(encodedTransmitData,i), osWaitForever, queueSEND_TO_BACK);
	}
	osMutexRelease(ProtocolStruct->m_TxQueueMutexHandle);


	//Notify usb tx task about new available packet
	xTaskNotifyFromISR(UsbTxTaskHandle, AC1_Notify_NewDataAvaiable, eSetBits, &xHigherPriorityTaskWoken);
}
void ackCommandBuf(OMD_Protocol_t *ProtocolStruct, uint8_t command, uint8_t result)
{
	uint8_t commandBuffer[1 + sizeof(command) + sizeof(result)];
	uint8_t encodedBuffer[1 + sizeof(command) + sizeof(result) + ENCODEING_EXTRA_SIZE];
	cobs_encode_result res;

	memset(commandBuffer, 0x00, sizeof(commandBuffer));

	/* Add command type byte */
	commandBuffer[0] = commandAck;

	/* Add command value byte */
	commandBuffer[1] = command;

	/* Add command result value byte */
	commandBuffer[2] = result;

	/* Encode the packet */
	res = omdEncodeBuf(encodedBuffer, sizeof(encodedBuffer), commandBuffer, sizeof(commandBuffer));
	if(res.status == COBS_ENCODE_OK)
	{
		osMutexWait(ProtocolStruct->m_TxQueueMutexHandle, osWaitForever);
		for ( uint8_t i = 0; i < sizeof(encodedBuffer)/sizeof(encodedBuffer[0]); i++)
		{
			xQueueGenericSend(usb_TX_Queue, &encodedBuffer[i], osWaitForever, queueSEND_TO_BACK);
		}
		osMutexRelease(ProtocolStruct->m_TxQueueMutexHandle);
	}

	//Notify usb tx task about new available packet
	xTaskGenericNotify(UsbTxTaskHandle, AC1_Notify_NewDataAvaiable, eSetBits, NULL);
}

void saveUserConfig(void)
{
	xTaskNotify(MainTaskHandle, AC1_Notify_SaveConfig, eSetBits);
}

void setConfigMode(uint8_t EnableState)
{
	if (EnableState == 0)
	{
		__IMU_TURN_ON(&IMU);
		xTaskNotify(MainTaskHandle, AC1_Notify_AnalysisON, eSetBits);
	}
	else
	{
		__IMU_TURN_OFF(&IMU);
		xTaskNotify(MainTaskHandle, AC1_Notify_AnalysisOFF, eSetBits);
	}
}
uint8_t getConfigMode()
{
	return !AC1_StatusWord.IMUAnalysisON;
}
void setTelemetryEn(uint8_t EnableState)
{
//	xTaskNotify(MainTaskHandle, AC1_Notify_TelemetryEnChange, eSetBits);
	if (EnableState)
	{
		IMU.TelemetryON = 1;
	}
	else
	{
		IMU.TelemetryON = 0;
	}
}
uint8_t getTelemetryEn()
{
	return IMU.TelemetryON;
}

/* Setter functions:
 *
 * @param *imu 			Pointer to a IIM42652_t structure that contains
 *						the configuration information for the specified IMU device.
 * @param (uint8_t) 	Value to set into proper register
 *
 * @return Status of operation (0 = success, 1 = error)
 */

/* Getter functions:
 * @param *imu 		Pointer to a IIM42652_t structure that contains
 *					the configuration information for the specified IMU device.
 * @param (uint8_t) Actual requested value
 *
 * @return Status of operation (0 = success, 1 = error)
 */

uint8_t setAccelODR(IIM42652_t *imu, uint8_t AccelODR)
{
	uint8_t status = IIM42652_SetAccelODR(imu, AccelODR);
	if (status == HAL_OK)
	{
		imu->m_actUserConfig->OMD_CONFIG_ACC_ODR = AccelODR;
	}
	return status;
}
uint8_t getAccelODR(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_GetAccelODR(imu, value);
}
uint8_t setGyroODR(IIM42652_t *imu, uint8_t GyroODR)
{
	uint8_t status = IIM42652_SetGyroODR(imu, GyroODR);
	if (status == HAL_OK)
	{
		imu->m_actUserConfig->OMD_CONFIG_GYRO_ODR = GyroODR;
	}
	return status;
}
uint8_t getGyroODR(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_GetGyroODR(imu, value);
}
uint8_t setAccelFSR(IIM42652_t *imu, uint8_t AccelFSR)
{
	uint8_t status = IIM42652_SetAccelFSR(imu, AccelFSR);
	if (status == HAL_OK)
	{
		imu->m_actUserConfig->OMD_CONFIG_ACC_FSR = AccelFSR;
	}
	return status;
}
uint8_t getAccelFSR(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_GetAccelFSR(imu, value);
}
uint8_t setGyroFSR(IIM42652_t *imu, uint8_t GyroFSR)
{
	uint8_t status = IIM42652_SetGyroFSR(imu, GyroFSR);
	if (status == HAL_OK)
	{
		imu->m_actUserConfig->OMD_CONFIG_GYRO_FSR = GyroFSR;
	}
	return status;
}
uint8_t getGyroFSR(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_GetGyroFSR(imu, value);
}
uint8_t setAccelBW(IIM42652_t *imu, uint8_t AccelBW)
{
	uint8_t status = IIM42652_SetAccelBW(imu, AccelBW);
	if (status == HAL_OK)
	{
		imu->m_actUserConfig->OMD_CONFIG_ACC_FILT_BW = AccelBW;
	}
	return status;
}
uint8_t getAccelBW(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_GetAccelBW(imu, value);
}
uint8_t setGyroBW(IIM42652_t *imu, uint8_t GyroBW)
{
	uint8_t status = IIM42652_SetGyroBW(imu, GyroBW);
	if (status == HAL_OK)
	{
		imu->m_actUserConfig->OMD_CONFIG_GYRO_FILT_BW = GyroBW;
	}
	return status;
}
uint8_t getGyroBW(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_GetGyroBW(imu, value);
}
uint8_t setWomXth(IIM42652_t *imu, uint8_t threshold)
{
	uint8_t status = IIM42652_SetWomXTh(imu, threshold);
	if (status == HAL_OK)
	{
		imu->m_actUserConfig->OMD_CONFIG_WOM_X_TH = threshold;
	}
	return status;
}
uint8_t getWomXth(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_GetWomXTh(imu, value);
}
uint8_t setWomYth(IIM42652_t *imu, uint8_t threshold)
{
	uint8_t status = IIM42652_SetWomYTh(imu, threshold);
	if (status == HAL_OK)
	{
		imu->m_actUserConfig->OMD_CONFIG_WOM_Y_TH = threshold;
	}
	return status;
}
uint8_t getWomYth(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_GetWomYTh(imu, value);
}
uint8_t setWomZth(IIM42652_t *imu, uint8_t threshold)
{
	uint8_t status = IIM42652_SetWomZTh(imu, threshold);
	if (status == HAL_OK)
	{
		imu->m_actUserConfig->OMD_CONFIG_WOM_Z_TH = threshold;
	}
	return status;
}
uint8_t getWomZth(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_GetWomZTh(imu, value);
}
uint8_t setWomIntMode(IIM42652_t *imu, uint8_t mode)
{
	uint8_t status = IIM42652_SetWOMIntMode(imu, mode);
	if (status == HAL_OK)
	{
		imu->m_actUserConfig->OMD_CONFIG_WOM_INT_MODE = mode;
	}
	return status;
}
uint8_t getWomIntMode(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_GetWOMIntMode(imu, value);
}
uint8_t setWomMode(IIM42652_t *imu, uint8_t mode)
{
	uint8_t status = IIM42652_SetWOMMode(imu, mode);
	if (status == HAL_OK)
	{
		imu->m_actUserConfig->OMD_CONFIG_WOM_MODE = mode;
	}
	return status;
}
uint8_t getWomMode(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_GetWOMMode(imu, value);
}
uint8_t setSmdMode(IIM42652_t *imu, uint8_t mode)
{
	uint8_t status = IIM42652_SetSMDMode(imu, mode);
	if (status == HAL_OK)
	{
		imu->m_actUserConfig->OMD_CONFIG_SMD_MODE = mode;
	}
	return status;
}
uint8_t getSmdMode(IIM42652_t *imu, uint8_t* value)
{
	return IIM42652_GetSMDMode(imu, value);
}
uint8_t getPingPong(IIM42652_t *imu, uint8_t* chipID)
{
	return IIM42652_GetWhoIAm(imu, chipID);
}
