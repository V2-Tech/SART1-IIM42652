/*
 * USBCommProtocol_def.h
 *
 *  Created on: Feb 18, 2022
 *      Author: Valerio.Mazzoni
 */

#ifndef OMD_USBCOMMPROTOCOL_DEF_H_
#define OMD_USBCOMMPROTOCOL_DEF_H_

//--------------------------------------
// --------- DEFINED VALUES ------------
//--------------------------------------
#define COMMAND_DATA_SIZE	( 1 )
#define ENCODEING_EXTRA_SIZE	( 2 )
//--------------------------------------
// -------- CLASS VARIABLES ------------
//--------------------------------------

typedef enum  {
	commandSaveUserData = 0xF0,
	commandLoadUserData = 0xF1,
	commandSetTelemetry = 0xFA,
	commandGetTelemetry = 0xFB,
	commandSetConfigMode = 0xFC,
	commandGetConfigMode = 0xFD,
	commandDeviceReset = 0xFE,

	commandAck = 0xAC,

	commandIMUdata = 0xD0,
	commandDOState = 0xD1,
	commandString = 0xD2,
	commandPingPong = 0xD3,

	commandSetAccODR = 0x10,
	commandGetAccODR = 0x11,
	commandSetAccFSR = 0x12,
	commandGetAccFSR = 0x13,
	commandSetAccBW = 0x14,
	commandGetAccBW = 0x15,

	commandSetGyroODR = 0x20,
	commandGetGyroODR = 0x21,
	commandSetGyroFSR = 0x22,
	commandGetGyroFSR = 0x23,
	commandSetGyroBW = 0x24,
	commandGetGyroBW = 0x25,

	commandSetWomXTh = 0x30,
	commandGetWomXTh = 0x31,
	commandSetWomYTh = 0x32,
	commandGetWomYTh = 0x33,
	commandSetWomZTh = 0x34,
	commandGetWomZTh = 0x35,
	commandSetWomIntMode = 0x36,
	commandGetWomIntMode = 0x37,
	commandSetWomMode = 0x38,
	commandGetWomMode = 0x39,
	commandSetSmdMode = 0x3A,
	commandGetSmdMode = 0x3B,
} OMDProtocolCommand;

typedef struct {
	uint8_t command;
	uint16_t value;
} OMD_Command_Packet_t; //If necessary, also update COMMAND_PACKET_LENGTH_BYTES.
/* !!!!! PAY ATTENTION !!!!!
 *
 * sizeof(OMD_Command_Packet_t) return value is 4 instead of 3
 *
 * !!!!! PAY ATTENTION !!!!!
 */

typedef enum
{
    OMD_COMM_ACK_OK	= 0x10,
	OMD_COMM_ACK_ERR = 0x01,
} OMD_ack_values;


#endif /* OMD_USBCOMMPROTOCOL_DEF_H_ */
