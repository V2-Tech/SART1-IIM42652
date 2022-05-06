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

/*
 * Set high a single bit of a word
 * @param  	*pXXXXXXXX pointer to the word you want to change the bit's state.
 *
 * @param	bitPosition Bit's position you want to change (count start from right, LSB bit in 0 position).
 */

#include "common_def.h"
#include "common.h"
#include "usb_device.h"

//--------------------------------------
//------------ VARIABLES ---------------
//--------------------------------------
AC1_StatusWord_t AC1_StatusWord;
static uint8_t transmitBuf[APP_TX_DATA_SIZE];

//--------------------------------------
//------------ FUNCTIONS ---------------
//--------------------------------------
void PrepareJumpToBootloader(void)
{
	RTC_HandleTypeDef RtcHandle;
	RtcHandle.Instance = RTC;

	HAL_PCD_DevDisconnect( hUsbDeviceFS.pData );
	USBD_Stop(&hUsbDeviceFS);
	USBD_DeInit(&hUsbDeviceFS);

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

	HAL_Delay(2000);


	// Write Back Up Register 1 Data
	__HAL_RTC_WRITEPROTECTION_DISABLE(&RtcHandle);
	HAL_PWR_EnableBkUpAccess();
	// Writes a data in a RTC Backup data Register 1
	HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR1, MAGIC_VALUE);
	HAL_PWR_DisableBkUpAccess();

	//Do System Reset
	NVIC_SystemReset();
}

static void JumpToBootloader(void) {
    void (*SysMemBootJump)(void);

    /**
     * Step: Set system memory address.
     *
     *       For STM32F429, system memory is on 0x1FFF 0000
     *       For other families, check AN2606 document table 110 with descriptions of memory addresses
     */
    volatile uint32_t addr = 0x1FFF0000;

    /**
     * Step: Disable RCC, set it to default (after reset) settings
     *       Internal clock, no PLL, etc.
     */
    HAL_RCC_DeInit();
    HAL_DeInit(); // add by ctien

    /**
     * Step: Disable systick timer and reset it to default values
     */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    /**
     * Step: Disable all interrupts
     */
    __disable_irq(); // changed by ctien

    /**
     * Step: Remap system memory to address 0x0000 0000 in address space
     *       For each family registers may be different.
     *       Check reference manual for each family.
     *
     *       For STM32F4xx, MEMRMP register in SYSCFG is used (bits[1:0])
     *       For STM32F0xx, CFGR1 register in SYSCFG is used (bits[1:0])
     *       For others, check family reference manual
     */
    //Remap by hand... {
//    SYSCFG->MEMRMP = 0x01;

    //} ...or if you use HAL drivers
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();    //Call HAL macro to do this for you

    /**
     * Step: Set jump memory location for system memory
     *       Use address with 4 bytes offset which specifies jump location where program starts
     */
    SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));

    /**
     * Step: Set main stack pointer.
     *       This step must be done last otherwise local variables in this function
     *       don't have proper value since stack pointer is located on different position
     *
     *       Set direct address location which specifies stack pointer in SRAM location
     */
    __set_MSP(*(uint32_t *)addr);

    /**
     * Step: Actually call our function to jump to set location
     *       This will start system memory execution
     */
    SysMemBootJump();
}
void CheckBootloaderJump() {
	RTC_HandleTypeDef RtcHandle;
	RtcHandle.Instance = RTC;
	//Check if we did a system reset and want to jump into bootloader
	if (HAL_RTCEx_BKUPRead(&RtcHandle, RTC_BKP_DR1) == MAGIC_VALUE)
	{
		// Write Back Up Register 1 Data
		__HAL_RTC_WRITEPROTECTION_DISABLE(&RtcHandle);
		HAL_PWR_EnableBkUpAccess();
		// Writes a data in a RTC Backup data Register 1
		HAL_RTCEx_BKUPWrite(&RtcHandle, RTC_BKP_DR1, 0x0);
		HAL_PWR_DisableBkUpAccess();
		JumpToBootloader();
		while(1);
	}
}

uint8_t BitSetW(uint32_t *pWord, uint8_t bitPosition)
{
	if (bitPosition < 32)
	{
		*pWord |= 1UL << bitPosition;
		return 1;
	}
	else
	{
		return 0;
	}
}
uint8_t BitSetHW(uint16_t *pHalfWord, uint8_t bitPosition)
{
	if (bitPosition < 16)
	{
		*pHalfWord |= 1UL << bitPosition;
		return 1;
	}
	else
	{
		return 0;
	}
}
uint8_t BitSetB(uint8_t *pByte, uint8_t bitPosition)
{
	if (bitPosition < 8)
	{
		*pByte |= 1UL << bitPosition;
		return 1;
	}
	else
	{
		return 0;
	}
}

/*
 * 	Set high a single bit of a word:
 * 	That will clear the nth bit of word.
 * 	We invert the bit string with the bitwise NOT operator (~),
 * 	then AND it with the word.
 * @param  	*pXXXXXXXX pointer to the word you want to change the bit's state.
 *
 * @param	bitPosition Bit's position you want to change (count start from right, LSB bit in 0 position).
 */

uint8_t BitResetW(uint32_t *pWord, uint8_t bitPosition)
{
	if (bitPosition < 32)
	{
		*pWord &= ~(1UL << bitPosition);
		return 1;
	}
	else
	{
		return 0;
	}
}
uint8_t BitResetHW(uint16_t *pHalfWord, uint8_t bitPosition)
{
	if (bitPosition < 16)
	{
		*pHalfWord &= ~(1UL << bitPosition);
		return 1;
	}
	else
	{
		return 0;
	}
}
uint8_t BitResetB(uint8_t *pByte, uint8_t bitPosition)
{
	if (bitPosition < 8)
	{
		*pByte &= ~(1UL << bitPosition);
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t BitCheckW(uint32_t Word, uint8_t bitPosition)
{
	if(bitPosition > 31)
	{
		return -1;
	}
	return (Word >> bitPosition) & 1U;
}
uint8_t BitCheckHW(uint16_t HalfWord, uint8_t bitPosition)
{
	if(bitPosition > 15)
	{
		return -1;
	}
	return (HalfWord >> bitPosition) & 1U;
}
uint8_t BitCheckB(uint8_t Byte, uint8_t bitPosition)
{
	if(bitPosition > 7)
	{
		return -1;
	}
	return (Byte >> bitPosition) & 1U;
}

uint8_t _SAVE_AC1_SETTINGS(AC1_Config_t actConfig)
{
	int numofwords = (sizeof(actConfig)/4)+((sizeof(actConfig)%4)!=0);

	return Flash_Write_Data(USER_DATA_SECTOR_START_ADDRESS, (uint32_t *)&actConfig, numofwords);
}
uint8_t _LOAD_AC1_SETTINGS(AC1_Config_t *actConfig)
{
	AC1_Config_t dummyConfig; //Useful only for calculating the number of needed words

	int numofwords = (sizeof(dummyConfig)/4)+((sizeof(dummyConfig)%4)!=0);

	Flash_Read_Data(USER_DATA_SECTOR_START_ADDRESS, (uint32_t *)actConfig, numofwords);
}
/* Read about:
 *
 * https://stackoverflow.com/questions/28503808/allocating-memory-in-flash-for-user-data-stm32f4-hal
 *
 */
//uint8_t _SAVE_AC1_SETTINGS(uint8_t* pFlashStorage, AC1_Config_t *actConfig, size_t nBytes)
//{
//	HAL_FLASH_Unlock();
//
//	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR );
//
//	FLASH_Erase_Sector(FLASH_SECTOR_0, VOLTAGE_RANGE_3);
//
//	for ( int8_t index = 0; index < nBytes; index++)
//	{
//		HAL_FLASH_Program(TYPEPROGRAM_WORD, pFlashStorage+index, actConfig+index);
//	}
//
//	HAL_FLASH_Lock();
//}

void _USB_SEND_OPT(uint8_t* byteBuf, size_t byteBufLen)
{
	static size_t transmitBufSize = 0;

	if ((transmitBufSize + byteBufLen) < APP_TX_DATA_SIZE)
	{
		memcpy(&transmitBuf[transmitBufSize], byteBuf, byteBufLen);
		transmitBufSize += byteBufLen;
	}
	else
	{
		CDC_Transmit_FS(transmitBuf, transmitBufSize);
		memcpy(&transmitBuf[0], byteBuf, byteBufLen);
		transmitBufSize = byteBufLen;
	}
}
