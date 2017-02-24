#include "sdcard.h"
// WORK IN PROGRESS!!!

/** The interrupt handler ............
  *
  * @return Void.
***/
void SPI1_IRQHandler(void) {
	if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET) {
		SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_RXNE);
	}
}

/** The interrupt handler ............
  *
  * @return Void.
***/
void DMA2_Stream3_IRQHandler(void) {
	if (DMA_GetITStatus(DMA2_Stream3, DMA_IT_TCIF3) == SET) {
		DMA_ClearITPendingBit(DMA2_Stream3, DMA_IT_TCIF3);

		DMA_Cmd(DMA2_Stream3, DISABLE);
	}
}

/** Enable clocks, configure GPIO pin A4 as Output (Chip Select), configure pin A5 (SCK),
  * pin A6 (MISO), and pin A7 (MOSI) as Alternate Function (SPI1),
  * configure DMA2_Stream3_Channel3, and enable transfer complete interrupt for DMA and SPI.
  *
  * @return The status of configuration.
***/
int8_t sdcard_config() {
	GPIO_InitTypeDef  	GPIO_InitStruct;
	SPI_InitTypeDef 	SPI_InitStruct;
	DMA_InitTypeDef		DMA_InitStruct;
	NVIC_InitTypeDef  	NVIC_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	SPI_I2S_DeInit(SPI1);
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(SPI1, &SPI_InitStruct);
	SPI_CalculateCRC(SPI1, DISABLE);

	NVIC_InitStruct.NVIC_IRQChannel = DMA2_Stream3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = SPI1_IRQn;
	NVIC_Init(&NVIC_InitStruct);

	// DMA TX CHANNEL (WRITE)
	DMA_DeInit(DMA2_Stream3);
	DMA_InitStruct.DMA_Channel = DMA_Channel_3;
	DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
	DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)sdcard_dma_buffer_rx;
	DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStruct.DMA_BufferSize =  SDCARD_BUFFER_SIZE;
	DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable; /// FIGURE OUT HOW FATFS/SPI WORKS FIRST
	DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStruct.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_Init(DMA2_Stream3, &DMA_InitStruct);

	DMA_ITConfig(DMA2_Stream3, DMA_IT_TC, ENABLE);
	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);

	SPI_Cmd(SPI1, ENABLE);

	delay(20);

	return 0;
}

/** Initialize variables and prepare to begin normal operation.
  *
  * @return The status of initialization.
***/
int8_t sdcard_init() {
	sdcard_stat = STA_NOINIT;

	for (uint32_t i = 0; i < SDCARD_BUFFER_SIZE; i++)
		sdcard_dma_buffer_rx[i] = 0;

	for (uint8_t i = 0; i < 4; i++)
		sdcard_path[i] = 0x00;

	sdcard_read_count = 0;
	sdcard_write_count = 0;

	sdcard_result = FR_NOT_READY;

	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	for (uint8_t i = 0; i < 10; i++)
		SPI_I2S_SendData(SPI1, 0xFF);

	return 0;
}

//	EXTERN FUNCTION IMPLEMENTATIONS		//

/** FatFS implementation for initializing the disk.
  *
  * @return The disk status.
***/
DSTATUS sdcard_disk_initialize() {
	sdcard_stat = STA_NOINIT;

	return 0;
}

/** FaFS implementation for getting the disk status.
  *
  * @return The disk status.
***/
DSTATUS sdcard_disk_status() {
	return sdcard_stat;
}

/** FatFS implementation for reading from the disk.
  *
  * @param *buff:	Pointer to the location to store the data that is read.
  * @param sector:	The sector of the disk to read.
  * @param count:	The amount of data to read.
  *
  * @return The result status of the disk read.
***/
DRESULT sdcard_disk_read(BYTE* buff, DWORD sector, UINT count) {
	if (1 || buff || sector || count) //////////// GET RID OF UNUSED VARIABLE COMPILE ERRORS
		return RES_PARERR; // PLACEHOLDER
}

/** FatFS implementation for writing to the disk.
  *
  * @param *buff:	Pointer to the data to write to the disk.
  * @param sector:	The sector of the disk to write to.
  * @param count:	The amount of data to write.
  *
  * @return The result status of the disk write.
***/
DRESULT sdcard_disk_write(const BYTE* buff, DWORD sector, UINT count) {
	if (1 || buff || sector || count) //////////// GET RID OF UNUSED VARIABLE COMPILE ERRORS
		return RES_PARERR; // PLACEHOLDER
}

/** FatFS implementation for issuing IO control commands.
  *
  * @param cmd:		The IO command to issue.
  * @param *buff:	Pointer to the buffer to store command data.
  *
  * @return The result status of the issued IO control command.
***/
DRESULT sdcard_disk_ioctl(BYTE cmd, void* buff) {
	DRESULT res = RES_ERROR;

	if (buff) {} //////////// GET RID OF UNUSED VARIABLE COMPILE ERRORS

	switch (cmd) {
	case CTRL_SYNC:
		break;
	case GET_SECTOR_COUNT:
		break;
	case GET_SECTOR_SIZE:
		break;
	case GET_BLOCK_SIZE:
		break;
	case CTRL_TRIM:
		break;
	}

	return res;
}

/** FatFS implementation for getting the time
  * (integer formatted as time since January 1, 1980 00:00:00).
  *
  * @return The time in integer format.
***/
DWORD get_fattime(void) { // (For now set to October 1, 2016, use GPS when available)
	return ((DWORD)(2016 - 1980) << 25) | ((DWORD)10 << 21) | ((DWORD)1 << 16) |
            ((DWORD)0 << 11) | ((DWORD)0 << 5) | ((DWORD)0 >> 1); // NOTE: Only even seconds are stored so divide by two ( >> 1)
}
