#ifndef _SDCARD_H
#define _SDCARD_H


#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_spi.h"
#include "misc.h"

#include "ff.h"
#include "diskio.h"

#include "state.h"
#include "other_stuff.h"


#define FATFS_USE_SDIO					0
#define BLOCK_SIZE						512
#define SDCARD_BUFFER_SIZE				(67 * 100) // Estimated packet size is 67 so just testing here for now...


static volatile DSTATUS		sdcard_stat;
uint8_t						sdcard_dma_buffer_rx[SDCARD_BUFFER_SIZE];
FATFS 						sdcard_fs;
FIL 						sdcard_file;
char 						sdcard_path[4];
FRESULT 					sdcard_result;
uint32_t 					sdcard_read_count;
uint32_t					sdcard_write_count;


int8_t			sdcard_config();
int8_t			sdcard_init();
DSTATUS 		sdcard_disk_initialize ();
DSTATUS 		sdcard_disk_status ();
DRESULT 		sdcard_disk_read(BYTE* buff, DWORD sector, UINT count);
DRESULT 		sdcard_disk_write(const BYTE* buff, DWORD sector, UINT count);
DRESULT 		sdcard_disk_ioctl(BYTE cmd, void* buff);
DWORD			get_fattime(void);


#endif
