#ifndef SPRESENSE_FLASH_H_
#define SPRESENSE_FLASH_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SERIAL_FLASH 0
#define MICRO_SD     1
#define RAM          2

#define SAMPLE_MEMORY MICRO_SD

#define SONY_SPRESENSE_FS_BLOCK_ERASE_TIME_MS		90

/** Sony_Spresense fs return values */
typedef enum
{
	SONY_SPRESENSE_FS_CMD_OK = 0,						/**!< All is well				 */
	SONY_SPRESENSE_FS_CMD_NOT_INIT,					/**!< FS is not initialised		 */
	SONY_SPRESENSE_FS_CMD_READ_ERROR,					/**!< Error occured during read  */
	SONY_SPRESENSE_FS_CMD_WRITE_ERROR,					/**!< Error occured during write */
	SONY_SPRESENSE_FS_CMD_ERASE_ERROR,					/**!< Erase error occured		 */    
	SONY_SPRESENSE_FS_CMD_NULL_POINTER,				/**!< Null pointer parsed		 */
    SONY_SPRESENSE_FS_CMD_FILE_ERROR,
} ei_sony_spresense_ret_t;

#if (SAMPLE_MEMORY == SERIAL_FLASH)
extern uint32_t flash_write(uint32_t address, const uint8_t *buffer, uint32_t bufferSize);
extern uint32_t flash_read_data(uint32_t byteAddress, uint8_t *buffer, uint32_t readBytes);
extern uint32_t flash_erase_sectors(uint32_t startAddress, uint32_t nSectors);
#endif

#ifdef __cplusplus
}
#endif 

#endif
