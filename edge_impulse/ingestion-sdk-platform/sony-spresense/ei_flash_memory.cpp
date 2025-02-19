/* The Clear BSD License
 *
 * Copyright (c) 2025 EdgeImpulse Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 *   * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


/* Includes ---------------------------------------------------------------- */
#include "ei_flash_memory.h"
#include "spresense_flash.h"
#include "ei_board_ctrl.h"
#include "edge-impulse-sdk/porting/ei_classifier_porting.h"

#define SIZE_RAM_BUFFER 0x10000//(0x20000)
#define RAM_BLOCK_SIZE	1024
#define RAM_N_BLOCKS    (SIZE_RAM_BUFFER / RAM_BLOCK_SIZE)

#define FILE_NAME_CONFIG    "config.bin"
#define FILE_NAME_SAMPLE    "sample.bin"
#define FILE_MAX_SIZE       0x200000
#define FILE_BLOCK_SIZE     1024
#define FILE_N_BLOCKS       (FILE_MAX_SIZE / FILE_BLOCK_SIZE)

/** Number of retries for SPI Flash */
#define MX25R_RETRY		10000

/** SPI Flash Memory layout */
#define MX25R_PAGE_SIZE			256			/**!< Page program size			 */
#define MX25R_SECTOR_SIZE		4096		/**!< Size of sector			 */


EiFlashMemory::EiFlashMemory(uint32_t config_size):
    EiDeviceMemory(config_size, SONY_SPRESENSE_FS_BLOCK_ERASE_TIME_MS, FILE_MAX_SIZE, FILE_BLOCK_SIZE)
{
    sd_card_inserted = true;
}

/**
 * @brief 
 * 
 * @param sample_data 
 * @param address 
 * @param sample_data_size 
 * @return uint32_t 
 */
uint32_t EiFlashMemory::read_sample_data(uint8_t *sample_data, uint32_t address, uint32_t sample_data_size)
{
    return this->read_data(sample_data, address, sample_data_size);
}

/**
 * @brief 
 * 
 * @param sample_data 
 * @param address 
 * @param sample_data_size 
 * @return uint32_t 
 */
uint32_t EiFlashMemory::write_sample_data(const uint8_t *sample_data, uint32_t address, uint32_t sample_data_size)
{
    return this->write_data(sample_data, address, sample_data_size);
}

/**
 * @brief 
 * 
 * @param address 
 * @param num_bytes 
 * @return uint32_t 
 */
uint32_t EiFlashMemory::erase_sample_data(uint32_t address, uint32_t num_bytes)
{
    return this->erase_data(address, num_bytes);
}

uint32_t EiFlashMemory::read_data(uint8_t *data, uint32_t address, uint32_t num_bytes)
{
#if (SAMPLE_MEMORY == RAM)
    if ((address_offset + n_read_bytes) > SIZE_RAM_BUFFER) {
        return SONY_SPRESENSE_FS_CMD_READ_ERROR;
    }
    else if (sample_buffer == 0) {
        return SONY_SPRESENSE_FS_CMD_NULL_POINTER;
    }

    for (int i = 0; i < num_bytes; i++) {
        *((char *)sample_buffer + i) = ram_memory[address + i];
    }
    return SONY_SPRESENSE_FS_CMD_OK;

#elif (SAMPLE_MEMORY == SERIAL_FLASH)

    int retVal = SONY_SPRESENSE_FS_CMD_OK;

    if (flash_wait_while_busy() == 0) {
        retVal = SONY_SPRESENSE_FS_CMD_READ_ERROR;
    }
    else {
        if (flash_read_data(
                MX25R_BLOCK64_SIZE + address,
                (uint8_t *)data,
                num_bytes) != 0) {
            retVal = SONY_SPRESENSE_FS_CMD_READ_ERROR;
        }
    }

    return retVal;

#elif (SAMPLE_MEMORY == MICRO_SD)    
    int retVal = 0;
    retVal = spresense_readFromFile(FILE_NAME_SAMPLE, (uint8_t *)data, address, num_bytes);        

    return retVal;
#endif
    return 0;
}

void EiFlashMemory::rewind(void)
{
    spresense_rewind();
}


uint32_t EiFlashMemory::write_data(const uint8_t *data, uint32_t address, uint32_t num_bytes)
{
    #if (SAMPLE_MEMORY == RAM)
    uint32_t n_word_samples = WORD_ALIGN(num_bytes);

    if ((address + n_word_samples) > SIZE_RAM_BUFFER) {
        return SONY_SPRESENSE_FS_CMD_WRITE_ERROR;
    }
    else if (sample_buffer == 0) {
        return SONY_SPRESENSE_FS_CMD_NULL_POINTER;
    }

    for (int i = 0; i < n_word_samples; i++) {
        ram_memory[address + i] = *((char *)data + i);
    }
    return SONY_SPRESENSE_FS_CMD_OK;

#elif (SAMPLE_MEMORY == SERIAL_FLASH)
    uint32_t n_word_samples = WORD_ALIGN(num_bytes);

    return flash_write(
        MX25R_BLOCK64_SIZE + address,
        (const uint8_t *)data,
        n_word_samples);

#elif (SAMPLE_MEMORY == MICRO_SD)
    int retVal = num_bytes;

    retVal = spresense_writeToFile((const char *)FILE_NAME_SAMPLE, (const uint8_t *)data, num_bytes);
#endif

    return retVal;
}

uint32_t EiFlashMemory::erase_data(uint32_t address, uint32_t num_bytes)
{
#if (SAMPLE_MEMORY == RAM)
    return num_bytes;
#elif (SAMPLE_MEMORY == SERIAL_FLASH)
    return flash_erase_sectors(MX25R_BLOCK64_SIZE, end_address / MX25R_SECTOR_SIZE);
#elif (SAMPLE_MEMORY == MICRO_SD)
    //return (int)spresense_openFile((const char *)FILE_NAME_SAMPLE, true);
    int retval = 0;
    openSampleFile(true);
    retval = spresense_deleteFile((const char *)FILE_NAME_SAMPLE);
    close_sample_file();
    return (int)retval;
#endif
}

/**
 * @brief 
 * 
 * @param write 
 * @return true 
 * @return false 
 */
bool EiFlashMemory::openSampleFile(bool write)
{
#if (SAMPLE_MEMORY == MICRO_SD)
    return spresense_openFile((const char *)FILE_NAME_SAMPLE, write);
#else
    return false;
#endif
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool EiFlashMemory::close_sample_file(void)
{
#if (SAMPLE_MEMORY == MICRO_SD)
    return spresense_closeFile((const char *)FILE_NAME_SAMPLE);
#else
    return false;
#endif
}

bool EiFlashMemory::save_config(const uint8_t *config, uint32_t config_size)
{
    ei_sony_spresense_ret_t retVal = SONY_SPRESENSE_FS_CMD_OK;

    if (config == NULL) {
        return false;
    }

#if (SAMPLE_MEMORY == RAM)

    return retVal;

#elif (SAMPLE_MEMORY == SERIAL_FLASH)

    retVal = flash_erase_sectors(0, 1);

    return (retVal == SONY_SPRESENSE_FS_CMD_OK) ? flash_write(0, (const uint8_t *)config, config_size)
                                     : retVal;

#elif (SAMPLE_MEMORY == MICRO_SD)

    if (sd_card_inserted == false) {
        retVal = SONY_SPRESENSE_FS_CMD_OK;
    }
    else if (spresense_openFile((const char *)FILE_NAME_CONFIG, true) != 0) {
        retVal = SONY_SPRESENSE_FS_CMD_FILE_ERROR;
    }
    else if (spresense_writeToFile((const char *)FILE_NAME_CONFIG, (const uint8_t *)config, config_size) != config_size) {
        retVal = SONY_SPRESENSE_FS_CMD_WRITE_ERROR;
    }

    else if (spresense_closeFile((const char *)FILE_NAME_CONFIG) == false) {    
        retVal = SONY_SPRESENSE_FS_CMD_FILE_ERROR;
    }

    return (retVal == SONY_SPRESENSE_FS_CMD_OK);
#endif
}

bool EiFlashMemory::load_config(uint8_t *config, uint32_t config_size)
{
    int retVal = SONY_SPRESENSE_FS_CMD_OK;

    if (config == NULL) {
        return false;
    }

#if (SAMPLE_MEMORY == RAM)

    return (retVal == SONY_SPRESENSE_FS_CMD_OK);

#elif (SAMPLE_MEMORY == SERIAL_FLASH)

    if (flash_wait_while_busy() == 0) {
        retVal = SONY_SPRESENSE_FS_CMD_READ_ERROR;
    }
    else {
        if (flash_read_data(0, (uint8_t *)config, config_size) != 0) {
            retVal = SONY_SPRESENSE_FS_CMD_READ_ERROR;
        }
    }

    return (retVal == SONY_SPRESENSE_FS_CMD_OK);

#elif (SAMPLE_MEMORY == MICRO_SD)

    if (spresense_openFile((const char *)FILE_NAME_CONFIG, false) != 0) {

        /* Try to create and write to the config file */
        if (spresense_openFile((const char *)FILE_NAME_CONFIG, true) != 0) {
            /* Missing SD card, return OK so default config can be loaded */
            ei_printf("File cannot open %s is the SD card inserted?\r\n", FILE_NAME_CONFIG);
            sd_card_inserted = false;
        }
        else {
            spresense_writeToFile((const char *)FILE_NAME_CONFIG, (const uint8_t *)config, config_size);
            spresense_closeFile((const char *)FILE_NAME_CONFIG);
        }

        return SONY_SPRESENSE_FS_CMD_OK;
    }

    if (spresense_readFromFile(FILE_NAME_CONFIG, (uint8_t *)config, 0, config_size) < 0) {
        retVal = SONY_SPRESENSE_FS_CMD_READ_ERROR;
    }
    else if (spresense_closeFile((const char *)FILE_NAME_CONFIG) == false) {
        retVal = SONY_SPRESENSE_FS_CMD_FILE_ERROR;
    }
   return (retVal == SONY_SPRESENSE_FS_CMD_OK);
#endif
}
