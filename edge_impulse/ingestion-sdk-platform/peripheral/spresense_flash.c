
/*
 * Copyright (c) 2022 Edge Impulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an "AS
 * IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "spresense_flash.h"

#define MX25R_BLOCK32_SIZE		(MX25R_SECTOR_SIZE * 8) /**!< 32K Block 	 */
#define MX25R_BLOCK64_SIZE		(MX25R_BLOCK32_SIZE * 2)/**!< 64K Block	 	 */
#define MX25R_CHIP_SIZE			(MX25R_BLOCK64_SIZE * 128)/**!< 64Mb on chip */

/** MX25R Register defines */
#define MX25R_PP				0x02		/**!< Program page				 */
#define MX25R_READ				0x03		/**!< Read data command			 */
#define MX25R_RDSR				0x05		/**!< Status Register 			 */
#define MX25R_WREN				0x06		/**!< Write enable bit 			 */
#define MX25R_SE				0x20		/**!< Sector erase				 */
#define MX25R_PGM				0x7A		/**!< Resume programming		 */
#define MX25R_BE				0xD8		/**!< Block (64K) erase			 */

/** MX25R Status register bit defines */

#define MX25R_STAT_SRWD			(1<<7)		/**!< Status reg write protect	 */
#define MX25R_STAT_QE			(1<<6)		/**!< Quad enable 				 */
#define MX25R_STAT_BP3			(1<<5)		/**!< Level of protect block	 */
#define MX25R_STAT_BP2			(1<<4)		/**!< Level of protect block	 */
#define MX25R_STAT_BP1			(1<<3)		/**!< Level of protect block	 */
#define MX25R_STAT_BP0			(1<<2)		/**!< Level of protect block	 */
#define MX25R_STAT_WEL			(1<<1)		/**!< Write enable latch		 */
#define MX25R_STAT_WIP			(1<<0)		/**!< Write in progress bit		 */

/* Private function prototypes --------------------------------------------- */
#if (SAMPLE_MEMORY == SERIAL_FLASH)
static uint8_t flash_status_register(void);
static void flash_write_enable(void);
static uint32_t flash_wait_while_busy(void);
#endif

/* Private variables ------------------------------------------------------- */
#if (SAMPLE_MEMORY == RAM)
static uint8_t ram_memory[SIZE_RAM_BUFFER];
#endif

/** 32-bit align write buffer size */
#define WORD_ALIGN(a) ((a & 0x3) ? (a & ~0x3) + 0x4 : a)



#if (SAMPLE_MEMORY == SERIAL_FLASH)

uint32_t flash_write(uint32_t address, const uint8_t *buffer, uint32_t bufferSize)
{
    int stat;
    int retry = MX25R_RETRY;
    int offset = 0;

    if (flash_wait_while_busy() == 0)
        return SONY_SPRESENSE_FS_CMD_WRITE_ERROR;

    do {
        uint32_t n_bytes;

        retry = MX25R_RETRY;
        do {
            flash_write_enable();
            stat = flash_status_register();
        } while (!(stat & MX25R_STAT_WEL) && --retry);

        if (!retry) {
            return SONY_SPRESENSE_FS_CMD_WRITE_ERROR;
        }

        if (bufferSize > MX25R_PAGE_SIZE) {
            n_bytes = MX25R_PAGE_SIZE;
            bufferSize -= MX25R_PAGE_SIZE;
        }
        else {
            n_bytes = bufferSize;
            bufferSize = 0;
        }

        /* If write overflows page, split up in 2 writes */
        if ((((address + offset) & 0xFF) + n_bytes) > MX25R_PAGE_SIZE) {

            int diff = MX25R_PAGE_SIZE - ((address + offset) & 0xFF);

            flash_program_page(address + offset, ((uint8_t *)buffer + offset), diff);

            if (flash_wait_while_busy() == 0)
                return SONY_SPRESENSE_FS_CMD_WRITE_ERROR;

            // retry = MX25R_RETRY;
            // do {
            // 	flash_write_enable();
            // 	stat = flash_status_register();
            // }while(!(stat & MX25R_STAT_WEL) && --retry);

            // if(!retry) {
            // 	return SONY_SPRESENSE_FS_CMD_WRITE_ERROR;
            // }

            /* Update index pointers */
            n_bytes -= diff;
            offset += diff;

            bufferSize += n_bytes;
        }

        else {
            flash_program_page(address + offset, ((uint8_t *)buffer + offset), n_bytes);

            offset += n_bytes;
        }

        // flash_program_page(address + offset, ((uint8_t *)buffer + offset), n_bytes);

        if (flash_wait_while_busy() == 0)
            return SONY_SPRESENSE_FS_CMD_WRITE_ERROR;

        // offset += MX25R_PAGE_SIZE;

    } while (bufferSize);

    return SONY_SPRESENSE_FS_CMD_OK;
}

/**
 * @brief      Erase mulitple flash sectors
 *
 * @param[in]  startAddress  The start address
 * @param[in]  nSectors      The sectors
 *
 * @return     ei_sony_spresense_ret_t
 */
uint32_t flash_erase_sectors(uint32_t startAddress, uint32_t nSectors)
{
    int stat;
    int retry = MX25R_RETRY;
    uint32_t curSector = 0;

    do {
        if (flash_wait_while_busy() == 0) {
            return SONY_SPRESENSE_FS_CMD_ERASE_ERROR;
        }

        do {
            flash_write_enable();
            stat = flash_status_register();
        } while (!(stat & MX25R_STAT_WEL) && --retry);

        flash_erase_sector(startAddress + (MX25R_SECTOR_SIZE * curSector));

    } while (++curSector < nSectors);

    return SONY_SPRESENSE_FS_CMD_OK;
}

/**
 * @brief      Send read command and get data over SPI
 *
 * @param[in]  byteAddress  The byte address
 * @param      buffer       The buffer
 * @param[in]  readBytes    The read bytes
 *
 * @return     Sony_Spresense status
 */
uint32_t flash_read_data(uint32_t byteAddress, uint8_t *buffer, uint32_t readBytes)
{
    uint8_t spiTransfer[4];
    //TODO
    return 0;
}

/**
 * @brief      Read status register and check WIP (write in progress)
 * @return     n retries, if 0 device is hanging
 */
static uint32_t flash_wait_while_busy(void)
{
    uint32_t stat;
    uint32_t retry = MX25R_RETRY;

    stat = flash_status_register();

    while ((stat & MX25R_STAT_WIP) && --retry) {

        stat = flash_status_register();
    }

    return retry;
}

/**
 * @brief      Send the write enable command over SPI
 */
static void flash_write_enable(void)
{
    //uint8_t spiTransfer[1];

    //spiTransfer[0] = MX25R_WREN;
    //Sony_SpresenseCspSpiTransferPoll(
    //    SONY_SPRESENSE_SPI_NUM,
    //    &spiTransfer[0],
    //    1,
    //    &spiTransfer[0],
    //    0,
    //    SONY_SPRESENSE_BSP_SPIFLASH_CS_NUM,
    //    eSpiSequenceFirstLast);
}

/**
 * @brief      Send a read status register frame over SPI
 *
 * @return     Chip status register
 */
static uint8_t flash_status_register(void)
{
    uint8_t spiTransfer[2];
    // TODO
    return spiTransfer[0];
}
#endif