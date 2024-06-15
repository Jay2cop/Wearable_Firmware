#include "winbond_w25n01gwzeig_HAL.h"
#include "winbond_w25n01gwzeig.h"
#include <stdio.h>


void NAND_Reset(void) {
    while(NAND_CheckWIP()) {
    }

    // Enable Reset
    HAL_SendData(0x66, 0xFFFFFFFF, 0, NULL, 0);
    HAL_delay_ms(1);

    // Reset Device
    HAL_SendData(0x99, 0xFFFFFFFF, 0, NULL, 0);
    HAL_delay_ms(1);

    // Read the status register to check the Write Enable Latch (WEL) bit
	uint8_t statusRegValue[1];
	HAL_ReceiveData(W25N_READ_STATUS_REG, 0, 0, statusRegValue, 1);

	// Interpret the status register value
	if (statusRegValue[0] & 0x02) {
		printf("Write Enable Latch (WEL) is set.\n");
	} else {
		printf("Write Enable Latch (WEL) is not set.\n");
	}
}

int NAND_WriteEnable(void) {
    // Send the Write Enable command
    HAL_SendData(0x06, 0xFFFFFFFF, 0, NULL, 0);

    HAL_delay_ms(1);

    // Read the status register to check the Write Enable Latch (WEL) bit
    uint8_t statusRegValue[1];
    HAL_ReceiveData(W25N_READ_STATUS_REG, 0, 0, statusRegValue, 1);

    while(NAND_CheckWIP()) {}
    // Interpret the status register value
    if (statusRegValue[0] & 0x02) { // WEL bit is in the second bit position
        printf("Write Enable is set.\r\n");
        return 0;
    } else {
        printf("Write Enable is not set.\r\n");
        return -1;
    }
}

int NAND_BlockErase(uint32_t pageAdd) {
	HAL_SendData(W25N_BLOCK_ERASE, pageAdd, 0, NULL, 0);
    return 0;
}

int NAND_BulkErase(void) {
    const uint32_t totalBlocks = W25N01GV_MAX_PAGE / PAGES_PER_BLOCK;
    for(uint32_t block = 0; block < totalBlocks; ++block) {
        uint32_t pageAddress = block * PAGES_PER_BLOCK;
        if(NAND_BlockErase(pageAddress) != 0) {
            return 1;
        }
        while(NAND_CheckWIP()) {
        }
    }
    return 0;
}


int NAND_LoadProgData(uint16_t columnAdd, char* buf, uint32_t dataLen) {
	while(NAND_CheckWIP()) {
	    }
    uint32_t address = columnAdd;
    HAL_SendData(W25N_PROG_DATA_LOAD, address, 0,(uint8_t*)buf, dataLen);
    return 0;
}

int NAND_ProgramExecute(uint32_t pageAdd) {
    HAL_SendData(W25N_PROG_EXECUTE, pageAdd, 0, NULL, 0);

    while(NAND_CheckWIP()) {
    }

    return 0;
}

int NAND_PageDataRead(uint32_t pageAdd) {
    HAL_SendData(W25N_PAGE_DATA_READ, pageAdd, 0, NULL, 0);
    while(NAND_CheckWIP()) {
    }
    return 0;
}

int NAND_Read(uint32_t pageAdd, uint16_t columnAdd, char* buf, uint32_t dataLen) {
    // Initiate a page data read before attempting to read
    NAND_PageDataRead(pageAdd);

    uint32_t fullAddress = (pageAdd << 16) | columnAdd;
    HAL_ReceiveData(W25N_READ, fullAddress, 1, buf, dataLen);
    return 0;
}

int NAND_CheckWIP(void) {
    uint8_t statusValue[1];
    HAL_ReceiveData(W25N_READ_STATUS_REG, 0, 0, statusValue, 1);

    // Check the WIP bit in the status register value
    if (statusValue[0] & 0x01) {
        //printf("NAND is busy.\n");
        return 1; // NAND is busy
    } else {
        //printf("NAND is ready.\n");
        return 0; // NAND is not busy
    }
}
