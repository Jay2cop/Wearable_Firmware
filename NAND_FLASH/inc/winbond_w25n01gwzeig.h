#ifndef WinbondW25N_H
#define WinbondW25N_H

#include "winbond_w25n01gwzeig_HAL.h"

#define W25M_DIE_SELECT           0xC2

#define W25N_RESET                0xFF
#define W25N_JEDEC_ID             0x9F
#define W25N_READ_STATUS_REG      0x05
#define W25N_WRITE_STATUS_REG     0x01
#define W25N_WRITE_ENABLE         0x06
#define W25N_WRITE_DISABLE        0x04
#define W25N_BB_MANAGE            0xA1
#define W25N_READ_BBM             0xA5
#define W25N_LAST_ECC_FAIL        0xA9
#define W25N_BLOCK_ERASE          0xD8
#define W25N_PROG_DATA_LOAD       0x02
#define W25N_RAND_PROG_DATA_LOAD  0x84
#define W25N_PROG_EXECUTE         0x10
#define W25N_PAGE_DATA_READ       0x13
#define W25N_READ                 0x03
#define W25N_FAST_READ            0x0B

#define W25N_PROT_REG             0xA0
#define W25N_CONFIG_REG           0xB0
#define W25N_STAT_REG             0xC0

#define WINBOND_MAN_ID            0xEF
#define W25N01GV_DEV_ID           0xAA21

#define W25M02GV_DEV_ID           0xAB21

#define W25N01GV_MAX_PAGE         65535
#define W25N_MAX_COLUMN           2112
#define W25M02GV_MAX_PAGE         131071
#define W25M02GV_MAX_DIES         2
#define PAGES_PER_BLOCK			  64

void NAND_Reset(void);
int NAND_WriteEnable(void);
int NAND_BlockErase(uint32_t pageAdd);
int NAND_BulkErase(void);
int NAND_LoadProgData(uint16_t columnAdd, char* buf, uint32_t dataLen);
int NAND_ProgramExecute(uint32_t pageAdd);
int NAND_Read(uint32_t pageAdd, uint16_t columnAdd, char* buf, uint32_t dataLen);
int NAND_CheckWIP(void);
int NAND_PageDataRead(uint32_t pageAdd);

#endif
