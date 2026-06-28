/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include "main.h"
#include "user_diskio.h"
extern SPI_HandleTypeDef hspi2;

/* Private typedef -----------------------------------------------------------*/
#define SD_CS_GPIO_Port GPIOB
#define SD_CS_Pin       GPIO_PIN_12

/* Private define ------------------------------------------------------------*/
#define SD_SECTOR_SIZE  512U

/* Private variables ---------------------------------------------------------*/
#define CMD0    (0x40 + 0)
#define CMD8    (0x40 + 8)
#define CMD17   (0x40 + 17)
#define CMD24   (0x40 + 24)
#define CMD55   (0x40 + 55)
#define CMD58   (0x40 + 58)
#define ACMD41  (0x40 + 41)

static uint8_t sd_type = 0;   /* 0 = none, 1 = SDSC, 2 = SDHC */

static void SD_Select(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

static void SD_Deselect(void)
{
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

static uint8_t SD_TxRx(uint8_t data)
{
    uint8_t rx = 0xFF;
    HAL_SPI_TransmitReceive(&hspi2, &data, &rx, 1, 100);
    return rx;
}

static void SD_SendDummyClocks(uint8_t n)
{
    while (n--) SD_TxRx(0xFF);
}

static uint8_t SD_WaitReady(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    do
    {
        if (SD_TxRx(0xFF) == 0xFF) return 1;
    } while ((HAL_GetTick() - t0) < timeout_ms);

    return 0;
}

static uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    uint8_t res;
    uint8_t frame[6];

    frame[0] = cmd;
    frame[1] = (uint8_t)(arg >> 24);
    frame[2] = (uint8_t)(arg >> 16);
    frame[3] = (uint8_t)(arg >> 8);
    frame[4] = (uint8_t)arg;
    frame[5] = crc;

    SD_Deselect();
    SD_TxRx(0xFF);
    SD_Select();

    if (!SD_WaitReady(500)) return 0xFF;

    HAL_SPI_Transmit(&hspi2, frame, 6, 100);

    for (uint8_t i = 0; i < 10; i++)
    {
        res = SD_TxRx(0xFF);
        if ((res & 0x80) == 0) return res;
    }

    return 0xFF;
}


static uint8_t SD_InitCard(void)
{
    uint8_t r1;
    uint8_t ocr[4];

    SD_Deselect();
    SD_SendDummyClocks(10);   /* >= 80 clocks */

    r1 = SD_SendCommand(CMD0, 0, 0x95);
    if (r1 != 0x01)
    {
        SD_Deselect();
        return 0;
    }

    r1 = SD_SendCommand(CMD8, 0x1AA, 0x87);
    if (r1 == 0x01)
    {
        for (int i = 0; i < 4; i++) ocr[i] = SD_TxRx(0xFF);

        uint32_t t0 = HAL_GetTick();
        do
        {
            SD_SendCommand(CMD55, 0, 0x01);
            r1 = SD_SendCommand(ACMD41, 0x40000000, 0x01);
        } while ((r1 != 0x00) && ((HAL_GetTick() - t0) < 1000));

        if (r1 != 0x00)
        {
            SD_Deselect();
            return 0;
        }

        r1 = SD_SendCommand(CMD58, 0, 0x01);
        if (r1 == 0x00)
        {
            for (int i = 0; i < 4; i++) ocr[i] = SD_TxRx(0xFF);
            sd_type = (ocr[0] & 0x40) ? 2 : 1;
        }
    }
    else
    {
        SD_Deselect();
        return 0;
    }

    SD_Deselect();
    SD_TxRx(0xFF);
    return 1;
}


static uint8_t SD_ReadBlock(uint8_t *buff, uint32_t sector)
{
    uint8_t token;
    uint32_t addr = (sd_type == 2) ? sector : (sector * 512U);

    if (SD_SendCommand(CMD17, addr, 0x01) != 0x00)
    {
        SD_Deselect();
        return 0;
    }

    uint32_t t0 = HAL_GetTick();
    do
    {
        token = SD_TxRx(0xFF);
        if (token == 0xFE) break;
    } while ((HAL_GetTick() - t0) < 200);

    if (token != 0xFE)
    {
        SD_Deselect();
        return 0;
    }

    for (uint16_t i = 0; i < 512; i++)
        buff[i] = SD_TxRx(0xFF);

    SD_TxRx(0xFF);
    SD_TxRx(0xFF);
    SD_Deselect();
    SD_TxRx(0xFF);

    return 1;
}


static uint8_t SD_WriteBlock(const uint8_t *buff, uint32_t sector)
{
    uint8_t resp;
    uint32_t addr = (sd_type == 2) ? sector : (sector * 512U);

    if (SD_SendCommand(CMD24, addr, 0x01) != 0x00)
    {
        SD_Deselect();
        return 0;
    }

    SD_TxRx(0xFF);
    SD_TxRx(0xFE);

    for (uint16_t i = 0; i < 512; i++)
        SD_TxRx(buff[i]);

    SD_TxRx(0xFF);
    SD_TxRx(0xFF);

    resp = SD_TxRx(0xFF);
    if ((resp & 0x1F) != 0x05)
    {
        SD_Deselect();
        return 0;
    }

    if (!SD_WaitReady(500))
    {
        SD_Deselect();
        return 0;
    }

    SD_Deselect();
    SD_TxRx(0xFF);
    return 1;
}



/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
	if (pdrv != 0) return STA_NOINIT;

	    if (SD_InitCard())
	    {
	        Stat = 0;
	    }
	    else
	    {
	        Stat = STA_NOINIT;
	    }

	    return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
	 if (pdrv != 0) return STA_NOINIT;
	 return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
	if (pdrv != 0 || !count) return RES_PARERR;
	    if (Stat & STA_NOINIT)   return RES_NOTRDY;

	    while (count--)
	    {
	        if (!SD_ReadBlock(buff, sector))
	            return RES_ERROR;

	        buff += 512;
	        sector++;
	    }

	    return RES_OK;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
  /* USER CODE HERE */
	if (pdrv != 0 || !count) return RES_PARERR;
	    if (Stat & STA_NOINIT)   return RES_NOTRDY;

	    while (count--)
	    {
	        if (!SD_WriteBlock(buff, sector))
	            return RES_ERROR;

	        buff += 512;
	        sector++;
	    }

	    return RES_OK;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
	 if (pdrv != 0) return RES_PARERR;
	    if (Stat & STA_NOINIT) return RES_NOTRDY;

	    switch (cmd)
	    {
	        case CTRL_SYNC:
	            return SD_WaitReady(500) ? RES_OK : RES_ERROR;

	        case GET_SECTOR_SIZE:
	            *(WORD*)buff = 512;
	            return RES_OK;

	        case GET_BLOCK_SIZE:
	            *(DWORD*)buff = 1;
	            return RES_OK;

	        case GET_SECTOR_COUNT:
	            *(DWORD*)buff = 0; /* optional: bisa diisi nanti kalau mau baca CSD */
	            return RES_OK;

	        default:
	            return RES_PARERR;
	    }

  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

