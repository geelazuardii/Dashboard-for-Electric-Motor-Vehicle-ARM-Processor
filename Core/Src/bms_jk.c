/*
 * bms_jk.c
 *
 *  Created on: May 9, 2026
 *      Author: Muhammad Galan
 */


#include "bms_jk.h"
#include <string.h>

#define JK_BMS_RX_BUF_SIZE          768U
#define JK_BMS_POLL_PERIOD_MS       3000U
#define JK_BMS_RESPONSE_TIMEOUT_MS  2000U
#define JK_BMS_FRAME_GAP_MS         120U

static UART_HandleTypeDef *jk_uart = NULL;

static uint8_t jk_rx_byte;
static volatile uint8_t jk_rx_buf[JK_BMS_RX_BUF_SIZE];
static volatile uint16_t jk_rx_len = 0;
static volatile uint8_t jk_rx_overflow = 0;
static volatile uint32_t jk_last_byte_tick = 0;

static uint8_t jk_parse_buf[JK_BMS_RX_BUF_SIZE];

static uint32_t jk_last_poll_tick = 0;
static uint32_t jk_request_tick = 0;
static uint8_t jk_waiting_response = 0;

volatile uint8_t  jk_bms_soc = 0;
volatile float    jk_bms_voltage = 0.0f;
volatile float    jk_bms_current = 0.0f;
volatile uint8_t  jk_bms_connected = 0;
volatile uint32_t jk_bms_last_update_ms = 0;
volatile uint32_t jk_bms_ok_count = 0;
volatile uint32_t jk_bms_fail_count = 0;
volatile uint32_t jk_bms_echo_count = 0;
volatile uint16_t jk_bms_last_rx_len = 0;

/* Request JK-BMS GPS/UART protocol */
static const uint8_t JK_READ_ALL[] = {
    0x4E, 0x57, 0x00, 0x13,
    0x00, 0x00, 0x00, 0x00,
    0x06, 0x03, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x68, 0x00, 0x00, 0x01,
    0x29
};

static uint16_t readU16BE(const uint8_t *buf, int index)
{
    return ((uint16_t)buf[index] << 8) | buf[index + 1];
}

static uint8_t isEchoRequestAt(const uint8_t *buf, int len, int pos)
{
    if (pos < 0) return 0;
    if ((pos + (int)sizeof(JK_READ_ALL)) > len) return 0;

    for (int i = 0; i < (int)sizeof(JK_READ_ALL); i++)
    {
        if (buf[pos + i] != JK_READ_ALL[i])
        {
            return 0;
        }
    }

    return 1;
}

static int findValidJkHeader(const uint8_t *buf, int len)
{
    for (int i = 0; i < len - 1; i++)
    {
        if ((buf[i] == 0x4E) && (buf[i + 1] == 0x57))
        {
            /* Skip kalau ini cuma echo request */
            if (isEchoRequestAt(buf, len, i))
            {
                continue;
            }

            return i;
        }
    }

    return -1;
}

static int getTagDataLength(uint8_t tag, const uint8_t *buf, int index, int len)
{
    switch (tag)
    {
        case 0x79:
            if ((index + 1) < len)
            {
                return 1 + buf[index + 1];
            }
            return -1;

        case 0x80:
        case 0x81:
        case 0x82:
        case 0x83:
        case 0x84:
        case 0x87:
        case 0x8A:
        case 0x8B:
        case 0x8C:
        case 0x8E:
        case 0x8F:
        case 0x90:
        case 0x91:
        case 0x92:
        case 0x93:
        case 0x94:
        case 0x95:
        case 0x96:
        case 0x97:
        case 0x98:
        case 0x99:
        case 0x9A:
        case 0x9B:
        case 0x9C:
        case 0x9E:
        case 0x9F:
        case 0xA0:
        case 0xA1:
        case 0xA2:
        case 0xA3:
        case 0xA4:
        case 0xA5:
        case 0xA6:
        case 0xA7:
        case 0xA8:
        case 0xB0:
            return 2;

        case 0x85:
        case 0x86:
        case 0x9D:
        case 0xA9:
        case 0xAB:
        case 0xAC:
        case 0xAE:
        case 0xAF:
        case 0xB1:
        case 0xB3:
        case 0xB8:
        case 0xC0:
            return 1;

        case 0x89:
        case 0xAA:
        case 0xB5:
        case 0xB6:
        case 0xB9:
            return 4;

        case 0xB2:
            return 10;

        case 0xB4:
            return 8;

        case 0xB7:
            return 15;

        case 0xBA:
            return 24;

        default:
            return -1;
    }
}

static uint8_t JK_BMS_ParseFrame(const uint8_t *buf, int len)
{
    int header = findValidJkHeader(buf, len);

    if (header < 0)
    {
        return 0;
    }

    int i = header + 11;

    uint8_t foundSoc = 0;
    uint8_t foundVoltage = 0;
    uint8_t foundCurrent = 0;

    uint8_t parsedSoc = 0;
    float parsedVoltage = 0.0f;
    float parsedCurrent = 0.0f;

    while (i < len - 5)
    {
        uint8_t tag = buf[i];

        if (tag == 0x68)
        {
            break;
        }

        int dataLen = getTagDataLength(tag, buf, i, len);

        if (dataLen < 0)
        {
            i++;
            continue;
        }

        int dataIndex = i + 1;

        if ((dataIndex + dataLen) > len)
        {
            break;
        }

        switch (tag)
        {
            case 0x83:
            {
                uint16_t rawVoltage = readU16BE(buf, dataIndex);
                parsedVoltage = rawVoltage * 0.01f;
                foundVoltage = 1;
                break;
            }

            case 0x84:
            {
                uint16_t rawCurrent = readU16BE(buf, dataIndex);

                if (rawCurrent & 0x8000U)
                {
                    parsedCurrent = (float)(rawCurrent & 0x7FFFU) * 0.01f;
                }
                else
                {
                    parsedCurrent = -((float)rawCurrent * 0.01f);
                }

                foundCurrent = 1;
                break;
            }

            case 0x85:
            {
                parsedSoc = buf[dataIndex];
                foundSoc = 1;
                break;
            }

            default:
                break;
        }

        i += 1 + dataLen;
    }

    if (!foundSoc)
    {
        return 0;
    }

    jk_bms_soc = parsedSoc;

    if (foundVoltage)
    {
        jk_bms_voltage = parsedVoltage;
    }

    if (foundCurrent)
    {
        jk_bms_current = parsedCurrent;
    }

    jk_bms_connected = 1;
    jk_bms_last_update_ms = HAL_GetTick();
    jk_bms_ok_count++;

    return 1;
}

void JK_BMS_Init(UART_HandleTypeDef *huart)
{
    jk_uart = huart;

    jk_rx_len = 0;
    jk_rx_overflow = 0;
    jk_waiting_response = 0;

    HAL_UART_Receive_IT(jk_uart, &jk_rx_byte, 1);
}

void JK_BMS_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (jk_uart == NULL) return;
    if (huart->Instance != jk_uart->Instance) return;

    if (jk_rx_len < JK_BMS_RX_BUF_SIZE)
    {
        jk_rx_buf[jk_rx_len++] = jk_rx_byte;
        jk_last_byte_tick = HAL_GetTick();
    }
    else
    {
        jk_rx_overflow = 1;
    }

    HAL_UART_Receive_IT(jk_uart, &jk_rx_byte, 1);
}

void JK_BMS_Task(void)
{
    if (jk_uart == NULL) return;

    uint32_t now = HAL_GetTick();

    if ((jk_bms_connected == 1U) &&
        ((now - jk_bms_last_update_ms) > 10000U))
    {
        jk_bms_connected = 0U;
    }

    if (!jk_waiting_response)
    {
        if ((now - jk_last_poll_tick) >= JK_BMS_POLL_PERIOD_MS)
        {
            __disable_irq();
            jk_rx_len = 0;
            jk_rx_overflow = 0;
            __enable_irq();

            HAL_UART_Transmit(jk_uart, (uint8_t *)JK_READ_ALL, sizeof(JK_READ_ALL), 30);

            jk_request_tick = now;
            jk_last_poll_tick = now;
            jk_waiting_response = 1;
        }

        return;
    }

    uint16_t len_snapshot = jk_rx_len;

    if ((len_snapshot > 0U) && ((now - jk_last_byte_tick) > JK_BMS_FRAME_GAP_MS))
    {
        uint16_t copy_len;

        __disable_irq();
        copy_len = jk_rx_len;

        if (copy_len > JK_BMS_RX_BUF_SIZE)
        {
            copy_len = JK_BMS_RX_BUF_SIZE;
        }

        memcpy(jk_parse_buf, (const void *)jk_rx_buf, copy_len);
        jk_rx_len = 0;
        __enable_irq();

        jk_bms_last_rx_len = copy_len;
        jk_waiting_response = 0;

        if ((copy_len == sizeof(JK_READ_ALL)) &&
            isEchoRequestAt(jk_parse_buf, copy_len, 0))
        {
            jk_bms_echo_count++;
            jk_bms_fail_count++;
            return;
        }

        if (!JK_BMS_ParseFrame(jk_parse_buf, copy_len))
        {
            jk_bms_fail_count++;
        }

        return;
    }

    if ((now - jk_request_tick) > JK_BMS_RESPONSE_TIMEOUT_MS)
    {
        jk_waiting_response = 0;
        jk_bms_fail_count++;

        __disable_irq();
        jk_rx_len = 0;
        __enable_irq();
    }
}

