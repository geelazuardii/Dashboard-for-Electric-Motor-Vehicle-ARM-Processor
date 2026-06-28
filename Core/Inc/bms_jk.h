#ifndef BMS_JK_H
#define BMS_JK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

void JK_BMS_Init(UART_HandleTypeDef *huart);
void JK_BMS_Task(void);
void JK_BMS_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* Variabel ini bisa dilihat di Live Expressions */
extern volatile uint8_t  jk_bms_soc;
extern volatile float    jk_bms_voltage;
extern volatile float    jk_bms_current;
extern volatile uint8_t  jk_bms_connected;
extern volatile uint32_t jk_bms_last_update_ms;
extern volatile uint32_t jk_bms_ok_count;
extern volatile uint32_t jk_bms_fail_count;
extern volatile uint32_t jk_bms_echo_count;
extern volatile uint16_t jk_bms_last_rx_len;

#ifdef __cplusplus
}
#endif

#endif
