/*
 * vesc_uart_rx.c
 *
 *  Created on: Jun 22, 2026
 *      Author: Muhammad Galan
 */


/* ===========================================================
 * vesc_uart_rx.c
 *
 * Penerima data VESC dari ESP32 via UART (STM32 HAL)
 * Parsing frame binary 20 byte dari ESP32.
 *
 * CARA INTEGRASI KE main.c (lihat bagian bawah file ini)
 * ===========================================================*/

#include <string.h>

/* ===========================================================
 * VARIABEL INTERNAL (PRIVATE)
 * ===========================================================*/

/* Buffer penerima per-byte (interrupt driven) */
static uint8_t  rx_byte;          /* Buffer satu byte untuk HAL interrupt */
static uint8_t  rx_buf[VESC_PKT_LEN]; /* Akumulasi frame */
static uint8_t  rx_idx;           /* Indeks byte yang sedang diterima      */
static uint8_t  rx_syncing;       /* 1 = sedang mencari START_BYTE         */

/* ===========================================================
 * DATA HASIL PARSING - accessible secara global
 * ===========================================================*/
static VescRxData_t vesc_rx_data;

/* ===========================================================
 * DEBUG COUNTERS - tambahkan ke Live Expression
 * ===========================================================*/
volatile uint32_t vesc_dbg_rx_count  = 0;
volatile uint32_t vesc_dbg_crc_err   = 0;
volatile uint32_t vesc_dbg_frame_err = 0;
volatile uint8_t  vesc_dbg_last_byte = 0;

/* ===========================================================
 * TIMEOUT KONEKSI (ms)
 * Jika tidak ada paket selama ini, connected = 0
 * ===========================================================*/
#define VESC_CONN_TIMEOUT_MS  2000U

/* ===========================================================
 * FUNGSI BANTU: BACA INT16 BIG-ENDIAN
 * ===========================================================*/
static int16_t readInt16BE(const uint8_t *buf) {
    return (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}

/* ===========================================================
 * PARSING FRAME 20 BYTE
 * Dipanggil saat 20 byte sudah terkumpul di rx_buf
 * ===========================================================*/
static void VESC_ParseFrame(void)
{
    /* Cek START, PACKET_ID, END */
    if (rx_buf[0] != VESC_PKT_START ||
        rx_buf[1] != VESC_PKT_ID    ||
        rx_buf[19] != VESC_PKT_END)
    {
        vesc_dbg_frame_err++;
        return;
    }

    /* Verifikasi checksum: XOR byte 1..17 harus == byte 18 */
    uint8_t cs = 0;
    for (uint8_t i = 1; i <= 17; i++) {
        cs ^= rx_buf[i];
    }
    if (cs != rx_buf[18]) {
        vesc_dbg_crc_err++;
        return;
    }

    /* Parsing data ke struct --------------------------------*/

    /* Byte 2-3 : speed km/h (int16) */
    vesc_rx_data.speed_kmh = readInt16BE(rx_buf + 2);
    if (vesc_rx_data.speed_kmh < 0) vesc_rx_data.speed_kmh = 0;

    /* Byte 4-5 : battery x10 -> float persen */
    int16_t batt_x10 = readInt16BE(rx_buf + 4);
    vesc_rx_data.battery_pct = (float)batt_x10 / 10.0f;
    if (vesc_rx_data.battery_pct < 0.0f)   vesc_rx_data.battery_pct = 0.0f;
    if (vesc_rx_data.battery_pct > 100.0f) vesc_rx_data.battery_pct = 100.0f;

    /* Byte 6-7 : input voltage x10 -> float Volt */
    int16_t volt_x10 = readInt16BE(rx_buf + 6);
    vesc_rx_data.input_voltage = (float)volt_x10 / 10.0f;

    /* Byte 8-9 : motor current x10 -> float Ampere */
    int16_t imot_x10 = readInt16BE(rx_buf + 8);
    vesc_rx_data.motor_current = (float)imot_x10 / 10.0f;

    /* Byte 10-11: input current x10 -> float Ampere */
    int16_t iin_x10 = readInt16BE(rx_buf + 10);
    vesc_rx_data.input_current = (float)iin_x10 / 10.0f;

    /* Byte 12-13: erpm/10 -> restore erpm */
    int16_t erpm_d10 = readInt16BE(rx_buf + 12);
    vesc_rx_data.erpm = (int32_t)erpm_d10 * 10;

    /* Byte 14: fault code */
    vesc_rx_data.fault_code = rx_buf[14];

    /* Byte 15: suhu MOSFET */
    vesc_rx_data.temp_mos = rx_buf[15];

    /* Byte 16: suhu motor */
    vesc_rx_data.temp_motor = rx_buf[16];

    /* Byte 17: flags */
    uint8_t flags = rx_buf[17];
    vesc_rx_data.regen_active  = (flags >> 0) & 0x01U;
    vesc_rx_data.values_valid  = (flags >> 1) & 0x01U;
    vesc_rx_data.setup_valid   = (flags >> 2) & 0x01U;

    /* Update timestamp dan status connected */
    vesc_rx_data.last_rx_tick = HAL_GetTick();
    vesc_rx_data.connected    = 1U;

    /* Counter debug */
    vesc_dbg_rx_count++;

    /* --------------------------------------------------------
     * PERBARUI VARIABEL GLOBAL TOUCHGFX
     * Variabel ini dideklarasikan extern volatile di main.c
     * --------------------------------------------------------*/
    extern volatile int32_t  velocity;
    extern volatile uint8_t  faultCode;

    /* velocity = kecepatan km/h (sudah dikonversi di ESP32)
     * Gunakan langsung nilainya; tidak perlu konversi ERPM lagi di sini
     * karena ESP32 sudah menghitung dengan spesifikasi ATV yang benar. */
    velocity  = (int32_t)vesc_rx_data.speed_kmh;
    faultCode = vesc_rx_data.fault_code;

    /* CATATAN: battery_pct dan data lain diambil oleh MainMenuView
     * melalui pointer VESC_GetData() atau variabel tambahan di bawah. */
}

/* ===========================================================
 * PROSES BYTE MASUK (State machine per-byte)
 * Dipanggil dari callback interrupt UART
 * ===========================================================*/
static void VESC_ProcessByte(uint8_t byte)
{
    vesc_dbg_last_byte = byte;

    if (rx_syncing)
    {
        /* Cari START byte */
        if (byte == VESC_PKT_START)
        {
            rx_buf[0]  = byte;
            rx_idx     = 1;
            rx_syncing = 0;
        }
        return;
    }

    /* Akumulasi byte */
    if (rx_idx < VESC_PKT_LEN)
    {
        rx_buf[rx_idx++] = byte;
    }

    /* Frame penuh? */
    if (rx_idx >= VESC_PKT_LEN)
    {
        VESC_ParseFrame();
        rx_idx     = 0;
        rx_syncing = 1; /* Tunggu START berikutnya */
    }
}

/* ===========================================================
 * INISIALISASI
 * Panggil setelah MX_USARTx_UART_Init() di main.c
 * ===========================================================*/
void VESC_UART_Init(void)
{
    memset(&vesc_rx_data, 0, sizeof(vesc_rx_data));
    rx_idx     = 0;
    rx_syncing = 1;  /* Mulai dalam mode sinkronisasi */

    /* Mulai receive interrupt per-byte */
    HAL_UART_Receive_IT(&HUART_VESC, &rx_byte, 1);
}

/* ===========================================================
 * TASK - panggil di while(1) loop main.c
 * Hanya mengecek timeout koneksi
 * ===========================================================*/
void VESC_UART_Task(void)
{
    /* Cek timeout: jika tidak ada paket > 2 detik, tandai disconnect */
    if (vesc_rx_data.connected)
    {
        uint32_t elapsed = HAL_GetTick() - vesc_rx_data.last_rx_tick;
        if (elapsed > VESC_CONN_TIMEOUT_MS)
        {
            vesc_rx_data.connected = 0U;

            /* Reset kecepatan ke 0 saat koneksi terputus */
            extern volatile int32_t velocity;
            velocity = 0;
        }
    }
}

/* ===========================================================
 * CALLBACK - dipanggil dari HAL_UART_RxCpltCallback di main.c
 * ===========================================================*/
void VESC_UART_RxCallback(void)
{
    VESC_ProcessByte(rx_byte);

    /* Re-arm interrupt untuk byte berikutnya */
    HAL_UART_Receive_IT(&HUART_VESC, &rx_byte, 1);
}

/* ===========================================================
 * GETTER DATA
 * ===========================================================*/
const VescRxData_t* VESC_GetData(void)
{
    return &vesc_rx_data;
}

