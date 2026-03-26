#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"

/* ====================================================================
 * 1. CONFIGURAÇÃO DE HARDWARE (SEM CONFLITOS)
 * ==================================================================== */

// --- Display 7 Segmentos (Pinos Originais Mantidos) ---
const int segment_pins[] = {13, 15, 14, 27, 26, 25, 33}; // A, B, C, D, E, F, G
#define SEG_DP      32
const int digit_pins[] = {23, 22, 21}; // DIG1, DIG2, DIG3

// --- Comunicação RS485 (Pinos Originais Mantidos) ---
#define UART_RS485      UART_NUM_2
#define RS485_TX_PIN    17
#define RS485_RX_PIN    16
#define RS485_EN_PIN    4

// --- LEDs de Status (Novos Pinos para evitar conflito com 13, 15 e 4) ---
static const gpio_num_t LED_PINS[] = {GPIO_NUM_18, GPIO_NUM_19, GPIO_NUM_5, GPIO_NUM_2, GPIO_NUM_12};
#define LED_ON  0
#define LED_OFF 1

// --- Simulação UART (Console) ---
#define UART_CONSOLE    UART_NUM_0

/* ====================================================================
 * 2. ESTRUTURAS E ESTADOS
 * ==================================================================== */
typedef enum { STATE_READY, STATE_RUN, STATE_ERROR } system_state_t;

typedef struct {
    uint8_t  buttons;      // Bit 0: Start, Bit 1: Stop
    uint16_t target_freq;  // x100 (Ex: 6000 = 60.00Hz)
    uint8_t  direction;    // 0: FWD, 1: REV
} ihm_control_t;

typedef struct {
    uint16_t current_freq;
    uint16_t current_amp;
    uint8_t  temp;
} mi_telemetry_t;

// Globais
static ihm_control_t g_ctrl = {0, 3000, 0};
static mi_telemetry_t g_tel = {0};
static system_state_t g_state = STATE_READY;
volatile uint8_t display_buffer[3] = {0, 0, 0};
static bool comm_fail = false;

const uint8_t segment_map[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

/* ====================================================================
 * 3. TASK DE SIMULAÇÃO VIA UART (SUBSTITUI BTNS FÍSICOS)
 * ==================================================================== */
static void uart_console_task(void *arg) {
    uint8_t data[64];
    while (1) {
        int len = uart_read_bytes(UART_CONSOLE, data, sizeof(data) - 1, pdMS_TO_TICKS(50));
        if (len > 0) {
            data[len] = '\0';
            char *cmd = (char *)data;

            if (strstr(cmd, "START")) {
                g_ctrl.buttons |= 0x01;
                g_ctrl.buttons &= ~0x02;
                printf("[Simulação] Comando: START\n");
            } 
            else if (strstr(cmd, "STOP")) {
                g_ctrl.buttons |= 0x02;
                g_ctrl.buttons &= ~0x01;
                printf("[Simulação] Comando: STOP\n");
            }
            else if (strstr(cmd, "UP")) {
                if (g_ctrl.target_freq < 6000) g_ctrl.target_freq += 200;
                printf("[Simulação] Freq Alvo: %d\n", g_ctrl.target_freq);
            }
            else if (strstr(cmd, "DOWN")) {
                if (g_ctrl.target_freq > 0) g_ctrl.target_freq -= 200;
                printf("[Simulação] Freq Alvo: %d\n", g_ctrl.target_freq);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ====================================================================
 * 4. CONTROLE DO DISPLAY (MULTIPLEXAÇÃO)
 * ==================================================================== */
static void IRAM_ATTR multiplex_timer_callback(void* arg) {
    static uint8_t digit = 0;
    for(int i=0; i<3; i++) gpio_set_level(digit_pins[i], 1); // Desliga dígitos

    uint8_t segs = display_buffer[digit];
    for(int i=0; i<7; i++) gpio_set_level(segment_pins[i], (segs >> i) & 0x01);
    gpio_set_level(SEG_DP, (segs >> 7) & 0x01);

    gpio_set_level(digit_pins[digit], 0); // Liga dígito atual
    digit = (digit + 1) % 3;
}

void update_display_logic() {
    if (comm_fail) {
        display_buffer[0] = 0x79; // E
        display_buffer[1] = 0x3F; // 0
        display_buffer[2] = 0x7F; // 8
        return;
    }

    if (g_state == STATE_READY) {
        display_buffer[0] = 0x50; // r
        display_buffer[1] = 0x5E; // d
        display_buffer[2] = 0x6E; // y
    } else {
        uint16_t val = g_tel.current_freq / 10;
        display_buffer[0] = segment_map[(val/100)%10];
        display_buffer[1] = segment_map[(val/10)%10] | 0x80; // Ponto decimal
        display_buffer[2] = segment_map[val%10];
    }
}

/* ====================================================================
 * 5. COMUNICAÇÃO RS485 (CORE 1)
 * ==================================================================== */
static void ihm_sync_task(void *arg) {
    uint8_t dummy_rx[16];
    while (1) {
        // Simulação de Frame: Envia Controle, Recebe Telemetria
        gpio_set_level(RS485_EN_PIN, 1);
        uart_write_bytes(UART_RS485, &g_ctrl, sizeof(ihm_control_t));
        uart_wait_tx_done(UART_RS485, pdMS_TO_TICKS(10));
        gpio_set_level(RS485_EN_PIN, 0);

        int rx_len = uart_read_bytes(UART_RS485, dummy_rx, sizeof(dummy_rx), pdMS_TO_TICKS(100));
        
        if (rx_len > 0) {
            comm_fail = false;
            // Atualiza g_tel com dados reais vindos do STM32 aqui
            g_state = (g_ctrl.buttons & 0x01) ? STATE_RUN : STATE_READY;
        } else {
            comm_fail = true; 
        }

        update_display_logic();
        
        // Atualiza LEDs
        gpio_set_level(LED_PINS[0], (g_state == STATE_RUN) ? LED_ON : LED_OFF);
        gpio_set_level(LED_PINS[1], comm_fail ? LED_ON : LED_OFF);

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/* ====================================================================
 * 6. CONFIGURAÇÃO INICIAL
 * ==================================================================== */
void app_main(void) {
    nvs_flash_init();

    // Configuração de Saídas
    uint64_t out_mask = (1ULL << SEG_DP) | (1ULL << RS485_EN_PIN);
    for(int i=0; i<7; i++) out_mask |= (1ULL << segment_pins[i]);
    for(int i=0; i<3; i++) out_mask |= (1ULL << digit_pins[i]);
    for(int i=0; i<5; i++) out_mask |= (1ULL << LED_PINS[i]);

    gpio_config_t io_conf = {
        .pin_bit_mask = out_mask,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    /* // BOTÕES FÍSICOS (COMENTADOS PARA APLICAÇÃO REAL FUTURA)
    static const gpio_num_t BTN_PINS[] = {GPIO_NUM_34, GPIO_NUM_35, GPIO_NUM_36};
    uint64_t in_mask = 0;
    for(int i=0; i<3; i++) in_mask |= (1ULL << BTN_PINS[i]);
    gpio_config_t in_conf = { .pin_bit_mask = in_mask, .mode = GPIO_MODE_INPUT, .pull_up_en = 1 };
    gpio_config(&in_conf);
    */

    // UART RS485
    uart_config_t uart_cfg = { .baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_EVEN, .stop_bits = UART_STOP_BITS_1 };
    uart_driver_install(UART_RS485, 1024, 0, 0, NULL, 0);
    uart_param_config(UART_RS485, &uart_cfg);
    uart_set_pin(UART_RS485, RS485_TX_PIN, RS485_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Timer Display
    const esp_timer_create_args_t timer_args = { .callback = &multiplex_timer_callback, .name = "mux" };
    esp_timer_handle_t timer_h;
    esp_timer_create(&timer_args, &timer_h);
    esp_timer_start_periodic(timer_h, 4000);

    // Tasks
    xTaskCreatePinnedToCore(ihm_sync_task, "sync", 4096, NULL, 5, NULL, 1);
    xTaskCreate(uart_console_task, "console", 2048, NULL, 4, NULL);

    printf("IHM Integrada: Simulação UART Ativa. Use START/STOP/UP/DOWN no terminal.\n");
}
