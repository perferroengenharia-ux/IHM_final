#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "IHM_AXON";

/* ============================================================
 * HARDWARE FIXO PEDIDO PELO USUÁRIO
 * - Pinos do display mantidos
 * - Pinos da comunicação mantidos
 * - Botões físicos apenas comentados para referência
 * ============================================================ */

/* -------- Display 7 segmentos (cátodo comum) -------- */
#define SEG_A               GPIO_NUM_13
#define SEG_B               GPIO_NUM_15
#define SEG_C               GPIO_NUM_14
#define SEG_D               GPIO_NUM_27
#define SEG_E               GPIO_NUM_26
#define SEG_F               GPIO_NUM_25
#define SEG_G               GPIO_NUM_33
#define SEG_DP              GPIO_NUM_32

#define DIGIT_1             GPIO_NUM_23
#define DIGIT_2             GPIO_NUM_22
#define DIGIT_3             GPIO_NUM_21

/* -------- Comunicação RS485 com MI -------- */
#define RS485_UART          UART_NUM_2
#define RS485_TX_PIN        GPIO_NUM_17
#define RS485_RX_PIN        GPIO_NUM_16
#define RS485_EN_PIN        GPIO_NUM_4
#define RS485_BAUD          115200

/* -------- LEDs remapeados --------
 * Os pinos 21/22/23 do código de periféricos conflitam com os dígitos do display.
 * Por isso os LEDs foram realocados.
 * Observação: GPIO0 é pino de strap; mantenha-o em nível alto no boot.
 */
#define LED_ACTIVE_ON       0
#define LED_ACTIVE_OFF      1
#define LED_SWING_PIN       GPIO_NUM_18
#define LED_DRENO_PIN       GPIO_NUM_19
#define LED_CLIMA_PIN       GPIO_NUM_2
#define LED_VENT_PIN        GPIO_NUM_5
#define LED_EXAUSTAO_PIN    GPIO_NUM_0

/* -------- Botões físicos (somente referência, NÃO usados) --------
static const gpio_num_t BTN_PINS[] = {
    GPIO_NUM_32, GPIO_NUM_33, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27,
    GPIO_NUM_14, GPIO_NUM_12, GPIO_NUM_13, GPIO_NUM_4, GPIO_NUM_5
};
*/

#define TOTAL_DIGITS            3
#define MULTIPLEX_INTERVAL_US   4000
#define BLINK_INTERVAL_MS       400
#define BUTTON_PULSE_MS         250
#define CONSOLE_BUF_SZ          128

#define HEARTBEAT_EXPECT_MS     500
#define STATUS_TIMEOUT_MS       3000
#define COMM_FAULT_HOLD_MS      3000
#define WD_CHECK_MS             100
#define COMM_STABLE_CLEAR_MS    600
#define COMM_STARTUP_GRACE_MS   4000

#define FAULT_HOLD_MS           200
#define FAULT_CLEAR_HOLD_MS     600

#define DC_OV_THRESHOLD         410
#define DC_UV_THRESHOLD         180
#define IGBT_OT_THRESHOLD       90

#define MAX_PAYLOAD             128
#define MAX_FRAME_RAW           (1 + 4 + MAX_PAYLOAD + 2)
#define MAX_FRAME_ESC           (MAX_FRAME_RAW * 2)
#define SOF                     0x7E
#define ESC                     0x7D
#define ESC_XOR                 0x20
#define ADDR_STM32              0x01
#define TYPE_READ_STATUS        0x04
#define TYPE_WRITE_PARAM        0x05

#define BTN_BIT_START           (1u << 0)
#define BTN_BIT_STOP            (1u << 1)
#define BTN_BIT_UP              (1u << 2)
#define BTN_BIT_DOWN            (1u << 3)

#define AUX_BIT_BOMBA           (1u << 0)
#define AUX_BIT_SWING           (1u << 1)
#define AUX_BIT_EXAUSTAO        (1u << 2)
#define AUX_BIT_DRENO           (1u << 3)
#define AUX_BIT_SYSTEM_ON       (1u << 4)
#define MI_STATUS_WATER_SHORTAGE (1u << 0)

#define E02_DC_OV               2
#define E03_DC_UV               3
#define E04_IGBT_OT             4
#define E05_OVL                 5
#define E07_LINE_UV             7
#define E08_COMM                8

/* ============================================================
 * ESTADOS E ESTRUTURAS
 * ============================================================ */

typedef enum {
    BTN_MAIS = 0,
    BTN_MENOS,
    BTN_CLIMATIZAR,
    BTN_VENTILAR,
    BTN_DRENO,
    BTN_SWING,
    BTN_EXAUSTAO,
    BTN_ONOFF,
    BTN_SET,
    BTN_RESET_WIFI
} button_id_t;

typedef enum {
    DRENO_IDLE = 0,
    DRENO_AGUARDANDO_LED,
    DRENO_EM_CURSO
} dreno_state_t;

typedef enum {
    STATE_READY = 0,
    STATE_RUN,
    STATE_MENU_SEL,
    STATE_MENU_EDIT,
    STATE_ERROR
} system_state_t;

typedef struct {
    int id;
    int value;
    int def_val;
    int min_val;
    int max_val;
    bool read_only;
    bool pending_sync;
} parameter_t;

typedef struct {
    uint8_t addr, type, seq, len;
    uint8_t payload[MAX_PAYLOAD];
} frame_t;

typedef enum {
    PS_WAIT_SOF = 0,
    PS_HDR_ADDR,
    PS_HDR_TYPE,
    PS_HDR_SEQ,
    PS_HDR_LEN,
    PS_PAYLOAD,
    PS_CRC_L,
    PS_CRC_H
} parse_state_t;

typedef struct {
    parse_state_t st;
    bool esc_next;
    uint8_t addr, type, seq, len;
    uint8_t payload[MAX_PAYLOAD];
    uint8_t pay_i, crc_l, crc_h;
} frame_parser_t;

typedef struct {
    uint16_t current_freq_centi_hz;
    uint16_t v_bus;
    uint16_t i_out;
    uint16_t v_out;
    uint8_t temp_igbt;
    uint8_t status_flags;
} mi_telemetry_t;

enum {
    IDX_P00 = 0,
    IDX_P01,
    IDX_P02,
    IDX_P03,
    IDX_P04,
    IDX_P05,
    IDX_P06,
    IDX_P10,
    IDX_P11,
    IDX_P12,
    IDX_P20,
    IDX_P21,
    IDX_P30,
    IDX_P31,
    IDX_P32,
    IDX_P33,
    IDX_P35,
    IDX_P42,
    IDX_P43,
    IDX_P44,
    IDX_P45,
    IDX_P51,
    IDX_P80,
    IDX_P81,
    IDX_P82,
    IDX_P83,
    IDX_P84,
    IDX_P85,
    IDX_P86,
    IDX_P90,
    IDX_P91,
    PARAM_COUNT
};

static parameter_t params[PARAM_COUNT] = {
    { 0,   0,   0,   0, 999, false, false},
    { 1,   0,   0,   0,  90, true,  false},
    { 2, 310, 310,   0, 410, true,  false},
    { 3,   0,   0,   0,  20, true,  false},
    { 4,   0,   0,   0, 410, true,  false},
    { 5,  45,  45,   0, 100, true,  false},
    { 6,   0,   0,   0,  99, true,  false},
    {10,  10,  10,   5,  60, false, false},
    {11,  10,  10,   5,  60, false, false},
    {12,   0,   0,   0,   1, false, false},
    {20,   1,   1,   1,  24, false, false},
    {21,  60,  60,  23,  90, false, false},
    {30,  10,  10,   0, 240, false, false},
    {31,   5,   5,   0, 240, false, false},
    {32,  30,  30,   0,  90, false, false},
    {33,   0,   0,   0,   1, false, false},
    {35,   0,   0,   0,   9, false, false},
    {42,   5,   5,   5,  15, false, false},
    {43,   5,   5,   1,   9, false, false},
    {44,   0,   0,   0,   1, false, false},
    {45, 180, 180, 100, 200, false, false},
    {51,   0,   0,   0,   1, false, false},
    {80,   0,   0,   0,   2, false, false},
    {81,   0,   0,   0,   1, false, false},
    {82,   1,   1,   0,   2, false, false},
    {83,  10,  10,   1, 240, false, false},
    {84,   5,   5,   1, 240, false, false},
    {85,   1,   1,   0,   2, false, false},
    {86,   0,   0,   0, 240, false, false},
    {90,   0,   0,   0, 100, true,  false},
    {91,   5,   5,   0,  40, false, false}
};

static const gpio_num_t segment_pins[7] = {
    SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G
};
static const gpio_num_t digit_pins[TOTAL_DIGITS] = {
    DIGIT_1, DIGIT_2, DIGIT_3
};
static const gpio_num_t led_pins[5] = {
    LED_SWING_PIN, LED_DRENO_PIN, LED_CLIMA_PIN, LED_VENT_PIN, LED_EXAUSTAO_PIN
};

static portMUX_TYPE meas_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE status_mux = portMUX_INITIALIZER_UNLOCKED;

static QueueHandle_t g_frame_q = NULL;
static frame_parser_t g_parser;

static system_state_t current_state = STATE_READY;
static dreno_state_t dreno_status = DRENO_IDLE;
static bool dreno_post_wait_active = false;
static int64_t dreno_auto_off_us = 0;
static int64_t dreno_ready_release_us = 0;

static bool system_on = false;
static bool motor_running = false;
static bool is_locked = true;
static bool bomba_on = false;
static bool swing_on = false;
static bool exaustao_on = false;
static bool saved_bomba_on = false;
static bool saved_resume_bomba_on = false;
static bool saved_resume_swing_on = false;
static bool saved_resume_exaustao_on = false;
static bool saved_exaustao_swing_on = false;
static bool exaustao_restart_pending = false;
static bool exaustao_exit_pending = false;
static int exaustao_saved_frequency = 0;
static int64_t exaustao_end_us = 0;
static uint8_t exaustao_pending_direction = 0;
static bool wifi_lost = false;
static bool water_shortage = false;
static bool g_show_logs = false;
static bool sim_ws = false;
static bool sim_wl = false;

static bool prewet_active = false;
static bool dryrun_active = false;
static bool dryrun_restart_pending = false;
static bool dryrun_exit_restore_pending = false;
static uint8_t dryrun_restore_direction = 0;
static uint8_t dryrun_pending_direction = 0;
static int64_t prewet_end_us = 0;
static int64_t dryrun_end_us = 0;
static int saved_target_before_stop = 0;
static bool saved_error_bomba_on = false;
static bool saved_error_swing_on = false;
static bool saved_error_exaustao_on = false;
static bool saved_error_system_on = false;

static int current_param_idx = IDX_P00;
static int current_error_code = 0;
static bool fault_clear_pending_ack = false;
static int temp_edit_value = 0;
static int saved_frequency = 1;
static int target_frequency = 1;            /* Hz */
static int target_before_error = 0;
static bool motor_was_running_before_error = false;
static float output_frequency = 0.0f;       /* Hz */

static bool blink_visible = true;
static bool dp_blink_visible = true;
static uint8_t display_buffer[3] = {0, 0, 0};

#define ERR_HIST_SIZE 5
static uint8_t err_hist[ERR_HIST_SIZE] = {0};
static uint8_t err_head = 0;
static uint8_t err_count = 0;
static int err_view_offset = 0;

static nvs_handle_t g_nvs = 0;
static const char *NVS_NS = "ihm";

static mi_telemetry_t g_telemetry = {0};
static volatile int64_t last_status_rx_us = 0;
static volatile uint32_t status_rx_packets = 0;
static volatile bool comm_good_recent = false;

static uint8_t g_button_pulse_mask = 0;
static int64_t g_button_pulse_until_us = 0;
static uint8_t g_direction = 0; /* 0=FWD, 1=REV */

static void update_display_logic(void);
static void update_leds(void);
static void dreno_service(void);
static void phase_service(void);
static void exaustao_service(void);
static void request_exaustao_stop(void);
static void finish_prewet_now(void);
static void finish_dryrun_now(bool immediate_stop);
static void force_finish_all_timed_cycles(void);
static void set_motor_running_ex(bool run, bool skip_timers);
static void handle_dreno_end(void);
static void request_mi_factory_reset(void);

/* ============================================================
 * UTILITÁRIOS
 * ============================================================ */

static int clamp_i(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static bool is_mi_synced_index(int idx) {
    switch (idx) {
        case IDX_P10:
        case IDX_P11:
        case IDX_P20:
        case IDX_P21:
        case IDX_P35:
        case IDX_P42:
        case IDX_P43:
        case IDX_P44:
        case IDX_P45:
        case IDX_P51:
        case IDX_P80:
        case IDX_P81:
        case IDX_P82:
        case IDX_P83:
        case IDX_P84:
        case IDX_P85:
            return true;
        default:
            return false;
    }
}

static void mark_param_pending_if_synced_to_mi(int idx) {
    if (idx >= 0 && idx < PARAM_COUNT && is_mi_synced_index(idx)) {
        params[idx].pending_sync = true;
    }
}

static void pulse_buttons(uint8_t bits) {
    g_button_pulse_mask |= bits;
    g_button_pulse_until_us = esp_timer_get_time() + ((int64_t)BUTTON_PULSE_MS * 1000LL);
}

static uint8_t consume_button_mask_for_tx(void) {
    if (g_button_pulse_mask == 0) {
        return 0;
    }

    if (esp_timer_get_time() > g_button_pulse_until_us) {
        g_button_pulse_mask = 0;
        return 0;
    }

    return g_button_pulse_mask;
}

static void normalize_local_modes_by_params(void) {
    /* P81: 0=desabilitado, 1=NA, 2=NF.\n     * Na IHM, 1 e 2 significam 'swing habilitado'; a polaridade real\n     * é aplicada no MI. Só P81=0 força desligado. */
    if (params[IDX_P81].value == 0) {
        swing_on = false;
    }

    if (params[IDX_P82].value == 0 || params[IDX_P85].value == 0) {
        bomba_on = false;
        saved_bomba_on = false;
        saved_resume_bomba_on = false;
    }
}

static void maybe_clear_button_pulse_after_tx(void) {
    if (esp_timer_get_time() > g_button_pulse_until_us) {
        g_button_pulse_mask = 0;
    }
}

static uint8_t get_aux_flags(void) {
    normalize_local_modes_by_params();

    bool motor_active_ui = (motor_running || output_frequency > 0.1f) && !prewet_active && !dryrun_active;

    uint8_t f = 0;
    if (bomba_on)      f |= AUX_BIT_BOMBA;
    if (swing_on && motor_active_ui) f |= AUX_BIT_SWING;
    if (exaustao_on)   f |= AUX_BIT_EXAUSTAO;
    if (dreno_status != DRENO_IDLE) f |= AUX_BIT_DRENO;
    if (system_on)     f |= AUX_BIT_SYSTEM_ON;
    return f;
}

static int telemetry_current_ma_to_display_a(uint16_t current_ma) {
    return (int)((current_ma + 500U) / 1000U);
}

static bool telemetry_overload_trip(uint16_t current_ma) {
    int threshold_a = clamp_i(params[IDX_P43].value, params[IDX_P43].min_val, params[IDX_P43].max_val);
    uint32_t threshold_ma = (uint32_t)threshold_a * 1000U;
    return (uint32_t)current_ma > threshold_ma;
}

static uint8_t get_char_pattern(char c) {
    switch (toupper((unsigned char)c)) {
        case '0': return 0b00111111;
        case '1': return 0b00000110;
        case '2': return 0b01011011;
        case '3': return 0b01001111;
        case '4': return 0b01100110;
        case '5': return 0b01101101;
        case '6': return 0b01111101;
        case '7': return 0b00000111;
        case '8': return 0b01111111;
        case '9': return 0b01101111;
        case 'P': return 0b01110011;
        case 'E': return 0b01111001;
        case 'R': return 0b01010000;
        case 'D': return 0b01011110;
        case 'Y': return 0b01101110;
        case 'O': return 0b00111111;
        case 'F': return 0b01110001;
        case 'L': return 0b00111000;
        case 'I': return 0b00000110;
        case 'S': return 0b01101101;
        case 'C': return 0b00111001;
        case 'N': return 0b01010100;
        case ' ': return 0b00000000;
        default:  return 0b00000000;
    }
}

static uint8_t err_hist_get_by_offset(int offset_from_newest) {
    if (err_count == 0) return 0;
    if (offset_from_newest < 0) offset_from_newest = 0;
    if (offset_from_newest >= err_count) offset_from_newest = err_count - 1;

    int idx = (int)err_head - 1 - offset_from_newest;
    while (idx < 0) idx += ERR_HIST_SIZE;
    return err_hist[idx % ERR_HIST_SIZE];
}

static void trim_spaces(char *s) {
    if (!s) return;
    while (*s && isspace((unsigned char)*s)) {
        memmove(s, s + 1, strlen(s));
    }
    size_t n = strlen(s);
    while (n > 0 && isspace((unsigned char)s[n - 1])) {
        s[--n] = '\0';
    }
}

/* ============================================================
 * NVS / PARÂMETROS
 * ============================================================ */

static void enforce_param_coherence(void) {
    int fmin = clamp_i(params[IDX_P20].value, params[IDX_P20].min_val, params[IDX_P20].max_val);
    int fmax = clamp_i(params[IDX_P21].value, params[IDX_P21].min_val, params[IDX_P21].max_val);

    if (fmin >= fmax) {
        fmin = fmax - 1;
        if (fmin < params[IDX_P20].min_val) {
            fmin = params[IDX_P20].min_val;
        }
    }

    params[IDX_P20].value = fmin;
    params[IDX_P21].value = fmax;

    params[IDX_P32].value = clamp_i(params[IDX_P32].value, params[IDX_P32].min_val, params[IDX_P21].value);
    params[IDX_P12].value = clamp_i(params[IDX_P12].value, params[IDX_P12].min_val, params[IDX_P12].max_val);
    params[IDX_P91].value = clamp_i(params[IDX_P91].value, 0, 40);
    params[IDX_P90].value = clamp_i(params[IDX_P90].value, 0, 100);

    saved_frequency = clamp_i(saved_frequency, params[IDX_P20].value, params[IDX_P21].value);
    target_frequency = clamp_i(target_frequency, params[IDX_P20].value, params[IDX_P21].value);
}

static void nvs_save_param_by_index(int idx) {
    if (!g_nvs || idx < 0 || idx >= PARAM_COUNT || params[idx].read_only) {
        return;
    }
    char key[8];
    snprintf(key, sizeof(key), "P%02d", params[idx].id);
    nvs_set_i32(g_nvs, key, (int32_t)params[idx].value);
    nvs_commit(g_nvs);
}

static void nvs_save_saved_frequency(void) {
    if (!g_nvs) return;
    nvs_set_i32(g_nvs, "saveF", (int32_t)saved_frequency);
    nvs_commit(g_nvs);
}

static void nvs_save_lock(void) {
    if (!g_nvs) return;
    nvs_set_u8(g_nvs, "locked", (uint8_t)(is_locked ? 1 : 0));
    nvs_commit(g_nvs);
}

static void nvs_save_error_history(void) {
    if (!g_nvs) return;
    nvs_set_blob(g_nvs, "err5", err_hist, sizeof(err_hist));
    nvs_set_u8(g_nvs, "eh", err_head);
    nvs_set_u8(g_nvs, "ec", err_count);
    nvs_commit(g_nvs);
}

static void nvs_init_and_open(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(nvs_open(NVS_NS, NVS_READWRITE, &g_nvs));
}

static void nvs_load_all(void) {
    if (!g_nvs) return;

    for (int i = 0; i < PARAM_COUNT; i++) {
        if (params[i].read_only) continue;
        char key[8];
        snprintf(key, sizeof(key), "P%02d", params[i].id);
        int32_t v = 0;
        if (nvs_get_i32(g_nvs, key, &v) == ESP_OK) {
            params[i].value = (int)v;
        }
    }

    int32_t sf = 0;
    if (nvs_get_i32(g_nvs, "saveF", &sf) == ESP_OK) {
        saved_frequency = (int)sf;
    }

    uint8_t lk = 1;
    if (nvs_get_u8(g_nvs, "locked", &lk) == ESP_OK) {
        is_locked = (lk != 0);
    }

    size_t sz = sizeof(err_hist);
    if (nvs_get_blob(g_nvs, "err5", err_hist, &sz) != ESP_OK || sz != sizeof(err_hist)) {
        memset(err_hist, 0, sizeof(err_hist));
        err_head = 0;
        err_count = 0;
    } else {
        uint8_t eh = 0;
        uint8_t ec = 0;
        if (nvs_get_u8(g_nvs, "eh", &eh) == ESP_OK) err_head = eh % ERR_HIST_SIZE;
        if (nvs_get_u8(g_nvs, "ec", &ec) == ESP_OK) err_count = (ec > ERR_HIST_SIZE) ? ERR_HIST_SIZE : ec;
    }

    enforce_param_coherence();
    err_view_offset = 0;
    params[IDX_P06].value = (int)err_hist_get_by_offset(0);
}

static void error_history_push(uint8_t code) {
    err_hist[err_head] = code;
    err_head = (err_head + 1) % ERR_HIST_SIZE;
    if (err_count < ERR_HIST_SIZE) err_count++;
    err_view_offset = 0;
    params[IDX_P06].value = (int)err_hist_get_by_offset(0);
    nvs_save_error_history();
}

/* ============================================================
 * DISPLAY E LEDs
 * ============================================================ */

static void update_leds(void) {
    normalize_local_modes_by_params();

    if (dreno_status != DRENO_IDLE || dreno_post_wait_active) {
        gpio_set_level(LED_SWING_PIN,    LED_ACTIVE_OFF);
        gpio_set_level(LED_DRENO_PIN,    LED_ACTIVE_ON);
        gpio_set_level(LED_CLIMA_PIN,    LED_ACTIVE_OFF);
        gpio_set_level(LED_VENT_PIN,     LED_ACTIVE_OFF);
        gpio_set_level(LED_EXAUSTAO_PIN, LED_ACTIVE_OFF);
        return;
    }

    if (!system_on) {
        for (int i = 0; i < 5; i++) {
            gpio_set_level(led_pins[i], LED_ACTIVE_OFF);
        }
        return;
    }

    bool motor_active_ui = (motor_running || output_frequency > 0.1f) && !prewet_active && !dryrun_active;
    gpio_set_level(LED_SWING_PIN,    (swing_on && motor_active_ui) ? LED_ACTIVE_ON : LED_ACTIVE_OFF);
    gpio_set_level(LED_DRENO_PIN,    LED_ACTIVE_OFF);
    gpio_set_level(LED_CLIMA_PIN,    (bomba_on && !exaustao_on) ? LED_ACTIVE_ON : LED_ACTIVE_OFF);
    gpio_set_level(LED_VENT_PIN,     (!bomba_on && !exaustao_on) ? LED_ACTIVE_ON : LED_ACTIVE_OFF);
    gpio_set_level(LED_EXAUSTAO_PIN, exaustao_on ? LED_ACTIVE_ON : LED_ACTIVE_OFF);
}

static void refresh_run_ready_state_from_output(void) {
    if (current_state == STATE_ERROR || current_state == STATE_MENU_SEL || current_state == STATE_MENU_EDIT) {
        return;
    }

    if (dreno_status != DRENO_IDLE || dreno_post_wait_active || prewet_active || dryrun_active) {
        return;
    }

    if (motor_running || output_frequency > 0.1f) {
        current_state = STATE_RUN;
    } else {
        current_state = STATE_READY;
    }
}

static void update_display_logic(void) {
    int val = 0;

    if (current_state != STATE_ERROR && current_state != STATE_MENU_SEL && current_state != STATE_MENU_EDIT) {
        if (prewet_active) {
            display_buffer[0] = get_char_pattern('L');
            display_buffer[1] = get_char_pattern('I');
            display_buffer[2] = get_char_pattern('P');
            return;
        }
        if (dryrun_active) {
            display_buffer[0] = get_char_pattern('S');
            display_buffer[1] = get_char_pattern('E');
            display_buffer[2] = get_char_pattern('C');
            return;
        }
        if (dreno_status != DRENO_IDLE || dreno_post_wait_active) {
            display_buffer[0] = get_char_pattern('D');
            display_buffer[1] = get_char_pattern('R');
            display_buffer[2] = get_char_pattern('N');
            return;
        }
    }

    switch (current_state) {
        case STATE_READY:
            display_buffer[0] = get_char_pattern('R');
            display_buffer[1] = get_char_pattern('D');
            display_buffer[2] = get_char_pattern('Y');
            return;

        case STATE_RUN:
            val = (int)output_frequency;
            break;

        case STATE_MENU_SEL:
            display_buffer[0] = get_char_pattern('P');
            display_buffer[1] = get_char_pattern((params[current_param_idx].id / 10) % 10 + '0');
            display_buffer[2] = get_char_pattern(params[current_param_idx].id % 10 + '0');
            return;

        case STATE_MENU_EDIT: {
            int p_id = params[current_param_idx].id;
            if (p_id == 6) {
                int e = (int)err_hist_get_by_offset(err_view_offset);
                display_buffer[0] = get_char_pattern('E');
                display_buffer[1] = get_char_pattern((e / 10) % 10 + '0');
                display_buffer[2] = get_char_pattern(e % 10 + '0');
                return;
            }
            val = temp_edit_value;
        } break;

        case STATE_ERROR:
            display_buffer[0] = get_char_pattern('E');
            display_buffer[1] = get_char_pattern((current_error_code / 10) % 10 + '0');
            display_buffer[2] = get_char_pattern(current_error_code % 10 + '0');
            return;
    }

    display_buffer[0] = (val >= 100) ? get_char_pattern((val / 100) % 10 + '0') : get_char_pattern(' ');
    display_buffer[1] = (val >= 10)  ? get_char_pattern((val / 10) % 10 + '0') : get_char_pattern(' ');
    display_buffer[2] = get_char_pattern(val % 10 + '0');

    if (val == 0) {
        display_buffer[0] = get_char_pattern(' ');
        display_buffer[1] = get_char_pattern(' ');
        display_buffer[2] = get_char_pattern('0');
    }
}

static void multiplex_timer_callback(void *arg) {
    (void)arg;
    static int current_pos = 0;
    static uint32_t blink_ticks = 0;
    static uint32_t dp_ticks = 0;

    blink_ticks++;
    if (blink_ticks >= (BLINK_INTERVAL_MS * 1000 / MULTIPLEX_INTERVAL_US)) {
        blink_visible = !blink_visible;
        blink_ticks = 0;
    }

    dp_ticks++;
    if (dp_ticks >= (250000 / MULTIPLEX_INTERVAL_US)) {
        dp_blink_visible = !dp_blink_visible;
        dp_ticks = 0;
    }

    for (int i = 0; i < TOTAL_DIGITS; i++) {
        gpio_set_level(digit_pins[i], 0);
    }

    bool should_hide = false;
    if (current_state == STATE_ERROR && !blink_visible) {
        should_hide = true;
    }

    if (current_state == STATE_MENU_EDIT && !blink_visible) {
        int p_id = params[current_param_idx].id;
        bool motor_active = (motor_running || output_frequency > 0.1f);
        bool can_actually_edit = true;

        if (p_id == 6 || params[current_param_idx].read_only) {
            can_actually_edit = false;
        } else if (is_locked && p_id != 0) {
            can_actually_edit = false;
        } else if (motor_active &&
                   (p_id == 10 || p_id == 11 || p_id == 20 || p_id == 21 ||
                    p_id == 32 || p_id == 35 || p_id == 42 || p_id == 43 ||
                    p_id == 45 || p_id == 51)) {
            can_actually_edit = false;
        }

        if (can_actually_edit) {
            should_hide = true;
        }
    }

    if (!should_hide) {
        uint8_t pattern = display_buffer[current_pos];
        for (int i = 0; i < 7; i++) {
            gpio_set_level(segment_pins[i], (pattern >> i) & 0x01);
        }

        bool dp_state =
            (current_pos == 0 && water_shortage && dp_blink_visible) ||
            (current_pos == 2 && wifi_lost      && dp_blink_visible);
        gpio_set_level(SEG_DP, dp_state ? 1 : 0);

        gpio_set_level(digit_pins[current_pos], 1);
    }

    current_pos = (current_pos + 1) % TOTAL_DIGITS;
}

/* ============================================================
 * ERROS
 * ============================================================ */

static void enter_error(int code) {
    if (current_state == STATE_ERROR && current_error_code != E08_COMM) {
        return;
    }

    if (current_state != STATE_ERROR) {
        motor_was_running_before_error = motor_running;
        target_before_error = target_frequency;
        saved_error_bomba_on = bomba_on;
        saved_error_swing_on = swing_on;
        saved_error_exaustao_on = exaustao_on;
        saved_error_system_on = system_on;
    }

    prewet_active = false;
    prewet_end_us = 0;
    dryrun_active = false;
    dryrun_restart_pending = false;
    dryrun_exit_restore_pending = false;
    dryrun_end_us = 0;

    current_error_code = code;
    fault_clear_pending_ack = false;
    params[IDX_P06].value = code;
    error_history_push((uint8_t)code);

    current_state = STATE_ERROR;
    motor_running = false;
    system_on = false;
    pulse_buttons(BTN_BIT_STOP);
    update_display_logic();
    update_leds();
}

static void clear_error_manual_ack(void) {
    current_error_code = 0;
    fault_clear_pending_ack = false;
    prewet_active = false;
    prewet_end_us = 0;
    dryrun_active = false;
    dryrun_restart_pending = false;
    dryrun_exit_restore_pending = false;
    dryrun_end_us = 0;
    motor_running = false;

    /* Com P44=0, reconhecer o erro NÃO deve religar o sistema nem periféricos.
     * Apenas memoriza o estado anterior para o próximo comando de partida. */
    saved_resume_bomba_on = saved_error_bomba_on;
    saved_resume_swing_on = saved_error_swing_on;
    saved_resume_exaustao_on = saved_error_exaustao_on;
    saved_bomba_on = saved_error_bomba_on;

    system_on = false;
    bomba_on = false;
    swing_on = false;
    exaustao_on = false;
    exaustao_restart_pending = false;
    exaustao_exit_pending = false;
    g_direction = (uint8_t)params[IDX_P51].value;
    current_state = STATE_READY;

    normalize_local_modes_by_params();
    refresh_run_ready_state_from_output();
    update_display_logic();
    update_leds();
}

static void clear_error_auto(void) {
    current_error_code = 0;
    fault_clear_pending_ack = false;
    prewet_active = false;
    prewet_end_us = 0;
    dryrun_active = false;
    dryrun_restart_pending = false;
    dryrun_exit_restore_pending = false;
    dryrun_end_us = 0;

    bomba_on = saved_error_bomba_on;
    swing_on = saved_error_swing_on;
    exaustao_on = saved_error_exaustao_on;
    saved_resume_bomba_on = bomba_on;
    saved_resume_swing_on = swing_on;
    saved_resume_exaustao_on = exaustao_on;
    normalize_local_modes_by_params();
    g_direction = exaustao_on ? (uint8_t)(params[IDX_P51].value ? 0 : 1) : (uint8_t)params[IDX_P51].value;

    if (params[IDX_P44].value == 1 && motor_was_running_before_error) {
        motor_running = true;
        system_on = true;
        current_state = STATE_RUN;
        target_frequency = (params[IDX_P12].value == 1)
            ? clamp_i(target_before_error, params[IDX_P20].value, params[IDX_P21].value)
            : params[IDX_P20].value;
        pulse_buttons(BTN_BIT_START);
    } else {
        motor_running = false;
        system_on = saved_error_system_on;
        current_state = STATE_READY;
        refresh_run_ready_state_from_output();
    }

    update_display_logic();
    update_leds();
}

/* ============================================================
 * RS485 / PROTOCOLO
 * ============================================================ */

static uint16_t crc16_ibm(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

static void parser_reset(frame_parser_t *p) {
    p->st = PS_WAIT_SOF;
    p->esc_next = false;
    p->pay_i = 0;
}

static bool parser_feed(frame_parser_t *p, uint8_t byte, frame_t *out_frame) {
    if (byte == SOF) {
        parser_reset(p);
        p->st = PS_HDR_ADDR;
        return false;
    }
    if (p->st == PS_WAIT_SOF) {
        return false;
    }
    if (p->esc_next) {
        byte ^= ESC_XOR;
        p->esc_next = false;
    } else if (byte == ESC) {
        p->esc_next = true;
        return false;
    }

    switch (p->st) {
        case PS_HDR_ADDR:
            p->addr = byte;
            p->st = PS_HDR_TYPE;
            break;
        case PS_HDR_TYPE:
            p->type = byte;
            p->st = PS_HDR_SEQ;
            break;
        case PS_HDR_SEQ:
            p->seq = byte;
            p->st = PS_HDR_LEN;
            break;
        case PS_HDR_LEN:
            if (byte > MAX_PAYLOAD) {
                parser_reset(p);
                return false;
            }
            p->len = byte;
            p->st = (p->len == 0) ? PS_CRC_L : PS_PAYLOAD;
            break;
        case PS_PAYLOAD:
            p->payload[p->pay_i++] = byte;
            if (p->pay_i >= p->len) {
                p->st = PS_CRC_L;
            }
            break;
        case PS_CRC_L:
            p->crc_l = byte;
            p->st = PS_CRC_H;
            break;
        case PS_CRC_H: {
            p->crc_h = byte;
            uint8_t tmp[4 + MAX_PAYLOAD];
            tmp[0] = p->addr;
            tmp[1] = p->type;
            tmp[2] = p->seq;
            tmp[3] = p->len;
            if (p->len) memcpy(&tmp[4], p->payload, p->len);
            uint16_t calc = crc16_ibm(tmp, 4 + p->len);
            uint16_t recv = (uint16_t)p->crc_l | ((uint16_t)p->crc_h << 8);
            if (calc == recv) {
                out_frame->addr = p->addr;
                out_frame->type = p->type;
                out_frame->seq = p->seq;
                out_frame->len = p->len;
                if (p->len) memcpy(out_frame->payload, p->payload, p->len);
                parser_reset(p);
                return true;
            }
            parser_reset(p);
        } break;
        default:
            parser_reset(p);
            break;
    }

    return false;
}

static size_t build_frame(uint8_t addr, uint8_t type, uint8_t seq,
                          const uint8_t *payload, uint8_t len,
                          uint8_t *out_esc, size_t out_max) {
    uint8_t raw[MAX_FRAME_RAW];
    size_t r_idx = 0;

    (void)out_max;
    raw[r_idx++] = SOF;
    raw[r_idx++] = addr;
    raw[r_idx++] = type;
    raw[r_idx++] = seq;
    raw[r_idx++] = len;
    if (len && payload) {
        memcpy(&raw[r_idx], payload, len);
        r_idx += len;
    }

    uint16_t crc = crc16_ibm(&raw[1], r_idx - 1);
    raw[r_idx++] = (uint8_t)(crc & 0xFF);
    raw[r_idx++] = (uint8_t)((crc >> 8) & 0xFF);

    size_t e_idx = 0;
    out_esc[e_idx++] = SOF;
    for (size_t i = 1; i < r_idx; i++) {
        if (raw[i] == SOF || raw[i] == ESC) {
            out_esc[e_idx++] = ESC;
            out_esc[e_idx++] = raw[i] ^ ESC_XOR;
        } else {
            out_esc[e_idx++] = raw[i];
        }
    }

    return e_idx;
}

static inline void rs485_set_tx(bool en) {
    gpio_set_level(RS485_EN_PIN, en ? 1 : 0);
}

static void status_touch(void) {
    portENTER_CRITICAL(&status_mux);
    last_status_rx_us = esp_timer_get_time();
    status_rx_packets++;
    comm_good_recent = true;
    portEXIT_CRITICAL(&status_mux);
}

static bool rs485_request(uint8_t type, const uint8_t *payload, uint8_t len,
                          frame_t *reply, uint32_t timeout_ms) {
    static uint8_t seq = 0;
    seq++;

    uint8_t txbuf[MAX_FRAME_ESC];
    size_t txlen = build_frame(ADDR_STM32, type, seq, payload, len, txbuf, sizeof(txbuf));

    frame_t dump;
    while (xQueueReceive(g_frame_q, &dump, 0) == pdTRUE) {
        /* flush */
    }

    rs485_set_tx(true);
    esp_rom_delay_us(50);
    uart_write_bytes(RS485_UART, (const char *)txbuf, txlen);
    uart_wait_tx_done(RS485_UART, pdMS_TO_TICKS(100));
    rs485_set_tx(false);

    frame_t fr;
    if (xQueueReceive(g_frame_q, &fr, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        if (fr.seq == seq) {
            if (reply) *reply = fr;
            return true;
        }
    }

    return false;
}

static bool sync_params_to_mi(bool force_all) {
    uint8_t tx_payload[3];
    frame_t rep;

    for (int i = 0; i < PARAM_COUNT; i++) {
        if (!is_mi_synced_index(i)) continue;
        if (!(force_all || params[i].pending_sync)) continue;

        tx_payload[0] = (uint8_t)params[i].id;
        tx_payload[1] = (uint8_t)((uint16_t)params[i].value >> 8);
        tx_payload[2] = (uint8_t)((uint16_t)params[i].value & 0xFF);

        if (!rs485_request(TYPE_WRITE_PARAM, tx_payload, 3, &rep, 200)) {
            return false;
        }

        params[i].pending_sync = false;
        ESP_LOGI(TAG, "Sync MI: P%02d=%d", params[i].id, params[i].value);
    }

    return true;
}

static void request_mi_factory_reset(void) {
    uint8_t tx_payload[3];
    frame_t rep;
    tx_payload[0] = 0u;
    tx_payload[1] = 0u;
    tx_payload[2] = 101u;

    if (!rs485_request(TYPE_WRITE_PARAM, tx_payload, 3, &rep, 200)) {
        ESP_LOGW(TAG, "MI factory reset command failed.");
    } else {
        ESP_LOGI(TAG, "MI factory reset command sent.");
    }
}

static void init_rs485_uart(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << RS485_EN_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    rs485_set_tx(false);

    uart_config_t cfg = {
        .baud_rate = RS485_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(RS485_UART, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(RS485_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(RS485_UART, RS485_TX_PIN, RS485_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

/* ============================================================
 * TAREFAS
 * ============================================================ */

static void rs485_rx_task(void *arg) {
    (void)arg;
    uint8_t buf[128];
    parser_reset(&g_parser);

    while (1) {
        int n = uart_read_bytes(RS485_UART, buf, sizeof(buf), pdMS_TO_TICKS(10));
        if (n > 0) {
            for (int i = 0; i < n; i++) {
                frame_t fr;
                if (parser_feed(&g_parser, buf[i], &fr)) {
                    if (g_frame_q) {
                        xQueueSend(g_frame_q, &fr, 0);
                    }
                }
            }
        }
    }
}

static void sim_task(void *arg) {
    (void)arg;

    while (1) {
        wifi_lost = sim_wl;
        water_shortage = sim_ws || ((g_telemetry.status_flags & MI_STATUS_WATER_SHORTAGE) != 0u);
        dreno_service();
        phase_service();
        update_display_logic();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void ihm_sync_task(void *arg) {
    (void)arg;
    frame_t rep;
    uint8_t tx_payload[9];

    vTaskDelay(pdMS_TO_TICKS(1500));

    ESP_LOGI(TAG, "Iniciando handshake MI...");
    while (!sync_params_to_mi(true)) {
        ESP_LOGW(TAG, "Falha no handshake, tentando novamente...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    ESP_LOGI(TAG, "Handshake MI concluído.");

    while (1) {

        tx_payload[0] = consume_button_mask_for_tx();
        tx_payload[1] = (uint8_t)(((uint16_t)(target_frequency * 100)) >> 8);
        tx_payload[2] = (uint8_t)(((uint16_t)(target_frequency * 100)) & 0xFF);
        tx_payload[3] = g_direction;
        tx_payload[4] = get_aux_flags();
        tx_payload[5] = (uint8_t)dreno_status;
        tx_payload[6] = (uint8_t)params[IDX_P90].value;
        tx_payload[7] = (uint8_t)params[IDX_P91].value;
        tx_payload[8] = (uint8_t)(current_state == STATE_ERROR && current_error_code == E08_COMM);

        if (g_show_logs) {
            ESP_LOGI(TAG, "TX BTN=0x%02X FREQ=%d DIR=%u AUX=0x%02X DRENO=%d",
                     tx_payload[0], target_frequency * 100, g_direction, tx_payload[4], dreno_status);
        }

        if (rs485_request(TYPE_READ_STATUS, tx_payload, 9, &rep, 120)) {
            status_touch();
            maybe_clear_button_pulse_after_tx();

            if (rep.len >= 9) {
                portENTER_CRITICAL(&meas_mux);
                g_telemetry.current_freq_centi_hz = (uint16_t)((rep.payload[0] << 8) | rep.payload[1]);
                g_telemetry.i_out = (uint16_t)((rep.payload[2] << 8) | rep.payload[3]);
                g_telemetry.v_bus = (uint16_t)((rep.payload[4] << 8) | rep.payload[5]);
                g_telemetry.v_out = (uint16_t)((rep.payload[6] << 8) | rep.payload[7]);
                g_telemetry.temp_igbt = rep.payload[8];
                g_telemetry.status_flags = (rep.len >= 10) ? rep.payload[9] : 0u;
                portEXIT_CRITICAL(&meas_mux);

                output_frequency = ((float)g_telemetry.current_freq_centi_hz) / 100.0f;
                params[IDX_P01].value = (int)output_frequency;
                params[IDX_P02].value = g_telemetry.v_bus;
                params[IDX_P03].value = clamp_i(telemetry_current_ma_to_display_a(g_telemetry.i_out), 0, params[IDX_P03].max_val);
                params[IDX_P04].value = g_telemetry.v_out;
                params[IDX_P05].value = g_telemetry.temp_igbt;
                water_shortage = sim_ws || ((g_telemetry.status_flags & MI_STATUS_WATER_SHORTAGE) != 0u);
                wifi_lost = sim_wl;
                refresh_run_ready_state_from_output();

                if (g_show_logs) {
                    ESP_LOGI(TAG, "RX Freq=%.2fHz I=%umA Vbus=%u Vout=%u Temp=%u",
                             output_frequency, g_telemetry.i_out, g_telemetry.v_bus,
                             g_telemetry.v_out, g_telemetry.temp_igbt);
                }

                bool needs_sync = false;
                for (int i = 0; i < PARAM_COUNT; i++) {
                    if (params[i].pending_sync) {
                        needs_sync = true;
                        break;
                    }
                }
                if (needs_sync) {
                    ESP_LOGI(TAG, "Sincronizando pendências com o MI...");
                    sync_params_to_mi(false);
                }
            }
        } else if (g_show_logs) {
            ESP_LOGW(TAG, "Timeout MI");
        }

        update_display_logic();
        update_leds();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void status_watchdog_task(void *arg) {
    (void)arg;
    const int64_t hb_us = (int64_t)HEARTBEAT_EXPECT_MS * 1000LL;
    int64_t expected_acc_us = 0;
    int64_t last_loop_us = esp_timer_get_time();
    int64_t boot_us = last_loop_us;
    uint32_t last_rx_total = 0;
    float loss_ema = 0.0f;
    const float alpha = 0.20f;
    int fault_hold_ms = 0;
    int stable_ms = 0;

    while (1) {
        int64_t now = esp_timer_get_time();
        int64_t dt = now - last_loop_us;
        last_loop_us = now;
        bool in_grace = ((now - boot_us) < (int64_t)COMM_STARTUP_GRACE_MS * 1000LL);

        expected_acc_us += dt;
        uint32_t expected_pkts = 0;
        while (expected_acc_us >= hb_us) {
            expected_pkts++;
            expected_acc_us -= hb_us;
        }

        int64_t last_rx_us_local;
        uint32_t rx_total;
        bool good_pulse;
        portENTER_CRITICAL(&status_mux);
        last_rx_us_local = last_status_rx_us;
        rx_total = status_rx_packets;
        good_pulse = comm_good_recent;
        comm_good_recent = false;
        portEXIT_CRITICAL(&status_mux);

        uint32_t rx_delta = rx_total - last_rx_total;
        last_rx_total = rx_total;

        bool timed_out = (last_rx_us_local == 0) || ((now - last_rx_us_local) > (int64_t)STATUS_TIMEOUT_MS * 1000LL);

        float inst_loss = loss_ema;
        if (expected_pkts > 0) {
            float r = (float)rx_delta / (float)expected_pkts;
            if (r > 1.0f) r = 1.0f;
            if (r < 0.0f) r = 0.0f;
            inst_loss = 1.0f - r;
        }

        if (good_pulse && !timed_out) {
            loss_ema *= 0.5f;
        }
        loss_ema = (1.0f - alpha) * loss_ema + alpha * inst_loss;

        params[IDX_P90].value = clamp_i((int)(loss_ema * 100.0f + 0.5f), 0, 100);

        bool comm_bad = timed_out || (params[IDX_P90].value > params[IDX_P91].value);

        if (in_grace) {
            fault_hold_ms = 0;
            stable_ms = 0;
        } else {
            if (comm_bad) {
                fault_hold_ms += WD_CHECK_MS;
                stable_ms = 0;
            } else {
                fault_hold_ms = 0;
                stable_ms += WD_CHECK_MS;
            }

            if (fault_hold_ms >= COMM_FAULT_HOLD_MS) {
                if (!(current_state == STATE_ERROR && current_error_code != E08_COMM)) {
                    enter_error(E08_COMM);
                }
            }

            if (current_state == STATE_ERROR && current_error_code == E08_COMM) {
                if (stable_ms >= COMM_STABLE_CLEAR_MS) {
                    if (params[IDX_P44].value == 1) {
                        clear_error_auto();
                    } else {
                        clear_error_manual_ack();
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(WD_CHECK_MS));
    }
}

static void mi_fault_task(void *arg) {
    (void)arg;
    int hold_e02 = 0, hold_e03 = 0, hold_e04 = 0, hold_e05 = 0, hold_e07 = 0;
    int clr_e02 = 0,  clr_e03 = 0,  clr_e04 = 0,  clr_e05 = 0,  clr_e07 = 0;
    int mi_line_voltage_local = 220;

    while (1) {
        int dc, tmp, cur_a, line, outv;
        uint16_t cur_ma;
        portENTER_CRITICAL(&meas_mux);
        dc   = g_telemetry.v_bus;
        tmp  = g_telemetry.temp_igbt;
        cur_ma = g_telemetry.i_out;
        cur_a = telemetry_current_ma_to_display_a(cur_ma);
        outv = g_telemetry.v_out;
        line = mi_line_voltage_local;
        portEXIT_CRITICAL(&meas_mux);

        params[IDX_P02].value = clamp_i(dc,  params[IDX_P02].min_val, params[IDX_P02].max_val);
        params[IDX_P03].value = clamp_i(cur_a, params[IDX_P03].min_val, params[IDX_P03].max_val);
        params[IDX_P04].value = clamp_i(outv, params[IDX_P04].min_val, params[IDX_P04].max_val);
        params[IDX_P05].value = clamp_i(tmp, params[IDX_P05].min_val, params[IDX_P05].max_val);

        bool allow_override = (current_state != STATE_ERROR) || (current_error_code == E08_COMM);

        bool e02 = (dc > DC_OV_THRESHOLD);
        hold_e02 = e02 ? (hold_e02 + WD_CHECK_MS) : 0;
        if (allow_override && hold_e02 >= FAULT_HOLD_MS) enter_error(E02_DC_OV);
        if (current_state == STATE_ERROR && current_error_code == E02_DC_OV) {
            clr_e02 = (!e02) ? (clr_e02 + WD_CHECK_MS) : 0;
            if (clr_e02 >= FAULT_CLEAR_HOLD_MS) {
                if (params[IDX_P44].value == 1) clear_error_auto();
                else clear_error_manual_ack();
            }
        } else clr_e02 = 0;

        bool e03 = (dc > 0 && dc < DC_UV_THRESHOLD);
        hold_e03 = e03 ? (hold_e03 + WD_CHECK_MS) : 0;
        if (allow_override && hold_e03 >= FAULT_HOLD_MS) enter_error(E03_DC_UV);
        if (current_state == STATE_ERROR && current_error_code == E03_DC_UV) {
            clr_e03 = (!e03) ? (clr_e03 + WD_CHECK_MS) : 0;
            if (clr_e03 >= FAULT_CLEAR_HOLD_MS) {
                if (params[IDX_P44].value == 1) clear_error_auto();
                else clear_error_manual_ack();
            }
        } else clr_e03 = 0;

        bool e04 = (tmp > IGBT_OT_THRESHOLD);
        hold_e04 = e04 ? (hold_e04 + WD_CHECK_MS) : 0;
        if (allow_override && hold_e04 >= FAULT_HOLD_MS) enter_error(E04_IGBT_OT);
        if (current_state == STATE_ERROR && current_error_code == E04_IGBT_OT) {
            clr_e04 = (!e04) ? (clr_e04 + WD_CHECK_MS) : 0;
            if (clr_e04 >= FAULT_CLEAR_HOLD_MS) {
                if (params[IDX_P44].value == 1) clear_error_auto();
                else clear_error_manual_ack();
            }
        } else clr_e04 = 0;

        bool e05 = telemetry_overload_trip(cur_ma);
        hold_e05 = e05 ? (hold_e05 + WD_CHECK_MS) : 0;
        if (allow_override && hold_e05 >= FAULT_HOLD_MS) enter_error(E05_OVL);
        if (current_state == STATE_ERROR && current_error_code == E05_OVL) {
            clr_e05 = (!e05) ? (clr_e05 + WD_CHECK_MS) : 0;
            if (clr_e05 >= FAULT_CLEAR_HOLD_MS) {
                if (params[IDX_P44].value == 1) clear_error_auto();
                else clear_error_manual_ack();
            }
        } else clr_e05 = 0;

        bool e07 = (line > 0 && line < params[IDX_P45].value);
        hold_e07 = e07 ? (hold_e07 + WD_CHECK_MS) : 0;
        if (allow_override && hold_e07 >= FAULT_HOLD_MS) enter_error(E07_LINE_UV);
        if (current_state == STATE_ERROR && current_error_code == E07_LINE_UV) {
            clr_e07 = (!e07) ? (clr_e07 + WD_CHECK_MS) : 0;
            if (clr_e07 >= FAULT_CLEAR_HOLD_MS) {
                if (params[IDX_P44].value == 1) clear_error_auto();
                else clear_error_manual_ack();
            }
        } else clr_e07 = 0;

        update_display_logic();
        vTaskDelay(pdMS_TO_TICKS(WD_CHECK_MS));
    }
}

/* ============================================================
 * LÓGICA DE IHM / BOTÕES SIMULADOS
 * ============================================================ */

static void apply_menu_back(void) {
    if (current_state == STATE_ERROR) return;

    if (current_state == STATE_MENU_EDIT) {
        if (params[current_param_idx].id == 0) params[IDX_P00].value = 0;
        current_state = STATE_MENU_SEL;
    } else if (current_state == STATE_MENU_SEL) {
        current_state = (motor_running || output_frequency > 0.1f) ? STATE_RUN : STATE_READY;
    } else {
        current_param_idx = IDX_P00;
        current_state = STATE_MENU_SEL;
    }
}

static void apply_menu_enter_or_confirm(void) {
    if (current_state == STATE_ERROR) return;

    if (current_state == STATE_MENU_SEL) {
        if (params[current_param_idx].id == 6) err_view_offset = 0;
        temp_edit_value = params[current_param_idx].value;
        current_state = STATE_MENU_EDIT;
        return;
    }

    if (current_state == STATE_MENU_EDIT) {
        int p_id = params[current_param_idx].id;
        bool motor_active = (motor_running || output_frequency > 0.1f);

        if (p_id == 6) {
            current_state = STATE_MENU_SEL;
            return;
        }

        if (p_id == 0) {
            if (temp_edit_value == 7) {
                is_locked = !is_locked;
                nvs_save_lock();
            } else if (temp_edit_value == 101 && !motor_active) {
                request_mi_factory_reset();
                for (int i = 0; i < PARAM_COUNT; i++) {
                    if (!params[i].read_only) {
                        params[i].value = params[i].def_val;
                    }
                }
                saved_frequency = params[IDX_P20].value;
                target_frequency = params[IDX_P20].value;
                is_locked = true;
                enforce_param_coherence();
                for (int i = 0; i < PARAM_COUNT; i++) {
                    if (!params[i].read_only) nvs_save_param_by_index(i);
                    if (is_mi_synced_index(i)) params[i].pending_sync = true;
                }
                nvs_save_saved_frequency();
                nvs_save_lock();
            }
            params[IDX_P00].value = 0;
        } else if (!is_locked && !params[current_param_idx].read_only) {
            bool blocked = motor_active &&
                           (p_id == 10 || p_id == 11 || p_id == 20 || p_id == 21 ||
                            p_id == 32 || p_id == 35 || p_id == 42 || p_id == 43 ||
                            p_id == 45 || p_id == 51);
            if (!blocked) {
                int max_limit = (p_id == 32) ? params[IDX_P21].value : params[current_param_idx].max_val;
                if (temp_edit_value >= params[current_param_idx].min_val && temp_edit_value <= max_limit) {
                    if (!(p_id == 20 && temp_edit_value >= params[IDX_P21].value) &&
                        !(p_id == 21 && temp_edit_value <= params[IDX_P20].value)) {
                        params[current_param_idx].value = temp_edit_value;
                        enforce_param_coherence();
                        if (current_param_idx == IDX_P51) {
                            g_direction = exaustao_on ? (uint8_t)(params[IDX_P51].value ? 0 : 1) : (uint8_t)params[IDX_P51].value;
                        }
                        nvs_save_param_by_index(current_param_idx);
                        mark_param_pending_if_synced_to_mi(current_param_idx);
                    }
                }
            }
        }

        current_state = STATE_MENU_SEL;
        return;
    }

    current_param_idx = IDX_P00;
    current_state = STATE_MENU_SEL;
}

static void change_menu_or_frequency(int dir) {
    if (current_state == STATE_RUN || current_state == STATE_READY) {
        if (dir > 0) {
            if (target_frequency < params[IDX_P21].value) target_frequency++;
            pulse_buttons(BTN_BIT_UP);
        } else {
            if (target_frequency > params[IDX_P20].value) target_frequency--;
            pulse_buttons(BTN_BIT_DOWN);
        }
        if (params[IDX_P12].value == 1) {
            saved_frequency = target_frequency;
            nvs_save_saved_frequency();
        }
        return;
    }

    if (current_state == STATE_MENU_SEL) {
        if (dir > 0) current_param_idx = (current_param_idx + 1) % PARAM_COUNT;
        else current_param_idx = (current_param_idx - 1 + PARAM_COUNT) % PARAM_COUNT;
        return;
    }

    if (current_state == STATE_MENU_EDIT) {
        int p_id = params[current_param_idx].id;
        if (p_id == 6) {
            if (dir > 0) {
                if (err_count > 0 && err_view_offset < (int)err_count - 1) err_view_offset++;
            } else {
                if (err_count > 0 && err_view_offset > 0) err_view_offset--;
            }
            return;
        }

        bool motor_active = (motor_running || output_frequency > 0.1f);
        bool can_change = true;
        if (params[current_param_idx].read_only || (is_locked && p_id != 0)) can_change = false;
        if (motor_active &&
            (p_id == 10 || p_id == 11 || p_id == 20 || p_id == 21 ||
             p_id == 32 || p_id == 35 || p_id == 42 || p_id == 43 ||
             p_id == 45 || p_id == 51)) {
            can_change = false;
        }

        if (!can_change) return;

        if (p_id == 42) {
            if (dir > 0 && temp_edit_value < 15) temp_edit_value += 5;
            if (dir < 0 && temp_edit_value > 5) temp_edit_value -= 5;
        } else {
            int max_limit = (p_id == 32) ? params[IDX_P21].value : params[current_param_idx].max_val;
            if (dir > 0 && temp_edit_value < max_limit) temp_edit_value++;
            if (dir < 0 && temp_edit_value > params[current_param_idx].min_val) temp_edit_value--;
        }
    }
}

static void finish_prewet_now(void) {
    if (!prewet_active) return;
    prewet_active = false;
    prewet_end_us = 0;
    motor_running = true;
    system_on = true;
    current_state = STATE_RUN;
    pulse_buttons(BTN_BIT_START);
    update_display_logic();
    update_leds();
}

static void finish_dryrun_now(bool immediate_stop) {
    if (!dryrun_active) return;
    dryrun_active = false;
    dryrun_restart_pending = false;
    dryrun_end_us = 0;

    if (params[IDX_P33].value == 1 && (motor_running || output_frequency > 0.1f)) {
        motor_running = false;
        system_on = false;
        dryrun_exit_restore_pending = true;
        pulse_buttons(BTN_BIT_STOP);
        current_state = (output_frequency > 0.1f) ? STATE_RUN : STATE_READY;
        update_display_logic();
        update_leds();
        ESP_LOGW(TAG, "Secagem P31: aguardando motor parar para restaurar o sentido.");
        return;
    }

    motor_running = false;
    system_on = false;
    dryrun_exit_restore_pending = false;
    g_direction = dryrun_restore_direction;
    if (params[IDX_P12].value == 1) {
        saved_frequency = clamp_i(saved_target_before_stop, params[IDX_P20].value, params[IDX_P21].value);
        nvs_save_saved_frequency();
    }
    pulse_buttons(BTN_BIT_STOP);
    if (immediate_stop) {
        current_state = STATE_READY;
    } else {
        refresh_run_ready_state_from_output();
        current_state = STATE_READY;
    }
    update_display_logic();
    update_leds();
}

static void force_finish_all_timed_cycles(void) {
    if (prewet_active) {
        finish_prewet_now();
        return;
    }
    if (dryrun_active) {
        finish_dryrun_now(true);
        return;
    }
    if (dreno_status != DRENO_IDLE || dreno_post_wait_active) {
        handle_dreno_end();
    }
}

static void phase_service(void) {
    int64_t now = esp_timer_get_time();

    if (prewet_active) {
        if (water_shortage) {
            ESP_LOGW(TAG, "P30 interrompido por falta de água: ligando motor imediatamente.");
            finish_prewet_now();
        } else if (prewet_end_us > 0 && now >= prewet_end_us) {
            finish_prewet_now();
        }
    }

    if (dryrun_restart_pending && output_frequency <= 0.1f) {
        dryrun_restart_pending = false;
        g_direction = dryrun_pending_direction;
        motor_running = true;
        system_on = true;
        current_state = STATE_RUN;
        pulse_buttons(BTN_BIT_START);
        update_display_logic();
        update_leds();
        ESP_LOGW(TAG, "Secagem P31: motor religado em sentido invertido.");
        return;
    }

    if (dryrun_exit_restore_pending && output_frequency <= 0.1f) {
        dryrun_exit_restore_pending = false;
        g_direction = dryrun_restore_direction;
        if (params[IDX_P12].value == 1) {
            saved_frequency = clamp_i(saved_target_before_stop, params[IDX_P20].value, params[IDX_P21].value);
            nvs_save_saved_frequency();
        }
        current_state = STATE_READY;
        update_display_logic();
        update_leds();
        ESP_LOGW(TAG, "Secagem P31: sentido restaurado com o motor parado.");
        return;
    }

    if (dryrun_active && dryrun_end_us > 0 && now >= dryrun_end_us) {
        finish_dryrun_now(false);
    }

    exaustao_service();
}

static void exaustao_service(void) {
    int64_t now = esp_timer_get_time();

    if (current_state == STATE_ERROR) return;

    if (exaustao_restart_pending && output_frequency <= 0.1f) {
        exaustao_restart_pending = false;
        g_direction = exaustao_pending_direction;
        motor_running = true;
        system_on = true;
        current_state = STATE_RUN;
        pulse_buttons(BTN_BIT_START);
        if (params[IDX_P86].value > 0) {
            exaustao_end_us = now + ((int64_t)params[IDX_P86].value * 60LL * 1000000LL);
        } else {
            exaustao_end_us = 0;
        }
        update_display_logic();
        update_leds();
        ESP_LOGW(TAG, "Exaustão: motor religado em sentido invertido.");
        return;
    }

    if (exaustao_on && !exaustao_restart_pending && !exaustao_exit_pending && exaustao_end_us > 0 && now >= exaustao_end_us) {
        ESP_LOGW(TAG, "Tempo de exaustão P86 concluído.");
        request_exaustao_stop();
        return;
    }

    if (exaustao_exit_pending && output_frequency <= 0.1f) {
        exaustao_exit_pending = false;
        exaustao_on = false;
        exaustao_end_us = 0;
        bomba_on = false;
        swing_on = false;
        g_direction = (uint8_t)params[IDX_P51].value;
        target_frequency = clamp_i(exaustao_saved_frequency, params[IDX_P20].value, params[IDX_P21].value);
        motor_running = false;
        system_on = false;
        current_state = STATE_READY;
        saved_resume_exaustao_on = false;
        exaustao_pending_direction = (uint8_t)params[IDX_P51].value;
        update_display_logic();
        update_leds();
        ESP_LOGW(TAG, "Exaustão finalizada: sistema voltou para pronto.");
    }
}

static void set_motor_running_ex(bool run, bool skip_timers) {
    if (run) {
        system_on = true;
        bomba_on = saved_resume_bomba_on;
        swing_on = saved_resume_swing_on;
        exaustao_on = saved_resume_exaustao_on;
        saved_bomba_on = saved_resume_bomba_on;
        g_direction = exaustao_on ? (uint8_t)(params[IDX_P51].value ? 0 : 1) : (uint8_t)params[IDX_P51].value;
        normalize_local_modes_by_params();
        target_frequency = (params[IDX_P12].value == 1) ? saved_frequency : params[IDX_P20].value;

        dryrun_active = false;
        dryrun_restart_pending = false;
        dryrun_exit_restore_pending = false;
        dryrun_restore_direction = g_direction;
        dryrun_pending_direction = g_direction;
        dryrun_end_us = 0;
        exaustao_end_us = 0;

        if (!skip_timers && bomba_on && !exaustao_on && params[IDX_P30].value > 0) {
            prewet_active = true;
            prewet_end_us = esp_timer_get_time() + ((int64_t)params[IDX_P30].value * 60LL * 1000000LL);
            motor_running = false;
            current_state = STATE_RUN;
        } else {
            prewet_active = false;
            prewet_end_us = 0;
            motor_running = true;
            current_state = STATE_RUN;
            pulse_buttons(BTN_BIT_START);
        }
    } else {
        saved_resume_bomba_on = exaustao_on ? saved_bomba_on : bomba_on;
        exaustao_end_us = 0;
        saved_resume_swing_on = swing_on;
        saved_resume_exaustao_on = exaustao_on;
        saved_target_before_stop = clamp_i(target_frequency, params[IDX_P20].value, params[IDX_P21].value);

        if (!skip_timers && motor_running && bomba_on && !exaustao_on && params[IDX_P31].value > 0) {
            prewet_active = false;
            prewet_end_us = 0;
            dryrun_active = true;
            dryrun_restart_pending = false;
            dryrun_exit_restore_pending = false;
            dryrun_restore_direction = g_direction;
            dryrun_pending_direction = (uint8_t)(g_direction ? 0 : 1);
            dryrun_end_us = esp_timer_get_time() + ((int64_t)params[IDX_P31].value * 60LL * 1000000LL);
            bomba_on = false;
            current_state = STATE_RUN;
            if (params[IDX_P32].value == 0) {
                float hold_src = (output_frequency > 0.1f) ? output_frequency : (float)target_frequency;
                int hold_freq = (int)(hold_src + 0.5f);
                target_frequency = clamp_i(hold_freq, params[IDX_P20].value, params[IDX_P21].value);
            } else {
                target_frequency = clamp_i(params[IDX_P32].value, params[IDX_P20].value, params[IDX_P21].value);
            }
            if (params[IDX_P33].value == 1) {
                motor_running = false;
                system_on = true;
                dryrun_restart_pending = true;
                pulse_buttons(BTN_BIT_STOP);
                ESP_LOGW(TAG, "Secagem P31: parando motor para inverter o sentido.");
            }
        } else {
            prewet_active = false;
            prewet_end_us = 0;
            dryrun_active = false;
            dryrun_restart_pending = false;
            dryrun_exit_restore_pending = false;
            dryrun_end_us = 0;
            motor_running = false;
            system_on = false;
            refresh_run_ready_state_from_output();
            if (params[IDX_P12].value == 1) {
                saved_frequency = clamp_i(saved_target_before_stop, params[IDX_P20].value, params[IDX_P21].value);
                nvs_save_saved_frequency();
            }
            pulse_buttons(BTN_BIT_STOP);
        }
    }
    update_display_logic();
    update_leds();
}

static void handle_dreno_led(void) {
    if (dreno_status == DRENO_AGUARDANDO_LED) {
        dreno_status = DRENO_EM_CURSO;
        ESP_LOGW(TAG, "MI confirmou início do dreno.");
        update_leds();
    }
}

static void request_exaustao_stop(void) {
    int held_freq = (output_frequency > 0.1f) ? (int)(output_frequency + 0.5f) : target_frequency;
    held_freq = clamp_i(held_freq, params[IDX_P20].value, params[IDX_P21].value);

    exaustao_saved_frequency = held_freq;
    exaustao_restart_pending = false;
    exaustao_end_us = 0;

    if (motor_running || output_frequency > 0.1f) {
        motor_running = false;
        system_on = true;
        current_state = STATE_RUN;
        exaustao_exit_pending = true;
        pulse_buttons(BTN_BIT_STOP);
        ESP_LOGW(TAG, "Saindo da exaustão: parando motor para voltar ao pronto.");
    } else {
        exaustao_exit_pending = false;
        exaustao_on = false;
        g_direction = (uint8_t)params[IDX_P51].value;
        motor_running = false;
        system_on = false;
        current_state = STATE_READY;
        saved_resume_exaustao_on = false;
        exaustao_pending_direction = (uint8_t)params[IDX_P51].value;
        ESP_LOGW(TAG, "Saindo da exaustão: sistema voltou ao pronto.");
    }
    update_display_logic();
    update_leds();
}

static void handle_dreno_end(void) {
    if (dreno_status == DRENO_EM_CURSO || dreno_status == DRENO_AGUARDANDO_LED || dreno_post_wait_active) {
        /* Preserve the peripheral state that existed before dreno started.
         * set_motor_running_ex(false, ...) normally snapshots the CURRENT states,
         * but during dreno they are intentionally forced off. If we do not preserve
         * them here, the next start will forget climatizar/swing/exaustao. */
        bool resume_bomba = saved_resume_bomba_on;
        bool resume_swing = saved_resume_swing_on;
        bool resume_exaustao = saved_resume_exaustao_on;

        dreno_status = DRENO_IDLE;
        dreno_post_wait_active = false;
        dreno_auto_off_us = 0;
        dreno_ready_release_us = 0;
        bomba_on = false;
        swing_on = false;
        exaustao_on = false;
        prewet_active = false;
        prewet_end_us = 0;
        dryrun_active = false;
        dryrun_restart_pending = false;
        dryrun_exit_restore_pending = false;
        dryrun_restore_direction = g_direction;
        dryrun_pending_direction = g_direction;
        dryrun_end_us = 0;
        exaustao_end_us = 0;
        set_motor_running_ex(false, true);

        saved_resume_bomba_on = resume_bomba;
        saved_resume_swing_on = resume_swing;
        saved_resume_exaustao_on = resume_exaustao;
        saved_bomba_on = resume_bomba;

        current_state = STATE_READY;
        ESP_LOGW(TAG, "MI informou fim do dreno.");
        update_leds();
        update_display_logic();
    }
}

static void dreno_service(void) {
    int p80 = params[IDX_P80].value;
    int64_t now = esp_timer_get_time();

    if (dreno_status == DRENO_EM_CURSO && p80 == 2 && dreno_auto_off_us > 0 && now >= dreno_auto_off_us) {
        dreno_status = DRENO_IDLE; /* desliga saída no MI */
        dreno_auto_off_us = 0;
        dreno_post_wait_active = true;
        dreno_ready_release_us = now + ((int64_t)params[IDX_P84].value * 60LL * 1000000LL);
        ESP_LOGW(TAG, "Tempo de dreno P83 concluído. Aguardando P84 para voltar ao pronto...");
        update_leds();
        update_display_logic();
    }

    if (dreno_post_wait_active && dreno_ready_release_us > 0 && now >= dreno_ready_release_us) {
        dreno_post_wait_active = false;
        dreno_ready_release_us = 0;
        current_state = STATE_READY;
        system_on = false;
        motor_running = false;
        bomba_on = false;
        swing_on = false;
        exaustao_on = false;
        refresh_run_ready_state_from_output();
        update_leds();
        update_display_logic();
        ESP_LOGW(TAG, "Ciclo de dreno concluído. Estado pronto liberado.");
    }
}

static void handle_button_event(button_id_t id, bool long_press) {
    if (id == BTN_RESET_WIFI) {
        ESP_LOGW(TAG, "Apagando NVS e reiniciando...");
        nvs_flash_erase();
        esp_restart();
        return;
    }

    if ((exaustao_on || exaustao_restart_pending || exaustao_exit_pending) && id != BTN_EXAUSTAO && id != BTN_MAIS && id != BTN_MENOS) {
        ESP_LOGW(TAG, "Comando bloqueado: durante a exaustão, somente EXAUSTAO e ajuste de frequência podem atuar.");
        return;
    }

    if (id == BTN_SET) {
        if (long_press) apply_menu_back();
        else apply_menu_enter_or_confirm();
        update_display_logic();
        return;
    }

    if (id == BTN_MAIS) {
        change_menu_or_frequency(+1);
        update_display_logic();
        return;
    }
    if (id == BTN_MENOS) {
        change_menu_or_frequency(-1);
        update_display_logic();
        return;
    }

    if (id == BTN_ONOFF) {
        if (long_press && (prewet_active || dryrun_active || dreno_status != DRENO_IDLE || dreno_post_wait_active)) {
            force_finish_all_timed_cycles();
            return;
        }

        if (current_state == STATE_ERROR && current_error_code != E08_COMM) {
            if (params[IDX_P44].value == 0 && fault_clear_pending_ack) {
                clear_error_manual_ack();
            }
            return;
        }

        if (dreno_status != DRENO_IDLE || dreno_post_wait_active) {
            ESP_LOGW(TAG, "Comando ONOFF ignorado: ciclo de dreno em andamento.");
            return;
        }

        if (prewet_active) {
            if (long_press) {
                finish_prewet_now();
            } else {
                ESP_LOGW(TAG, "Aguardando fim do tempo de molhamento P30. Use ONOFFL para pular.");
            }
            return;
        }

        if (dryrun_active) {
            if (long_press) {
                finish_dryrun_now(true);
            } else {
                ESP_LOGW(TAG, "Aguardando fim do tempo de desligamento P31. Use ONOFFL para pular.");
            }
            return;
        }

        set_motor_running_ex(!motor_running, long_press);
        dreno_status = DRENO_IDLE;
        dreno_post_wait_active = false;
        dreno_auto_off_us = 0;
        dreno_ready_release_us = 0;
        update_leds();
        return;
    }

    if (prewet_active) {
        ESP_LOGW(TAG, "Comando bloqueado: durante o tempo de molhamento P30, somente ONOFFL pode pular o ciclo.");
        return;
    }

    if (dryrun_active) {
        ESP_LOGW(TAG, "Comando bloqueado: durante o tempo de secagem P31, somente ONOFFL pode pular o ciclo.");
        return;
    }

    if (!system_on && id != BTN_DRENO && id != BTN_EXAUSTAO) {
        ESP_LOGW(TAG, "Comando ignorado: sistema desligado.");
        return;
    }

    if (dreno_status != DRENO_IDLE || dreno_post_wait_active) {
        bool manual_dreno_toggle = (id == BTN_DRENO && params[IDX_P80].value == 1 && dreno_status != DRENO_IDLE && !dreno_post_wait_active);
        if (!manual_dreno_toggle) {
            ESP_LOGW(TAG, "Comando bloqueado: dreno ativo.");
            return;
        }
    }

    switch (id) {
        case BTN_CLIMATIZAR:
            if (params[IDX_P82].value == 0 || params[IDX_P85].value == 0) {
                bomba_on = false;
                ESP_LOGW(TAG, "Climatizar indisponível: P82=0 ou P85=0.");
            } else {
                bomba_on = true;
            }
            break;
        case BTN_VENTILAR:
            bomba_on = false;
            break;
        case BTN_SWING:
            if (params[IDX_P81].value == 0) {
                swing_on = false;
                ESP_LOGW(TAG, "Swing desabilitado: P81=0.");
            } else if (!(motor_running || output_frequency > 0.1f) || prewet_active || dryrun_active) {
                ESP_LOGW(TAG, "Swing só pode ser acionado com o motor ligado.");
            } else {
                swing_on = !swing_on;
            }
            break;
        case BTN_EXAUSTAO: {
            int held_freq = (output_frequency > 0.1f) ? (int)(output_frequency + 0.5f) : ((target_frequency > 0) ? target_frequency : saved_frequency);
            held_freq = clamp_i(held_freq, params[IDX_P20].value, params[IDX_P21].value);

            if (!exaustao_on && !exaustao_restart_pending && !exaustao_exit_pending) {
                if (params[IDX_P86].value == 0) {
                    ESP_LOGW(TAG, "Exaustão desabilitada por P86=0.");
                    break;
                }

                saved_resume_bomba_on = bomba_on;
                saved_resume_swing_on = swing_on;
                saved_resume_exaustao_on = false;
                saved_bomba_on = bomba_on;
                saved_exaustao_swing_on = swing_on;
                exaustao_saved_frequency = held_freq;

                exaustao_on = true;
                bomba_on = false;
                swing_on = false;
                target_frequency = exaustao_saved_frequency;
                exaustao_pending_direction = (uint8_t)(params[IDX_P51].value ? 0 : 1);

                if (motor_running || output_frequency > 0.1f) {
                    motor_running = false;
                    system_on = true;
                    current_state = STATE_RUN;
                    exaustao_restart_pending = true;
                    exaustao_end_us = 0;
                    pulse_buttons(BTN_BIT_STOP);
                    ESP_LOGW(TAG, "Exaustão iniciada: parando motor para reiniciar em sentido invertido.");
                } else {
                    g_direction = exaustao_pending_direction;
                    motor_running = true;
                    system_on = true;
                    current_state = STATE_RUN;
                    exaustao_restart_pending = false;
                    exaustao_end_us = esp_timer_get_time() + ((int64_t)params[IDX_P86].value * 60LL * 1000000LL);
                    pulse_buttons(BTN_BIT_START);
                    ESP_LOGW(TAG, "Exaustão iniciada: motor ligado em sentido invertido.");
                }
            } else {
                request_exaustao_stop();
            }
            break;
        }
        case BTN_DRENO: {
            int mode = params[IDX_P80].value;
            if (mode == 0) {
                ESP_LOGW(TAG, "Dreno desabilitado por P80=0.");
                return;
            }

            if (mode == 1 && dreno_status != DRENO_IDLE) {
                dreno_status = DRENO_IDLE;
                dreno_post_wait_active = false;
                dreno_auto_off_us = 0;
                dreno_ready_release_us = 0;
                system_on = false;
                motor_running = false;
                current_state = STATE_READY;
                saved_bomba_on = saved_resume_bomba_on;
                pulse_buttons(BTN_BIT_STOP);
                ESP_LOGW(TAG, "Dreno desligado manualmente por BTN_DRENO.");
                break;
            }

            if (dreno_status != DRENO_IDLE || dreno_post_wait_active) {
                ESP_LOGW(TAG, "Dreno já está em andamento.");
                return;
            }

            saved_resume_bomba_on = bomba_on;
            saved_resume_swing_on = swing_on;
            saved_resume_exaustao_on = exaustao_on;
            saved_bomba_on = bomba_on;
            bomba_on = false;
            swing_on = false;
            exaustao_on = false;
            motor_running = false;
            system_on = true;
            current_state = STATE_RUN; /* evita mostrar rdy durante o ciclo */
            pulse_buttons(BTN_BIT_STOP);
            dreno_status = DRENO_EM_CURSO;
            dreno_post_wait_active = false;
            dreno_ready_release_us = 0;

            if (mode == 2) {
                dreno_auto_off_us = esp_timer_get_time() + ((int64_t)params[IDX_P83].value * 60LL * 1000000LL);
                ESP_LOGW(TAG, "Dreno temporizado iniciado por BTN_DRENO. P83=%d min, P84=%d min.", params[IDX_P83].value, params[IDX_P84].value);
            } else {
                dreno_auto_off_us = 0;
                ESP_LOGW(TAG, "Dreno manual iniciado por BTN_DRENO.");
            }
            break;
        }
        default:
            break;
    }

    update_leds();
    update_display_logic();
}

/* ============================================================
 * CONSOLE / SERIAL MONITOR
 * ============================================================ */

static void print_help(void) {
    printf("\n=== COMANDOS IHM ===\n");
    printf("ONOFF           -> liga/desliga motor\n");
    printf("ONOFFL          -> longo pressionamento do ONOFF (pula tempos)\n");
    printf("MAIS / MENOS    -> ajuste de frequência ou navegação no menu\n");
    printf("SET             -> entra/edita/confirma menu\n");
    printf("SETL            -> longo pressionamento do SET (voltar/sair)\n");
    printf("CLIMA           -> climatizar\n");
    printf("VENT            -> ventilar\n");
    printf("SWING           -> toggle swing\n");
    printf("EXAUSTAO        -> toggle exaustão\n");
    printf("DRENO           -> aciona dreno conforme P80\n");
    printf("DL              -> simula confirmação de início do dreno pelo MI\n");
    printf("DF              -> simula fim do dreno pelo MI\n");
    printf("DIR=0|1         -> sentido FWD/REV\n");
    printf("Pxx=valor       -> escreve parâmetro\n");
    printf("MON / SIL       -> logs on/off\n");
    printf("SIMWS1/0        -> simula falta de água\n");
    printf("SIMWL1/0        -> simula perda Wi-Fi\n");
    printf("RESETWIFI       -> apaga NVS e reinicia\n");
    printf("HELP            -> mostra ajuda\n");
}

static void process_console_command(char *cmd) {
    trim_spaces(cmd);
    if (!cmd || *cmd == '\0') return;

    if (strcasecmp(cmd, "HELP") == 0) {
        print_help();
        return;
    }
    if (strcasecmp(cmd, "MON") == 0) {
        g_show_logs = true;
        printf("\n[LOGS ON]\n");
        return;
    }
    if (strcasecmp(cmd, "SIL") == 0) {
        g_show_logs = false;
        printf("\n[LOGS OFF]\n");
        return;
    }
    if (strcasecmp(cmd, "SIMWS1") == 0) { sim_ws = true;  water_shortage = true; update_display_logic(); return; }
    if (strcasecmp(cmd, "SIMWS0") == 0) { sim_ws = false; water_shortage = ((g_telemetry.status_flags & MI_STATUS_WATER_SHORTAGE) != 0u); update_display_logic(); return; }
    if (strcasecmp(cmd, "SIMWL1") == 0) { sim_wl = true;  wifi_lost = true; update_display_logic(); return; }
    if (strcasecmp(cmd, "SIMWL0") == 0) { sim_wl = false; wifi_lost = false; update_display_logic(); return; }

    if (strncasecmp(cmd, "DIR=", 4) == 0) {
        g_direction = (uint8_t)(atoi(cmd + 4) ? 1 : 0);
        params[IDX_P51].value = g_direction;
        nvs_save_param_by_index(IDX_P51);
        mark_param_pending_if_synced_to_mi(IDX_P51);
        printf("\n[DIR] %s\n", g_direction ? "REV" : "FWD");
        return;
    }

    if (strncasecmp(cmd, "P", 1) == 0) {
        unsigned id = 0;
        int value = 0;
        if (sscanf(cmd, "P%u=%d", &id, &value) == 2) {
            for (int i = 0; i < PARAM_COUNT; i++) {
                if ((unsigned)params[i].id == id) {
                    if (params[i].read_only) {
                        printf("\n[P%02u] somente leitura\n", id);
                        return;
                    }
                    params[i].value = clamp_i(value, params[i].min_val, params[i].max_val);
                    enforce_param_coherence();
                    nvs_save_param_by_index(i);
                    mark_param_pending_if_synced_to_mi(i);
                    printf("\n[P%02u] = %d\n", id, params[i].value);
                    update_display_logic();
                    return;
                }
            }
            printf("\n[P%02u] não encontrado\n", id);
            return;
        }
    }

    if (strcasecmp(cmd, "ONOFFL") == 0) {
        handle_button_event(BTN_ONOFF, true);
        return;
    }
    if (strcasecmp(cmd, "ONOFF") == 0 || strcasecmp(cmd, "POWER") == 0 || strcasecmp(cmd, "S") == 0) {
        handle_button_event(BTN_ONOFF, false);
        return;
    }
    if (strcasecmp(cmd, "MAIS") == 0 || strcmp(cmd, "+") == 0) {
        handle_button_event(BTN_MAIS, false);
        return;
    }
    if (strcasecmp(cmd, "MENOS") == 0 || strcmp(cmd, "-") == 0) {
        handle_button_event(BTN_MENOS, false);
        return;
    }
    if (strcasecmp(cmd, "SET") == 0 || strcasecmp(cmd, "E") == 0) {
        handle_button_event(BTN_SET, false);
        return;
    }
    if (strcasecmp(cmd, "SETL") == 0 || strcasecmp(cmd, "M") == 0) {
        handle_button_event(BTN_SET, true);
        return;
    }
    if (strcasecmp(cmd, "CLIMA") == 0 || strcasecmp(cmd, "CLIMATIZAR") == 0) {
        handle_button_event(BTN_CLIMATIZAR, false);
        return;
    }
    if (strcasecmp(cmd, "VENT") == 0 || strcasecmp(cmd, "VENTILAR") == 0) {
        handle_button_event(BTN_VENTILAR, false);
        return;
    }
    if (strcasecmp(cmd, "SWING") == 0) {
        handle_button_event(BTN_SWING, false);
        return;
    }
    if (strcasecmp(cmd, "EXAUSTAO") == 0 || strcasecmp(cmd, "EX") == 0) {
        handle_button_event(BTN_EXAUSTAO, false);
        return;
    }
    if (strcasecmp(cmd, "DRENO") == 0) {
        handle_button_event(BTN_DRENO, false);
        return;
    }
    if (strcasecmp(cmd, "DL") == 0 || strcasecmp(cmd, "DRENO_LED") == 0) {
        handle_dreno_led();
        return;
    }
    if (strcasecmp(cmd, "DF") == 0 || strcasecmp(cmd, "DRENO_FIM") == 0) {
        handle_dreno_end();
        return;
    }
    if (strcasecmp(cmd, "RESETWIFI") == 0) {
        handle_button_event(BTN_RESET_WIFI, false);
        return;
    }

    printf("\n[CMD] desconhecido: %s\n", cmd);
}

static void console_task(void *arg) {
    (void)arg;
    char line[CONSOLE_BUF_SZ];
    int rx_pos = 0;
    setvbuf(stdin, NULL, _IONBF, 0);

    print_help();

    while (1) {
        int c = fgetc(stdin);
        if (c != EOF) {
            if (c != '\r') {
                putchar(c);
                fflush(stdout);
            }

            if (c == '\n' || c == '\r') {
                if (rx_pos > 0) {
                    line[rx_pos] = '\0';
                    process_console_command(line);
                    rx_pos = 0;
                }
            } else if ((c == 0x08 || c == 0x7F)) {
                if (rx_pos > 0) rx_pos--;
            } else if (rx_pos < (int)sizeof(line) - 1) {
                line[rx_pos++] = (char)c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ============================================================
 * INIT
 * ============================================================ */

void app_main(void) {
    uint64_t display_mask = (1ULL << SEG_DP);
    for (int i = 0; i < 7; i++) display_mask |= (1ULL << segment_pins[i]);
    for (int i = 0; i < TOTAL_DIGITS; i++) display_mask |= (1ULL << digit_pins[i]);

    gpio_config_t io_display = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = display_mask,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_display));

    uint64_t leds_mask = 0;
    for (int i = 0; i < 5; i++) leds_mask |= (1ULL << led_pins[i]);
    gpio_config_t io_leds = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = leds_mask,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_leds));
    update_leds();

    nvs_init_and_open();
    nvs_load_all();

    for (int i = 0; i < PARAM_COUNT; i++) {
        if (is_mi_synced_index(i)) {
            params[i].pending_sync = true;
        }
    }

    g_frame_q = xQueueCreate(10, sizeof(frame_t));
    init_rs485_uart();

    current_state = STATE_READY;
    current_error_code = 0;
    system_on = false;
    motor_running = false;
    g_direction = (uint8_t)params[IDX_P51].value;
    bomba_on = (params[IDX_P82].value != 0 && params[IDX_P85].value != 0);
    saved_bomba_on = bomba_on;
    saved_resume_bomba_on = bomba_on;
    saved_resume_swing_on = swing_on;
    saved_resume_exaustao_on = exaustao_on;
    exaustao_restart_pending = false;
    exaustao_exit_pending = false;
    target_frequency = clamp_i(saved_frequency, params[IDX_P20].value, params[IDX_P21].value);
    output_frequency = 0.0f;
    prewet_active = false;
    dryrun_active = false;
    prewet_end_us = 0;
    dryrun_end_us = 0;
    g_telemetry.status_flags = 0;
    update_display_logic();

    const esp_timer_create_args_t timer_args = {
        .callback = &multiplex_timer_callback,
        .name = "mux7seg"
    };
    esp_timer_handle_t timer_handle;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handle, MULTIPLEX_INTERVAL_US));

    xTaskCreate(rs485_rx_task,       "rs485_rx",   4096, NULL, 10, NULL);
    xTaskCreate(ihm_sync_task,       "ihm_sync",   4096, NULL,  8, NULL);
    xTaskCreate(status_watchdog_task,"wd_status",  3072, NULL,  7, NULL);
    xTaskCreate(mi_fault_task,       "mi_fault",   3072, NULL,  6, NULL);
    xTaskCreate(console_task,        "console",    4096, NULL,  5, NULL);
    xTaskCreate(sim_task,            "sim_task",   3072, NULL,  4, NULL);
}
