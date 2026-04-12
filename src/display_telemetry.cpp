#include "display_telemetry.h"
#include "driver/twai.h"
#include <Arduino.h>
#include <string.h>

DisplayCanState g_can_state = {};

// ── Helpers (mirrors bms_telemetry.cpp) ──────────────────────────────────────

static float unpack_float(const uint8_t *buf, int offset) {
    uint32_t raw = ((uint32_t)buf[offset + 0] << 24)
                 | ((uint32_t)buf[offset + 1] << 16)
                 | ((uint32_t)buf[offset + 2] <<  8)
                 | ((uint32_t)buf[offset + 3]      );
    float val;
    memcpy(&val, &raw, sizeof(val));
    return val;
}

static void pack_u16(uint8_t *buf, int offset, uint16_t val) {
    buf[offset + 0] = (val >> 8) & 0xFF;
    buf[offset + 1] = (val     ) & 0xFF;
}

static bool twai_tx(uint32_t id, const uint8_t *data, uint8_t dlc) {
    twai_message_t msg = {};
    msg.identifier       = id;
    msg.data_length_code = dlc;
    msg.extd             = 0;
    msg.rtr              = 0;
    memcpy(msg.data, data, dlc);

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(5));
    if (err != ESP_OK) {
        static uint32_t s_last_err_ms = 0;
        if ((millis() - s_last_err_ms) >= 5000) {
            s_last_err_ms = millis();
            Serial.printf("[disp-telem] TX failed id=0x%03X err=%s\n",
                          id, esp_err_to_name(err));
        }
        return false;
    }
    return true;
}

// ── Init ─────────────────────────────────────────────────────────────────────

void telemetry_display_init(void) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_GPIO,
        (gpio_num_t)CAN_RX_GPIO,
        TWAI_MODE_NORMAL
    );
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

    // Accept only cell voltage frames (0x100–0x10F) and fault frame (0x130)
    // TWAI hardware filter: code + mask, both 11-bit left-aligned in 32-bit
    // Accept 0x100-0x10F: code=0x100<<21, mask passes lower nibble
    // Since we need two disjoint ranges we use software filtering and
    // accept-all in hardware — simpler and the frame rate is low.
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        Serial.printf("[disp-telem] TWAI install failed: %s\n", esp_err_to_name(err));
        return;
    }
    err = twai_start();
    if (err != ESP_OK) {
        Serial.printf("[disp-telem] TWAI start failed: %s\n", esp_err_to_name(err));
        return;
    }
    Serial.println("[disp-telem] TWAI started (TX=GPIO13, RX=GPIO12, 500kbps)");
}

// ── Frame decoder ─────────────────────────────────────────────────────────────

static void handle_rx_frame(const twai_message_t *msg) {
    uint32_t id = msg->identifier;

    // Cell voltage frames: 0x100 to 0x10F (up to 10 frames, 2 floats each)
    if (id >= CAN_ID_CELLS_BASE && id <= (CAN_ID_CELLS_BASE + 0x0F)) {
        if (msg->data_length_code < 8) return;
        uint8_t frame_idx = id - CAN_ID_CELLS_BASE;  // 0..9 for IC1, 10..19 for IC2 etc.
        uint8_t cell_a    = frame_idx * 2;
        uint8_t cell_b    = cell_a + 1;

        if (cell_b < 20) {   // guard against overflow
            g_can_state.cell_v[cell_a] = unpack_float(msg->data, 0);
            g_can_state.cell_v[cell_b] = unpack_float(msg->data, 4);
            if (cell_b + 1 > g_can_state.cell_count)
                g_can_state.cell_count = cell_b + 1;
        }
        return;
    }

    // Fault frame: 0x130
    if (id == CAN_ID_FAULT) {
        if (msg->data_length_code < 8) return;
        g_can_state.fault_code  = ((uint32_t)msg->data[0] << 24)
                                 | ((uint32_t)msg->data[1] << 16)
                                 | ((uint32_t)msg->data[2] <<  8)
                                 | ((uint32_t)msg->data[3]      );
        g_can_state.fault_state = ((uint32_t)msg->data[4] << 24)
                                 | ((uint32_t)msg->data[5] << 16)
                                 | ((uint32_t)msg->data[6] <<  8)
                                 | ((uint32_t)msg->data[7]      );
        g_can_state.fault_active = true;
        Serial.printf("[disp-telem] Fault received: code=%lu state=%lu\n",
                      g_can_state.fault_code, g_can_state.fault_state);
        return;
    }
}

// ── Public task functions (defined here, declared extern in main .cpp) ────────

// taskCanRx: blocks on twai_receive, decodes frames into g_can_state
void taskCanRx(void *pvParameters) {
    twai_message_t msg;
    for (;;) {
        // Block indefinitely until a frame arrives
        if (twai_receive(&msg, portMAX_DELAY) == ESP_OK) {
            handle_rx_frame(&msg);
        }
    }
}

// Called from taskThrottle — sends raw ADC reading over CAN
void telemetry_send_throttle(uint16_t raw_adc) {
    uint8_t buf[2];
    pack_u16(buf, 0, raw_adc);
    twai_tx(CAN_ID_THROTTLE, buf, 2);
}