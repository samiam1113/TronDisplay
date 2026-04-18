#include "display_telemetry.h"
#include "driver/twai.h"
#include <Arduino.h>
#include <string.h>

DisplayCanState g_can_state = {};

// ── Helpers ───────────────────────────────────────────────────────────────────

static float unpack_float(const uint8_t *buf, int offset) {
    uint32_t raw = ((uint32_t)buf[offset + 0] << 24)
                 | ((uint32_t)buf[offset + 1] << 16)
                 | ((uint32_t)buf[offset + 2] <<  8)
                 | ((uint32_t)buf[offset + 3]      );
    float val;
    memcpy(&val, &raw, sizeof(val));
    return val;
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

// ── Init ──────────────────────────────────────────────────────────────────────

void telemetry_display_init(void) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_GPIO,
        (gpio_num_t)CAN_RX_GPIO,
        TWAI_MODE_NORMAL
    );
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

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
    Serial.printf("[disp-telem] TWAI started (TX=GPIO%d, RX=GPIO%d, 500kbps)\n",
                  CAN_TX_GPIO, CAN_RX_GPIO);
}

// ── Frame decoder ─────────────────────────────────────────────────────────────

static void handle_rx_frame(const twai_message_t *msg) {
    uint32_t id = msg->identifier;

    if (id == CAN_ID_SPEED) {
        if (msg->data_length_code < 4) return;
        g_can_state.speed_mph    = unpack_float(msg->data, 0);
        g_can_state.speed_updated = true;
        return;
    }

    if (id == CAN_ID_ACCEL) {
        if (msg->data_length_code < 4) return;
        g_can_state.accel_mss    = unpack_float(msg->data, 0);
        g_can_state.accel_updated = true;
        return;
    }

    if (id == CAN_ID_PACK_VOLTAGE) {
        if (msg->data_length_code < 4) return;
        g_can_state.pack_voltage    = unpack_float(msg->data, 0);
        g_can_state.voltage_updated = true;
        return;
    }

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
        Serial.printf("[disp-telem] Fault: code=%lu state=%lu\n",
                      g_can_state.fault_code, g_can_state.fault_state);
        return;
    }
}

// ── Tasks / public functions ──────────────────────────────────────────────────

void taskCanRx(void *pvParameters) {
    twai_message_t msg;
    for (;;) {
        if (twai_receive(&msg, portMAX_DELAY) == ESP_OK) {
            handle_rx_frame(&msg);
        }
    }
}

// Called from taskSpeedMode — transmits current mode over CAN 0x200
// mode: 0 = LOW, 1 = MED, 2 = HIGH
void telemetry_send_speed_mode(uint8_t mode) {
    twai_tx(CAN_ID_SPEED_MODE, &mode, 1);
}