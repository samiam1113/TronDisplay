#pragma once
#include <stdint.h>
#include <stdbool.h>

#define CAN_TX_GPIO   25
#define CAN_RX_GPIO   26

// ── CAN IDs — Transmit ───────────────────────────────────────────────────────
#define CAN_ID_SPEED_MODE   0x200
#define CAN_ID_FAULT        0x130   // your BMS fault frame, keep as-is

// ── VESC controller ID (set in VESC Tool → App Settings → Controller ID) ─────
#define VESC_CONTROLLER_ID  74

// ── Shared state ─────────────────────────────────────────────────────────────
typedef struct {
    // VESC STATUS (decoded from native CAN broadcast)
    int32_t  vesc_rpm;
    float    vesc_current;      // A
    float    vesc_duty;         // 0.0–1.0
    float    vesc_input_voltage; // V  (from STATUS_5)
    bool     vesc_updated;

    // Fault
    uint32_t fault_code;
    uint32_t fault_state;
    bool     fault_active;

    // Freshness flags
    bool     speed_updated;
    bool     voltage_updated;

    // Derived — written by taskAux after decoding rpm
    float    speed_mph;
    float    pack_voltage;
} DisplayCanState;

extern DisplayCanState g_can_state;

// ── Motor/wheel constants — adjust to your hardware ──────────────────────────
#define MOTOR_POLE_PAIRS    7        // electrical pole pairs
#define WHEEL_CIRC_M        1.35f    // wheel circumference in metres
#define GEAR_RATIO          1.0f     // motor:wheel, 1.0 = direct drive

void telemetry_display_init(void);
void taskCanRx(void *pvParameters);
void telemetry_send_speed_mode(uint8_t mode);