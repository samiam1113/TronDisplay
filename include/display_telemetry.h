#pragma once
#include <stdint.h>
#include <stdbool.h>

// ── GPIO ─────────────────────────────────────────────────────────────────────
#define CAN_TX_GPIO   21      // update to your actual wiring
#define CAN_RX_GPIO   22      // update to your actual wiring

// ── CAN IDs — Receive ────────────────────────────────────────────────────────
#define CAN_ID_SPEED        0x110   // float  mph,         4 bytes
#define CAN_ID_ACCEL        0x111   // float  m/s²,        4 bytes
#define CAN_ID_PACK_VOLTAGE 0x112   // float  volts,       4 bytes
#define CAN_ID_FAULT        0x130   // u32 code | u32 state, 8 bytes

// ── CAN IDs — Transmit ───────────────────────────────────────────────────────
#define CAN_ID_SPEED_MODE   0x200   // u8 mode (0=LOW 1=MED 2=HIGH), 1 byte

// ── Shared state populated by taskCanRx ──────────────────────────────────────
typedef struct {
    // Received values
    float    speed_mph;         // latest speed from motor controller
    float    accel_mss;         // latest acceleration
    float    pack_voltage;      // latest pack voltage

    // Fault
    uint32_t fault_code;
    uint32_t fault_state;
    bool     fault_active;

    // Freshness flags — set true on RX, cleared by display after reading
    bool     speed_updated;
    bool     accel_updated;
    bool     voltage_updated;
} DisplayCanState;

extern DisplayCanState g_can_state;

// ── Public API ────────────────────────────────────────────────────────────────
void telemetry_display_init(void);
void taskCanRx(void *pvParameters);
void telemetry_send_speed_mode(uint8_t mode);