#pragma once
#include <stdint.h>

// CAN frame IDs (must match bms_telemetry.cpp)
#define CAN_ID_CELLS_BASE   0x100
#define CAN_ID_FAULT        0x130
#define CAN_ID_THROTTLE     0x200   // Display → motor controller

#define CAN_TX_GPIO         14
#define CAN_RX_GPIO         27

// Parsed battery state populated by taskCanRx, consumed by taskAux
typedef struct {
    float    cell_v[20];   // Up to 20 cells (10 frames × 2 cells)
    uint8_t  cell_count;   // How many cells actually received
    uint32_t fault_code;
    uint32_t fault_state;
    bool     fault_active;
} DisplayCanState;

// Shared state — written by taskCanRx, read by taskAux (guarded by tftMutex)
extern DisplayCanState g_can_state;

void telemetry_display_init(void);
void telemetry_send_throttle(uint16_t raw_adc); 
void taskCanRx(void *pvParameters);