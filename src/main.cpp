#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "display_telemetry.h"

TFT_eSPI tft = TFT_eSPI();

// ── Screen dimensions ─────────────────────────────────────────────────────────
#define SCREEN_W 480
#define SCREEN_H 320

// ── Colors ────────────────────────────────────────────────────────────────────
#define COL_BG       0x0000
#define COL_ARC_BG   0x1082
#define COL_ARC_FG   0x07FF
#define COL_SPEED    0x07FF
#define COL_UNIT     0x7BEF
#define COL_BATT_FG  0x07FF
#define COL_BATT_LOW 0xF800
#define COL_ODO      0x7BEF
#define COL_WHITE    0xFFFF
#define COL_ACCEL    0x07E0   // green for acceleration readout

// ── Arc gauge ─────────────────────────────────────────────────────────────────
#define ARC_CX      240
#define ARC_CY      210
#define ARC_R_OUT   195
#define ARC_R_IN    165
#define SPEED_MAX   100

// ── Speed mode switch pins ────────────────────────────────────────────────────
#define SPEED_PIN_LOW   25    // update to your actual GPIO
#define SPEED_PIN_MED   26
#define SPEED_PIN_HIGH  27

// ── Speed mode transmit interval ─────────────────────────────────────────────
#define SPEED_MODE_TX_MS  200   // send mode frame every 200 ms

typedef enum {
    SPEED_MODE_LOW  = 0,
    SPEED_MODE_MED  = 1,
    SPEED_MODE_HIGH = 2,
} SpeedMode;

// ── FreeRTOS handles ──────────────────────────────────────────────────────────
static QueueHandle_t     speedQueue;
static SemaphoreHandle_t tftMutex;

// ── Drawing helpers ───────────────────────────────────────────────────────────

void drawArcSegment(int cx, int cy, int rIn, int rOut,
                    float startDeg, float endDeg, uint16_t color) {
    for (float a = startDeg; a <= endDeg; a += 1.0f) {
        float rad  = a * DEG_TO_RAD;
        float cosA = cos(rad), sinA = sin(rad);
        tft.drawLine(cx + rIn  * cosA, cy + rIn  * sinA,
                     cx + rOut * cosA, cy + rOut * sinA, color);
    }
}

void drawArcBackground() {
    drawArcSegment(ARC_CX, ARC_CY, ARC_R_IN, ARC_R_OUT, 180, 360, COL_ARC_BG);

    for (int s = 0; s <= SPEED_MAX; s += 10) {
        float deg = (180.0f + (float)s / SPEED_MAX * 180.0f) * DEG_TO_RAD;
        tft.drawLine(ARC_CX + (ARC_R_IN  - 5) * cos(deg),
                     ARC_CY + (ARC_R_IN  - 5) * sin(deg),
                     ARC_CX + (ARC_R_OUT + 5) * cos(deg),
                     ARC_CY + (ARC_R_OUT + 5) * sin(deg), COL_WHITE);
    }

    float d0   = 180.0f * DEG_TO_RAD;
    float d100 = 360.0f * DEG_TO_RAD;
    tft.setTextColor(COL_UNIT, COL_BG);
    tft.setTextSize(1);
    tft.setCursor(ARC_CX + (ARC_R_IN - 20) * cos(d0)   - 4,
                  ARC_CY + (ARC_R_IN - 20) * sin(d0)   - 4);
    tft.print("0");
    tft.setCursor(ARC_CX + (ARC_R_IN - 20) * cos(d100) - 8,
                  ARC_CY + (ARC_R_IN - 20) * sin(d100) - 4);
    tft.print("100");
}

void updateArc(int speed) {
    float frac    = constrain((float)speed / SPEED_MAX, 0.0f, 1.0f);
    float fillEnd = 180.0f + frac * 180.0f;
    if (frac > 0)
        drawArcSegment(ARC_CX, ARC_CY, ARC_R_IN, ARC_R_OUT, 180, fillEnd, COL_ARC_FG);
    if (fillEnd < 360)
        drawArcSegment(ARC_CX, ARC_CY, ARC_R_IN, ARC_R_OUT, fillEnd, 360, COL_ARC_BG);
}

void drawSpeed(int speed) {
    tft.fillRect(170, 110, 160, 90, COL_BG);
    tft.setTextColor(COL_SPEED, COL_BG);
    tft.setTextSize(7);
    char buf[4];
    sprintf(buf, "%d", speed);
    tft.setCursor(240 - (strlen(buf) * 42) / 2, 115);
    tft.print(buf);
    tft.setTextColor(COL_UNIT, COL_BG);
    tft.setTextSize(2);
    tft.setCursor(290, 190);
    tft.print("mph");
}

void drawAccel(float accel) {
    tft.fillRect(5, 270, 140, 16, COL_BG);
    tft.setTextColor(COL_ACCEL, COL_BG);
    tft.setTextSize(1);
    char buf[16];
    sprintf(buf, "%.2f m/s2", accel);
    tft.setCursor(5, 270);
    tft.print(buf);
}

void drawPackVoltage(float volts) {
    tft.fillRect(5, 255, 140, 16, COL_BG);
    tft.setTextColor(COL_WHITE, COL_BG);
    tft.setTextSize(1);
    char buf[16];
    sprintf(buf, "%.1fV", volts);
    tft.setCursor(5, 255);
    tft.print(buf);
}

// Battery widget — driven by pack voltage → SOC lookup
static const float P42A_V[]   = { 2.80, 3.00, 3.30, 3.50, 3.60, 3.65,
                                   3.70, 3.75, 3.80, 3.90, 4.00, 4.10, 4.20 };
static const float P42A_SOC[] = { 0.00, 0.02, 0.06, 0.12, 0.20, 0.27,
                                   0.40, 0.55, 0.68, 0.80, 0.90, 0.97, 1.00 };
#define P42A_POINTS 13

// Derive per-cell voltage from pack voltage assuming 20S configuration
// Update CELL_COUNT to match your actual series cell count
#define CELL_COUNT 20

float voltage_to_soc(float cell_v) {
    if (cell_v <= P42A_V[0])              return 0.0f;
    if (cell_v >= P42A_V[P42A_POINTS-1]) return 100.0f;
    for (int i = 0; i < P42A_POINTS - 1; i++) {
        if (cell_v >= P42A_V[i] && cell_v < P42A_V[i + 1]) {
            float t = (cell_v - P42A_V[i]) / (P42A_V[i + 1] - P42A_V[i]);
            return (P42A_SOC[i] + t * (P42A_SOC[i + 1] - P42A_SOC[i])) * 100.0f;
        }
    }
    return 0.0f;
}

void drawBattery(float pct) {
    int bx = 430, by = 10, bw = 30, bh = 80;
    tft.drawRect(bx, by, bw, bh, COL_WHITE);
    tft.fillRect(bx + 8, by - 5, 14, 6, COL_WHITE);
    int fillH = (int)((pct / 100.0f) * (bh - 4));
    uint16_t fillColor = (pct < 20) ? COL_BATT_LOW : COL_BATT_FG;
    tft.fillRect(bx + 2, by + 2, bw - 4, bh - 4, COL_BG);
    tft.fillRect(bx + 2, by + 2 + (bh - 4 - fillH), bw - 4, fillH, fillColor);
    tft.fillRect(bx - 10, by + bh + 4, 50, 16, COL_BG);
    tft.setTextColor(COL_WHITE, COL_BG);
    tft.setTextSize(1);
    char buf[8];
    sprintf(buf, "%.0f%%", pct);
    tft.setCursor(bx + 2, by + bh + 6);
    tft.print(buf);
}

// Speed mode label — top-left corner
void drawSpeedMode(SpeedMode mode) {
    tft.fillRect(5, 5, 60, 16, COL_BG);
    tft.setTextSize(1);
    switch (mode) {
        case SPEED_MODE_LOW:
            tft.setTextColor(TFT_GREEN, COL_BG);
            tft.setCursor(5, 5);
            tft.print("MODE: LOW");
            break;
        case SPEED_MODE_MED:
            tft.setTextColor(TFT_YELLOW, COL_BG);
            tft.setCursor(5, 5);
            tft.print("MODE: MED");
            break;
        case SPEED_MODE_HIGH:
            tft.setTextColor(TFT_RED, COL_BG);
            tft.setCursor(5, 5);
            tft.print("MODE: HIGH");
            break;
    }
}

void drawStaticUI() {
    tft.fillScreen(COL_BG);
    drawArcBackground();
}

// ── Speed mode switch reader ──────────────────────────────────────────────────

static SpeedMode read_speed_mode(void) {
    if (digitalRead(SPEED_PIN_HIGH) == LOW) return SPEED_MODE_HIGH;
    if (digitalRead(SPEED_PIN_MED)  == LOW) return SPEED_MODE_MED;
    return SPEED_MODE_LOW;   // safe default
}

// ── FreeRTOS tasks ────────────────────────────────────────────────────────────

// Reads the 3-position switch, transmits mode over CAN, updates display label
void taskSpeedMode(void *pvParameters) {
    SpeedMode prevMode = (SpeedMode)-1;   // force draw on first iteration
    for (;;) {
        SpeedMode mode = read_speed_mode();

        // Only retransmit and redraw when mode actually changes
        if (mode != prevMode) {
            telemetry_send_speed_mode((uint8_t)mode);

            xSemaphoreTake(tftMutex, portMAX_DELAY);
            drawSpeedMode(mode);
            xSemaphoreGive(tftMutex);

            prevMode = mode;
        } else {
            // Periodic heartbeat so the motor controller knows we're alive
            telemetry_send_speed_mode((uint8_t)mode);
        }

        vTaskDelay(pdMS_TO_TICKS(SPEED_MODE_TX_MS));
    }
}

// Receives speed from CAN queue, updates arc gauge
void taskDisplay(void *pvParameters) {
    int prevSpeed = -1;
    int speed     =  0;

    for (;;) {
        if (xQueueReceive(speedQueue, &speed, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (speed != prevSpeed) {
                xSemaphoreTake(tftMutex, portMAX_DELAY);
                updateArc(speed);
                drawSpeed(speed);
                xSemaphoreGive(tftMutex);
                prevSpeed = speed;
            }
        }
    }
}

// Polls g_can_state for speed, accel, voltage, fault — updates display
void taskAux(void *pvParameters) {
    for (;;) {
        xSemaphoreTake(tftMutex, portMAX_DELAY);

        // Speed → queue for arc gauge (taskDisplay handles the arc)
        if (g_can_state.speed_updated) {
            int speed = (int)constrain(g_can_state.speed_mph, 0.0f, (float)SPEED_MAX);
            xQueueOverwrite(speedQueue, &speed);
            g_can_state.speed_updated = false;
        }

        // Pack voltage → SOC → battery widget
        if (g_can_state.voltage_updated) {
            float cellV  = g_can_state.pack_voltage / (float)CELL_COUNT;
            float battPct = voltage_to_soc(cellV);
            drawBattery(battPct);
            drawPackVoltage(g_can_state.pack_voltage);
            g_can_state.voltage_updated = false;
        }

        // Acceleration readout
        if (g_can_state.accel_updated) {
            drawAccel(g_can_state.accel_mss);
            g_can_state.accel_updated = false;
        }

        // Fault indicator
        if (g_can_state.fault_active) {
            tft.setTextColor(TFT_RED, COL_BG);
            tft.setTextSize(1);
            tft.setCursor(5, 20);
            tft.printf("FAULT %lu", g_can_state.fault_code);
        }

        xSemaphoreGive(tftMutex);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ── Setup ─────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.printf("[BOOT] Reset reason: %d\n", esp_reset_reason());

    pinMode(SPEED_PIN_LOW,  INPUT_PULLUP);
    pinMode(SPEED_PIN_MED,  INPUT_PULLUP);
    pinMode(SPEED_PIN_HIGH, INPUT_PULLUP);

    tft.init();
    tft.setRotation(1);
    drawStaticUI();
    drawBattery(0);
    drawSpeed(0);

    telemetry_display_init();   // start TWAI

    speedQueue = xQueueCreate(1, sizeof(int));
    tftMutex   = xSemaphoreCreateMutex();
    configASSERT(speedQueue != NULL);
    configASSERT(tftMutex  != NULL);

    // Core 0: CAN RX + speed mode TX (no display access)
    xTaskCreatePinnedToCore(taskCanRx,    "CanRx",     8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(taskSpeedMode,"SpeedMode", 2048, NULL, 2, NULL, 0);

    // Core 1: display tasks only
    xTaskCreatePinnedToCore(taskDisplay,  "Display",   8192, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(taskAux,      "Aux",       4096, NULL, 1, NULL, 1);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}