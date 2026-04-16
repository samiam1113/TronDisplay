#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "display_telemetry.h"
#include "esp_task_wdt.h"
#include "esp_system.h"


TFT_eSPI tft = TFT_eSPI();

// Screen dimensions
#define SCREEN_W 480
#define SCREEN_H 320

// Colors
#define COL_BG       0x0000
#define COL_ARC_BG   0x1082
#define COL_ARC_FG   0x07FF
#define COL_SPEED    0x07FF
#define COL_UNIT     0x7BEF
#define COL_BATT_BG  0x2104
#define COL_BATT_FG  0x07FF
#define COL_BATT_LOW 0xF800
#define COL_ODO      0x7BEF
#define COL_TURN_ON  0x07FF
#define COL_TURN_OFF 0x2104
#define COL_WHITE    0xFFFF

// Arc gauge
#define ARC_CX      240
#define ARC_CY      210
#define ARC_R_OUT   195
#define ARC_R_IN    165
#define ARC_START   0
#define ARC_END     180
#define SPEED_MAX   100

// Throttle
#define THROTTLE_PIN  13
#define THROTTLE_MIN  500
#define THROTTLE_MAX  3200

// Three-position switch
#define SPEED_PIN_LOW  25   // needs to be updated based on schematic
#define SPEED_PIN_MED  26   
#define SPEED_PIN_HIGH 27   

// Throttle ADC ceiling for each mode (out of THROTTLE_MAX = 3200)
#define THROTTLE_CAP_LOW  ((int)(THROTTLE_MAX * 0.30f))   // 30% 
#define THROTTLE_CAP_MED  ((int)(THROTTLE_MAX * 0.60f))   // 60% 
#define THROTTLE_CAP_HIGH  THROTTLE_MAX                    // 100% 

typedef enum {
    SPEED_MODE_LOW  = 0,
    SPEED_MODE_MED  = 1,
    SPEED_MODE_HIGH = 2,
} SpeedMode;

// FreeRTOS handles
static QueueHandle_t    speedQueue;
static SemaphoreHandle_t tftMutex;

// Shared aux state 
static float simBattery  = 75.0f;
static float simOdometer = 7028.3f;
static bool  turnLeft    = false;
static bool  turnRight   = false;

// ── Drawing helpers for image initialization and updates ────────────────────────────────────────────────

void drawArcSegment(int cx, int cy, int rIn, int rOut,
                    float startDeg, float endDeg, uint16_t color) {
  for (float a = startDeg; a <= endDeg; a += 1.0f) {
    float rad = a * DEG_TO_RAD;
    float cosA = cos(rad), sinA = sin(rad);
    tft.drawLine(cx + rIn  * cosA, cy + rIn  * sinA,
                 cx + rOut * cosA, cy + rOut * sinA, color);
  }
}

void drawArcBackground() {
  drawArcSegment(ARC_CX, ARC_CY, ARC_R_IN, ARC_R_OUT, 180, 360, COL_ARC_BG);

  for (int s = 0; s <= SPEED_MAX; s += 10) {
    float deg = (180 + (float)s / SPEED_MAX * 180) * DEG_TO_RAD;
    tft.drawLine(ARC_CX + (ARC_R_IN  - 5) * cos(deg),
                 ARC_CY + (ARC_R_IN  - 5) * sin(deg),
                 ARC_CX + (ARC_R_OUT + 5) * cos(deg),
                 ARC_CY + (ARC_R_OUT + 5) * sin(deg), COL_WHITE);
  }

  float d0   = (180 + ARC_START)           * DEG_TO_RAD;
  float d100 = (180 + ARC_START + ARC_END) * DEG_TO_RAD;
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
  float frac     = constrain((float)speed / SPEED_MAX, 0.0f, 1.0f);
  float fillEnd  = 180 + frac * 180;
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

void drawOdometer(float odo) {
  tft.fillRect(150, 285, 180, 18, COL_BG);
  tft.setTextColor(COL_ODO, COL_BG);
  tft.setTextSize(2);
  char buf[16];
  sprintf(buf, "%.1f mi", odo);
  tft.setCursor(240 - (strlen(buf) * 12) / 2, 286);
  tft.print(buf);
}

void drawStaticUI() {
  tft.fillScreen(COL_BG);
  tft.setTextColor(COL_ODO, COL_BG);
  tft.setTextSize(1);
  tft.setCursor(5, 300);
  tft.print("ODO");
  drawArcBackground();
}

static SpeedMode read_speed_mode(void) {
    if (digitalRead(SPEED_PIN_HIGH)  == LOW) return SPEED_MODE_HIGH;
    if (digitalRead(SPEED_PIN_MED)  == LOW) return SPEED_MODE_MED;
    return SPEED_MODE_LOW;   // default to LOW if no pin is pulled low
}

// ── FreeRTOS tasks ────────────────────────────────────────────────────────────

void taskThrottle(void *pvParameters) {
    for (;;) {
        uint16_t raw = (uint16_t)analogRead(THROTTLE_PIN);

        // Apply ADC ceiling based on speed mode switch
        SpeedMode mode = read_speed_mode();
        int cap;
        switch (mode) {
            case SPEED_MODE_LOW:  cap = THROTTLE_CAP_LOW;  break;
            case SPEED_MODE_MED:  cap = THROTTLE_CAP_MED;  break;
            case SPEED_MODE_HIGH: cap = THROTTLE_CAP_HIGH; break;
            default:              cap = THROTTLE_CAP_LOW;  break;  // safe fallback
        }
        uint16_t capped_raw = (uint16_t)constrain((int)raw, 0, cap);

        // Map the *capped* ADC value to display speed 
        int speed = (int)constrain(
                        map(capped_raw, THROTTLE_MIN, THROTTLE_MAX, 0, SPEED_MAX),
                        0, SPEED_MAX);

        xQueueOverwrite(speedQueue, &speed);
        telemetry_send_throttle(capped_raw);   // motor controller gets the limited value

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void taskDisplay(void *pvParameters) {
  int prevSpeed = -1;
  int speed     = 0;

  for (;;) {
    // Block until a new speed value arrives (or 100 ms timeout as a safety net)
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

// Lookup table for P42A cell voltage → SOC (linear interpolation between points)
  static const float P42A_V[]   = { 2.80, 3.00, 3.30, 3.50, 3.60, 3.65,
                                    3.70, 3.75, 3.80, 3.90, 4.00, 4.10, 4.20 };
  static const float P42A_SOC[] = { 0.00, 0.02, 0.06, 0.12, 0.20, 0.27,
                                    0.40, 0.55, 0.68, 0.80, 0.90, 0.97, 1.00 };
  #define P42A_POINTS 13

  float voltage_to_soc(float v) {
    if (v <= P42A_V[0])             return 0.0f;
    if (v >= P42A_V[P42A_POINTS-1]) return 100.0f;
    for (int i = 0; i < P42A_POINTS - 1; i++) {
        if (v >= P42A_V[i] && v < P42A_V[i + 1]) {
            float t = (v - P42A_V[i]) / (P42A_V[i + 1] - P42A_V[i]);
            return (P42A_SOC[i] + t * (P42A_SOC[i + 1] - P42A_SOC[i])) * 100.0f;
        }
    }
    return 0.0f;
}

void taskAux(void *pvParameters) {
    for (;;) {
        
    float avgCell = 0.0f;
    if (g_can_state.cell_count > 0) {
        for (int i = 0; i < g_can_state.cell_count; i++)
            avgCell += g_can_state.cell_v[i];
        avgCell /= g_can_state.cell_count;
    }

    float battPct = voltage_to_soc(avgCell);  

        xSemaphoreTake(tftMutex, portMAX_DELAY);
        drawBattery(battPct);
        drawOdometer(simOdometer);

        // Flash fault indicator if active
        if (g_can_state.fault_active) {
            tft.setTextColor(TFT_RED, COL_BG);
            tft.setTextSize(1);
            tft.setCursor(5, 5);
            tft.printf("FAULT %lu", g_can_state.fault_code);
        }
        xSemaphoreGive(tftMutex);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    esp_reset_reason_t reason = esp_reset_reason();
    Serial.printf("[BOOT] Reset reason: %d\n", reason);

    analogReadResolution(12);
    pinMode(THROTTLE_PIN, INPUT);

    //pinMode(SPEED_PIN_LOW,  INPUT_PULLUP);
    //pinMode(SPEED_PIN_MED,  INPUT_PULLUP);
    //pinMode(SPEED_PIN_HIGH, INPUT_PULLUP);

    tft.init();
    tft.setRotation(1);

    drawStaticUI();
    drawBattery(0);
    drawOdometer(simOdometer);
    drawSpeed(0);

    esp_task_wdt_init(10, false);
    telemetry_display_init();   // start TWAI

    speedQueue = xQueueCreate(1, sizeof(int));
    tftMutex   = xSemaphoreCreateMutex();
    configASSERT(speedQueue != NULL);
    configASSERT(tftMutex  != NULL);

    xTaskCreatePinnedToCore(taskThrottle, "Throttle", 2048, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(taskDisplay,  "Display",  4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(taskAux,      "Aux",      2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(taskCanRx,    "CanRx",    4096, NULL, 3, NULL, 0);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}