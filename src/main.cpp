#include <Arduino.h>   
#include <TFT_eSPI.h>
#include <SPI.h>
#include <math.h>

TFT_eSPI tft = TFT_eSPI();

// Screen dimensions
#define SCREEN_W 480
#define SCREEN_H 320

//  Colors 
#define COL_BG       0x0000   // Black
#define COL_ARC_BG   0x1082   // Dark grey arc track
#define COL_ARC_FG   0x07FF   // Cyan arc fill
#define COL_SPEED    0x07FF   // Cyan speed digits
#define COL_UNIT     0x7BEF   // Light grey "mph"
#define COL_BATT_BG  0x2104   // Dark green bg
#define COL_BATT_FG  0x07FF   // Green battery fill
#define COL_BATT_LOW 0xF800   // Red when low
#define COL_ODO      0x7BEF   // Light grey odometer
#define COL_TURN_ON  0x07FF   // Cyan arrow 
#define COL_TURN_OFF 0x2104   // Dark turn signal inactive
#define COL_WHITE    0xFFFF

// Arc gauge
#define ARC_CX      240       // Center X
#define ARC_CY      210       // Center Y 
#define ARC_R_OUT   195       // Outer radius
#define ARC_R_IN    165       // Inner radius
#define ARC_START   0         // Start angle (degrees, 0=right)
#define ARC_END     180       // End angle (sweep)
#define SPEED_MAX   100       // Max speed for full arc

// Throttle values
#define THROTTLE_PIN  13
#define THROTTLE_MIN  500    // ADC value at 0% throttle
#define THROTTLE_MAX  3200   // ADC value at 100% throttle


float simSpeed    = 0.0;
float simBattery  = 75.0;
float simOdometer = 7028.3;
float simAccelX   = 0.0;
float simAccelY   = 0.0;
float simAccelZ   = 1.0;
bool  turnLeft    = false;
bool  turnRight   = false;
int   prevSpeed   = -1;


unsigned long lastUpdate = 0;
unsigned long lastTurn   = 0;
int simPhase = 0;


void drawArcSegment(int cx, int cy, int rIn, int rOut, float startDeg, float endDeg, uint16_t color) {
  float step = 1.0;
  for (float a = startDeg; a <= endDeg; a += step) {
    float rad = a * DEG_TO_RAD;
    float cosA = cos(rad), sinA = sin(rad);
    int x0 = cx + rIn  * cosA;
    int y0 = cy + rIn  * sinA;
    int x1 = cx + rOut * cosA;
    int y1 = cy + rOut * sinA;
    tft.drawLine(x0, y0, x1, y1, color);
  }
}

void drawArcBackground() {
  float startRad = (180 + ARC_START) * DEG_TO_RAD; // flip for screen coords
  float totalDeg = ARC_END;
  drawArcSegment(ARC_CX, ARC_CY, ARC_R_IN, ARC_R_OUT,
               180, 360, COL_ARC_BG);

  // Tick marks every 10 mph
  for (int s = 0; s <= SPEED_MAX; s += 10) {
    float frac = (float)s / SPEED_MAX;
    float deg = (180 + frac * 180) * DEG_TO_RAD;
    int x0 = ARC_CX + (ARC_R_IN  - 5) * cos(deg);
    int y0 = ARC_CY + (ARC_R_IN  - 5) * sin(deg);
    int x1 = ARC_CX + (ARC_R_OUT + 5) * cos(deg);
    int y1 = ARC_CY + (ARC_R_OUT + 5) * sin(deg);
    tft.drawLine(x0, y0, x1, y1, COL_WHITE);
  }

  // "0" and "100" labels
  float d0   = (180 + ARC_START) * DEG_TO_RAD;
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

// Update arc fill for current speed
void updateArc(int speed) {
  float frac = constrain((float)speed / SPEED_MAX, 0.0, 1.0);
  float fillStart = 180;
  float fillEnd   = 180 + frac * 180;
  float fullEnd   = 360;

  // Fill
if (frac > 0)
  drawArcSegment(ARC_CX, ARC_CY, ARC_R_IN, ARC_R_OUT,
                 fillStart, fillEnd, COL_ARC_FG);

// Clear remainder
if (fillEnd < fullEnd)
  drawArcSegment(ARC_CX, ARC_CY, ARC_R_IN, ARC_R_OUT,
                 fillEnd, fullEnd, COL_ARC_BG);
}


// Speed mph
void drawSpeed(int speed) {
  // Clear old number
  tft.fillRect(170, 110, 160, 90, COL_BG);

  tft.setTextColor(COL_SPEED, COL_BG);
  tft.setTextSize(7);

  char buf[4];
  sprintf(buf, "%d", speed);

  // Center the text
  int len = strlen(buf);
  int x = 240 - (len * 42) / 2;  
  tft.setCursor(x, 115);
  tft.print(buf);

  // "mph" label
  tft.setTextColor(COL_UNIT, COL_BG);
  tft.setTextSize(2);
  tft.setCursor(290, 190);
  tft.print("mph");
}

// Battery indicator
void drawBattery(float pct) {
  int bx = 430, by = 10, bw = 30, bh = 80;

  // Outline
  tft.drawRect(bx, by, bw, bh, COL_WHITE);
  // Terminal nub
  tft.fillRect(bx + 8, by - 5, 14, 6, COL_WHITE);

  // Fill
  int fillH = (int)((pct / 100.0) * (bh - 4));
  uint16_t fillColor = (pct < 20) ? COL_BATT_LOW : COL_BATT_FG;

  // Clear interior
  tft.fillRect(bx + 2, by + 2, bw - 4, bh - 4, COL_BG);
  // Draw fill from bottom
  tft.fillRect(bx + 2, by + 2 + (bh - 4 - fillH), bw - 4, fillH, fillColor);

  // Percentage text
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
  int len = strlen(buf) * 12;
  tft.setCursor(240 - len / 2, 286);
  tft.print(buf);
}

void drawTurnSignal(bool left, bool right) {

  uint16_t lc = left  ? COL_TURN_ON : COL_TURN_OFF;
  uint16_t rc = right ? COL_TURN_ON : COL_TURN_OFF;

  int leftBaseX  = 95;   
  int rightBaseX = 385;  

  // LEFT arrow
  tft.fillTriangle(leftBaseX, 140,
                   leftBaseX + 30, 120,
                   leftBaseX + 30, 160, lc);
  tft.fillRect(leftBaseX + 30, 128, 25, 24, lc);

  // RIGHT arrow
  tft.fillTriangle(rightBaseX, 140,
                   rightBaseX - 30, 120,
                   rightBaseX - 30, 160, rc);
  tft.fillRect(rightBaseX - 55, 128, 25, 24, rc);
}


// Static UI
void drawStaticUI() {
  tft.fillScreen(COL_BG);

  // Odometer label
  tft.setTextColor(COL_ODO, COL_BG);
  tft.setTextSize(1);
  tft.setCursor(5, 300);
  tft.print("ODO");

  drawArcBackground();
}


void readThrottle() {
  int raw = analogRead(THROTTLE_PIN);
  
  // Map raw ADC to 0-100 speed range
  simSpeed = map(raw, THROTTLE_MIN, THROTTLE_MAX, 0, SPEED_MAX);
  simSpeed = constrain(simSpeed, 0, SPEED_MAX);
}


void setup() {
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);  
  pinMode(THROTTLE_PIN, INPUT);

  tft.init();
  tft.setRotation(1); // Set to landscape

  Serial.println("Display init done");

  drawStaticUI();
  drawTurnSignal(false, false);
  drawBattery(simBattery);
  drawOdometer(simOdometer);
  drawSpeed(0);

}

void loop() {
  unsigned long now = millis();
  if (now - lastUpdate < 50) return;  
  lastUpdate = now;

  readThrottle();

  int spd = (int)simSpeed;

  // Only redraw what changed
  if (spd != prevSpeed) {
    updateArc(spd);
    drawSpeed(spd);
    prevSpeed = spd;
  }

  

}