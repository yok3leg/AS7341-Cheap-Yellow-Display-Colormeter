/*
  AS7341_Spectrum.ino  —  DIY colormeter
  Live spectrum from an Adafruit AS7341 10-channel sensor on the
  ESP32-2432S028R "Cheap Yellow Display" (2.8" 240x320 ILI9341 + XPT2046 touch).

  Wiring (connector CN1, 4-pin JST 1.25 mm):  GND, GPIO22 = SCL, GPIO27 = SDA, 3V3
  Libraries: TFT_eSPI (with the CYD User_Setup.h), XPT2046_Touchscreen, Adafruit AS7341.

  Screens
    MAIN   live chart.  Buttons: SET BG / CLR BG, LED, RUN / HOLD, WIFI, SETUP
    SETUP  VIEW (bars/line/both), SCALE (norm/abs), GAIN (auto/1x..512x), CLR+NIR (show/hide), BACK
    WIFI   SSID suffix - / +, AP ON/OFF, BACK.  Open network, page at http://192.168.4.1

  Background subtraction: SET BG averages 10 readings and stores them; every later
  reading is shown with that background subtracted until CLR BG.
  Serial (115200) prints one CSV line per reading.
*/

#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <Adafruit_AS7341.h>
#include "webpage.h"

// ---------------- pins ----------------
#define I2C_SDA 27
#define I2C_SCL 22
#define XPT2046_IRQ  36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK  25
#define XPT2046_CS   33
#define RGB_LED_R 4
#define RGB_LED_G 16
#define RGB_LED_B 17

#define SCREEN_W 320
#define SCREEN_H 240

// XPT2046 raw-coordinate calibration (rotation 1), fitted from taps on this board
#define TOUCH_X_MIN  100
#define TOUCH_X_MAX  3450
#define TOUCH_Y_MIN  240
#define TOUCH_Y_MAX  3450

// ---------------- layout ----------------
#define HEADER_H   20
#define CHART_Y    HEADER_H
#define CHART_H    180
#define SPRITE_H   (CHART_H / 2)
#define BTN_Y      (CHART_Y + CHART_H + 4)   // 204
#define BTN_H      (SCREEN_H - BTN_Y - 2)    // 34
#define NUM_BTN    5
#define BTN_W      (SCREEN_W / NUM_BTN)      // 64

// inside the chart area
#define PLOT_X0    32
#define PLOT_X1    314
#define PLOT_TOP   8
#define PLOT_BOT   150
#define WL_ROW_Y   154
#define VAL_ROW_Y  166

// ---------------- sensor settings ----------------
const uint8_t  ATIME = 29;
const uint16_t ASTEP = 599;                                       // (30)(600)*2.78us = 50 ms
const uint32_t FULL_SCALE = (uint32_t)(ATIME + 1) * (ASTEP + 1);  // 18000 counts
const float    TINT_MS = (ATIME + 1) * (ASTEP + 1) * 2.78f / 1000.0f;
const uint16_t LED_CURRENT_MA = 20;
const int      BG_AVG_N = 10;

const as7341_gain_t GAIN_STEPS[] = {
  AS7341_GAIN_0_5X, AS7341_GAIN_1X, AS7341_GAIN_2X, AS7341_GAIN_4X, AS7341_GAIN_8X,
  AS7341_GAIN_16X, AS7341_GAIN_32X, AS7341_GAIN_64X, AS7341_GAIN_128X,
  AS7341_GAIN_256X, AS7341_GAIN_512X };
const float GAIN_VALUES[] = {0.5, 1, 2, 4, 8, 16, 32, 64, 128, 256, 512};
const int   NUM_GAINS = 11;
const int   MANUAL_GAIN_IDX[] = {1, 3, 5, 7, 9, 10};              // 1x 4x 16x 64x 256x 512x
const int   NUM_MANUAL = 6;

// ---------------- channels ----------------
// Index into the 12-word buffer from getAllChannels():
// 0..3 = F1..F4, 4 = Clear, 5 = NIR (low bank), 6..9 = F5..F8, 10 = Clear, 11 = NIR
#define NUM_CH   10
#define NUM_VIS  8
const uint8_t  CH_INDEX[NUM_CH] = {0, 1, 2, 3, 6, 7, 8, 9, 10, 11};
const uint16_t CH_WL[NUM_VIS]   = {415, 445, 480, 515, 555, 590, 630, 680};
const char    *CH_LABEL[NUM_CH] = {"415", "445", "480", "515", "555", "590", "630", "680", "CLR", "NIR"};
uint16_t chColor[NUM_CH];
float    chRGB[NUM_VIS][3];

// ---------------- objects ----------------
TFT_eSPI    tft;
TFT_eSprite chart = TFT_eSprite(&tft);
SPIClass    touchSPI(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);
Adafruit_AS7341 as7341;
WebServer   server(80);
Preferences prefs;
bool haveSprite = false;

// ---------------- state ----------------
enum ViewMode { VIEW_BARS, VIEW_LINE, VIEW_BOTH };
enum Screen   { SCR_MAIN, SCR_SETUP, SCR_WIFI };
Screen   screen = SCR_MAIN;
ViewMode viewMode = VIEW_BOTH;
bool normMode = true;
bool showExtra = false;      // CLR + NIR bars
bool ledOn = false;
bool hold = false;
int  manualSel = -1;         // -1 = auto gain, else index into MANUAL_GAIN_IDX
int  gainIdx = 7;            // start at 64x
bool sensorOk = false;
bool readingInProgress = false;

uint16_t raw[12];
float    basic[NUM_CH];      // counts / s / unit gain
float    disp[NUM_CH];       // basic minus background, clamped at 0
float    bg[NUM_CH];
float    bgAcc[NUM_CH];
bool     bgSet = false;
int      bgCapLeft = 0;      // >0 while capturing a background

float    absAxisMax = 100.0f;
int      absAxisLowCount = 0;
uint32_t lastReadMs = 0, readPeriodMs = 0, lastTouchMs = 0;
uint32_t lastSensorRetryMs = 0, lastWifiStatusMs = 0;

bool     apOn = false;
int      ssidSuffix = 1;
char     ssid[32];

// ============================================================
//  Colour helpers
// ============================================================
// Approximate sRGB of a monochromatic wavelength (Dan Bruton's algorithm)
void wavelengthToRGB(float wl, float &r, float &g, float &b) {
  float R = 0, G = 0, B = 0;
  if (wl >= 380 && wl < 440)      { R = -(wl - 440) / 60.0f; G = 0; B = 1; }
  else if (wl < 490)              { R = 0; G = (wl - 440) / 50.0f; B = 1; }
  else if (wl < 510)              { R = 0; G = 1; B = -(wl - 510) / 20.0f; }
  else if (wl < 580)              { R = (wl - 510) / 70.0f; G = 1; B = 0; }
  else if (wl < 645)              { R = 1; G = -(wl - 645) / 65.0f; B = 0; }
  else if (wl <= 780)             { R = 1; G = 0; B = 0; }
  float f = 1.0f;
  if (wl >= 380 && wl < 420)      f = 0.3f + 0.7f * (wl - 380) / 40.0f;
  else if (wl > 700 && wl <= 780) f = 0.3f + 0.7f * (780 - wl) / 80.0f;
  const float gamma = 0.8f;
  r = powf(R * f, gamma); g = powf(G * f, gamma); b = powf(B * f, gamma);
}

uint16_t rgbTo565(float r, float g, float b) {
  return tft.color565((uint8_t)(constrain(r, 0, 1) * 255), (uint8_t)(constrain(g, 0, 1) * 255),
                      (uint8_t)(constrain(b, 0, 1) * 255));
}

void initColors() {
  for (int i = 0; i < NUM_VIS; i++) {
    wavelengthToRGB(CH_WL[i], chRGB[i][0], chRGB[i][1], chRGB[i][2]);
    chColor[i] = rgbTo565(chRGB[i][0], chRGB[i][1], chRGB[i][2]);
  }
  chColor[8] = tft.color565(220, 220, 220);   // Clear
  chColor[9] = tft.color565(140, 0, 50);      // NIR
}

// ============================================================
//  Settings persistence
// ============================================================
void loadPrefs() {
  prefs.begin("colormeter", true);
  ssidSuffix = prefs.getInt("suffix", 1);
  showExtra  = prefs.getBool("extra", false);
  viewMode   = (ViewMode)prefs.getInt("view", VIEW_BOTH);
  normMode   = prefs.getBool("norm", true);
  prefs.end();
  ssidSuffix = constrain(ssidSuffix, 1, 99);
  snprintf(ssid, sizeof ssid, "Colormeter-%02d", ssidSuffix);
}

void savePrefs() {
  prefs.begin("colormeter", false);
  prefs.putInt("suffix", ssidSuffix);
  prefs.putBool("extra", showExtra);
  prefs.putInt("view", viewMode);
  prefs.putBool("norm", normMode);
  prefs.end();
}

// ============================================================
//  Sensor
// ============================================================
bool sensorInit() {
  if (!as7341.begin(AS7341_I2CADDR_DEFAULT, &Wire)) return false;
  as7341.setATIME(ATIME);
  as7341.setASTEP(ASTEP);
  as7341.setGain(GAIN_STEPS[gainIdx]);
  as7341.setLEDCurrent(LED_CURRENT_MA);
  as7341.enableLED(ledOn);
  readingInProgress = false;
  return true;
}

void applyGain(int idx) {
  idx = constrain(idx, 0, NUM_GAINS - 1);
  if (idx != gainIdx) {
    gainIdx = idx;
    as7341.setGain(GAIN_STEPS[gainIdx]);
  }
}

void autoGain() {
  uint16_t mx = 0;
  for (int i = 0; i < NUM_CH; i++) mx = max(mx, raw[CH_INDEX[i]]);
  if (mx >= FULL_SCALE * 0.90f && gainIdx > 0)                 applyGain(gainIdx - 1);
  else if (mx < FULL_SCALE * 0.08f && gainIdx < NUM_GAINS - 1) applyGain(gainIdx + 1);
}

void setLed(bool on) {
  ledOn = on;
  if (sensorOk) as7341.enableLED(ledOn);
}

void startBgCapture() {
  for (int i = 0; i < NUM_CH; i++) bgAcc[i] = 0;
  bgCapLeft = BG_AVG_N;
}

void clearBg() {
  bgSet = false;
  bgCapLeft = 0;
}

void onNewReading() {
  uint32_t now = millis();
  readPeriodMs = now - lastReadMs;
  lastReadMs = now;

  float denom = GAIN_VALUES[gainIdx] * TINT_MS / 1000.0f;   // counts per second at unit gain
  for (int i = 0; i < NUM_CH; i++) basic[i] = raw[CH_INDEX[i]] / denom;

  if (bgCapLeft > 0) {
    for (int i = 0; i < NUM_CH; i++) bgAcc[i] += basic[i];
    if (--bgCapLeft == 0) {
      for (int i = 0; i < NUM_CH; i++) bg[i] = bgAcc[i] / BG_AVG_N;
      bgSet = true;
      if (screen == SCR_MAIN) drawButtons();
    }
  }
  for (int i = 0; i < NUM_CH; i++) disp[i] = max(basic[i] - (bgSet ? bg[i] : 0.0f), 0.0f);

  // serial CSV: ms,gain,tint_ms,bg,raw F1..F8,Clear,NIR, value F1..F8,Clear,NIR
  Serial.printf("%lu,%g,%.1f,%d", (unsigned long)now, GAIN_VALUES[gainIdx], TINT_MS, bgSet ? 1 : 0);
  for (int i = 0; i < NUM_CH; i++) Serial.printf(",%u", raw[CH_INDEX[i]]);
  for (int i = 0; i < NUM_CH; i++) Serial.printf(",%.4g", disp[i]);
  Serial.println();

  if (screen == SCR_MAIN && !hold) { drawChart(); drawHeader(); }

  if (manualSel < 0) autoGain();   // only between readings
}

void pollSensor() {
  if (!sensorOk) {
    if (millis() - lastSensorRetryMs > 1000) {
      lastSensorRetryMs = millis();
      sensorOk = sensorInit();
      if (sensorOk) drawScreen();
      else showSensorError();
    }
    return;
  }
  if (!readingInProgress) {
    as7341.startReading();
    readingInProgress = true;
  } else if (as7341.checkReadingProgress()) {
    as7341.getAllChannels(raw);
    readingInProgress = false;
    onNewReading();
  }
}

// ============================================================
//  WiFi access point + web server
// ============================================================
void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
  String s;
  s.reserve(600);
  float hz = readPeriodMs > 0 ? 1000.0f / readPeriodMs : 0;
  s += "{\"t\":" + String(lastReadMs);
  s += ",\"gain\":" + String(GAIN_VALUES[gainIdx]);
  s += ",\"gsel\":" + String(manualSel < 0 ? -1 : MANUAL_GAIN_IDX[manualSel]);
  s += ",\"tint\":" + String(TINT_MS, 0);
  s += ",\"fs\":" + String(FULL_SCALE);
  s += ",\"hz\":" + String(hz, 2);
  s += ",\"bg\":" + String(bgSet ? "true" : "false");
  s += ",\"bgcap\":" + String(bgCapLeft);
  s += ",\"led\":" + String(ledOn ? "true" : "false");
  s += ",\"hold\":" + String(hold ? "true" : "false");
  s += ",\"extra\":" + String(showExtra ? "true" : "false");
  s += ",\"raw\":[";
  for (int i = 0; i < NUM_CH; i++) { if (i) s += ','; s += String(raw[CH_INDEX[i]]); }
  s += "],\"bc\":[";
  for (int i = 0; i < NUM_CH; i++) { if (i) s += ','; s += String(disp[i], 4); }
  s += "]}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", s);
}

void handleCmd() {
  if (server.hasArg("bg"))   { if (server.arg("bg") == "set") startBgCapture(); else clearBg(); }
  if (server.hasArg("led"))  setLed(server.arg("led") == "1");
  if (server.hasArg("hold")) hold = server.arg("hold") == "1";
  if (server.hasArg("gain")) {
    String g = server.arg("gain");
    if (g == "auto") manualSel = -1;
    else {
      int idx = constrain(g.toInt(), 0, NUM_GAINS - 1);
      // snap to the nearest manual step so the on-screen button agrees
      manualSel = 0;
      for (int k = 0; k < NUM_MANUAL; k++) if (MANUAL_GAIN_IDX[k] <= idx) manualSel = k;
      if (sensorOk) applyGain(MANUAL_GAIN_IDX[manualSel]);
    }
  }
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/plain", "ok");
  if (screen == SCR_MAIN) drawButtons();
}

void startAp() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid);               // open network, no password
  delay(100);
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/cmd", handleCmd);
  server.onNotFound(handleRoot);
  server.begin();
  apOn = true;
  Serial.printf("# AP %s up at %s\n", ssid, WiFi.softAPIP().toString().c_str());
}

void stopAp() {
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  apOn = false;
  Serial.println("# AP off");
}

// ============================================================
//  Drawing
// ============================================================
float niceCeil(float v) {           // 1-2-5 series >= v
  if (v <= 0) return 1;
  float p = powf(10, floorf(log10f(v)));
  float m = v / p;
  float n = (m <= 1) ? 1 : (m <= 2) ? 2 : (m <= 5) ? 5 : 10;
  return n * p;
}

void fmtValue(char *buf, size_t n, float v) {
  if (v >= 10000)     snprintf(buf, n, "%.0fk", v / 1000);
  else if (v >= 1000) snprintf(buf, n, "%.1fk", v / 1000);
  else if (v >= 100)  snprintf(buf, n, "%.0f", v);
  else if (v >= 10)   snprintf(buf, n, "%.1f", v);
  else                snprintf(buf, n, "%.2f", v);
}

// Catmull-Rom interpolation between p1 and p2
float catmull(float p0, float p1, float p2, float p3, float t) {
  float t2 = t * t, t3 = t2 * t;
  return 0.5f * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2 +
                 (-p0 + 3 * p1 - 3 * p2 + p3) * t3);
}

void drawChartTo(TFT_eSPI &g, int oy) {
  const int plotH = PLOT_BOT - PLOT_TOP;
  const int nShown = showExtra ? NUM_CH : NUM_VIS;
  const int slotW = (PLOT_X1 - PLOT_X0) / nShown;
  const int barW  = slotW - 8;
  g.fillRect(0, oy, SCREEN_W, CHART_H, TFT_BLACK);

  // ----- scale -----
  float scaleMax;
  float values[NUM_CH];
  float mx = 0;
  for (int i = 0; i < nShown; i++) mx = max(mx, disp[i]);
  if (normMode) {
    if (mx <= 0) mx = 1;
    for (int i = 0; i < NUM_CH; i++) values[i] = disp[i] / mx * 100.0f;
    scaleMax = 100.0f;
  } else {
    float want = niceCeil(mx * 1.05f);
    if (want > absAxisMax) { absAxisMax = want; absAxisLowCount = 0; }
    else if (want < absAxisMax * 0.5f) { if (++absAxisLowCount > 10) { absAxisMax = want; absAxisLowCount = 0; } }
    else absAxisLowCount = 0;
    for (int i = 0; i < NUM_CH; i++) values[i] = disp[i];
    scaleMax = absAxisMax;
  }

  // ----- grid + axis labels -----
  g.setTextFont(1);
  g.setTextColor(TFT_DARKGREY, TFT_BLACK);
  g.setTextDatum(MR_DATUM);
  for (int k = 0; k <= 4; k++) {
    int y = oy + PLOT_BOT - (plotH * k) / 4;
    g.drawFastHLine(PLOT_X0 - 2, y, slotW * nShown + 4, k == 0 ? TFT_LIGHTGREY : 0x2104);
    char buf[12];
    if (normMode) snprintf(buf, sizeof buf, "%d%%", k * 25);
    else fmtValue(buf, sizeof buf, scaleMax * k / 4);
    g.drawString(buf, PLOT_X0 - 4, y);
  }

  int h[NUM_CH], cx[NUM_CH];
  for (int i = 0; i < nShown; i++) {
    float f = constrain(values[i] / scaleMax, 0.0f, 1.0f);
    h[i]  = (int)(f * plotH + 0.5f);
    cx[i] = PLOT_X0 + i * slotW + slotW / 2;
  }

  // ----- bars -----
  if (viewMode == VIEW_BARS || viewMode == VIEW_BOTH) {
    for (int i = 0; i < nShown; i++) {
      if (h[i] > 0) g.fillRect(cx[i] - barW / 2, oy + PLOT_BOT - h[i], barW, h[i], chColor[i]);
      if (raw[CH_INDEX[i]] >= FULL_SCALE - 1)   // saturation marker
        g.drawRect(cx[i] - barW / 2, oy + PLOT_TOP, barW, plotH, TFT_RED);
    }
  }

  // ----- spectrum line through the 8 visible channels -----
  if (viewMode == VIEW_LINE || viewMode == VIEW_BOTH) {
    int xStart = cx[0], xEnd = cx[NUM_VIS - 1];
    int prevY = -1;
    for (int x = xStart; x <= xEnd; x++) {
      int seg = min((x - xStart) / slotW, NUM_VIS - 2);
      float t = (float)(x - cx[seg]) / slotW;
      float p0 = h[max(seg - 1, 0)], p1 = h[seg], p2 = h[seg + 1], p3 = h[min(seg + 2, NUM_VIS - 1)];
      float hh = constrain(catmull(p0, p1, p2, p3, t), 0.0f, (float)plotH);
      int y = oy + PLOT_BOT - (int)(hh + 0.5f);
      if (viewMode == VIEW_LINE) {
        float wl = CH_WL[seg] + t * (CH_WL[seg + 1] - CH_WL[seg]);
        float r, gg, b; wavelengthToRGB(wl, r, gg, b);
        g.drawFastVLine(x, y, oy + PLOT_BOT - y, rgbTo565(r * 0.75f, gg * 0.75f, b * 0.75f));
      }
      if (prevY >= 0) { g.drawLine(x - 1, prevY, x, y, TFT_WHITE); g.drawLine(x - 1, prevY - 1, x, y - 1, TFT_WHITE); }
      prevY = y;
    }
    if (viewMode == VIEW_LINE) {   // CLR / NIR as thin bars so they are not lost
      for (int i = NUM_VIS; i < nShown; i++)
        if (h[i] > 0) g.fillRect(cx[i] - 4, oy + PLOT_BOT - h[i], 8, h[i], chColor[i]);
    }
  }

  // ----- labels -----
  g.setTextDatum(TC_DATUM);
  for (int i = 0; i < nShown; i++) {
    g.setTextColor(chColor[i], TFT_BLACK);
    g.drawString(CH_LABEL[i], cx[i], oy + WL_ROW_Y);
    char buf[12];
    if (normMode) snprintf(buf, sizeof buf, "%.0f", values[i]);
    else fmtValue(buf, sizeof buf, values[i]);
    g.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    g.drawString(buf, cx[i], oy + VAL_ROW_Y);
  }
  g.setTextDatum(TL_DATUM);
  g.setTextColor(TFT_DARKGREY, TFT_BLACK);
  g.drawString(normMode ? "%" : "cnt/s/x", 2, oy + VAL_ROW_Y);
}

// The chart is 320x180x16bit = 115 KB, larger than the biggest block the ESP32
// heap can hand out, so it is rendered in two half-height passes through one
// 320x90 sprite (the sprite clips anything drawn outside itself).
void drawChart() {
  if (haveSprite) {
    drawChartTo(chart, 0);         chart.pushSprite(0, CHART_Y);
    drawChartTo(chart, -SPRITE_H); chart.pushSprite(0, CHART_Y + SPRITE_H);
  } else {
    drawChartTo(tft, CHART_Y);
  }
}

void drawHeader() {
  // estimated colour of the measured light, from the 8 visible channels
  float r = 0, g = 0, b = 0, mx = 0;
  for (int i = 0; i < NUM_VIS; i++) mx = max(mx, disp[i]);
  int peak = 0;
  for (int i = 0; i < NUM_VIS; i++) {
    float w = (mx > 0) ? disp[i] / mx : 0;
    r += w * chRGB[i][0]; g += w * chRGB[i][1]; b += w * chRGB[i][2];
    if (disp[i] > disp[peak]) peak = i;
  }
  float cmax = max(r, max(g, b)); if (cmax <= 0) cmax = 1;
  tft.fillRect(66, 2, 16, 16, rgbTo565(r / cmax, g / cmax, b / cmax));
  tft.drawRect(66, 2, 16, 16, TFT_DARKGREY);

  tft.setTextFont(2);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(80);
  char buf[40];
  snprintf(buf, sizeof buf, "peak %d nm", CH_WL[peak]);
  tft.drawString(buf, 88, 2);

  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(hold ? TFT_YELLOW : TFT_CYAN, TFT_BLACK);
  tft.setTextPadding(150);
  float hz = readPeriodMs > 0 ? 1000.0f / readPeriodMs : 0;
  char tag[16] = "";
  if (bgCapLeft > 0)   snprintf(tag, sizeof tag, "BG %d.. ", BG_AVG_N - bgCapLeft + 1);
  else if (bgSet)      strcpy(tag, "BG ");
  snprintf(buf, sizeof buf, "%s%s%s%gx %.0fms %.1fHz", apOn ? "AP " : "", tag, hold ? "HOLD " : "",
           GAIN_VALUES[gainIdx], TINT_MS, hz);
  tft.drawString(buf, SCREEN_W - 2, 2);
  tft.setTextPadding(0);
}

void drawButton(int i, const char *label, bool active) {
  int x = i * BTN_W + 1;
  if (label[0] == 0) { tft.fillRect(x, BTN_Y, BTN_W - 2, BTN_H, TFT_BLACK); return; }
  uint16_t bg = active ? 0x0339 : 0x2104;      // teal-blue when active, dark grey otherwise
  tft.fillRoundRect(x, BTN_Y, BTN_W - 2, BTN_H, 5, bg);
  tft.drawRoundRect(x, BTN_Y, BTN_W - 2, BTN_H, 5, active ? TFT_CYAN : TFT_DARKGREY);
  tft.setTextFont(2);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, bg);
  tft.drawString(label, x + (BTN_W - 2) / 2, BTN_Y + BTN_H / 2);
}

void drawButtons() {
  switch (screen) {
    case SCR_MAIN:
      drawButton(0, bgCapLeft > 0 ? "BG..." : bgSet ? "CLR BG" : "SET BG", bgSet || bgCapLeft > 0);
      drawButton(1, ledOn ? "LED ON" : "LED OFF", ledOn);
      drawButton(2, hold ? "HOLD" : "RUN", hold);
      drawButton(3, "WIFI", apOn);
      drawButton(4, "SETUP", false);
      break;
    case SCR_SETUP:
      drawButton(0, "VIEW", false);
      drawButton(1, "SCALE", false);
      drawButton(2, "GAIN", false);
      drawButton(3, "CLR/NIR", false);
      drawButton(4, "BACK", true);
      break;
    case SCR_WIFI:
      drawButton(0, "-", false);
      drawButton(1, "+", false);
      drawButton(2, apOn ? "AP OFF" : "AP ON", apOn);
      drawButton(3, "", false);
      drawButton(4, "BACK", true);
      break;
  }
}

void drawTitle(const char *t) {
  tft.fillRect(0, 0, SCREEN_W, HEADER_H, TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString(t, 2, 2);
}

void drawSetupScreen() {
  drawTitle("SETUP");
  tft.fillRect(0, CHART_Y, SCREEN_W, CHART_H, TFT_BLACK);
  tft.setTextFont(2);
  const char *view = (viewMode == VIEW_BARS) ? "BARS" : (viewMode == VIEW_LINE) ? "LINE" : "BARS + LINE";
  char gbuf[16];
  if (manualSel < 0) strcpy(gbuf, "AUTO");
  else snprintf(gbuf, sizeof gbuf, "%gx", GAIN_VALUES[MANUAL_GAIN_IDX[manualSel]]);
  const char *names[]  = {"View", "Scale", "Gain", "CLR / NIR bars", "Background"};
  const char *values[] = {view, normMode ? "NORM (tallest = 100%)" : "ABS (counts/s/gain)", gbuf,
                          showExtra ? "SHOW" : "HIDE", bgSet ? "set" : "none"};
  for (int i = 0; i < 5; i++) {
    int y = CHART_Y + 14 + i * 28;
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.drawString(names[i], 16, y);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString(values[i], 150, y);
  }
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("Settings are saved on BACK", 16, CHART_Y + CHART_H - 20);
  drawButtons();
}

void drawWifiStatus() {
  tft.setTextFont(2);
  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(280);
  char buf[48];
  if (apOn) {
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    snprintf(buf, sizeof buf, "AP ON   %d phone(s) connected", WiFi.softAPgetStationNum());
  } else {
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    strcpy(buf, "AP OFF  (press AP ON to start)");
  }
  tft.drawString(buf, 16, CHART_Y + 130);
  tft.setTextPadding(0);
}

void drawWifiScreen() {
  drawTitle("WIFI");
  tft.fillRect(0, CHART_Y, SCREEN_W, CHART_H, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextFont(2);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Network name (SSID)", 16, CHART_Y + 10);
  tft.setTextFont(4);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(280);
  tft.drawString(ssid, 16, CHART_Y + 30);
  tft.setTextPadding(0);
  tft.setTextFont(2);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("Open network, no password", 16, CHART_Y + 64);
  tft.drawString("1. Connect the phone to this network", 16, CHART_Y + 88);
  tft.drawString("2. Open  http://192.168.4.1  in a browser", 16, CHART_Y + 106);
  drawWifiStatus();
  drawButtons();
}

void drawMainScreen() {
  drawTitle("AS7341");
  drawButtons();
  drawChart();
  drawHeader();
}

void drawScreen() {
  tft.fillScreen(TFT_BLACK);
  switch (screen) {
    case SCR_MAIN:  drawMainScreen();  break;
    case SCR_SETUP: drawSetupScreen(); break;
    case SCR_WIFI:  drawWifiScreen();  break;
  }
}

void showSensorError() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextFont(2);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.drawString("AS7341 not found on I2C (0x39)", SCREEN_W / 2, 90);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("CN1:  GND  22=SCL  27=SDA  3V3", SCREEN_W / 2, 120);
  tft.drawString("retrying...", SCREEN_W / 2, 150);
}

// ============================================================
//  Touch
// ============================================================
void handleTouch() {
  if (!ts.tirqTouched() || !ts.touched()) return;
  if (millis() - lastTouchMs < 300) return;
  TS_Point p = ts.getPoint();
  lastTouchMs = millis();
  int x = map(p.x, TOUCH_X_MIN, TOUCH_X_MAX, 0, SCREEN_W);
  int y = map(p.y, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, SCREEN_H);
  Serial.printf("# touch raw=(%d,%d) -> (%d,%d)\n", p.x, p.y, x, y);
  if (y < BTN_Y - 30) return;         // generous: anything near the button row counts
  int btn = constrain(x / BTN_W, 0, NUM_BTN - 1);

  switch (screen) {
    case SCR_MAIN:
      switch (btn) {
        case 0: if (bgSet || bgCapLeft > 0) clearBg(); else startBgCapture(); break;
        case 1: setLed(!ledOn); break;
        case 2: hold = !hold; break;
        case 3: screen = SCR_WIFI;  drawScreen(); return;
        case 4: screen = SCR_SETUP; drawScreen(); return;
      }
      drawButtons();
      if (sensorOk) { drawChart(); drawHeader(); }
      break;

    case SCR_SETUP:
      switch (btn) {
        case 0: viewMode = (ViewMode)((viewMode + 1) % 3); break;
        case 1: normMode = !normMode; absAxisMax = 1; break;
        case 2:
          manualSel++;
          if (manualSel >= NUM_MANUAL) manualSel = -1;
          if (manualSel >= 0 && sensorOk) applyGain(MANUAL_GAIN_IDX[manualSel]);
          break;
        case 3: showExtra = !showExtra; break;
        case 4: savePrefs(); screen = SCR_MAIN; drawScreen(); return;
      }
      drawSetupScreen();
      break;

    case SCR_WIFI:
      switch (btn) {
        case 0: case 1:
          ssidSuffix = constrain(ssidSuffix + (btn == 0 ? -1 : 1), 1, 99);
          snprintf(ssid, sizeof ssid, "Colormeter-%02d", ssidSuffix);
          savePrefs();
          if (apOn) { stopAp(); startAp(); }   // rename the running network
          drawWifiScreen();
          break;
        case 2:
          if (apOn) stopAp(); else startAp();
          drawWifiScreen();
          break;
        case 4: screen = SCR_MAIN; drawScreen(); return;
      }
      break;
  }
}

// ============================================================
void setup() {
  Serial.begin(115200);
  // CYD on-board RGB LED is active-low: switch it off
  pinMode(RGB_LED_R, OUTPUT); pinMode(RGB_LED_G, OUTPUT); pinMode(RGB_LED_B, OUTPUT);
  digitalWrite(RGB_LED_R, HIGH); digitalWrite(RGB_LED_G, HIGH); digitalWrite(RGB_LED_B, HIGH);

  loadPrefs();
  WiFi.mode(WIFI_OFF);

  tft.init();
  tft.setRotation(1);                 // landscape, USB on the right
  tft.fillScreen(TFT_BLACK);
  initColors();

  chart.setColorDepth(16);
  haveSprite = (chart.createSprite(SCREEN_W, SPRITE_H) != nullptr);
  Serial.printf("# sprite %s, free heap %u\n", haveSprite ? "ok" : "FAILED (direct draw)", ESP.getFreeHeap());

  touchSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  ts.begin(touchSPI);
  ts.setRotation(1);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  sensorOk = sensorInit();
  if (sensorOk) {
    Serial.println("# AS7341 ready");
    Serial.println("ms,gain,tint_ms,bg,raw_415,raw_445,raw_480,raw_515,raw_555,raw_590,raw_630,raw_680,raw_CLR,raw_NIR,"
                   "val_415,val_445,val_480,val_515,val_555,val_590,val_630,val_680,val_CLR,val_NIR");
    drawScreen();
  } else {
    Serial.println("# AS7341 not found");
    showSensorError();
  }
  lastReadMs = millis();
}

void loop() {
  handleTouch();
  pollSensor();
  if (apOn) {
    server.handleClient();
    if (screen == SCR_WIFI && millis() - lastWifiStatusMs > 1000) {
      lastWifiStatusMs = millis();
      drawWifiStatus();
    }
  }
  delay(1);
}
