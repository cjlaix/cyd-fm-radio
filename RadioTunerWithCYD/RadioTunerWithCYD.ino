#include <SPI.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <RDA5807.h>
#include <EEPROM.h>
#include <XPT2046_Touchscreen.h>
#include <esp_task_wdt.h>

// Phone control over Bluetooth LE (needs the "NimBLE-Arduino" library, version 2.x).
// Set to 0 to build without Bluetooth: the radio then works exactly as before.
#define ENABLE_BLE 1
#if ENABLE_BLE
#include <NimBLEDevice.h>
#endif

// --- Physical Pin Mappings ---
#define I2C_SDA 27
#define I2C_SCL 18
#define SCREEN_BACKLIGHT_PIN 21

// The CYD touch controller (XPT2046) is wired to its OWN SPI pins,
// not to the display bus and not to the default SPI pins (18/19/23).
#define XPT2046_IRQ  36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK  25
#define XPT2046_CS   33

// --- Touch Calibration (raw XPT2046 readings) ---
// If taps land in the wrong place: set TOUCH_DEBUG to 1, open the Serial
// Monitor (115200), tap the four corners and copy the raw min/max values here.
#define TOUCH_DEBUG     0
#define TOUCH_X_MIN     340
#define TOUCH_X_MAX     3900
#define TOUCH_Y_MIN     240
#define TOUCH_Y_MAX     3850
#define TOUCH_RELEASE_MS 120   // gap that counts as a new tap (filters resistive jitter)
#define TOUCH_REPEAT_MS  220   // auto-repeat for held volume buttons

// --- Timing Constants ---
#define STATUS_POLL_MS 1000
#define RDS_POLL_MS    40
#define RDS_SCROLL_MS  350
#define STORE_TIME     10000
#define WDT_TIMEOUT_S  8       // restart automatically if the program freezes this long
#define RESTART_HOLD_MS 3000   // hold the power button this long to restart
#define STANDBY_BACKLIGHT 25   // 0..255 screen brightness while the radio is off

// --- FM Band (10 kHz units) ---
#define FREQ_MIN 8750
#define FREQ_MAX 10800
#define CHIP_FREQ_MIN 8700  // lowest frequency the chip can report (channel 0)

const uint8_t app_id = 43;
const int eeprom_address = 0;
const uint8_t cfg_id = 0x5A;
const int cfg_address = 16;   // settings page values live after the station/volume block
const uint8_t preset_id = 0x5C;
const int preset_address = 40;
#define PRESET_COUNT 6

// --- Colour Palette (RGB565) ---
#define RGB(r, g, b) (uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))
#define C_BG     RGB(8, 10, 18)
#define C_PANEL  RGB(22, 28, 44)
#define C_EDGE   RGB(52, 62, 92)
#define C_TEXT   RGB(225, 230, 240)
#define C_DIM    RGB(120, 130, 155)
#define C_ACCENT RGB(255, 176, 48)
#define C_CYAN   RGB(80, 200, 255)
#define C_GREEN  RGB(90, 225, 130)
#define C_RED    RGB(255, 80, 80)

// --- Layout ---
#define CARD_X 6
#define CARD_Y 30
#define CARD_W 308
#define CARD_H 80
#define FREQ_X 240          // right edge of the big frequency digits
#define FREQ_Y 80           // vertical centre of the big frequency digits
#define RT_Y   123          // radio text line
#define SCALE_X0 18
#define SCALE_X1 302
#define VOL_X  46
#define VOL_Y  167
#define RT_WINDOW 36        // characters visible when radio text scrolls

#define PWR_X      18
#define PWR_Y      15
#define GEAR_X     118
#define GEAR_Y     15
#define PILL_ST_X  146
#define PILL_ST_W  68
#define PILL_RDS_X 220
#define PILL_RDS_W 46
#define PILL_Y     5
#define PILL_H     20

// Settings page layout
#define SET_ROW_Y   34
#define SET_ROW_H   33
#define SET_CTRL_X  196     // left edge of the value control
#define SET_CTRL_W  118
#define SET_BTN_W   34      // width of the - and + boxes
#define RESTART_X   60      // restart button on the last settings page
#define RESTART_Y   146
#define RESTART_W   200
#define RESTART_H   48

enum Icon { ICON_SEEK_DOWN, ICON_VOL_DOWN, ICON_VOL_UP, ICON_SEEK_UP };

struct Button {
  int16_t x, y, w, h;
  const char* label;
  uint16_t color;
  Icon icon;
};

const Button buttons[] = {
  {   8, 184, 72, 50, "SEEK",  C_ACCENT, ICON_SEEK_DOWN },
  {  86, 184, 72, 50, "VOL",   C_CYAN,   ICON_VOL_DOWN  },
  { 164, 184, 72, 50, "VOL",   C_CYAN,   ICON_VOL_UP    },
  { 242, 184, 72, 50, "SEEK",  C_ACCENT, ICON_SEEK_UP   },
};
const uint8_t buttonCount = sizeof(buttons) / sizeof(buttons[0]);

// The library keeps its own copy of the chip registers but never loads
// registers 0x04 and 0x07 from the chip, so setting one field there clears the
// others. This subclass writes the fields we tune with all their neighbours intact.
class RadioChip : public RDA5807 {
public:
  const uint16_t* statusRegs() { return shadowStatusRegisters; }   // 0x0A..0x0F after getRdsReady()

  void setNewDemod(bool on) {
    reg02->refined.SEEK = 0;            // the shadow still has SEEK=1 after a seek
    reg02->refined.NEW_METHOD = on;
    setRegister(REG02, reg02->raw);
  }

  void setBassBoost(bool on) {
    reg02->refined.SEEK = 0;
    reg02->refined.BASS = on;
    setRegister(REG02, reg02->raw);
  }

  // Soft blend + its threshold (unit 2 dB). Other bits keep the chip's reset values.
  void setSoftBlendLevel(bool on, uint8_t threshold) {
    reg07->raw = ((uint16_t)(threshold & 0x1F) << 10) | (1 << 9) | (on ? 0x0002 : 0);
    setRegister(REG07, reg07->raw);
  }
};

// --- Tunable settings (settings page) ---
// Defaults reproduce how the radio behaved before the settings page existed.
struct Config {
  uint8_t seekTh    = 2;    // what setSeekThreshold(50) really set (only 4 bits are kept)
  uint8_t muteSeek  = 0;
  uint8_t lnaPort   = 2;    // LNAP, library default
  uint8_t lnaCur    = 0;    // 1.8 mA, library default
  uint8_t newDemod  = 0;
  uint8_t softBlend = 1;    // chip reset default
  uint8_t blendTh   = 16;   // chip reset default (32 dB)
  uint8_t softMute  = 0;
  uint8_t bass      = 1;    // library default
  uint8_t deemph50  = 0;    // 75 us, as the library leaves it
  uint8_t rdsStrict = 0;
  uint8_t ble       = 1;
};
Config cfg;

struct SettingDef {
  const char* name;
  const char* hint;
  uint8_t* value;
  uint8_t minV, maxV;
  bool stepper;               // true = [-] value [+], false = tap to cycle
  const char* const* labels;  // NULL = show the number
  uint8_t scale;              // shown number = value * scale
  const char* unit;
};

const char* const LBL_OFF_ON[] = { "OFF", "ON" };
const char* const LBL_ANT[]    = { "NONE", "LNAN", "LNAP", "DUAL" };
const char* const LBL_LNA[]    = { "1.8 mA", "2.1 mA", "2.5 mA", "3.0 mA" };
const char* const LBL_DEEMPH[] = { "75 us", "50 us" };
const char* const LBL_RDS[]    = { "BASIC", "STRICT" };

const SettingDef settingDefs[] = {
  // Page 1: reception
  { "Seek threshold",   "lower finds weaker stations",  &cfg.seekTh,    0, 15, true,  NULL,       1, ""    },
  { "Mute while seek",  "silence the scan noise",       &cfg.muteSeek,  0, 1,  false, LBL_OFF_ON, 1, ""    },
  { "Antenna input",    "pin the antenna is wired to",  &cfg.lnaPort,   1, 3,  false, LBL_ANT,    1, ""    },
  { "LNA current",      "higher may help weak signals", &cfg.lnaCur,    0, 3,  true,  LBL_LNA,    1, ""    },
  { "New demodulation", "+1 dB sensitivity, may hiss",  &cfg.newDemod,  0, 1,  false, LBL_OFF_ON, 1, ""    },
  // Page 2: sound & RDS
  { "Soft blend",       "fade to mono on weak signal",  &cfg.softBlend, 0, 1,  false, LBL_OFF_ON, 1, ""    },
  { "Blend threshold",  "level where blending starts",  &cfg.blendTh,   0, 31, true,  NULL,       2, " dB" },
  { "Soft mute",        "quieter on very weak signal",  &cfg.softMute,  0, 1,  false, LBL_OFF_ON, 1, ""    },
  { "Bass boost",       "low-end lift",                 &cfg.bass,      0, 1,  false, LBL_OFF_ON, 1, ""    },
  { "De-emphasis",      "75 less hiss, 50 brighter",    &cfg.deemph50,  0, 1,  false, LBL_DEEMPH, 1, ""    },
  { "RDS filter",       "STRICT: slower, no garbage",   &cfg.rdsStrict, 0, 1,  false, LBL_RDS,    1, ""    },
#if ENABLE_BLE
  // Page 3: phone
  { "Bluetooth",        "control from your phone",      &cfg.ble,       0, 1,  false, LBL_OFF_ON, 1, ""    },
#endif
};
const uint8_t settingCount = sizeof(settingDefs) / sizeof(settingDefs[0]);
const uint8_t pageFirst[] = { 0, 5, 11 };   // page 3: Bluetooth (if built in) + restart
const uint8_t pageCount = sizeof(pageFirst) / sizeof(pageFirst[0]);

// --- State ---
bool bSt = true;    // true = stereo allowed, false = forced mono
bool bRds = true;

uint16_t currentFrequency;
int8_t shownSignalBars = -1;
uint8_t lastRssi = 0;
uint16_t presets[PRESET_COUNT];   // 0 = empty
bool cfgDirty = false;

bool inSettings = false;
uint8_t settingsPage = 0;
unsigned long resetArmedAt = 0;   // DEFAULTS needs a second tap within 3 s
unsigned long restartArmedAt = 0; // RESTART needs a second tap within 3 s

bool poweredOn = true;            // "off" = muted, standby screen; not saved, so it always starts on
volatile unsigned long lastLoopMs = 0;

bool settingsDirty = false;
unsigned long lastChange = 0;
unsigned long lastStatus = 0;
unsigned long lastRdsPoll = 0;
unsigned long lastRdsScroll = 0;

struct RdsField {
  char shown[65];
  char candidate[65];
};
RdsField rdsName, rdsText, rdsClock;
uint8_t rtOffset = 0;

// STRICT RDS decoder state. Every 2- or 4-character piece is voted on
// separately, so a single corrupted group can never reach the screen.
struct RdsStrict {
  uint16_t pi;
  uint16_t lastB, lastC, lastD;
  char     ps[8];
  char     psCand[8];
  uint8_t  psHits[4];
  uint8_t  psGood;           // bit per confirmed PS segment
  char     rt[64];
  char     rtCand[64];
  uint8_t  rtHits[16];
  uint16_t rtGood;           // bit per confirmed RT segment
  uint8_t  rtSegLen;         // 4 for group 2A, 2 for group 2B
  uint8_t  rtLen;            // shortened once a carriage return is confirmed
  int8_t   rtAB;
};
RdsStrict srds;

// STRICT clock: needs two consistent time messages before it's trusted
bool ctHaveCandidate = false;
int16_t ctCandMinute;
unsigned long ctCandAt;
bool clockValid = false;
int16_t clockMinute;
unsigned long clockAt;
int16_t shownClockMinute = -1;

TFT_eSPI tft = TFT_eSPI();
RadioChip rx;

// Give the touch controller whichever hardware SPI port the display isn't using
#ifdef USE_HSPI_PORT
SPIClass touchSPI(VSPI);
#else
SPIClass touchSPI(HSPI);
#endif
XPT2046_Touchscreen touch(XPT2046_CS, XPT2046_IRQ);

#if ENABLE_BLE
#define BLE_NAME         "CYD Radio"
#define BLE_SERVICE_UUID "6e5a0001-3c1d-4f7a-9b2e-7c2d9f1a0c01"
#define BLE_STATE_UUID   "6e5a0002-3c1d-4f7a-9b2e-7c2d9f1a0c01"   // read + notify: JSON status
#define BLE_CMD_UUID     "6e5a0003-3c1d-4f7a-9b2e-7c2d9f1a0c01"   // write: short text commands

// BLE callbacks run in the Bluetooth task, so commands are queued and
// carried out in loop(), where it's safe to touch the screen and the radio.
struct BleCmd { char text[24]; };
QueueHandle_t bleCmdQueue;
NimBLEServer* bleServer = nullptr;
NimBLECharacteristic* bleState = nullptr;
volatile bool bleWantStatus = false;
bool bleShownConnected = false;
unsigned long lastBleStatus = 0;
char lastStatusJson[320] = "";
#endif

void markDirty() {
  settingsDirty = true;
  lastChange = millis();
}

// ============================================================
//  Receiver settings
// ============================================================

void applySettings() {
  rx.setSeekThreshold(cfg.seekTh);
  rx.setLnaPortSel(cfg.lnaPort);
  rx.setLnaIcSel(cfg.lnaCur);
  rx.setNewDemod(cfg.newDemod);
  rx.setBassBoost(cfg.bass);
  rx.setSoftBlendLevel(cfg.softBlend, cfg.blendTh);
  rx.setSoftmute(cfg.softMute);
  rx.setFmDeemphasis(cfg.deemph50);
#if ENABLE_BLE
  bleApply();
#endif
}

void saveConfig() {
  EEPROM.write(cfg_address, cfg_id);
  for (uint8_t i = 0; i < settingCount; i++) EEPROM.write(cfg_address + 1 + i, *settingDefs[i].value);
  EEPROM.commit();
}

void loadConfig() {
  if (EEPROM.read(cfg_address) != cfg_id) return;   // never saved: keep defaults
  for (uint8_t i = 0; i < settingCount; i++) {
    uint8_t v = EEPROM.read(cfg_address + 1 + i);
    if (v >= settingDefs[i].minV && v <= settingDefs[i].maxV) *settingDefs[i].value = v;
  }
}

// ============================================================
//  Drawing
// ============================================================

void drawPill(int16_t x, int16_t w, const char* label, bool on, uint16_t color) {
  uint16_t fill = on ? color : C_PANEL;
  tft.fillRoundRect(x, PILL_Y, w, PILL_H, PILL_H / 2, fill);
  tft.drawRoundRect(x, PILL_Y, w, PILL_H, PILL_H / 2, on ? color : C_EDGE);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(on ? C_BG : C_DIM, fill);
  tft.drawString(label, x + w / 2, PILL_Y + PILL_H / 2 + 1, 2);
}

void drawStereoPill() {
  if (inSettings) return;
  drawPill(PILL_ST_X, PILL_ST_W, bSt ? "STEREO" : "MONO", bSt, C_CYAN);
}

void drawRdsPill() {
  if (inSettings) return;
  drawPill(PILL_RDS_X, PILL_RDS_W, "RDS", bRds, C_GREEN);
}

void drawGear(int16_t cx, int16_t cy, uint16_t color, uint16_t bg) {
  for (uint8_t i = 0; i < 8; i++) {
    float a = i * PI / 4;
    tft.fillCircle(cx + cos(a) * 8, cy + sin(a) * 8, 2, color);
  }
  tft.fillCircle(cx, cy, 7, color);
  tft.fillCircle(cx, cy, 3, bg);
}

// Power symbol: ring with a gap at the top and a stem through it
void drawPowerIcon(int16_t cx, int16_t cy, int16_t r, uint16_t color, uint16_t bg) {
  int16_t stem = max(2, r / 4);
  tft.fillCircle(cx, cy, r, color);
  tft.fillCircle(cx, cy, r - stem - 1, bg);
  tft.fillRect(cx - r / 2, cy - r - 1, r, r, bg);
  tft.fillRect(cx - stem / 2, cy - r - 2, stem, r + 2, color);
}

void drawStandby() {
  char buf[16];
  tft.fillScreen(TFT_BLACK);
  drawPowerIcon(160, 90, 30, C_EDGE, TFT_BLACK);
  tft.setTextPadding(0);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(C_DIM, TFT_BLACK);
  tft.drawString("Radio off", 160, 150, 4);
  tft.setTextColor(C_EDGE, TFT_BLACK);
  tft.drawString("Tap anywhere to turn on", 160, 180, 2);
  snprintf(buf, sizeof(buf), "%u.%u MHz", currentFrequency / 100, (currentFrequency % 100) / 10);
  tft.drawString(buf, 160, 205, 2);
}

void drawSignal(bool force) {
  uint8_t rssi = rx.getRssi();
  lastRssi = rssi;
  if (inSettings) return;
  int8_t bars = rssi >= 50 ? 5 : rssi >= 40 ? 4 : rssi >= 30 ? 3 : rssi >= 20 ? 2 : rssi >= 10 ? 1 : 0;
  if (!force && bars == shownSignalBars) return;
  shownSignalBars = bars;
  for (uint8_t i = 0; i < 5; i++) {
    int16_t h = 4 + i * 4;
    tft.fillRect(274 + i * 8, 25 - h, 6, h, i < bars ? C_GREEN : C_EDGE);
  }
}

void drawFrequency() {
  if (inSettings) return;
  char buf[8];
  snprintf(buf, sizeof(buf), "%u.%u", currentFrequency / 100, (currentFrequency % 100) / 10);
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(C_ACCENT, C_PANEL);
  tft.setTextPadding(tft.textWidth("188.8", 7));
  tft.drawString(buf, FREQ_X, FREQ_Y, 7);
  tft.setTextPadding(0);
}

int16_t freqToX(uint16_t f) {
  f = constrain(f, FREQ_MIN, FREQ_MAX);
  return SCALE_X0 + (int32_t)(f - FREQ_MIN) * (SCALE_X1 - SCALE_X0) / (FREQ_MAX - FREQ_MIN);
}

void drawScale() {
  if (inSettings) return;
  tft.fillRect(0, 136, 320, 26, C_BG);
  tft.drawFastHLine(SCALE_X0, 156, SCALE_X1 - SCALE_X0 + 1, C_EDGE);
  tft.setTextDatum(BC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(C_DIM, C_BG);
  for (uint8_t mhz = 88; mhz <= 108; mhz++) {
    int16_t x = freqToX(mhz * 100);
    bool major = (mhz % 4 == 0);
    tft.drawFastVLine(x, major ? 149 : 152, major ? 7 : 4, major ? C_DIM : C_EDGE);
    if (major) tft.drawNumber(mhz, x, 147, 1);
  }
  int16_t nx = freqToX(currentFrequency);
  tft.fillRect(nx - 1, 141, 3, 20, C_RED);
  tft.fillTriangle(nx - 5, 137, nx + 5, 137, nx, 143, C_RED);
}

void drawVolume() {
  if (inSettings) return;
  uint8_t v = rx.getVolume();
  for (uint8_t i = 0; i < 16; i++) {
    uint16_t c = C_EDGE;
    if (i < v) c = i < 10 ? C_CYAN : i < 13 ? C_ACCENT : C_RED;
    tft.fillRoundRect(VOL_X + i * 16, VOL_Y, 13, 10, 2, c);
  }
}

void drawStationName() {
  if (inSettings) return;
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_GREEN, C_PANEL);
  tft.setTextPadding(130);
  tft.drawString(rdsName.shown, CARD_X + 10, CARD_Y + 6, 2);
  tft.setTextPadding(0);
}

void drawRdsTime() {
  if (inSettings) return;
  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(C_DIM, C_PANEL);
  tft.setTextPadding(90);
  tft.drawString(rdsClock.shown, CARD_X + CARD_W - 10, CARD_Y + 6, 2);
  tft.setTextPadding(0);
}

void drawRadioText() {
  if (inSettings) return;
  char win[RT_WINDOW + 1];
  const char* radioText = rdsText.shown;
  uint8_t len = strlen(radioText);

  // Padding overwrites the old text in place, so nothing is cleared first (no flicker)
  tft.setTextPadding(304);

  if (!bRds) {
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(C_EDGE, C_BG);
    tft.drawString("RDS off", 160, RT_Y, 2);
    tft.setTextPadding(0);
    return;
  }

  if (len <= RT_WINDOW) {
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(C_CYAN, C_BG);
    tft.drawString(radioText, 160, RT_Y, 2);
    tft.setTextPadding(0);
    return;
  }

  // Marquee: loop the text with a small gap between repeats
  uint8_t total = len + 4;
  for (uint8_t i = 0; i < RT_WINDOW; i++) {
    uint8_t idx = (rtOffset + i) % total;
    win[i] = idx < len ? radioText[idx] : ' ';
  }
  win[RT_WINDOW] = '\0';
  rtOffset = (rtOffset + 1) % total;

  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(C_CYAN, C_BG);
  tft.drawString(win, 10, RT_Y, 2);
  tft.setTextPadding(0);
}

void drawButton(const Button& b, bool pressed) {
  if (inSettings) return;
  uint16_t fill = pressed ? b.color : C_PANEL;
  uint16_t fg = pressed ? C_BG : b.color;
  int16_t cx = b.x + b.w / 2;
  int16_t iy = b.y + 17;

  tft.fillRoundRect(b.x, b.y, b.w, b.h, 8, fill);
  tft.drawRoundRect(b.x, b.y, b.w, b.h, 8, b.color);

  switch (b.icon) {
    case ICON_SEEK_DOWN:
      tft.fillTriangle(cx + 1, iy - 7, cx + 1, iy + 7, cx - 9, iy, fg);
      tft.fillTriangle(cx + 11, iy - 7, cx + 11, iy + 7, cx + 1, iy, fg);
      tft.fillRect(cx - 12, iy - 7, 3, 15, fg);
      break;
    case ICON_SEEK_UP:
      tft.fillTriangle(cx - 1, iy - 7, cx - 1, iy + 7, cx + 9, iy, fg);
      tft.fillTriangle(cx - 11, iy - 7, cx - 11, iy + 7, cx - 1, iy, fg);
      tft.fillRect(cx + 10, iy - 7, 3, 15, fg);
      break;
    case ICON_VOL_DOWN:
    case ICON_VOL_UP:
      tft.fillRect(cx - 14, iy - 3, 5, 7, fg);
      tft.fillTriangle(cx - 11, iy, cx - 3, iy - 8, cx - 3, iy + 8, fg);
      tft.fillRect(cx + 2, iy - 1, 11, 3, fg);
      if (b.icon == ICON_VOL_UP) tft.fillRect(cx + 6, iy - 5, 3, 11, fg);
      break;
  }

  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(fg, fill);
  tft.drawString(b.label, cx, b.y + b.h - 12, 2);
}

void drawUI() {
  tft.fillScreen(C_BG);

  // Header
  tft.setTextDatum(ML_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(C_ACCENT, C_BG);
  tft.drawString("FM", 40, 15, 4);
  tft.setTextColor(C_TEXT, C_BG);
  tft.drawString("RADIO", 76, 16, 1);
  drawPowerIcon(PWR_X, PWR_Y, 9, C_GREEN, C_BG);
  drawGear(GEAR_X, GEAR_Y, C_DIM, C_BG);
#if ENABLE_BLE
  drawBleDot();
#endif
  drawStereoPill();
  drawRdsPill();
  drawSignal(true);

  // Frequency card
  tft.fillRoundRect(CARD_X, CARD_Y, CARD_W, CARD_H, 10, C_PANEL);
  tft.drawRoundRect(CARD_X, CARD_Y, CARD_W, CARD_H, 10, C_EDGE);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(C_DIM, C_PANEL);
  tft.drawString("MHz", FREQ_X + 10, FREQ_Y + 8, 4);
  drawFrequency();
  drawStationName();
  drawRdsTime();

  drawRadioText();
  drawScale();

  // Volume
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(C_DIM, C_BG);
  tft.drawString("VOL", 8, VOL_Y + 5, 2);
  drawVolume();

  for (uint8_t i = 0; i < buttonCount; i++) drawButton(buttons[i], false);
}

// ============================================================
//  Settings page
// ============================================================

uint8_t pageEnd(uint8_t page) {
  return page + 1 < pageCount ? pageFirst[page + 1] : settingCount;
}

void formatSetting(const SettingDef& s, char* buf, size_t n) {
  uint8_t v = *s.value;
  if (s.labels) snprintf(buf, n, "%s", s.labels[v]);
  else snprintf(buf, n, "%u%s", v * s.scale, s.unit);
}

void drawSettingRow(uint8_t idx) {
  const SettingDef& s = settingDefs[idx];
  int16_t y = SET_ROW_Y + (idx - pageFirst[settingsPage]) * SET_ROW_H;
  char buf[12];

  tft.fillRect(0, y, 320, SET_ROW_H - 1, C_BG);
  tft.drawFastHLine(8, y + SET_ROW_H - 1, 304, C_PANEL);

  tft.setTextPadding(0);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_TEXT, C_BG);
  tft.drawString(s.name, 10, y + 2, 2);
  tft.setTextColor(C_DIM, C_BG);
  tft.drawString(s.hint, 10, y + 20, 1);

  formatSetting(s, buf, sizeof(buf));
  int16_t cy = y + (SET_ROW_H - 1) / 2;
  int16_t h = SET_ROW_H - 7;
  tft.setTextDatum(MC_DATUM);

  if (s.stepper) {
    int16_t plusX = SET_CTRL_X + SET_CTRL_W - SET_BTN_W;
    tft.fillRoundRect(SET_CTRL_X, y + 3, SET_BTN_W, h, 6, C_PANEL);
    tft.fillRoundRect(plusX, y + 3, SET_BTN_W, h, 6, C_PANEL);
    tft.drawRoundRect(SET_CTRL_X, y + 3, SET_BTN_W, h, 6, C_CYAN);
    tft.drawRoundRect(plusX, y + 3, SET_BTN_W, h, 6, C_CYAN);
    tft.setTextColor(C_CYAN, C_PANEL);
    tft.drawString("-", SET_CTRL_X + SET_BTN_W / 2, cy, 4);
    tft.drawString("+", plusX + SET_BTN_W / 2, cy, 4);
    tft.setTextColor(C_ACCENT, C_BG);
    tft.drawString(buf, SET_CTRL_X + SET_CTRL_W / 2, cy + 1, 2);
  } else {
    bool on = !(s.labels == LBL_OFF_ON && *s.value == 0);
    uint16_t fill = on ? C_PANEL : C_BG;
    tft.fillRoundRect(SET_CTRL_X, y + 3, SET_CTRL_W, h, h / 2, fill);
    tft.drawRoundRect(SET_CTRL_X, y + 3, SET_CTRL_W, h, h / 2, on ? C_ACCENT : C_EDGE);
    tft.setTextColor(on ? C_ACCENT : C_DIM, fill);
    tft.drawString(buf, SET_CTRL_X + SET_CTRL_W / 2, cy + 1, 2);
  }
}

// Live frequency + signal strength, handy while trying antenna / LNA settings
void drawSettingsSignal() {
  char buf[24];
  uint8_t rssi = rx.getRssi();
  lastRssi = rssi;
  snprintf(buf, sizeof(buf), "%u.%u  RSSI %u", currentFrequency / 100, (currentFrequency % 100) / 10, rssi);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(C_GREEN, C_BG);
  tft.setTextPadding(110);
  tft.drawString(buf, 72, 16, 2);
  tft.setTextPadding(0);
}

void drawSettingsHeader() {
  char buf[8];
  bool armed = resetArmedAt && millis() - resetArmedAt < 3000;

  tft.fillRect(0, 0, 320, SET_ROW_Y - 2, C_BG);
  tft.fillRoundRect(6, 4, 60, 24, 12, C_PANEL);
  tft.drawRoundRect(6, 4, 60, 24, 12, C_CYAN);
  tft.setTextDatum(MC_DATUM);
  tft.setTextPadding(0);
  tft.setTextColor(C_CYAN, C_PANEL);
  tft.drawString("< BACK", 36, 17, 2);

  snprintf(buf, sizeof(buf), "%u/%u >", settingsPage + 1, pageCount);
  tft.fillRoundRect(186, 4, 52, 24, 12, C_PANEL);
  tft.drawRoundRect(186, 4, 52, 24, 12, C_CYAN);
  tft.drawString(buf, 212, 17, 2);

  uint16_t rc = armed ? C_RED : C_DIM;
  tft.fillRoundRect(242, 4, 72, 24, 12, armed ? C_RED : C_PANEL);
  tft.drawRoundRect(242, 4, 72, 24, 12, rc);
  tft.setTextColor(armed ? C_BG : C_DIM, armed ? C_RED : C_PANEL);
  tft.drawString(armed ? "SURE?" : "DEFAULTS", 278, 17, 2);

  drawSettingsSignal();
}

void drawSettings() {
  tft.fillScreen(C_BG);
  drawSettingsHeader();
  for (uint8_t i = pageFirst[settingsPage]; i < pageEnd(settingsPage); i++) drawSettingRow(i);
#if ENABLE_BLE
  drawBleStatusLine();
#endif
  drawRestartButton();
}

void drawRestartButton() {
  if (settingsPage != pageCount - 1) return;
  bool armed = restartArmedAt && millis() - restartArmedAt < 3000;
  tft.fillRoundRect(RESTART_X, RESTART_Y, RESTART_W, RESTART_H, 12, armed ? C_RED : C_PANEL);
  tft.drawRoundRect(RESTART_X, RESTART_Y, RESTART_W, RESTART_H, 12, C_RED);
  tft.setTextPadding(0);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(armed ? C_BG : C_RED, armed ? C_RED : C_PANEL);
  tft.drawString(armed ? "TAP AGAIN TO RESTART" : "RESTART RADIO", 160, RESTART_Y + RESTART_H / 2 + 1, 2);
  tft.setTextColor(C_DIM, C_BG);
  tft.drawString("Use if the radio stops responding.", 160, RESTART_Y + RESTART_H + 16, 1);
  tft.drawString("Or hold the power button for 3 seconds.", 160, RESTART_Y + RESTART_H + 30, 1);
}

void openSettings() {
  inSettings = true;
  settingsPage = 0;
  resetArmedAt = restartArmedAt = 0;
  drawSettings();
}

void closeSettings() {
  saveConfig();
  inSettings = false;
  drawUI();
  clearRds();   // start RDS fresh (also needed if the RDS filter was switched)
}

void changeSetting(uint8_t idx, int8_t delta) {
  const SettingDef& s = settingDefs[idx];
  int16_t v = *s.value + delta;
  if (s.stepper) v = constrain(v, s.minV, s.maxV);
  else if (v > s.maxV) v = s.minV;   // tap cycles round
  if (v == *s.value) return;
  *s.value = v;
  applySettings();          // takes effect immediately so you can hear it
  drawSettingRow(idx);
#if ENABLE_BLE
  drawBleStatusLine();
#endif
}

void handleSettingsTouch(int16_t x, int16_t y, bool newPress, bool canRepeat) {
  if (y < SET_ROW_Y) {
    if (!newPress) return;
    if (x < 72) {
      closeSettings();
    } else if (x >= 182 && x < 240) {
      settingsPage = (settingsPage + 1) % pageCount;
      drawSettings();
    } else if (x >= 240) {
      if (resetArmedAt && millis() - resetArmedAt < 3000) {
        cfg = Config();
        applySettings();
        resetArmedAt = 0;
        drawSettings();
      } else {
        resetArmedAt = millis();
        drawSettingsHeader();
      }
    }
    return;
  }

  if (settingsPage == pageCount - 1 && inRect(x, y, RESTART_X, RESTART_Y - 6, RESTART_W, RESTART_H + 12)) {
    if (!newPress) return;
    if (restartArmedAt && millis() - restartArmedAt < 3000) restartRadio();
    restartArmedAt = millis();
    drawRestartButton();
    return;
  }

  uint8_t idx = pageFirst[settingsPage] + (y - SET_ROW_Y) / SET_ROW_H;
  if (idx >= pageEnd(settingsPage) || x < SET_CTRL_X - 6) return;
  const SettingDef& s = settingDefs[idx];

  if (s.stepper) {
    if (!newPress && !canRepeat) return;
    if (x < SET_CTRL_X + SET_BTN_W + 4) changeSetting(idx, -1);
    else if (x >= SET_CTRL_X + SET_CTRL_W - SET_BTN_W - 4) changeSetting(idx, 1);
  } else if (newPress) {
    changeSetting(idx, 1);
  }
}

// ============================================================
//  RDS
// ============================================================

// BASIC: the library decodes; a value is only shown once the same
// cleaned-up text has been read twice in a row.
// Returns true when the confirmed value of the field changed.
bool updateRds(RdsField& f, const char* src, size_t maxLen) {
  char tmp[65];
  size_t i = 0;
  if (src == NULL) return false;
  for (; i < maxLen && i < sizeof(tmp) - 1 && src[i] != '\0'; i++)
    tmp[i] = (src[i] >= 32 && src[i] < 127) ? src[i] : ' ';
  while (i > 0 && tmp[i - 1] == ' ') i--;
  tmp[i] = '\0';
  size_t start = 0;
  while (tmp[start] == ' ') start++;

  const char* clean = tmp + start;
  if (*clean == '\0' || strcmp(clean, f.shown) == 0) return false;
  if (strcmp(clean, f.candidate) != 0) {
    strcpy(f.candidate, clean);
    return false;
  }
  strcpy(f.shown, clean);
  return true;
}

void resetStrictRds() {
  memset(&srds, 0, sizeof(srds));
  srds.rtAB = -1;
  srds.rtLen = 64;
  srds.rtSegLen = 4;
  memset(srds.ps, ' ', sizeof(srds.ps));
  memset(srds.rt, ' ', sizeof(srds.rt));
}

void clearRds() {
  rx.clearRdsBuffer();
  memset(&rdsName, 0, sizeof(rdsName));
  memset(&rdsText, 0, sizeof(rdsText));
  memset(&rdsClock, 0, sizeof(rdsClock));
  resetStrictRds();
  shownClockMinute = -1;   // STRICT clock keeps running; redraw it on the next tick
  rtOffset = 0;
  drawStationName();
  drawRdsTime();
  drawRadioText();
}

char rdsChar(uint8_t c) {
  if (c == 0x0D) return '\r';
  return (c >= 0x20 && c < 0x7F) ? c : ' ';
}

// Counts how often the same text arrives for one segment.
// Returns true at the moment the segment becomes confirmed.
bool voteSegment(char* cand, uint8_t& hits, const char* got, uint8_t n) {
  if (memcmp(cand, got, n) == 0) {
    if (hits < 255) hits++;
  } else {
    memcpy(cand, got, n);
    hits = 1;
  }
  return hits == 2;
}

void trimRight(char* s) {
  int8_t i = strlen(s);
  while (i > 0 && s[i - 1] == ' ') s[--i] = '\0';
}

void publishStrictPs() {
  if (srds.psGood != 0x0F) return;   // wait for all four segments
  char tmp[9];
  memcpy(tmp, srds.ps, 8);
  tmp[8] = '\0';
  trimRight(tmp);
  if (strcmp(tmp, rdsName.shown) == 0) return;
  strcpy(rdsName.shown, tmp);
  drawStationName();
}

// Shows the confirmed beginning of the radio text; it grows as more arrives.
void publishStrictRt() {
  char tmp[65];
  uint8_t n = 0;
  for (uint8_t seg = 0; seg < 16 && n < srds.rtLen; seg++) {
    if (!(srds.rtGood & (1 << seg))) break;
    for (uint8_t k = 0; k < srds.rtSegLen && n < srds.rtLen; k++) tmp[n++] = srds.rt[seg * srds.rtSegLen + k];
  }
  tmp[n] = '\0';
  trimRight(tmp);
  uint8_t start = 0;
  while (tmp[start] == ' ') start++;
  if (strcmp(tmp + start, rdsText.shown) == 0) return;
  strcpy(rdsText.shown, tmp + start);
  rtOffset = 0;
  drawRadioText();
}

void strictPsGroup(uint16_t B, uint16_t D) {
  uint8_t seg = B & 0x03;
  char got[2] = { rdsChar(D >> 8), rdsChar(D & 0xFF) };
  for (uint8_t k = 0; k < 2; k++) if (got[k] == '\r') got[k] = ' ';
  if (!voteSegment(&srds.psCand[seg * 2], srds.psHits[seg], got, 2)) return;
  memcpy(&srds.ps[seg * 2], got, 2);
  srds.psGood |= 1 << seg;
  publishStrictPs();
}

void strictRtGroup(uint16_t B, uint16_t C, uint16_t D, bool versionB) {
  uint8_t segLen = versionB ? 2 : 4;
  int8_t ab = (B >> 4) & 1;

  // A/B flag flip (or 2A/2B switch) means the station started a new text
  if (ab != srds.rtAB || segLen != srds.rtSegLen) {
    memset(srds.rtCand, 0, sizeof(srds.rtCand));
    memset(srds.rtHits, 0, sizeof(srds.rtHits));
    memset(srds.rt, ' ', sizeof(srds.rt));
    srds.rtGood = 0;
    srds.rtLen = versionB ? 32 : 64;
    srds.rtAB = ab;
    srds.rtSegLen = segLen;
  }

  uint8_t seg = B & 0x0F;
  char got[4];
  if (versionB) {
    got[0] = rdsChar(D >> 8); got[1] = rdsChar(D & 0xFF);
  } else {
    got[0] = rdsChar(C >> 8); got[1] = rdsChar(C & 0xFF);
    got[2] = rdsChar(D >> 8); got[3] = rdsChar(D & 0xFF);
  }
  if (!voteSegment(&srds.rtCand[seg * segLen], srds.rtHits[seg], got, segLen)) return;

  for (uint8_t k = 0; k < segLen; k++) {
    uint8_t pos = seg * segLen + k;
    if (got[k] == '\r') {
      if (pos < srds.rtLen) srds.rtLen = pos;
      srds.rt[pos] = ' ';
    } else {
      srds.rt[pos] = got[k];
    }
  }
  srds.rtGood |= 1 << seg;
  publishStrictRt();
}

// Group 4A: clock time. Trusted only when two messages agree with the time that passed.
void strictClockGroup(uint16_t C, uint16_t D) {
  uint8_t hour = ((C & 1) << 4) | (D >> 12);
  uint8_t minute = (D >> 6) & 0x3F;
  int8_t offset = D & 0x1F;                 // local offset in half hours
  if (hour > 23 || minute > 59 || offset > 28) return;
  if (D & 0x20) offset = -offset;

  int16_t local = ((int16_t)hour * 60 + minute + offset * 30 + 1440) % 1440;
  unsigned long now = millis();

  if (ctHaveCandidate) {
    int16_t expected = (ctCandMinute + (now - ctCandAt + 30000) / 60000) % 1440;
    int16_t diff = abs(local - expected);
    if (diff <= 1 || diff >= 1439) {
      clockValid = true;
      clockMinute = local;
      clockAt = now;
      shownClockMinute = -1;
    }
  }
  ctHaveCandidate = true;
  ctCandMinute = local;
  ctCandAt = now;
}

void processStrictRds(const uint16_t* r) {
  if (r[0] == 0xFFFF || r[1] == 0xFFFF) return;   // failed I2C read
  if (!(r[0] & 0x1000) || (r[1] & 0x0010)) return; // not synchronised / block E
  if ((r[1] & 0x03) != 0) return;                  // block B (type + address) must be clean

  uint16_t A = r[2], B = r[3], C = r[4], D = r[5];
  if (B == srds.lastB && C == srds.lastC && D == srds.lastD) return;   // same group read twice

  if (((r[1] >> 2) & 0x03) == 0) {
    if (srds.pi != 0 && A != srds.pi) clearRds();                     // different station
    srds.pi = A;
  }
  srds.lastB = B; srds.lastC = C; srds.lastD = D;

  bool versionB = B & 0x0800;
  switch (B >> 12) {
    case 0: strictPsGroup(B, D); break;
    case 2: strictRtGroup(B, C, D, versionB); break;
    case 4: if (!versionB) strictClockGroup(C, D); break;
  }
}

void updateStrictClock() {
  if (!cfg.rdsStrict || !clockValid) return;
  int16_t m = (clockMinute + (millis() - clockAt) / 60000) % 1440;
  if (m == shownClockMinute) return;
  shownClockMinute = m;
  snprintf(rdsClock.shown, sizeof(rdsClock.shown), "%02d:%02d", m / 60, m % 60);
  drawRdsTime();
}

void checkRDS() {
  if (!rx.getRdsReady()) return;   // also reads all status registers
  if (cfg.rdsStrict) {
    processStrictRds(rx.statusRegs());
    return;
  }
  if (!rx.hasRdsInfo()) return;
  if (updateRds(rdsName, rx.getRdsStationName(), 8)) drawStationName();
  if (updateRds(rdsClock, rx.getRdsTime(), 16)) drawRdsTime();
  if (updateRds(rdsText, rx.getRdsProgramInformation(), 64)) {
    rtOffset = 0;
    drawRadioText();
  }
}

// ============================================================
//  Radio actions
// ============================================================

bool inBand(uint16_t f) {
  return f >= CHIP_FREQ_MIN && f <= FREQ_MAX;
}

void onFrequencyChanged(uint16_t f) {
  currentFrequency = f;
  drawFrequency();
  drawScale();
  clearRds();
  markDirty();
}

// Called by rx.seek() on every step so the display follows the scan
void showFrequencySeek() {
  esp_task_wdt_reset();
  uint16_t f = rx.getFrequency();
  if (!inBand(f) || f == currentFrequency) return;   // skip glitched I2C reads
  currentFrequency = f;
  drawFrequency();
  drawScale();
}

// Reads the channel the chip is really on, retrying through I2C glitches.
// A failed read comes back as all ones = channel 1023 = 189.3 MHz. Returns 0 if no good read.
uint16_t readTunedFrequency() {
  for (uint8_t i = 0; i < 5; i++) {
    uint16_t f = rx.getRealFrequency();
    if (inBand(f)) return f;
    delay(5);
  }
  return 0;
}

void seek(uint8_t direction) {
  uint16_t start = currentFrequency;
  if (cfg.muteSeek) rx.setMute(true);
  rx.seek(RDA_SEEK_WRAP, direction, showFrequencySeek);

  // The library trusts every read during seek, so check where it really ended up.
  // If it can't be confirmed, go back to the station we started from.
  uint16_t f = readTunedFrequency();
  if (f == 0) f = start;
  if (f != rx.getFrequency()) rx.setFrequency(f);
  if (cfg.muteSeek) rx.setMute(false);

  onFrequencyChanged(f);
  drawSignal(true);
}

void toggleStereo() {
  bSt = !bSt;
  rx.setMono(!bSt);
  drawStereoPill();
  markDirty();
}

void toggleRds() {
  bRds = !bRds;
  rx.setRDS(bRds);
  rx.setRdsFifo(bRds);
  drawRdsPill();
  clearRds();
  markDirty();
}

// ============================================================
//  Power and restart
// ============================================================

// "Off" mutes the tuner and shows a dim standby screen; any tap turns it back on
void setPower(bool on) {
  if (on == poweredOn) return;
  poweredOn = on;
  if (on) {
    rx.setMute(false);
    analogWrite(SCREEN_BACKLIGHT_PIN, 255);
    drawUI();
    clearRds();
  } else {
    if (inSettings) {
      inSettings = false;
      saveConfig();
    }
    rx.setMute(true);
    if (settingsDirty) {
      saveAllReceiverInformation();
      if (cfgDirty) saveConfig();
      settingsDirty = cfgDirty = false;
    }
    drawStandby();
    analogWrite(SCREEN_BACKLIGHT_PIN, STANDBY_BACKLIGHT);
  }
}

// Saves everything, resets the tuner chip (in case it is what hung) and reboots the ESP32
void restartRadio() {
  saveAllReceiverInformation();
  saveConfig();
  analogWrite(SCREEN_BACKLIGHT_PIN, 255);
  tft.fillScreen(C_BG);
  tft.setTextPadding(0);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(C_ACCENT, C_BG);
  tft.drawString("Restarting...", 160, 120, 4);
  rx.setMute(true);
  rx.softReset();
  delay(200);
  ESP.restart();
}

// The ESP32 restarts by itself if loop() stops running for WDT_TIMEOUT_S
void startWatchdog() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdt = { .timeout_ms = WDT_TIMEOUT_S * 1000, .idle_core_mask = 0, .trigger_panic = true };
  if (esp_task_wdt_reconfigure(&wdt) != ESP_OK) esp_task_wdt_init(&wdt);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);
}

// ============================================================
//  Remote control (commands from the phone)
// ============================================================

void tuneTo(uint16_t f) {
  if (!inBand(f)) return;
  rx.setFrequency(f);
  onFrequencyChanged(f);
  drawSignal(true);
}

// Commands: S+ S-  V+ V- V=n  T=freq  ST=0/1  RDS=0/1  P=n (tune preset)
//           PS=n (save current to preset)  PC=n (clear preset)  C=index,value (setting)
//           PWR=0/1 (off/on)  RESET (restart)
void handleRemoteCommand(const char* c) {
  int a, b;
  if (!strcmp(c, "RESET")) { restartRadio(); return; }
  if (sscanf(c, "PWR=%d", &a) == 1) { setPower(a != 0); return; }
  if (!poweredOn) return;   // everything else waits until the radio is turned on
  if (!strcmp(c, "S+")) seek(1);
  else if (!strcmp(c, "S-")) seek(0);
  else if (!strcmp(c, "V+")) { rx.setVolumeUp(); drawVolume(); markDirty(); }
  else if (!strcmp(c, "V-")) { rx.setVolumeDown(); drawVolume(); markDirty(); }
  else if (sscanf(c, "V=%d", &a) == 1) { rx.setVolume(constrain(a, 0, 15)); drawVolume(); markDirty(); }
  else if (sscanf(c, "T=%d", &a) == 1) tuneTo(a);
  else if (sscanf(c, "ST=%d", &a) == 1) { if ((a != 0) != bSt) toggleStereo(); }
  else if (sscanf(c, "RDS=%d", &a) == 1) { if ((a != 0) != bRds) toggleRds(); }
  else if (sscanf(c, "PS=%d", &a) == 1) { if (a >= 0 && a < PRESET_COUNT) { presets[a] = currentFrequency; savePresets(); } }
  else if (sscanf(c, "PC=%d", &a) == 1) { if (a >= 0 && a < PRESET_COUNT) { presets[a] = 0; savePresets(); } }
  else if (sscanf(c, "P=%d", &a) == 1) { if (a >= 0 && a < PRESET_COUNT && presets[a]) tuneTo(presets[a]); }
  else if (sscanf(c, "C=%d,%d", &a, &b) == 2) {
    if (a < 0 || a >= settingCount) return;
    const SettingDef& s = settingDefs[a];
    if (b < s.minV || b > s.maxV || b == *s.value) return;
    *s.value = b;
    applySettings();
    cfgDirty = true;
    markDirty();
    if (inSettings && a >= pageFirst[settingsPage] && a < pageEnd(settingsPage)) drawSettingRow(a);
  }
}

#if ENABLE_BLE
// ============================================================
//  Bluetooth LE
// ============================================================

class BleServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& info) override {
    bleWantStatus = true;
  }
  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& info, int reason) override {
    if (cfg.ble) NimBLEDevice::startAdvertising();
  }
};

class BleCmdCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* ch, NimBLEConnInfo& info) override {
    BleCmd cmd = {};
    const NimBLEAttValue& v = ch->getValue();
    size_t n = min((size_t)v.length(), sizeof(cmd.text) - 1);
    memcpy(cmd.text, v.data(), n);
    // loop() is stuck, so the queue would never be read: restart right here
    if (!strcmp(cmd.text, "RESET") && millis() - lastLoopMs > 3000) ESP.restart();
    xQueueSend(bleCmdQueue, &cmd, 0);
  }
};

void bleStart() {
  NimBLEDevice::init(BLE_NAME);
  NimBLEDevice::setPower(3);   // plenty inside a car, and less RF next to the FM tuner

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new BleServerCallbacks());
  bleServer->advertiseOnDisconnect(false);   // we restart it ourselves, only if Bluetooth is on

  NimBLEService* svc = bleServer->createService(BLE_SERVICE_UUID);
  bleState = svc->createCharacteristic(BLE_STATE_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic* cmd = svc->createCharacteristic(BLE_CMD_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  cmd->setCallbacks(new BleCmdCallbacks());
  svc->start();

  // 128-bit service UUID in the advert, name in the scan response (both don't fit in one packet)
  NimBLEAdvertisementData advData, scanData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  advData.setCompleteServices(NimBLEUUID(BLE_SERVICE_UUID));
  scanData.setName(BLE_NAME);
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanData);
  adv->enableScanResponse(true);
}

// Starts or stops Bluetooth to match the setting
void bleApply() {
  if (cfg.ble) {
    if (!bleServer) bleStart();
    if (bleServer->getConnectedCount() == 0) NimBLEDevice::startAdvertising();
  } else if (bleServer) {
    NimBLEDevice::stopAdvertising();
    for (uint16_t h : bleServer->getPeerDevices()) bleServer->disconnect(h);
  }
}

bool bleConnected() {
  return bleServer && bleServer->getConnectedCount() > 0;
}

// Small dot next to the gear: lit while a phone is connected
void drawBleDot() {
  if (inSettings || !poweredOn) return;
  tft.fillCircle(GEAR_X + 13, GEAR_Y - 9, 3, bleShownConnected ? C_CYAN : C_BG);
}

void drawBleStatusLine() {
  if (settingsPage != pageCount - 1) return;
  const char* msg = !cfg.ble ? "Bluetooth is off"
                  : bleShownConnected ? "Phone connected"
                  : "Waiting for phone: \"" BLE_NAME "\"";
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(bleShownConnected ? C_GREEN : C_DIM, C_BG);
  tft.setTextPadding(300);
  tft.drawString(msg, 10, SET_ROW_Y + SET_ROW_H + 14, 2);
  tft.setTextPadding(0);
}

void jsonString(char*& p, char* end, const char* key, const char* val) {
  p += snprintf(p, end - p, ",\"%s\":\"", key);
  for (; *val && p < end - 3; val++) {
    if (*val == '"' || *val == '\\') *p++ = '\\';
    *p++ = *val;
  }
  p += snprintf(p, end - p, "\"");
}

void buildStatusJson(char* out, size_t n) {
  char* p = out;
  char* end = out + n;
  p += snprintf(p, end - p, "{\"f\":%u,\"v\":%u,\"st\":%d,\"rds\":%d,\"rssi\":%u,\"pwr\":%d",
                currentFrequency, rx.getVolume(), bSt, bRds, lastRssi, poweredOn);
  jsonString(p, end, "ps", rdsName.shown);
  jsonString(p, end, "rt", rdsText.shown);
  jsonString(p, end, "ct", rdsClock.shown);
  p += snprintf(p, end - p, ",\"p\":[");
  for (uint8_t i = 0; i < PRESET_COUNT; i++) p += snprintf(p, end - p, i ? ",%u" : "%u", presets[i]);
  p += snprintf(p, end - p, "],\"c\":[");
  for (uint8_t i = 0; i < settingCount; i++) p += snprintf(p, end - p, i ? ",%u" : "%u", *settingDefs[i].value);
  snprintf(p, end - p, "]}");
}

void bleLoop() {
  BleCmd cmd;
  bool handled = false;
  while (xQueueReceive(bleCmdQueue, &cmd, 0) == pdTRUE) {
    handleRemoteCommand(cmd.text);
    handled = true;
  }

  bool conn = bleConnected();
  if (conn != bleShownConnected) {
    bleShownConnected = conn;
    drawBleDot();
    if (inSettings) drawBleStatusLine();
  }
  if (!conn) return;

  // Send the status when something changed (checked every 300 ms, or right after a command)
  if (handled || bleWantStatus || millis() - lastBleStatus > 300) {
    char json[sizeof(lastStatusJson)];
    buildStatusJson(json, sizeof(json));
    if (bleWantStatus || strcmp(json, lastStatusJson) != 0) {
      strcpy(lastStatusJson, json);
      bleState->setValue((const uint8_t*)json, strlen(json));
      bleState->notify();
    }
    bleWantStatus = false;
    lastBleStatus = millis();
  }
}
#endif

// ============================================================
//  Touch
// ============================================================

bool inRect(int16_t x, int16_t y, int16_t rx_, int16_t ry, int16_t rw, int16_t rh) {
  return x >= rx_ && x < rx_ + rw && y >= ry && y < ry + rh;
}

void handleTouch() {
  static unsigned long lastSeen = 0;
  static unsigned long lastAction = 0;
  static unsigned long powerPressAt = 0;   // power button: tap = off, hold = restart
  static bool waitRelease = false;         // the tap that woke the radio shouldn't also press a button

  bool touched = touch.touched();

  if (waitRelease) {
    if (touched) lastSeen = millis();
    else if (millis() - lastSeen > TOUCH_RELEASE_MS) waitRelease = false;
    return;
  }

  if (powerPressAt) {
    if (touched) {
      if (millis() - powerPressAt > RESTART_HOLD_MS) restartRadio();
      lastSeen = millis();
    } else if (millis() - lastSeen > TOUCH_RELEASE_MS) {
      powerPressAt = 0;
      setPower(false);
    }
    return;
  }

  if (!touched) return;

  TS_Point p = touch.getPoint();
  int16_t x = constrain(map(p.x, TOUCH_X_MIN, TOUCH_X_MAX, 0, 319), 0, 319);
  int16_t y = constrain(map(p.y, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, 239), 0, 239);

#if TOUCH_DEBUG
  Serial.printf("touch raw x=%d y=%d z=%d -> screen x=%d y=%d\n", p.x, p.y, p.z, x, y);
  tft.fillCircle(x, y, 2, C_RED);
#endif

  bool newPress = millis() - lastSeen > TOUCH_RELEASE_MS;
  lastSeen = millis();
  bool canRepeat = millis() - lastAction > TOUCH_REPEAT_MS;

  if (!poweredOn) {
    setPower(true);
    waitRelease = true;
    return;
  }

  if (inSettings) {
    if (newPress || canRepeat) lastAction = millis();
    handleSettingsTouch(x, y, newPress, canRepeat);
    lastSeen = millis();
    return;
  }

  // Bottom buttons
  for (uint8_t i = 0; i < buttonCount; i++) {
    const Button& b = buttons[i];
    if (!inRect(x, y, b.x - 3, b.y - 6, b.w + 6, b.h + 6)) continue;

    bool isVol = (b.icon == ICON_VOL_DOWN || b.icon == ICON_VOL_UP);
    if (!newPress && !(isVol && canRepeat)) return;

    drawButton(b, true);
    switch (b.icon) {
      case ICON_SEEK_DOWN: seek(0); break;
      case ICON_SEEK_UP:   seek(1); break;
      case ICON_VOL_DOWN:  rx.setVolumeDown(); drawVolume(); markDirty(); delay(60); break;
      case ICON_VOL_UP:    rx.setVolumeUp();   drawVolume(); markDirty(); delay(60); break;
    }
    drawButton(b, false);
    lastAction = millis();
    lastSeen = millis();   // don't treat time spent seeking as a release
    return;
  }

  // Header: gear and toggles (touch zones are taller than the icons for easier hits)
  if (newPress && y < 34) {
    if (inRect(x, y, 0, 0, 38, 34)) {
      powerPressAt = millis();
      drawPowerIcon(PWR_X, PWR_Y, 9, C_RED, C_BG);   // red while held
    }
    else if (inRect(x, y, GEAR_X - 22, 0, 44, 34)) openSettings();
    else if (inRect(x, y, PILL_ST_X - 4, 0, PILL_ST_W + 6, 34)) toggleStereo();
    else if (inRect(x, y, PILL_RDS_X - 2, 0, PILL_RDS_W + 6, 34)) toggleRds();
    lastAction = millis();
    lastSeen = millis();
  }
}

// ============================================================
//  EEPROM
// ============================================================

void saveAllReceiverInformation() {
  EEPROM.write(eeprom_address, app_id);
  EEPROM.write(eeprom_address + 1, rx.getVolume());
  EEPROM.write(eeprom_address + 2, currentFrequency >> 8);
  EEPROM.write(eeprom_address + 3, currentFrequency & 0xFF);
  EEPROM.write(eeprom_address + 4, (uint8_t)bRds);
  EEPROM.write(eeprom_address + 5, (uint8_t)bSt);
  EEPROM.commit();
}

void savePresets() {
  EEPROM.write(preset_address, preset_id);
  for (uint8_t i = 0; i < PRESET_COUNT; i++) {
    EEPROM.write(preset_address + 1 + i * 2, presets[i] >> 8);
    EEPROM.write(preset_address + 2 + i * 2, presets[i] & 0xFF);
  }
  EEPROM.commit();
}

void loadPresets() {
  if (EEPROM.read(preset_address) != preset_id) return;
  for (uint8_t i = 0; i < PRESET_COUNT; i++) {
    uint16_t f = (EEPROM.read(preset_address + 1 + i * 2) << 8) | EEPROM.read(preset_address + 2 + i * 2);
    presets[i] = inBand(f) ? f : 0;
  }
}

void readAllReceiverInformation() {
  rx.setVolume(EEPROM.read(eeprom_address + 1));
  currentFrequency = EEPROM.read(eeprom_address + 2) << 8;
  currentFrequency |= EEPROM.read(eeprom_address + 3);
  if (!inBand(currentFrequency)) currentFrequency = 9340;

  bRds = (bool)EEPROM.read(eeprom_address + 4);
  rx.setRDS(bRds);
  rx.setRdsFifo(bRds);

  bSt = (bool)EEPROM.read(eeprom_address + 5);
  rx.setMono(!bSt);
}

// ============================================================

void setup() {
  Serial.begin(115200);
  EEPROM.begin(64);

  pinMode(SCREEN_BACKLIGHT_PIN, OUTPUT);
  digitalWrite(SCREEN_BACKLIGHT_PIN, HIGH);

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(C_BG);

  touchSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touch.begin(touchSPI);
  touch.setRotation(3);   // keep in step with tft.setRotation()

  Wire.begin(I2C_SDA, I2C_SCL);
  rx.setup();

  if (EEPROM.read(eeprom_address) == app_id) {
    readAllReceiverInformation();
  } else {
    rx.setVolume(6);
    rx.setMono(false);
    rx.setRDS(true);
    rx.setRdsFifo(true);
    currentFrequency = 9340;
  }

  loadConfig();
  loadPresets();
#if ENABLE_BLE
  bleCmdQueue = xQueueCreate(8, sizeof(BleCmd));
#endif
  applySettings();
  rx.setFrequency(currentFrequency);
  resetStrictRds();

  drawUI();

  lastStatus = lastRdsPoll = lastRdsScroll = millis();
  startWatchdog();
}

void loop() {
  esp_task_wdt_reset();
  lastLoopMs = millis();

  handleTouch();
#if ENABLE_BLE
  bleLoop();
#endif

  if (!poweredOn) {
    delay(20);
    return;
  }

  if (inSettings) {
    if (millis() - lastStatus > 500) {
      drawSettingsSignal();
      if (resetArmedAt && millis() - resetArmedAt >= 3000) {
        resetArmedAt = 0;
        drawSettingsHeader();
      }
      if (restartArmedAt && millis() - restartArmedAt >= 3000) {
        restartArmedAt = 0;
        drawRestartButton();
      }
      lastStatus = millis();
    }
    delay(5);
    return;
  }

  if (millis() - lastStatus > STATUS_POLL_MS) {
    uint16_t f = rx.getFrequency();
    if (inBand(f) && f != currentFrequency) onFrequencyChanged(f);
    drawSignal(false);
    updateStrictClock();
    lastStatus = millis();
  }

  if (bRds && millis() - lastRdsPoll > RDS_POLL_MS) {
    checkRDS();
    lastRdsPoll = millis();
  }

  if (bRds && strlen(rdsText.shown) > RT_WINDOW && millis() - lastRdsScroll > RDS_SCROLL_MS) {
    drawRadioText();
    lastRdsScroll = millis();
  }

  if (settingsDirty && millis() - lastChange > STORE_TIME) {
    saveAllReceiverInformation();
    if (cfgDirty) saveConfig();
    settingsDirty = cfgDirty = false;
  }

  delay(5);
}
