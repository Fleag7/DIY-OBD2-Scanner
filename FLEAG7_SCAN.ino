#include <U8g2lib.h>
#include <STM32_CAN.h>

// ─── OBD2 PID METADATA TABLE ─────────────────────────────────────────────────
// IMPORTANT: this struct MUST stay right here, immediately below the includes.
// The Arduino IDE auto-generates function prototypes for every function in
// this sketch and inserts them near the top of the translated file, before
// any of your own code below this point. If PIDInfo is defined further down
// (like it was before), the auto-generated prototype for decodePIDValue()
// references a type that doesn't exist yet -> "PIDInfo does not name a type".
// Defining it here, before that insertion point, fixes it permanently.
struct PIDInfo {
  uint8_t pid;
  const char* label;   // kept short — display is only ~10 chars wide per row
  const char* unit;
  bool twoByte;         // true = uses (A*256)+B, false = uses A only
  float scale;
  float offset;
};

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ─── ENCODER PINS ───────────────────────────────────────────────────────────
#define CLK PA8  // Rotary encoder clock pin
#define DT  PB15  // Rotary encoder data pin — read to determine direction
#define SW  PB14  // Rotary encoder pushbutton

// ─── ENCODER STATE MACHINE (quadrature decode) ─────────────────────────────
// Ben Buxton's full-step table. Tracks CLK+DT as a combined state and only
// registers a count after a complete, valid 4-transition sequence — bounce
// and noise land on invalid transitions and get discarded, not counted.
// Driven by attachInterrupt() on both CLK and DT (see setup()), so it can't
// miss transitions while the display is blocked doing a blocking I2C redraw.

#define R_START      0x0
#define R_CW_FINAL   0x1
#define R_CW_BEGIN   0x2
#define R_CW_NEXT    0x3
#define R_CCW_BEGIN  0x4
#define R_CCW_FINAL  0x5
#define R_CCW_NEXT   0x6
#define DIR_NONE     0x0
#define DIR_CW       0x10
#define DIR_CCW      0x20

const unsigned char ttable[7][4] = {
  {R_START,     R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  {R_CW_NEXT,   R_START,     R_CW_FINAL,  R_START | DIR_CW},
  {R_CW_NEXT,   R_CW_BEGIN,  R_START,     R_START},
  {R_CW_NEXT,   R_CW_BEGIN,  R_CW_FINAL,  R_START},
  {R_CCW_NEXT,  R_START,     R_CCW_BEGIN, R_START},
  {R_CCW_NEXT,  R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  {R_CCW_NEXT,  R_CCW_FINAL, R_CCW_BEGIN, R_START},
};

volatile unsigned char encoderState = R_START;
volatile int encoderDelta = 0; // written only in encoderISR(), read/cleared in loop()

void encoderISR() {
  unsigned char pinstate = (digitalRead(CLK) << 1) | digitalRead(DT);
  encoderState = ttable[encoderState & 0x0F][pinstate];
  unsigned char result = encoderState & 0x30;
  if (result == DIR_CW)  encoderDelta++;
  if (result == DIR_CCW) encoderDelta--;
}

// ─── BUTTON (SW) ─────────────────────────────────────────────────────────────
// Simple polled debounce — SW doesn't need interrupt treatment like CLK/DT do,
// a press held for one 250ms window is plenty responsive for menu navigation.
// pollButton() itself is defined further down, AFTER needsRedraw is declared
// (see the REDRAW FLAG section) — it references needsRedraw, so it has to
// come after that declaration or the compiler won't know what it is yet.
bool buttonPressed = false;
unsigned long lastButtonTime = 0;

// ─── CAN BUS ────────────────────────────────────────────────────────────────
STM32_CAN Can(PA_11, PA_12); // explicit PinName format — ensures correct alternate function mapping
//
// CAN test result strings — updated by the test routine and displayed on screen
const char* canTestStatus = "WAITING";
const char* canTestDetail = "Press to run test";
bool lastClearSuccess = false;

// ─── SCREEN STATES ──────────────────────────────────────────────────────────
enum Screen {
  SCREEN_SPLASH,
  SCREEN_MAIN_MENU,
  SCREEN_DTC_SCANNING,
  SCREEN_NO_DTCS,
  SCREEN_DTC_LIST,
  SCREEN_CLEARING_DTCS,
  SCREEN_DTCS_CLEARED,
  SCREEN_VEHICLE_DATA,
  SCREEN_MONITORS,
  SCREEN_CAN_TEST  // CAN bus loopback test screen
};

// ─── MENU ───────────────────────────────────────────────────────────────────
// To add menu items: add string here and increment MENU_COUNT
const char* menuItems[] = {
  "DTC Scan",
  "Clear DTCs",
  "Vehicle Data",
  "Monitors",
  "CAN Bus Test",

};
const int MENU_COUNT   = 5;
const int VISIBLE_ROWS = 4;
int menuIndex  = 0;
int menuScroll = 0;

// ─── DTC DATA ───────────────────────────────────────────────────────────────
// !! REPLACE dtcList WITH REAL CAN DATA WHEN READY !!
// !! UPDATE DTC_COUNT TO MATCH !!
struct DTC { char code[6]; };
#define MAX_DTC_COUNT 32   // generous ceiling — real PCMs won't get near this
DTC dtcList[MAX_DTC_COUNT];
int DTC_COUNT = 0;

int dtcScroll = 0;
int dtcIndex  = 0;

// ─── VEHICLE DATA ────────────────────────────────────────────────────────────
struct VehicleDataItem {
  const char* label;
  char value[10];   // formatted number as text, e.g. "820" or "3.21"
  const char* unit;
};
#define MAX_VDATA_COUNT 40
uint8_t discoveredPIDs[MAX_VDATA_COUNT];
int discoveredPIDCount = 0;

VehicleDataItem vehicleData[MAX_VDATA_COUNT];
int VDATA_COUNT = 0; // no longer const — set after discovery
int vdataScroll = 0;

// NOTE: struct PIDInfo itself now lives at the very top of the file — see the
// note there. pidTable[] just uses it here like normal.
const PIDInfo pidTable[] = {
  {0x04, "Eng Load",  "%",   false, 100.0f/255.0f,  0.0f},
  {0x05, "Coolant",   "F",   false, 1.8f,          -40.0f},
  {0x06, "STFT B1",   "%",   false, 100.0f/128.0f, -100.0f},
  {0x07, "LTFT B1",   "%",   false, 100.0f/128.0f, -100.0f},
  {0x08, "STFT B2",   "%",   false, 100.0f/128.0f, -100.0f},
  {0x09, "LTFT B2",   "%",   false, 100.0f/128.0f, -100.0f},
  {0x0A, "Fuel Prs",  "psi", false, 0.435114f,      0.0f},
  {0x0B, "MAP",       "psi", false, 0.145038f,      0.0f},
  {0x0C, "RPM",       "rpm", true,  0.25f,          0.0f},
  {0x0D, "Speed",     "mph", false, 0.621371f,      0.0f},
  {0x0E, "Tmg Adv",   "deg", false, 0.5f,          -64.0f},
  {0x0F, "Intk Tmp",  "F",   false, 1.8f,          -40.0f},
  {0x10, "MAF",       "g/s", true,  0.01f,          0.0f},
  {0x11, "Throttle",  "%",   false, 100.0f/255.0f,  0.0f},
  {0x14, "O2 B1S1",   "V",   false, 0.005f,         0.0f},
  {0x15, "O2 B1S2",   "V",   false, 0.005f,         0.0f},
  {0x16, "O2 B1S3",   "V",   false, 0.005f,         0.0f},
  {0x17, "O2 B1S4",   "V",   false, 0.005f,         0.0f},
  {0x18, "O2 B2S1",   "V",   false, 0.005f,         0.0f},
  {0x19, "O2 B2S2",   "V",   false, 0.005f,         0.0f},
  {0x1A, "O2 B2S3",   "V",   false, 0.005f,         0.0f},
  {0x1B, "O2 B2S4",   "V",   false, 0.005f,         0.0f},
  {0x1F, "Runtime",   "sec", true,  1.0f,           0.0f},
  {0x21, "Dist MIL",  "mi",  true,  0.621371f,      0.0f},
  {0x2C, "Cmd EGR",   "%",   false, 100.0f/255.0f,  0.0f},
  {0x2D, "EGR Err",   "%",   false, 100.0f/128.0f, -100.0f},
  {0x2E, "Evap Prg",  "%",   false, 100.0f/255.0f,  0.0f},
  {0x2F, "Fuel Lvl",  "%",   false, 100.0f/255.0f,  0.0f},
  {0x30, "WarmUps",   "",    false, 1.0f,           0.0f},
  {0x31, "Dist Clr",  "mi",  true,  0.621371f,      0.0f},
  {0x33, "Baro",      "psi", false, 0.145038f,      0.0f},
  {0x3C, "Cat T B1S1","F",   true,  0.18f,         -40.0f},
  {0x3D, "Cat T B2S1","F",   true,  0.18f,         -40.0f},
  {0x3E, "Cat T B1S2","F",   true,  0.18f,         -40.0f},
  {0x3F, "Cat T B2S2","F",   true,  0.18f,         -40.0f},
  {0x42, "Mod Volt",  "V",   true,  0.001f,         0.0f},
  {0x43, "Abs Load",  "%",   true,  100.0f/255.0f,  0.0f},
  {0x45, "Rel Thrtl",  "%",   false, 100.0f/255.0f,  0.0f},
  {0x46, "Amb Temp",  "F",   false, 1.8f,          -40.0f},
  {0x47, "Thrtl B",   "%",   false, 100.0f/255.0f,  0.0f},
  {0x48, "Thrtl C",   "%",   false, 100.0f/255.0f,  0.0f},
  {0x49, "Pedal D",   "%",   false, 100.0f/255.0f,  0.0f},
  {0x4A, "Pedal E",   "%",   false, 100.0f/255.0f,  0.0f},
  {0x4B, "Pedal F",   "%",   false, 100.0f/255.0f,  0.0f},
  {0x4C, "Cmd Thrtl", "%",   false, 100.0f/255.0f,  0.0f},
  {0x4D, "Time MIL",  "min", true,  1.0f,           0.0f},
  {0x4E, "Time Clr",  "min", true,  1.0f,           0.0f},
  {0x52, "Ethanol",   "%",   false, 100.0f/255.0f,  0.0f},
  {0x59, "Rail Prs",  "psi", true,  1.45038f,       0.0f},
  {0x5A, "Rel Pedal", "%",   false, 100.0f/255.0f,  0.0f},
  {0x5C, "Oil Temp",  "F",   false, 1.8f,          -40.0f},
  {0x5E, "Fuel Rate", "gal/h", true, 0.0132086f,    0.0f},
};
const int PID_TABLE_COUNT = sizeof(pidTable) / sizeof(pidTable[0]);


// Decodes raw response bytes A,B using the PID's known formula.
float decodePIDValue(const PIDInfo &info, uint8_t A, uint8_t B) {
  float raw = info.twoByte ? ((A * 256) + B) : A;
  return (raw * info.scale) + info.offset;
}

// Finds a PID's metadata by its ID. Returns nullptr if not in our table.
const PIDInfo* findPIDInfo(uint8_t pid) {
  for (int i = 0; i < PID_TABLE_COUNT; i++) {
    if (pidTable[i].pid == pid) return &pidTable[i];
  }
  return nullptr;
}



// ─── MONITOR DATA ────────────────────────────────────────────────────────────
// !! REPLACE ready flags WITH REAL CAN DATA WHEN READY !!
// !! UPDATE MONITOR_COUNT TO MATCH !!
struct MonitorItem {
  const char* name;
  bool ready;
};
#define MONITOR_COUNT_MAX 11 // 3 continuous + 8 non-continuous, spark ignition
MonitorItem monitors[MONITOR_COUNT_MAX];
int MONITOR_COUNT = 0; // no longer const — set by performMonitorScan()
int monitorScroll = 0;


// Continuous monitors — byte B, bits 0-2 support / bits 4-6 not-complete
struct ContinuousMonitorDef {
  uint8_t bitPos; // 0,1,2 — used for BOTH support and (bitPos+4) not-complete check
  const char* name;
};
const ContinuousMonitorDef continuousMonitors[] = {
  {0, "Misfire"},
  {1, "Fuel System"},
  {2, "Components"},
};
const int CONTINUOUS_MONITOR_COUNT = 3;

// Non-continuous monitors — byte C = support, byte D = not-complete, same bit positions
// Spark ignition only (gas engines) — compression ignition uses a different set entirely
struct MonitorDef {
  uint8_t bitPos;
  const char* name;
};
const MonitorDef monitorDefs[] = {
  {0, "Catalyst"},
  {1, "Ht Catalyst"},
  {2, "Evap System"},
  {3, "Sec Air Sys"},
  {4, "AC Refrig"},
  {5, "O2 Sensor"},
  {6, "O2 Heater"},
  {7, "EGR System"},
};
const int MONITOR_DEF_COUNT = 8;

// ─── TIMING ─────────────────────────────────────────────────────────────────
Screen currentScreen = SCREEN_SPLASH;
unsigned long splashStart  = 0;
unsigned long scanStart    = 0;
unsigned long clearStart   = 0;
unsigned long clearedStart = 0;

// ─── REDRAW FLAG ─────────────────────────────────────────────────────────────
// Only redraw when input or screen change occurs.
bool needsRedraw = true;

void pollButton() {
  if (digitalRead(SW) == LOW) {
    unsigned long now = millis();
    if (now - lastButtonTime > 250) {
      buttonPressed = true;
      lastButtonTime = now;
      needsRedraw = true;
    }
  }
}

// ─── SETUP ──────────────────────────────────────────────────────────────────
void setup() {
  u8g2.begin();
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT,  INPUT_PULLUP);
  pinMode(SW,  INPUT_PULLUP);

  // CLK and DT are driven by interrupts now, not polling — this is what
  // lets the decoder catch every transition even while a blocking I2C
  // redraw (u8g2.sendBuffer()) is running in the main loop.
  attachInterrupt(digitalPinToInterrupt(CLK), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(DT),  encoderISR, CHANGE);

  splashStart = millis();
  Serial1.begin(115200);
}

// ─── SCROLL HELPER ──────────────────────────────────────────────────────────
void applyScroll(int delta, int& index, int& scroll, int count) {
  index += delta;
  if (index < 0) index = 0;
  if (index >= count) index = count - 1;
  if (index < scroll) scroll = index;
  if (index >= scroll + VISIBLE_ROWS) scroll = index - VISIBLE_ROWS + 1;
}

// ─── DRAW FUNCTIONS ─────────────────────────────────────────────────────────

void drawSplash() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso16_tf);
  int w = u8g2.getStrWidth("FLEAG7-SCAN");
  u8g2.drawStr((128 - w) / 2, 32, "FLEAG7-SCAN");
  u8g2.setFont(u8g2_font_6x10_tf);
  w = u8g2.getStrWidth("v1.1");
  u8g2.drawStr((128 - w) / 2, 50, "v1.1");
  u8g2.sendBuffer();
}

void drawMainMenu() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "FLEAG7-SCAN");
  u8g2.drawHLine(0, 13, 128);
  for (int i = 0; i < VISIBLE_ROWS; i++) {
    int itemIndex = menuScroll + i;
    if (itemIndex >= MENU_COUNT) break;
    int y = 26 + (i * 12);
    if (itemIndex == menuIndex) {
      u8g2.drawBox(0, y - 9, 128, 11);
      u8g2.setDrawColor(0);
      u8g2.drawStr(4, y, menuItems[itemIndex]);
      u8g2.setDrawColor(1);
    } else {
      u8g2.drawStr(4, y, menuItems[itemIndex]);
    }
  }
  if (MENU_COUNT > VISIBLE_ROWS) {
    if (menuScroll > 0) u8g2.drawStr(120, 22, "^");
    if (menuScroll + VISIBLE_ROWS < MENU_COUNT) u8g2.drawStr(120, 62, "v");
  }
  u8g2.sendBuffer();
}

void drawDTCScanning() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "DTC SCAN");
  u8g2.drawHLine(0, 13, 128);
  int w = u8g2.getStrWidth("Scanning...");
  u8g2.drawStr((128 - w) / 2, 38, "Scanning...");
  u8g2.sendBuffer();
}

void drawNoDTCs() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "DTC SCAN");
  u8g2.drawHLine(0, 13, 128);
  int w = u8g2.getStrWidth("No DTCs Found");
  u8g2.drawStr((128 - w) / 2, 35, "No DTCs Found");
  u8g2.setFont(u8g2_font_5x7_tf);
  w = u8g2.getStrWidth("Press to return");
  u8g2.drawStr((128 - w) / 2, 52, "Press to return");
  u8g2.sendBuffer();
}

void drawDTCList() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  char header[20];
  snprintf(header, sizeof(header), "DTCs (%d found)", DTC_COUNT);
  u8g2.drawStr(0, 10, header);
  u8g2.drawHLine(0, 13, 128);
  for (int i = 0; i < VISIBLE_ROWS; i++) {
    int itemIndex = dtcScroll + i;
    if (itemIndex >= DTC_COUNT) break;
    int y = 26 + (i * 12);
    if (itemIndex == dtcIndex) {
      u8g2.drawBox(0, y - 9, 128, 11);
      u8g2.setDrawColor(0);
      u8g2.drawStr(4, y, dtcList[itemIndex].code);
      u8g2.setDrawColor(1);
    } else {
      u8g2.drawStr(4, y, dtcList[itemIndex].code);
    }
  }
  if (DTC_COUNT > VISIBLE_ROWS) {
    if (dtcScroll > 0) u8g2.drawStr(120, 22, "^");
    if (dtcScroll + VISIBLE_ROWS < DTC_COUNT) u8g2.drawStr(120, 62, "v");
  }
  u8g2.sendBuffer();
}

void drawClearingDTCs() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "CLEAR DTCs");
  u8g2.drawHLine(0, 13, 128);
  int w = u8g2.getStrWidth("Clearing...");
  u8g2.drawStr((128 - w) / 2, 38, "Clearing...");
  u8g2.sendBuffer();
}

void drawDTCsCleared(bool success) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  const char* msg = success ? "Cleared!" : "No Response";
  int w = u8g2.getStrWidth(msg);
  u8g2.drawStr((128 - w) / 2, 38, msg);
  u8g2.sendBuffer();
}

void drawVehicleData() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "VEHICLE DATA");
  u8g2.drawHLine(0, 13, 128);
  for (int i = 0; i < VISIBLE_ROWS; i++) {
    int itemIndex = vdataScroll + i;
    if (itemIndex >= VDATA_COUNT) break;
    int y = 26 + (i * 12);
    char line[22];
    snprintf(line, sizeof(line), "%-10s%4s%-4s",
             vehicleData[itemIndex].label,
             vehicleData[itemIndex].value,
             vehicleData[itemIndex].unit);
    u8g2.drawStr(2, y, line);
  }
  if (VDATA_COUNT > VISIBLE_ROWS) {
    if (vdataScroll > 0) u8g2.drawStr(120, 22, "^");
    if (vdataScroll + VISIBLE_ROWS < VDATA_COUNT) u8g2.drawStr(120, 62, "v");
  }
  u8g2.sendBuffer();
}

void drawMonitors() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "MONITORS");
  u8g2.drawHLine(0, 13, 128);
  for (int i = 0; i < VISIBLE_ROWS; i++) {
    int itemIndex = monitorScroll + i;
    if (itemIndex >= MONITOR_COUNT) break;
    int y = 26 + (i * 12);
    u8g2.drawStr(2, y, monitors[itemIndex].name);
    u8g2.drawStr(84, y, monitors[itemIndex].ready ? "READY" : "NOT RDY");
  }
  if (MONITOR_COUNT > VISIBLE_ROWS) {
    if (monitorScroll > 0) u8g2.drawStr(120, 22, "^");
    if (monitorScroll + VISIBLE_ROWS < MONITOR_COUNT) u8g2.drawStr(120, 62, "v");
  }
  u8g2.sendBuffer();
}

// ─── DRAW: CAN BUS TEST ──────────────────────────────────────────────────────
void drawCANTest(const char* status, const char* detail) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 10, "CAN BUS TEST");
  u8g2.drawHLine(0, 13, 128);
  u8g2.drawStr(0, 28, "Status:");
  u8g2.drawStr(50, 28, status);
  u8g2.drawStr(0, 42, detail);
  u8g2.setFont(u8g2_font_5x7_tf);
  u8g2.drawStr(0, 63, "Press to return");
  u8g2.sendBuffer();
}

// ─── SCREEN DRAW DISPATCHER ──────────────────────────────────────────────────
void drawCurrentScreen() {
  switch (currentScreen) {
    case SCREEN_SPLASH:         drawSplash();       break;
    case SCREEN_MAIN_MENU:      drawMainMenu();     break;
    case SCREEN_DTC_SCANNING:   drawDTCScanning();  break;
    case SCREEN_NO_DTCS:        drawNoDTCs();       break;
    case SCREEN_DTC_LIST:       drawDTCList();      break;
    case SCREEN_CLEARING_DTCS:  drawClearingDTCs(); break;
    case SCREEN_DTCS_CLEARED:   drawDTCsCleared(lastClearSuccess); break;
    case SCREEN_VEHICLE_DATA:   drawVehicleData();  break;
    case SCREEN_MONITORS:       drawMonitors();     break;
    case SCREEN_CAN_TEST: drawCANTest(canTestStatus, canTestDetail); break;
  }
}

// ─── CAN TEST VARIANTS ──────────────────────────────────────────────────────
struct TestVariant {
  const char* label;
  uint32_t id;
  uint8_t pad;
};
TestVariant testVariants[] = {
  {"7DF PAD00", 0x7DF, 0x00},
  {"7DF PAD55", 0x7DF, 0x55},
  {"7E0 PAD00", 0x7E0, 0x00},
  {"7DF PADFF", 0x7DF, 0xFF},
};
const int VARIANT_COUNT = 4;
int variantIndex = 0;
static char variantDetail[22];

void updateVariantDetail() {
  snprintf(variantDetail, sizeof(variantDetail), "Turn:%s", testVariants[variantIndex].label);
  canTestDetail = variantDetail;
}

// ─── OBD2 TRANSACTION CORE ──────────────────────────────────────────────────
void canInitOBD() {
  Can.setAutoBusOffRecovery(true); // before begin — listed as pre-begin setting
  Can.begin(true);
  Can.setMode(STM32_CAN::NORMAL);
  Can.setBaudRate(500000);
  Can.setAutoRetransmission(true);

  // Wait for peripheral to sync to bus before transmitting
  delay(500);

  // Only pass 0x7E8-0x7EF (physical ECU response range) through in hardware
  Can.setFilterSingleMask(0, 0x7E8, 0x7F8, AUTO);

  // Flush any stale backlog that built up before the filter was applied
  CAN_message_t flushMsg;
  while (Can.read(flushMsg)) {
    // discard — just emptying the buffer
  }
}

void canShutdownOBD() {
  Can.end();
}

bool sendOBDRequest(uint32_t reqId, uint8_t mode, const uint8_t* data, uint8_t dataLen,
                     CAN_message_t &response, unsigned long timeoutMs = 1000) {
  CAN_message_t txMsg;
  txMsg.id  = reqId;
  txMsg.len = 8;

  txMsg.buf[0] = dataLen + 1; // PCI byte: number of bytes following (mode + data)
  txMsg.buf[1] = mode;
  for (uint8_t i = 0; i < dataLen; i++) {
    txMsg.buf[2 + i] = data[i];
  }
  for (uint8_t i = 2 + dataLen; i < 8; i++) {
    txMsg.buf[i] = 0x00;
  }

  if (!Can.write(txMsg)) {
    return false;
  }

  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (Can.read(response)) {
      if (response.id >= 0x7E0 && response.id <= 0x7EF) {
        return true;
      }
    }
  }
  return false;
}

//CAN BUS TEST
void runCANTest() {
  Serial1.println("=== CAN TEST START ===");
  canTestStatus = "STARTING";
  canTestDetail = "Init CAN...";
  needsRedraw = true;
  drawCurrentScreen();
  needsRedraw = false;

  Can.setAutoBusOffRecovery(true);
  Can.begin(true);
  Can.setAutoBusOffRecovery(true);
  Can.setMode(STM32_CAN::NORMAL);
  Can.setBaudRate(500000);
  Can.setAutoRetransmission(true);

  delay(500);

  Can.setFilterSingleMask(0, 0x7E8, 0x7F8, AUTO);

  CAN_message_t flushMsg;
  while (Can.read(flushMsg)) {
    // discard — just emptying the buffer
  }

  CAN_message_t txMsg;
  txMsg.id     = testVariants[variantIndex].id;
  txMsg.len    = 8;
  txMsg.buf[0] = 0x01;
  txMsg.buf[1] = 0x03;
  txMsg.buf[2] = 0xFF;
  uint8_t pad  = testVariants[variantIndex].pad;
  txMsg.buf[3] = pad;
  txMsg.buf[4] = pad;
  txMsg.buf[5] = pad;
  txMsg.buf[6] = pad;
  txMsg.buf[7] = pad;

  bool sent = Can.write(txMsg);

  if (!sent) {
    canTestStatus = "TX FAIL";
    canTestDetail = "Write rejected";
    needsRedraw = true;
    drawCurrentScreen();
    needsRedraw = false;
    Can.end();
    return;
  }

  canTestStatus = "SENT";
  canTestDetail = "Listening...";
  needsRedraw = true;
  drawCurrentScreen();
  needsRedraw = false;

  CAN_message_t rxMsg;
  unsigned long start = millis();
  bool received = false;

  while (millis() - start < 1000) {
    if (Can.read(rxMsg)) {
      unsigned long t = millis() - start;

      if (t <= 200) {
        Serial1.print("t+"); Serial1.print(t); Serial1.print("ms ID:0x");
        Serial1.print(rxMsg.id, HEX);
        Serial1.print(" DLC:"); Serial1.print(rxMsg.len);
        Serial1.print(" DATA:");
        for (int i = 0; i < rxMsg.len; i++) {
          if (rxMsg.buf[i] < 0x10) Serial1.print("0");
          Serial1.print(rxMsg.buf[i], HEX);
          Serial1.print(" ");
        }
        Serial1.println();
      }

      if (rxMsg.id >= 0x7E0 && rxMsg.id <= 0x7EF) {
        static char detail[22];
        snprintf(detail, sizeof(detail), "ID:%03X %02X %02X %02X %02X",
                 rxMsg.id, rxMsg.buf[0], rxMsg.buf[1],
                 rxMsg.buf[2], rxMsg.buf[3]);
        canTestStatus = "RESPONSE";
        canTestDetail = detail;
        received = true;
        break;
      }
      if (rxMsg.flags.extended && rxMsg.id >= 0x18DAF100 && rxMsg.id <= 0x18DAF1FF) {
        static char detail[22];
        snprintf(detail, sizeof(detail), "X:%07X %02X%02X%02X",
                 rxMsg.id, rxMsg.buf[0], rxMsg.buf[1], rxMsg.buf[2]);
        canTestStatus = "EXT RESP";
        canTestDetail = detail;
        received = true;
        break;
      }
    }
  }

  Can.end();

  if (!received) {
    uint32_t tsr = CAN1->TSR;
    bool txOk = tsr & (CAN_TSR_TXOK0 | CAN_TSR_TXOK1 | CAN_TSR_TXOK2);
    bool alst = tsr & (CAN_TSR_ALST0 | CAN_TSR_ALST1 | CAN_TSR_ALST2);
    bool terr = tsr & (CAN_TSR_TERR0 | CAN_TSR_TERR1 | CAN_TSR_TERR2);

    static char detail[22];
    snprintf(detail, sizeof(detail), "TX:%d ALST:%d TERR:%d", txOk, alst, terr);
    canTestStatus = "NO RESP";
    canTestDetail = detail;
    needsRedraw = true;
    drawCurrentScreen();
    needsRedraw = false;
  } else {
    needsRedraw = true;
    drawCurrentScreen();
    needsRedraw = false;
  }
}

// Decodes 2 raw DTC bytes into a 5-character code string like "P0420".
void decodeDTC(uint8_t byteA, uint8_t byteB, char* out) {
  const char letters[4]  = {'P', 'C', 'B', 'U'};
  const char hexChars[]  = "0123456789ABCDEF";

  uint8_t type   = (byteA >> 6) & 0x03;
  uint8_t digit1 = (byteA >> 4) & 0x03;
  uint8_t digit2 = byteA & 0x0F;
  uint8_t digit3 = (byteB >> 4) & 0x0F;
  uint8_t digit4 = byteB & 0x0F;

  out[0] = letters[type];
  out[1] = '0' + digit1;
  out[2] = hexChars[digit2];
  out[3] = hexChars[digit3];
  out[4] = hexChars[digit4];
  out[5] = '\0';
}

#define ISOTP_MAX_LEN 64   // raw byte ceiling for one reassembled response — real PCMs won't approach this

bool receiveMultiFrameResponse(uint8_t* outBuf, uint16_t &outLen, unsigned long timeoutMs = 1000) {
  CAN_message_t msg;
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    if (!Can.read(msg)) continue;
    if (msg.id < 0x7E0 || msg.id > 0x7EF) continue;

    uint8_t pciType = (msg.buf[0] >> 4) & 0x0F;

    if (pciType == 0x0) {
      uint8_t len = msg.buf[0] & 0x0F;
      if (len > 7) len = 7;
      for (uint8_t i = 0; i < len; i++) outBuf[i] = msg.buf[1 + i];
      outLen = len;
      return true;
    }

    if (pciType == 0x1) {
      uint16_t totalLen = ((msg.buf[0] & 0x0F) << 8) | msg.buf[1];
      if (totalLen > ISOTP_MAX_LEN) totalLen = ISOTP_MAX_LEN;

      uint16_t received = 0;
      for (uint8_t i = 0; i < 6 && received < totalLen; i++) {
        outBuf[received++] = msg.buf[2 + i];
      }

      CAN_message_t fc;
      fc.id  = msg.id - 8;
      fc.len = 8;
      fc.buf[0] = 0x30;
      fc.buf[1] = 0x00;
      fc.buf[2] = 0x00;
      for (int i = 3; i < 8; i++) fc.buf[i] = 0x00;
      Can.write(fc);

      unsigned long cfStart = millis();
      while (received < totalLen && (millis() - cfStart < timeoutMs)) {
        if (!Can.read(msg)) continue;
        if (msg.id != (fc.id + 8)) continue;
        if (((msg.buf[0] >> 4) & 0x0F) != 0x2) continue;

        for (uint8_t i = 0; i < 7 && received < totalLen; i++) {
          outBuf[received++] = msg.buf[1 + i];
        }
        cfStart = millis();
      }

      outLen = received;
      return received > 0;
    }
  }
  return false;
}

void performDTCScan() {
  canInitOBD();
  DTC_COUNT = 0;

  CAN_message_t txMsg;
  txMsg.id  = 0x7DF;
  txMsg.len = 8;
  txMsg.buf[0] = 0x01;
  txMsg.buf[1] = 0x03;
  for (int i = 2; i < 8; i++) txMsg.buf[i] = 0x00;

  if (Can.write(txMsg)) {
    uint8_t rawData[ISOTP_MAX_LEN];
    uint16_t rawLen = 0;

    if (receiveMultiFrameResponse(rawData, rawLen, 1000)) {
      if (rawLen >= 1 && rawData[0] == 0x43) {
        uint16_t dtcByteCount = rawLen - 1;
        int numDTCs = dtcByteCount / 2;
        if (numDTCs > MAX_DTC_COUNT) numDTCs = MAX_DTC_COUNT;

        for (int i = 0; i < numDTCs; i++) {
          uint8_t byteA = rawData[1 + (i * 2)];
          uint8_t byteB = rawData[2 + (i * 2)];
          if (byteA == 0x00 && byteB == 0x00) continue;
          decodeDTC(byteA, byteB, dtcList[DTC_COUNT].code);
          DTC_COUNT++;
        }
      }
    }
  }

  canShutdownOBD();
}

bool performClearDTCs() {
  canInitOBD();

  CAN_message_t resp;
  bool success = false;

  if (sendOBDRequest(0x7DF, 0x04, nullptr, 0, resp)) {
    if (resp.buf[1] == 0x44) {
      success = true;
    }
  }

  canShutdownOBD();
  return success;
}

void discoverPIDBlock(uint8_t basePID) {
  CAN_message_t resp;
  uint8_t reqData[1] = { basePID };

  if (sendOBDRequest(0x7DF, 0x01, reqData, 1, resp)) {
    if (resp.buf[1] == 0x41 && resp.buf[2] == basePID) {
      uint32_t bitmask = ((uint32_t)resp.buf[3] << 24) | ((uint32_t)resp.buf[4] << 16)
                        | ((uint32_t)resp.buf[5] << 8)  | resp.buf[6];

      for (int bit = 0; bit < 32; bit++) {
        if (bitmask & (1UL << (31 - bit))) {
          uint8_t pid = basePID + bit + 1;
          if (findPIDInfo(pid) != nullptr && discoveredPIDCount < MAX_VDATA_COUNT) {
            discoveredPIDs[discoveredPIDCount++] = pid;
          }
        }
      }
    }
  }
}

void discoverSupportedPIDs() {
  canInitOBD();
  discoveredPIDCount = 0;

  discoverPIDBlock(0x00);
  discoverPIDBlock(0x20);
  discoverPIDBlock(0x40);

  canShutdownOBD();
}

void discoverSupportedPIDsNoInit() {
  discoveredPIDCount = 0;
  discoverPIDBlock(0x00);
  discoverPIDBlock(0x20);
  discoverPIDBlock(0x40);
}

void buildVehicleDataFromDiscovery() {
  VDATA_COUNT = 0;
  for (int i = 0; i < discoveredPIDCount && i < MAX_VDATA_COUNT; i++) {
    const PIDInfo* info = findPIDInfo(discoveredPIDs[i]);
    if (info == nullptr) continue;
    vehicleData[VDATA_COUNT].label = info->label;
    vehicleData[VDATA_COUNT].unit  = info->unit;
    strcpy(vehicleData[VDATA_COUNT].value, "---");
    VDATA_COUNT++;
  }
}

unsigned long lastVDataPoll = 0;
const unsigned long VDATA_POLL_INTERVAL = 30;
int vdataPollCursor = 0;

// Formats a float to one decimal place without relying on printf's %f
// support, which Newlib Nano (STM32's default C runtime) doesn't include.
void formatFloat(float value, char* out, size_t outSize) {
  bool negative = value < 0;
  if (negative) value = -value;

  int whole = (int)value;
  int tenths = (int)((value - whole) * 10.0f + 0.5f);
  if (tenths >= 10) {
    tenths = 0;
    whole++;
  }

  snprintf(out, outSize, "%s%d.%d", negative ? "-" : "", whole, tenths);
}

void pollVehicleDataTick() {
  if (VDATA_COUNT == 0) return;
  if (millis() - lastVDataPoll < VDATA_POLL_INTERVAL) return;
  lastVDataPoll = millis();

  int visibleCount = min(VISIBLE_ROWS, VDATA_COUNT - vdataScroll);
  if (visibleCount <= 0) return;

  int itemIndex = vdataScroll + (vdataPollCursor % visibleCount);
  vdataPollCursor++;

  uint8_t pid = discoveredPIDs[itemIndex];
  const PIDInfo* info = findPIDInfo(pid);
  if (info == nullptr) return;

  CAN_message_t resp;
  uint8_t reqData[1] = { pid };

  if (sendOBDRequest(0x7DF, 0x01, reqData, 1, resp, 100)) {
    if (resp.buf[1] == 0x41 && resp.buf[2] == pid) {
      float value = decodePIDValue(*info, resp.buf[3], resp.buf[4]);
      formatFloat(value, vehicleData[itemIndex].value, sizeof(vehicleData[itemIndex].value));
      needsRedraw = true;
    }
  }
}

bool milOn = false;
int dtcCountFromMonitor = 0;

void performMonitorScan() {
  canInitOBD();
  MONITOR_COUNT = 0;

  CAN_message_t resp;
  uint8_t reqData[1] = { 0x01 };

  if (sendOBDRequest(0x7DF, 0x01, reqData, 1, resp)) {
    if (resp.buf[1] == 0x41 && resp.buf[2] == 0x01) {
      uint8_t byteA = resp.buf[3];
      uint8_t byteB = resp.buf[4];
      uint8_t byteC = resp.buf[5];
      uint8_t byteD = resp.buf[6];

      milOn = (byteA & 0x80) != 0;
      dtcCountFromMonitor = byteA & 0x7F;

      for (int i = 0; i < CONTINUOUS_MONITOR_COUNT && MONITOR_COUNT < MONITOR_COUNT_MAX; i++) {
        bool supported = (byteB & (1 << continuousMonitors[i].bitPos)) != 0;
        if (!supported) continue;
        bool notComplete = (byteB & (1 << (continuousMonitors[i].bitPos + 4))) != 0;
        monitors[MONITOR_COUNT].name  = continuousMonitors[i].name;
        monitors[MONITOR_COUNT].ready = !notComplete;
        MONITOR_COUNT++;
      }

      for (int i = 0; i < MONITOR_DEF_COUNT && MONITOR_COUNT < MONITOR_COUNT_MAX; i++) {
        bool supported = (byteC & (1 << monitorDefs[i].bitPos)) != 0;
        if (!supported) continue;
        bool notComplete = (byteD & (1 << monitorDefs[i].bitPos)) != 0;
        monitors[MONITOR_COUNT].name  = monitorDefs[i].name;
        monitors[MONITOR_COUNT].ready = !notComplete;
        MONITOR_COUNT++;
      }
    }
  }

  canShutdownOBD();
}

// ─── MAIN LOOP ───────────────────────────────────────────────────────────────
void loop() {
  // Encoder is ISR-driven now — just atomically grab and clear whatever
  // encoderISR() has accumulated since the last time we checked.
  noInterrupts();
  int delta = encoderDelta;
  encoderDelta = 0;
  needsRedraw = true;
  interrupts();

  pollButton();
  bool pressed = buttonPressed;
  buttonPressed = false;

  switch (currentScreen) {

    case SCREEN_SPLASH:
      if (millis() - splashStart > 5000) {
        currentScreen = SCREEN_MAIN_MENU;
        needsRedraw = true;
      }
      break;

    case SCREEN_MAIN_MENU:
      if (delta != 0) applyScroll(delta, menuIndex, menuScroll, MENU_COUNT);
      if (pressed) {
        dtcScroll = 0; dtcIndex = 0;
        vdataScroll = 0;
        monitorScroll = 0;
        switch (menuIndex) {
          case 0:
            currentScreen = SCREEN_DTC_SCANNING;
            drawCurrentScreen();
            performDTCScan();
            currentScreen = (DTC_COUNT > 0) ? SCREEN_DTC_LIST : SCREEN_NO_DTCS;
            break;
          case 1:
            currentScreen = SCREEN_CLEARING_DTCS;
            drawCurrentScreen();
            lastClearSuccess = performClearDTCs();
            clearedStart = millis();
            currentScreen = SCREEN_DTCS_CLEARED;
            break;
          case 2:
            currentScreen = SCREEN_VEHICLE_DATA;
            drawCurrentScreen();
            canInitOBD();
            discoverSupportedPIDsNoInit();
            buildVehicleDataFromDiscovery();
            vdataPollCursor = 0;
            needsRedraw = true;
            break;
          case 3:
            currentScreen = SCREEN_MONITORS;
            drawCurrentScreen();
            performMonitorScan();
            needsRedraw = true;
            break;
          case 4:
            canTestStatus = "WAITING";
            updateVariantDetail();
            currentScreen = SCREEN_CAN_TEST;
            needsRedraw = true;
            break;
        }
        needsRedraw = true;
      }
      break;

    case SCREEN_DTC_SCANNING:
      if (millis() - scanStart > 1500) {
        currentScreen = (DTC_COUNT > 0) ? SCREEN_DTC_LIST : SCREEN_NO_DTCS;
        needsRedraw = true;
      }
      break;

    case SCREEN_NO_DTCS:
      if (pressed) {
        currentScreen = SCREEN_MAIN_MENU;
        needsRedraw = true;
      }
      break;

    case SCREEN_DTC_LIST:
      if (delta != 0) applyScroll(delta, dtcIndex, dtcScroll, DTC_COUNT);
      if (pressed) {
        currentScreen = SCREEN_MAIN_MENU;
        needsRedraw = true;
      }
      break;

    case SCREEN_CLEARING_DTCS:
      if (millis() - clearStart > 1500) {
        clearedStart = millis();
        currentScreen = SCREEN_DTCS_CLEARED;
        needsRedraw = true;
      }
      break;

    case SCREEN_DTCS_CLEARED:
      if (millis() - clearedStart > 1500) {
        currentScreen = SCREEN_MAIN_MENU;
        needsRedraw = true;
      }
      break;

    case SCREEN_VEHICLE_DATA:
      if (delta != 0) {
        vdataScroll += delta;
        if (vdataScroll < 0) vdataScroll = 0;
        if (vdataScroll > VDATA_COUNT - VISIBLE_ROWS) vdataScroll = VDATA_COUNT - VISIBLE_ROWS;
      }
      if (pressed) {
        canShutdownOBD();
        currentScreen = SCREEN_MAIN_MENU;
        needsRedraw = true;
      }
      break;

    case SCREEN_MONITORS:
      if (delta != 0) {
        monitorScroll += delta;
        if (monitorScroll < 0) monitorScroll = 0;
        if (MONITOR_COUNT > VISIBLE_ROWS) {
          if (monitorScroll > MONITOR_COUNT - VISIBLE_ROWS) monitorScroll = MONITOR_COUNT - VISIBLE_ROWS;
        } else {
          monitorScroll = 0;
        }
      }
      if (pressed) {
        currentScreen = SCREEN_MAIN_MENU;
        needsRedraw = true;
      }
      break;

    case SCREEN_CAN_TEST:
      if (delta != 0) {
        variantIndex += delta;
        if (variantIndex < 0) variantIndex = VARIANT_COUNT - 1;
        if (variantIndex >= VARIANT_COUNT) variantIndex = 0;
        canTestStatus = "WAITING";
        updateVariantDetail();
        needsRedraw = true;
      }
      if (pressed) {
        runCANTest();
      }
      break;
  }

  if (currentScreen == SCREEN_VEHICLE_DATA) {
    pollVehicleDataTick();
  }

  if (needsRedraw) {
    drawCurrentScreen();
    needsRedraw = false;
  }
}
