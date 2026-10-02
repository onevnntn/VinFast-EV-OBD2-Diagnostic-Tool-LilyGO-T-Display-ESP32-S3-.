// =====================================================================
// EVO200 CAN Diagnostic — PORTED to Waveshare ESP32-S3-Touch-LCD-7B
// (RGB 1024x600 display, GT911 capacitive touch, CAN via IO-Expander)
//
// - CAN 500 kbps (TWAI native)
// - Multi-page touch UI (Overview, Cell voltages, DTC, Log)
// - UDS: read/clear DTC, read VIN
// - Log to Serial (USB CDC) and to TF/SD card (text log + CSV telemetry)
// - No WiFi
//
// FIXES applied from CAN log analysis (LOGCAN_1_.TXT / LOGCAN.TXT) — kept as-is:
//  1. UDS response d[0] is ECU node byte — ISO-TP PCI starts at d[1]
//  2. Added Single Frame (0x0) handler in processDiagnosticResponse
//  3. Added sendFlowControl() after First Frame received
//  4. SOC moved to 0x322 d[0]/d[1] (average of 2 modules); removed from 0x309
//  5. Pack voltage = 0x309 d[2:3]/1000.0 * 2 (two 11S modules in series)
//  6. Cell array [22], starts from 0x311 (0x310 is NOT cell data)
//  7. 0x315-0x318 are unused — skip zero-value cells
//  8. 0x303 d[1]=0x30 also signals charging mode
//  9. sendDiagnosticRequest data[0] corrected to 0x03 (3-byte payload)
// 10. isBMSalive resets after 3s timeout
// 11. Touch debounce uses timestamp instead of delay()
// 12. UDS buffer off-by-one in multi-frame parsing fixed
//
// PORT CHANGES for 7B board (this file):
//  P1. TFT_eSPI (ST7789) -> Arduino_GFX_Library RGB panel driver, 1024x600
//  P2. FT6236 touch -> TAMC_GT911 (shared I2C bus with IO Expander, addr 0x24)
//  P3. CAN pins GPIO17/18 -> GPIO20 (TX) / GPIO19 (RX) per 7B schematic
//  P4. Added init_io_expander(): must run before gfx->begin() / touch.begin()
//      to release touch reset (EXIO1), enable backlight (EXIO2),
//      select CAN mode (EXIO5) and enable LCD VCOM (EXIO6)
//  P5. Touch zones rescaled from ~240x320 coordinate space to 1024x600
//  P6. Removed X/Y swap hack that was specific to T-Display S3 rotation
//
//
// NO-DISPLAY FIX (v2) — root causes found:
//  F1. The 7B "IO expander" is a CH32V003 MCU with its own register map:
//      mode=0x02 (must be 0xFF), output=0x03, PWM=0x05. The old code wrote to
//      register 0x01 -> every write ignored -> no backlight, LCD in reset.
//  F2. LCD_RST (EXIO3) was left low; now released. Touch reset uses proper
//      GT911 address-select sequence (INT low while RST rises -> 0x5D).
//  F3. RGB timings replaced by the values verified on real 7B boards
//      (pclk 30 MHz, hsync 48/162/152, vsync 3/45/13).
//  F4. Backlight enabled only after the panel is initialised (backlight_on()).
//  F5. Drawing is now in-place (no full-screen clear every 200 ms -> no flicker).
//  Arduino IDE -> Tools: Board ESP32S3 Dev Module, Flash 16MB, PSRAM "OPI PSRAM",
//  USB CDC On Boot: Disabled (use the UART Type-C port; native USB is switched to CAN).
//
// SD CARD LOGGING (added):
//  S1. TF card slot per 7B wiki: MOSI=GPIO11, SCK=GPIO12, MISO=GPIO13.
//      SD_CS is NOT a normal GPIO — it is EXIO4 on the IO Expander, active-low.
//      We leave EXIO4 unset (LOW) in init_io_expander() so the card stays
//      permanently selected, and pass an unused GPIO (6) to SD.begin() purely
//      as a placeholder CS argument the Arduino SD library requires.
//  S2. Every addLog() line is mirrored to a text file on SD (LOG_xxxxx.TXT).
//  S3. Telemetry (SOC, pack V, ignition, DTC, VIN, 22 cell voltages) is
//      appended once per second as CSV (DATA_xxxxx.CSV) for easy analysis.
//  S4. Serial command 's' toggles logging on/off at runtime.
//  S5. Session number persisted in /NEXTLOG.TXT so each boot gets new files.
// =====================================================================

#include <Arduino.h>
#include <stdarg.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <driver/twai.h>
#include <Arduino_GFX_Library.h>
#include <TAMC_GT911.h>

// ==================== IO Expander / I2C ====================
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define IO_EXPANDER_ADDR 0x24

// The "IO expander" on the 7B is a CH32V003 MCU running Waveshare firmware
// (NOT a CH422G / PCA9554). Register map from Waveshare's io_extension driver:
#define IOEXP_REG_MODE    0x02   // pin direction, bit=1 -> output (MUST be set to 0xFF first!)
#define IOEXP_REG_OUTPUT  0x03   // output levels
#define IOEXP_REG_INPUT   0x04   // input levels
#define IOEXP_REG_PWM     0x05   // backlight PWM, INVERTED: low = bright, high = dim (safe max 247)
#define IOEXP_REG_ADC     0x06   // battery voltage ADC

// EXIO pin bits (bit index == EXIO number)
#define IOEXP_BIT_TP_RST     1   // touch reset, HIGH = running
#define IOEXP_BIT_DISP_BL    2   // backlight enable, HIGH = on
#define IOEXP_BIT_LCD_RST    3   // LCD reset, HIGH = running
#define IOEXP_BIT_SD_CS      4   // TF card CS, LOW = card selected
#define IOEXP_BIT_CAN_SEL    5   // HIGH = CAN mode, LOW = USB mode
#define IOEXP_BIT_LCD_VDD_EN 6   // VCOM enable, HIGH = on

#define BACKLIGHT_PWM_FULL   1   // 1 = brightest ... 247 = dimmest (0/255 avoided per Waveshare)
#define TOUCH_INT_GPIO       4   // GT911 INT (used to select I2C address 0x5D during reset)

// ==================== Pin definitions ====================
#define CAN_TX_PIN GPIO_NUM_20   // FIX P3: was GPIO_NUM_17 on T-Display S3
#define CAN_RX_PIN GPIO_NUM_19   // FIX P3: was GPIO_NUM_18 on T-Display S3

// ==================== SD / TF card (S1) ====================
#define SD_MOSI_PIN 11
#define SD_SCK_PIN  12
#define SD_MISO_PIN 13
// Real SD_CS is EXIO4 on the IO Expander (active-low), held LOW permanently
// in init_io_expander(). This GPIO is only a placeholder for the SD library's
// mandatory CS argument — it is not wired to the card's CS line. GPIO6 is not
// used anywhere else on the 7B board's official pinout.
#define SD_CS_DUMMY_PIN 6

// ==================== CAN IDs (unchanged from log analysis) ====================
#define ID_IGNITION         0x101
#define ID_VEHICLE_MODE     0x110
#define ID_BMS_STATUS       0x303
#define ID_PACK_VOLTAGE     0x309
#define ID_CELL_START       0x311   // was 0x310
#define ID_CELL_END         0x31B
#define ID_SOC              0x322
#define ID_CHARGER_STATUS   0x342
#define ID_DIAG_REQ         0x006
#define ID_DIAG_RESP        0x002

// ==================== Display (Arduino_GFX RGB panel, 7B pin map) ====================
// Pin map confirmed against official wiki: waveshare.com/wiki/ESP32-S3-Touch-LCD-7B
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
  5  /* DE */, 3  /* VSYNC */, 46 /* HSYNC */, 7  /* PCLK */,
  1  /* R0 -> R3 */, 2  /* R1 -> R4 */, 42 /* R2 -> R5 */, 41 /* R3 -> R6 */, 40 /* R4 -> R7 */,
  39 /* G0 -> G2 */, 0  /* G1 -> G3 */, 45 /* G2 -> G4 */, 48 /* G3 -> G5 */, 47 /* G4 -> G6 */, 21 /* G5 -> G7 */,
  14 /* B0 -> B3 */, 38 /* B1 -> B4 */, 18 /* B2 -> B5 */, 17 /* B3 -> B6 */, 10 /* B4 -> B7 */,
  // Timings verified on real 7B boards (ESPHome community): pclk 30MHz, inverted pclk
  0 /* hsync_polarity */, 48 /* hsync_front_porch */, 162 /* hsync_pulse_width */, 152 /* hsync_back_porch */,
  0 /* vsync_polarity */, 3 /* vsync_front_porch */, 45 /* vsync_pulse_width */, 13 /* vsync_back_porch */,
  1 /* pclk_active_neg */, 30000000 /* prefer_speed (lower to 16000000 if image drifts) */
);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(1024, 600, rgbpanel);

#define BLACK   0x0000
#define WHITE   0xFFFF
#define RED     0xF800
#define GREEN   0x07E0
#define YELLOW  0xFFE0

// ==================== Touch (GT911, replaces FT6236) ====================
#define TOUCH_SDA 8
#define TOUCH_SCL 9
#define TOUCH_INT -1  // GPIO4 (TP_IRQ) exists on board, polling mode used instead
#define TOUCH_RST -1  // reset handled via IO Expander (EXIO1), not a direct GPIO
#define TOUCH_WIDTH  1024
#define TOUCH_HEIGHT 600
TAMC_GT911 touch = TAMC_GT911(TOUCH_SDA, TOUCH_SCL, TOUCH_INT, TOUCH_RST, TOUCH_WIDTH, TOUCH_HEIGHT);

// ==================== Global variables ====================
float cellVoltages[22] = {0};
float packVoltage = 0.0;
uint8_t soc = 0;
bool isCharging = false;
bool ignitionOn = false;
bool isBMSalive = false;
uint32_t lastBMSFrame = 0;

String dtcList = "";
bool hasDTC = false;
String vin = "";

#define LOG_MAX 100
String logBuffer[LOG_MAX];
uint8_t logIndex = 0;
uint8_t logCount = 0;

uint8_t currentPage = 0;
const uint8_t PAGE_COUNT = 4;   // 0:Overview, 1:Cells, 2:DTC, 3:Log

bool requestDTC = false;
bool clearDTC = false;
bool requestVIN = false;

uint32_t lastDisplay = 0;
uint32_t lastTouchTime = 0;

// ==================== SD logging state (S2/S3/S4) ====================
bool   sdReady = false;
bool   sdLoggingEnabled = true;
File   sdLogFile;   // text mirror of addLog()
File   sdDataFile;  // CSV telemetry, 1 row/sec
String sdLogFileName;
String sdDataFileName;
uint32_t lastSDFlush = 0;
uint32_t lastSDData  = 0;

// ==================== Logging ====================
void addLog(const char *msg) {
    String line = String(millis()) + "ms: " + msg;
    logBuffer[logIndex] = line;
    logIndex = (logIndex + 1) % LOG_MAX;
    if (logCount < LOG_MAX) logCount++;
    Serial.println(line);

    if (sdReady && sdLoggingEnabled && sdLogFile) {
        sdLogFile.println(line);
        if (millis() - lastSDFlush > 2000) {
            sdLogFile.flush();
            lastSDFlush = millis();
        }
    }
}
void addLog(String msg) { addLog(msg.c_str()); }

// ==================== Flow Control ====================
void sendFlowControl() {
    twai_message_t fc = {};   // zero-init to clear extd/rtr/self flags
    fc.identifier = ID_DIAG_REQ;
    fc.data_length_code = 8;
    fc.data[0] = 0x30;
    fc.data[1] = 0x00;
    fc.data[2] = 0x00;
    for (int i = 3; i < 8; i++) fc.data[i] = 0xCC;
    twai_transmit(&fc, pdMS_TO_TICKS(10));
}

// ==================== IO Expander init (P4) ====================
static uint8_t ioexp_out = 0xFF;   // shadow of output register (official driver also starts at 0xFF)

static bool ioexp_write_reg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(IO_EXPANDER_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

static void ioexp_set(uint8_t bit, bool level) {
    if (level) ioexp_out |= (1 << bit);
    else       ioexp_out &= ~(1 << bit);
    ioexp_write_reg(IOEXP_REG_OUTPUT, ioexp_out);
}

// Runs BEFORE gfx->begin(). Backlight stays off until backlight_on().
void init_io_expander() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    // 1. All EXIO pins -> outputs. Without this every later write is ignored
    //    (this was the cause of the black screen in the previous version).
    if (!ioexp_write_reg(IOEXP_REG_MODE, 0xFF)) {
        Serial.println("ERROR: IO expander (0x24) not responding - check I2C");
    }

    // 2. Baseline: TP_RST=0, LCD_RST=0, backlight off, SD_CS=0 (card selected),
    //    CAN_SEL=1, LCD_VDD_EN=1 (VCOM on), other pins high.
    ioexp_out = 0xFF;
    ioexp_out &= ~((1 << IOEXP_BIT_TP_RST) | (1 << IOEXP_BIT_LCD_RST) |
                   (1 << IOEXP_BIT_DISP_BL) | (1 << IOEXP_BIT_SD_CS));
    ioexp_write_reg(IOEXP_REG_OUTPUT, ioexp_out);

    // 3. GT911 address select: INT must be LOW while RESET rises -> I2C addr 0x5D
    pinMode(TOUCH_INT_GPIO, OUTPUT);
    digitalWrite(TOUCH_INT_GPIO, LOW);
    delay(20);
    ioexp_set(IOEXP_BIT_LCD_RST, true);   // release LCD reset
    ioexp_set(IOEXP_BIT_TP_RST, true);    // release touch reset (rising edge)
    delay(10);
    pinMode(TOUCH_INT_GPIO, INPUT);       // let GT911 drive INT again
    delay(60);                            // GT911 needs >50ms before I2C access

    Serial.println("IO Expander: outputs set, LCD/touch reset released, VCOM on, CAN selected, SD_CS asserted");
}

// Call AFTER the RGB panel is initialised so no garbage is shown.
void backlight_on() {
    ioexp_write_reg(IOEXP_REG_PWM, BACKLIGHT_PWM_FULL);
    ioexp_set(IOEXP_BIT_DISP_BL, true);
}

// ==================== SD card init & telemetry logging (S1-S5) ====================
bool initSD() {
    // SD_CS (EXIO4) was left LOW by init_io_expander() -> card is already
    // permanently selected. We still init the SPI bus on the TF pins and
    // pass a dummy CS pin to satisfy the SD library's API.
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_DUMMY_PIN);

    if (!SD.begin(SD_CS_DUMMY_PIN, SPI, 20000000)) {
        Serial.println("SD init FAILED (no card or mount error)");
        return false;
    }
    if (SD.cardType() == CARD_NONE) {
        Serial.println("SD: no card detected");
        return false;
    }

    // Persisted session counter so every boot creates fresh log files
    uint32_t session = 1;
    if (SD.exists("/NEXTLOG.TXT")) {
        File f = SD.open("/NEXTLOG.TXT", FILE_READ);
        if (f) {
            session = f.parseInt();
            f.close();
        }
    }
    File fw = SD.open("/NEXTLOG.TXT", FILE_WRITE);
    if (fw) {
        fw.print(session + 1);
        fw.close();
    }

    char nameBuf[24];
    snprintf(nameBuf, sizeof(nameBuf), "/LOG_%05lu.TXT", (unsigned long)session);
    sdLogFileName = nameBuf;
    snprintf(nameBuf, sizeof(nameBuf), "/DATA_%05lu.CSV", (unsigned long)session);
    sdDataFileName = nameBuf;

    sdLogFile  = SD.open(sdLogFileName, FILE_WRITE);
    sdDataFile = SD.open(sdDataFileName, FILE_WRITE);
    if (!sdLogFile || !sdDataFile) {
        Serial.println("SD: could not create log files");
        return false;
    }

    sdDataFile.println(
        "timestamp_ms,ignition,soc_pct,pack_v,charging,bms_alive,has_dtc,dtc_list,vin,"
        "c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,c16,c17,c18,c19,c20,c21,c22");
    sdDataFile.flush();

    return true;
}

void logTelemetryToSD() {
    if (!sdReady || !sdLoggingEnabled || !sdDataFile) return;
    if (millis() - lastSDData < 1000) return;
    lastSDData = millis();

    sdDataFile.printf("%lu,%d,%u,%.2f,%d,%d,%d,\"%s\",\"%s\"",
        (unsigned long)millis(), ignitionOn ? 1 : 0, soc, packVoltage,
        isCharging ? 1 : 0, isBMSalive ? 1 : 0, hasDTC ? 1 : 0,
        dtcList.c_str(), vin.c_str());
    for (int i = 0; i < 22; i++) {
        sdDataFile.printf(",%.3f", cellVoltages[i]);
    }
    sdDataFile.println();
    sdDataFile.flush();
}

// ==================== Setup ====================
void setup() {
    Serial.begin(115200);
    delay(100);
    addLog("=== EVO200 CAN Diagnostic (7B) ===");

    // --- IO Expander (must run first: powers backlight/VCOM/CAN, releases touch reset) ---
    init_io_expander();

    // --- Display ---
    if (!gfx->begin()) {
        addLog("Display init FAILED");
    }
    gfx->fillScreen(BLACK);
    gfx->setTextColor(WHITE, BLACK);
    gfx->setTextSize(2);
    gfx->setCursor(0, 0);
    backlight_on();
    Serial.printf("PSRAM: %s (%u bytes)\n", psramFound() ? "OK" : "NOT FOUND - enable OPI PSRAM in Tools!", (unsigned)ESP.getPsramSize());
    addLog("Display init OK");

    // --- Touch ---
    touch.begin();
    touch.setRotation(ROTATION_NORMAL);
    addLog("Touch init OK (GT911)");

    // --- CAN (TWAI) ---
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL
    );
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err == ESP_OK) {
        twai_start();
        addLog("CAN 500k started");
    } else {
        addLog("CAN init FAILED");
    }

    // --- SD card (must be after Serial/addLog is usable) ---
    sdReady = initSD();
    if (sdReady) {
        addLog("SD ready: " + sdDataFileName);
    } else {
        addLog("SD not available - logging disabled");
    }

    delay(500);
    requestVIN = true;
    requestDTC = true;
    addLog("Initial requests sent (VIN, DTC)");
}

// ==================== Main Loop ====================
void loop() {
    twai_message_t msg;
    if (twai_receive(&msg, pdMS_TO_TICKS(5)) == ESP_OK) {
        processCANFrame(&msg);
    }

    if (isBMSalive && (millis() - lastBMSFrame > 3000)) {
        isBMSalive = false;
        addLog("BMS timeout");
    }

    if (requestDTC) { sendDiagnosticRequest(0x19, 0x02, 0x08); requestDTC = false; }
    if (clearDTC)   { sendDiagnosticRequest(0x14, 0xFF, 0xFF); clearDTC = false; addLog("DTC clear requested"); }
    if (requestVIN) { sendDiagnosticRequest(0x22, 0xF1, 0x90); requestVIN = false; addLog("VIN request sent"); }

    if (millis() - lastDisplay > 200) {
        lastDisplay = millis();
        drawPage(currentPage);
    }

    handleTouch();

    // 6. SD telemetry (1 row/sec CSV)
    logTelemetryToSD();

    if (Serial.available()) {
        char ch = Serial.read();
        if (ch == 'd') { requestDTC = true;  addLog("Manual DTC request"); }
        if (ch == 'c') { clearDTC = true;    addLog("Manual clear DTC"); }
        if (ch == 'v') { requestVIN = true;  addLog("Manual VIN request"); }
        if (ch == 'b') {   // cycle backlight PWM (low value = bright)
            static const uint8_t lv[] = {1, 60, 120, 180, 247};
            static uint8_t li = 0;
            li = (li + 1) % 5;
            ioexp_write_reg(IOEXP_REG_PWM, lv[li]);
            addLog("Backlight PWM=" + String(lv[li]) + " (1=brightest, 247=dimmest)");
        }
        if (ch == 's') {
            sdLoggingEnabled = !sdLoggingEnabled;
            addLog(sdLoggingEnabled ? "SD logging ON" : "SD logging OFF");
        }
    }
}

// ==================== CAN Frame Processing (logic unchanged) ====================
void processCANFrame(twai_message_t *msg) {
    uint32_t id  = msg->identifier;
    uint8_t  *d  = msg->data;
    uint8_t  dlc = msg->data_length_code;

    if (id == ID_IGNITION && dlc >= 1) {
        ignitionOn = (d[0] == 0x11);
    }

    if (id == ID_VEHICLE_MODE && dlc >= 1) {
        isCharging = (d[0] == 0x30);
    }

    if (id == ID_BMS_STATUS && dlc >= 2) {
        static uint8_t lastCounter = 0xFF;
        if (d[0] != lastCounter) {
            isBMSalive = true;
            lastBMSFrame = millis();
        }
        lastCounter = d[0];
        if (d[1] == 0x30) isCharging = true;
    }

    if (id == ID_PACK_VOLTAGE && dlc >= 4) {
        uint16_t raw = (d[2] << 8) | d[3];
        packVoltage = (raw / 1000.0f) * 2.0f;
    }

    if (id == ID_SOC && dlc >= 2) {
        soc = (d[0] + d[1]) / 2;
    }

    if (id >= ID_CELL_START && id <= ID_CELL_END && dlc == 8) {
        int cellIdx = (id - ID_CELL_START) * 4;
        for (int i = 0; i < 8; i += 2) {
            int ci = cellIdx + i / 2;
            if (ci < 22) {
                uint16_t raw = (d[i] << 8) | d[i + 1];
                if (raw > 0x1000) {
                    cellVoltages[ci] = raw / 10000.0f;
                }
            }
        }
    }

    if (id == ID_CHARGER_STATUS && dlc >= 8) {
        if (d[0] != 0xFF) {
            isCharging = true;
        }
    }

    if (id == ID_DIAG_RESP) {
        processDiagnosticResponse(msg);
    }
}

// ==================== UDS Response Handling (logic unchanged) ====================
static uint8_t  udsBuffer[1024];
static uint16_t udsIdx      = 0;
static uint16_t udsTotal    = 0;
static bool     udsReceiving = false;

void processDiagnosticResponse(twai_message_t *msg) {
    uint8_t *d   = msg->data;
    uint8_t  dlc = msg->data_length_code;

    if (dlc < 2) return;

    uint8_t pci  = d[1];
    uint8_t type = pci >> 4;

    switch (type) {
        case 0x0: {
            uint8_t sfLen = pci & 0x0F;
            if (sfLen > 0 && sfLen <= dlc - 2) {
                parseDiagnosticData(&d[2], sfLen);
            }
            break;
        }
        case 0x1: {
            udsTotal = ((pci & 0x0F) << 8) | d[2];
            udsIdx = 0;
            udsReceiving = true;
            for (int i = 3; i < dlc; i++) {
                if (udsIdx < sizeof(udsBuffer)) udsBuffer[udsIdx++] = d[i];
            }
            sendFlowControl();
            break;
        }
        case 0x2: {
            if (!udsReceiving) break;
            for (int i = 2; i < dlc; i++) {
                if (udsIdx < sizeof(udsBuffer) && udsIdx < udsTotal) {
                    udsBuffer[udsIdx++] = d[i];
                }
            }
            if (udsIdx >= udsTotal) {
                udsReceiving = false;
                parseDiagnosticData(udsBuffer, udsTotal);
            }
            break;
        }
        case 0x3:
            break;
        default:
            break;
    }
}

void parseDiagnosticData(uint8_t *data, uint16_t len) {
    if (len < 2) return;
    uint8_t sid = data[0];

    if (sid == 0x59) {
        dtcList = "";
        for (int i = 3; i + 2 < len; i += 3) {
            char buf[10];
            sprintf(buf, " %02X%02X%02X", data[i], data[i+1], data[i+2]);
            dtcList += buf;
        }
        hasDTC = (dtcList.length() > 0);
        addLog(hasDTC ? ("DTC: " + dtcList) : "No DTC");
    }
    else if (sid == 0x54) {
        hasDTC = false;
        dtcList = "Cleared";
        addLog("DTC cleared OK");
    }
    else if (sid == 0x62) {
        int payloadLen = len - 3;
        if (payloadLen > 0 && payloadLen <= 20) {
            char buf[21] = {0};
            for (int i = 0; i < payloadLen; i++) {
                buf[i] = (data[3 + i] >= 0x20 && data[3 + i] < 0x7F)
                         ? (char)data[3 + i] : '.';
            }
            vin = String(buf);
            addLog("VIN/ID: " + vin);
        }
    }
    else if (sid == 0x7F) {
        addLog("UDS NRC: SID=0x" + String(data[1], HEX) +
               " NRC=0x" + String(data[2], HEX));
    }
    else {
        addLog("UDS unknown SID: 0x" + String(sid, HEX));
    }
}

// ==================== Send Diagnostic Request (logic unchanged) ====================
void sendDiagnosticRequest(uint8_t sid, uint8_t sub, uint8_t data1) {
    twai_message_t msg = {};   // zero-init
    msg.identifier       = ID_DIAG_REQ;
    msg.data_length_code = 8;
    msg.data[0] = 0x03;
    msg.data[1] = sid;
    msg.data[2] = sub;
    msg.data[3] = data1;

    if (twai_transmit(&msg, pdMS_TO_TICKS(100)) != ESP_OK) {
        addLog("Tx diag FAILED");
    }
}

// ==================== Touch Handling (P2, P5, P6) ====================
void handleTouch() {
    touch.read();
    if (!touch.isTouched) return;

    if (millis() - lastTouchTime < 300) return;
    lastTouchTime = millis();

    // GT911 already reports native landscape coordinates for this panel,
    // no X/Y swap needed (unlike the T-Display S3 rotated-portrait case).
    uint16_t x = touch.points[0].x;
    uint16_t y = touch.points[0].y;

    // Tap left edge: previous page. Tap right edge: next page.
    // Zones rescaled for 1024px width (was 320px on T-Display S3).
    if (x < 150 && currentPage > 0) {
        currentPage--;
        addLog("Page " + String(currentPage));
    } else if (x > 874 && currentPage < PAGE_COUNT - 1) {
        currentPage++;
        addLog("Page " + String(currentPage));
    }

    // DTC page: tap bottom-center box to clear DTC.
    // Box drawn in drawDTC() at roughly x:400-624, y:500-580.
    if (currentPage == 2 && x > 400 && x < 624 && y > 500 && y < 580) {
        clearDTC = true;
        addLog("Touch: clear DTC");
    }
}

// ==================== Drawing Functions (Arduino_GFX, 1024x600) ====================
// Lines are drawn IN PLACE (padded with spaces, opaque black background) instead of
// clearing the whole screen every 200 ms -> no flicker on the RGB panel.
#define MARGIN_X 20
#define MARGIN_Y 16
static int16_t cy = MARGIN_Y;
static uint8_t curSize = 2;

static void setSize(uint8_t s) { curSize = s; gfx->setTextSize(s); }

static void gline(const char *fmt, ...) {
    int cols = (1024 - 2 * MARGIN_X) / (6 * curSize);
    char buf[100];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    int n = strlen(buf);
    if (n > cols) n = cols;
    while (n < cols && n < (int)sizeof(buf) - 1) buf[n++] = ' ';
    buf[n] = 0;
    gfx->setCursor(MARGIN_X, cy);
    gfx->print(buf);
    cy += 8 * curSize + 6;
}

void drawPage(uint8_t page) {
    static uint8_t lastPageDrawn = 0xFF;
    if (page != lastPageDrawn) {       // full clear only when the page changes
        gfx->fillScreen(BLACK);
        lastPageDrawn = page;
    }
    cy = MARGIN_Y;
    gfx->setTextColor(WHITE, BLACK);
    setSize(2);

    switch (page) {
        case 0: drawOverview(); break;
        case 1: drawCells();    break;
        case 2: drawDTC();      break;
        case 3: drawLog();      break;
        default: break;
    }
}

void drawOverview() {
    setSize(3);
    gline("=== OVERVIEW ===");
    setSize(2);
    gline("IGN: %s", ignitionOn ? "ON" : "OFF");
    gline("SOC: %d%%   Pack: %.1fV", soc, packVoltage);
    gline("Mode: %s", isCharging ? "CHARGING" : "DRIVE");
    gline("BMS: %s", isBMSalive ? "ALIVE" : "---");
    gline("DTC: %s", hasDTC ? "YES (next page ->)" : "NONE");
    gline("ID:  %s", vin.c_str());
    gline("SD:  %s", sdReady ? (sdLoggingEnabled ? "LOGGING" : "PAUSED") : "NOT FOUND");
    gline("");
    gline("< tap left/right edge to change page >");
}

void drawCells() {
    setSize(3);
    gline("=== CELL VOLTAGES ===");
    setSize(2);

    float minV = 9.9f, maxV = 0.0f;
    int validCount = 0;
    for (int i = 0; i < 22; i++) {
        if (cellVoltages[i] > 2.0f) {
            if (cellVoltages[i] < minV) minV = cellVoltages[i];
            if (cellVoltages[i] > maxV) maxV = cellVoltages[i];
            validCount++;
        }
    }

    for (int row = 0; row < 6; row++) {          // 4 cells per row, 22 cells
        char line[100] = {0};
        for (int k = 0; k < 4; k++) {
            int i = row * 4 + k;
            if (i >= 22) break;
            char c[24];
            snprintf(c, sizeof(c), "C%02d:%.3f   ", i + 1, cellVoltages[i]);
            strncat(line, c, sizeof(line) - strlen(line) - 1);
        }
        gline("%s", line);
    }
    gline("");
    gline("Active: %d cells", validCount);
    if (validCount > 0) {
        gline("Diff: %.3fV", maxV - minV);
        if (maxV - minV > 0.05f) {
            gfx->setTextColor(YELLOW, BLACK);
            gline("WARN: imbalance!");
            gfx->setTextColor(WHITE, BLACK);
        } else {
            gline("");
        }
    } else {
        gline("");
        gline("");
    }
}

void drawDTC() {
    setSize(3);
    gline("=== DTC LIST ===");
    setSize(2);
    if (hasDTC) {
        gfx->setTextColor(RED, BLACK);
        gline("%s", dtcList.c_str());
    } else {
        gfx->setTextColor(GREEN, BLACK);
        gline("%s", dtcList.length() > 0 ? dtcList.c_str() : "No DTC");
    }
    gfx->setTextColor(WHITE, BLACK);

    // Clear-DTC touch box (must match handleTouch() zone: x 400-624, y 500-580)
    gfx->drawRect(400, 500, 224, 80, WHITE);
    gfx->setCursor(420, 515);
    gfx->print("TOUCH TO ");
    gfx->setCursor(420, 545);
    gfx->print("CLEAR DTC");
}

void drawLog() {
    setSize(3);
    gline("=== LOG (%d lines) ===", logCount);
    setSize(2);
    for (int r = 0; r < 18; r++) {
        if (r < logCount) {
            int idx = (logIndex - 1 - r + LOG_MAX) % LOG_MAX;
            gline("%s", logBuffer[idx].c_str());
        } else {
            gline("");
        }
    }
    gline("< tap left/right edge to change page >");
}
