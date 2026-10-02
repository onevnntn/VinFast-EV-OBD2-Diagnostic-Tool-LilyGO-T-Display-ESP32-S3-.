# VinFast EV OBD2 Diagnostic Tool (Waveshare ESP32-S3-Touch-LCD-7B & LilyGO T-Display S3)

Dự án nghiên cứu và phát triển thiết bị chẩn đoán, đọc thông số kỹ thuật thời gian thực và quản lý mã lỗi (DTC) qua cổng CAN-BUS / UDS dành cho các dòng xe điện **VinFast (EVO200, Feliz S, Klara S, VF Series,...)** sử dụng vi điều khiển **ESP32-S3** (hỗ trợ bo màn hình cảm ứng **Waveshare ESP32-S3-Touch-LCD-7B** và **LilyGO T-Display S3**).

![VinFast EV Diagnostic Interface](https://github.com/user-attachments/assets/3ab6c50e-0783-4ad7-8861-2cce75575c91)

---

## 📌 Tính Nổi Bật

- **Giao tiếp CAN-BUS 500 kbps chuẩn:** Sử dụng bộ điều khiển TWAI tích hợp trên ESP32-S3 kết hợp transceiver giao tiếp trực tiếp với hệ thống xe điện VinFast.
- **Giám sát khối pin BMS toàn diện:** Đọc điện áp tổng (Pack Voltage), dung lượng pin (SOC %), trạng thái dòng sạc/xả và theo dõi chi tiết điện áp từng Cell (lên đến 22 cells).
- **Phân tích cảnh báo & Lệch áp Cell:** Tự động tính toán mức chênh lệch điện áp $V_{max} - V_{min}$, phát hiện cảnh báo mất cân bằng cell (Cell Imbalance Warning) khi vượt ngưỡng 0.05V.
- **Chẩn đoán chuẩn UDS (ISO 14229):**
  - Đọc và giải mã mã lỗi DTC (Diagnostic Trouble Codes).
  - Hỗ trợ xóa mã lỗi hệ thống (Clear DTC).
  - Đọc số khung / Thông tin định danh xe (VIN / ID).
- **Giao diện đa màn hình cảm ứng (Multi-page Touch UI):** Hỗ trợ chuyển đổi nhanh giữa các màn hình (Tổng quan, Điện áp Cell, Danh sách DTC, Nhật ký hệ thống Log) thông qua màn hình cảm ứng điện dung GT911.
- **Ghi nhật ký telemetry & Log vào thẻ nhớ SD:** Tự động lưu toàn bộ dữ liệu hoạt động theo thời gian thực dưới dạng file nhật ký `.TXT` và file dữ liệu `.CSV` định kỳ 1 giây/lần.

---

## 🛠️ Sơ Đồ Kết Nối Phần Cứng (Hardware Wiring)

### 1. Bo Mạch Waveshare ESP32-S3-Touch-LCD-7B $\leftrightarrow$ Cổng CAN / OBD2 Trái Tim Xe

| Chân trên Bo Mạch Waveshare 7B | Chân trên Cổng CAN / Transceiver | Ghi Chú |
| :--- | :--- | :--- |
| **GPIO 20** | **CAN_TX** | Tín hiệu truyền CAN (TWAI TX) |
| **GPIO 19** | **CAN_RX** | Tín hiệu nhận CAN (TWAI RX) |
| **EXIO 5 (IO Expander)** | **CAN_SEL** | Đặt mức HIGH để kích hoạt chế độ CAN Bus |
| **GND** | **GND (Âm hệ thống)** | Nối chung Mass toàn bộ thiết bị |

> ⚠️ **Lưu ý quan trọng:** Đảm bảo hệ thống CAN transceiver trên mạch đã được cấp nguồn thích hợp và có trở định thiên kết thúc bus (Termination Resistor $120\Omega$) ở hai đầu đường truyền CAN.

### 2. Giao Tiếp Cảm Ứng, Đèn Nền & Thẻ Nhớ Qua Mạch Mở Rộng I2C (CH32V003 IO Expander)

- **Địa chỉ I2C IO Expander:** `0x24` (I2C SDA: `GPIO 8`, SCL: `GPIO 9`).
- **Giao tiếp Thẻ nhớ TF/SD Card:** MOSI (`GPIO 11`), SCK (`GPIO 12`), MISO (`GPIO 13`), CS (`EXIO 4` giữ mức LOW).
- **Giao tiếp Cảm ứng GT911:** Địa chỉ I2C `0x5D`, Chân ngắt INT (`GPIO 4`), Chân Reset (`EXIO 1`).

---

## 💻 Mã Nguồn Arduino C++ (`evo200_can_diag_7b.ino`)

```cpp
// =====================================================================
// EVO200 CAN Diagnostic — PORTED to Waveshare ESP32-S3-Touch-LCD-7B
// (RGB 1024x600 display, GT911 capacitive touch, CAN via IO-Expander)
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

#define IOEXP_REG_MODE    0x02   // pin direction, bit=1 -> output (MUST be set to 0xFF first!)
#define IOEXP_REG_OUTPUT  0x03   // output levels
#define IOEXP_REG_INPUT   0x04   // input levels
#define IOEXP_REG_PWM     0x05   // backlight PWM

#define IOEXP_BIT_TP_RST     1   // touch reset
#define IOEXP_BIT_DISP_BL    2   // backlight enable
#define IOEXP_BIT_LCD_RST    3   // LCD reset
#define IOEXP_BIT_SD_CS      4   // TF card CS
#define IOEXP_BIT_CAN_SEL    5   // HIGH = CAN mode
#define IOEXP_BIT_LCD_VDD_EN 6   // VCOM enable

#define BACKLIGHT_PWM_FULL   1   
#define TOUCH_INT_GPIO       4   

// ==================== Pin definitions ====================
#define CAN_TX_PIN GPIO_NUM_20   
#define CAN_RX_PIN GPIO_NUM_19   

// ==================== SD / TF card ====================
#define SD_MOSI_PIN 11
#define SD_SCK_PIN  12
#define SD_MISO_PIN 13
#define SD_CS_DUMMY_PIN 6

// ==================== CAN IDs ====================
#define ID_IGNITION         0x101
#define ID_VEHICLE_MODE     0x110
#define ID_BMS_STATUS       0x303
#define ID_PACK_VOLTAGE     0x309
#define ID_CELL_START       0x311   
#define ID_CELL_END         0x31B
#define ID_SOC              0x322
#define ID_CHARGER_STATUS   0x342
#define ID_DIAG_REQ         0x006
#define ID_DIAG_RESP        0x002

// ==================== Display (Arduino_GFX RGB panel, 1024x600) ====================
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
  5, 3, 46, 7,
  1, 2, 42, 41, 40,
  39, 0, 45, 48, 47, 21,
  14, 38, 18, 17, 10,
  0, 48, 162, 152,
  0, 3, 45, 13,
  1, 30000000
);
Arduino_RGB_Display *gfx = new Arduino_RGB_Display(1024, 600, rgbpanel);

#define BLACK   0x0000
#define WHITE   0xFFFF
#define RED     0xF800
#define GREEN   0x07E0
#define YELLOW  0xFFE0

// ==================== Touch (GT911) ====================
TAMC_GT911 touch = TAMC_GT911(8, 9, -1, -1, 1024, 600);

// ==================== Global Variables ====================
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

// ==================== SD Logging State ====================
bool   sdReady = false;
bool   sdLoggingEnabled = true;
File   sdLogFile;   
File   sdDataFile;  
String sdLogFileName;
String sdDataFileName;
uint32_t lastSDFlush = 0;
uint32_t lastSDData  = 0;

// ==================== Function Prototypes ====================
void addLog(const char *msg);
void processCANFrame(twai_message_t *msg);
void processDiagnosticResponse(twai_message_t *msg);
void parseDiagnosticData(uint8_t *data, uint16_t len);
void sendDiagnosticRequest(uint8_t sid, uint8_t sub, uint8_t data1);
void handleTouch();
void drawPage(uint8_t page);
void drawOverview();
void drawCells();
void drawDTC();
void drawLog();

static uint8_t ioexp_out = 0xFF;

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

void init_io_expander() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);

    ioexp_write_reg(IOEXP_REG_MODE, 0xFF);

    ioexp_out = 0xFF;
    ioexp_out &= ~((1 << IOEXP_BIT_TP_RST) | (1 << IOEXP_BIT_LCD_RST) |
                   (1 << IOEXP_BIT_DISP_BL) | (1 << IOEXP_BIT_SD_CS));
    ioexp_write_reg(IOEXP_REG_OUTPUT, ioexp_out);

    pinMode(TOUCH_INT_GPIO, OUTPUT);
    digitalWrite(TOUCH_INT_GPIO, LOW);
    delay(20);
    ioexp_set(IOEXP_BIT_LCD_RST, true);   
    ioexp_set(IOEXP_BIT_TP_RST, true);    
    delay(10);
    pinMode(TOUCH_INT_GPIO, INPUT);       
    delay(60);                            
}

void backlight_on() {
    ioexp_write_reg(IOEXP_REG_PWM, BACKLIGHT_PWM_FULL);
    ioexp_set(IOEXP_BIT_DISP_BL, true);
}

bool initSD() {
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_DUMMY_PIN);

    if (!SD.begin(SD_CS_DUMMY_PIN, SPI, 20000000)) return false;
    if (SD.cardType() == CARD_NONE) return false;

    uint32_t session = 1;
    if (SD.exists("/NEXTLOG.TXT")) {
        File f = SD.open("/NEXTLOG.TXT", FILE_READ);
        if (f) { session = f.parseInt(); f.close(); }
    }
    File fw = SD.open("/NEXTLOG.TXT", FILE_WRITE);
    if (fw) { fw.print(session + 1); fw.close(); }

    char nameBuf[24];
    snprintf(nameBuf, sizeof(nameBuf), "/LOG_%05lu.TXT", (unsigned long)session);
    sdLogFileName = nameBuf;
    snprintf(nameBuf, sizeof(nameBuf), "/DATA_%05lu.CSV", (unsigned long)session);
    sdDataFileName = nameBuf;

    sdLogFile  = SD.open(sdLogFileName, FILE_WRITE);
    sdDataFile = SD.open(sdDataFileName, FILE_WRITE);
    if (!sdLogFile || !sdDataFile) return false;

    sdDataFile.println("timestamp_ms,ignition,soc_pct,pack_v,charging,bms_alive,has_dtc,dtc_list,vin,"
                       "c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,c16,c17,c18,c19,c20,c21,c22");
    sdDataFile.flush();
    return true;
}

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

void setup() {
    Serial.begin(115200);
    delay(100);
    addLog("=== VinFast EV OBD2 Diagnostic (7B) ===");

    init_io_expander();

    if (!gfx->begin()) addLog("Display init FAILED");
    gfx->fillScreen(BLACK);
    gfx->setTextColor(WHITE, BLACK);
    gfx->setTextSize(2);
    gfx->setCursor(0, 0);
    backlight_on();

    touch.begin();
    touch.setRotation(ROTATION_NORMAL);

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
        addLog("CAN 500k started");
    } else {
        addLog("CAN init FAILED");
    }

    sdReady = initSD();
    if (sdReady) addLog("SD ready: " + sdDataFileName);

    delay(500);
    requestVIN = true;
    requestDTC = true;
}

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

    if (sdReady && sdLoggingEnabled && sdDataFile && (millis() - lastSDData >= 1000)) {
        lastSDData = millis();
        sdDataFile.printf("%lu,%d,%u,%.2f,%d,%d,%d,\"%s\",\"%s\"",
            (unsigned long)millis(), ignitionOn ? 1 : 0, soc, packVoltage,
            isCharging ? 1 : 0, isBMSalive ? 1 : 0, hasDTC ? 1 : 0,
            dtcList.c_str(), vin.c_str());
        for (int i = 0; i < 22; i++) sdDataFile.printf(",%.3f", cellVoltages[i]);
        sdDataFile.println();
        sdDataFile.flush();
    }
}

// [Mã nguồn xử lý CAN, UDS và Hiển thị đầy đủ như mô tả trong dự án]
```

---

## 📖 Cảnh Báo Miễn Trừ Trách Nhiệm (Disclaimer)

> Dự án phục vụ mục đích nghiên cứu học thuật, chẩn đoán kỹ thuật và tham khảo cá nhân. Tác giả không chịu trách nhiệm đối với bất kỳ rủi ro, hư hỏng thiết bị, ảnh hưởng đến chế độ bảo hành của xe hoặc sự cố an toàn nào phát sinh khi áp dụng thực tế trên các hệ thống xe điện VinFast.

---

## ☕ Ủng Hộ Tác Giả (Donate)

Ủng hộ mình nếu thấy dự án có ích! 

Zalo: **0844491666** (Tôi sẽ trả lời khi rảnh do không có nhiều thời gian vì phải đi kiếm tiền).

| VietQR Techcombank | Thông Tin Chuyển Khoản |
| :---: | :--- |
| ![Techcombank QR](https://github.com/user-attachments/assets/3ab6c50e-0783-4ad7-8861-2cce75575c91) | **Chủ tài khoản:** TRAN DUY THO<br>**Số tài khoản:** `3013 2838 69`<br>**Ngân hàng:** Techcombank |