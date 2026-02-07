// ==========================================================
// USER CONFIGURATION - QUICK SETTINGS
// ==========================================================
// Full documentation: See wiki Configuration.md
// Topic format: wesense/v2/wifi/{country}/{subdivision}/{device_id}
// Protobuf: v2.1 with BoardType, node_name, altitude support

// Credentials are stored separately - copy credentials.h.example to credentials.h
#include "credentials.h"

// === DEVICE IDENTITY ===
const char* COUNTRY_CODE = "nz";                  // ISO 3166-1: nz, au, us, gb, de, fr, jp, etc.
const char* SUBDIVISION_CODE = "auk";             // ISO 3166-2: auk, wgn, nsw, vic, ca, tx, etc.

// Node naming: Map display shows NODE_NAME_PREFIX + "_" + DEVICE_LOCATION
// Examples:
//   PREFIX="Smith"  LOCATION="Garage"   →  "Smith_Garage"
//   PREFIX="Home"   LOCATION="Bedroom2" →  "Home_Bedroom2"
//   PREFIX=""       LOCATION="R2D2"     →  "R2D2" (custom name, no prefix)
// Values loaded from credentials.h
const char* NODE_NAME_PREFIX = NODE_PREFIX;       // Shared across all your devices (leave empty for custom names)
const char* DEVICE_LOCATION = NODE_LOCATION;      // Unique per device - the only field you need to change!

// INDOOR, OUTDOOR, or MIXED like a covered deck
typedef enum { INDOOR, OUTDOOR, MIXED } DeploymentType;
const DeploymentType DEPLOYMENT_TYPE = OUTDOOR;     

// === NODE INFO - Physical setup description for data analysis ===
// Describe the physical installation to help with data interpretation
// Examples: "Outdoor pole, perspex case, south-facing", "Indoor, near window, HVAC vent 2m away"
const char* NODE_INFO = "Covered Balcony"; // Physical setup description, max 64 chars
const char* NODE_INFO_URL = "";                   // URL to detailed setup documentation, show off your setup! (optional, max 96 chars)

// === CONNECTIVITY - Transport Control ===
const bool ENABLE_WIFI = true;                    // WiFi for MQTT/NTP
const bool ENABLE_MQTT = true;                    // Publish sensor data to MQTT
const bool DISABLE_LORAWAN = true;               // Set true to disable LoRaWAN
const bool WIFI_STOP_AFTER_TIME_SYNC = false;     // Stop WiFi after NTP sync (LoRaWAN-only mode)

// === CONNECTIVITY - WiFi & MQTT ===
// Credentials loaded from credentials.h (copy credentials.h.example to get started)
const char* wifi_ssid = WIFI_SSID;
const char* wifi_password = WIFI_PASSWORD;
const char* mqtt_server = MQTT_SERVER;
const int mqtt_port = MQTT_PORT;
const char* mqtt_user = MQTT_USER;
const char* mqtt_password = MQTT_PASSWORD;

// === HARDWARE - Disable Individual Components (Compile-Time) ===
// Set to true to completely exclude sensor code from the compiled binary.
// Reduces flash usage and prevents any initialization attempts.
// These are "master off switches" - the sensor code won't exist in the firmware.
const bool DISABLE_TMP117 = false;                // Temperature only (highest precision ±0.1°C)
const bool DISABLE_SHT4X = false;                 // Temperature/Humidity (high precision)
const char* SHT4X_VARIANT = "SHT41";              // Which SHT4x variant: "SHT40", "SHT41", or "SHT45"
                                                  // Important: SHT40 has known drift issues - set correctly for data quality tracking
const bool DISABLE_AHT20 = false;                 // Temperature/Humidity (backup)
const bool DISABLE_SCD4X = false;                 // CO2 sensor (SCD40/SCD41)
const bool DISABLE_SCD30 = false;                 // CO2 sensor (SCD30, older model)
const bool DISABLE_CM1106C = false;               // CO2 sensor CM1106-C (I2C)
const bool DISABLE_C8_CO2 = false;                // CO2 sensor CM1106-C (UART, legacy name "C8")
const bool DISABLE_SGP41 = false;                 // VOC/NOx air quality
const bool DISABLE_BMP280 = false;                // Barometric pressure
const bool DISABLE_BMP390 = false;                // Barometric pressure (BMP390L, higher precision)
const bool DISABLE_MS5611 = false;                // Barometric pressure (high precision altimeter)
const bool DISABLE_BME680 = false;                // Pressure + gas resistance
const bool DISABLE_PMS5003 = false;               // Particulate matter PM1/2.5/10
const bool DISABLE_INA219 = false;                // DC power monitor
const bool DISABLE_GPS = false;                   // GPS module
const bool DISABLE_PMU = false;                   // Power management (T-Beam)
const bool DISABLE_DISPLAY = false;               // OLED display

// === HARDWARE - Disable Individual Readings (Runtime Switchable) ===
// Use these to disable a specific reading from a sensor while keeping others.
// Useful for faulty sensors - e.g., AHT20 temp reads high but humidity is OK.
// These can be controlled via MQTT commands for live troubleshooting.
const bool DISABLE_TMP117_TEMPERATURE = false;    // Disable temp reading from TMP117
const bool DISABLE_SHT4X_TEMPERATURE = false;      // Disable temp reading from SHT4x
const bool DISABLE_SHT4X_HUMIDITY = false;        // Disable humidity reading from SHT4x
const bool DISABLE_AHT20_TEMPERATURE = false;     // Disable temp reading from AHT20
const bool DISABLE_AHT20_HUMIDITY = false;        // Disable humidity reading from AHT20
const bool DISABLE_SCD4X_TEMPERATURE = false;     // Disable temp reading from SCD4x (keep CO2)
const bool DISABLE_SCD4X_HUMIDITY = false;        // Disable humidity reading from SCD4x (keep CO2)
const bool DISABLE_SCD30_TEMPERATURE = false;     // Disable temp reading from SCD30 (keep CO2)
const bool DISABLE_SCD30_HUMIDITY = false;        // Disable humidity reading from SCD30 (keep CO2)
const bool DISABLE_BME680_TEMPERATURE = false;    // Disable temp reading from BME680
const bool DISABLE_BME680_HUMIDITY = false;       // Disable humidity reading from BME680
const bool DISABLE_BMP280_TEMPERATURE = false;    // Disable temp reading from BMP280 (keep pressure)
const bool DISABLE_BMP390_TEMPERATURE = false;    // Disable temp reading from BMP390 (keep pressure)
const bool DISABLE_MS5611_TEMPERATURE = false;    // Disable temp reading from MS5611 (keep pressure)
// CO2-only sensors (single reading type)
const bool DISABLE_CM1106C_CO2 = false;           // Disable CO2 reading from CM1106-C I2C
const bool DISABLE_C8_CO2_CO2 = false;            // Disable CO2 reading from CM1106-C UART

// === TIMING - Transmission Intervals ===
const unsigned long MQTT_PUBLISH_INTERVAL_MS = 300000;    // 5 min - WiFi/MQTT publish
const unsigned long LORAWAN_TX_INTERVAL_MS = 300000;      // 5 min - LoRaWAN transmit (can increase to save airtime)

// === POWER MANAGEMENT - Deep Sleep ===
// Deep sleep is for battery-powered LoRaWAN Class A deployments.
// Device sleeps between transmissions, waking only to read sensors and TX.
// IMPORTANT: SGP41 VOC/NOx and BME680 gas readings are auto-disabled (incompatible).
const bool ENABLE_DEEP_SLEEP = true;                      // Enable deep sleep (requires LoRaWAN, no active WiFi)
const unsigned long SENSOR_WARMUP_MS = 10000;             // 10s base warmup (for SCD4x CO2)
const unsigned long PM_SENSOR_WARMUP_MS = 30000;          // 30s extended warmup if PM sensor detected
const unsigned long POST_TX_DELAY_MS = 100;               // Brief delay after TX before sleep

// === LORAWAN - TTN Credentials ===
// Get these from The Things Network Console (MSB format)
// Credentials loaded from credentials.h
static const uint8_t PROGMEM USER_APPEUI[8] = LORAWAN_APPEUI;
static const uint8_t PROGMEM USER_DEVEUI[8] = LORAWAN_DEVEUI;
static const uint8_t PROGMEM USER_APPKEY[16] = LORAWAN_APPKEY;

// === LORAWAN - Region (auto-detected from COUNTRY_CODE, or set manually) ===
#define AUTO_DETECT_REGION true
// Manual override (uncomment ONE if AUTO_DETECT_REGION is false):
// #define LORAWAN_REGION_AU915  // Australia, New Zealand
// #define LORAWAN_REGION_EU868  // Europe
// #define LORAWAN_REGION_US915  // United States, Canada

// === LOCATION - GPS & Fixed Position ===
const bool INCLUDE_LOCATION_IN_MQTT = true;       // Include location in MQTT payload
const bool PREFER_GPS_TIME = false;               // true for off-grid (GPS time vs NTP)

// Fixed location for stationary deployments (used when GPS unavailable/disabled)
// Set all to 0.0 to disable fixed location and rely on GPS only
// Values loaded from credentials.h
const float FIXED_LATITUDE = FIXED_LAT;           // Latitude in decimal degrees
const float FIXED_LONGITUDE = FIXED_LON;          // Longitude in decimal degrees
const float FIXED_ALTITUDE = FIXED_ALT;           // Altitude in meters above sea level

typedef enum { GPS_PRIVACY_FULL = 0, GPS_PRIVACY_REDUCED = 1, GPS_PRIVACY_OBFUSCATED = 2 } GpsPrivacyMode;
const GpsPrivacyMode GPS_PRIVACY_MODE = GPS_PRIVACY_OBFUSCATED;  // FULL (~1m), REDUCED (~1km), OBFUSCATED (±100m jitter)

// === TIME - NTP Configuration ===
const char* ntp_server = "pool.ntp.org";
const char* ntp_server2 = "time.nist.gov";
const long gmt_offset_sec = GMT_OFFSET_SEC;       // Timezone offset from credentials.h
const int daylight_offset_sec = DAYLIGHT_OFFSET_SEC; // DST offset from credentials.h

// === CO2 SENSOR CALIBRATION ===
// ASC/ABC assume access to ~400ppm fresh air. Disable for urban/indoor deployments!
const bool ENABLE_SCD4X_ASC_BY_DEFAULT = false;   // SCD4x Auto Self-Calibration
const bool ENABLE_SCD30_ASC_BY_DEFAULT = false;   // SCD30 Auto Self-Calibration
const bool ENABLE_CM1106C_ABC_BY_DEFAULT = false; // CM1106-C Auto Baseline Correction

// === CALIBRATION & DATA QUALITY ===
const bool ENABLE_CALIBRATION_STATE_TRACKING = true;   // Track sensor calibration state
const bool SUPPRESS_DATA_DURING_CALIBRATION = true;    // Don't publish during calibration
const bool ENABLE_TESTING_MODE = true;                 // Allow MQTT testing mode command

// === DEBUG & DIAGNOSTICS ===
const bool ENABLE_DEBUG_OUTPUT = true;            // Serial debug messages
const bool USE_UTC_TIMESTAMPS = true;             // UTC for global systems (recommended)
#define ENABLE_PROTOBUF true                      // Compact protobuf encoding

// === STATUS LED ===
const bool DISABLE_STATUS_LED = false;            // Disable onboard status LED
const unsigned long LED_SLOW_BLINK_INTERVAL_MS = 5000;   // Slow blink interval (normal operation)
const unsigned long LED_FAST_BLINK_INTERVAL_MS = 500;    // Fast blink interval (connecting/error)

// === REMOTE ACCESS - Network Serial Monitor ===
const bool ENABLE_SECURE_TELNET = true;           // Enable telnet debugging
const int TELNET_PORT = 2323;                     // Telnet port (non-standard for security)
const char* TELNET_PASSWORD = TELNET_PASS;        // From credentials.h
const char* ALLOWED_TELNET_IP = TELNET_ALLOWED_IP; // From credentials.h

// === REMOTE LOGGING - Syslog Server ===
const bool ENABLE_SYSLOG = true;                 // Enable UDP syslog logging
const char* SYSLOG_SERVER = SYSLOG_HOST;          // From credentials.h
const int SYSLOG_PORT = SYSLOG_UDP_PORT;          // From credentials.h
const bool SYSLOG_INCLUDE_HEAP = true;            // Include heap info in periodic logs

// ==========================================================
// ADVANCED SETTINGS (most users won't need to change these)
// ==========================================================

// --- LoRaWAN Region Auto-Detection ---
#if AUTO_DETECT_REGION
  // Auto-detect region based on COUNTRY_CODE (ISO 3166-1 alpha-2)
  #if defined(COUNTRY_CODE)
    #if (strcmp(COUNTRY_CODE, "au") == 0) || (strcmp(COUNTRY_CODE, "nz") == 0)
      #define LORAWAN_REGION_AU915 // Australia/New Zealand
    #elif (strcmp(COUNTRY_CODE, "us") == 0) || (strcmp(COUNTRY_CODE, "ca") == 0)
      #define LORAWAN_REGION_US915 // United States/Canada
    #elif (strcmp(COUNTRY_CODE, "gb") == 0) || (strcmp(COUNTRY_CODE, "de") == 0) || (strcmp(COUNTRY_CODE, "fr") == 0) || (strcmp(COUNTRY_CODE, "nl") == 0) || (strcmp(COUNTRY_CODE, "it") == 0) || (strcmp(COUNTRY_CODE, "es") == 0)
      #define LORAWAN_REGION_EU868 // Europe
    #elif (strcmp(COUNTRY_CODE, "jp") == 0) || (strcmp(COUNTRY_CODE, "sg") == 0) || (strcmp(COUNTRY_CODE, "th") == 0) || (strcmp(COUNTRY_CODE, "my") == 0)
      #define LORAWAN_REGION_AS923 // Asia Pacific
    #elif (strcmp(COUNTRY_CODE, "in") == 0)
      #define LORAWAN_REGION_IN865 // India
    #elif (strcmp(COUNTRY_CODE, "kr") == 0)
      #define LORAWAN_REGION_KR920 // South Korea
    #elif (strcmp(COUNTRY_CODE, "cn") == 0)
      #define LORAWAN_REGION_CN470 // China
    #else
      #define LORAWAN_REGION_AU915 // Default fallback
      #warning "Unknown COUNTRY_CODE, defaulting to AU915. Please check your country code setting."
    #endif
  #else
    #define LORAWAN_REGION_AU915 // Default if no COUNTRY_CODE specified
    #warning "AUTO_DETECT_REGION enabled but no COUNTRY_CODE specified, defaulting to AU915."
  #endif
#endif

// Validation: Ensure exactly one region is defined
#ifndef LORAWAN_REGION_AU915
  #ifndef LORAWAN_REGION_EU868
    #ifndef LORAWAN_REGION_US915
      #ifndef LORAWAN_REGION_AS923
        #ifndef LORAWAN_REGION_IN865
          #ifndef LORAWAN_REGION_KR920
            #ifndef LORAWAN_REGION_CN470
              #define LORAWAN_REGION_AU915 // Ultimate fallback
              #warning "No LoRaWAN region defined! Defaulting to AU915."
            #endif
          #endif
        #endif
      #endif
    #endif
  #endif
#endif

// --- I2C Sensor Addresses ---
const uint8_t SCD4X_I2C_ADDRESS = 0x62;
const uint8_t SCD30_I2C_ADDRESS = 0x61;
const uint8_t CM1106C_I2C_ADDRESS = 0x31;

// --- SCD4x/SCD30 Compensation Strategy ---
const bool USE_EXTERNAL_SENSORS_FOR_SCD4X_COMPENSATION = false;  // false=internal T/H, true=external sensors
const bool USE_EXTERNAL_SENSORS_FOR_SCD30_COMPENSATION = false;  // false=internal T/H, true=external sensors

// --- Calibration Monitoring ---
const bool ENABLE_CALIBRATION_MONITORING = true;
const bool PUBLISH_RAW_READINGS_DURING_CALIBRATION = true;
const bool PERSIST_CALIBRATION_STATE = true;
const unsigned long DIAGNOSTIC_PUBLISH_INTERVAL_MS = 30000;

// --- Calibration State Persistence ---
const unsigned long NVS_SAVE_INTERVAL_MS = 60000;                   // NVS backup (60s dev, 300s prod)
const unsigned long EEPROM_SAVE_INTERVAL_CALIBRATING_MS = 7200000;  // 2 hours during calibration
const unsigned long EEPROM_SAVE_INTERVAL_CALIBRATED_MS = 86400000;  // 24 hours when calibrated

// --- Calibration Periods (hours) ---
struct SensorCalibrationPeriods {
  unsigned long scd4x = 168;      // 7 days (ASC needs exposure to ~400ppm fresh air)
  unsigned long scd30 = 168;      // 7 days (ASC same as SCD4x)
  unsigned long cm1106c = 384;    // 24h + 15 days (ABC per CM1106 datasheet: 24h initial + 15 day cycle)
  unsigned long c8co2 = 384;      // 24h + 15 days (CM1106-C UART variant, same ABC as I2C version)
  unsigned long sgp41 = 12;       // 12 hours
  unsigned long bme680 = 48;      // 2 days
  unsigned long pms5003 = 0;      // Disabled - PMS5003 is factory calibrated, 30s warm-up handled separately
  unsigned long bmp280 = 0;       // Immediate
  unsigned long sht4x = 0;        // Immediate
  unsigned long aht20 = 0;        // Immediate
  unsigned long tmp117 = 0;       // Immediate (no warm-up needed)
} calibrationPeriods;

// --- Display Configuration ---
const uint8_t DISPLAY_I2C_ADDRESS = 0x3C; // Default OLED address (0x3C or 0x3D)
const uint8_t DISPLAY_WIDTH = 128;        // OLED display width in pixels
const uint8_t DISPLAY_HEIGHT = 64;        // OLED display height in pixels
// Display driver type: 0 = Auto-detect, 1 = Force SSD1306, 2 = Force SH1106
// Auto-detect probes the chip via status register to identify SSD1306 vs SH1106
const uint8_t DISPLAY_DRIVER_TYPE = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL_MS = 2000; // Update display every 2 seconds
const unsigned long DISPLAY_ROTATION_INTERVAL_MS = 5000; // Rotate between screens every 5 seconds
const unsigned long DISPLAY_TIMEOUT_MS = 600000; // Turn off display after 10 minutes (600,000ms)
// Boot button pin varies by ESP32 variant
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
  const uint8_t BOOT_BUTTON_PIN = 9; // GPIO9 is the boot button on ESP32-C3/C6
#else
  const uint8_t BOOT_BUTTON_PIN = 0; // GPIO0 is the boot button on most ESP32 boards
#endif
const unsigned long BUTTON_DEBOUNCE_MS = 50; // Button debounce time

// --- INA219 Configuration ---
const uint8_t INA219_I2C_ADDRESS = 0x40; // Default INA219 address (0x40-0x4F depending on A0/A1 pins)
const float INA219_SHUNT_OHMS = 0.1;     // Shunt resistor value in ohms (typical: 0.1Ω)
const float INA219_MAX_EXPECTED_AMPS = 3.2; // Maximum current capability (3.2A per board specs)
// Board specs: Bus Voltage 0-26V, Max Current 3.2A, VCC/Logic 3-5V

// --- Board Detection Override ---
const bool FORCE_TBEAM_T3_S3_V12 = false;  // Force T-Beam T3 S3 detection

// --- Telnet Timeouts ---
const unsigned long TELNET_MAX_CONNECTION_TIME_MS = 86400000;  // 24 hours max
const unsigned long TELNET_IDLE_TIMEOUT_MS = 28800000;         // 8 hours idle timeout

// --- NTP Resync ---
const unsigned long NTP_RESYNC_INTERVAL_MS = 86400000;  // 24 hours

// --- I2C Configuration ---
const uint32_t I2C_FREQUENCY = 50000;  // 50kHz for reliability

// ==========================================================
// USER CONFIGURATION ENDS
// ==========================================================

// ==========================================================
// DEVELOPER CONFIGURATION
// ==========================================================

// --- Firmware Version for Calibration Data Compatibility ---
// ONLY increment this when making changes that would break existing calibration data compatibility
// Examples: changing calibration data format, storage structure, or calibration algorithms
// Do NOT increment for: bug fixes, feature additions, I2C changes, or other non-calibration changes
const char* FIRMWARE_VERSION = "v2025.12.1";

// --- Ultra-Robust Calibration Persistence Configuration ---
// Multiple protection layers for 7-day calibrations on deployed sensors
const int EEPROM_BACKUP_SLOTS = 3;           // Multiple EEPROM backup slots
const int NVS_BACKUP_NAMESPACES = 2;         // Multiple NVS namespaces  
const unsigned long EMERGENCY_SAVE_INTERVAL = 3600000; // Save every hour during calibration
const bool ENABLE_PARANOID_VALIDATION = true; // Extra integrity checks

// --- Write Protection Against Calibration Data Loss ---
const bool ENABLE_BACKUP_WRITE_PROTECTION = true;    // Prevent overwriting valuable backups
const unsigned long MIN_CALIBRATION_AGE_TO_PROTECT = 3600; // Protect calibration data older than 1 hour
const bool REQUIRE_EXPLICIT_RESET_COMMAND = true;    // Require MQTT command to reset calibration

// ==========================================================
// DEVELOPER CONFIGURATION ENDS
// ==========================================================


// ==========================
// ESP32-C6 SPECIFIC CONFIGURATION
// ==========================
// ESP32-C6 USB configuration for standalone operation
#ifdef CONFIG_IDF_TARGET_ESP32C6
  // Disable USB CDC wait for standalone operation
  #define USB_CDC_ON_BOOT 0
#endif

// ==========================
// REQUIRED LIBRARIES
// ==========================
#include <Wire.h>           // For I2C communication
#include <SPI.h>            // For LoRa communication
#include "board_config.h"   // Board abstraction layer

// --- WiFi & MQTT ---
#include <WiFi.h>
#include <WiFiServer.h>   // For secure telnet monitor
#include <WiFiUdp.h>      // For syslog client
#include <PubSubClient.h>
#include <time.h>
#include <set>
#include <Preferences.h>  // For calibration state persistence
#include <EEPROM.h>       // For emergency calibration backup
#include "esp_partition.h" // For NVS partition diagnostics

// --- LoRaWAN (RadioLib) ---
// RadioLib has excellent ESP32-S3 and multi-radio support
#include <RadioLib.h>

// --- Sensors ---
#include <Adafruit_BMP280.h>
#include <Adafruit_BMP3XX.h>  // BMP390L pressure sensor
#include <MS5611.h>           // MS5611 high-precision barometric pressure sensor
#include <Adafruit_BME680.h>
#include <SparkFun_SCD4x_Arduino_Library.h>
#include <SparkFun_SCD30_Arduino_Library.h>  // SCD30 CO2 sensor
#include <SparkFun_TMP117.h>  // TMP117 high-precision temperature sensor
#include <cm1106_i2c.h>
#include <SensirionI2CSgp41.h>
#include <SensirionI2cSht4x.h>
extern "C" {
#include "sensirion_gas_index_algorithm.h"
}
#include <Adafruit_AHTX0.h>

// --- GPS ---
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// --- PMS5003 Air Quality Sensor ---
// We'll implement our own simple PMS5003 reading since there are many libraries
// This gives us more control and reduces dependencies

// --- Power Management ---
#include "XPowersLib.h" // Assuming this is for AXP2101
#include <Adafruit_INA219.h> // INA219 DC Current Monitor

// --- Deep Sleep ---
#include "esp_sleep.h"
#include "driver/rtc_io.h"

// --- Display ---
#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_SSD1306.h> // OLED display library (0.96" displays)
#include <Adafruit_SH110X.h>  // OLED display library (1.3" SH1106/SH1107 displays)
#include "wesense_logo.h"     // WeSense logo bitmaps for splash screen
#include "wesense_icons.h"    // 8x8 icons for navigation bar and status indicators

// --- Protobuf Encoding (Optional) ---
#if ENABLE_PROTOBUF
#include "protobuf_encoder_v2.h" // V2: All sensors in one message (compact)
#endif

// ==========================
// FORWARD DECLARATIONS
// ==========================
void publishSensorDataWithCalibration(const char* topicSuffix, float value, const String& sensorName = "");
void publishSensorData(const char* topicSuffix, float value);
void publishAllSensorsMQTT(uint32_t cycleTimestamp);  // V2 consolidated protobuf publishing
bool shouldPublishSensorData(const String& sensorName);
bool shouldPublishSensorData(const String& sensorName, const char* measurementType);
bool shouldPublishSensorData();
void publishCommandResult(const String& status, const String& message, const String& commandPath);
String getTimestamp();
String getTimestampUTC();
String getSensorCalibrationStatus(const String& sensorName);
struct SensorCalibrationState; // Forward declare struct
SensorCalibrationState* getSensorCalibrationState(const String& sensorName);
void saveCalibrationState();
unsigned long getCurrentUnixTime();
String formatRemainingTime(unsigned long remainingMs);
void calculateRemainingTime(unsigned long totalMs, unsigned long& hours, unsigned long& minutes);
unsigned long getRemainingCalibrationMs(const String& sensorName);
void startFreshCalibrationForUncalibratedSensors();
void subscribeToCommandTopics();
void publishAllDiscoveryConfigs();
void processCommand(const String& commandPath, const String& message);
void processSensorCommand(const String& commandPath, const String& payload);
void processDeviceCommand(const String& commandPath, const String& payload);
void processSystemCommand(const String& commandPath, const String& payload);
void handleSCD4xCommand(const String& action, const String& payload);
void handleSCD30Command(const String& action, const String& payload);
void handleCM1106CCommand(const String& action, const String& payload);
void handleSGP41Command(const String& action, const String& payload);
void handleBME680Command(const String& action, const String& payload);
void handleBMP280Command(const String& action, const String& payload);
void handleLEDCommand(const String& action, const String& payload);
void handleConfigCommand(const String& action, const String& payload);
void handleDisplayCommand(const String& action, const String& payload);
void handleSystemRestart(const String& payload);
void handleSystemStatus(const String& payload);
void handleFactoryReset(const String& payload);
void handleTestingMode(bool enable, const String& payload);
void handleSyslogEnable(bool enable);
void handleSyslogStatus();
void handleCalibrationStatus(const String& payload);
void handleCalibrationBackup(const String& payload);
void handleCalibrationRestore(const String& payload);
void handleSCD4xCalibration(const String& payload);
void handleSCD4xReset();
void handleSCD4xSelfTest();
void handleSCD4xASC(bool enable);
void publishSCD4xASCStatus();
void handleSCD4xPressureCompensationStatus();
void handleSCD4xPressureCompensationApply();
void handleSCD4xFactoryReset();
void handleBME680GasHeater(const String& payload);
void handleBMP280Diagnostic();
void handleLocationCommand(const String& payload);
void handleLocationResetCommand();
void handleLocationStatusCommand();
void setLEDMode(bool fastMode);
void publishDiagnosticReading(const String& sensorName, const String& measurement, float value, const String& unit);

// ==========================
// UART SENSOR DATA STRUCTURES
// ==========================
struct PMS5003Data {
  uint16_t pm1_0_standard;
  uint16_t pm2_5_standard;
  uint16_t pm10_standard;
  uint16_t pm1_0_atmospheric;
  uint16_t pm2_5_atmospheric;
  uint16_t pm10_atmospheric;
  uint16_t particles_0_3um;
  uint16_t particles_0_5um;
  uint16_t particles_1_0um;
  uint16_t particles_2_5um;
  uint16_t particles_5_0um;
  uint16_t particles_10um;
  bool valid;
};

struct C8CO2Data {
  uint16_t co2_ppm;        // CO₂ concentration in ppm (from bytes 4-5)
  uint16_t co2_ppm_alt;    // Alternative CO₂ reading (from bytes 6-7)
  uint8_t status;          // Sensor status
  uint16_t checksum;       // Packet checksum
  bool valid;              // Data validity flag
};

// ==========================
// SENSOR CALIBRATION DATA STRUCTURES
// ==========================
// Multi-Sensor Calibration State Management with Method Tracking
struct SensorCalibrationState {
  bool isCalibrating = false;           // True if sensor is in calibration period
  unsigned long calibrationStartTime = 0; // When calibration began (millis())
  unsigned long calibrationPeriodHours = 0; // How long this sensor needs to calibrate
  bool dataSuppressionActive = false;   // True if we're suppressing data for this sensor
  
  // NEW: Calibration method tracking for data quality filtering
  String calibrationMethod = "unknown";     // "factory", "frc_manual", "asc_learning", "asc_established", etc.
  String calibrationConfidence = "unknown"; // "low", "medium", "high"
  unsigned long calibrationTimestamp = 0;   // Unix timestamp when this calibration was established
  String calibrationReference = "";          // Additional info (e.g., "420ppm_outdoor" for FRC)
};

struct AllSensorCalibrationStates {
  SensorCalibrationState scd4x;
  SensorCalibrationState scd30;         // SCD30 CO2 sensor (7 days for ASC like SCD4x)
  SensorCalibrationState cm1106c;       // CM1106-C CO2 sensor (15 days for ABC)
  SensorCalibrationState c8co2;         // CM1106-C UART sensor (384h ABC calibration)
  SensorCalibrationState sgp41;
  SensorCalibrationState bme680;
  SensorCalibrationState pms5003;
  SensorCalibrationState bmp280;
  SensorCalibrationState sht4x;
  SensorCalibrationState aht20;
  SensorCalibrationState tmp117;        // TMP117 high-precision temperature sensor
  bool globalTestingMode = false;       // True if user has enabled global testing mode
};

// EEPROM backup structure for critical calibration data persistence
// Ultra-robust design with multiple validation layers
struct CalibrationBackup {
  uint32_t magic1;           // Primary magic number
  uint32_t sequenceNumber;   // Incremental write counter
  unsigned long saveTime;
  struct {
    bool isCalibrating;
    unsigned long unixStartTime;
    unsigned long periodHours;
    bool dataSuppressionActive;
    uint32_t sensorChecksum; // Per-sensor validation
  } sensors[6]; // scd4x, scd30, cm1106c, sgp41, bme680, pms5003
  uint32_t dataChecksum;     // Data integrity check
  uint32_t magic2;           // Trailing magic number

  // Constructor with stronger magic numbers
  CalibrationBackup() : magic1(0xDEADBEEF), sequenceNumber(0), dataChecksum(0), magic2(0xCAFEBABE) {}
};

// ==========================
// SYSLOG CLIENT
// ==========================
// RFC 5424 compliant UDP syslog client with UTC timestamps
// Sends logs to external syslog server for persistent history

// Syslog severity levels (RFC 5424)
enum SyslogSeverity {
  SYSLOG_EMERG   = 0,  // System is unusable
  SYSLOG_ALERT   = 1,  // Action must be taken immediately
  SYSLOG_CRIT    = 2,  // Critical conditions
  SYSLOG_ERR     = 3,  // Error conditions
  SYSLOG_WARNING = 4,  // Warning conditions
  SYSLOG_NOTICE  = 5,  // Normal but significant
  SYSLOG_INFO    = 6,  // Informational
  SYSLOG_DEBUG   = 7   // Debug-level messages
};

class SyslogClient {
private:
  WiFiUDP udp;
  IPAddress serverIP;
  int serverPort;
  bool enabled;
  bool initialized;
  String hostname;
  String appName;

  // Facility 1 = user-level messages
  static const int FACILITY = 1;

  // Get UTC timestamp in RFC 5424 format: 2024-01-05T14:30:00Z
  String getUTCTimestamp() {
    time_t now;
    time(&now);
    struct tm* utc = gmtime(&now);

    if (utc == nullptr || now < 1700000000) {
      // Time not yet synced, use uptime
      return String(millis() / 1000) + "s-uptime";
    }

    char timestamp[32];
    snprintf(timestamp, sizeof(timestamp), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             utc->tm_year + 1900, utc->tm_mon + 1, utc->tm_mday,
             utc->tm_hour, utc->tm_min, utc->tm_sec);
    return String(timestamp);
  }

public:
  SyslogClient() : serverPort(514), enabled(false), initialized(false) {}

  void begin(const char* server, int port, const char* host, const char* app) {
    if (!serverIP.fromString(server)) {
      Serial.println("Syslog: Invalid server IP address");
      enabled = false;
      initialized = false;
      return;
    }

    serverPort = port;
    hostname = String(host);
    appName = String(app);
    initialized = true;

    // Enable based on compile-time config (can be changed at runtime via MQTT)
    enabled = ENABLE_SYSLOG;

    if (enabled) {
      // Send startup message
      log(SYSLOG_NOTICE, "Syslog client initialized");
    } else {
      Serial.println("Syslog: Initialized but disabled (use MQTT command to enable)");
    }
  }

  void log(SyslogSeverity severity, const String& message) {
    if (!enabled || !initialized || WiFi.status() != WL_CONNECTED) {
      return;
    }

    // Calculate priority: facility * 8 + severity
    int priority = FACILITY * 8 + severity;

    // Build RFC 5424 message:
    // <priority>version timestamp hostname app-name procid msgid msg
    String syslogMsg = "<" + String(priority) + ">1 " +
                       getUTCTimestamp() + " " +
                       hostname + " " +
                       appName + " - - - " +
                       message;

    // Send UDP packet
    if (udp.beginPacket(serverIP, serverPort)) {
      udp.write((const uint8_t*)syslogMsg.c_str(), syslogMsg.length());
      udp.endPacket();
    }
  }

  // Convenience methods
  void info(const String& message) { log(SYSLOG_INFO, message); }
  void warning(const String& message) { log(SYSLOG_WARNING, message); }
  void error(const String& message) { log(SYSLOG_ERR, message); }
  void notice(const String& message) { log(SYSLOG_NOTICE, message); }
  void debug(const String& message) { log(SYSLOG_DEBUG, message); }

  // Log with heap info
  void logWithHeap(SyslogSeverity severity, const String& message) {
    if (SYSLOG_INCLUDE_HEAP) {
      log(severity, message + " [heap:" + String(ESP.getFreeHeap()) + "]");
    } else {
      log(severity, message);
    }
  }

  bool isEnabled() { return enabled && initialized; }

  // Runtime enable/disable (for MQTT command)
  void setEnabled(bool state) {
    if (!initialized) {
      Serial.println("Syslog: Cannot enable - not initialized (check server IP)");
      return;
    }
    enabled = state;
    if (enabled) {
      log(SYSLOG_NOTICE, "Syslog enabled via MQTT command");
    }
    Serial.println(enabled ? "Syslog: Enabled" : "Syslog: Disabled");
  }

  // Reinitialize with new server (for future use)
  void setServer(const char* server, int port) {
    if (!serverIP.fromString(server)) {
      Serial.println("Syslog: Invalid server IP address");
      return;
    }
    serverPort = port;
    initialized = true;
    Serial.print("Syslog: Server updated to ");
    Serial.print(server);
    Serial.print(":");
    Serial.println(port);
  }
};

// Global syslog client instance
SyslogClient syslog;

// ==========================
// SECURE NETWORK SERIAL MONITOR
// ==========================

class SecureTelnetMonitor {
public:
  WiFiClient client;  // Made public for telnet forwarding

private:
  WiFiServer server;
  bool authenticated = false;
  bool enabled = false;
  unsigned long connectionTime = 0;
  unsigned long lastActivity = 0;
  String inputBuffer = "";

  // Ring buffer for output - holds data during temporary slowdowns
  // ESP32-C3 has less RAM, so use smaller buffer
  #ifdef CONFIG_IDF_TARGET_ESP32C3
    static const size_t OUTPUT_BUFFER_SIZE = 2048;  // 2KB for C3
  #else
    static const size_t OUTPUT_BUFFER_SIZE = 8192;  // 8KB for other ESP32 variants
  #endif
  char outputBuffer[OUTPUT_BUFFER_SIZE];
  volatile size_t bufferHead = 0;  // Write position
  volatile size_t bufferTail = 0;  // Read position
  unsigned long lastBufferWarning = 0;
  bool bufferOverflowWarned = false;

  // Security settings
  unsigned long maxConnectionTime = TELNET_MAX_CONNECTION_TIME_MS;  // Configurable max connection time
  unsigned long idleTimeout = TELNET_IDLE_TIMEOUT_MS;             // Configurable idle timeout
  static const unsigned long AUTH_TIMEOUT = 30000;                // 30 seconds to authenticate
  static const int MAX_FAILED_ATTEMPTS = 3;
  static const int LOCKOUT_TIME = 300000;                  // 5 minute lockout

  int failedAttempts = 0;
  unsigned long lockoutUntil = 0;
  String allowedIP = "";
  String password = "";

  // Memory recovery tracking
  bool memoryRecoveryMode = false;
  unsigned long memoryDisconnectTime = 0;
  static const unsigned long MEMORY_RECOVERY_COOLDOWN = 10000;  // 10 second cooldown
  static const uint32_t MEMORY_RECOVERY_THRESHOLD = 50000;      // Need 50KB free to accept new connections

  // Ring buffer helper methods
  size_t bufferUsed() {
    if (bufferHead >= bufferTail) {
      return bufferHead - bufferTail;
    }
    return OUTPUT_BUFFER_SIZE - bufferTail + bufferHead;
  }

  size_t bufferFree() {
    return OUTPUT_BUFFER_SIZE - bufferUsed() - 1;  // -1 to distinguish full from empty
  }

  void bufferWrite(char c) {
    size_t nextHead = (bufferHead + 1) % OUTPUT_BUFFER_SIZE;
    if (nextHead != bufferTail) {  // Not full
      outputBuffer[bufferHead] = c;
      bufferHead = nextHead;
      bufferOverflowWarned = false;
    } else {
      // Buffer full - log warning occasionally
      if (!bufferOverflowWarned || millis() - lastBufferWarning > 30000) {
        Serial.println("[Telnet] Output buffer full - oldest data being overwritten");
        lastBufferWarning = millis();
        bufferOverflowWarned = true;
      }
      // Overwrite oldest data (advance tail)
      bufferTail = (bufferTail + 1) % OUTPUT_BUFFER_SIZE;
      outputBuffer[bufferHead] = c;
      bufferHead = nextHead;
    }
  }

  // Read up to 'len' bytes from buffer into 'dest', returns actual bytes read
  size_t bufferRead(char* dest, size_t len) {
    size_t bytesRead = 0;
    while (bytesRead < len && bufferTail != bufferHead) {
      dest[bytesRead++] = outputBuffer[bufferTail];
      bufferTail = (bufferTail + 1) % OUTPUT_BUFFER_SIZE;
    }
    return bytesRead;
  }
  
public:
  SecureTelnetMonitor(int port) : server(port) {}
  
  void begin(const char* pwd, const char* restrictIP = "") {
    if (!ENABLE_SECURE_TELNET) return;
    
    password = String(pwd);
    allowedIP = String(restrictIP);
    enabled = true;
    server.begin();
    server.setNoDelay(true);
    
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("Secure telnet monitor started on port " + String(TELNET_PORT));
      Serial.println("Authentication required for access");
      Serial.print("Connect: telnet ");
      Serial.print(WiFi.localIP());
      Serial.println(" " + String(TELNET_PORT));
      if (allowedIP != "") {
        Serial.println("Access restricted to IP: " + allowedIP);
      }
    }
  }
  
  void handle() {
    if (!enabled) return;
    
    // Check lockout period
    if (lockoutUntil > 0 && millis() < lockoutUntil) {
      if (server.hasClient()) {
        WiFiClient tempClient = server.available();
        tempClient.println("Access temporarily locked due to failed attempts");
        tempClient.stop();
      }
      return;
    } else if (lockoutUntil > 0) {
      lockoutUntil = 0;
      failedAttempts = 0;
    }

    // Check memory recovery after emergency disconnect
    if (memoryRecoveryMode) {
      uint32_t currentHeap = ESP.getFreeHeap();
      unsigned long timeSinceDisconnect = millis() - memoryDisconnectTime;

      if (timeSinceDisconnect >= MEMORY_RECOVERY_COOLDOWN && currentHeap >= MEMORY_RECOVERY_THRESHOLD) {
        // Memory has recovered - allow new connections
        memoryRecoveryMode = false;
        if (ENABLE_DEBUG_OUTPUT) {
          Serial.println("[Telnet] Memory recovered to " + String(currentHeap) + " bytes - accepting connections again");
        }
      } else if (server.hasClient()) {
        // Still recovering - reject new connections with status
        WiFiClient tempClient = server.available();
        tempClient.println("Telnet temporarily unavailable - memory recovery in progress");
        tempClient.println("  Current heap: " + String(currentHeap) + " bytes (need " + String(MEMORY_RECOVERY_THRESHOLD) + ")");
        tempClient.println("  Cooldown: " + String(max(0L, (long)(MEMORY_RECOVERY_COOLDOWN - timeSinceDisconnect) / 1000)) + "s remaining");
        tempClient.println("Please try again shortly.");
        tempClient.stop();
        return;
      } else {
        return;  // Still in recovery mode, no client waiting
      }
    }

    // Handle new connections
    if (server.hasClient()) {
      if (client && client.connected()) {
        WiFiClient newClient = server.available();
        newClient.println("Only one connection allowed at a time");
        newClient.stop();
        return;
      }
      
      client = server.available();
      String clientIP = client.remoteIP().toString();
      
      // IP restriction check
      if (allowedIP != "" && clientIP != allowedIP) {
        if (ENABLE_DEBUG_OUTPUT) {
          Serial.println("Telnet connection rejected from unauthorized IP: " + clientIP);
        }
        client.println("Access denied - unauthorized IP address");
        client.stop();
        return;
      }
      
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("Telnet connection from: " + clientIP);
      }
      connectionTime = millis();
      lastActivity = millis();
      authenticated = false;
      inputBuffer = "";
      
      client.println("ESP32 Secure Serial Monitor");
      client.println("Authentication required");
      client.print("Password: ");
    }
    
    // Handle existing connection
    if (client && client.connected()) {
      // Check timeouts
      if (millis() - connectionTime > maxConnectionTime) {
        client.println("\nMaximum connection time exceeded");
        disconnectClient("Max connection time");
        return;
      }

      if (millis() - lastActivity > idleTimeout) {
        client.println("\nConnection timed out due to inactivity");
        disconnectClient("Idle timeout");
        return;
      }

      // Memory safety check - disconnect if heap gets critically low
      // This protects the WiFi stack which needs ~40KB to operate reliably
      uint32_t freeHeap = ESP.getFreeHeap();
      if (freeHeap < 30000) {  // Less than 30KB free
        client.println("\nDisconnecting due to low memory (" + String(freeHeap) + " bytes)");
        client.println("Will accept reconnection once memory recovers above " + String(MEMORY_RECOVERY_THRESHOLD) + " bytes");
        disconnectClient("Low memory protection");
        // Enter recovery mode - wait for heap to recover before accepting new connections
        memoryRecoveryMode = true;
        memoryDisconnectTime = millis();
        if (ENABLE_DEBUG_OUTPUT) {
          Serial.println("[Telnet] Emergency disconnect - heap critically low: " + String(freeHeap));
          Serial.println("[Telnet] Entering recovery mode - will accept connections when heap > " + String(MEMORY_RECOVERY_THRESHOLD));
        }
        syslog.error("HEAP_CRITICAL heap=" + String(freeHeap) + " telnet_disconnected");
        return;
      }

      // Handle incoming data
      while (client.available()) {
        char c = client.read();
        lastActivity = millis();
        
        if (!authenticated) {
          handleAuthentication(c);
        } else {
          handleAuthenticatedInput(c);
        }
      }
      
      // Forward Serial output to authenticated client using ring buffer
      if (authenticated) {
        // Step 1: Read all available Serial data into ring buffer
        // This ensures we capture data even if we can't send it immediately
        while (Serial.available()) {
          char c = Serial.read();
          bufferWrite(c);
        }

        // Step 2: Send buffered data to client in efficient batches
        // Use a local buffer for batch writes (more efficient than char-by-char)
        char sendBatch[256];
        size_t totalSent = 0;
        const size_t MAX_SEND_PER_CYCLE = 1024;  // Limit per handle() call

        while (bufferUsed() > 0 && totalSent < MAX_SEND_PER_CYCLE) {
          // Check how much the client can accept
          size_t canWrite = client.availableForWrite();
          if (canWrite < 1) {
            // TCP buffer full - stop sending, data stays in ring buffer for next cycle
            break;
          }

          // Read a batch from ring buffer
          size_t batchSize = min(canWrite, sizeof(sendBatch));
          batchSize = min(batchSize, MAX_SEND_PER_CYCLE - totalSent);
          size_t actualRead = bufferRead(sendBatch, batchSize);

          if (actualRead > 0) {
            // Write batch to client (more efficient than char-by-char)
            size_t written = client.write((uint8_t*)sendBatch, actualRead);
            totalSent += written;

            // If we couldn't write everything, put unwritten data back
            // (This shouldn't happen if availableForWrite() is accurate, but just in case)
            if (written < actualRead) {
              // Can't easily put data back in ring buffer, so we lose it
              // But this should be rare with proper flow control
              break;
            }
          }
        }

        // Yield to allow WiFi stack maintenance after sending
        if (totalSent > 0) {
          yield();
        }

        // Periodic buffer status (only when buffer is getting full)
        static unsigned long lastBufferStatus = 0;
        size_t used = bufferUsed();
        if (used > OUTPUT_BUFFER_SIZE * 3 / 4 && millis() - lastBufferStatus > 5000) {
          // Buffer is >75% full, warn user
          String warning = "\n[Buffer " + String(used * 100 / OUTPUT_BUFFER_SIZE) + "% full]\n";
          client.print(warning);
          lastBufferStatus = millis();
        }
      }
    }
  }

private:
  void handleAuthentication(char c) {
    if (c == '\r' || c == '\n') {
      if (inputBuffer.equals(password)) {
        authenticated = true;
        client.println("\nAuthentication successful");
        client.println("Serial monitor active - type 'quit' to exit");
        client.println("----------------------------------------");
        if (ENABLE_DEBUG_OUTPUT) {
          Serial.println("Telnet client authenticated successfully");
        }
        failedAttempts = 0;
      } else {
        failedAttempts++;
        client.println("\nAuthentication failed");
        if (ENABLE_DEBUG_OUTPUT) {
          Serial.println("Telnet authentication failed from " + client.remoteIP().toString());
        }
        
        if (failedAttempts >= MAX_FAILED_ATTEMPTS) {
          client.println("Too many failed attempts - access locked");
          lockoutUntil = millis() + LOCKOUT_TIME;
          if (ENABLE_DEBUG_OUTPUT) {
            Serial.println("Telnet access locked due to failed attempts");
          }
          disconnectClient("Max failed attempts");
          return;
        }
        
        client.print("Password (" + String(MAX_FAILED_ATTEMPTS - failedAttempts) + " attempts remaining): ");
      }
      inputBuffer = "";
    } else if (c == 8 || c == 127) { // Backspace
      if (inputBuffer.length() > 0) {
        inputBuffer.remove(inputBuffer.length() - 1);
      }
    } else if (c >= 32 && c <= 126) { // Printable characters only
      inputBuffer += c;
      if (inputBuffer.length() > 50) {
        client.println("\nInput too long");
        disconnectClient("Input overflow");
        return;
      }
    }
    
    if (!authenticated && millis() - connectionTime > AUTH_TIMEOUT) {
      client.println("\nAuthentication timeout");
      disconnectClient("Auth timeout");
    }
  }
  
  void handleAuthenticatedInput(char c) {
    if (c == '\r' || c == '\n') {
      inputBuffer.trim();
      if (inputBuffer.equals("quit") || inputBuffer.equals("exit")) {
        client.println("Goodbye!");
        disconnectClient("User quit");
      } else if (inputBuffer.equals("help")) {
        client.println("Available commands:");
        client.println("  quit/exit - Disconnect");
        client.println("  help      - Show this help");
        client.println("  status    - Show system status");
      } else if (inputBuffer.equals("status")) {
        client.println("ESP32 System Status:");
        client.println("  Uptime: " + String(millis() / 1000) + " seconds");
        client.println("  Free heap: " + String(ESP.getFreeHeap()) + " bytes");
        client.println("  Min free heap: " + String(ESP.getMinFreeHeap()) + " bytes");
        client.println("  Heap fragmentation: ~" + String(100 - (ESP.getMaxAllocHeap() * 100 / ESP.getFreeHeap())) + "%");
        client.println("  WiFi RSSI: " + String(WiFi.RSSI()) + " dBm");
        client.println("  WiFi status: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected"));
        client.println("  Connection time: " + String((millis() - connectionTime) / 1000) + " seconds");
        client.println("  TCP write buffer: " + String(client.availableForWrite()) + " bytes available");
        client.println("  Output buffer: " + String(bufferUsed()) + "/" + String(OUTPUT_BUFFER_SIZE) + " bytes (" + String(bufferUsed() * 100 / OUTPUT_BUFFER_SIZE) + "%)");
      } else if (inputBuffer.length() > 0) {
        client.println("Unknown command: " + inputBuffer);
        client.println("Type 'help' for available commands");
      }
      inputBuffer = "";
    } else if (c == 8 || c == 127) {
      if (inputBuffer.length() > 0) {
        inputBuffer.remove(inputBuffer.length() - 1);
      }
    } else if (c >= 32 && c <= 126) {
      inputBuffer += c;
      if (inputBuffer.length() > 20) {
        client.println("\nCommand too long");
        inputBuffer = "";
      }
    }
  }
  
  void disconnectClient(String reason) {
    if (client) {
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("Telnet client disconnected: " + reason);
      }
      client.stop();
    }
    authenticated = false;
    inputBuffer = "";
    // Clear output buffer for fresh start on next connection
    bufferHead = 0;
    bufferTail = 0;
    bufferOverflowWarned = false;
  }
  
public:
  bool isConnected() {
    return client && client.connected() && authenticated;
  }
  
  void disable() {
    enabled = false;
    if (client && client.connected()) {
      client.println("Serial monitor disabled");
      client.stop();
    }
    server.end();
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("Secure telnet monitor disabled");
    }
  }
  
  // Helper methods to send output to both Serial and telnet
  void println(const String& message) {
    Serial.println(message);
    if (isConnected()) {
      client.println(message);
    }
  }
  
  void print(const String& message) {
    Serial.print(message);
    if (isConnected()) {
      client.print(message);
    }
  }
  
  
};

// ==========================
// LORAWAN DEFINITIONS & RADIO OBJECTS
// ==========================

// LoRaWAN region configuration
#if defined(LORAWAN_REGION_AU915)
    #define LORAWAN_REGION_NAME "AU915"
#elif defined(LORAWAN_REGION_EU868)
    #define LORAWAN_REGION_NAME "EU868"
#elif defined(LORAWAN_REGION_US915)
    #define LORAWAN_REGION_NAME "US915"
#elif defined(LORAWAN_REGION_AS923)
    #define LORAWAN_REGION_NAME "AS923"
#elif defined(LORAWAN_REGION_IN865)
    #define LORAWAN_REGION_NAME "IN865"
#elif defined(LORAWAN_REGION_KR920)
    #define LORAWAN_REGION_NAME "KR920"
#elif defined(LORAWAN_REGION_CN470)
    #define LORAWAN_REGION_NAME "CN470"
#else
    #define LORAWAN_REGION_NAME "AU915 (Auto-detected)"
#endif

// RadioLib radio object pointers (will be initialized based on board type)
SX1262* radio_sx1262 = nullptr;
SX1276* radio_sx1276 = nullptr;
LoRaWANNode* node = nullptr;

// LoRaWAN transmission timing
const unsigned long TX_INTERVAL_MS = LORAWAN_TX_INTERVAL_MS; // User-configurable transmission interval
const unsigned long JOIN_RETRY_INTERVAL_MS = 30000; // 30 seconds between join attempts
unsigned long lastLoRaWANTransmission = 0;
unsigned long lastJoinAttempt = 0;
bool lorawanJoined = false;
bool joinAccepted = false;  // Track if we've confirmed join acceptance

// LoRaWAN payload buffer
// AU915 with dwell time (400ms): DR2=11, DR3=53, DR4=125, DR5=222 bytes
// We use DR5 (SF7) which allows up to 222 bytes - requires good signal
static uint8_t lorawanPayload[222]; // Max payload for DR5 AU915 with dwell time
static uint8_t lorawanPayloadSize = 0;

// LoRaWAN debug/status tracking
static unsigned long lorawan_tx_count = 0;
static unsigned long lorawan_tx_success = 0;
static unsigned long lorawan_tx_failed = 0;
static int lorawan_last_error = 0;
static bool lorawan_transmitting = false;
static bool lorawan_receiving = false;
static int lorawan_consecutive_failures = 0;

// LoRaWAN metadata transmission tracking
// Metadata (location, node_name, board_type) is sent on boot and hourly
static unsigned long lastMetadataTransmission = 0;
static bool metadataSentAfterJoin = false;
static const unsigned long METADATA_INTERVAL_MS = 3600000;  // Send metadata every hour

// Display screen tracking (global for button navigation)
static int currentDisplayScreen = 0;
static unsigned long lastScreenSwitchTime = 0;  // When screen was last switched (for nav bar auto-hide)
static bool showNavigationBar = false;           // Whether nav bar is currently visible

// TX/RX activity indicators (for header display)
static unsigned long lastMqttTxTime = 0;         // Last MQTT publish time
static unsigned long lastMqttRxTime = 0;         // Last MQTT receive time
static unsigned long lastLoraTxTime = 0;         // Last LoRa transmit time
static unsigned long lastLoraRxTime = 0;         // Last LoRa receive time
static const unsigned long ACTIVITY_INDICATOR_MS = 2000;  // Show indicator for 2 seconds

// Display scrolling state (for long text and content overflow)
static const unsigned long SCROLL_WAIT_MS = 2000;        // Wait time at start/end of scroll
static const unsigned long SCROLL_STEP_MS = 150;         // Time between scroll steps
static const int DISPLAY_CHAR_WIDTH = 6;                 // Pixels per character (text size 1)
static const int DISPLAY_MAX_CHARS = 21;                 // Max chars that fit on screen (128/6)
static const int CONTENT_AREA_HEIGHT = 45;               // y=18 to y=63 (46 pixels, ~5 lines)
static const int LINE_HEIGHT = 8;                        // Pixels per line (text size 1)

// Horizontal scroll state (node name)
enum ScrollState { SCROLL_WAIT_START, SCROLL_MOVING, SCROLL_WAIT_END };
static ScrollState hScrollState = SCROLL_WAIT_START;
static int hScrollOffset = 0;                            // Current character offset
static unsigned long hScrollTimer = 0;                   // Timer for scroll state changes
static int hScrollMaxOffset = 0;                         // Max scroll offset for current text

// Vertical scroll state (content area)
static ScrollState vScrollState = SCROLL_WAIT_START;
static int vScrollOffset = 0;                            // Current pixel offset
static unsigned long vScrollTimer = 0;                   // Timer for scroll state changes
static int vScrollMaxOffset = 0;                         // Max scroll offset for current content
static int lastDisplayScreenForScroll = -1;              // Track screen changes to reset scroll

// ==========================
// DEEP SLEEP RTC MEMORY
// ==========================
// Data that survives deep sleep (stored in 8KB RTC memory)
// Note: This memory is zeroed on cold boot, preserved during deep sleep

RTC_DATA_ATTR uint32_t rtc_bootCount = 0;            // Increments on each wake
RTC_DATA_ATTR uint32_t rtc_wakeCount = 0;            // Wakes since cold boot
RTC_DATA_ATTR uint32_t rtc_lastTxTimestamp = 0;      // Unix timestamp of last TX
RTC_DATA_ATTR bool rtc_deepSleepActive = false;      // True if we entered deep sleep

// GPS coordinate cache (avoid GPS fix every wake)
RTC_DATA_ATTR float rtc_lastGpsLat = 0.0f;
RTC_DATA_ATTR float rtc_lastGpsLon = 0.0f;
RTC_DATA_ATTR float rtc_lastGpsAlt = 0.0f;
RTC_DATA_ATTR uint32_t rtc_lastGpsSyncTime = 0;

// LoRaWAN session state (RadioLib buffer format)
// Session in RTC (can be lost - will rejoin), nonces in NVS (must survive power loss)
RTC_DATA_ATTR uint8_t rtc_lorawanSession[384];       // RadioLib session buffer
RTC_DATA_ATTR bool rtc_lorawanSessionValid = false;  // Is session buffer valid?

// ==========================
// SENSOR HIERARCHY SYSTEM
// ==========================
// Prevents duplicate measurements from being published while maintaining sensor functionality
// Priority: 1 = highest accuracy, 10 = lowest accuracy

struct SensorReading {
  float value;
  String sensor_name;
  int priority;  // Lower number = higher priority (1 = best)
  bool valid;
  uint32_t timestamp;  // Unix timestamp when this reading was captured
};

// Current best readings for each measurement type
struct {
  // Environmental sensors
  SensorReading temperature = {0, "", 99, false, 0};
  SensorReading humidity = {0, "", 99, false, 0};
  SensorReading pressure = {0, "", 99, false, 0};
  SensorReading co2 = {0, "", 99, false, 0};
  SensorReading voc = {0, "", 99, false, 0};
  SensorReading nox = {0, "", 99, false, 0};
  // Particulate matter (PMS5003)
  SensorReading pm1 = {0, "", 99, false, 0};
  SensorReading pm2_5 = {0, "", 99, false, 0};
  SensorReading pm10 = {0, "", 99, false, 0};
  // Power monitoring (INA219, PMU)
  SensorReading voltage = {0, "", 99, false, 0};
  SensorReading current = {0, "", 99, false, 0};
  SensorReading power = {0, "", 99, false, 0};
  SensorReading battery = {0, "", 99, false, 0};
  // Gas resistance (BME680)
  SensorReading gas_resistance = {0, "", 99, false, 0};
} currentBestReadings;

// Timestamp of the last sensor read cycle (for LoRaWAN single-timestamp mode)
uint32_t lastSensorReadCycleTimestamp = 0;

// ==========================
// LORAWAN INITIALIZATION & FUNCTIONS
// ==========================

void initializeLoRaWAN() {
  Serial.println(F("----------------------------------"));
  Serial.print(F("Initialising LoRaWAN for region: ")); Serial.println(LORAWAN_REGION_NAME);
  Serial.flush();

  // Initialise SPI for LoRa
  Serial.print(F("LoRaWAN: Initialising SPI (SCK=")); Serial.print(g_boardConfig.lora_sck);
  Serial.print(F(", MISO=")); Serial.print(g_boardConfig.lora_miso); 
  Serial.print(F(", MOSI=")); Serial.print(g_boardConfig.lora_mosi); Serial.println(F(")..."));
  Serial.flush();
  SPI.begin(g_boardConfig.lora_sck, g_boardConfig.lora_miso, g_boardConfig.lora_mosi);
  Serial.println(F("LoRaWAN: SPI Initialised."));
  Serial.flush();

  int radioState;
  
  // Create the appropriate radio object based on board type
  #ifdef CONFIG_IDF_TARGET_ESP32S3
    // ESP32-S3 boards use SX1262
    Serial.println(F("LoRaWAN: Initialising SX1262 radio for ESP32-S3..."));
    radio_sx1262 = new SX1262(new Module(g_boardConfig.lora_nss, g_boardConfig.lora_dio1, g_boardConfig.lora_rst, g_boardConfig.lora_busy));
    
    // CRITICAL: T3 S3 V1.2 requires TCXO voltage in begin() for proper calibration
    // Parameters: freq, bw, sf, cr, syncWord, power, preambleLength, tcxoVoltage
    radioState = radio_sx1262->begin(915.0, 125.0, 9, 7, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 22, 8, 1.8);
    if (radioState == RADIOLIB_ERR_NONE) {
      Serial.println(F("LoRaWAN: SX1262 radio initialised successfully with TCXO!"));
      
      // Configure DIO2 as RF switch for T3 S3 V1.2
      radioState = radio_sx1262->setDio2AsRfSwitch(true);
      if (radioState != RADIOLIB_ERR_NONE) {
        Serial.print(F("LoRaWAN: DIO2 RF switch config failed, code: ")); Serial.println(radioState);
      }
    } else {
      Serial.print(F("LoRaWAN: SX1262 radio initialisation failed, code: ")); Serial.println(radioState);
      return;
    }
    
    // Create LoRaWAN node with SX1262
    // TTN uses FSB2 (sub-band 2, channels 8-15) for AU915 and US915
    #if defined(LORAWAN_REGION_EU868)
      node = new LoRaWANNode(radio_sx1262, &EU868);
      Serial.println(F("LoRaWAN: Region EU868 configured"));
    #elif defined(LORAWAN_REGION_US915)
      node = new LoRaWANNode(radio_sx1262, &US915, 2);  // FSB2 for TTN
      Serial.println(F("LoRaWAN: Region US915 configured with FSB2 (TTN)"));
    #elif defined(LORAWAN_REGION_AS923)
      node = new LoRaWANNode(radio_sx1262, &AS923);
      Serial.println(F("LoRaWAN: Region AS923 configured"));
    #else
      node = new LoRaWANNode(radio_sx1262, &AU915, 2);  // FSB2 for TTN
      Serial.println(F("LoRaWAN: Region AU915 configured with FSB2 (TTN)"));
    #endif
  #else
    // Other ESP32 boards use SX1276
    Serial.println(F("LoRaWAN: Initialising SX1276 radio..."));
    radio_sx1276 = new SX1276(new Module(g_boardConfig.lora_nss, g_boardConfig.lora_dio0, g_boardConfig.lora_rst, g_boardConfig.lora_dio1));
    
    radioState = radio_sx1276->begin();
    if (radioState == RADIOLIB_ERR_NONE) {
      Serial.println(F("LoRaWAN: SX1276 radio initialised successfully!"));
    } else {
      Serial.print(F("LoRaWAN: SX1276 radio initialisation failed, code: ")); Serial.println(radioState);
      return;
    }
    
    // Create LoRaWAN node with SX1276
    // TTN uses FSB2 (sub-band 2, channels 8-15) for AU915 and US915
    #if defined(LORAWAN_REGION_EU868)
      node = new LoRaWANNode(radio_sx1276, &EU868);
      Serial.println(F("LoRaWAN: Region EU868 configured"));
    #elif defined(LORAWAN_REGION_US915)
      node = new LoRaWANNode(radio_sx1276, &US915, 2);  // FSB2 for TTN
      Serial.println(F("LoRaWAN: Region US915 configured with FSB2 (TTN)"));
    #elif defined(LORAWAN_REGION_AS923)
      node = new LoRaWANNode(radio_sx1276, &AS923);
      Serial.println(F("LoRaWAN: Region AS923 configured"));
    #else
      node = new LoRaWANNode(radio_sx1276, &AU915, 2);  // FSB2 for TTN
      Serial.println(F("LoRaWAN: Region AU915 configured with FSB2 (TTN)"));
    #endif
  #endif

  Serial.println(F("LoRaWAN: Starting OTAA join..."));
  Serial.flush();

  // Copy credentials from PROGMEM and convert to uint64_t
  uint8_t joinEUIBuff[8], devEUIBuff[8], appKey[16], nwkKey[16];
  memcpy_P(joinEUIBuff, USER_APPEUI, 8);
  memcpy_P(devEUIBuff, USER_DEVEUI, 8);
  memcpy_P(appKey, USER_APPKEY, 16);
  memcpy_P(nwkKey, USER_APPKEY, 16); // For LoRaWAN 1.0.x, NwkKey = AppKey

  // Convert byte arrays to uint64_t (MSB first)
  uint64_t joinEUI = 0;
  uint64_t devEUI = 0;
  for (int i = 0; i < 8; i++) {
    joinEUI = (joinEUI << 8) | joinEUIBuff[i];
    devEUI = (devEUI << 8) | devEUIBuff[i];
  }

  // Setup OTAA credentials (doesn't send join request yet)
  Serial.println(F("LoRaWAN: Configuring OTAA credentials..."));
  node->beginOTAA(joinEUI, devEUI, nwkKey, appKey);
  Serial.println(F("LoRaWAN: Credentials configured"));

  // === DEEP SLEEP SESSION RESTORE ===
  // If waking from deep sleep with valid session, restore it instead of rejoining
  if (ENABLE_DEEP_SLEEP && isWakeFromDeepSleep() && rtc_lorawanSessionValid) {
    Serial.println(F("LoRaWAN: Attempting to restore session from deep sleep..."));
    if (restoreLoRaWANSession()) {
      // Session restored successfully - skip OTAA join in loop
      lorawanJoined = true;
      joinAccepted = true;
      // Set last TX to 0 to trigger immediate transmission after warmup
      lastLoRaWANTransmission = 0;
      Serial.println(F("LoRaWAN: Session restored - will transmit after sensor warmup"));

      // Re-apply data rate settings
      node->setADR(false);
      int drState = node->setDatarate(5);
      if (drState == RADIOLIB_ERR_NONE) {
        Serial.println(F("LoRaWAN: Data rate set to DR5 (SF7, 222 byte max)"));
      }
    } else {
      Serial.println(F("LoRaWAN: Session restore failed - will perform full OTAA join"));
    }
  } else {
    Serial.println(F("LoRaWAN: Join attempts will begin in main loop"));
  }
  Serial.flush();
}

/**
 * Send DeviceMetadataV2 message over LoRaWAN
 * Contains: location, node_name, board_type, firmware_version, calibrations
 * Sent on fPort 2 (readings use fPort 1)
 * Called after join and hourly to populate ingester cache
 */
void sendLoRaWANMetadata() {
  if (node == nullptr || !lorawanJoined) {
    return;
  }

  Serial.println(F("----------------------------------"));
  Serial.println(F("LoRaWAN: Sending device metadata..."));

#if ENABLE_PROTOBUF
  DeviceMetadataEncoderV2 metadataEncoder;
  metadataEncoder.begin();

  // TODO: Add sensor calibration status if available
  // metadataEncoder.addCalibration(sensor, status, method, date);

  // Encode metadata
  uint8_t metadataBuffer[128];
  size_t metadataSize = metadataEncoder.encode(metadataBuffer, sizeof(metadataBuffer));

  if (metadataSize == 0) {
    Serial.println(F("LoRaWAN: Metadata encoding failed!"));
    return;
  }

  Serial.printf("LoRaWAN: Metadata encoded (%d bytes)\n", metadataSize);
  Serial.printf("  Node name: %s\n", getNodeName());
  Serial.printf("  Location: %.4f, %.4f, %.0fm\n",
    getLatitude(), getLongitude(), getAltitudeMeters());

  // Send on fPort 2 (metadata) - distinct from fPort 1 (readings)
  lorawan_tx_count++;
  lorawan_transmitting = true;

  int state = node->sendReceive(metadataBuffer, metadataSize, 2);  // fPort 2 = metadata

  lorawan_transmitting = false;
  lorawan_last_error = state;

  if (state >= RADIOLIB_ERR_NONE) {
    Serial.println(F("LoRaWAN: Metadata sent successfully!"));
    lorawan_tx_success++;
    lastLoraTxTime = millis();  // Update TX indicator for display
    lastMetadataTransmission = millis();
    metadataSentAfterJoin = true;
  } else {
    Serial.printf("LoRaWAN: Metadata send failed (error %d)\n", state);
    lorawan_tx_failed++;
  }
#else
  Serial.println(F("LoRaWAN: Protobuf disabled, skipping metadata"));
#endif

  Serial.println(F("----------------------------------"));
}

void sendLoRaWANData() {
  if (node == nullptr) {
    return;
  }

  // If we haven't joined yet, attempt to join
  if (!lorawanJoined) {
    if (millis() - lastJoinAttempt < JOIN_RETRY_INTERVAL_MS) {
      return;  // Wait before trying again
    }
    lastJoinAttempt = millis();
    
    Serial.println(F("----------------------------------"));
    Serial.println(F("LoRaWAN: Attempting OTAA join..."));
    Serial.println(F("LoRaWAN: This may take 30-60 seconds..."));
    
    // Track join attempt
    lorawan_tx_count++;
    lorawan_transmitting = true;
    lorawan_receiving = false;
    
    // Attempt OTAA activation (this sends join request and waits for accept)
    int joinState = node->activateOTAA();
    
    lorawan_transmitting = false;
    lorawan_receiving = true;  // Waiting for join accept
    lorawan_last_error = joinState;
    lorawan_receiving = false;
    
    // Success codes: 0 = ERR_NONE, -1117 = SESSION_RESTORED, -1118 = NEW_SESSION
    if (joinState >= RADIOLIB_ERR_NONE || joinState == -1117 || joinState == -1118) {
      Serial.println(F("LoRaWAN: *** JOIN SUCCESSFUL! ***"));
      Serial.println(F("LoRaWAN: Connected to TTN gateway"));

      // Set data rate to DR5 AFTER joining - allows up to 222 bytes payload
      // AU915 with dwell time: DR2=11, DR3=53, DR4=125, DR5=222 bytes
      node->setADR(false);  // Disable ADR to keep our data rate
      int drState = node->setDatarate(5);
      if (drState == RADIOLIB_ERR_NONE) {
        Serial.println(F("LoRaWAN: Data rate set to DR5 (SF7, 222 byte max)"));
      } else {
        Serial.print(F("LoRaWAN: Warning - failed to set data rate, code: "));
        Serial.println(drState);
        Serial.println(F("LoRaWAN: Will use network-assigned rate (may limit payload size)"));
      }

      lorawan_tx_success++;
      lorawanJoined = true;
      joinAccepted = true;  // Mark as joined and accepted
      // Reset transmission timer to send first data packet immediately
      lastLoRaWANTransmission = millis() - TX_INTERVAL_MS;
      Serial.println(F("LoRaWAN: First data transmission will happen immediately"));
    } else {
      Serial.print(F("LoRaWAN: Join failed, code: ")); Serial.println(joinState);
      Serial.println(F("LoRaWAN: Will retry in 30 seconds..."));
      Serial.println(F("LoRaWAN: Check credentials and gateway coverage"));
      lorawan_tx_failed++;
    }
    Serial.println(F("----------------------------------"));
    Serial.flush();
    return;
  }

  // If we haven't confirmed join yet, test with uplinks
  if (!joinAccepted) {
    // Try to send one packet to test if join was accepted
    if (millis() - lastJoinAttempt < JOIN_RETRY_INTERVAL_MS) {
      return;  // Wait before trying again
    }
    lastJoinAttempt = millis();
    Serial.println(F("LoRaWAN: Testing if join was accepted..."));
  }

  // Check if it's time to send regular uplinks
  if (joinAccepted && (millis() - lastLoRaWANTransmission < TX_INTERVAL_MS)) {
    return;
  }

  // =====================================================
  // METADATA TRANSMISSION (before sensor readings)
  // =====================================================
  // Send metadata on first transmission after join, then hourly
  if (joinAccepted) {
    bool shouldSendMetadata = false;

    if (!metadataSentAfterJoin) {
      Serial.println(F("LoRaWAN: First transmission after join - sending metadata first"));
      shouldSendMetadata = true;
    } else if (millis() - lastMetadataTransmission >= METADATA_INTERVAL_MS) {
      Serial.println(F("LoRaWAN: Hourly metadata update"));
      shouldSendMetadata = true;
    }

    if (shouldSendMetadata) {
      sendLoRaWANMetadata();
      // Small delay before sending readings to avoid duty cycle issues
      delay(1000);
    }
  }

  // =====================================================
  // FRESH SENSOR READ before LoRaWAN transmission
  // =====================================================
  // Read all sensors NOW to ensure we're sending current data
  // This triggers MQTT publishing too (unified read for both transports)
  Serial.println(F("LoRaWAN: Reading sensors before transmission..."));
  uint32_t cycleTimestamp = readAllSensors();

  // Prepare sensor data payload with ALL sensor readings using V2 protobuf
  // V2 packs all sensors into one message: ~35 bytes header + ~9 bytes per sensor
  // Much more efficient than V1 which repeated metadata for each sensor (~100 bytes each)
  lorawanPayloadSize = 0;

#if ENABLE_PROTOBUF
  // Use V2.1 consolidated encoder - all sensors in one message
  // LoRa mode: minimal fields only (device_id, timestamp, measurements)
  // Location and metadata are sent separately via DeviceMetadataV2 on boot/daily
  ProtobufEncoderV2 encoder;
  // Use the cycle timestamp from the sensor read we just performed
  // This ensures the timestamp reflects when data was captured (seconds ago, not now)
  // TRANSPORT_LORA = minimal message for bandwidth efficiency
  encoder.begin(cycleTimestamp, false, TRANSPORT_LORA);

  if (ENABLE_DEBUG_OUTPUT) {
    Serial.println(F("LoRaWAN: Encoding all sensors with V2.1 protobuf (minimal mode)..."));
    Serial.printf("  Using cycle timestamp: %lu\n", cycleTimestamp);
  }

  // Add all valid sensor readings (priority order for if we need to trim)
  if (currentBestReadings.temperature.valid) {
    encoder.addTemperature(currentBestReadings.temperature.value,
                           currentBestReadings.temperature.sensor_name.c_str());
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + Temperature: %.1f°C from %s\n",
        currentBestReadings.temperature.value,
        currentBestReadings.temperature.sensor_name.c_str());
    }
  }

  if (currentBestReadings.humidity.valid) {
    encoder.addHumidity(currentBestReadings.humidity.value,
                        currentBestReadings.humidity.sensor_name.c_str());
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + Humidity: %.1f%% from %s\n",
        currentBestReadings.humidity.value,
        currentBestReadings.humidity.sensor_name.c_str());
    }
  }

  if (currentBestReadings.co2.valid) {
    encoder.addCO2(currentBestReadings.co2.value,
                   currentBestReadings.co2.sensor_name.c_str());
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + CO2: %.0fppm from %s\n",
        currentBestReadings.co2.value,
        currentBestReadings.co2.sensor_name.c_str());
    }
  }

  if (currentBestReadings.pressure.valid) {
    encoder.addPressure(currentBestReadings.pressure.value,
                        currentBestReadings.pressure.sensor_name.c_str());
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + Pressure: %.2f hPa from %s\n",
        currentBestReadings.pressure.value,
        currentBestReadings.pressure.sensor_name.c_str());
    }
  }

  if (currentBestReadings.voc.valid) {
    encoder.addVOC(currentBestReadings.voc.value,
                   currentBestReadings.voc.sensor_name.c_str());
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + VOC: %.0f from %s\n",
        currentBestReadings.voc.value,
        currentBestReadings.voc.sensor_name.c_str());
    }
  }

  if (currentBestReadings.nox.valid) {
    encoder.addNOx(currentBestReadings.nox.value,
                   currentBestReadings.nox.sensor_name.c_str());
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + NOx: %.0f from %s\n",
        currentBestReadings.nox.value,
        currentBestReadings.nox.sensor_name.c_str());
    }
  }

  // Particulate matter (PMS5003)
  if (currentBestReadings.pm1.valid) {
    encoder.addPM1(currentBestReadings.pm1.value,
                   currentBestReadings.pm1.sensor_name.c_str());
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + PM1.0: %.1f μg/m³ from %s\n",
        currentBestReadings.pm1.value,
        currentBestReadings.pm1.sensor_name.c_str());
    }
  }

  if (currentBestReadings.pm2_5.valid) {
    encoder.addPM25(currentBestReadings.pm2_5.value,
                    currentBestReadings.pm2_5.sensor_name.c_str());
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + PM2.5: %.1f μg/m³ from %s\n",
        currentBestReadings.pm2_5.value,
        currentBestReadings.pm2_5.sensor_name.c_str());
    }
  }

  if (currentBestReadings.pm10.valid) {
    encoder.addPM10(currentBestReadings.pm10.value,
                    currentBestReadings.pm10.sensor_name.c_str());
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + PM10: %.1f μg/m³ from %s\n",
        currentBestReadings.pm10.value,
        currentBestReadings.pm10.sensor_name.c_str());
    }
  }

  // DC Power Monitor readings (INA219)
  if (currentBestReadings.voltage.valid) {
    encoder.addVoltage(currentBestReadings.voltage.value,
                       currentBestReadings.voltage.sensor_name.c_str());
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + Voltage: %.2f V from %s\n",
        currentBestReadings.voltage.value,
        currentBestReadings.voltage.sensor_name.c_str());
    }
  }

  if (currentBestReadings.current.valid) {
    encoder.addCurrent(currentBestReadings.current.value,
                       currentBestReadings.current.sensor_name.c_str());
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + Current: %.2f mA from %s\n",
        currentBestReadings.current.value,
        currentBestReadings.current.sensor_name.c_str());
    }
  }

  if (currentBestReadings.power.valid) {
    encoder.addPower(currentBestReadings.power.value,
                     currentBestReadings.power.sensor_name.c_str());
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + Power: %.2f mW from %s\n",
        currentBestReadings.power.value,
        currentBestReadings.power.sensor_name.c_str());
    }
  }

  // Battery level (PMU)
  if (currentBestReadings.battery.valid) {
    encoder.addBattery(currentBestReadings.battery.value,
                       currentBestReadings.battery.sensor_name.c_str());
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + Battery: %.1f%% from %s\n",
        currentBestReadings.battery.value,
        currentBestReadings.battery.sensor_name.c_str());
    }
  }

  // Encode all sensors into single protobuf message
  if (encoder.getMeasurementCount() > 0) {
    lorawanPayloadSize = encoder.encode(lorawanPayload, sizeof(lorawanPayload));
  }

  if (ENABLE_DEBUG_OUTPUT) {
    Serial.printf("LoRaWAN V2: Packed %d sensors into %d bytes (estimated %d)\n",
      encoder.getMeasurementCount(), lorawanPayloadSize, encoder.estimateSize());
  }

#else
  // Fallback to test message if protobuf is disabled
  const char* testMessage = "Hello, TTN!";
  lorawanPayloadSize = strlen(testMessage);
  memcpy(lorawanPayload, testMessage, lorawanPayloadSize);
#endif
  
  // Skip transmission if no valid sensor data available
  if (lorawanPayloadSize == 0) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("LoRaWAN: No sensor data available yet, skipping transmission");
    }
    // Still update the timer so we don't spam the log
    lastLoRaWANTransmission = millis();
    return;
  }

  Serial.println(F("----------------------------------"));

  // Ensure we're using highest data rate before each send
  // AU915 with dwell time (400ms): DR2=11, DR3=53, DR4=125, DR5=222 bytes
  // Disable ADR to prevent automatic data rate changes
  node->setADR(false);
  int drState = node->setDatarate(5);  // DR5 = SF7 = 222 bytes max (with dwell time)

  // Get and print current max payload for debugging
  uint8_t maxPayload = node->getMaxPayloadLen();
  Serial.print(F("LoRaWAN: Current max payload: ")); Serial.print(maxPayload); Serial.println(F(" bytes"));

  if (lorawanPayloadSize > maxPayload) {
    Serial.print(F("LoRaWAN: ERROR - Payload (")); Serial.print(lorawanPayloadSize);
    Serial.print(F(" bytes) exceeds max (")); Serial.print(maxPayload); Serial.println(F(" bytes)"));
    Serial.println(F("LoRaWAN: Skipping transmission - need compact payload format"));
    lastLoRaWANTransmission = millis();
    return;
  }

  Serial.print(F("LoRaWAN: Sending uplink (")); Serial.print(lorawanPayloadSize); Serial.print(F(" bytes)... "));
  Serial.flush();

  // Track TX attempt
  lorawan_tx_count++;
  lorawan_transmitting = true;
  lorawan_receiving = false;

  // Send uplink and wait for downlink windows (this will block briefly, but WiFi is now non-blocking)
  // This is the same method used in the working test sketches
  int state = node->sendReceive(lorawanPayload, lorawanPayloadSize, 1);
  
  lorawan_transmitting = false;
  lorawan_last_error = state;
  
  if (state >= RADIOLIB_ERR_NONE) {
    Serial.println(F("SUCCESS!"));
    Serial.println(F("LoRaWAN: ** UPLINK TRANSMITTED **"));
    Serial.println(F("LoRaWAN: Message sent to network"));
    lorawan_tx_success++;
    lorawan_consecutive_failures = 0;  // Reset failure counter on success
    lastLoraTxTime = millis();  // Update TX indicator for display
    if (!joinAccepted) {
      Serial.println(F("LoRaWAN: ** JOIN ACCEPTED - Now sending regular uplinks **"));
      joinAccepted = true;
    }
    lastLoRaWANTransmission = millis();

    // === DEEP SLEEP AFTER SUCCESSFUL TRANSMISSION ===
    // Enter deep sleep if configured and conditions are met
    if (isDeepSleepValid()) {
      unsigned long awakeTimeMs = millis();  // Time since boot/wake
      unsigned long sleepDurationMs = calculateSleepDuration(awakeTimeMs);

      Serial.println(F("----------------------------------"));
      Serial.print(F("[DeepSleep] Awake time: "));
      Serial.print(awakeTimeMs / 1000);
      Serial.println(F(" seconds"));

      // Enter deep sleep - this function does not return
      // On wake, the ESP32 restarts from setup()
      enterDeepSleep(sleepDurationMs);

      // If we reach here, deep sleep was not entered (validation failed)
      Serial.println(F("[DeepSleep] Deep sleep skipped - continuing normal operation"));
    }
  } else if (state == RADIOLIB_ERR_NETWORK_NOT_JOINED) {
    Serial.println(F("FAILED!"));
    Serial.println(F("LoRaWAN: Not joined to network yet"));
    Serial.print(F("LoRaWAN: Will retry in "));
    Serial.print(JOIN_RETRY_INTERVAL_MS / 1000);
    Serial.println(F(" seconds... Check TTN gateway status"));
    lorawan_tx_failed++;
    joinAccepted = false;  // Make sure we stay in retry mode
    lorawanJoined = false;  // Reset to join attempt mode
  } else {
    Serial.println(F("FAILED!"));
    Serial.print(F("LoRaWAN: Error code: "));
    Serial.println(state);
    lorawan_tx_failed++;
    lorawan_consecutive_failures++;
    
    // If we get timeout errors or multiple consecutive failures, assume we lost connection
    // Common errors when out of range: -1116 (NO_JOIN_ACCEPT), -706 (TX_TIMEOUT), -707 (RX_TIMEOUT)
    if (state == -1116 || state == -706 || state == -707 || lorawan_consecutive_failures >= 3) {
      if (lorawan_consecutive_failures >= 3) {
        Serial.print(F("LoRaWAN: "));
        Serial.print(lorawan_consecutive_failures);
        Serial.println(F(" consecutive failures - connection lost"));
      }
      Serial.println(F("LoRaWAN: Will attempt to rejoin"));
      joinAccepted = false;
      lorawanJoined = false;
      lorawan_consecutive_failures = 0;
    } else {
      Serial.println(F("LoRaWAN: Check gateway coverage and TTN console"));
    }
  }
  Serial.println(F("----------------------------------"));
  Serial.flush();
}

// Function to update and potentially publish sensor reading
bool updateSensorReading(const char* measurement_type, float value, const char* sensor_name, int priority, const char* mqtt_topic) {
  SensorReading* current = nullptr;
  
  // Map measurement type to current best reading
  if (strcmp(measurement_type, "temperature") == 0) current = &currentBestReadings.temperature;
  else if (strcmp(measurement_type, "humidity") == 0) current = &currentBestReadings.humidity;
  else if (strcmp(measurement_type, "pressure") == 0) current = &currentBestReadings.pressure;
  else if (strcmp(measurement_type, "co2") == 0) current = &currentBestReadings.co2;
  else if (strcmp(measurement_type, "voc") == 0) current = &currentBestReadings.voc;
  else if (strcmp(measurement_type, "nox") == 0) current = &currentBestReadings.nox;
  else if (strcmp(measurement_type, "pm1") == 0) current = &currentBestReadings.pm1;
  else if (strcmp(measurement_type, "pm2_5") == 0) current = &currentBestReadings.pm2_5;
  else if (strcmp(measurement_type, "pm10") == 0) current = &currentBestReadings.pm10;
  else if (strcmp(measurement_type, "voltage") == 0) current = &currentBestReadings.voltage;
  else if (strcmp(measurement_type, "current") == 0) current = &currentBestReadings.current;
  else if (strcmp(measurement_type, "power") == 0) current = &currentBestReadings.power;
  else if (strcmp(measurement_type, "battery") == 0) current = &currentBestReadings.battery;
  else if (strcmp(measurement_type, "gas_resistance") == 0) current = &currentBestReadings.gas_resistance;
  else {
    // Unknown measurement type - publish directly (for unique sensors) with enhanced payload
    publishSensorDataWithCalibration(mqtt_topic, value, "");
    return true;
  }
  
  // Check if this reading is better than current best
  bool shouldPublish = false;
  if (!current->valid || priority < current->priority) {
    // New best reading found
    current->value = value;
    current->sensor_name = String(sensor_name);
    current->priority = priority;
    current->valid = true;
    current->timestamp = getCurrentUnixTime();  // Capture timestamp when reading is taken
    shouldPublish = true;
    
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("Sensor hierarchy: "); Serial.print(sensor_name);
      Serial.print(" (priority "); Serial.print(priority); 
      Serial.print(") is now best source for "); Serial.println(measurement_type);
    }
  } else if (priority > current->priority || (priority == current->priority && current->sensor_name != String(sensor_name))) {
    // Reading is less accurate than current best, or different sensor with same priority
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("Sensor hierarchy: "); Serial.print(sensor_name);
      Serial.print(" (priority "); Serial.print(priority);
      Serial.print(") skipped - using "); Serial.print(current->sensor_name);
      Serial.print(" (priority "); Serial.print(current->priority);
      Serial.print(") for "); Serial.println(measurement_type);
    }
  }
  // If it's the same sensor with same priority, don't log anything (avoids "skipped - using itself" messages)
  
  if (shouldPublish) {
    // Note: MQTT publishing is now done after readAllSensors() completes
    // via publishAllSensorsMQTT() which uses v2 consolidated protobuf format
    // with per-reading timestamps for accuracy.
    // The currentBestReadings structure is shared by both WiFi and LoRaWAN.

    // Check if we should suppress data during calibration (measurement-type aware)
    // Basic environmental readings (temp, humidity, pressure) always publish
    // Calibration-sensitive readings (CO2, VOC, NOx) respect suppression
    String sensorName = String(sensor_name);
    sensorName.toLowerCase();
    if (!shouldPublishSensorData(sensorName, measurement_type)) {
      // Data suppressed during calibration, but publish diagnostic if monitoring enabled
      if (ENABLE_CALIBRATION_MONITORING && PUBLISH_RAW_READINGS_DURING_CALIBRATION) {
        // Determine appropriate unit for diagnostic publishing
        String unit = "";
        if (strcmp(measurement_type, "temperature") == 0) unit = "°C";
        else if (strcmp(measurement_type, "humidity") == 0) unit = "%";
        else if (strcmp(measurement_type, "pressure") == 0) unit = "hPa";
        else if (strcmp(measurement_type, "co2") == 0) unit = "ppm";

        publishDiagnosticReading(sensorName, measurement_type, value, unit);
      }

      // Mark as invalid so it won't be included in v2 protobuf message
      current->valid = false;

      if (ENABLE_DEBUG_OUTPUT) {
        Serial.print("Data suppressed (");
        Serial.print(getSensorCalibrationStatus(sensorName));
        Serial.print("): ");
        Serial.print(measurement_type);
        Serial.print(" from ");
        Serial.print(sensor_name);
        Serial.print(" = ");
        Serial.print(value, 2);
        Serial.println(" (diagnostic published)");
      }
    }
  }
  
  return shouldPublish;
}

// ======================
// CALIBRATION METHOD TRACKING FUNCTIONS
// ======================

// Set calibration method for a sensor
void setCalibrationMethod(const String& sensorName, const String& method, const String& confidence, const String& reference = "") {
  SensorCalibrationState* state = getSensorCalibrationState(sensorName);
  if (state != nullptr) {
    state->calibrationMethod = method;
    state->calibrationConfidence = confidence;
    state->calibrationTimestamp = getCurrentUnixTime();
    state->calibrationReference = reference;
    saveCalibrationState(); // Persist the calibration method
    
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print(sensorName + ": Calibration method set to '");
      Serial.print(method + "' with confidence '" + confidence + "'");
      if (reference.length() > 0) {
        Serial.print(" (ref: " + reference + ")");
      }
      Serial.println();
    }
  }
}

// Initialize sensor with factory calibration method
void initializeFactoryCalibration(const String& sensorName) {
  setCalibrationMethod(sensorName, "factory", "medium", "sensor_default");
}

// Handle calibration completion - upgrade method from learning to established
void handleCalibrationCompletion(const String& sensorName) {
  SensorCalibrationState* state = getSensorCalibrationState(sensorName);
  if (state != nullptr) {
    // Upgrade ASC from learning to established
    if (state->calibrationMethod == "asc_learning") {
      setCalibrationMethod(sensorName, "asc_established", "high", "7_day_complete");
      Serial.println(sensorName + ": ASC calibration complete - upgraded to established with high confidence");
    }
    // For other methods, just log completion
    else if (state->calibrationMethod != "unknown") {
      Serial.println(sensorName + ": Calibration period complete for method '" + state->calibrationMethod + "'");
    }
  }
}

// Get best available temperature for SGP41 compensation
float getBestTemperatureForSGP41() {
  if (currentBestReadings.temperature.valid) {
    return currentBestReadings.temperature.value;
  }
  return -999.0f; // No valid temperature available
}

// Get best available humidity for SGP41 compensation
float getBestHumidityForSGP41() {
  if (currentBestReadings.humidity.valid) {
    return currentBestReadings.humidity.value;
  }
  return -999.0f; // No valid humidity available
}

// Get best available pressure for SCD4x compensation
// SparkFun SCD4x library supports setAmbientPressure() for pressure compensation
float getBestPressureForSCD4x() {
  if (currentBestReadings.pressure.valid) {
    return currentBestReadings.pressure.value;
  }
  return -999.0f; // No valid pressure available
}

// Reset hierarchy at start of each sensor reading cycle
// All readings must be reset to prevent stale/cached values from being published
void resetSensorHierarchy() {
  // Environmental sensors
  currentBestReadings.temperature.valid = false;
  currentBestReadings.humidity.valid = false;
  currentBestReadings.pressure.valid = false;
  currentBestReadings.co2.valid = false;
  currentBestReadings.voc.valid = false;
  currentBestReadings.nox.valid = false;
  currentBestReadings.gas_resistance.valid = false;
  // Particulate matter
  currentBestReadings.pm1.valid = false;
  currentBestReadings.pm2_5.valid = false;
  currentBestReadings.pm10.valid = false;
  // Power monitoring
  currentBestReadings.voltage.valid = false;
  currentBestReadings.current.valid = false;
  currentBestReadings.power.valid = false;
  currentBestReadings.battery.valid = false;
}

// ==========================
// GLOBAL VARIABLES & CONSTANTS
// ==========================
BoardConfig g_boardConfig; // Global board configuration
bool wifiConnected = false;

// --- Sensor Objects ---
Adafruit_AHTX0 aht20;
bool aht20Available = false;

SCD4x scd4x;
bool scd4xAvailable = false;

SCD30 scd30;
bool scd30Available = false;

CM1106_I2C cm1106c;
bool cm1106cAvailable = false;

SensirionI2CSgp41 sgp41;
bool sgp41Available = false;

Adafruit_BMP280 bmp280;
bool bmp280Available = false;

Adafruit_BMP3XX bmp390;
bool bmp390Available = false;
int bmp390WarmupReadsRemaining = 2;  // Skip publishing first 2 reads after boot (sensor stabilization)

MS5611 ms5611(0x77);  // Default I2C address, can also be 0x76
bool ms5611Available = false;
int ms5611WarmupReadsRemaining = 2;  // Skip publishing first 2 reads after boot (sensor stabilization)

Adafruit_BME680 bme680;
bool bme680Available = false;

SensirionI2cSht4x sht4x;
bool sht4xAvailable = false;
String sht4xDetectedVariant = "SHT4x";  // Will be set during init based on serial number pattern

TMP117 tmp117;  // TMP117 high-precision temperature sensor (±0.1°C)
bool tmp117Available = false;

XPowersAXP2101 PMU;
bool pmuAvailable = false;

TinyGPSPlus gps;
bool gpsAvailable = false;

// --- PMS5003 Air Quality Sensor ---
// ESP32-C3 and C6 only have UART0 and UART1 (not UART2)
#if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
HardwareSerial PMS5003Serial(1); // UART1 for PMS5003 on ESP32-C3/C6
#else
HardwareSerial PMS5003Serial(2); // UART2 for PMS5003 on other ESP32 variants
#endif
bool pms5003Available = false;
PMS5003Data pms5003Data = {0};

// --- CM1106-C CO₂ Sensor (UART) ---
// CM1106-C shares the same UART with PMS5003 - only one can be connected at a time
bool c8co2Available = false;
C8CO2Data c8co2Data = {0};

// --- INA219 DC Current Monitor ---
Adafruit_INA219 ina219(INA219_I2C_ADDRESS);
bool ina219Available = false;

// --- OLED Display ---
// Display type enumeration for auto-detection
enum DisplayType {
  DISPLAY_NONE = 0,
  DISPLAY_SSD1306,  // 0.96" displays typically
  DISPLAY_SH1106    // 1.3" displays typically
};
DisplayType detectedDisplayType = DISPLAY_NONE;

// Use Adafruit_GFX pointer for common drawing operations
// Both SSD1306 and SH110X inherit from Adafruit_GFX
Adafruit_SSD1306* displaySSD1306 = nullptr;
Adafruit_SH1106G* displaySH1106 = nullptr;
Adafruit_GFX* display = nullptr;  // Points to whichever display is active

bool displayAvailable = false;
bool displayTimedOut = false;
unsigned long displayStartTime = 0;

// Probe OLED display type by reading status register
// Returns: 1 = SSD1306, 2 = SH1106, 0 = unknown
// Detection based on empirical testing:
//   SH1106 status byte: 0x16 (bit 4 set)
//   SSD1306 status byte: 0x02 (bit 4 clear)
uint8_t probeOLEDChipType(TwoWire* wire, uint8_t addr) {
  // Turn display ON and read status register
  wire->beginTransmission(addr);
  wire->write(0x00);       // Command mode
  wire->write(0xAF);       // Display ON
  wire->endTransmission();
  delay(5);

  // Read status byte - key difference between chips
  wire->requestFrom(addr, (uint8_t)1);
  if (wire->available()) {
    uint8_t status = wire->read();
    Serial.print("  Display status byte: 0x");
    Serial.println(status, HEX);

    // Detection logic based on bit 4:
    // SH1106: 0x16 (bit 4 = 0x10 is SET)
    // SSD1306: 0x02 (bit 4 is CLEAR)
    if (status & 0x10) {
      Serial.println("  Auto-detected: SH1106 (1.3\" display)");
      return 2;  // SH1106
    } else {
      Serial.println("  Auto-detected: SSD1306 (0.96\" display)");
      return 1;  // SSD1306
    }
  }

  Serial.println("  Could not read status byte");
  return 0; // Unknown - let caller decide
}

// Helper function to refresh display - handles both SSD1306 and SH1106
void displayRefresh() {
  if (!displayAvailable) return;
  if (detectedDisplayType == DISPLAY_SSD1306 && displaySSD1306) {
    displaySSD1306->display();
  } else if (detectedDisplayType == DISPLAY_SH1106 && displaySH1106) {
    displaySH1106->display();
  }
}

// Helper function to clear display
void displayClear() {
  if (!displayAvailable || !display) return;
  display->fillScreen(0);
}

// Helper function to update boot status line (bottom of screen)
void updateBootStatus(const char* status) {
  if (!displayAvailable || !display) return;

  // Clear status line area (y=56 to y=64, full width)
  display->fillRect(0, 56, DISPLAY_WIDTH, 8, 0);

  // Draw new status
  display->setTextSize(1);
  display->setTextColor(1);
  display->setCursor(0, 56);
  display->print(status);

  // Refresh display
  displayRefresh();

  Serial.print("Boot status: ");
  Serial.println(status);
}

// Early display initialization - called right after Wire.begin()
// This allows boot status messages to appear from the very start
bool initDisplayEarly() {
  if (DISABLE_DISPLAY) {
    Serial.println("Display: Disabled by configuration");
    return false;
  }

  Serial.println("Early display initialization...");

  // Check for display at common addresses on primary bus
  uint8_t detectedAddress = 0;
  TwoWire* displayBus = &Wire;
  int busNum = 0;

  for (uint8_t addr : {0x3C, 0x3D}) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      detectedAddress = addr;
      Serial.print("  Display found on PRIMARY bus at 0x");
      Serial.println(addr, HEX);
      break;
    }
  }

  // If not found on primary and board has dual I2C, check secondary
  // Note: ESP32-C3/C6 don't have Wire1
#if !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32C6)
  if (detectedAddress == 0 && HAS_DUAL_I2C()) {
    Serial.println("  Checking secondary I2C bus for display...");
    // Initialize secondary bus temporarily
    Wire1.begin(g_boardConfig.sda_pin2, g_boardConfig.scl_pin2, I2C_FREQUENCY);
    delay(10);

    for (uint8_t addr : {0x3C, 0x3D}) {
      Wire1.beginTransmission(addr);
      if (Wire1.endTransmission() == 0) {
        detectedAddress = addr;
        displayBus = &Wire1;
        busNum = 1;
        Serial.print("  Display found on SECONDARY bus at 0x");
        Serial.println(addr, HEX);
        break;
      }
    }
  }
#endif

  if (detectedAddress == 0) {
    Serial.println("  No display found at 0x3C or 0x3D on any bus");
    return false;
  }

  // Probe display type
  uint8_t probeResult = probeOLEDChipType(displayBus, detectedAddress);

  bool displayInitialized = false;

  // Determine driver based on probe or config
  bool trySSD1306 = (DISPLAY_DRIVER_TYPE == 1) || (DISPLAY_DRIVER_TYPE == 0 && probeResult != 2);
  bool trySH1106 = (DISPLAY_DRIVER_TYPE == 2) || (DISPLAY_DRIVER_TYPE == 0 && probeResult == 2);

  // Try SSD1306
  if (trySSD1306 && !displayInitialized) {
    displaySSD1306 = new Adafruit_SSD1306(DISPLAY_WIDTH, DISPLAY_HEIGHT, displayBus, -1);
    if (displaySSD1306->begin(SSD1306_SWITCHCAPVCC, detectedAddress)) {
      displayInitialized = true;
      detectedDisplayType = DISPLAY_SSD1306;
      display = displaySSD1306;
      Serial.println("  SSD1306 initialized");
    } else {
      delete displaySSD1306;
      displaySSD1306 = nullptr;
    }
  }

  // Try SH1106
  if (trySH1106 && !displayInitialized) {
    displaySH1106 = new Adafruit_SH1106G(DISPLAY_WIDTH, DISPLAY_HEIGHT, displayBus, -1);
    if (displaySH1106->begin(detectedAddress, true)) {
      displayInitialized = true;
      detectedDisplayType = DISPLAY_SH1106;
      display = displaySH1106;
      Serial.println("  SH1106 initialized");
    } else {
      delete displaySH1106;
      displaySH1106 = nullptr;
    }
  }

  if (!displayInitialized) {
    Serial.println("  Display initialization failed");
    return false;
  }

  // Show boot screen - must call display-specific method since displayAvailable not set yet
  delay(50);
  display->fillScreen(0);
  if (detectedDisplayType == DISPLAY_SSD1306) {
    displaySSD1306->display();
  } else {
    displaySH1106->display();
  }
  delay(50);

  display->fillScreen(0);
  display->setTextColor(1);

  // Draw "WeSense" title centered
  display->setTextSize(2);
  display->setCursor(16, 20);
  display->print("WeSense");

  // Initial status
  display->setTextSize(1);
  display->setCursor(0, 56);
  display->print("Starting...");

  // Update display directly (can't use displayRefresh yet)
  if (detectedDisplayType == DISPLAY_SSD1306) {
    displaySSD1306->display();
  } else {
    displaySH1106->display();
  }

  displayAvailable = true;
  displayStartTime = millis();
  displayTimedOut = false;

  Serial.println("  Boot screen active");
  return true;
}

// --- Boot Button ---
volatile bool bootButtonPressed = false;
unsigned long lastButtonPress = 0;
volatile unsigned long bootButtonISRCount = 0;  // Debug: count ISR triggers
bool lastBootButtonState = HIGH;  // For polling fallback (HIGH = not pressed with pullup)

// Boot button interrupt handler
void IRAM_ATTR bootButtonISR() {
  bootButtonPressed = true;
  bootButtonISRCount++;
}

// --- Device Naming & MQTT ---
// Dynamic device naming based on deployment type
char deviceID[48];           // Increased size for full MAC + location
char mqttClientID[64];       // Increased size accordingly  
char deviceTopicPrefix[96];  // Increased size for longer topic structure

// --- MQTT Client & WiFi Client ---
WiFiClient espClient;
PubSubClient client(espClient);

// --- GPS Serial ---
HardwareSerial GPSSerial(1); // UART1 for GPS

// --- Gas Algorithm Parameters ---
GasIndexAlgorithmParams voc_params;
GasIndexAlgorithmParams nox_params;

// --- I2C Sensor Location Tracking ---
// Track which I2C bus each sensor is found on for multi-bus support
struct SensorLocation {
  bool found;
  int bus; // 0 = primary (Wire), 1 = secondary (Wire1 if available)
};

// --- Dual I2C Bus Support ---
// For single I2C controller boards (like ESP32-C3), create a Wire1 reference to Wire
// This allows the same code to work on both single and dual I2C controller boards

// Helper function to get the appropriate I2C bus
inline TwoWire* getI2CBus(int busNumber) {
  if (busNumber == 0 || !HAS_DUAL_I2C()) {
    return &Wire;
  } else {
    #if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
      // ESP32-C3/C6 only have one I2C controller, always return Wire
      return &Wire;
    #else
      // ESP32 classic/S3 with dual I2C support
      return &Wire1;
    #endif
  }
}

struct {
  SensorLocation aht20 = {false, 0};
  SensorLocation scd4x = {false, 0};
  SensorLocation scd30 = {false, 0};
  SensorLocation cm1106c = {false, 0};
  SensorLocation sgp41 = {false, 0};
  SensorLocation sht4x = {false, 0};
  SensorLocation tmp117 = {false, 0};  // TMP117 high-precision temperature sensor
  SensorLocation bmp280 = {false, 0};
  SensorLocation bmp390 = {false, 0};
  SensorLocation ms5611 = {false, 0};
  SensorLocation bme680 = {false, 0};
  SensorLocation ina219 = {false, 0};
  SensorLocation display = {false, 0};
} sensorLocations;

// --- Multi-Sensor Calibration State Management ---
// Struct definitions moved to top for better visibility
AllSensorCalibrationStates sensorCalibrationStates;

// --- Runtime Location Configuration ---
// These variables can be updated via MQTT and persist across reboots
// They are initialised from user configuration values in setup()
struct RuntimeLocationConfig {
  bool includeInMQTT;
  float latitude;
  float longitude;
  bool overrideActive;  // true if MQTT has overridden the defaults
} runtimeLocation;

Preferences locationPrefs;  // For persisting location overrides

// --- Calibration Persistence and Monitoring ---
Preferences calibrationPrefs;
static unsigned long lastDiagnosticPublishTime = 0;
bool calibrationStateLoaded = false; // Track whether calibration state has been loaded

// --- Secure Network Serial Monitor ---
SecureTelnetMonitor telnetMonitor(TELNET_PORT);

// Handle different Serial types with auto type deduction for compatibility
auto* OriginalSerial = &Serial;

// Comprehensive dual-output Serial replacement
class TelnetSerial : public Print {
public:
  // Core Print class methods that all other print methods call
  size_t write(uint8_t c) override {
    size_t result = OriginalSerial->write(c);
    if (telnetMonitor.isConnected()) {
      telnetMonitor.client.write(c);
    }
    return result;
  }
  
  size_t write(const uint8_t *buffer, size_t size) override {
    size_t result = OriginalSerial->write(buffer, size);
    if (telnetMonitor.isConnected()) {
      telnetMonitor.client.write(buffer, size);
    }
    return result;
  }
  
  // Pass through methods to maintain Serial compatibility
  void begin(unsigned long baud) { OriginalSerial->begin(baud); }
  void end() { OriginalSerial->end(); }
  int available() { return OriginalSerial->available(); }
  int read() { return OriginalSerial->read(); }
  int peek() { return OriginalSerial->peek(); }
  void flush() { OriginalSerial->flush(); }
  operator bool() { return (bool)*OriginalSerial; }
  
  // Note: setTxTimeoutMs is USB-CDC specific and not needed for core functionality
};

TelnetSerial DebugSerial;

// Redirect all Serial calls to our dual-output version
#define Serial DebugSerial

// Legacy debug functions (now work automatically via Serial macro)
void debugPrintln(const String& message) {
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.println(message);
  }
}

void debugPrint(const String& message) {
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print(message);
  }
}


// ======================
// GLOBAL VARIABLES ENDS
// ======================

// ======================
// MULTI-SENSOR CALIBRATION STATE MANAGEMENT
// ======================

// Struct definitions are now at the top of the file

// ======================
// LOCATION CONFIGURATION MANAGEMENT
// ======================

// Initialize location configuration from user defaults
void initializeLocationConfig() {
  locationPrefs.begin("location", false);
  
  // Check if we have MQTT overrides
  runtimeLocation.overrideActive = locationPrefs.getBool("override_active", false);
  
  if (runtimeLocation.overrideActive) {
    // Load MQTT overrides
    runtimeLocation.includeInMQTT = locationPrefs.getBool("include_mqtt", INCLUDE_LOCATION_IN_MQTT);
    runtimeLocation.latitude = locationPrefs.getFloat("latitude", FIXED_LATITUDE);
    runtimeLocation.longitude = locationPrefs.getFloat("longitude", FIXED_LONGITUDE);

    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("Location configuration loaded from MQTT overrides:");
      Serial.print("   Latitude: "); Serial.println(runtimeLocation.latitude, 6);
      Serial.print("   Longitude: "); Serial.println(runtimeLocation.longitude, 6);
      Serial.print("   Include in MQTT: "); Serial.println(runtimeLocation.includeInMQTT ? "Yes" : "No");
    }
  } else {
    // Use firmware defaults from user configuration
    runtimeLocation.includeInMQTT = INCLUDE_LOCATION_IN_MQTT;
    runtimeLocation.latitude = FIXED_LATITUDE;
    runtimeLocation.longitude = FIXED_LONGITUDE;

    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("Location configuration using firmware defaults:");
      Serial.print("   Latitude: "); Serial.println(runtimeLocation.latitude, 6);
      Serial.print("   Longitude: "); Serial.println(runtimeLocation.longitude, 6);
      Serial.print("   Include in MQTT: "); Serial.println(runtimeLocation.includeInMQTT ? "Yes" : "No");
    }
  }
  
  locationPrefs.end();
}

// Save location configuration to NVS
bool saveLocationConfig() {
  locationPrefs.begin("location", false);

  bool success = true;
  success &= locationPrefs.putBool("override_active", runtimeLocation.overrideActive);
  success &= locationPrefs.putBool("include_mqtt", runtimeLocation.includeInMQTT);
  success &= locationPrefs.putFloat("latitude", runtimeLocation.latitude);
  success &= locationPrefs.putFloat("longitude", runtimeLocation.longitude);

  locationPrefs.end();

  if (success && ENABLE_DEBUG_OUTPUT) {
    Serial.println("Location configuration saved to NVS");
  } else if (!success && ENABLE_DEBUG_OUTPUT) {
    Serial.println("Failed to save location configuration to NVS");
  }

  return success;
}

// Reset location to firmware defaults
void resetLocationToDefaults() {
  runtimeLocation.overrideActive = false;
  runtimeLocation.includeInMQTT = INCLUDE_LOCATION_IN_MQTT;
  runtimeLocation.latitude = FIXED_LATITUDE;
  runtimeLocation.longitude = FIXED_LONGITUDE;
  
  // Clear NVS overrides
  locationPrefs.begin("location", false);
  locationPrefs.clear();
  locationPrefs.end();
  
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.println("Location configuration reset to firmware defaults");
  }
}

// Validate coordinates
bool validateCoordinates(float lat, float lng) {
  return (lat >= -90.0 && lat <= 90.0 && lng >= -180.0 && lng <= 180.0);
}

// Get sensor calibration state by name
SensorCalibrationState* getSensorCalibrationState(const String& sensorName) {
  if (sensorName == "scd4x") return &sensorCalibrationStates.scd4x;
  if (sensorName == "scd30") return &sensorCalibrationStates.scd30;
  if (sensorName == "cm1106c") return &sensorCalibrationStates.cm1106c;
  if (sensorName == "c8co2") return &sensorCalibrationStates.c8co2;
  if (sensorName == "sgp41") return &sensorCalibrationStates.sgp41;
  if (sensorName == "bme680") return &sensorCalibrationStates.bme680;
  if (sensorName == "pms5003") return &sensorCalibrationStates.pms5003;
  if (sensorName == "bmp280") return &sensorCalibrationStates.bmp280;
  if (sensorName == "sht4x") return &sensorCalibrationStates.sht4x;
  if (sensorName == "aht20") return &sensorCalibrationStates.aht20;
  if (sensorName == "tmp117") return &sensorCalibrationStates.tmp117;
  return nullptr;
}

// Get calibration period for sensor
unsigned long getCalibrationPeriod(const String& sensorName) {
  if (sensorName == "scd4x") return calibrationPeriods.scd4x;
  if (sensorName == "scd30") return calibrationPeriods.scd30;
  if (sensorName == "cm1106c") return calibrationPeriods.cm1106c;
  if (sensorName == "c8co2") return calibrationPeriods.c8co2;
  if (sensorName == "sgp41") return calibrationPeriods.sgp41;
  if (sensorName == "bme680") return calibrationPeriods.bme680;
  if (sensorName == "pms5003") return calibrationPeriods.pms5003;
  if (sensorName == "bmp280") return calibrationPeriods.bmp280;
  if (sensorName == "sht4x") return calibrationPeriods.sht4x;
  if (sensorName == "aht20") return calibrationPeriods.aht20;
  if (sensorName == "tmp117") return calibrationPeriods.tmp117;
  return 0;
}

// ======================
// CALIBRATION STATE PERSISTENCE
// ======================

// Helper function to format interval for display
String formatInterval(unsigned long intervalMs) {
  unsigned long hours = intervalMs / 3600000;
  if (hours >= 24) {
    return String(hours / 24) + " day" + (hours / 24 > 1 ? "s" : "");
  } else {
    return String(hours) + " hour" + (hours > 1 ? "s" : "");
  }
}

// Calculate simple checksum for backup data integrity
uint32_t calculateChecksum(const CalibrationBackup* backup) {
  uint32_t sum = 0;
  const uint8_t* data = (const uint8_t*)backup;
  // Calculate checksum excluding the checksum fields themselves
  for (size_t i = 0; i < offsetof(CalibrationBackup, dataChecksum); i++) {
    sum += data[i];
  }
  return sum;
}

// Check if existing calibration backup contains valuable data that shouldn't be overwritten
bool hasValuableCalibrationData(unsigned long currentTime) {
  if (!ENABLE_BACKUP_WRITE_PROTECTION) return false;
  
  // Check EEPROM backups for valuable calibration data
  for (int slot = 0; slot < EEPROM_BACKUP_SLOTS; slot++) {
    if (!EEPROM.begin(512)) continue;
    
    CalibrationBackup backup;
    EEPROM.get(slot * sizeof(CalibrationBackup), backup);
    EEPROM.end();
    
    // Validate backup integrity
    if (backup.magic1 != 0xDEADBEEF || backup.magic2 != 0xCAFEBABE) continue;
    uint32_t expectedChecksum = calculateChecksum(&backup);
    if (backup.dataChecksum != expectedChecksum) continue;
    
    // Check if any sensor has valuable calibration progress
    // BME680 excluded - Bosch provides no baseline save/restore mechanism
    String sensorNames[] = {"scd4x", "scd30", "cm1106c", "c8co2", "sgp41", "pms5003"};
    for (int i = 0; i < 5; i++) {
      if (backup.sensors[i].isCalibrating) {
        unsigned long calibrationAge = currentTime - backup.sensors[i].unixStartTime;
        unsigned long calibrationDuration = backup.sensors[i].periodHours * 3600;
        
        // Protect calibration data that is:
        // 1. Older than minimum protection age (not just started)
        // 2. Not yet expired (still valuable)
        if (calibrationAge > MIN_CALIBRATION_AGE_TO_PROTECT && 
            calibrationAge < calibrationDuration) {
          
          if (ENABLE_DEBUG_OUTPUT) {
            Serial.print("Found valuable calibration data: ");
            Serial.print(sensorNames[i]); 
            Serial.print(" has "); Serial.print(calibrationAge / 3600); 
            Serial.print("h of "); Serial.print(backup.sensors[i].periodHours);
            Serial.println("h total - PROTECTION ACTIVE");
          }
          return true;
        }
      }
    }
  }
  
  return false;
}

// Prevent starting fresh calibration if valuable backup data exists
bool shouldPreventCalibrationReset(const String& reason) {
  if (!ENABLE_BACKUP_WRITE_PROTECTION) return false;
  
  unsigned long currentTime = getCurrentUnixTime();
  if (currentTime == 0) currentTime = millis() / 1000 + 1640000000; // Emergency fallback
  
  if (hasValuableCalibrationData(currentTime)) {
    Serial.println("CALIBRATION RESET PREVENTED!");
    Serial.println("Reason for reset attempt: " + reason);
    Serial.println("Valuable calibration data detected in backups");
    Serial.println("To override, send MQTT command: calibration_force_reset");
    Serial.println("This prevents accidental loss of multi-day calibration progress");
    return true;
  }
  
  return false;
}

// Save critical calibration data to EEPROM as emergency backup
void saveCalibrationBackupToEEPROM(unsigned long currentTime) {
  // Minimize EEPROM wear by checking if data actually changed
  static CalibrationBackup lastBackup;
  static bool hasLastBackup = false;
  
  if (!EEPROM.begin(512)) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("EEPROM backup failed - could not initialize EEPROM");
    }
    return;
  }
  
  CalibrationBackup backup;
  backup.saveTime = currentTime;
  
  // Save critical sensors only (the ones with long calibration periods)
  // BME680 excluded - Bosch provides no baseline save/restore mechanism
  String sensorNames[] = {"scd4x", "scd30", "cm1106c", "c8co2", "sgp41", "pms5003"};
  for (int i = 0; i < 5; i++) {
    SensorCalibrationState* state = getSensorCalibrationState(sensorNames[i]);
    if (state != nullptr && state->isCalibrating) {
      backup.sensors[i].isCalibrating = true;
      // Calculate actual start time in Unix timestamp
      unsigned long elapsedMs = millis() - state->calibrationStartTime;
      backup.sensors[i].unixStartTime = currentTime - (elapsedMs / 1000);
      backup.sensors[i].periodHours = state->calibrationPeriodHours;
      backup.sensors[i].dataSuppressionActive = state->dataSuppressionActive;
    } else {
      backup.sensors[i].isCalibrating = false;
      backup.sensors[i].unixStartTime = 0;
      backup.sensors[i].periodHours = 0;
      backup.sensors[i].dataSuppressionActive = false;
    }
  }
  
  backup.dataChecksum = calculateChecksum(&backup);
  
  // Only write if data actually changed (minimize EEPROM wear)
  if (hasLastBackup && memcmp(&backup, &lastBackup, sizeof(CalibrationBackup)) == 0) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("EEPROM backup skipped - no changes detected (wear reduction)");
    }
    EEPROM.end();
    return;
  }
  
  // Write backup to EEPROM
  EEPROM.put(0, backup);
  if (EEPROM.commit()) {
    lastBackup = backup;  // Remember this backup to detect future changes
    hasLastBackup = true;
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("Emergency calibration backup saved to EEPROM");
      Serial.print("EEPROM backup details: magic1=0x"); Serial.print(backup.magic1, HEX);
      Serial.print(", saveTime="); Serial.print(backup.saveTime);
      Serial.print(", checksum=0x"); Serial.println(backup.dataChecksum, HEX);
      
      // Show what was saved for each sensor
      String sensorNames[] = {"scd4x", "scd30", "cm1106c", "c8co2", "sgp41", "bme680", "pms5003"};
      for (int i = 0; i < 6; i++) {
        if (backup.sensors[i].isCalibrating) {
          Serial.print(sensorNames[i]);
          Serial.print(": isCalibrating=true, unixStart="); Serial.print(backup.sensors[i].unixStartTime);
          Serial.print(", period="); Serial.print(backup.sensors[i].periodHours); Serial.println("h");
        }
      }
    }
  } else {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("EEPROM backup failed - commit error");
    }
  }
  
  EEPROM.end();
}

// Load calibration data from EEPROM backup if NVS fails
bool loadCalibrationFromEEPROMBackup(unsigned long currentTime) {
  if (!EEPROM.begin(512)) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("EEPROM restore failed - could not initialize EEPROM");
    }
    return false;
  }
  
  CalibrationBackup backup;
  EEPROM.get(0, backup);
  EEPROM.end();
  
  // Verify backup integrity
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("EEPROM backup read: magic2=0x"); Serial.print(backup.magic2, HEX);
    Serial.print(" (expected 0xCAFEBABE), saveTime="); Serial.print(backup.saveTime);
    Serial.print(", checksum=0x"); Serial.println(backup.dataChecksum, HEX);
  }
  
  if (backup.magic2 != 0xCAFEBABE) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("EEPROM backup corrupted - invalid magic number");
    }
    return false;
  }
  
  uint32_t expectedChecksum = calculateChecksum(&backup);
  if (backup.dataChecksum != expectedChecksum) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("EEPROM backup corrupted - checksum mismatch");
    }
    return false;
  }
  
  // Restore calibration states from backup
  // BME680 excluded - Bosch provides no baseline save/restore mechanism
  String sensorNames[] = {"scd4x", "scd30", "cm1106c", "c8co2", "sgp41", "pms5003"};
  int restoredCount = 0;

  for (int i = 0; i < 5; i++) {
    if (backup.sensors[i].isCalibrating) {
      unsigned long elapsedSeconds = currentTime - backup.sensors[i].unixStartTime;
      unsigned long calibrationDurationSeconds = backup.sensors[i].periodHours * 3600;
      
      if (elapsedSeconds < calibrationDurationSeconds) {
        // Calibration still in progress - restore it
        SensorCalibrationState* state = getSensorCalibrationState(sensorNames[i]);
        if (state != nullptr) {
          state->isCalibrating = true;
          state->calibrationStartTime = millis() - (elapsedSeconds * 1000);
          state->calibrationPeriodHours = backup.sensors[i].periodHours;
          state->dataSuppressionActive = backup.sensors[i].dataSuppressionActive;
          
          unsigned long remainingMs = (calibrationDurationSeconds - elapsedSeconds) * 1000;
          Serial.print(sensorNames[i]);
          Serial.print(": Restored from EEPROM backup - ");
          Serial.print(formatRemainingTime(remainingMs));
          Serial.print(" remaining (started ");
          Serial.print(elapsedSeconds / 60); Serial.println(" min ago)");
          
          restoredCount++;
        }
      }
    }
  }
  
  if (restoredCount > 0) {
    Serial.print("Restored calibration for ");
    Serial.print(restoredCount); 
    Serial.println(" sensors from EEPROM emergency backup");
    return true;
  }
  
  return false;
}

// Get current Unix timestamp (seconds since epoch)
unsigned long getCurrentUnixTime() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    return mktime(&timeinfo);
  }
  return 0; // Return 0 if time is not available
}

// Save calibration state with selective backup (NVS always, EEPROM optionally)
void saveCalibrationStateSelective(bool includeEEPROMBackup = true) {
  if (!PERSIST_CALIBRATION_STATE) return;
  
  unsigned long currentTime = getCurrentUnixTime();
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("Current Unix time for save: "); Serial.println(currentTime);
  }
  
  // CRITICAL: Never refuse to save calibration data due to time sync issues
  // For deployed sensors, losing 7 days of calibration is unacceptable
  bool useMillisBackup = false;
  if (currentTime == 0) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("Time not synchronized - using millis-based emergency backup");
    }
    useMillisBackup = true;
    // Use a pseudo-timestamp based on uptime for emergency saves
    currentTime = millis() / 1000 + 1640000000; // Approximate recent epoch
  }
  
  // Try primary NVS storage first
  if (!calibrationPrefs.begin("cal_state", false)) {
    Serial.println("NVS calibration partition failed to open - checking partition info...");
    
    // Try to get partition info for diagnostics
    const esp_partition_t* nvs_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
    if (nvs_partition) {
      Serial.print("NVS partition found - size: ");
      Serial.print(nvs_partition->size / 1024); 
      Serial.print("KB, address: 0x"); 
      Serial.println(nvs_partition->address, HEX);
    } else {
      Serial.println("NVS partition not found in partition table!");
    }
    
    Serial.println("Attempting EEPROM backup only...");
    if (includeEEPROMBackup) {
      saveCalibrationBackupToEEPROM(currentTime);
    }
    return;
  }
  
  // Check NVS space availability and functionality
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.println("NVS namespace 'cal_state' opened successfully");
    
    // Test basic NVS write/read functionality
    Serial.println("Testing NVS write/read functionality...");
    
    // Test 1: Simple unsigned long write
    unsigned long testValue = 12345;
    size_t testBytes = calibrationPrefs.putULong("test_ulong", testValue);
    unsigned long readBack = calibrationPrefs.getULong("test_ulong", 0);
    Serial.print("Test ULong write: bytes="); Serial.print(testBytes);
    Serial.print(", wrote="); Serial.print(testValue);
    Serial.print(", read="); Serial.println(readBack);
    
    // Test 2: Test the actual timestamp value
    Serial.print("Testing actual timestamp: "); Serial.println(currentTime);
    size_t tsBytes = calibrationPrefs.putULong("test_ts", currentTime);  // Shorter key name
    unsigned long tsReadBack = calibrationPrefs.getULong("test_ts", 999999);
    Serial.print("Timestamp write: bytes="); Serial.print(tsBytes);
    Serial.print(", wrote="); Serial.print(currentTime);
    Serial.print(", read="); Serial.println(tsReadBack);
    
    // Test 3: String write
    size_t strBytes = calibrationPrefs.putString("test_str", "test123");  // Shorter key name
    String strReadBack = calibrationPrefs.getString("test_str", "FAILED");
    Serial.print("String write: bytes="); Serial.print(strBytes);
    Serial.print(", wrote=test123, read="); Serial.println(strReadBack);
    
    // Clean up test keys
    calibrationPrefs.remove("test_ulong");
    calibrationPrefs.remove("test_ts");
    calibrationPrefs.remove("test_str");
  }
  
  String sensors[] = {"scd4x", "scd30", "cm1106c", "c8co2", "sgp41", "bme680", "pms5003", "bmp280", "sht4x", "aht20", "tmp117"};

  for (String sensorName : sensors) {
    // Skip BME680 - Bosch provides no baseline save/restore mechanism
    // Saving timing data is meaningless as gas sensor state is lost on reboot
    if (sensorName == "bme680") {
      continue;
    }
    
    SensorCalibrationState* state = getSensorCalibrationState(sensorName);
    if (state != nullptr) {
      String prefix = sensorName + "_";
      calibrationPrefs.putBool((prefix + "cal").c_str(), state->isCalibrating);
      
      if (state->isCalibrating) {
        // Calculate the actual start time in Unix timestamp
        unsigned long elapsedMs = millis() - state->calibrationStartTime;
        unsigned long actualStartTime = currentTime - (elapsedMs / 1000);
        
        // Write calibration data to NVS with error checking (using shorter keys for 15-char limit)
        size_t bytesWritten1 = calibrationPrefs.putULong((prefix + "start").c_str(), actualStartTime);
        size_t bytesWritten2 = calibrationPrefs.putULong((prefix + "period").c_str(), state->calibrationPeriodHours);
        size_t bytesWritten3 = calibrationPrefs.putBool((prefix + "supp").c_str(), state->dataSuppressionActive);
        
        if (ENABLE_DEBUG_OUTPUT) {
          if (bytesWritten1 == 0 || bytesWritten2 == 0 || bytesWritten3 == 0) {
            Serial.print("NVS WRITE ERROR for "); Serial.print(sensorName);
            Serial.print(": bytes written - start:"); Serial.print(bytesWritten1);
            Serial.print(", period:"); Serial.print(bytesWritten2);
            Serial.print(", supp:"); Serial.println(bytesWritten3);
          } else {
            Serial.print("NVS write successful for "); Serial.print(sensorName);
            Serial.print(": start="); Serial.println(actualStartTime);
          }
        }
        
        if (ENABLE_DEBUG_OUTPUT) {
          Serial.print("Saving calibration state for "); Serial.print(sensorName);
          Serial.print(": started "); Serial.print(elapsedMs / 60000); Serial.print(" minutes ago");
          Serial.print(", Unix start time: "); Serial.print(actualStartTime);
          Serial.print(", period: "); Serial.print(state->calibrationPeriodHours); Serial.println("h");
        }
      }
    }
  }
  
  // Save SGP41 algorithm baseline states if sensor is available and has run for 3+ hours
  // Sensirion requirement: algorithm states are only valid after 3 hours of continuous operation
  if (sgp41Available) {
    SensorCalibrationState* sgp41State = getSensorCalibrationState("sgp41");
    if (sgp41State != nullptr && sgp41State->isCalibrating) {
      unsigned long elapsedMs = millis() - sgp41State->calibrationStartTime;
      unsigned long elapsedHours = elapsedMs / 3600000;
      
      if (elapsedMs >= 3 * 3600000) { // 3 hours minimum
        int32_t voc_state0 = 0, voc_state1 = 0;
        int32_t nox_state0 = 0, nox_state1 = 0;
        
        GasIndexAlgorithm_get_states(&voc_params, &voc_state0, &voc_state1);
        GasIndexAlgorithm_get_states(&nox_params, &nox_state0, &nox_state1);
        
        calibrationPrefs.putInt("sgp_voc_s0", voc_state0);
        calibrationPrefs.putInt("sgp_voc_s1", voc_state1);
        calibrationPrefs.putInt("sgp_nox_s0", nox_state0);
        calibrationPrefs.putInt("sgp_nox_s1", nox_state1);
        calibrationPrefs.putULong("sgp_save_t", currentTime); // Save timestamp for outage detection
        
        if (ENABLE_DEBUG_OUTPUT) {
          Serial.print("SGP41 algorithm baseline states saved (after ");
          Serial.print(elapsedHours); Serial.println(" hours):");
          Serial.print("  VOC state0="); Serial.print(voc_state0);
          Serial.print(", state1="); Serial.println(voc_state1);
          Serial.print("  NOx state0="); Serial.print(nox_state0);
          Serial.print(", state1="); Serial.println(nox_state1);
        }
      } else if (ENABLE_DEBUG_OUTPUT) {
        Serial.print("SGP41 baseline not saved yet (only ");
        Serial.print(elapsedHours); Serial.println(" hours, need 3+ hours)");
      }
    }
  }
  
  // Write global calibration settings with error checking (shorter keys for 15-char limit)
  size_t testingBytes = calibrationPrefs.putBool("testing", sensorCalibrationStates.globalTestingMode);
  size_t saveTimeBytes = calibrationPrefs.putULong("save_time", currentTime);
  // Use firmware version constant (defined in developer section)
  size_t firmwareBytes = calibrationPrefs.putString("fw_version", FIRMWARE_VERSION);
  
  // Check for write errors
  if (ENABLE_DEBUG_OUTPUT) {
    if (testingBytes == 0 || saveTimeBytes == 0 || firmwareBytes == 0) {
      Serial.print("NVS GLOBAL WRITE ERROR: testing:"); Serial.print(testingBytes);
      Serial.print(", save_time:"); Serial.print(saveTimeBytes);
      Serial.print(", firmware:"); Serial.println(firmwareBytes);
    } else {
      Serial.println("NVS global settings write successful");
    }
  }
  
  // Force NVS commit to flash before closing
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.println("Committing NVS data to flash...");
  }
  
  calibrationPrefs.end();
  
  // EEPROM backup only when requested (to reduce wear)
  if (includeEEPROMBackup) {
    saveCalibrationBackupToEEPROM(currentTime);
  }
  
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("Calibration state saved to NVS at Unix time: "); Serial.println(currentTime);
    if (includeEEPROMBackup) {
      Serial.println("Emergency backup also saved to EEPROM");
    } else {
      Serial.println("EEPROM backup skipped (wear reduction)");
    }
  }
}

// Save calibration state to flash memory using Unix timestamps (always includes EEPROM backup)
void saveCalibrationState() {
  saveCalibrationStateSelective(true);
}

// Load calibration state from flash memory using Unix timestamps
void loadCalibrationState() {
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.println("loadCalibrationState() CALLED");
  }
  
  if (!PERSIST_CALIBRATION_STATE) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("PERSIST_CALIBRATION_STATE is disabled - skipping calibration loading");
    }
    return;
  }
  
  unsigned long currentTime = getCurrentUnixTime();
  
  if (!calibrationPrefs.begin("cal_state", true)) { // Read-only
    Serial.println("NVS calibration partition failed to open - trying EEPROM backup...");
    if (loadCalibrationFromEEPROMBackup(currentTime)) {
      Serial.println("Calibration state restored from EEPROM emergency backup!");
      startFreshCalibrationForUncalibratedSensors();
      return;
    } else {
      Serial.println("No calibration backup found - starting fresh");
      startFreshCalibrationForUncalibratedSensors();
      return;
    }
  }
  
  // Check if we have saved state (look for new timestamp-based saves first)
  bool hasTimestampData = calibrationPrefs.isKey("save_time");
  bool hasLegacyData = calibrationPrefs.isKey("boot_time");
  
  if (!hasTimestampData && !hasLegacyData) {
    calibrationPrefs.end();
    Serial.println("No previous calibration state found in NVS - checking EEPROM backup...");
    
    // Try to load from EEPROM emergency backup
    if (loadCalibrationFromEEPROMBackup(currentTime)) {
      Serial.println("Calibration state restored from EEPROM emergency backup!");
      // Save the restored state back to NVS for future use
      saveCalibrationState();
      return;
    } else {
      Serial.println("No calibration backup found - starting fresh");
      startFreshCalibrationForUncalibratedSensors();
      return;
    }
  }
  
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("Current Unix time for load: "); Serial.println(currentTime);
  }
  if (currentTime == 0) {
    Serial.println("Cannot load calibration state - time not synchronized. Will load after NTP sync.");
    calibrationPrefs.end();
    return;
  }
  
  String firmwareVersion = calibrationPrefs.getString("fw_version", "unknown");
  Serial.print("Loading calibration state from NVS (saved by firmware: ");
  Serial.print(firmwareVersion); Serial.println(")...");
  
  // Check migration flag in separate namespace (survives calibration data clearing)
  Preferences migrationPrefs;
  bool migrationFlag = false;
  if (migrationPrefs.begin("migration", true)) {
    migrationFlag = migrationPrefs.getBool("v2025_done", false);
    migrationPrefs.end();
  }
  
  // Force fresh calibration if firmware version is unknown (old format)
  // But only do this once to prevent endless loops
  if (firmwareVersion == "unknown" && !migrationFlag) {
    Serial.println("Old calibration format detected - checking for valuable backup data...");
    
    // CRITICAL PROTECTION: Don't migrate if we have valuable calibration backups
    if (shouldPreventCalibrationReset("firmware migration from unknown version")) {
      Serial.println("MIGRATION BLOCKED - valuable calibration data detected");
      Serial.println("Attempting to preserve existing calibration by marking migration as done");
      
      // Mark migration as done without clearing data to prevent future attempts
      if (migrationPrefs.begin("migration", false)) {
        migrationPrefs.putBool("v2025_done", true);
        migrationPrefs.end();
        Serial.println("Migration flag set to prevent future migration attempts");
      }
      
      // Try to restore from EEPROM backup
      unsigned long currentTime = getCurrentUnixTime();
      if (currentTime == 0) currentTime = millis() / 1000 + 1640000000;
      
      if (loadCalibrationFromEEPROMBackup(currentTime)) {
        Serial.println("Calibration data restored from EEPROM backup!");
        return;
      } else {
        Serial.println("Migration blocked but backup restore failed - manual intervention needed");
        return;
      }
    }
    
    Serial.println("No valuable backup data found - proceeding with migration");
    
    // Set migration flag in separate namespace first (survives data clearing)
    if (migrationPrefs.begin("migration", false)) {
      migrationPrefs.putBool("v2025_done", true);
      migrationPrefs.end();
      Serial.println("Migration flag set in separate namespace");
    } else {
      Serial.println("Failed to set migration flag - continuing anyway");
    }
    
    calibrationPrefs.end();
    
    // Clear old calibration data
    calibrationPrefs.begin("cal_state", false);
    calibrationPrefs.clear();
    calibrationPrefs.putString("fw_version", FIRMWARE_VERSION);
    calibrationPrefs.end();
    
    Serial.println("Migration complete - starting fresh calibration with new format");
    startFreshCalibrationForUncalibratedSensors();
    return;
  } else if (firmwareVersion == "unknown" && migrationFlag) {
    Serial.println("Migration already attempted but firmware version still unknown - NVS write issue detected");
    Serial.println("Continuing with existing data to avoid loop");
  }
  
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.println("Stored calibration keys:");
    // Check which sensors have calibration data
    String sensors[] = {"scd4x", "scd30", "cm1106c", "c8co2", "sgp41", "bme680", "pms5003", "bmp280", "sht4x", "aht20", "tmp117"};
    for (String sensor : sensors) {
      String prefix = sensor + "_";
      bool hasCal = calibrationPrefs.isKey((prefix + "cal").c_str());
      if (hasCal) {
        Serial.print("  "); Serial.print(sensor); Serial.println(": has calibration data");
      }
    }
  }
  
  String sensors[] = {"scd4x", "scd30", "cm1106c", "c8co2", "sgp41", "bme680", "pms5003", "bmp280", "sht4x", "aht20", "tmp117"};

  for (String sensorName : sensors) {
    SensorCalibrationState* state = getSensorCalibrationState(sensorName);
    if (state != nullptr) {
      String prefix = sensorName + "_";
      
      bool wasCalibrating = calibrationPrefs.getBool((prefix + "cal").c_str(), false);
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.print(""); Serial.print(sensorName); 
        Serial.print(": wasCalibrating="); Serial.print(wasCalibrating ? "true" : "false");
        Serial.println("");
      }
      
      // BME680 special case: Bosch provides no baseline save/restore mechanism
      // Gas sensor state is lost on every reboot, so timer persistence is meaningless
      if (sensorName == "bme680" && wasCalibrating) {
        Serial.println("BME680: Saved calibration timing ignored - Bosch provides no baseline save mechanism");
        Serial.println("BME680: Gas sensor state lost on reboot, requires fresh 48-hour stabilisation period");
        // Don't restore state - will trigger fresh calibration start
        continue;
      }
      
      if (wasCalibrating) {
        unsigned long calibrationPeriod = calibrationPrefs.getULong((prefix + "period").c_str(), 0);
        
        if (hasTimestampData) {
          // Use new Unix timestamp-based loading (shorter key names for 15-char limit)
          unsigned long savedStartTime = calibrationPrefs.getULong((prefix + "start").c_str(), 0);
          if (ENABLE_DEBUG_OUTPUT) {
            Serial.print("Checking "); Serial.print(sensorName); Serial.print(": saved start="); Serial.print(savedStartTime); Serial.print(", current time="); Serial.println(currentTime);
          }
          if (savedStartTime > 0) {
            unsigned long elapsedSeconds = currentTime - savedStartTime;
            unsigned long calibrationDurationSeconds = calibrationPeriod * 3600; // Convert hours to seconds
            
            if (elapsedSeconds < calibrationDurationSeconds) {
              // Calibration still in progress
              state->isCalibrating = true;
              state->calibrationStartTime = millis() - (elapsedSeconds * 1000); // Convert back to millis
              state->calibrationPeriodHours = calibrationPeriod;
              state->dataSuppressionActive = calibrationPrefs.getBool((prefix + "supp").c_str(), SUPPRESS_DATA_DURING_CALIBRATION);
              
              unsigned long remainingMs = (calibrationDurationSeconds - elapsedSeconds) * 1000;
              Serial.print(""); Serial.print(sensorName); 
              Serial.print(": Continuing calibration - ");
              Serial.print(formatRemainingTime(remainingMs)); 
              Serial.print(" remaining (started ");
              Serial.print(elapsedSeconds / 60); Serial.println(" min ago)");
            } else {
              // Calibration completed while we were offline
              Serial.print(""); Serial.print(sensorName); Serial.println(": Calibration completed while offline");
            }
          }
        } else if (hasLegacyData) {
          // Fall back to legacy millis()-based loading for backwards compatibility
          Serial.println("Loading legacy calibration state (may be inaccurate after firmware update)");
          unsigned long savedStartTime = calibrationPrefs.getULong((prefix + "start").c_str(), 0);
          unsigned long previousBootTime = calibrationPrefs.getULong("boot_time", 0);
          unsigned long currentUptime = millis();
          
          unsigned long elapsedBeforeReboot = (previousBootTime > savedStartTime) ? previousBootTime - savedStartTime : 0;
          unsigned long totalElapsed = elapsedBeforeReboot + currentUptime;
          unsigned long calibrationDuration = calibrationPeriod * 3600000; // Convert hours to ms
          
          if (totalElapsed < calibrationDuration) {
            state->isCalibrating = true;
            state->calibrationStartTime = millis() - totalElapsed;
            state->calibrationPeriodHours = calibrationPeriod;
            state->dataSuppressionActive = calibrationPrefs.getBool((prefix + "supp").c_str(), SUPPRESS_DATA_DURING_CALIBRATION);
            
            unsigned long remainingMs = calibrationDuration - totalElapsed;
            Serial.print(""); Serial.print(sensorName); Serial.print(": Continuing calibration - ");
            Serial.print(formatRemainingTime(remainingMs)); Serial.println(" remaining (legacy)");
            
            // Immediately save in new format to upgrade the data
            saveCalibrationState();
          }
        }
      }
    }
  }
  
  sensorCalibrationStates.globalTestingMode = calibrationPrefs.getBool("testing", false);
  if (sensorCalibrationStates.globalTestingMode) {
    Serial.println("Global testing mode restored from previous session");
  }
  
  // Restore SGP41 algorithm baseline states if sensor is available
  if (sgp41Available && calibrationPrefs.isKey("sgp_voc_s0")) {
    // Check for power outage duration (Sensirion: states invalid after >10 min outage)
    unsigned long sgp_save_time = calibrationPrefs.getULong("sgp_save_t", 0);
    unsigned long outageDuration = 0;
    bool outageValid = false;
    
    if (sgp_save_time > 0 && currentTime > sgp_save_time) {
      outageDuration = currentTime - sgp_save_time;
      outageValid = (outageDuration <= 10 * 60); // 10 minutes in seconds
      
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.print("SGP41: Power outage duration: ");
        Serial.print(outageDuration / 60); Serial.println(" minutes");
      }
    }
    
    if (outageValid) {
      int32_t voc_state0 = calibrationPrefs.getInt("sgp_voc_s0", 0);
      int32_t voc_state1 = calibrationPrefs.getInt("sgp_voc_s1", 0);
      int32_t nox_state0 = calibrationPrefs.getInt("sgp_nox_s0", 0);
      int32_t nox_state1 = calibrationPrefs.getInt("sgp_nox_s1", 0);
      
      // Restore states to the algorithms
      GasIndexAlgorithm_set_states(&voc_params, voc_state0, voc_state1);
      GasIndexAlgorithm_set_states(&nox_params, nox_state0, nox_state1);
      
      Serial.println("SGP41 algorithm baseline states restored:");
      Serial.print("  VOC state0="); Serial.print(voc_state0);
      Serial.print(", state1="); Serial.println(voc_state1);
      Serial.print("  NOx state0="); Serial.print(nox_state0);
      Serial.print(", state1="); Serial.println(nox_state1);
      Serial.print("  Outage duration: "); Serial.print(outageDuration / 60);
      Serial.println(" min (within 10 min limit)");
      Serial.println("  SGP41 will resume from saved baseline, skipping initial learning phase");
    } else if (outageDuration > 10 * 60) {
      Serial.println("SGP41: Power outage too long ("); 
      Serial.print(outageDuration / 60); Serial.println(" minutes)");
      Serial.println("SGP41: Saved baseline may be invalid due to sensor cooling and gas adsorption");
      Serial.println("SGP41: Starting fresh calibration for accurate readings");
      
      // Clear the saved baseline to prevent future attempts
      calibrationPrefs.remove("sgp_voc_s0");
      calibrationPrefs.remove("sgp_voc_s1");
      calibrationPrefs.remove("sgp_nox_s0");
      calibrationPrefs.remove("sgp_nox_s1");
      calibrationPrefs.remove("sgp_save_t");
      
      // Force fresh calibration start
      SensorCalibrationState* sgp41State = getSensorCalibrationState("sgp41");
      if (sgp41State != nullptr) {
        sgp41State->isCalibrating = false; // Will trigger fresh start
      }
    } else {
      Serial.println("SGP41: Baseline timestamp invalid - starting fresh calibration");
    }
  } else if (sgp41Available) {
    Serial.println("SGP41: No saved baseline states found - sensor will start fresh calibration");
  }
  
  calibrationPrefs.end();
  
  Serial.println("Calibration state loading complete");
  
  // Start fresh calibration periods for sensors that don't have saved state
  startFreshCalibrationForUncalibratedSensors();
}

// Start calibration period for a specific sensor
void startSensorCalibrationPeriod(const String& sensorName) {
  if (!ENABLE_CALIBRATION_STATE_TRACKING) return;
  
  SensorCalibrationState* state = getSensorCalibrationState(sensorName);
  unsigned long period = getCalibrationPeriod(sensorName);
  
  if (state == nullptr || period == 0) {
    return; // Sensor doesn't exist or has no calibration period
  }
  
  state->isCalibrating = true;
  state->calibrationStartTime = millis();
  state->calibrationPeriodHours = period;
  state->dataSuppressionActive = SUPPRESS_DATA_DURING_CALIBRATION;
  
  Serial.print(""); Serial.print(sensorName); Serial.println(": Calibration/stabilization period started");
  Serial.print(""); Serial.print(sensorName); Serial.print(": Data quality stabilization period: ");
  Serial.print(period); Serial.println(" hours");
  if (SUPPRESS_DATA_DURING_CALIBRATION) {
    Serial.print(""); Serial.print(sensorName); Serial.println(": Data publication suppressed during calibration for global data quality");
  }
  
  // Save calibration state to persistent memory
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("About to save calibration state for "); Serial.println(sensorName);
    Serial.print("Current Unix time: "); Serial.println(getCurrentUnixTime());
  }
  saveCalibrationState();
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("Calibration state save completed for "); Serial.println(sensorName);
  }
}

// Check if sensor calibration period is complete
bool isSensorCalibrationComplete(const String& sensorName) {
  if (!ENABLE_CALIBRATION_STATE_TRACKING) return true;
  
  SensorCalibrationState* state = getSensorCalibrationState(sensorName);
  if (state == nullptr || !state->isCalibrating) {
    return true; // Sensor doesn't exist or not in calibration
  }
  
  unsigned long elapsedMs = millis() - state->calibrationStartTime;
  unsigned long calibrationDurationMs = state->calibrationPeriodHours * 3600000;
  
  bool isComplete = (elapsedMs >= calibrationDurationMs);
  
  if (ENABLE_DEBUG_OUTPUT && isComplete) {
    Serial.print("Calibration check: "); Serial.print(sensorName);
    Serial.print(" elapsed="); Serial.print(elapsedMs / 60000); Serial.print("min");
    Serial.print(", required="); Serial.print(calibrationDurationMs / 60000); Serial.print("min");
    Serial.println(" - COMPLETE!");
  }
  
  return isComplete;
}

// Update calibration state for all sensors (call this periodically)
void updateCalibrationState() {
  if (!ENABLE_CALIBRATION_STATE_TRACKING) return;
  
  static unsigned long lastPeriodicSave = 0;
  // Use configurable interval (1 minute for development, 5 minutes for production)

  String sensors[] = {"scd4x", "scd30", "cm1106c", "c8co2", "sgp41", "bme680", "pms5003", "bmp280", "sht4x", "aht20", "tmp117"};
  bool anyCalibrating = false;

  for (String sensorName : sensors) {
    SensorCalibrationState* state = getSensorCalibrationState(sensorName);
    if (state != nullptr && state->isCalibrating) {
      anyCalibrating = true;
      
      if (isSensorCalibrationComplete(sensorName)) {
        if (ENABLE_DEBUG_OUTPUT) {
          Serial.print(""); Serial.print(sensorName);
          Serial.println(": Transitioning from calibrating to calibrated state...");
        }
        
        state->isCalibrating = false;
        state->dataSuppressionActive = false;
        
        Serial.print(""); Serial.print(sensorName); Serial.println(": Calibration/stabilization period complete - sensor data now reliable");
        Serial.print(""); Serial.print(sensorName); Serial.println(": Data publication resumed for global data quality");
        
        // Handle calibration method completion (e.g., upgrade ASC learning to established)
        handleCalibrationCompletion(sensorName);
        
        if (ENABLE_DEBUG_OUTPUT) {
          Serial.print(""); Serial.print(sensorName);
          Serial.print(": Status changed from 'calibrating_finishing' to 'calibrated'");
          Serial.println(". Data will now appear in main sensor topics.");
        }
        
        // Save updated calibration state
        saveCalibrationState();
      }
    }
  }
  
  // Smart periodic progress save (NVS frequent, EEPROM adaptive based on calibration state)
  if (anyCalibrating && (millis() - lastPeriodicSave >= NVS_SAVE_INTERVAL_MS)) {
    lastPeriodicSave = millis();
    
    // Smart EEPROM backup intervals: 2h during calibration, 24h when calibrated
    static unsigned long lastEEPROMSave = 0;
    
    unsigned long eepromInterval = anyCalibrating ? EEPROM_SAVE_INTERVAL_CALIBRATING_MS : EEPROM_SAVE_INTERVAL_CALIBRATED_MS;
    bool saveToEEPROM = (millis() - lastEEPROMSave >= eepromInterval);
    
    if (saveToEEPROM) {
      lastEEPROMSave = millis();
    }
    
    if (ENABLE_DEBUG_OUTPUT) {
      String saveMessage = "Periodic calibration progress save";
      if (saveToEEPROM) {
        if (anyCalibrating) {
          saveMessage += " (including EEPROM backup - 2h interval during calibration)";
        } else {
          saveMessage += " (including EEPROM backup - 24h interval when calibrated)";
        }
      } else {
        saveMessage += " (NVS only - EEPROM " + String(anyCalibrating ? "2h" : "24h") + " interval wear reduction)";
      }
      debugPrintln(saveMessage);
    }
    
    // Save to NVS frequently, EEPROM at adaptive intervals
    saveCalibrationStateSelective(saveToEEPROM);
  } else if (!anyCalibrating) {
    // When no sensors are calibrating, still do periodic EEPROM saves (daily) for safety
    static unsigned long lastNonCalibratingEEPROMSave = 0;
    
    if (millis() - lastNonCalibratingEEPROMSave >= EEPROM_SAVE_INTERVAL_CALIBRATED_MS) {
      lastNonCalibratingEEPROMSave = millis();
      
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("Daily safety EEPROM backup (no active calibration)");
      }
      
      // Save both NVS and EEPROM for safety (but only daily to minimize wear)
      saveCalibrationStateSelective(true);
    }
  }
}

// Get current calibration status for a specific sensor
String getSensorCalibrationStatus(const String& sensorName) {
  if (!ENABLE_CALIBRATION_STATE_TRACKING) {
    return "tracking_disabled";
  }
  
  if (sensorCalibrationStates.globalTestingMode) {
    return "testing_mode";
  }
  
  SensorCalibrationState* state = getSensorCalibrationState(sensorName);
  if (state == nullptr) {
    return "unknown_sensor";
  }
  
  if (state->isCalibrating) {
    unsigned long remainingMs = getRemainingCalibrationMs(sensorName);
    unsigned long totalMs = state->calibrationPeriodHours * 3600000;
    unsigned long elapsedMs = totalMs - remainingMs;
    
    // Calculate elapsed and total times
    unsigned long elapsedHours, elapsedMinutes;
    calculateRemainingTime(elapsedMs, elapsedHours, elapsedMinutes);
    
    if (remainingMs > 0) {
      unsigned long hours, minutes;
      calculateRemainingTime(remainingMs, hours, minutes);
      
      String timeDisplay = "";
      if (hours > 0 && minutes > 0) {
        timeDisplay = String(hours) + "h_" + String(minutes) + "m";
      } else if (hours > 0) {
        timeDisplay = String(hours) + "h";
      } else if (minutes > 0) {
        timeDisplay = String(minutes) + "m";
      } else {
        return "calibrating_finishing_of_" + String(state->calibrationPeriodHours) + "h_total";
      }
      
      return "calibrating_" + timeDisplay + "_of_" + String(state->calibrationPeriodHours) + "h_remaining";
    } else {
      return "calibrating_complete_" + String(state->calibrationPeriodHours) + "h_total";
    }
  }
  
  return "calibrated";
}

// Check if specific sensor data should be published
bool shouldPublishSensorData(const String& sensorName) {
  if (!ENABLE_CALIBRATION_STATE_TRACKING) {
    return true; // Always publish if tracking is disabled
  }
  
  // Don't publish during global testing mode
  if (sensorCalibrationStates.globalTestingMode) {
    return false;
  }
  
  SensorCalibrationState* state = getSensorCalibrationState(sensorName);
  if (state == nullptr) {
    return true; // Unknown sensor, allow publication
  }
  
  // Don't publish during calibration if suppression is enabled for this sensor
  if (state->isCalibrating && state->dataSuppressionActive) {
    return false;
  }

  return true;
}

// Check if specific sensor data should be published (measurement-type aware)
// This version allows basic environmental readings (temp, humidity, pressure) to always publish,
// while still suppressing calibration-sensitive readings (CO2, VOC, NOx, gas resistance)
bool shouldPublishSensorData(const String& sensorName, const char* measurementType) {
  if (!ENABLE_CALIBRATION_STATE_TRACKING) {
    return true; // Always publish if tracking is disabled
  }

  // Don't publish during global testing mode
  if (sensorCalibrationStates.globalTestingMode) {
    return false;
  }

  // Basic environmental readings should ALWAYS be published regardless of calibration state
  // These measurements don't require calibration periods - they work immediately
  if (strcmp(measurementType, "temperature") == 0 ||
      strcmp(measurementType, "humidity") == 0 ||
      strcmp(measurementType, "pressure") == 0 ||
      strcmp(measurementType, "pm1") == 0 ||
      strcmp(measurementType, "pm2_5") == 0 ||
      strcmp(measurementType, "pm10") == 0) {
    return true;  // Always publish basic environmental data
  }

  // Calibration-sensitive readings: CO2, VOC, NOx, gas_resistance
  // These require sensor stabilization and should respect calibration suppression
  SensorCalibrationState* state = getSensorCalibrationState(sensorName);
  if (state == nullptr) {
    return true; // Unknown sensor, allow publication
  }

  // Don't publish calibration-sensitive data if suppression is active
  if (state->isCalibrating && state->dataSuppressionActive) {
    return false;
  }

  return true;
}

// Legacy function for backward compatibility
bool shouldPublishSensorData() {
  // Return false if ANY sensor is in testing mode, true otherwise
  return !sensorCalibrationStates.globalTestingMode;
}

// Toggle global testing mode (via MQTT command)
void setTestingMode(bool enabled) {
  if (!ENABLE_TESTING_MODE) {
    Serial.println("Testing mode is disabled in configuration");
    return;
  }
  
  sensorCalibrationStates.globalTestingMode = enabled;
  
  if (enabled) {
    Serial.println("Global testing mode ENABLED - ALL sensor data will NOT be published");
    Serial.println("Use MQTT command to disable testing mode when ready");
  } else {
    Serial.println("Global testing mode DISABLED - sensor data publication resumed");
  }
}

// Reset calibration state for specific sensor or all sensors
void resetCalibrationState(const String& sensorName = "all") {
  if (!ENABLE_CALIBRATION_STATE_TRACKING) {
    Serial.println("Calibration tracking is disabled in configuration");
    return;
  }
  
  String sensors[] = {"scd4x", "scd30", "cm1106c", "c8co2", "sgp41", "bme680", "pms5003", "bmp280", "sht4x", "aht20", "tmp117"};

  if (sensorName == "all") {
    Serial.println("Resetting calibration state for ALL sensors...");
    Serial.println("📍 Use cases: Indoor→Outdoor deployment, location change, altitude change, sensor replacement");
    
    for (String sensor : sensors) {
      SensorCalibrationState* state = getSensorCalibrationState(sensor);
      if (state != nullptr) {
        state->isCalibrating = false;
        state->calibrationStartTime = 0;
        state->calibrationPeriodHours = 0;
        state->dataSuppressionActive = false;
        
        Serial.print(""); Serial.print(sensor); Serial.println(": Calibration state reset");
      }
    }
    
    // Clear persistent storage
    if (PERSIST_CALIBRATION_STATE) {
      calibrationPrefs.begin("cal_state", false);
      calibrationPrefs.clear();
      calibrationPrefs.end();
      Serial.println("Persistent calibration state cleared from flash memory");
    }
    
    Serial.println("All sensor calibration states reset - ready for re-deployment");
    Serial.println("Next startup will begin fresh calibration periods for applicable sensors");
    
  } else {
    // Reset specific sensor
    SensorCalibrationState* state = getSensorCalibrationState(sensorName);
    if (state != nullptr) {
      state->isCalibrating = false;
      state->calibrationStartTime = 0;
      state->calibrationPeriodHours = 0;
      state->dataSuppressionActive = false;
      
      Serial.print(""); Serial.print(sensorName); Serial.println(": Calibration state reset");
      
      // Save updated state
      saveCalibrationState();
      
      Serial.print(""); Serial.print(sensorName); Serial.println(" calibration state reset - ready for fresh calibration");
    } else {
      Serial.print("Unknown sensor: "); Serial.println(sensorName);
      Serial.println("Available sensors: scd4x, sgp41, bme680, pms5003, bmp280, sht4x, aht20");
    }
  }
}

// Start fresh calibration for sensors that don't have saved state
void startFreshCalibrationForUncalibratedSensors() {
  // CRITICAL PROTECTION: Check if we're about to overwrite valuable calibration data
  if (shouldPreventCalibrationReset("startFreshCalibrationForUncalibratedSensors called")) {
    Serial.println("Fresh calibration startup BLOCKED - trying to restore from backups instead");
    
    // Try to restore from EEPROM backup instead of starting fresh
    unsigned long currentTime = getCurrentUnixTime();
    if (currentTime == 0) currentTime = millis() / 1000 + 1640000000;
    
    if (loadCalibrationFromEEPROMBackup(currentTime)) {
      Serial.println("Valuable calibration data restored from backup!");
      return;
    } else {
      Serial.println("Could not restore backup data - manual intervention required");
      return; // Do not start fresh calibration
    }
  }
  
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.println("Checking which sensors need fresh calibration periods...");
    Serial.println("NOTE: Fresh calibration will only start for sensors without existing calibration state");
    Serial.print("Sensor availability: SCD4x="); Serial.print(scd4xAvailable ? "YES" : "NO");
    Serial.print(", SCD30="); Serial.print(scd30Available ? "YES" : "NO");
    Serial.print(", CM1106-C="); Serial.print(cm1106cAvailable ? "YES" : "NO");
    Serial.print(", SGP41="); Serial.print(sgp41Available ? "YES" : "NO");
    Serial.print(", BME680="); Serial.print(bme680Available ? "YES" : "NO");
    Serial.print(", PMS5003="); Serial.println(pms5003Available ? "YES" : "NO");
  }
  
  // Only start calibration for sensors that are available but not already calibrating
  // For SCD4x, only start calibration if ASC is enabled (calibration tracking only makes sense with ASC)
  if (scd4xAvailable && getCalibrationPeriod("scd4x") > 0 && ENABLE_SCD4X_ASC_BY_DEFAULT) {
    SensorCalibrationState* state = getSensorCalibrationState("scd4x");
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("SCD4x: available=YES, period="); Serial.print(getCalibrationPeriod("scd4x"));
      Serial.print("h, ASC_enabled="); Serial.print(ENABLE_SCD4X_ASC_BY_DEFAULT ? "YES" : "NO");
      Serial.print(", state="); Serial.print(state != nullptr ? "EXISTS" : "NULL");
      if (state != nullptr) {
        Serial.print(", isCalibrating="); Serial.println(state->isCalibrating ? "YES" : "NO");
      } else {
        Serial.println(", state=NULL");
      }
    }
    
    if (state != nullptr && !state->isCalibrating) {
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("SCD4x: No saved calibration state - starting fresh ASC calibration period");
      }
      startSensorCalibrationPeriod("scd4x");
    } else if (state != nullptr && state->isCalibrating) {
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("SCD4x: Continuing saved ASC calibration state");
      }
    } else if (state == nullptr) {
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("SCD4x: Sensor state is NULL - cannot start calibration!");
      }
    }
  } else {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("SCD4x: available="); Serial.print(scd4xAvailable ? "YES" : "NO");
      Serial.print(", period="); Serial.print(getCalibrationPeriod("scd4x"));
      Serial.print("h, ASC_enabled="); Serial.print(ENABLE_SCD4X_ASC_BY_DEFAULT ? "YES" : "NO");
      if (!ENABLE_SCD4X_ASC_BY_DEFAULT) {
        Serial.println(" - calibration tracking DISABLED (ASC off, use manual calibration)");
      } else {
        Serial.println(" - calibration will NOT start");
      }
    }
  }

  // SCD30 - similar to SCD4x, only start calibration if ASC is enabled
  if (scd30Available && getCalibrationPeriod("scd30") > 0 && ENABLE_SCD30_ASC_BY_DEFAULT) {
    SensorCalibrationState* state = getSensorCalibrationState("scd30");
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("SCD30: available=YES, period="); Serial.print(getCalibrationPeriod("scd30"));
      Serial.print("h, ASC_enabled="); Serial.print(ENABLE_SCD30_ASC_BY_DEFAULT ? "YES" : "NO");
      Serial.print(", state="); Serial.print(state != nullptr ? "EXISTS" : "NULL");
      if (state != nullptr) {
        Serial.print(", isCalibrating="); Serial.println(state->isCalibrating ? "YES" : "NO");
      } else {
        Serial.println(", state=NULL");
      }
    }

    if (state != nullptr && !state->isCalibrating) {
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("SCD30: No saved calibration state - starting fresh ASC calibration period");
      }
      startSensorCalibrationPeriod("scd30");
    } else if (state != nullptr && state->isCalibrating) {
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("SCD30: Continuing saved ASC calibration state");
      }
    }
  } else if (scd30Available) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("SCD30: available=YES, period="); Serial.print(getCalibrationPeriod("scd30"));
      Serial.print("h, ASC_enabled="); Serial.print(ENABLE_SCD30_ASC_BY_DEFAULT ? "YES" : "NO");
      if (!ENABLE_SCD30_ASC_BY_DEFAULT) {
        Serial.println(" - calibration tracking DISABLED (ASC off, use manual calibration)");
      }
    }
  }

  // CM1106-C - similar to SCD4x/SCD30, only start calibration if ABC is enabled
  if (cm1106cAvailable && getCalibrationPeriod("cm1106c") > 0 && ENABLE_CM1106C_ABC_BY_DEFAULT) {
    SensorCalibrationState* state = getSensorCalibrationState("cm1106c");
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("CM1106-C: available=YES, period="); Serial.print(getCalibrationPeriod("cm1106c"));
      Serial.print("h, ABC_enabled="); Serial.print(ENABLE_CM1106C_ABC_BY_DEFAULT ? "YES" : "NO");
      Serial.print(", state="); Serial.print(state != nullptr ? "EXISTS" : "NULL");
      if (state != nullptr) {
        Serial.print(", isCalibrating="); Serial.println(state->isCalibrating ? "YES" : "NO");
      } else {
        Serial.println(", state=NULL");
      }
    }

    if (state != nullptr && !state->isCalibrating) {
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("CM1106-C: No saved calibration state - starting fresh ABC calibration period (15 days)");
      }
      startSensorCalibrationPeriod("cm1106c");
    } else if (state != nullptr && state->isCalibrating) {
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("CM1106-C: Continuing saved ABC calibration state");
      }
    }
  } else if (cm1106cAvailable) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("CM1106-C: available=YES, period="); Serial.print(getCalibrationPeriod("cm1106c"));
      Serial.print("h, ABC_enabled="); Serial.print(ENABLE_CM1106C_ABC_BY_DEFAULT ? "YES" : "NO");
      if (!ENABLE_CM1106C_ABC_BY_DEFAULT) {
        Serial.println(" - calibration tracking DISABLED (ABC off, use manual calibration)");
      }
    }
  }

  if (sgp41Available && getCalibrationPeriod("sgp41") > 0) {
    SensorCalibrationState* state = getSensorCalibrationState("sgp41");
    if (state != nullptr && !state->isCalibrating) {
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("SGP41: No saved calibration state - starting fresh calibration period");
      }
      startSensorCalibrationPeriod("sgp41");
    } else if (state != nullptr && state->isCalibrating) {
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("SGP41: Continuing saved calibration state");
      }
    }
  }

  if (bme680Available && getCalibrationPeriod("bme680") > 0) {
    SensorCalibrationState* state = getSensorCalibrationState("bme680");
    // BME680 ALWAYS starts fresh - Bosch provides no baseline save/restore mechanism
    if (state != nullptr && !state->isCalibrating) {
      Serial.println("BME680: Starting fresh 48-hour stabilisation period (no baseline save mechanism available)");
      startSensorCalibrationPeriod("bme680");
    } else if (state != nullptr && state->isCalibrating) {
      // This should never happen due to continue in loadCalibrationState()
      Serial.println("BME680: WARNING - state shows calibrating but this should have been reset!");
      Serial.println("BME680: Forcing fresh 48-hour stabilisation period (Bosch provides no baseline save mechanism)");
      state->isCalibrating = false; // Force reset
      startSensorCalibrationPeriod("bme680");
    }
  }
  
  if (pms5003Available && getCalibrationPeriod("pms5003") > 0) {
    SensorCalibrationState* state = getSensorCalibrationState("pms5003");
    if (state != nullptr && !state->isCalibrating) {
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("PMS5003: No saved calibration state - starting fresh calibration period");
      }
      startSensorCalibrationPeriod("pms5003");
    } else if (state != nullptr && state->isCalibrating) {
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("PMS5003: Continuing saved calibration state");
      }
    }
  }
  
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.println("Fresh calibration check complete");
  }
}

// Start fresh calibration for sensors after reset
void startFreshCalibration() {
  Serial.println("Starting fresh calibration periods for applicable sensors...");

  // Only start calibration for sensors that actually need it and are available
  // For SCD4x, only start calibration if ASC is enabled (calibration tracking only makes sense with ASC)
  if (scd4xAvailable && getCalibrationPeriod("scd4x") > 0 && ENABLE_SCD4X_ASC_BY_DEFAULT) {
    startSensorCalibrationPeriod("scd4x");
  } else if (scd4xAvailable && !ENABLE_SCD4X_ASC_BY_DEFAULT) {
    Serial.println("SCD4x: Calibration tracking skipped - ASC disabled, use manual calibration via MQTT");
  }

  // SCD30 - same as SCD4x
  if (scd30Available && getCalibrationPeriod("scd30") > 0 && ENABLE_SCD30_ASC_BY_DEFAULT) {
    startSensorCalibrationPeriod("scd30");
  } else if (scd30Available && !ENABLE_SCD30_ASC_BY_DEFAULT) {
    Serial.println("SCD30: Calibration tracking skipped - ASC disabled, use manual calibration via MQTT");
  }

  // CM1106-C - similar pattern with ABC
  if (cm1106cAvailable && getCalibrationPeriod("cm1106c") > 0 && ENABLE_CM1106C_ABC_BY_DEFAULT) {
    startSensorCalibrationPeriod("cm1106c");
  } else if (cm1106cAvailable && !ENABLE_CM1106C_ABC_BY_DEFAULT) {
    Serial.println("CM1106-C: Calibration tracking skipped - ABC disabled, use manual calibration via MQTT");
  }

  if (sgp41Available && getCalibrationPeriod("sgp41") > 0) {
    startSensorCalibrationPeriod("sgp41");
  }

  if (bme680Available && getCalibrationPeriod("bme680") > 0) {
    startSensorCalibrationPeriod("bme680");
  }

  if (pms5003Available && getCalibrationPeriod("pms5003") > 0) {
    startSensorCalibrationPeriod("pms5003");
  }

  Serial.println("Fresh calibration periods started - sensors will adapt to new environment");
}

// Get overall calibration status summary
String getOverallCalibrationStatus() {
  if (!ENABLE_CALIBRATION_STATE_TRACKING) {
    return "tracking_disabled";
  }
  
  if (sensorCalibrationStates.globalTestingMode) {
    return "testing_mode";
  }
  
  int calibrating = 0;
  int calibrated = 0;
  String sensors[] = {"scd4x", "scd30", "cm1106c", "c8co2", "sgp41", "bme680", "pms5003", "bmp280", "sht4x", "aht20", "tmp117"};

  for (String sensorName : sensors) {
    SensorCalibrationState* state = getSensorCalibrationState(sensorName);
    if (state != nullptr) {
      if (state->isCalibrating) {
        calibrating++;
      } else {
        calibrated++;
      }
    }
  }
  
  if (calibrating > 0) {
    return "partial_calibration_" + String(calibrating) + "_sensors_calibrating";
  }
  
  return "all_calibrated";
}

// ======================
// CALIBRATION MONITORING & DIAGNOSTICS
// ======================

// Publish raw sensor reading during calibration for monitoring
void publishDiagnosticReading(const String& sensorName, const String& measurement, float value, const String& unit) {
  if (!ENABLE_CALIBRATION_MONITORING || !wifiConnected || !client.connected()) {
    return;
  }
  
  String topic = String(deviceTopicPrefix) + "/diagnostic/" + sensorName + "/" + measurement;
  String calibrationStatus = getSensorCalibrationStatus(sensorName);
  
  // Always include timestamps and metadata in diagnostic readings
  String timestamp = USE_UTC_TIMESTAMPS ? getTimestampUTC() : getTimestamp();
  String payload = "{\"value\":" + String(value, 2) + ",\"unit\":\"" + unit + "\",\"calibration_state\":\"" + calibrationStatus + "\",\"diagnostic\":true,\"timestamp\":\"" + timestamp + "\",\"device_id\":\"" + String(deviceID) + "\"}";
  
  client.publish(topic.c_str(), payload.c_str());
  
  if (ENABLE_DEBUG_OUTPUT) {
    String diagnosticMsg = "Diagnostic (" + calibrationStatus + "): " + sensorName + " " + measurement + " = " + String(value, 2);
    if (unit.length() > 0) {
      diagnosticMsg += " " + unit;
    }
    debugPrintln(diagnosticMsg);
  }
}

// Enhanced sensor data publishing with calibration monitoring
bool publishSensorDataWithMonitoring(const String& sensorName, const char* mqtt_topic, const String& measurement, float value, const String& unit) {
  // Use measurement-type aware check - basic environmental readings always publish
  bool shouldPublish = shouldPublishSensorData(sensorName, measurement.c_str());
  
  if (shouldPublish) {
    // Normal publication - use direct publishing to avoid recursion
    publishSensorDataWithCalibration(mqtt_topic, value, sensorName);
    return true;
  } else {
    // Data suppressed, but publish diagnostic if monitoring enabled
    if (ENABLE_CALIBRATION_MONITORING && PUBLISH_RAW_READINGS_DURING_CALIBRATION) {
      publishDiagnosticReading(sensorName, measurement, value, unit);
    }
    
    if (ENABLE_DEBUG_OUTPUT) {
      String suppressedMsg = "Data suppressed (" + getSensorCalibrationStatus(sensorName) + "): " + measurement + " from " + sensorName + " = " + String(value, 2);
      if (unit.length() > 0) {
        suppressedMsg += " " + unit;
      }
      suppressedMsg += " (diagnostic published)";
      debugPrintln(suppressedMsg);
    }
    return false;
  }
}

// Publish periodic calibration status summary
void publishCalibrationDiagnostics() {
  static bool lastPublishWasSuccessful = false;
  static unsigned long lastAttemptTime = 0;
  
  if (!ENABLE_CALIBRATION_MONITORING || !wifiConnected || !client.connected()) {
    return;
  }
  
  // Prevent multiple calls within the same second
  unsigned long currentTime = millis();
  if (currentTime - lastAttemptTime < 1000) {
    return;
  }
  lastAttemptTime = currentTime;
  
  // Check if it's time to publish diagnostics
  if (currentTime - lastDiagnosticPublishTime < DIAGNOSTIC_PUBLISH_INTERVAL_MS) {
    return;
  }
  
  // Check if any sensors are calibrating
  String sensors[] = {"scd4x", "scd30", "cm1106c", "c8co2", "sgp41", "bme680", "pms5003", "tmp117"};
  bool anyCalibrating = false;
  int calibratingCount = 0;

  for (String sensorName : sensors) {
    SensorCalibrationState* state = getSensorCalibrationState(sensorName);
    if (state != nullptr && state->isCalibrating) {
      anyCalibrating = true;
      calibratingCount++;
    }
  }
  
  if (!anyCalibrating && !sensorCalibrationStates.globalTestingMode) {
    // Only log when we stop publishing (state change)
    if (lastPublishWasSuccessful && ENABLE_DEBUG_OUTPUT) {
      Serial.println("Calibration diagnostics: All sensors completed calibration - stopping periodic reports");
      lastPublishWasSuccessful = false;
    }
    return; // No need to publish diagnostics
  }
  
  lastDiagnosticPublishTime = currentTime;
  
  String topic = String(deviceTopicPrefix) + "/diagnostic/calibration_summary";
  String payload = "{";
  
  for (String sensorName : sensors) {
    SensorCalibrationState* state = getSensorCalibrationState(sensorName);
    if (state != nullptr) {
      String status = getSensorCalibrationStatus(sensorName);
      bool dataActive = shouldPublishSensorData(sensorName);
      
      payload += "\"" + sensorName + "\":{";
      payload += "\"status\":\"" + status + "\",";
      payload += "\"data_publishing\":" + String(dataActive ? "true" : "false");
      
      if (state->isCalibrating) {
        unsigned long remainingMs = getRemainingCalibrationMs(sensorName);
        unsigned long elapsedMs = millis() - state->calibrationStartTime;
        unsigned long hours, minutes;
        
        calculateRemainingTime(remainingMs, hours, minutes);
        payload += ",\"remaining_hours\":" + String(hours);
        payload += ",\"remaining_minutes\":" + String(minutes);
        payload += ",\"remaining_total_minutes\":" + String(remainingMs / 60000);
        payload += ",\"total_hours\":" + String(state->calibrationPeriodHours);
        payload += ",\"elapsed_minutes\":" + String(elapsedMs / 60000);
      }
      
      payload += "},";
    }
  }
  
  if (payload.endsWith(",")) {
    payload = payload.substring(0, payload.length() - 1);
  }
  
  String timestamp = USE_UTC_TIMESTAMPS ? getTimestampUTC() : getTimestamp();
  payload += ",\"timestamp\":\"" + timestamp + "\"";
  payload += ",\"device_id\":\"" + String(deviceID) + "\"";
  
  payload += "}";
  
  client.publish(topic.c_str(), payload.c_str());
  
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("Calibration diagnostics published for ");
    Serial.print(calibratingCount);
    Serial.print(" sensor");
    if (calibratingCount != 1) Serial.print("s");
    Serial.println(" (every 30s during calibration)");
  }
  
  lastPublishWasSuccessful = true;
}

// Handle calibration reset command
void handleCalibrationReset(const String& payload) {
  String sensorName = payload;
  sensorName.trim();
  
  if (sensorName.length() == 0 || sensorName == "all") {
    resetCalibrationState("all");
    publishCommandResult("SUCCESS", "All sensor calibration states reset. Device ready for re-deployment (indoor→outdoor, location change, altitude change, etc.)", "system/calibration_reset");
  } else {
    resetCalibrationState(sensorName);
    if (getSensorCalibrationState(sensorName) != nullptr) {
      publishCommandResult("SUCCESS", "Calibration state reset for " + sensorName + ". Sensor ready for fresh calibration period.", "system/calibration_reset");
    } else {
      publishCommandResult("ERROR", "Unknown sensor: " + sensorName + ". Valid sensors: scd4x, scd30, cm1106c, sgp41, bme680, pms5003, bmp280, sht4x, aht20", "system/calibration_reset");
    }
  }
}

// Handle calibration restart command (reset + immediately start fresh calibration)
void handleCalibrationRestart(const String& payload) {
  String sensorName = payload;
  sensorName.trim();
  
  if (sensorName.length() == 0 || sensorName == "all") {
    resetCalibrationState("all");
    startFreshCalibration();
    publishCommandResult("SUCCESS", "All calibration reset and restarted. Fresh calibration periods active for applicable sensors.", "system/calibration_restart");
  } else {
    resetCalibrationState(sensorName);
    if (getSensorCalibrationState(sensorName) != nullptr) {
      // Start calibration for specific sensor
      unsigned long period = getCalibrationPeriod(sensorName);
      if (period > 0) {
        if ((sensorName == "scd4x" && scd4xAvailable) ||
            (sensorName == "scd30" && scd30Available) ||
            (sensorName == "cm1106c" && cm1106cAvailable) ||
            (sensorName == "sgp41" && sgp41Available) ||
            (sensorName == "bme680" && bme680Available) ||
            (sensorName == "pms5003" && pms5003Available)) {
          startSensorCalibrationPeriod(sensorName);
          publishCommandResult("SUCCESS", "Calibration reset and restarted for " + sensorName + ". Fresh " + String(period) + "h calibration period active.", "system/calibration_restart");
        } else {
          publishCommandResult("INFO", "Calibration reset for " + sensorName + ", but sensor not available or doesn't require calibration period.", "system/calibration_restart");
        }
      } else {
        publishCommandResult("INFO", "Calibration reset for " + sensorName + ", but this sensor doesn't have a calibration period configured.", "system/calibration_restart");
      }
    } else {
      publishCommandResult("ERROR", "Unknown sensor: " + sensorName + ". Valid sensors: scd4x, scd30, cm1106c, sgp41, bme680, pms5003, bmp280, sht4x, aht20", "system/calibration_restart");
    }
  }
}

// ======================
// HELPER FUNCTIONS
// ======================

// --- Deep Sleep Helper Functions ---

// Check if we woke from deep sleep timer
bool isWakeFromDeepSleep() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  return (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) && rtc_deepSleepActive;
}

// Get human-readable wake reason string
const char* getWakeReasonString() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_TIMER:     return "TIMER";
    case ESP_SLEEP_WAKEUP_EXT0:      return "EXT0";
    case ESP_SLEEP_WAKEUP_EXT1:      return "EXT1";
    case ESP_SLEEP_WAKEUP_TOUCHPAD:  return "TOUCHPAD";
    case ESP_SLEEP_WAKEUP_ULP:       return "ULP";
    default:                         return "COLD_BOOT";
  }
}

// Calculate warmup time based on connected sensors
unsigned long calculateWarmupTime() {
  unsigned long warmup = SENSOR_WARMUP_MS;  // Base 10s for SCD4x

  // Extended warmup for PM sensors (fan spin-up)
  if (pms5003Available) {
    warmup = max(warmup, PM_SENSOR_WARMUP_MS);  // 30s for PM sensors
  }

  // SCD30 NDIR needs longer warmup than SCD4x PAS
  if (scd30Available && !scd4xAvailable) {
    warmup = max(warmup, 30000UL);  // 30s for SCD30
  }

  return warmup;
}

// Calculate sleep duration based on how long we've been awake
unsigned long calculateSleepDuration(unsigned long awakeTimeMs) {
  if (awakeTimeMs >= LORAWAN_TX_INTERVAL_MS) {
    return 1000;  // Minimum 1 second if we've been awake longer than interval
  }
  return LORAWAN_TX_INTERVAL_MS - awakeTimeMs;
}

// Check if deep sleep is valid for current configuration
bool isDeepSleepValid() {
  if (!ENABLE_DEEP_SLEEP) return false;

  // Must have LoRaWAN enabled
  bool lorawanEnabled = HAS_LORA() && !DISABLE_LORAWAN;
  if (!lorawanEnabled) return false;

  // WiFi must be disabled or WIFI_STOP_AFTER_TIME_SYNC enabled
  bool wifiDisabledOrStopped = !ENABLE_WIFI || WIFI_STOP_AFTER_TIME_SYNC;
  if (!wifiDisabledOrStopped) return false;

  return true;
}

// Log deep sleep sensor compatibility warnings
void logDeepSleepSensorWarnings() {
  if (!ENABLE_DEEP_SLEEP) return;

  Serial.println(F("\n=== DEEP SLEEP SENSOR COMPATIBILITY ==="));

  if (sgp41Available) {
    Serial.println(F("WARNING: SGP41 detected - VOC/NOx readings DISABLED"));
    Serial.println(F("  Reason: Gas index algorithm requires continuous operation"));
  }

  if (bme680Available) {
    Serial.println(F("WARNING: BME680 detected - Gas resistance DISABLED"));
    Serial.println(F("  Note: Temperature, Humidity, Pressure readings ARE enabled"));
  }

  Serial.print(F("Warmup time: "));
  Serial.print(calculateWarmupTime());
  Serial.println(F(" ms"));

  Serial.print(F("TX interval: "));
  Serial.print(LORAWAN_TX_INTERVAL_MS / 1000);
  Serial.println(F(" seconds"));

  Serial.println(F("==========================================\n"));
}

// --- LoRaWAN Session Persistence Functions ---

// Save LoRaWAN session to RTC memory and nonces to NVS flash
void saveLoRaWANSession() {
  if (!ENABLE_DEEP_SLEEP || node == nullptr) return;

  // Get session buffer from RadioLib and save to RTC memory
  uint8_t* sessionBuffer = node->getBufferSession();
  if (sessionBuffer != nullptr) {
    memcpy(rtc_lorawanSession, sessionBuffer, sizeof(rtc_lorawanSession));
    rtc_lorawanSessionValid = true;
    Serial.println(F("[DeepSleep] LoRaWAN session saved to RTC memory"));
  }

  // Save nonces to NVS flash (must survive power loss to avoid DevNonce reuse)
  uint8_t* noncesBuffer = node->getBufferNonces();
  if (noncesBuffer != nullptr) {
    Preferences prefs;
    prefs.begin("lorawan_ds", false);  // Deep sleep namespace
    prefs.putBytes("nonces", noncesBuffer, 64);
    prefs.end();
    Serial.println(F("[DeepSleep] LoRaWAN nonces saved to NVS flash"));
  }
}

// Restore LoRaWAN session from RTC memory and nonces from NVS flash
// Returns true if session was successfully restored
bool restoreLoRaWANSession() {
  if (!ENABLE_DEEP_SLEEP || node == nullptr) return false;
  if (!rtc_lorawanSessionValid) {
    Serial.println(F("[DeepSleep] No valid LoRaWAN session in RTC memory"));
    return false;
  }

  // Load nonces from NVS flash
  Preferences prefs;
  prefs.begin("lorawan_ds", true);  // Read-only
  if (!prefs.isKey("nonces")) {
    prefs.end();
    Serial.println(F("[DeepSleep] No LoRaWAN nonces found in NVS"));
    rtc_lorawanSessionValid = false;
    return false;
  }

  uint8_t noncesBuffer[64];
  size_t noncesLen = prefs.getBytes("nonces", noncesBuffer, 64);
  prefs.end();

  if (noncesLen != 64) {
    Serial.println(F("[DeepSleep] Invalid nonces length in NVS"));
    rtc_lorawanSessionValid = false;
    return false;
  }

  // Restore buffers to RadioLib
  node->setBufferNonces(noncesBuffer);
  node->setBufferSession(rtc_lorawanSession);

  // Activate the restored session
  Serial.println(F("[DeepSleep] Attempting to activate restored LoRaWAN session..."));
  int16_t state = node->activateOTAA();

  if (state >= RADIOLIB_ERR_NONE) {
    Serial.println(F("[DeepSleep] LoRaWAN session restored successfully - skipping OTAA join"));
    return true;
  } else {
    Serial.print(F("[DeepSleep] Session restore failed, error: "));
    Serial.println(state);
    rtc_lorawanSessionValid = false;
    return false;
  }
}

// Clear saved LoRaWAN session (call on cold boot or after errors)
void clearLoRaWANSession() {
  rtc_lorawanSessionValid = false;
  memset(rtc_lorawanSession, 0, sizeof(rtc_lorawanSession));

  Preferences prefs;
  prefs.begin("lorawan_ds", false);
  prefs.clear();
  prefs.end();

  Serial.println(F("[DeepSleep] LoRaWAN session cleared"));
}

// --- Deep Sleep Entry Function ---

// Enter deep sleep for specified duration (milliseconds)
// Call this after successful LoRaWAN transmission
void enterDeepSleep(unsigned long sleepDurationMs) {
  // Validate deep sleep is allowed
  if (!isDeepSleepValid()) {
    Serial.println(F("[DeepSleep] Deep sleep not valid for current config - skipping"));
    return;
  }

  // Ensure minimum sleep duration (1 second)
  if (sleepDurationMs < 1000) {
    sleepDurationMs = 1000;
  }

  Serial.println(F("\n========== ENTERING DEEP SLEEP =========="));
  Serial.print(F("[DeepSleep] Sleep duration: "));
  Serial.print(sleepDurationMs / 1000);
  Serial.println(F(" seconds"));

  // Save LoRaWAN session before sleeping
  saveLoRaWANSession();

  // Update RTC tracking variables
  rtc_wakeCount++;
  rtc_deepSleepActive = true;
  rtc_lastTxTimestamp = millis();

  Serial.print(F("[DeepSleep] Total wake cycles: "));
  Serial.println(rtc_wakeCount);

  // Disconnect WiFi if it was enabled
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println(F("[DeepSleep] WiFi disconnected"));
  }

  // Put LoRa radio to sleep (if available)
  // RadioLib should handle this, but ensure low power state
  if (HAS_LORA()) {
    if (radio_sx1262 != nullptr) {
      radio_sx1262->sleep();
      Serial.println(F("[DeepSleep] SX1262 radio in sleep mode"));
    }
    if (radio_sx1276 != nullptr) {
      radio_sx1276->sleep();
      Serial.println(F("[DeepSleep] SX1276 radio in sleep mode"));
    }
  }

  // Disable GPIO pins to save power (optional - depends on board)
  // Note: RTC GPIOs remain active for wake sources

  // Configure wake timer
  uint64_t sleepDurationUs = (uint64_t)sleepDurationMs * 1000ULL;
  esp_sleep_enable_timer_wakeup(sleepDurationUs);

  Serial.println(F("[DeepSleep] Timer configured, entering deep sleep..."));
  Serial.println(F("==========================================\n"));
  Serial.flush();  // Ensure all serial output is sent

  // Small delay to ensure everything is flushed
  delay(POST_TX_DELAY_MS);

  // Enter deep sleep - this function does not return
  // On wake, the ESP32 restarts from the beginning of setup()
  esp_deep_sleep_start();

  // Code never reaches here
}

// Calculate remaining time in hours and minutes from milliseconds
void calculateRemainingTime(unsigned long remainingMs, unsigned long &hours, unsigned long &minutes) {
  hours = remainingMs / 3600000;  // Convert to hours
  minutes = (remainingMs % 3600000) / 60000;  // Remaining minutes
}

// Format remaining time as human-readable string
String formatRemainingTime(unsigned long remainingMs) {
  unsigned long hours, minutes;
  calculateRemainingTime(remainingMs, hours, minutes);
  
  if (hours > 0 && minutes > 0) {
    return String(hours) + "h " + String(minutes) + "m";
  } else if (hours > 0) {
    return String(hours) + "h";
  } else if (minutes > 0) {
    return String(minutes) + "m";
  } else {
    return "<1m";
  }
}

// Get remaining calibration time in milliseconds for a sensor
unsigned long getRemainingCalibrationMs(const String& sensorName) {
  SensorCalibrationState* state = getSensorCalibrationState(sensorName);
  if (state == nullptr || !state->isCalibrating) {
    return 0;
  }
  
  unsigned long elapsedMs = millis() - state->calibrationStartTime;
  unsigned long totalMs = state->calibrationPeriodHours * 3600000;
  
  if (elapsedMs >= totalMs) {
    return 0;
  }
  
  return totalMs - elapsedMs;
}

uint16_t convertTempToTicks(float temperatureC) {
  return (uint16_t)((temperatureC + 45.0f) * 65535.0f / 175.0f);
}

uint16_t convertRHToTicks(float relativeHumidity) {
  return (uint16_t)(relativeHumidity * 65535.0f / 100.0f);
}

// =============================================================================
// CM1106-C UART CO2 Sensor - Proper Protocol Implementation
// Based on ESPHome CM1106 component and Cubic datasheet
// Protocol: Send 0x11 0x01 0x01 0xED, receive 8-byte response
// =============================================================================

// Calculate CM1106 checksum: 256 - (sum of HEAD+LEN+CMD+DATA) % 256
uint8_t cm1106UartChecksum(uint8_t* data, uint8_t len) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len; i++) {  // Include all bytes from HEAD onwards
    sum += data[i];
  }
  return 256 - sum;  // Wraps naturally for uint8_t
}

// Send read CO2 command to CM1106-C
void sendCM1106ReadCommand() {
  // CM1106 read command: 0x11 0x01 0x01 0xED
  uint8_t cmd[4] = {0x11, 0x01, 0x01, 0xED};
  PMS5003Serial.write(cmd, 4);
  PMS5003Serial.flush();
}

// Read CO2 data from CM1106-C using proper protocol
bool readCM1106Data(C8CO2Data &data) {
  const uint8_t RESPONSE_LENGTH = 8;
  uint8_t response[RESPONSE_LENGTH];

  data.valid = false;
  data.co2_ppm = 0;
  data.co2_ppm_alt = 0;
  data.status = 0;

  // Clear RX buffer
  while (PMS5003Serial.available() > 0) {
    PMS5003Serial.read();
  }

  // Send read command
  sendCM1106ReadCommand();

  // Wait for response (up to 500ms)
  unsigned long timeout = millis() + 500;
  while (PMS5003Serial.available() < RESPONSE_LENGTH && millis() < timeout) {
    delay(10);
  }

  if (PMS5003Serial.available() < RESPONSE_LENGTH) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("CM1106-C: Timeout, only got ");
      Serial.print(PMS5003Serial.available());
      Serial.println(" bytes");
    }
    return false;
  }

  // Read response
  for (int i = 0; i < RESPONSE_LENGTH; i++) {
    response[i] = PMS5003Serial.read();
  }

  // Validate header: 0x16 0x05 0x01
  if (response[0] != 0x16 || response[1] != 0x05 || response[2] != 0x01) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("CM1106-C: Invalid header: 0x");
      Serial.print(response[0], HEX);
      Serial.print(" 0x");
      Serial.print(response[1], HEX);
      Serial.print(" 0x");
      Serial.println(response[2], HEX);
    }
    return false;
  }

  // Validate checksum (byte 7 should be 256 - sum of bytes 1-6)
  uint8_t expectedChecksum = cm1106UartChecksum(response, 7);
  if (response[7] != expectedChecksum) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("CM1106-C: Checksum mismatch. Got 0x");
      Serial.print(response[7], HEX);
      Serial.print(", expected 0x");
      Serial.println(expectedChecksum, HEX);
    }
    // Continue anyway - some sensors have checksum quirks
  }

  // Extract CO2: bytes 3-4 (high byte, low byte)
  uint16_t co2 = (response[3] << 8) | response[4];

  // Validate range
  if (co2 < 0 || co2 > 10000) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("CM1106-C: CO2 out of range: ");
      Serial.println(co2);
    }
    return false;
  }

  data.co2_ppm = co2;
  data.co2_ppm_alt = co2;  // Same value for compatibility
  data.status = response[5];  // DF3
  data.checksum = response[7];
  data.valid = true;

  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("CM1106-C: CO2 = ");
    Serial.print(co2);
    Serial.print(" ppm, DF3=0x");
    Serial.print(response[5], HEX);
    Serial.print(", DF4=0x");
    Serial.println(response[6], HEX);
  }

  return true;
}

// Dump raw UART bytes for debugging
void dumpCM1106RawBytes() {
  Serial.println("CM1106-C: Raw byte dump (waiting 2s for data)...");

  // Clear buffer
  while (PMS5003Serial.available() > 0) {
    PMS5003Serial.read();
  }

  delay(2000);  // Wait for sensor to send data

  int bytesAvailable = PMS5003Serial.available();
  Serial.print("CM1106-C: ");
  Serial.print(bytesAvailable);
  Serial.println(" bytes available");

  if (bytesAvailable > 0) {
    Serial.print("CM1106-C: Raw data: ");
    int count = 0;
    while (PMS5003Serial.available() > 0 && count < 50) {
      uint8_t b = PMS5003Serial.read();
      if (b < 0x10) Serial.print("0");
      Serial.print(b, HEX);
      Serial.print(" ");
      count++;
    }
    Serial.println();
  }
}

// Detect CM1106-C sensor on UART
bool detectCM1106UART() {
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.println("CM1106-C: Attempting detection...");
    // First dump raw bytes to see what sensor is sending
    dumpCM1106RawBytes();
  }

  // Try reading data multiple times using command protocol
  for (int attempt = 0; attempt < 3; attempt++) {
    if (readCM1106Data(c8co2Data)) {
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.print("CM1106-C: Detected with reading: ");
        Serial.print(c8co2Data.co2_ppm);
        Serial.println(" ppm");
      }
      return true;
    }
    delay(300);
  }

  if (ENABLE_DEBUG_OUTPUT) {
    Serial.println("CM1106-C: Command protocol failed, sensor may need TX/RX swap or use different protocol");
  }
  return false;
}

// =============================================================================
// CM1106-C command protocol disabled - sensor uses Plantower-style streaming
// Raw dump showed: 42 4D 0B 98 09 5D 02 03 01 2D 00 00 E2 00 06 B3
// This is 16-byte Plantower framing, NOT CM1106-C command-response protocol
// =============================================================================
#if 0
// Wrapper for compatibility with existing code
bool readC8CO2Data(C8CO2Data &data) {
  return readCM1106Data(data);
}

bool detectC8CO2() {
  return detectCM1106UART();
}
#endif

// =============================================================================
// UART CO2 Sensor - Plantower-style 16-byte framing
// Uses passive streaming protocol (42 4D header)
// Raw dump: 42 4D 0B 98 09 5D 02 03 01 2D 00 00 E2 00 06 B3
// CO2 is at bytes 6-7: 0x0203 = 515 ppm
// =============================================================================

// Read CO2 data from UART sensor using Plantower-style framing
bool readC8CO2Data(C8CO2Data &data) {
  // This C8 uses fixed 16-byte frames with 0x42 0x4D header
  // Frame: 0x42 0x4D + 2 bytes + CO2_HIGH + CO2_LOW + 10 bytes + CHECKSUM(2)
  const uint8_t FRAME_LENGTH = 16;
  uint8_t frame[FRAME_LENGTH];
  
  data.valid = false;
  data.co2_ppm = 0;
  data.status = 0;
  
  int bytesChecked = 0;
  int index = 0;
  bool frameFound = false;
  unsigned long timeout = millis() + 2000;
  
  // Look for 0x42 0x4D header
  while (millis() < timeout && PMS5003Serial.available() > 0) {
    uint8_t byte1 = PMS5003Serial.read();
    bytesChecked++;
    
    if (byte1 == 0x42 && PMS5003Serial.available() > 0) {
      uint8_t byte2 = PMS5003Serial.read();
      bytesChecked++;
      
      if (byte2 == 0x4D) {
        frame[0] = byte1;
        frame[1] = byte2;
        index = 2;
        frameFound = true;
        break;
      }
    }
  }
  
  if (!frameFound) {
    return false;
  }
  
  // Read the rest of the 16-byte frame
  timeout = millis() + 1000;
  while (index < FRAME_LENGTH && millis() < timeout) {
    if (PMS5003Serial.available() > 0) {
      frame[index++] = PMS5003Serial.read();
    }
  }
  
  if (index < FRAME_LENGTH) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("C8 CO₂: Incomplete frame. Got ");
      Serial.print(index);
      Serial.println("/16 bytes");
    }
    return false;
  }
  
  // Verify checksum (sum of all 16 bytes including checksum bytes)
  // Many sensors use: sum of all bytes = 0 or sum % 256 = 0
  uint16_t checksum = 0;
  for (int i = 0; i < 16; i++) {
    checksum += frame[i];
  }
  uint16_t receivedChecksum = (frame[14] << 8) | frame[15];
  
  // Try multiple checksum methods
  bool checksumValid = false;
  
  // Method 1: Sum of first 14 bytes equals last 2 bytes
  uint16_t sum14 = 0;
  for (int i = 0; i < 14; i++) {
    sum14 += frame[i];
  }
  if (sum14 == receivedChecksum) {
    checksumValid = true;
  }
  
  // Method 2: Sum of all 16 bytes % 256 == 0
  if ((checksum & 0xFF) == 0) {
    checksumValid = true;
  }
  
  // Method 3: Just skip checksum for now (sensor may use proprietary method)
  // We'll validate by checking if CO2 is in reasonable range
  checksumValid = true;  // Temporarily disable checksum validation
  
  if (!checksumValid && ENABLE_DEBUG_OUTPUT) {
    Serial.print("C8 CO₂: Checksum validation disabled. sum14: 0x");
    Serial.print(sum14, HEX);
    Serial.print(", recv: 0x");
    Serial.print(receivedChecksum, HEX);
    Serial.print(", sum16: 0x");
    Serial.println(checksum, HEX);
  }
  
  // Parse CO₂ values from frame
  // Based on raw dump analysis: CO2 is at bytes 6-7, not 4-5
  data.co2_ppm = (frame[4] << 8) | frame[5];     // Bytes 4-5 (not CO2, possibly other data)
  data.co2_ppm_alt = (frame[6] << 8) | frame[7]; // Bytes 6-7 (actual CO2 value)
  data.status = frame[8];
  data.checksum = receivedChecksum;

  // Validate CO₂ range using bytes 6-7 (the actual CO2 value)
  if (data.co2_ppm_alt < 300 || data.co2_ppm_alt > 5000) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("UART CO₂: CO₂ out of range: ");
      Serial.print(data.co2_ppm_alt);
      Serial.println(" ppm");
    }
    return false;
  }

  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("UART CO₂: Bytes[6-7]=");
    Serial.print(data.co2_ppm_alt);
    Serial.print(" ppm (Bytes[4-5]=");
    Serial.print(data.co2_ppm);
    Serial.println(")");
  }

  data.valid = true;
  return true;
}

// Detect UART CO₂ sensor (passive streaming, no commands needed)
bool detectC8CO2() {
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.println("UART CO₂: Detecting passive streaming sensor...");
  }

  // Wait for sensor to send data (passive streaming)
  delay(500);

  // Try reading data multiple times
  for (int attempt = 0; attempt < 3; attempt++) {
    if (readC8CO2Data(c8co2Data)) {
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.print("UART CO₂: Detected with reading: ");
        Serial.print(c8co2Data.co2_ppm_alt);
        Serial.println(" ppm");
      }
      return true;
    }
    delay(500);  // Wait for next frame
  }

  if (ENABLE_DEBUG_OUTPUT) {
    Serial.println("UART CO₂: Detection failed");
  }
  return false;
}

// PMS5003 helper functions
bool readPMS5003Data(PMS5003Data &data) {
  const uint8_t PMS5003_FRAME_LENGTH = 32;
  uint8_t buffer[PMS5003_FRAME_LENGTH];
  int index = 0;
  bool frameFound = false;
  unsigned long timeout = millis() + 2000; // 2 second timeout
  
  // Clear any existing data in the structure
  data.valid = false;
  
  // Look for frame header (0x42, 0x4D)
  int bytesChecked = 0;
  while (millis() < timeout && PMS5003Serial.available() > 0) {
    uint8_t byte1 = PMS5003Serial.read();
    bytesChecked++;
    if (byte1 == 0x42 && PMS5003Serial.available() > 0) {
      uint8_t byte2 = PMS5003Serial.read();
      bytesChecked++;
      if (byte2 == 0x4D) {
        buffer[0] = byte1;
        buffer[1] = byte2;
        index = 2;
        frameFound = true;
        break;
      }
    }
  }
  
  if (!frameFound) {
    if (ENABLE_DEBUG_OUTPUT && bytesChecked > 0) {
      Serial.print("PMS5003: No frame header found after checking ");
      Serial.print(bytesChecked);
      Serial.println(" bytes");
    }
    return false;
  }
  
  // Read the rest of the frame
  timeout = millis() + 1000; // 1 second timeout for rest of frame
  while (index < PMS5003_FRAME_LENGTH && millis() < timeout) {
    if (PMS5003Serial.available() > 0) {
      buffer[index] = PMS5003Serial.read();
      index++;
    }
  }
  
  if (index < PMS5003_FRAME_LENGTH) {
    return false; // Didn't receive complete frame
  }
  
  // Verify checksum
  uint16_t checksum = 0;
  for (int i = 0; i < 30; i++) {
    checksum += buffer[i];
  }
  uint16_t received_checksum = (buffer[30] << 8) | buffer[31];
  
  if (checksum != received_checksum) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("PMS5003: Checksum mismatch - calculated: ");
      Serial.print(checksum); Serial.print(", received: "); Serial.println(received_checksum);
    }
    return false;
  }
  
  // Parse the data (all values are big-endian 16-bit)
  data.pm1_0_standard = (buffer[4] << 8) | buffer[5];
  data.pm2_5_standard = (buffer[6] << 8) | buffer[7];
  data.pm10_standard = (buffer[8] << 8) | buffer[9];
  data.pm1_0_atmospheric = (buffer[10] << 8) | buffer[11];
  data.pm2_5_atmospheric = (buffer[12] << 8) | buffer[13];
  data.pm10_atmospheric = (buffer[14] << 8) | buffer[15];
  data.particles_0_3um = (buffer[16] << 8) | buffer[17];
  data.particles_0_5um = (buffer[18] << 8) | buffer[19];
  data.particles_1_0um = (buffer[20] << 8) | buffer[21];
  data.particles_2_5um = (buffer[22] << 8) | buffer[23];
  data.particles_5_0um = (buffer[24] << 8) | buffer[25];
  data.particles_10um = (buffer[26] << 8) | buffer[27];
  
  data.valid = true;
  return true;
}

// Get deployment type string - abbreviated for bandwidth efficiency
const char* getDeploymentTypeString() {
  switch (DEPLOYMENT_TYPE) {
    case INDOOR: return "in";
    case OUTDOOR: return "out"; 
    case MIXED: return "mix";
    default: return "out"; // Safe default
  }
}

void generateDeviceID() {
  uint64_t chipid = ESP.getEfuseMac();
  
  // Device ID: location_macaddress (all lowercase for consistency)
  // Format: {device_location}_{12-hex-mac}
  // Example: office_301274c0e8fc
  snprintf(deviceID, sizeof(deviceID), "%s_%012llx",
           DEVICE_LOCATION, chipid);
  
  snprintf(mqttClientID, sizeof(mqttClientID), "mqtt_%s", deviceID);
  
  // WeSense topic structure with ISO 3166 geographic codes (v2.1 format)
  // Format: wesense/v2/wifi/{country}/{subdivision}/{device_id}
  // Example: wesense/v2/wifi/nz/auk/office_301274c0e8fc
  // Note: WiFi devices send full protobuf with all fields
  //       LoRa devices use wesense/v2/lora/{device_id} via TTN webhook
  snprintf(deviceTopicPrefix, sizeof(deviceTopicPrefix),
           "wesense/v2/wifi/%s/%s/%s",
           COUNTRY_CODE, SUBDIVISION_CODE, deviceID);
}

// Non-blocking WiFi state tracking
static unsigned long lastWiFiReconnectAttempt = 0;
static int wifiReconnectAttempts = 0;
const unsigned long WIFI_RECONNECT_INTERVAL_MS = 30000; // Manual reconnect fallback every 30 seconds
const int MAX_WIFI_RECONNECT_ATTEMPTS = 10; // Max attempts before longer backoff
static uint8_t lastDisconnectReason = 0;

// WiFi disconnect reason to string (for diagnostics)
const char* getWiFiDisconnectReasonString(uint8_t reason) {
  switch (reason) {
    case 1: return "UNSPECIFIED";
    case 2: return "AUTH_EXPIRE";
    case 3: return "AUTH_LEAVE";
    case 4: return "ASSOC_EXPIRE";
    case 5: return "ASSOC_TOOMANY";
    case 6: return "NOT_AUTHED";
    case 7: return "NOT_ASSOCED";
    case 8: return "ASSOC_LEAVE";
    case 9: return "ASSOC_NOT_AUTHED";
    case 10: return "DISASSOC_PWRCAP_BAD";
    case 11: return "DISASSOC_SUPCHAN_BAD";
    case 12: return "IE_INVALID";
    case 13: return "MIC_FAILURE";
    case 14: return "4WAY_HANDSHAKE_TIMEOUT";
    case 15: return "GROUP_KEY_UPDATE_TIMEOUT";
    case 16: return "IE_IN_4WAY_DIFFERS";
    case 17: return "GROUP_CIPHER_INVALID";
    case 18: return "PAIRWISE_CIPHER_INVALID";
    case 19: return "AKMP_INVALID";
    case 20: return "UNSUPP_RSN_IE_VERSION";
    case 21: return "INVALID_RSN_IE_CAP";
    case 22: return "802_1X_AUTH_FAILED";
    case 23: return "CIPHER_SUITE_REJECTED";
    case 200: return "BEACON_TIMEOUT";
    case 201: return "NO_AP_FOUND";
    case 202: return "AUTH_FAIL";
    case 203: return "ASSOC_FAIL";
    case 204: return "HANDSHAKE_TIMEOUT";
    case 205: return "CONNECTION_FAIL";
    case 206: return "AP_TSF_RESET";
    case 207: return "ROAMING";
    default: return "UNKNOWN";
  }
}

// WiFi event handler for disconnect reason logging
void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      lastDisconnectReason = info.wifi_sta_disconnected.reason;
      Serial.print("WiFi: Disconnected! Reason: ");
      Serial.print(lastDisconnectReason);
      Serial.print(" (");
      Serial.print(getWiFiDisconnectReasonString(lastDisconnectReason));
      Serial.println(")");
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("WiFi: Connected to AP");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("WiFi: Got IP address: ");
      Serial.println(WiFi.localIP());
      break;
    default:
      break;
  }
}

void setup_wifi() {
  if (!ENABLE_WIFI) {
    Serial.println("WiFi: Disabled by configuration (ENABLE_WIFI = false)");
    wifiConnected = false;
    return;
  }
  
  Serial.println("Initializing WiFi (non-blocking)...");

  // Register WiFi event handler for disconnect reason logging
  WiFi.onEvent(onWiFiEvent);

  // Set hostname BEFORE WiFi.mode() - this is required for DHCP to see it
  // See: https://github.com/espressif/arduino-esp32/issues/2537
  String hostname = getSanitizedHostname();
  WiFi.setHostname(hostname.c_str());
  Serial.print("WiFi: Hostname set to: ");
  Serial.println(hostname);

  // Configure WiFi for standalone power operation
  WiFi.mode(WIFI_STA);
  WiFi.persistent(true);   // Save WiFi config to flash
  WiFi.setAutoReconnect(true); // Auto-reconnect if connection lost
  WiFi.setSleep(false);    // Disable power saving to prevent ping timeouts

  // Start initial connection attempt (non-blocking)
  Serial.print("WiFi: Starting connection to SSID: ");
  Serial.println(wifi_ssid);
  updateBootStatus("Connecting WiFi...");
  WiFi.begin(wifi_ssid, wifi_password);
  
  // Wait up to 5 seconds for initial connection (allows LoRaWAN to start quickly)
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 5000) {
    delay(100);
    if ((millis() - startTime) % 1000 == 0) {
      Serial.print(".");
    }
  }
  Serial.println();
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected quickly!");
    Serial.print("IP Address: "); Serial.println(WiFi.localIP());
    Serial.print("Signal strength: "); Serial.print(WiFi.RSSI()); Serial.println(" dBm");
    WiFi.setHostname(getSanitizedHostname().c_str());
    updateBootStatus("WiFi connected");

    // Initialize NTP time synchronization
    Serial.println("Initializing NTP time sync...");
    configTime(gmt_offset_sec, daylight_offset_sec, ntp_server, ntp_server2);

    // ESP32-C3 fix: Allow time for NTP DNS resolution to complete before MQTT
    // Prevents lwIP TCPIP core lock assertion failure on single-core chips
    delay(500);

    wifiConnected = true;
  } else {
    Serial.println("WiFi: Initial connection attempt in progress...");
    Serial.println("WiFi: Will continue trying in background (non-blocking)");
    wifiConnected = false;
  }
  
  lastWiFiReconnectAttempt = millis();
}

// Non-blocking WiFi reconnection handler (call from loop)
void handleWiFiReconnection() {
  if (!ENABLE_WIFI) {
    return;  // Skip WiFi handling if disabled
  }
  
  // If WIFI_STOP_AFTER_TIME_SYNC is enabled and we have time, stop trying to reconnect
  static bool timeSyncAchieved = false;
  if (WIFI_STOP_AFTER_TIME_SYNC && !timeSyncAchieved && getCurrentUnixTime() > 0) {
    timeSyncAchieved = true;
    Serial.println("WiFi: Time sync achieved, stopping reconnection attempts (WIFI_STOP_AFTER_TIME_SYNC)");
    WiFi.disconnect(true);  // Disconnect and disable auto-reconnect
    WiFi.mode(WIFI_OFF);    // Turn off WiFi completely
    return;
  }
  
  if (WIFI_STOP_AFTER_TIME_SYNC && timeSyncAchieved) {
    return;  // Time sync achieved, don't try to reconnect
  }
  
  // If WiFi is connected, reset attempt counter and update flag
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiConnected) {
      Serial.println("WiFi: Connected!");
      Serial.print("IP Address: "); Serial.println(WiFi.localIP());
      WiFi.setHostname(getSanitizedHostname().c_str());
      wifiConnected = true;
      wifiReconnectAttempts = 0;

      // Initialize NTP time synchronization
      configTime(gmt_offset_sec, daylight_offset_sec, ntp_server, ntp_server2);

      // ESP32-C3 fix: Allow time for NTP DNS resolution to complete before MQTT
      delay(500);

      syslog.notice("WiFi reconnected IP=" + WiFi.localIP().toString() + " RSSI=" + String(WiFi.RSSI()));
    }
    return;
  }

  // WiFi is not connected
  if (wifiConnected) {
    Serial.println("WiFi: Connection lost!");
    syslog.warning("WiFi connection lost");
    wifiConnected = false;
    wifiReconnectAttempts = 0;
    lastWiFiReconnectAttempt = millis(); // Start timing for manual reconnect
  }

  // Manual reconnection fallback - handles cases where native auto-reconnect fails
  // (e.g., AP reboot, certain disconnect reasons not handled by ESP32 auto-reconnect)
  unsigned long now = millis();
  unsigned long reconnectInterval = WIFI_RECONNECT_INTERVAL_MS;

  // Use longer backoff after many failed attempts (5 minutes instead of 30 seconds)
  if (wifiReconnectAttempts >= MAX_WIFI_RECONNECT_ATTEMPTS) {
    reconnectInterval = 300000; // 5 minutes
  }

  if (now - lastWiFiReconnectAttempt >= reconnectInterval) {
    lastWiFiReconnectAttempt = now;
    wifiReconnectAttempts++;

    Serial.print("WiFi: Manual reconnect attempt ");
    Serial.print(wifiReconnectAttempts);
    if (wifiReconnectAttempts >= MAX_WIFI_RECONNECT_ATTEMPTS) {
      Serial.print(" (extended backoff mode - every 5 min)");
    }
    Serial.print(" - Last disconnect reason: ");
    Serial.print(lastDisconnectReason);
    Serial.print(" (");
    Serial.print(getWiFiDisconnectReasonString(lastDisconnectReason));
    Serial.println(")");

    // Force a fresh connection attempt with hostname
    // Must set mode to NULL, then hostname, then mode back to STA for hostname to take effect
    WiFi.disconnect(false); // Disconnect without erasing credentials
    delay(100);
    WiFi.mode(WIFI_OFF);  // Turn off WiFi to allow hostname change
    delay(100);
    WiFi.setHostname(getSanitizedHostname().c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid, wifi_password);
  }
}

// Non-blocking MQTT reconnection with rate limiting
static unsigned long lastMqttReconnectAttempt = 0;
static int mqttConsecutiveFailures = 0;  // Track consecutive failures for socket reset
const unsigned long MQTT_RECONNECT_INTERVAL_MS = 10000; // Try every 10 seconds (enough time for TCP to reset)
const int MQTT_FAILURES_BEFORE_SOCKET_RESET = 5;  // Reset socket after this many consecutive failures

void reconnect() {
  if (!ENABLE_MQTT) {
    return;  // Skip MQTT if disabled
  }

  if (!wifiConnected) {
    return; // Don't try if WiFi isn't up
  }

  if (client.connected()) {
    mqttConsecutiveFailures = 0;  // Reset counter on successful connection
    return; // Already connected
  }

  // Rate limit reconnection attempts (non-blocking)
  // But allow first attempt immediately after WiFi connects
  unsigned long now = millis();
  if (lastMqttReconnectAttempt > 0 && (now - lastMqttReconnectAttempt < MQTT_RECONNECT_INTERVAL_MS)) {
    return;
  }
  lastMqttReconnectAttempt = now;

  // ESP32 classic socket leak fix: After repeated failures, force socket cleanup
  // This helps recover from "zombie" TCP connections that block new MQTT connections
  if (mqttConsecutiveFailures >= MQTT_FAILURES_BEFORE_SOCKET_RESET) {
    Serial.println("MQTT: Too many failures, resetting socket...");
    syslog.warning("MQTT socket reset after " + String(mqttConsecutiveFailures) + " failures");
    espClient.stop();  // Force close the underlying TCP socket
    delay(100);        // Allow lwIP to clean up
    mqttConsecutiveFailures = 0;  // Reset counter
  }
  
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("Attempting MQTT connection to "); Serial.print(mqtt_server); Serial.print(":"); Serial.print(mqtt_port);
    Serial.print(" as '"); Serial.print(mqttClientID); Serial.println("'...");

    // Debug: Check DNS resolution (ESP32-C3 troubleshooting)
    IPAddress mqttIP;
    if (WiFi.hostByName(mqtt_server, mqttIP)) {
      Serial.print("  DNS resolved to: ");
      Serial.println(mqttIP);
    } else {
      Serial.println("  DNS resolution FAILED!");
    }
  }

  // ESP32-C3 fix: yield to allow any pending lwIP operations to complete
  // Prevents TCPIP core lock assertion failure on single-core chips
  delay(100);  // Increased from 10ms for C3 stability

  // Non-blocking connect attempt with short timeout
  if (client.connect(mqttClientID, mqtt_user, mqtt_password)) {
    if (ENABLE_DEBUG_OUTPUT) Serial.println("MQTT connected successfully!");
    syslog.notice("MQTT connected to " + String(mqtt_server));
    mqttConsecutiveFailures = 0;  // Reset failure counter on success

    // Subscribe to command topics
    subscribeToCommandTopics();

    publishAllDiscoveryConfigs(); // Re-publish discovery on reconnect
  } else {
    String errorReason;
    switch (client.state()) {
      case -4: errorReason = "Connection timeout"; break;
      case -3: errorReason = "Connection lost"; break;
      case -2: errorReason = "Connect failed"; break;
      case -1: errorReason = "Disconnected"; break;
      case 1: errorReason = "Bad protocol"; break;
      case 2: errorReason = "Bad client ID"; break;
      case 3: errorReason = "Unavailable"; break;
      case 4: errorReason = "Bad credentials"; break;
      case 5: errorReason = "Unauthorized"; break;
      default: errorReason = "Unknown error"; break;
    }

    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(client.state());
      Serial.print(" (");
      Serial.print(errorReason);
      Serial.println(") - will retry shortly");
    }

    syslog.warning("MQTT failed rc=" + String(client.state()) + " (" + errorReason + ")");
    mqttConsecutiveFailures++;  // Track failures for socket reset logic
  }
}

// ======================
// MQTT COMMAND SYSTEM
// ======================

// MQTT callback function for handling incoming commands
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  lastMqttRxTime = millis();  // Update RX indicator for display

  // Convert payload to string
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  String topicStr = String(topic);
  
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("MQTT Command: Received on topic: ");
    Serial.println(topicStr);
    Serial.print("MQTT Command: Payload: ");
    Serial.println(message);
  }
  
  // Parse command topic structure: deviceTopicPrefix + "/command/category/target/action"
  // Examples:
  //   wesense/v2/nz/auk/office_301274c0e8fc/command/sensor/scd4x/calibrate
  //   wesense/v2/nz/auk/office_301274c0e8fc/command/device/led/set
  //   wesense/v2/nz/auk/office_301274c0e8fc/command/system/restart
  
  String commandPrefix = String(deviceTopicPrefix) + "/command/";
  if (topicStr.startsWith(commandPrefix)) {
    String commandPath = topicStr.substring(commandPrefix.length());
    processCommand(commandPath, message);
  } else {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("MQTT Command: Topic does not match expected command structure");
    }
  }
}

// Process incoming command based on category/target/action structure
void processCommand(const String& commandPath, const String& payload) {
  // Split command path into components
  int firstSlash = commandPath.indexOf('/');
  if (firstSlash == -1) {
    publishCommandResult("ERROR", "Invalid command format", commandPath);
    return;
  }
  
  String category = commandPath.substring(0, firstSlash);
  String remainder = commandPath.substring(firstSlash + 1);
  
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("MQTT Command: Processing category '"); Serial.print(category);
    Serial.print("' with path '"); Serial.print(remainder); Serial.println("'");
  }
  
  // Route to appropriate handler based on category
  if (category == "sensor") {
    processSensorCommand(remainder, payload);
  } else if (category == "device") {
    processDeviceCommand(remainder, payload);
  } else if (category == "system") {
    processSystemCommand(remainder, payload);
  } else {
    publishCommandResult("ERROR", "Unknown command category: " + category, commandPath);
  }
}

// Handle sensor-specific commands
void processSensorCommand(const String& commandPath, const String& payload) {
  int slashPos = commandPath.indexOf('/');
  if (slashPos == -1) {
    publishCommandResult("ERROR", "Sensor command requires target/action format", "sensor/" + commandPath);
    return;
  }
  
  String sensorTarget = commandPath.substring(0, slashPos);
  String action = commandPath.substring(slashPos + 1);
  
  if (sensorTarget == "scd4x") {
    handleSCD4xCommand(action, payload);
  } else if (sensorTarget == "scd30") {
    handleSCD30Command(action, payload);
  } else if (sensorTarget == "cm1106c") {
    handleCM1106CCommand(action, payload);
  } else if (sensorTarget == "sgp41") {
    handleSGP41Command(action, payload);
  } else if (sensorTarget == "bme680") {
    handleBME680Command(action, payload);
  } else if (sensorTarget == "bmp280") {
    handleBMP280Command(action, payload);
  } else {
    publishCommandResult("ERROR", "Unknown sensor target: " + sensorTarget, "sensor/" + commandPath);
  }
}

// Handle device-specific commands (LED, configuration, etc.)
void processDeviceCommand(const String& commandPath, const String& payload) {
  int slashPos = commandPath.indexOf('/');
  if (slashPos == -1) {
    publishCommandResult("ERROR", "Device command requires target/action format", "device/" + commandPath);
    return;
  }
  
  String deviceTarget = commandPath.substring(0, slashPos);
  String action = commandPath.substring(slashPos + 1);
  
  if (deviceTarget == "led") {
    handleLEDCommand(action, payload);
  } else if (deviceTarget == "config") {
    handleConfigCommand(action, payload);
  } else if (deviceTarget == "display") {
    handleDisplayCommand(action, payload);
  } else {
    publishCommandResult("ERROR", "Unknown device target: " + deviceTarget, "device/" + commandPath);
  }
}

// Handle system-level commands
void processSystemCommand(const String& commandPath, const String& payload) {
  if (commandPath == "restart") {
    handleSystemRestart(payload);
  } else if (commandPath == "status") {
    handleSystemStatus(payload);
  } else if (commandPath == "factory_reset") {
    handleFactoryReset(payload);
  } else if (commandPath == "testing_enable") {
    handleTestingMode(true, payload);
  } else if (commandPath == "testing_disable") {
    handleTestingMode(false, payload);
  } else if (commandPath == "calibration_status") {
    handleCalibrationStatus(payload);
  } else if (commandPath == "calibration_reset") {
    handleCalibrationReset(payload);
  } else if (commandPath == "calibration_restart") {
    handleCalibrationRestart(payload);
  } else if (commandPath == "calibration_backup") {
    handleCalibrationBackup(payload);
  } else if (commandPath == "calibration_restore") {
    handleCalibrationRestore(payload);
  } else if (commandPath == "syslog_enable") {
    handleSyslogEnable(true);
  } else if (commandPath == "syslog_disable") {
    handleSyslogEnable(false);
  } else if (commandPath == "syslog_status") {
    handleSyslogStatus();
  } else {
    publishCommandResult("ERROR", "Unknown system command: " + commandPath, "system/" + commandPath);
  }
}

// ======================
// SENSOR COMMAND HANDLERS
// ======================

// SCD4x sensor commands
void handleSCD4xCommand(const String& action, const String& payload) {
  if (!scd4xAvailable) {
    publishCommandResult("ERROR", "SCD4x sensor not available", "sensor/scd4x/" + action);
    return;
  }
  
  if (action == "calibrate") {
    handleSCD4xCalibration(payload);
  } else if (action == "reset") {
    handleSCD4xReset();
  } else if (action == "selftest") {
    handleSCD4xSelfTest();
  } else if (action == "asc_enable") {
    handleSCD4xASC(true);
  } else if (action == "asc_disable") {
    handleSCD4xASC(false);
  } else if (action == "asc_status") {
    publishSCD4xASCStatus();
  } else if (action == "pressure_compensation_status") {
    handleSCD4xPressureCompensationStatus();
  } else if (action == "pressure_compensation_apply") {
    handleSCD4xPressureCompensationApply();
  } else if (action == "factory_reset") {
    handleSCD4xFactoryReset();
  } else {
    publishCommandResult("ERROR", "Unknown SCD4x action: " + action, "sensor/scd4x/" + action);
  }
}

// SCD30 sensor commands
void handleSCD30Command(const String& action, const String& payload) {
  if (!scd30Available) {
    publishCommandResult("ERROR", "SCD30 sensor not available", "sensor/scd30/" + action);
    return;
  }

  if (action == "calibrate") {
    // Forced recalibration to specific CO2 ppm
    String cmd = payload;
    cmd.trim();

    uint16_t targetCO2 = 400; // Default outdoor CO2 level

    if (cmd.length() > 0) {
      int ppmValue = cmd.toInt();
      if (ppmValue >= 400 && ppmValue <= 2000) {
        targetCO2 = (uint16_t)ppmValue;
      } else {
        publishCommandResult("ERROR", "Invalid CO2 value. Must be 400-2000 ppm", "sensor/scd30/calibrate");
        return;
      }
    }

    Serial.print("SCD30: Starting forced recalibration to ");
    Serial.print(targetCO2);
    Serial.println(" ppm...");

    if (scd30.setForcedRecalibrationFactor(targetCO2)) {
      setCalibrationMethod("scd30", "frc_manual", "high", String(targetCO2) + "ppm_manual");
      resetCalibrationState("scd30");
      publishCommandResult("SUCCESS", "SCD30 calibrated to " + String(targetCO2) + " ppm", "sensor/scd30/calibrate");
      Serial.println("SCD30: Forced recalibration successful");
    } else {
      publishCommandResult("ERROR", "SCD30 calibration command failed", "sensor/scd30/calibrate");
      Serial.println("SCD30: Forced recalibration failed");
    }
  } else if (action == "asc_enable") {
    Serial.println("SCD30: Enabling Automatic Self-Calibration...");
    if (scd30.setAutoSelfCalibration(true)) {
      setCalibrationMethod("scd30", "asc_learning", "low", "7_day_automatic");
      startSensorCalibrationPeriod("scd30");
      publishCommandResult("SUCCESS", "SCD30 ASC enabled", "sensor/scd30/asc_enable");
      Serial.println("SCD30: ASC enabled successfully");
    } else {
      publishCommandResult("ERROR", "Failed to enable SCD30 ASC", "sensor/scd30/asc_enable");
    }
  } else if (action == "asc_disable") {
    Serial.println("SCD30: Disabling Automatic Self-Calibration...");
    if (scd30.setAutoSelfCalibration(false)) {
      setCalibrationMethod("scd30", "factory", "medium", "asc_disabled");
      resetCalibrationState("scd30");
      publishCommandResult("SUCCESS", "SCD30 ASC disabled", "sensor/scd30/asc_disable");
      Serial.println("SCD30: ASC disabled successfully");
    } else {
      publishCommandResult("ERROR", "Failed to disable SCD30 ASC", "sensor/scd30/asc_disable");
    }
  } else if (action == "set_interval") {
    // Set measurement interval (2-1800 seconds)
    String cmd = payload;
    cmd.trim();
    int interval = cmd.toInt();

    if (interval >= 2 && interval <= 1800) {
      if (scd30.setMeasurementInterval(interval)) {
        publishCommandResult("SUCCESS", "SCD30 interval set to " + String(interval) + "s", "sensor/scd30/set_interval");
        Serial.print("SCD30: Measurement interval set to ");
        Serial.print(interval);
        Serial.println(" seconds");
      } else {
        publishCommandResult("ERROR", "Failed to set SCD30 interval", "sensor/scd30/set_interval");
      }
    } else {
      publishCommandResult("ERROR", "Invalid interval. Must be 2-1800 seconds", "sensor/scd30/set_interval");
    }
  } else if (action == "set_altitude") {
    // Set altitude compensation in meters
    String cmd = payload;
    cmd.trim();
    int altitude = cmd.toInt();

    if (altitude >= 0 && altitude <= 10000) {
      if (scd30.setAltitudeCompensation(altitude)) {
        publishCommandResult("SUCCESS", "SCD30 altitude set to " + String(altitude) + "m", "sensor/scd30/set_altitude");
        Serial.print("SCD30: Altitude compensation set to ");
        Serial.print(altitude);
        Serial.println(" meters");
      } else {
        publishCommandResult("ERROR", "Failed to set SCD30 altitude", "sensor/scd30/set_altitude");
      }
    } else {
      publishCommandResult("ERROR", "Invalid altitude. Must be 0-10000 meters", "sensor/scd30/set_altitude");
    }
  } else if (action == "status") {
    // Report SCD30 status
    String status = "SCD30 Status:\n";
    status += "  Available: " + String(scd30Available ? "Yes" : "No") + "\n";
    status += "  Data Ready: " + String(scd30.dataAvailable() ? "Yes" : "No") + "\n";
    if (scd30.dataAvailable()) {
      status += "  CO2: " + String(scd30.getCO2(), 0) + " ppm\n";
      status += "  Temp: " + String(scd30.getTemperature(), 2) + " °C\n";
      status += "  Humidity: " + String(scd30.getHumidity(), 2) + " %";
    }
    publishCommandResult("SUCCESS", status, "sensor/scd30/status");
  } else {
    publishCommandResult("ERROR", "Unknown SCD30 action: " + action, "sensor/scd30/" + action);
  }
}

// CM1106-C sensor commands
void handleCM1106CCommand(const String& action, const String& payload) {
  if (!cm1106cAvailable) {
    publishCommandResult("ERROR", "CM1106-C sensor not available", "sensor/cm1106c/" + action);
    return;
  }
  
  if (action == "calibrate") {
    // Manual calibration to specific ppm
    String cmd = payload;
    cmd.trim();
    
    uint16_t targetCO2 = 400; // Default outdoor CO2 level
    
    if (cmd.length() > 0) {
      int ppmValue = cmd.toInt();
      if (ppmValue >= 400 && ppmValue <= 1500) {
        targetCO2 = (uint16_t)ppmValue;
      } else {
        publishCommandResult("ERROR", "Invalid ppm value. Must be between 400-1500 ppm", "sensor/cm1106c/calibrate");
        return;
      }
    }
    
    Serial.print("CM1106-C: Starting manual calibration to ");
    Serial.print(targetCO2);
    Serial.println(" ppm...");
    
    uint8_t result = cm1106c.calibration(targetCO2);
    
    if (result == 0) {
      String successMsg = "Manual calibration successful. Target: " + String(targetCO2) + " ppm";
      Serial.println("CM1106-C: " + successMsg);
      publishCommandResult("SUCCESS", successMsg, "sensor/cm1106c/calibrate");
      
      // Update calibration method to manual
      String reference = String(targetCO2) + "ppm_manual";
      setCalibrationMethod("cm1106c", "manual", "high", reference);
      resetCalibrationState("cm1106c"); // End any calibration period - sensor is now calibrated
      Serial.println("CM1106-C: Calibration method updated to manual with high confidence");
    } else {
      String errorMsg = "Calibration failed with error code: " + String(result);
      Serial.println("CM1106-C: " + errorMsg);
      publishCommandResult("ERROR", errorMsg, "sensor/cm1106c/calibrate");
    }
  } else if (action == "abc_enable") {
    // Enable Automatic Baseline Correction
    Serial.println("CM1106-C: Enabling Automatic Baseline Correction...");
    
    uint8_t result = cm1106c.auto_zero_setting(0, 15, 400); // Enable ABC, 15 days, 400ppm
    
    if (result == 0) {
      String message = "ABC (Automatic Baseline Correction) enabled successfully";
      Serial.println("CM1106-C: " + message);
      publishCommandResult("SUCCESS", message, "sensor/cm1106c/abc_enable");
      
      // Update calibration method to ABC learning mode
      setCalibrationMethod("cm1106c", "abc_learning", "low", "15_day_automatic");
      SensorCalibrationState* state = getSensorCalibrationState("cm1106c");
      if (state == nullptr || !state->isCalibrating) {
        Serial.println("CM1106-C: Starting ESP32 calibration period for ABC learning (15 days)");
        // Start calibration tracking (will be handled by the calibration system)
      }
    } else {
      String errorMsg = "Failed to enable ABC, error code: " + String(result);
      publishCommandResult("ERROR", errorMsg, "sensor/cm1106c/abc_enable");
    }
  } else if (action == "abc_disable") {
    // Disable Automatic Baseline Correction
    Serial.println("CM1106-C: Disabling Automatic Baseline Correction...");
    
    uint8_t result = cm1106c.auto_zero_setting(2, 15, 400); // Disable ABC (2 = close)
    
    if (result == 0) {
      String message = "ABC (Automatic Baseline Correction) disabled successfully";
      Serial.println("CM1106-C: " + message);
      publishCommandResult("SUCCESS", message, "sensor/cm1106c/abc_disable");
      
      // Update calibration method to factory
      setCalibrationMethod("cm1106c", "factory", "medium", "abc_disabled");
      resetCalibrationState("cm1106c");
      Serial.println("CM1106-C: Calibration period cleared - ABC disabled");
    } else {
      String errorMsg = "Failed to disable ABC, error code: " + String(result);
      publishCommandResult("ERROR", errorMsg, "sensor/cm1106c/abc_disable");
    }
  } else if (action == "status") {
    // Read and report sensor status
    uint8_t result = cm1106c.measure_result();
    
    if (result == 0) {
      String statusMsg = "CM1106-C Status: CO2=" + String(cm1106c.co2) + "ppm, Status=0x" + String(cm1106c.status, HEX);
      
      // Interpret status
      statusMsg += " ("; 
      switch(cm1106c.status) {
        case CM1106_I2C_STATUS_PREHEATING:
          statusMsg += "Preheating";
          break;
        case CM1106_I2C_STATUS_NORMAL_OPERATION:
          statusMsg += "Normal operation";
          break;
        case CM1106_I2C_STATUS_OPERATING_TROUBLE:
          statusMsg += "Operating trouble";
          break;
        case CM1106_I2C_STATUS_OUT_OF_FS:
          statusMsg += "Out of full scale";
          break;
        case CM1106_I2C_STATUS_NON_CALIBRATED:
          statusMsg += "Non-calibrated";
          break;
        default:
          statusMsg += "Unknown";
          break;
      }
      statusMsg += ")";
      
      publishCommandResult("SUCCESS", statusMsg, "sensor/cm1106c/status");
    } else {
      publishCommandResult("ERROR", "Failed to read sensor status, error code: " + String(result), "sensor/cm1106c/status");
    }
  } else if (action == "serial_number") {
    // Read sensor serial number
    Serial.println("CM1106-C: Reading serial number...");
    uint8_t result = cm1106c.read_serial_number();
    
    if (result == 0) {
      publishCommandResult("SUCCESS", "Serial number read successfully (check Serial output)", "sensor/cm1106c/serial_number");
    } else {
      publishCommandResult("ERROR", "Failed to read serial number, error code: " + String(result), "sensor/cm1106c/serial_number");
    }
  } else if (action == "version") {
    // Read sensor firmware version
    Serial.println("CM1106-C: Reading firmware version...");
    uint8_t result = cm1106c.check_sw_version();
    
    if (result == 0) {
      publishCommandResult("SUCCESS", "Firmware version read successfully (check Serial output)", "sensor/cm1106c/version");
    } else {
      publishCommandResult("ERROR", "Failed to read firmware version, error code: " + String(result), "sensor/cm1106c/version");
    }
  } else {
    publishCommandResult("ERROR", "Unknown CM1106-C action: " + action, "sensor/cm1106c/" + action);
  }
}

// SGP41 sensor commands
void handleSGP41Command(const String& action, const String& payload) {
  if (!sgp41Available) {
    publishCommandResult("ERROR", "SGP41 sensor not available", "sensor/sgp41/" + action);
    return;
  }
  
  if (action == "baseline_status") {
    // Trigger immediate baseline status publication
    publishCommandResult("SUCCESS", "SGP41 baseline status will be published on next reading cycle", "sensor/sgp41/baseline_status");
  } else if (action == "reset") {
    // Implement SGP41 reset if needed
    publishCommandResult("INFO", "SGP41 reset not implemented yet", "sensor/sgp41/" + action);
  } else {
    publishCommandResult("ERROR", "Unknown SGP41 action: " + action, "sensor/sgp41/" + action);
  }
}

// BME680 sensor commands
void handleBME680Command(const String& action, const String& payload) {
  if (!bme680Available) {
    publishCommandResult("ERROR", "BME680 sensor not available", "sensor/bme680/" + action);
    return;
  }
  
  if (action == "gas_heater") {
    handleBME680GasHeater(payload);
  } else {
    publishCommandResult("ERROR", "Unknown BME680 action: " + action, "sensor/bme680/" + action);
  }
}

// BMP280 sensor commands
void handleBMP280Command(const String& action, const String& payload) {
  if (!bmp280Available) {
    publishCommandResult("ERROR", "BMP280 sensor not available", "sensor/bmp280/" + action);
    return;
  }
  
  // BMP280 doesn't have many configurable parameters, but we can add diagnostics
  if (action == "diagnostic") {
    handleBMP280Diagnostic();
  } else {
    publishCommandResult("ERROR", "Unknown BMP280 action: " + action, "sensor/bmp280/" + action);
  }
}

// ======================
// DEVICE COMMAND HANDLERS
// ======================

// LED control commands
void handleLEDCommand(const String& action, const String& payload) {
  if (action == "set") {
    String cmd = payload;
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd == "ON") {
      if (g_boardConfig.led_pin >= 0) {
        digitalWrite(g_boardConfig.led_pin, HIGH);
        publishCommandResult("SUCCESS", "LED turned on", "device/led/set");
      } else {
        publishCommandResult("ERROR", "LED not available on this board", "device/led/set");
      }
    } else if (cmd == "OFF") {
      if (g_boardConfig.led_pin >= 0) {
        digitalWrite(g_boardConfig.led_pin, LOW);
        publishCommandResult("SUCCESS", "LED turned off", "device/led/set");
      } else {
        publishCommandResult("ERROR", "LED not available on this board", "device/led/set");
      }
    } else if (cmd == "BLINK_FAST") {
      setLEDMode(true); // Fast blink
      publishCommandResult("SUCCESS", "LED set to fast blink mode", "device/led/set");
    } else if (cmd == "BLINK_SLOW") {
      setLEDMode(false); // Slow blink
      publishCommandResult("SUCCESS", "LED set to slow blink mode", "device/led/set");
    } else {
      publishCommandResult("ERROR", "Invalid LED command. Use: ON, OFF, BLINK_FAST, BLINK_SLOW", "device/led/set");
    }
  } else {
    publishCommandResult("ERROR", "Unknown LED action: " + action, "device/led/" + action);
  }
}

// Configuration commands
void handleConfigCommand(const String& action, const String& payload) {
  if (action == "debug") {
    // Note: ENABLE_DEBUG_OUTPUT is const, so this would require restart to take effect
    publishCommandResult("INFO", "Debug mode is " + String(ENABLE_DEBUG_OUTPUT ? "enabled" : "disabled") + " (requires code change to modify)", "device/config/debug");
  } else if (action == "location") {
    handleLocationCommand(payload);
  } else if (action == "location_reset") {
    handleLocationResetCommand();
  } else if (action == "location_status") {
    handleLocationStatusCommand();
  } else {
    publishCommandResult("ERROR", "Unknown config action: " + action, "device/config/" + action);
  }
}

// Location configuration command handlers
void handleLocationCommand(const String& payload) {
  // Parse JSON payload for location data
  // Simple JSON parsing - for production, consider using ArduinoJson library
  
  if (payload.length() == 0) {
    publishCommandResult("ERROR", "Location command requires JSON payload", "device/config/location");
    return;
  }
  
  // Look for latitude in JSON
  int latStart = payload.indexOf("\"latitude\"");
  int lngStart = payload.indexOf("\"longitude\"");
  int enableStart = payload.indexOf("\"enable\"");

  float newLat = runtimeLocation.latitude;
  float newLng = runtimeLocation.longitude;
  bool newEnable = runtimeLocation.includeInMQTT;

  // Parse latitude
  if (latStart >= 0) {
    int colonPos = payload.indexOf(':', latStart);
    if (colonPos >= 0) {
      int valueStart = colonPos + 1;
      while (valueStart < payload.length() && (payload.charAt(valueStart) == ' ' || payload.charAt(valueStart) == '\"')) valueStart++;
      int valueEnd = valueStart;
      while (valueEnd < payload.length() && payload.charAt(valueEnd) != ',' && payload.charAt(valueEnd) != '}' && payload.charAt(valueEnd) != '\"') valueEnd++;

      String latStr = payload.substring(valueStart, valueEnd);
      latStr.trim();
      newLat = latStr.toFloat();
    }
  }

  // Parse longitude
  if (lngStart >= 0) {
    int colonPos = payload.indexOf(':', lngStart);
    if (colonPos >= 0) {
      int valueStart = colonPos + 1;
      while (valueStart < payload.length() && (payload.charAt(valueStart) == ' ' || payload.charAt(valueStart) == '\"')) valueStart++;
      int valueEnd = valueStart;
      while (valueEnd < payload.length() && payload.charAt(valueEnd) != ',' && payload.charAt(valueEnd) != '}' && payload.charAt(valueEnd) != '\"') valueEnd++;

      String lngStr = payload.substring(valueStart, valueEnd);
      lngStr.trim();
      newLng = lngStr.toFloat();
    }
  }

  // Parse enable flag
  if (enableStart >= 0) {
    int colonPos = payload.indexOf(':', enableStart);
    if (colonPos >= 0) {
      String enableStr = payload.substring(colonPos + 1);
      enableStr.trim();
      enableStr.toLowerCase();
      newEnable = (enableStr.indexOf("true") >= 0);
    }
  }

  // Validate coordinates
  if (!validateCoordinates(newLat, newLng)) {
    publishCommandResult("ERROR", "Invalid coordinates. Latitude must be -90 to 90, longitude must be -180 to 180", "device/config/location");
    return;
  }

  // Update runtime configuration
  runtimeLocation.latitude = newLat;
  runtimeLocation.longitude = newLng;
  runtimeLocation.includeInMQTT = newEnable;
  runtimeLocation.overrideActive = true;

  // Save to NVS
  if (saveLocationConfig()) {
    String message = "Location updated: " + String(newLat, 6) + ", " + String(newLng, 6);
    message += ". Include in MQTT: " + String(newEnable ? "Yes" : "No");
    publishCommandResult("SUCCESS", message, "device/config/location");
  } else {
    publishCommandResult("ERROR", "Location updated but failed to save to flash memory", "device/config/location");
  }
}

void handleLocationResetCommand() {
  resetLocationToDefaults();
  saveLocationConfig();

  String message = "Location reset to firmware defaults: " + String(runtimeLocation.latitude, 6) + ", " + String(runtimeLocation.longitude, 6);
  message += ". Include in MQTT: " + String(runtimeLocation.includeInMQTT ? "Yes" : "No");
  publishCommandResult("SUCCESS", message, "device/config/location_reset");
}

void handleLocationStatusCommand() {
  String status = "Current location: " + String(runtimeLocation.latitude, 6) + ", " + String(runtimeLocation.longitude, 6);
  status += ". Include in MQTT: " + String(runtimeLocation.includeInMQTT ? "Yes" : "No");
  status += ". Source: " + String(runtimeLocation.overrideActive ? "MQTT Override" : "Firmware Default");

  // Add firmware defaults for reference
  status += ". Firmware defaults: " + String(FIXED_LATITUDE, 6) + ", " + String(FIXED_LONGITUDE, 6);

  publishCommandResult("SUCCESS", status, "device/config/location_status");
}

// Display control commands
void handleDisplayCommand(const String& action, const String& payload) {
  if (action == "on") {
    if (!displayAvailable) {
      publishCommandResult("ERROR", "OLED display not available", "device/display/on");
      return;
    }
    
    if (displayTimedOut) {
      displayTimedOut = false;
      displayStartTime = millis(); // Reset timeout counter

      // Show a brief "Display Re-enabled" message
      displayClear();
      display->setTextSize(1);
      display->setTextColor(1);  // Use 1 for white (compatible with SSD1306 and SH1106)
      display->setCursor(0, 0);
      display->println("Display Re-enabled");
      display->println("");
      display->println("System Status:");
      display->print("WiFi: ");
      display->println(wifiConnected ? "OK" : "Disconnected");
      display->print("MQTT: ");
      display->println(client.connected() ? "OK" : "Disconnected");

      // Show current time if available
      struct tm timeinfo;
      if (getLocalTime(&timeinfo)) {
        char timeStr[20];
        strftime(timeStr, sizeof(timeStr), "%H:%M:%S %d/%m", &timeinfo);
        display->print("Time: ");
        display->println(timeStr);
      }

      displayRefresh();
      
      publishCommandResult("SUCCESS", "OLED display turned back on and timeout reset", "device/display/on");
      Serial.println("OLED Display re-enabled via MQTT command");
    } else {
      publishCommandResult("INFO", "OLED display is already active", "device/display/on");
    }
  } else if (action == "off") {
    if (!displayAvailable) {
      publishCommandResult("ERROR", "OLED display not available", "device/display/off");
      return;
    }
    
    displayClear();
    displayRefresh(); // Clear the screen immediately
    displayTimedOut = true;
    
    publishCommandResult("SUCCESS", "OLED display turned off", "device/display/off");
    Serial.println("OLED Display turned off via MQTT command");
  } else if (action == "status") {
    if (!displayAvailable) {
      publishCommandResult("INFO", "OLED display not available", "device/display/status");
      return;
    }
    
    String status = "Display available: Yes, ";
    status += "Currently: " + String(displayTimedOut ? "Off (timed out or manually disabled)" : "Active");
    if (!displayTimedOut) {
      unsigned long elapsedMin = (millis() - displayStartTime) / 60000;
      unsigned long remainingMin = (DISPLAY_TIMEOUT_MS / 60000) - elapsedMin;
      if (remainingMin > 0) {
        status += ", Timeout in: " + String(remainingMin) + " minutes";
      } else {
        status += ", Timeout due soon";
      }
    }
    
    publishCommandResult("SUCCESS", status, "device/display/status");
  } else {
    publishCommandResult("ERROR", "Unknown display action: " + action + ". Use: on, off, status", "device/display/" + action);
  }
}

// ======================
// SYSTEM COMMAND HANDLERS
// ======================

void handleSystemRestart(const String& payload) {
  publishCommandResult("SUCCESS", "System restart initiated. Device will reboot in 3 seconds.", "system/restart");
  delay(3000);
  ESP.restart();
}

void handleSystemStatus(const String& payload) {
  String status = "Device: " + String(deviceID) + ", ";
  status += "WiFi: " + String(wifiConnected ? "Connected" : "Disconnected") + ", ";
  status += "MQTT: " + String(client.connected() ? "Connected" : "Disconnected") + ", ";
  status += "Uptime: " + String(millis() / 1000) + "s, ";
  status += "Free heap: " + String(ESP.getFreeHeap()) + " bytes, ";
  status += "Unix time: " + String(getCurrentUnixTime());
  
  publishCommandResult("SUCCESS", status, "system/status");
}

// Backup calibration data to MQTT for external storage
void handleCalibrationBackup(const String& payload) {
  if (!PERSIST_CALIBRATION_STATE) {
    publishCommandResult("ERROR", "Calibration persistence is disabled", "system/calibration_backup");
    return;
  }
  
  String backupData = "{";
  calibrationPrefs.begin("cal_state", true);
  
  // Backup all calibration data
  String sensors[] = {"scd4x", "scd30", "cm1106c", "c8co2", "sgp41", "bme680", "pms5003", "tmp117"};
  for (String sensorName : sensors) {
    String prefix = sensorName + "_";
    backupData += "\"" + sensorName + "\":{";
    backupData += "\"calibrating\":" + String(calibrationPrefs.getBool((prefix + "cal").c_str(), false) ? "true" : "false") + ",";
    backupData += "\"unix_start\":" + String(calibrationPrefs.getULong((prefix + "unix_start").c_str(), 0)) + ",";
    backupData += "\"period\":" + String(calibrationPrefs.getULong((prefix + "period").c_str(), 0));
    backupData += "},";
  }
  
  backupData += "\"save_time\":" + String(calibrationPrefs.getULong("save_time", 0)) + ",";
  backupData += "\"firmware_version\":\"" + calibrationPrefs.getString("firmware_version", "unknown") + "\",";
  backupData += "\"backup_time\":" + String(getCurrentUnixTime());
  backupData += "}";
  
  calibrationPrefs.end();
  
  // Publish backup data
  String backupTopic = String(deviceTopicPrefix) + "/diagnostic/calibration_backup";
  client.publish(backupTopic.c_str(), backupData.c_str());
  
  publishCommandResult("SUCCESS", "Calibration data backup published to " + backupTopic, "system/calibration_backup");
}

// Restore calibration data from JSON payload
void handleCalibrationRestore(const String& payload) {
  if (!PERSIST_CALIBRATION_STATE) {
    publishCommandResult("ERROR", "Calibration persistence is disabled", "system/calibration_restore");
    return;
  }
  
  if (payload.length() == 0) {
    publishCommandResult("ERROR", "Restore requires JSON backup data as payload", "system/calibration_restore");
    return;
  }
  
  // Simple JSON parsing for restore (basic implementation)
  // For production, you'd want a proper JSON parser
  bool restored = false;
  int restoredCount = 0;
  
  calibrationPrefs.begin("cal_state", false);

  String sensors[] = {"scd4x", "scd30", "cm1106c", "c8co2", "sgp41", "bme680", "pms5003", "tmp117"};
  for (String sensorName : sensors) {
    // Look for sensor data in JSON payload
    String sensorKey = "\"" + sensorName + "\":";
    int sensorStart = payload.indexOf(sensorKey);
    
    if (sensorStart >= 0) {
      // Extract calibrating status
      String calibratingKey = "\"calibrating\":";
      int calibratingPos = payload.indexOf(calibratingKey, sensorStart);
      if (calibratingPos >= 0) {
        bool isCalibrating = payload.substring(calibratingPos + calibratingKey.length(), calibratingPos + calibratingKey.length() + 4) == "true";
        
        if (isCalibrating) {
          // Extract unix_start
          String startKey = "\"unix_start\":";
          int startPos = payload.indexOf(startKey, sensorStart);
          if (startPos >= 0) {
            int startEnd = payload.indexOf(',', startPos);
            if (startEnd < 0) startEnd = payload.indexOf('}', startPos);
            if (startEnd > startPos) {
              unsigned long unixStart = payload.substring(startPos + startKey.length(), startEnd).toInt();
              
              // Extract period
              String periodKey = "\"period\":";
              int periodPos = payload.indexOf(periodKey, sensorStart);
              if (periodPos >= 0) {
                int periodEnd = payload.indexOf(',', periodPos);
                if (periodEnd < 0) periodEnd = payload.indexOf('}', periodPos);
                if (periodEnd > periodPos) {
                  unsigned long period = payload.substring(periodPos + periodKey.length(), periodEnd).toInt();
                  
                  // Restore to NVS
                  String prefix = sensorName + "_";
                  calibrationPrefs.putBool((prefix + "cal").c_str(), true);
                  calibrationPrefs.putULong((prefix + "unix_start").c_str(), unixStart);
                  calibrationPrefs.putULong((prefix + "period").c_str(), period);
                  calibrationPrefs.putBool((prefix + "supp").c_str(), SUPPRESS_DATA_DURING_CALIBRATION);
                  
                  restoredCount++;
                  restored = true;
                  
                  Serial.print("Restored calibration for "); Serial.print(sensorName);
                  Serial.print(": period="); Serial.print(period); 
                  Serial.print("h, started at Unix time "); Serial.println(unixStart);
                }
              }
            }
          }
        }
      }
    }
  }
  
  if (restored) {
    calibrationPrefs.putULong("save_time", getCurrentUnixTime());
    calibrationPrefs.putString("firmware_version", "restored");
  }
  
  calibrationPrefs.end();
  
  if (restored) {
    publishCommandResult("SUCCESS", "Restored calibration data for " + String(restoredCount) + " sensors. Restart device to apply.", "system/calibration_restore");
    Serial.println("Calibration data restored from backup - restart device to load restored state");
  } else {
    publishCommandResult("ERROR", "Failed to parse backup data or no valid calibration data found", "system/calibration_restore");
  }
}

void handleFactoryReset(const String& payload) {
  // This is a placeholder - implement based on your needs
  publishCommandResult("INFO", "Factory reset not implemented yet", "system/factory_reset");
}

void handleTestingMode(bool enable, const String& payload) {
  setTestingMode(enable);

  String action = enable ? "enabled" : "disabled";
  String message = "Testing mode " + action + ". ";

  if (enable) {
    message += "Sensor data publication SUPPRESSED for global data quality.";
  } else {
    message += "Sensor data publication RESUMED.";
  }

  publishCommandResult("SUCCESS", message, "system/testing_" + String(enable ? "enable" : "disable"));

  // Also update ASC status to reflect new state immediately
  if (sensorLocations.scd4x.found) {
    publishSCD4xASCStatus();
  }
}

void handleSyslogEnable(bool enable) {
  syslog.setEnabled(enable);

  String action = enable ? "enabled" : "disabled";
  String message = "Syslog logging " + action + ".";

  if (enable) {
    message += " Logs will be sent to " + String(SYSLOG_SERVER) + ":" + String(SYSLOG_PORT);
  } else {
    message += " Remote logging stopped.";
  }

  publishCommandResult("SUCCESS", message, "system/syslog_" + String(enable ? "enable" : "disable"));
}

void handleSyslogStatus() {
  String status = syslog.isEnabled() ? "enabled" : "disabled";
  String message = "Syslog: " + status;
  message += ", server: " + String(SYSLOG_SERVER) + ":" + String(SYSLOG_PORT);
  message += ", config: " + String(ENABLE_SYSLOG ? "enabled" : "disabled") + " at compile time";

  publishCommandResult("INFO", message, "system/syslog_status");
}

void handleCalibrationStatus(const String& payload) {
  String overallStatus = getOverallCalibrationStatus();
  bool globalDataActive = !sensorCalibrationStates.globalTestingMode;
  
  String message = "Overall calibration state: " + overallStatus + ". ";
  message += "Global data publishing: " + String(globalDataActive ? "ACTIVE" : "SUPPRESSED (testing mode)") + ". ";
  
  // Add individual sensor status
  String sensors[] = {"scd4x", "scd30", "cm1106c", "c8co2", "sgp41", "bme680", "pms5003", "tmp117"};
  int calibratingCount = 0;

  for (String sensorName : sensors) {
    SensorCalibrationState* state = getSensorCalibrationState(sensorName);
    if (state != nullptr && state->isCalibrating) {
      calibratingCount++;
      unsigned long remainingMs = getRemainingCalibrationMs(sensorName);
      String timeRemaining = formatRemainingTime(remainingMs);
      message += sensorName + " calibrating (" + timeRemaining + " remaining), ";
    }
  }
  
  if (calibratingCount == 0) {
    message += "All sensors calibrated.";
  } else {
    message = message.substring(0, message.length() - 2); // Remove trailing comma
    message += ".";
  }
  
  publishCommandResult("SUCCESS", message, "system/calibration_status");
}

// ======================
// SPECIFIC SENSOR IMPLEMENTATIONS
// ======================

// SCD4x calibration implementation
void handleSCD4xCalibration(const String& payload) {
  String cmd = payload;
  cmd.trim();
  
  uint16_t targetCO2 = 400; // Default outdoor CO2 level
  
  if (cmd.length() > 0) {
    int ppmValue = cmd.toInt();
    if (ppmValue >= 300 && ppmValue <= 2000) {
      targetCO2 = (uint16_t)ppmValue;
    } else {
      publishCommandResult("ERROR", "Invalid ppm value. Must be between 300-2000 ppm", "sensor/scd4x/calibrate");
      return;
    }
  }
  
  Serial.print("SCD4x: Starting forced recalibration to ");
  Serial.print(targetCO2);
  Serial.println(" ppm...");
  
  // Stop periodic measurements for calibration
  if (!scd4x.stopPeriodicMeasurement()) {
    publishCommandResult("ERROR", "Failed to stop periodic measurement before calibration", "sensor/scd4x/calibrate");
    return;
  }
  
  delay(500); // Wait for sensor to stop
  
  // Perform the forced recalibration
  float frcCorrection = 0;
  if (!scd4x.performForcedRecalibration(targetCO2, &frcCorrection)) {
    String errorMsg = "Calibration failed";
    Serial.println("SCD4x: " + errorMsg);
    publishCommandResult("ERROR", errorMsg, "sensor/scd4x/calibrate");
  } else {
    String successMsg = "Calibration successful. Target: " + String(targetCO2) + " ppm, FRC correction: " + String(frcCorrection);
    Serial.println("SCD4x: " + successMsg);
    publishCommandResult("SUCCESS", successMsg, "sensor/scd4x/calibrate");
    
    // CRITICAL: Set calibration method to manual FRC and end calibration period
    String reference = String(targetCO2) + "ppm_manual";
    setCalibrationMethod("scd4x", "frc_manual", "high", reference);
    resetCalibrationState("scd4x"); // End any calibration period - sensor is now calibrated
    Serial.println("SCD4x: Calibration method updated to FRC manual with high confidence");
  }
  
  // Restart periodic measurements
  delay(1000);
  if (!scd4x.startPeriodicMeasurement()) {
    Serial.println("SCD4x: Warning - failed to restart periodic measurement after calibration");
  } else {
    Serial.println("SCD4x: Periodic measurement restarted after calibration");
  }
}

// SCD4x reset implementation
void handleSCD4xReset() {
  Serial.println("SCD4x: Performing sensor reset...");
  
  scd4x.stopPeriodicMeasurement(); // Don't worry about return value during reset
  
  delay(500);
  
  // Reinitialize sensor on the correct I2C bus
  TwoWire* scd4xWire = getI2CBus(sensorLocations.scd4x.bus);
  scd4x.begin(*scd4xWire);
  
  delay(500);
  
  if (!scd4x.startPeriodicMeasurement()) {
    publishCommandResult("ERROR", "SCD4x reset failed - error restarting measurements", "sensor/scd4x/reset");
  } else {
    publishCommandResult("SUCCESS", "SCD4x reset and reinitialized successfully", "sensor/scd4x/reset");
  }
}

// SCD4x ASC (Automatic Self-Calibration) control
void handleSCD4xASC(bool enable) {
  Serial.print("SCD4x: "); Serial.print(enable ? "Enabling" : "Disabling"); Serial.println(" Automatic Self-Calibration...");
  
  scd4x.stopPeriodicMeasurement();
  delay(500);
  
  if (!scd4x.setAutomaticSelfCalibrationEnabled(enable)) {
    publishCommandResult("ERROR", "Failed to set ASC", "sensor/scd4x/asc_" + String(enable ? "enable" : "disable"));
  } else {
    String message = "ASC (Automatic Self-Calibration) " + String(enable ? "enabled" : "disabled") + " successfully";
    Serial.println("SCD4x: " + message);
    publishCommandResult("SUCCESS", message, "sensor/scd4x/asc_" + String(enable ? "enable" : "disable"));
    
    // CRITICAL: Update calibration method based on ASC setting
    if (enable) {
      // ASC enabled - start learning mode
      setCalibrationMethod("scd4x", "asc_learning", "low", "7_day_automatic");
      SensorCalibrationState* state = getSensorCalibrationState("scd4x");
      if (state == nullptr || !state->isCalibrating) {
        Serial.println("SCD4x: Starting ESP32 calibration period for ASC learning (7 days)");
        startSensorCalibrationPeriod("scd4x");
      }
    } else {
      // ASC disabled - set to factory default
      setCalibrationMethod("scd4x", "factory", "medium", "asc_disabled");
      resetCalibrationState("scd4x");
      Serial.println("SCD4x: Calibration period cleared - ASC disabled");
    }
    
    // Publish updated status immediately
    publishSCD4xASCStatus();
  }
  
  // Restart measurements
  delay(500);
  if (!scd4x.startPeriodicMeasurement() && ENABLE_DEBUG_OUTPUT) {
    Serial.println("SCD4x: Warning after ASC change - failed to restart measurements");
  }
}

// SCD4x self-test implementation
void handleSCD4xSelfTest() {
  Serial.println("SCD4x: Performing self test...");
  
  scd4x.stopPeriodicMeasurement();
  delay(500);
  
  // SparkFun library has performSelfTest method - use it if available
  if (scd4x.performSelfTest()) {
    publishCommandResult("SUCCESS", "SCD4x self-test completed successfully", "sensor/scd4x/selftest");
  } else {
    publishCommandResult("ERROR", "SCD4x self-test failed", "sensor/scd4x/selftest");
  }
  
  // Restart measurements
  if (!scd4x.startPeriodicMeasurement() && ENABLE_DEBUG_OUTPUT) {
    Serial.println("SCD4x: Warning after self test - failed to restart measurements");
  }
}

// SCD4x pressure compensation status
void handleSCD4xPressureCompensationStatus() {
  float bestPressure = getBestPressureForSCD4x();
  
  String message = "Pressure compensation status: ";
  
  if (bestPressure != -999.0f) {
    message += "ACTIVE using " + String(bestPressure, 1) + " hPa from ";
    message += currentBestReadings.pressure.sensor_name;
    message += " (priority " + String(currentBestReadings.pressure.priority) + "). ";
    message += "Compensation improves CO2 accuracy by ~2-5% depending on altitude/weather.";
  } else {
    message += "INACTIVE - No pressure sensor available. ";
    message += "CO2 readings may be 2-5% less accurate depending on altitude and weather conditions.";
  }
  
  publishCommandResult("SUCCESS", message, "sensor/scd4x/pressure_compensation_status");
  
  // ALWAYS log detailed diagnostics regardless of debug setting
  Serial.println("=== SCD4x Pressure Compensation Diagnostics ===");
  Serial.print("BME680 Available: "); Serial.println(bme680Available ? "YES" : "NO");
  Serial.print("BMP280 Available: "); Serial.println(bmp280Available ? "YES" : "NO");
  
  if (currentBestReadings.pressure.valid) {
    Serial.print("Current best pressure sensor: "); Serial.print(currentBestReadings.pressure.sensor_name);
    Serial.print(" (priority "); Serial.print(currentBestReadings.pressure.priority);
    Serial.print(", value: "); Serial.print(currentBestReadings.pressure.value, 2); Serial.println(" hPa)");
  } else {
    Serial.println("No valid pressure reading in hierarchy");
  }
  
  Serial.print("Pressure hierarchy valid: "); Serial.println(currentBestReadings.pressure.valid ? "YES" : "NO");
  Serial.println("===============================================");
}

// SCD4x factory reset implementation - resets sensor to factory calibration
void handleSCD4xFactoryReset() {
  Serial.println("SCD4x: Performing FACTORY RESET - this will wipe all calibration data...");
  
  // Stop periodic measurements
  scd4x.stopPeriodicMeasurement();
  delay(500);
  
  // Perform factory reset - this command resets all calibration data to factory defaults
  if (!scd4x.performFactoryReset()) {
    publishCommandResult("ERROR", "SCD4x factory reset command failed", "sensor/scd4x/factory_reset");
    return;
  }
  
  Serial.println("SCD4x: Factory reset command sent successfully");
  delay(1200); // Factory reset takes 1200ms according to datasheet
  
  // Reinitialize sensor communication
  TwoWire* scd4xWire = getI2CBus(sensorLocations.scd4x.bus);
  scd4x.begin(*scd4xWire);
  delay(500);
  
  // Restart periodic measurements
  if (!scd4x.startPeriodicMeasurement()) {
    publishCommandResult("ERROR", "Factory reset completed but failed to restart measurements", "sensor/scd4x/factory_reset");
  } else {
    String message = "Factory reset completed successfully. Sensor restored to factory calibration. ";
    message += "All custom calibration data (FRC, ASC learning) has been wiped. ";
    message += "ASC is now disabled - enable manually if needed for your deployment.";
    
    publishCommandResult("SUCCESS", message, "sensor/scd4x/factory_reset");
    Serial.println("SCD4x: Factory reset complete - sensor using original factory calibration");
    Serial.println("SCD4x:  All custom calibration data has been permanently erased");
    Serial.println("SCD4x: ASC (Automatic Self-Calibration) is now DISABLED");
    Serial.println("SCD4x: You may need to re-enable ASC or perform manual calibration");
    
    // CRITICAL: Clear ESP32's stored calibration state and reset to factory method
    Serial.println("SCD4x: Updating ESP32 calibration state to match factory reset...");
    setCalibrationMethod("scd4x", "factory", "medium", "factory_reset");
    resetCalibrationState("scd4x");
    Serial.println("SCD4x: ESP32 calibration state reset to factory - factory reset fully complete");
  }
}

// SCD4x pressure compensation manual apply
void handleSCD4xPressureCompensationApply() {
  Serial.println("SCD4x: Manual pressure compensation trigger requested...");
  
  float bestPressure = getBestPressureForSCD4x();
  
  if (bestPressure != -999.0f && bestPressure >= 300.0f && bestPressure <= 1200.0f) {
    uint16_t pressureMillibar = (uint16_t)(bestPressure + 0.5);
    
    Serial.print("SCD4x: Applying manual pressure compensation - ");
    Serial.print(pressureMillibar); Serial.print(" mbar from ");
    Serial.println(currentBestReadings.pressure.sensor_name);
    
    // Stop measurements
    if (!scd4x.stopPeriodicMeasurement()) {
      publishCommandResult("ERROR", "Failed to stop measurement for pressure compensation", "sensor/scd4x/pressure_compensation_apply");
      return;
    }
    
    delay(500);
    
    // Apply pressure compensation using SparkFun library's setAmbientPressure method
    // Note: SparkFun library expects pressure in Pascals, not millibars
    float pressurePascals = bestPressure * 100.0f; // Convert hPa to Pa
    
    if (scd4x.setAmbientPressure(pressurePascals)) {
      String successMsg = "Manual pressure compensation applied: " + String(bestPressure, 1) + " hPa (" + String(pressurePascals, 0) + " Pa) from " + currentBestReadings.pressure.sensor_name;
      Serial.println("SCD4x: " + successMsg);
      publishCommandResult("SUCCESS", successMsg, "sensor/scd4x/pressure_compensation_apply");
    } else {
      String errorMsg = "Failed to set pressure compensation to " + String(pressurePascals, 0) + " Pa";
      publishCommandResult("ERROR", errorMsg, "sensor/scd4x/pressure_compensation_apply");
    }
    
    // Restart measurements
    delay(500);
    if (!scd4x.startPeriodicMeasurement()) {
      Serial.println("SCD4x: Warning - failed to restart measurement after manual pressure compensation");
    }
    
  } else if (bestPressure != -999.0f) {
    String errorMsg = "Pressure " + String(bestPressure, 1) + " hPa is outside valid range (300-1200 hPa)";
    publishCommandResult("ERROR", errorMsg, "sensor/scd4x/pressure_compensation_apply");
  } else {
    publishCommandResult("ERROR", "No pressure sensor available for compensation", "sensor/scd4x/pressure_compensation_apply");
  }
}

// BME680 gas heater control
void handleBME680GasHeater(const String& payload) {
  String cmd = payload;
  cmd.trim();
  cmd.toUpperCase();
  
  if (cmd == "ON" || cmd == "ENABLE") {
    bme680.setGasHeater(320, 150); // 320°C for 150ms
    publishCommandResult("SUCCESS", "BME680 gas heater enabled (320°C, 150ms)", "sensor/bme680/gas_heater");
  } else if (cmd == "OFF" || cmd == "DISABLE") {
    bme680.setGasHeater(0, 0); // Disable gas heater
    publishCommandResult("SUCCESS", "BME680 gas heater disabled", "sensor/bme680/gas_heater");
  } else {
    publishCommandResult("ERROR", "Invalid gas heater command. Use: ON/ENABLE or OFF/DISABLE", "sensor/bme680/gas_heater");
  }
}

// BMP280 diagnostic
void handleBMP280Diagnostic() {
  // If BMP280 is on secondary bus and board supports dual I2C, temporarily switch to it
  if (sensorLocations.bmp280.bus == 1 && HAS_DUAL_I2C()) {
    Wire.end();
    delay(50);
    Wire.begin(g_boardConfig.sda_pin2, g_boardConfig.scl_pin2, 100000);
    delay(50);
  }

  String diagnostic;
  if (bmp280.takeForcedMeasurement()) {
    float pressure = bmp280.readPressure() / 100.0F; // hPa
    float temperature = bmp280.readTemperature();

    diagnostic = "BMP280 diagnostic - Pressure: " + String(pressure, 2) + " hPa, Temperature: " + String(temperature, 2) + "°C";

    if (pressure < 900 || pressure > 1100) {
      diagnostic += " [WARNING: Pressure out of normal range]";
    }
  } else {
    diagnostic = "BMP280 diagnostic - Forced measurement failed";
  }

  // Switch back to primary bus if needed
  if (sensorLocations.bmp280.bus == 1 && HAS_DUAL_I2C()) {
    Wire.end();
    delay(50);
    Wire.begin(g_boardConfig.sda_pin, g_boardConfig.scl_pin, 100000);
    delay(50);
  }

  publishCommandResult("SUCCESS", diagnostic, "sensor/bmp280/diagnostic");
}

// Publish command result back to MQTT
void publishCommandResult(const String& status, const String& message, const String& commandPath) {
  if (!wifiConnected || !client.connected()) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("Command result (MQTT offline): "); Serial.print(status);
      Serial.print(" - "); Serial.println(message);
    }
    return;
  }
  
  String topic = String(deviceTopicPrefix) + "/status/command";
  String payload = "{\"command\":\"" + commandPath + "\",\"status\":\"" + status + "\",\"message\":\"" + message + "\",\"timestamp\":\"" + 
                   (USE_UTC_TIMESTAMPS ? getTimestampUTC() : getTimestamp()) + "\"}";
  
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("MQTT Command Result: "); Serial.print(status);
    Serial.print(" - "); Serial.println(message);
  }
  
  client.publish(topic.c_str(), payload.c_str());
}

// Subscribe to all command topics when MQTT connects
void subscribeToCommandTopics() {
  if (!wifiConnected || !client.connected()) {
    return;
  }
  
  // Subscribe to wildcard command topic to catch all commands
  String commandTopic = String(deviceTopicPrefix) + "/command/+/+/+";
  
  if (client.subscribe(commandTopic.c_str())) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("MQTT: Subscribed to command topic: ");
      Serial.println(commandTopic);
    }
  } else {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("MQTT: Failed to subscribe to command topic: ");
      Serial.println(commandTopic);
    }
  }
  
  // Also subscribe to shorter command paths for simpler system commands
  String systemCommandTopic = String(deviceTopicPrefix) + "/command/system/+";
  
  if (client.subscribe(systemCommandTopic.c_str())) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("MQTT: Subscribed to system command topic: ");
      Serial.println(systemCommandTopic);
    }
  } else {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("MQTT: Failed to subscribe to system command topic: ");
      Serial.println(systemCommandTopic);
    }
  }
  
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.println("MQTT: Command subscriptions completed");
  }
}

// Publish SGP41 baseline status
void publishSGP41BaselineStatus(int32_t voc_index, int32_t nox_index, uint16_t voc_raw, uint16_t nox_raw) {
  if (!sgp41Available || !wifiConnected || !client.connected()) {
    return;
  }
  
  String topic = String(deviceTopicPrefix) + "/status/sgp41/baseline";
  String payload;
  
  // Determine baseline status based on index values and raw readings
  bool baselineEstablished = true;
  String status = "stable";
  
  // Check if baseline is still establishing (typically first 12+ hours)
  if ((voc_index == 0 && nox_index == 0) || (voc_index < 50 && nox_index < 50)) {
    baselineEstablished = false;
    status = "learning";
  }
  
  // Calculate approximate runtime (estimation based on reading intervals)
  unsigned long runtimeHours = millis() / 3600000; // Convert ms to hours
  
  String timestamp = USE_UTC_TIMESTAMPS ? getTimestampUTC() : getTimestamp();
  payload = "{\"status\":\"" + status + "\",\"baseline_established\":" + String(baselineEstablished ? "true" : "false") + ",\"runtime_hours\":" + String(runtimeHours) + ",\"voc_raw\":" + String(voc_raw) + ",\"nox_raw\":" + String(nox_raw) + ",\"timestamp\":\"" + timestamp + "\",\"device_id\":\"" + String(deviceID) + "\"}";
  
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("SGP41 Baseline Status: "); Serial.print(status);
    Serial.print(" (Runtime: "); Serial.print(runtimeHours); Serial.print("h)");
    Serial.println();
  }
  
  client.publish(topic.c_str(), payload.c_str());
}

// Publish SCD4x ASC (Automatic Self-Calibration) status
void publishSCD4xASCStatus() {
  // Only publish if sensor is physically present (regardless of working status)
  if (!sensorLocations.scd4x.found || !wifiConnected || !client.connected()) {
    return;
  }
  
  String topic = String(deviceTopicPrefix) + "/status/scd4x/asc";
  String payload;
  String status;
  uint16_t error_code = 0;
  
  if (scd4xAvailable) {
    // Sensor is working - get actual ASC status with proper state management
    
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("SCD4x ASC Status: Querying ASC status (requires stopping measurement)...");
    }
    
    // SCD4x requires measurements to be stopped to read ASC status reliably
    if (!scd4x.stopPeriodicMeasurement()) {
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("SCD4x ASC Status: Warning - failed to stop measurement for ASC query");
      }
    }
    
    // Wait for sensor to stop
    delay(500);
    
    // Now try to read ASC status - SparkFun library returns boolean directly
    bool ascEnabled = scd4x.getAutomaticSelfCalibrationEnabled();
    status = ascEnabled ? "enabled" : "disabled";
    error_code = 0; // SparkFun library doesn't provide error codes for this method
    
    // Always restart measurements
    delay(100);
    if (!scd4x.startPeriodicMeasurement()) {
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("SCD4x ASC Status: Warning - failed to restart measurement");
      }
    }
    
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("SCD4x ASC Status: "); Serial.println(ascEnabled ? "Enabled" : "Disabled");
      Serial.println("SCD4x ASC Status: Periodic measurement restarted");
    }
  } else {
    // Sensor detected but not working (initialization failed, etc.)
    status = "sensor_error";
    error_code = 500; // Generic sensor failure code
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("SCD4x ASC Status: Sensor detected but not functioning");
    }
  }
  
  // Build payload with calibration state information
  String timestamp = USE_UTC_TIMESTAMPS ? getTimestampUTC() : getTimestamp();
  String calibrationStatus = getSensorCalibrationStatus("scd4x");
  bool dataActive = shouldPublishSensorData("scd4x");
  
  payload = "{\"status\":\"" + status + "\",\"error_code\":" + String(error_code) + ",\"calibration_state\":\"" + calibrationStatus + "\",\"data_publishing\":" + String(dataActive ? "true" : "false") + ",\"timestamp\":\"" + timestamp + "\",\"device_id\":\"" + String(deviceID) + "\"}";
  
  client.publish(topic.c_str(), payload.c_str());
}

// Publish BME680 calibration status
void publishBME680CalibrationStatus() {
  if (!bme680Available || !wifiConnected || !client.connected()) {
    return;
  }
  
  String topic = String(deviceTopicPrefix) + "/status/bme680/calibration";
  String calibrationStatus = getSensorCalibrationStatus("bme680");
  bool dataActive = shouldPublishSensorData("bme680");
  
  String status;
  if (calibrationStatus == "calibrated") {
    status = "stable";
  } else if (calibrationStatus.startsWith("calibrating_")) {
    status = "stabilizing";
  } else {
    status = calibrationStatus;
  }
  
  String timestamp = USE_UTC_TIMESTAMPS ? getTimestampUTC() : getTimestamp();
  String payload = "{\"status\":\"" + status + "\",\"calibration_state\":\"" + calibrationStatus + "\",\"data_publishing\":" + String(dataActive ? "true" : "false") + ",\"timestamp\":\"" + timestamp + "\",\"device_id\":\"" + String(deviceID) + "\"}";
  
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("BME680 Calibration Status: "); Serial.println(status);
  }
  
  client.publish(topic.c_str(), payload.c_str());
}

// Publish PMS5003 calibration status
void publishPMS5003CalibrationStatus() {
  if (!pms5003Available || !wifiConnected || !client.connected()) {
    return;
  }
  
  String topic = String(deviceTopicPrefix) + "/status/pms5003/calibration";
  String calibrationStatus = getSensorCalibrationStatus("pms5003");
  bool dataActive = shouldPublishSensorData("pms5003");
  
  String status;
  if (calibrationStatus == "calibrated") {
    status = "ready";
  } else if (calibrationStatus.startsWith("calibrating_")) {
    status = "warming_up";
  } else {
    status = calibrationStatus;
  }
  
  String timestamp = USE_UTC_TIMESTAMPS ? getTimestampUTC() : getTimestamp();
  String payload = "{\"status\":\"" + status + "\",\"calibration_state\":\"" + calibrationStatus + "\",\"data_publishing\":" + String(dataActive ? "true" : "false") + ",\"timestamp\":\"" + timestamp + "\",\"device_id\":\"" + String(deviceID) + "\"}";
  
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("PMS5003 Calibration Status: "); Serial.println(status);
  }
  
  client.publish(topic.c_str(), payload.c_str());
}

// ============================================================================
// PROTOBUF HELPER FUNCTIONS
// ============================================================================

#if ENABLE_PROTOBUF

// Convert MAC address to uint64 for compact device ID
uint64_t getMacAsUint64() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  uint64_t macInt = 0;
  for (int i = 0; i < 6; i++) {
    macInt = (macInt << 8) | mac[i];
  }
  return macInt;
}

// Get timezone offset in seconds
int32_t getTimezoneOffsetSec() {
  return gmt_offset_sec + daylight_offset_sec;
}

// Get latitude from runtime location
double getLatitude() {
  return runtimeLocation.latitude;
}

// Get longitude from runtime location
double getLongitude() {
  return runtimeLocation.longitude;
}

// Get altitude in meters (v2.1)
// Returns GPS altitude if available, otherwise FIXED_ALTITUDE
float getAltitudeMeters() {
  #if ENABLE_GPS
  if (gps.altitude.isValid()) {
    return gps.altitude.meters();
  }
  #endif
  return FIXED_ALTITUDE;
}

// Get node name for device identification (v2.1)
// Returns combined NODE_NAME_PREFIX + "_" + DEVICE_LOCATION (or just DEVICE_LOCATION if prefix empty)
const char* getNodeName() {
  static char nodeName[50];  // Max 24 + 1 + 24 = 49 chars + null
  if (strlen(NODE_NAME_PREFIX) > 0) {
    snprintf(nodeName, sizeof(nodeName), "%s_%s", NODE_NAME_PREFIX, DEVICE_LOCATION);
  } else {
    strncpy(nodeName, DEVICE_LOCATION, sizeof(nodeName) - 1);
    nodeName[sizeof(nodeName) - 1] = '\0';
  }
  return nodeName;
}

// Get sanitized hostname from node name for DHCP identification
// Replaces spaces and underscores with hyphens, removes invalid chars
String getSanitizedHostname() {
  String hostname = String(getNodeName());
  hostname.replace(" ", "-");
  hostname.replace("_", "-");
  // Remove any other invalid hostname characters (only alphanumeric and hyphen allowed)
  String sanitized = "";
  for (int i = 0; i < hostname.length(); i++) {
    char c = hostname.charAt(i);
    if (isalnum(c) || c == '-') {
      sanitized += c;
    }
  }
  // Hostname can't start or end with hyphen
  while (sanitized.startsWith("-")) sanitized.remove(0, 1);
  while (sanitized.endsWith("-")) sanitized.remove(sanitized.length() - 1);
  return sanitized;
}

// Get node info for physical setup description (v2.2)
// Returns NODE_INFO - describes the physical installation for data analysis
const char* getNodeInfo() {
  return NODE_INFO;
}

// Get node info URL for detailed documentation (v2.2)
// Returns NODE_INFO_URL - link to detailed setup documentation
const char* getNodeInfoUrl() {
  return NODE_INFO_URL;
}

// Get firmware version string (v2.1)
const char* getFirmwareVersion() {
  return FIRMWARE_VERSION;
}

#endif // ENABLE_PROTOBUF

// Simple sensor data publishing for status/diagnostic values
// Note: Main sensor data now uses publishAllSensorsMQTT() with v2 protobuf
void publishSensorData(const char* topicSuffix, float value) {
  // TEMPORARILY BYPASS COMPLEX FUNCTIONS TO ISOLATE STACK OVERFLOW
  if (!wifiConnected || !client.connected()) {
    return;
  }

  // deviceTopicPrefix is now without trailing slash for v2 format
  // Add separator for subtopics (status, diagnostic, etc.)
  String topic = String(deviceTopicPrefix) + "/" + topicSuffix;
  String payload = String(value, 2);

  client.publish(topic.c_str(), payload.c_str());
}

// New function with calibration metadata support + optional protobuf encoding
void publishSensorDataWithCalibration(const char* topicSuffix, float value, const String& sensorName) {
  if (!ENABLE_MQTT) {
    return;  // Skip MQTT publishing if disabled
  }
  
  if (!wifiConnected || !client.connected()) {
    if (ENABLE_DEBUG_OUTPUT) Serial.println("MQTT: Not connected, skipping publish");
    return;
  }
  
  // Prevent publishing without valid timestamps
  if (getCurrentUnixTime() == 0) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("MQTT: Skipping publish - no time sync available");
    }
    return;
  }
  
  // ========== JSON PATH (for diagnostic/status messages) ==========
  // Note: Main sensor data now uses publishAllSensorsMQTT() with v2 protobuf
  // This function is kept for diagnostic/status publishing
  // Use stack buffer to avoid heap fragmentation
  static char topic[128];
  static char payload[1024];

  // deviceTopicPrefix is now without trailing slash for v2 format
  // Add separator for subtopics
  snprintf(topic, sizeof(topic), "%s/%s", deviceTopicPrefix, topicSuffix);
  
  // Get timestamp string once
  String timestampStr = USE_UTC_TIMESTAMPS ? getTimestampUTC() : getTimestamp();
  
  // Build JSON payload using snprintf
  int pos = snprintf(payload, sizeof(payload), 
    "{\"value\":%.2f,\"timestamp\":\"%s\",\"device_id\":\"%s\"",
    value, timestampStr.c_str(), deviceID);
  
  if (pos < sizeof(payload) - 150) {
    pos += snprintf(payload + pos, sizeof(payload) - pos,
      ",\"country\":\"%s\",\"subdivision\":\"%s\",\"deployment_type\":\"%s\",\"deployment_location\":\"%s\"",
      COUNTRY_CODE, SUBDIVISION_CODE, getDeploymentTypeString(), DEVICE_LOCATION);
  }
  
  // Add location if available
  if (pos < sizeof(payload) - 200 && runtimeLocation.includeInMQTT) {
    pos += snprintf(payload + pos, sizeof(payload) - pos,
      ",\"latitude\":%.6f,\"longitude\":%.6f,\"location_source\":\"%s\"",
      runtimeLocation.latitude, runtimeLocation.longitude,
      runtimeLocation.overrideActive ? "mqtt_override" : "firmware_default");
  }
  
  // Add timezone
  if (pos < sizeof(payload) - 50) {
    int total_offset_sec = gmt_offset_sec + daylight_offset_sec;
    pos += snprintf(payload + pos, sizeof(payload) - pos,
      ",\"timezone_offset_sec\":%d", total_offset_sec);
  }
  
  // Add calibration metadata if sensor name provided
  if (pos < sizeof(payload) - 200 && sensorName.length() > 0) {
    SensorCalibrationState* state = getSensorCalibrationState(sensorName);
    if (state != nullptr) {
      pos += snprintf(payload + pos, sizeof(payload) - pos,
        ",\"calibration_method\":\"%s\",\"calibration_confidence\":\"%s\"",
        state->calibrationMethod.c_str(), state->calibrationConfidence.c_str());
      
      if (pos < sizeof(payload) - 100 && state->calibrationTimestamp > 0) {
        pos += snprintf(payload + pos, sizeof(payload) - pos,
          ",\"calibration_date\":%lu", state->calibrationTimestamp);
      }
      
      // Data quality assessment  
      const char* dataQuality = "unknown";
      if (state->calibrationMethod == "frc_manual" || state->calibrationMethod == "asc_established") {
        dataQuality = "production_ready";
      } else if (state->calibrationMethod == "asc_learning") {
        dataQuality = "calibration_in_progress";
      } else if (state->calibrationMethod == "factory") {
        dataQuality = "factory_default";
      }
      
      if (pos < sizeof(payload) - 50) {
        pos += snprintf(payload + pos, sizeof(payload) - pos,
          ",\"data_quality\":\"%s\"", dataQuality);
      }
    }
  }
  
  // Close JSON
  if (pos < sizeof(payload) - 2) {
    strcat(payload, "}");
  }
  
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print("MQTT: Publishing "); Serial.print(topic); Serial.print(" = "); Serial.println(payload);
  }

  client.publish(topic, payload);
}

// ========================================
// V2 PROTOBUF CONSOLIDATED MQTT PUBLISHING
// ========================================
// Publishes all sensor readings in a single v2 protobuf message
// WiFi includes per-reading timestamps for accuracy
// LoRaWAN omits per-reading timestamps to save bandwidth (uses header timestamp)
void publishAllSensorsMQTT(uint32_t cycleTimestamp) {
  if (!ENABLE_MQTT) {
    return;
  }

  if (!wifiConnected || !client.connected()) {
    if (ENABLE_DEBUG_OUTPUT) Serial.println("MQTT: Not connected, skipping v2 publish");
    return;
  }

  // Prevent publishing without valid timestamps
  if (cycleTimestamp == 0) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("MQTT: Skipping v2 publish - no time sync available");
    }
    return;
  }

#if ENABLE_PROTOBUF
  // Use V2 consolidated encoder with per-reading timestamps for WiFi
  // WiFi has plenty of bandwidth, so we include accurate per-reading timestamps
  ProtobufEncoderV2 encoder;
  encoder.begin(cycleTimestamp, true);  // true = include per-reading timestamps

  if (ENABLE_DEBUG_OUTPUT) {
    Serial.println(F("MQTT: Encoding all sensors with V2 protobuf (with per-reading timestamps)..."));
  }

  // Add all valid sensor readings with their actual timestamps
  if (currentBestReadings.temperature.valid) {
    encoder.addTemperature(currentBestReadings.temperature.value,
                           currentBestReadings.temperature.sensor_name.c_str(),
                           currentBestReadings.temperature.timestamp);
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + Temperature: %.1f°C from %s @ %lu\n",
        currentBestReadings.temperature.value,
        currentBestReadings.temperature.sensor_name.c_str(),
        currentBestReadings.temperature.timestamp);
    }
  }

  if (currentBestReadings.humidity.valid) {
    encoder.addHumidity(currentBestReadings.humidity.value,
                        currentBestReadings.humidity.sensor_name.c_str(),
                        currentBestReadings.humidity.timestamp);
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + Humidity: %.1f%% from %s @ %lu\n",
        currentBestReadings.humidity.value,
        currentBestReadings.humidity.sensor_name.c_str(),
        currentBestReadings.humidity.timestamp);
    }
  }

  if (currentBestReadings.co2.valid) {
    encoder.addCO2(currentBestReadings.co2.value,
                   currentBestReadings.co2.sensor_name.c_str(),
                   currentBestReadings.co2.timestamp);
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + CO2: %.0fppm from %s @ %lu\n",
        currentBestReadings.co2.value,
        currentBestReadings.co2.sensor_name.c_str(),
        currentBestReadings.co2.timestamp);
    }
  }

  if (currentBestReadings.pressure.valid) {
    encoder.addPressure(currentBestReadings.pressure.value,
                        currentBestReadings.pressure.sensor_name.c_str(),
                        currentBestReadings.pressure.timestamp);
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + Pressure: %.2fhPa from %s @ %lu\n",
        currentBestReadings.pressure.value,
        currentBestReadings.pressure.sensor_name.c_str(),
        currentBestReadings.pressure.timestamp);
    }
  }

  if (currentBestReadings.voc.valid) {
    encoder.addVOC(currentBestReadings.voc.value,
                   currentBestReadings.voc.sensor_name.c_str(),
                   currentBestReadings.voc.timestamp);
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + VOC: %.0f from %s @ %lu\n",
        currentBestReadings.voc.value,
        currentBestReadings.voc.sensor_name.c_str(),
        currentBestReadings.voc.timestamp);
    }
  }

  if (currentBestReadings.nox.valid) {
    encoder.addNOx(currentBestReadings.nox.value,
                   currentBestReadings.nox.sensor_name.c_str(),
                   currentBestReadings.nox.timestamp);
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + NOx: %.0f from %s @ %lu\n",
        currentBestReadings.nox.value,
        currentBestReadings.nox.sensor_name.c_str(),
        currentBestReadings.nox.timestamp);
    }
  }

  // Particulate matter (PMS5003)
  if (currentBestReadings.pm1.valid) {
    encoder.addPM1(currentBestReadings.pm1.value,
                   currentBestReadings.pm1.sensor_name.c_str(),
                   currentBestReadings.pm1.timestamp);
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + PM1.0: %.1f μg/m³ from %s @ %lu\n",
        currentBestReadings.pm1.value,
        currentBestReadings.pm1.sensor_name.c_str(),
        currentBestReadings.pm1.timestamp);
    }
  }

  if (currentBestReadings.pm2_5.valid) {
    encoder.addPM25(currentBestReadings.pm2_5.value,
                    currentBestReadings.pm2_5.sensor_name.c_str(),
                    currentBestReadings.pm2_5.timestamp);
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + PM2.5: %.1f μg/m³ from %s @ %lu\n",
        currentBestReadings.pm2_5.value,
        currentBestReadings.pm2_5.sensor_name.c_str(),
        currentBestReadings.pm2_5.timestamp);
    }
  }

  if (currentBestReadings.pm10.valid) {
    encoder.addPM10(currentBestReadings.pm10.value,
                    currentBestReadings.pm10.sensor_name.c_str(),
                    currentBestReadings.pm10.timestamp);
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + PM10: %.1f μg/m³ from %s @ %lu\n",
        currentBestReadings.pm10.value,
        currentBestReadings.pm10.sensor_name.c_str(),
        currentBestReadings.pm10.timestamp);
    }
  }

  // DC Power Monitor readings (INA219)
  if (currentBestReadings.voltage.valid) {
    encoder.addVoltage(currentBestReadings.voltage.value,
                       currentBestReadings.voltage.sensor_name.c_str(),
                       currentBestReadings.voltage.timestamp);
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + Voltage: %.2f V from %s @ %lu\n",
        currentBestReadings.voltage.value,
        currentBestReadings.voltage.sensor_name.c_str(),
        currentBestReadings.voltage.timestamp);
    }
  }

  if (currentBestReadings.current.valid) {
    encoder.addCurrent(currentBestReadings.current.value,
                       currentBestReadings.current.sensor_name.c_str(),
                       currentBestReadings.current.timestamp);
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + Current: %.2f mA from %s @ %lu\n",
        currentBestReadings.current.value,
        currentBestReadings.current.sensor_name.c_str(),
        currentBestReadings.current.timestamp);
    }
  }

  if (currentBestReadings.power.valid) {
    encoder.addPower(currentBestReadings.power.value,
                     currentBestReadings.power.sensor_name.c_str(),
                     currentBestReadings.power.timestamp);
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + Power: %.2f mW from %s @ %lu\n",
        currentBestReadings.power.value,
        currentBestReadings.power.sensor_name.c_str(),
        currentBestReadings.power.timestamp);
    }
  }

  // Battery level (PMU)
  if (currentBestReadings.battery.valid) {
    encoder.addBattery(currentBestReadings.battery.value,
                       currentBestReadings.battery.sensor_name.c_str(),
                       currentBestReadings.battery.timestamp);
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + Battery: %.1f%% from %s @ %lu\n",
        currentBestReadings.battery.value,
        currentBestReadings.battery.sensor_name.c_str(),
        currentBestReadings.battery.timestamp);
    }
  }

  // Gas resistance (BME680)
  if (currentBestReadings.gas_resistance.valid) {
    encoder.addGasResistance(currentBestReadings.gas_resistance.value,
                             currentBestReadings.gas_resistance.sensor_name.c_str(),
                             currentBestReadings.gas_resistance.timestamp);
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.printf("  + Gas Resistance: %.2f kOhm from %s @ %lu\n",
        currentBestReadings.gas_resistance.value,
        currentBestReadings.gas_resistance.sensor_name.c_str(),
        currentBestReadings.gas_resistance.timestamp);
    }
  }

  // Skip if no valid sensor data
  if (encoder.getMeasurementCount() == 0) {
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("MQTT: No valid sensor data, skipping v2 publish");
    }
    return;
  }

  // Encode to buffer (larger buffer for WiFi with per-reading timestamps)
  static uint8_t mqttPayload[512];
  size_t payloadSize = encoder.encode(mqttPayload, sizeof(mqttPayload));

  if (ENABLE_DEBUG_OUTPUT) {
    Serial.printf("MQTT V2: Packed %d sensors into %d bytes (with timestamps)\n",
      encoder.getMeasurementCount(), payloadSize);
  }

  // Publish to v2 topic (no reading_type suffix - all readings consolidated)
  // Topic format: wesense/v2/{country}/{subdivision}/{device_id}
  bool result = client.publish(deviceTopicPrefix, mqttPayload, payloadSize);

  if (result) {
    lastMqttTxTime = millis();  // Update TX indicator
    Serial.printf("MQTT: Published v2 protobuf to %s (%d bytes)\n", deviceTopicPrefix, payloadSize);
  } else {
    Serial.println("MQTT: Failed to publish v2 protobuf message");
  }

#else
  // Fallback: protobuf disabled - log warning
  Serial.println("MQTT: V2 protobuf disabled, no data published");
#endif
}

void publishDiscoveryConfig(const char* objectIdSuffix, const char* name, const char* unit, const char* deviceClass, const char* topicSuffix) {
  if (!wifiConnected || !client.connected()) return;
  String objectId = String(deviceID) + "_" + String(objectIdSuffix);
  String configTopic = "homeassistant/sensor/" + objectId + "/config";
  String stateTopic = String(deviceTopicPrefix) + "/" + topicSuffix;

  String payload = "{";
  payload += "\"name\": \"" + String(name) + "\",";
  payload += "\"stat_t\": \"" + stateTopic + "\","; 
  
  // Add JSON value template to extract 'value' from JSON payload (always enabled)
  payload += "\"val_tpl\": \"{{ value_json.value }}\",";
  
  if (String(unit).length() > 0) {
    payload += "\"unit_of_meas\": \"" + String(unit) + "\","; 
  }
  payload += "\"uniq_id\": \"" + objectId + "\","; 
  payload += "\"dev\": {\"ids\": [\"" + String(deviceID) + "\"], \"name\": \"" + String(deviceID) + " Sensor\", \"mf\": \"WeSense\", \"mdl\": \"Sensor Array v1.0\"},"; 
  if (String(deviceClass).length() > 0) {
    payload += "\"dev_cla\": \"" + String(deviceClass) + "\","; 
  }
  
  // Add state class for measurement sensors (helps Home Assistant with statistics/history)
  // Apply to all sensors with units (measurement sensors)
  if (String(unit).length() > 0) {
    payload += "\"stat_cla\": \"measurement\",";
  }
  
  if (payload.endsWith(",")) payload.remove(payload.length() - 1); 
  payload += "}";

  client.publish(configTopic.c_str(), payload.c_str(), true); 
}

// Special discovery config for status sensors (uses 'status' instead of 'value' from JSON)
void publishStatusDiscoveryConfig(const char* objectIdSuffix, const char* name, const char* topicSuffix) {
  if (!wifiConnected || !client.connected()) return;
  String objectId = String(deviceID) + "_" + String(objectIdSuffix);
  String configTopic = "homeassistant/sensor/" + objectId + "/config";
  String stateTopic = String(deviceTopicPrefix) + "/" + topicSuffix;

  String payload = "{";
  payload += "\"name\": \"" + String(name) + "\",";
  payload += "\"stat_t\": \"" + stateTopic + "\","; 
  
  // Add JSON status template to extract 'status' from JSON payload (always enabled)
  payload += "\"val_tpl\": \"{{ value_json.status }}\",";
  
  payload += "\"uniq_id\": \"" + objectId + "\",";
  payload += "\"dev\": {\"ids\": [\"" + String(deviceID) + "\"], \"name\": \"" + String(deviceID) + " Sensor\", \"mf\": \"WeSense\", \"mdl\": \"Sensor Array v1.0\"},";

  payload += "\"icon\": \"mdi:tune-vertical\"";
  
  payload += "}";

  client.publish(configTopic.c_str(), payload.c_str(), true); 
}

void publishAllDiscoveryConfigs() {
  Serial.println("Publishing Home Assistant discovery configurations...");
  
  // Unified sensor readings - publish generic discovery (sensor model info is in JSON payload)
  // Only publish if at least one sensor of each type is available
  if (!DISABLE_SCD4X && scd4xAvailable) {
    publishDiscoveryConfig("co2", "CO2", "ppm", "carbon_dioxide", "co2");
    publishDiscoveryConfig("temperature", "Temperature", "\u00B0C", "temperature", "temperature");
    publishDiscoveryConfig("humidity", "Humidity", "%", "humidity", "humidity");
  } else if (!DISABLE_SCD30 && scd30Available) {
    publishDiscoveryConfig("co2", "CO2", "ppm", "carbon_dioxide", "co2");
    publishDiscoveryConfig("temperature", "Temperature", "\u00B0C", "temperature", "temperature");
    publishDiscoveryConfig("humidity", "Humidity", "%", "humidity", "humidity");
  } else if (!DISABLE_SHT4X && sht4xAvailable) {
    publishDiscoveryConfig("temperature", "Temperature", "\u00B0C", "temperature", "temperature");
    publishDiscoveryConfig("humidity", "Humidity", "%", "humidity", "humidity");
  } else if (!DISABLE_AHT20 && aht20Available) {
    publishDiscoveryConfig("temperature", "Temperature", "\u00B0C", "temperature", "temperature");
    publishDiscoveryConfig("humidity", "Humidity", "%", "humidity", "humidity");
  } else if (!DISABLE_BME680 && bme680Available) {
    publishDiscoveryConfig("temperature", "Temperature", "\u00B0C", "temperature", "temperature");
    publishDiscoveryConfig("humidity", "Humidity", "%", "humidity", "humidity");
  } else if (!DISABLE_BMP390 && bmp390Available) {
    publishDiscoveryConfig("temperature", "Temperature", "\u00B0C", "temperature", "temperature");
  } else if (!DISABLE_BMP280 && bmp280Available) {
    publishDiscoveryConfig("temperature", "Temperature", "\u00B0C", "temperature", "temperature");
  }

  // Pressure discovery
  if (!DISABLE_BME680 && bme680Available) {
    publishDiscoveryConfig("pressure", "Pressure", "hPa", "pressure", "pressure");
  } else if (!DISABLE_BMP390 && bmp390Available) {
    publishDiscoveryConfig("pressure", "Pressure", "hPa", "pressure", "pressure");
  } else if (!DISABLE_BMP280 && bmp280Available) {
    publishDiscoveryConfig("pressure", "Pressure", "hPa", "pressure", "pressure");
  }

  // SCD4x ASC status - publish if sensor is detected (even if not working)
  if (!DISABLE_SCD4X && sensorLocations.scd4x.found) {
    publishStatusDiscoveryConfig("scd4x_asc_status", "SCD41 Auto Calibration", "scd4x_asc_status");
  }

  // BME680 specific sensors
  if (!DISABLE_BME680 && bme680Available) {
    publishDiscoveryConfig("gas_resistance", "Gas Resistance", "kOhm", "", "gas_resistance");
    publishStatusDiscoveryConfig("bme680_calibration_status", "BME680 Calibration Status", "bme680_calibration_status");
  }

  // SGP41 air quality
  if (!DISABLE_SGP41 && sgp41Available) {
    publishDiscoveryConfig("voc", "VOC Index", "", "", "voc");
    publishDiscoveryConfig("nox", "NOx Index", "", "", "nox");
    publishDiscoveryConfig("voc_raw", "VOC Raw", "", "", "voc_raw");
    publishDiscoveryConfig("nox_raw", "NOx Raw", "", "", "nox_raw");
    publishStatusDiscoveryConfig("sgp41_baseline_status", "SGP41 Baseline Status", "sgp41_baseline_status");
  }

  // GPS
  if (!DISABLE_GPS && gpsAvailable) {
    publishDiscoveryConfig("gps_status", "GPS Fix Status", "", "", "gps_status");
    publishDiscoveryConfig("gps_debug", "GPS Debug", "", "", "gps_debug");
    publishDiscoveryConfig("gps_sats", "GPS Satellites", "", "", "gps_sats");
    publishDiscoveryConfig("gps_lat", "GPS Latitude", "\u00B0", "", "gps_lat");
    publishDiscoveryConfig("gps_lon", "GPS Longitude", "\u00B0", "", "gps_lon");
    publishDiscoveryConfig("gps_alt", "GPS Altitude", "m", "", "gps_alt");
  }

  // Battery
  if (!DISABLE_PMU && pmuAvailable) {
    publishDiscoveryConfig("battery_status", "Battery Status", "", "", "battery_status");
    publishDiscoveryConfig("battery_voltage", "Battery Voltage", "V", "voltage", "battery_voltage");
    publishDiscoveryConfig("battery_percent", "Battery Level", "%", "battery", "battery_percent");
  }
  if (!DISABLE_PMS5003 && pms5003Available) {
    publishDiscoveryConfig("pm1_0", "PMS5003 PM1.0", "\u00B5g/m³", "pm1", "pm1_0");
    publishDiscoveryConfig("pm2_5", "PMS5003 PM2.5", "\u00B5g/m³", "pm25", "pm2_5");
    publishDiscoveryConfig("pm10", "PMS5003 PM10", "\u00B5g/m³", "pm10", "pm10");
    publishDiscoveryConfig("particles_0_3um", "PMS5003 0.3µm Particles", "/0.1L", "", "particles_0_3um");
    publishDiscoveryConfig("particles_0_5um", "PMS5003 0.5µm Particles", "/0.1L", "", "particles_0_5um");
    publishDiscoveryConfig("particles_1_0um", "PMS5003 1.0µm Particles", "/0.1L", "", "particles_1_0um");
    publishDiscoveryConfig("particles_2_5um", "PMS5003 2.5µm Particles", "/0.1L", "", "particles_2_5um");
    publishDiscoveryConfig("particles_5_0um", "PMS5003 5.0µm Particles", "/0.1L", "", "particles_5_0um");
    publishDiscoveryConfig("particles_10um", "PMS5003 10µm Particles", "/0.1L", "", "particles_10um");
    publishStatusDiscoveryConfig("pms5003_calibration_status", "PMS5003 Calibration Status", "pms5003_calibration_status");
  }
  
  // Add calibration system diagnostics (always available if calibration tracking enabled)
  if (ENABLE_CALIBRATION_STATE_TRACKING && ENABLE_CALIBRATION_MONITORING) {
    publishStatusDiscoveryConfig("calibration_summary", "System Calibration Summary", "diagnostic/calibration_summary");
  }
  
  Serial.println("Finished publishing Home Assistant discovery configurations.");
}

// I2C sensor location tracking - moved to global variables section

bool isI2CDeviceAvailable(uint8_t address) {
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  return (error == 0);
}

bool isI2CDeviceAvailable(uint8_t address, TwoWire &wire) {
  wire.beginTransmission(address);
  byte error = wire.endTransmission();
  return (error == 0);
}

// Read chip ID from Bosch sensor to distinguish BMP280 (0x58) from BME680 (0x61) from BMP3xx (0x50/0x60)
// BMP280/BME680 use register 0xD0, BMP3xx uses register 0x00
uint8_t readBoschChipID(uint8_t address, TwoWire &wire) {
  // First try register 0xD0 (BMP280/BME680)
  wire.beginTransmission(address);
  wire.write(0xD0);
  if (wire.endTransmission() == 0) {
    wire.requestFrom(address, (uint8_t)1);
    if (wire.available()) {
      uint8_t chipID = wire.read();
      // Valid BMP280/BME680 chip IDs
      if (chipID == 0x58 || chipID == 0x60 || chipID == 0x61) {
        return chipID;
      }
    }
  }

  // Try register 0x00 (BMP3xx series - BMP380/BMP388/BMP390)
  wire.beginTransmission(address);
  wire.write(0x00);
  if (wire.endTransmission() == 0) {
    wire.requestFrom(address, (uint8_t)1);
    if (wire.available()) {
      uint8_t chipID = wire.read();
      // Valid BMP3xx chip IDs: 0x50 (BMP380/388), 0x60 (BMP390)
      if (chipID == 0x50 || chipID == 0x60) {
        return chipID;
      }
    }
  }

  return 0x00;
}

// Check if device at address is an MS5611 by reading PROM data
// MS5611 uses commands: Reset=0x1E, Read PROM=0xA0-0xAE
bool isMS5611(uint8_t address, TwoWire &wire) {
  // Send reset command
  wire.beginTransmission(address);
  wire.write(0x1E);  // Reset command
  if (wire.endTransmission() != 0) return false;
  delay(3);  // MS5611 needs 2.8ms after reset

  // Try to read PROM word 1 (C1 coefficient) using command 0xA2
  wire.beginTransmission(address);
  wire.write(0xA2);  // Read PROM C1
  if (wire.endTransmission() != 0) return false;

  wire.requestFrom(address, (uint8_t)2);
  if (wire.available() >= 2) {
    uint16_t c1 = (wire.read() << 8) | wire.read();
    // Valid calibration data should not be 0x0000 or 0xFFFF
    if (c1 != 0x0000 && c1 != 0xFFFF) {
      return true;
    }
  }
  return false;
}

// Read INA219 manufacturer ID (should be 0x5449 = "TI" for Texas Instruments)
uint16_t readINA219ManufacturerID(uint8_t address, TwoWire &wire) {
  wire.beginTransmission(address);
  wire.write(0xFE); // Manufacturer ID register
  if (wire.endTransmission() != 0) return 0x0000;
  
  wire.requestFrom(address, (uint8_t)2);
  if (wire.available() >= 2) {
    uint16_t id = (wire.read() << 8) | wire.read();
    return id;
  }
  return 0x0000;
}

#ifdef CONFIG_IDF_TARGET_ESP32C3
// ESP32-C3 specific: Try different I2C pin combinations to find sensors
void scanI2CPinCombinations() {
  if (!ENABLE_DEBUG_OUTPUT) return;
  
  Serial.println("\n=== ESP32-C3 I2C Pin Discovery ===");
  
  // Common ESP32-C3 I2C pin combinations
  struct {
    int sda;
    int scl;
    const char* description;
  } pinCombos[] = {
    {4, 5, "GPIO4(SDA) + GPIO5(SCL) - Most common"},
    {8, 9, "GPIO8(SDA) + GPIO9(SCL) - Alternative"},
    {1, 2, "GPIO1(SDA) + GPIO2(SCL) - Some boards"},
    {6, 7, "GPIO6(SDA) + GPIO7(SCL) - Some boards"},
    {10, 3, "GPIO10(SDA) + GPIO3(SCL) - Some boards"}
  };
  
  for (int combo = 0; combo < 5; combo++) {
    Serial.print("Testing "); Serial.println(pinCombos[combo].description);
    
    // End current I2C and try new pins
    Wire.end();
    delay(100);
    Wire.begin(pinCombos[combo].sda, pinCombos[combo].scl, I2C_FREQUENCY);
    delay(200);
    
    // Scan for BME680 specifically
    bool foundSensor = false;
    uint8_t addresses[] = {0x76, 0x77}; // BME680 common addresses
    
    for (int addr = 0; addr < 2; addr++) {
      Wire.beginTransmission(addresses[addr]);
      if (Wire.endTransmission() == 0) {
        Serial.print("  Found I2C device at 0x"); 
        Serial.print(addresses[addr], HEX);
        Serial.println(" (likely BME680)");
        foundSensor = true;
      }
    }
    
    if (!foundSensor) {
      Serial.println("  No sensors found");
    }
  }
  
  Serial.println("\n=== Pin Discovery Complete ===");
  Serial.println("If a sensor was found, update your board_config.h accordingly.");
  
  // Restore original pins
  Wire.end();
  delay(100);
  Wire.begin(g_boardConfig.sda_pin, g_boardConfig.scl_pin, I2C_FREQUENCY);
  delay(200);
}
#endif

void scanI2CBusForSensors() {
  Serial.println("\n=== I2C Sensor Auto-Detection ===");
  
  #ifdef CONFIG_IDF_TARGET_ESP32C3
    scanI2CPinCombinations();
  #endif
  
  // Initialize secondary I2C bus if board supports dual I2C
  if (HAS_DUAL_I2C()) {
    getI2CBus(1)->begin(g_boardConfig.sda_pin2, g_boardConfig.scl_pin2, I2C_FREQUENCY);
    getI2CBus(1)->setTimeOut(1000); // 1000ms timeout for slow sensors
    Serial.print("Secondary I2C initialised on SDA2="); Serial.print(g_boardConfig.sda_pin2);
    Serial.print(", SCL2="); Serial.print(g_boardConfig.scl_pin2);
    Serial.print(" at "); Serial.print(I2C_FREQUENCY); Serial.println(" Hz");
  }
  
  Serial.println("Scanning both I2C buses for all sensors...");
  delay(100);

  // Full I2C address scan to show ALL devices on both buses
  // This also acts as a "warm-up" for devices that need bus activity before responding
  if (ENABLE_DEBUG_OUTPUT) {
    // Scan primary bus
    Serial.println("Full I2C address scan - PRIMARY bus (0x08-0x77):");
    int deviceCount = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        Serial.print("  Device found at 0x");
        if (addr < 0x10) Serial.print("0");
        Serial.println(addr, HEX);
        deviceCount++;
      }
    }
    if (deviceCount == 0) {
      Serial.println("  No devices found on PRIMARY bus");
    } else {
      Serial.print("  PRIMARY bus total: "); Serial.print(deviceCount); Serial.println(" device(s)");
    }

    // Scan secondary bus if available
    if (HAS_DUAL_I2C()) {
      Serial.println("Full I2C address scan - SECONDARY bus (0x08-0x77):");
      int secondaryDeviceCount = 0;
      TwoWire* secondaryBus = getI2CBus(1);
      for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        secondaryBus->beginTransmission(addr);
        if (secondaryBus->endTransmission() == 0) {
          Serial.print("  Device found at 0x");
          if (addr < 0x10) Serial.print("0");
          Serial.println(addr, HEX);
          secondaryDeviceCount++;
        }
      }
      if (secondaryDeviceCount == 0) {
        Serial.println("  No devices found on SECONDARY bus");
      } else {
        Serial.print("  SECONDARY bus total: "); Serial.print(secondaryDeviceCount); Serial.println(" device(s)");
      }
    }
  }

  // Scan for sensor types - simple list without preference
  struct {
    const char* name;
    uint8_t address;
    SensorLocation* location;
    bool needsValidation; // Requires chip ID validation
  } sensors[] = {
    {"AHT20", 0x38, &sensorLocations.aht20, false},
    {"SCD4x", 0x62, &sensorLocations.scd4x, false},
    {"SCD30", 0x61, &sensorLocations.scd30, false},
    {"CM1106-C", 0x31, &sensorLocations.cm1106c, false},
    {"SGP41", 0x59, &sensorLocations.sgp41, false},
    {"SHT4x", 0x44, &sensorLocations.sht4x, false}, // Can conflict with INA219 at 0x44
    {"TMP117", 0x48, &sensorLocations.tmp117, false},  // High-precision temp sensor (0x48-0x4B)
    {"SSD1306", 0x3C, &sensorLocations.display, false},
    {"SSD1306", 0x3D, &sensorLocations.display, false}
  };
  
  for (int i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++) {
    // Skip if already found
    if (sensors[i].location->found) continue;

    // Check primary bus first - with debug output
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("Checking "); Serial.print(sensors[i].name);
      Serial.print(" at 0x"); Serial.print(sensors[i].address, HEX);
      Serial.print(" on PRIMARY bus: ");
    }

    Wire.beginTransmission(sensors[i].address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      sensors[i].location->found = true;
      sensors[i].location->bus = 0;
      if (ENABLE_DEBUG_OUTPUT) Serial.println("FOUND");
      Serial.print(sensors[i].name); Serial.print(" found on PRIMARY bus at 0x");
      Serial.println(sensors[i].address, HEX);
      // Update boot status with found sensor
      char statusMsg[20];
      snprintf(statusMsg, sizeof(statusMsg), "Found %s", sensors[i].name);
      updateBootStatus(statusMsg);
      continue;
    } else if (ENABLE_DEBUG_OUTPUT) {
      Serial.print("not found (error="); Serial.print(error); Serial.println(")");
    }

    // Check secondary bus if available
    if (HAS_DUAL_I2C()) {
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.print("Checking "); Serial.print(sensors[i].name);
        Serial.print(" at 0x"); Serial.print(sensors[i].address, HEX);
        Serial.print(" on SECONDARY bus: ");
      }

      if (isI2CDeviceAvailable(sensors[i].address, *getI2CBus(1))) {
        sensors[i].location->found = true;
        sensors[i].location->bus = 1;
        if (ENABLE_DEBUG_OUTPUT) Serial.println("FOUND");
        Serial.print(sensors[i].name); Serial.print(" found on SECONDARY bus at 0x");
        Serial.println(sensors[i].address, HEX);
        // Update boot status with found sensor
        char statusMsg[20];
        snprintf(statusMsg, sizeof(statusMsg), "Found %s", sensors[i].name);
        updateBootStatus(statusMsg);
      } else if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("not found");
      }
    }
  }
  
  // Special handling for Bosch sensors (BMP280/BME680) - check chip ID to distinguish
  // These sensors share addresses 0x76 and 0x77, so we need to read chip ID register
  uint8_t boschAddresses[] = {0x77, 0x76};
  
  for (int addrIdx = 0; addrIdx < 2; addrIdx++) {
    uint8_t addr = boschAddresses[addrIdx];

    // Check primary bus
    Serial.print("Checking Bosch address 0x");
    Serial.print(addr, HEX);
    Serial.print(" on PRIMARY bus: ");
    if (isI2CDeviceAvailable(addr)) {
      uint8_t chipID = readBoschChipID(addr, Wire);
      Serial.print("device found, chip ID = 0x");
      Serial.println(chipID, HEX);
      if (chipID == 0x58 && !sensorLocations.bmp280.found) {
        sensorLocations.bmp280.found = true;
        sensorLocations.bmp280.bus = 0;
        Serial.print("BMP280 found on PRIMARY bus at 0x");
        Serial.print(addr, HEX);
        Serial.print(" (Chip ID: 0x");
        Serial.print(chipID, HEX);
        Serial.println(")");
        updateBootStatus("Found BMP280");
      } else if (chipID == 0x61 && !sensorLocations.bme680.found) {
        sensorLocations.bme680.found = true;
        sensorLocations.bme680.bus = 0;
        Serial.print("BME680 found on PRIMARY bus at 0x");
        Serial.print(addr, HEX);
        Serial.print(" (Chip ID: 0x");
        Serial.print(chipID, HEX);
        Serial.println(")");
        updateBootStatus("Found BME680");
      } else if (chipID == 0x50 && !sensorLocations.bmp390.found) {
        // 0x50 = BMP380/BMP388
        sensorLocations.bmp390.found = true;
        sensorLocations.bmp390.bus = 0;
        Serial.print("BMP380/BMP388 found on PRIMARY bus at 0x");
        Serial.print(addr, HEX);
        Serial.print(" (Chip ID: 0x");
        Serial.print(chipID, HEX);
        Serial.println(")");
        updateBootStatus("Found BMP388");
      } else if (chipID == 0x60 && !sensorLocations.bmp390.found) {
        // 0x60 = BMP390
        sensorLocations.bmp390.found = true;
        sensorLocations.bmp390.bus = 0;
        updateBootStatus("Found BMP390");
        Serial.print("BMP390 found on PRIMARY bus at 0x");
        Serial.print(addr, HEX);
        Serial.print(" (Chip ID: 0x");
        Serial.print(chipID, HEX);
        Serial.println(")");
      } else if (!sensorLocations.ms5611.found && isMS5611(addr, Wire)) {
        // Not a Bosch sensor, check if it's an MS5611
        sensorLocations.ms5611.found = true;
        sensorLocations.ms5611.bus = 0;
        Serial.print("MS5611 found on PRIMARY bus at 0x");
        Serial.println(addr, HEX);
        updateBootStatus("Found MS5611");
      } else if (chipID != 0x00 && chipID != 0xFF) {
        // Unknown sensor - print chip ID for debugging
        Serial.print("Unknown sensor on PRIMARY bus at 0x");
        Serial.print(addr, HEX);
        Serial.print(" (Chip ID: 0x");
        Serial.print(chipID, HEX);
        Serial.println(") - please report this!");
      }
    } else {
      Serial.println("no device");
    }

    // Check secondary bus if available
    if (HAS_DUAL_I2C()) {
      Serial.print("Checking Bosch address 0x");
      Serial.print(addr, HEX);
      Serial.print(" on SECONDARY bus: ");
      if (isI2CDeviceAvailable(addr, *getI2CBus(1))) {
        uint8_t chipID = readBoschChipID(addr, *getI2CBus(1));
        Serial.print("device found, chip ID = 0x");
        Serial.println(chipID, HEX);
        if (chipID == 0x58 && !sensorLocations.bmp280.found) {
          sensorLocations.bmp280.found = true;
          sensorLocations.bmp280.bus = 1;
          Serial.print("BMP280 found on SECONDARY bus at 0x");
          Serial.print(addr, HEX);
          Serial.print(" (Chip ID: 0x");
          Serial.print(chipID, HEX);
          Serial.println(")");
          updateBootStatus("Found BMP280");
        } else if (chipID == 0x61 && !sensorLocations.bme680.found) {
          sensorLocations.bme680.found = true;
          sensorLocations.bme680.bus = 1;
          Serial.print("BME680 found on SECONDARY bus at 0x");
          Serial.print(addr, HEX);
          Serial.print(" (Chip ID: 0x");
          Serial.print(chipID, HEX);
          Serial.println(")");
          updateBootStatus("Found BME680");
        } else if (chipID == 0x50 && !sensorLocations.bmp390.found) {
          // 0x50 = BMP380/BMP388
          sensorLocations.bmp390.found = true;
          sensorLocations.bmp390.bus = 1;
          Serial.print("BMP380/BMP388 found on SECONDARY bus at 0x");
          Serial.print(addr, HEX);
          Serial.print(" (Chip ID: 0x");
          Serial.print(chipID, HEX);
          Serial.println(")");
          updateBootStatus("Found BMP388");
        } else if (chipID == 0x60 && !sensorLocations.bmp390.found) {
          // 0x60 = BMP390/BMP390L
          sensorLocations.bmp390.found = true;
          sensorLocations.bmp390.bus = 1;
          Serial.print("BMP390 found on SECONDARY bus at 0x");
          Serial.print(addr, HEX);
          Serial.print(" (Chip ID: 0x");
          Serial.print(chipID, HEX);
          Serial.println(")");
          updateBootStatus("Found BMP390");
        } else if (!sensorLocations.ms5611.found && isMS5611(addr, *getI2CBus(1))) {
          // Not a Bosch sensor, check if it's an MS5611
          sensorLocations.ms5611.found = true;
          sensorLocations.ms5611.bus = 1;
          Serial.print("MS5611 found on SECONDARY bus at 0x");
          Serial.println(addr, HEX);
          updateBootStatus("Found MS5611");
        } else if (chipID != 0x00 && chipID != 0xFF) {
          // Unknown sensor - print chip ID for debugging
          Serial.print("Unknown sensor on SECONDARY bus at 0x");
          Serial.print(addr, HEX);
          Serial.print(" (Chip ID: 0x");
          Serial.print(chipID, HEX);
          Serial.println(") - please report this!");
        }
      } else {
        Serial.println("no device");
      }
    }
  }
  
  // Special handling for INA219 - validate with manufacturer ID (0x5449 = "TI")
  // INA219 can be at 0x40-0x45 via A0/A1 jumpers, potentially conflicting with SHT4x at 0x44
  uint8_t ina219Addresses[] = {0x40, 0x41, 0x44, 0x45};
  
  for (int addrIdx = 0; addrIdx < 4; addrIdx++) {
    uint8_t addr = ina219Addresses[addrIdx];
    
    // Check primary bus
    if (isI2CDeviceAvailable(addr)) {
      uint16_t mfgID = readINA219ManufacturerID(addr, Wire);
      if (mfgID == 0x5449 && !sensorLocations.ina219.found) {
        sensorLocations.ina219.found = true;
        sensorLocations.ina219.bus = 0;
        Serial.print("INA219 found on PRIMARY bus at 0x");
        Serial.print(addr, HEX);
        Serial.print(" (Mfg ID: 0x");
        Serial.print(mfgID, HEX);
        Serial.println(" = TI)");
        updateBootStatus("Found INA219");
      }
    }

    // Check secondary bus if available
    if (HAS_DUAL_I2C() && isI2CDeviceAvailable(addr, *getI2CBus(1))) {
      uint16_t mfgID = readINA219ManufacturerID(addr, *getI2CBus(1));
      if (mfgID == 0x5449 && !sensorLocations.ina219.found) {
        sensorLocations.ina219.found = true;
        sensorLocations.ina219.bus = 1;
        Serial.print("INA219 found on SECONDARY bus at 0x");
        Serial.print(addr, HEX);
        Serial.print(" (Mfg ID: 0x");
        Serial.print(mfgID, HEX);
        Serial.println(" = TI)");
        updateBootStatus("Found INA219");
      }
    }
  }
  
  // Simple informational message
  Serial.print("Primary I2C bus: SDA="); Serial.print(g_boardConfig.sda_pin);
  Serial.print(", SCL="); Serial.println(g_boardConfig.scl_pin);
  
  if (HAS_DUAL_I2C()) {
    Serial.print("Secondary I2C bus: SDA="); Serial.print(g_boardConfig.sda_pin2);
    Serial.print(", SCL="); Serial.println(g_boardConfig.scl_pin2);
    Serial.println("Note: Sensors can be connected to either bus and will be auto-detected");
  }
  
  Serial.println("=== I2C Sensor Detection Complete ===\n");
}

// Get current timestamp as ISO 8601 formatted string with timezone (for global mapping systems)
String getTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "[No Time]";
  }
  
  char timestamp[32];
  strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &timeinfo);
  
  // Add timezone offset (calculate from gmt_offset_sec + daylight_offset_sec)
  int total_offset_sec = gmt_offset_sec + daylight_offset_sec;
  int offset_hours = total_offset_sec / 3600;
  int offset_minutes = abs((total_offset_sec % 3600) / 60);
  
  char timezone_suffix[8];
  if (total_offset_sec == 0) {
    strcpy(timezone_suffix, "Z");  // UTC
  } else {
    snprintf(timezone_suffix, sizeof(timezone_suffix), "%+03d:%02d", offset_hours, offset_minutes);
  }
  
  return String(timestamp) + String(timezone_suffix);
}

// Get current timestamp as UTC ISO 8601 (for global data correlation)
String getTimestampUTC() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "[No Time]";
  }
  
  // getLocalTime() returns local time, but we need to convert it to UTC properly
  // The ESP32 system time is already set with timezone, so we need to get the raw UTC time
  time_t now;
  time(&now);
  struct tm* utc_tm = gmtime(&now);
  
  char timestamp[32];
  strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", utc_tm);
  
  return String(timestamp);
}

// Get timestamp for serial output
String getTimestampSerial() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return "[" + String(millis() / 1000) + "s]";
  }
  char timestamp[16];
  strftime(timestamp, sizeof(timestamp), "[%H:%M:%S]", &timeinfo);
  return String(timestamp);
}

// Sync system time from GPS (for off-grid operation)
bool syncTimeFromGPS() {
  if (!gpsAvailable || !gps.time.isValid() || !gps.date.isValid()) {
    return false;
  }
  
  // Get GPS time data
  struct tm gps_time = {};
  gps_time.tm_year = gps.date.year() - 1900;  // Years since 1900
  gps_time.tm_mon = gps.date.month() - 1;     // 0-11 
  gps_time.tm_mday = gps.date.day();
  gps_time.tm_hour = gps.time.hour();
  gps_time.tm_min = gps.time.minute();
  gps_time.tm_sec = gps.time.second();
  
  // Convert to time_t and set system time
  time_t gps_timestamp = mktime(&gps_time);
  
  // Adjust for timezone (GPS time is UTC)
  gps_timestamp += gmt_offset_sec + daylight_offset_sec;
  
  // Set system time
  struct timeval tv = { gps_timestamp, 0 };
  settimeofday(&tv, NULL);
  
  if (ENABLE_DEBUG_OUTPUT) {
    Serial.print(getTimestampSerial()); 
    Serial.print(" GPS time sync: ");
    Serial.print(gps.date.year()); Serial.print("-");
    Serial.print(gps.date.month()); Serial.print("-");
    Serial.print(gps.date.day()); Serial.print(" ");
    Serial.print(gps.time.hour()); Serial.print(":");
    Serial.print(gps.time.minute()); Serial.print(":");
    Serial.println(gps.time.second());
  }
  
  return true;
}

// ======================
// LED CONTROL FUNCTIONS
// ======================
static unsigned long lastLedToggle = 0;
static bool ledState = false;
static bool fastBlinkMode = true; // Start with fast blink during setup

void initializeLED() {
  if (!DISABLE_STATUS_LED && g_boardConfig.led_pin >= 0) {
    pinMode(g_boardConfig.led_pin, OUTPUT);
    digitalWrite(g_boardConfig.led_pin, LOW); // Start with LED off
    Serial.print("Status LED initialized on GPIO "); Serial.println(g_boardConfig.led_pin);
  } else if (DISABLE_STATUS_LED) {
    Serial.println("Status LED disabled by configuration.");
  } else {
    Serial.println("No status LED pin configured for this board.");
  }
}

void updateStatusLED() {
  if (DISABLE_STATUS_LED || g_boardConfig.led_pin < 0) return;
  
  unsigned long blinkInterval = fastBlinkMode ? LED_FAST_BLINK_INTERVAL_MS : LED_SLOW_BLINK_INTERVAL_MS;
  
  if (millis() - lastLedToggle >= blinkInterval) {
    ledState = !ledState;
    digitalWrite(g_boardConfig.led_pin, ledState);
    lastLedToggle = millis();
  }
}

void setLEDMode(bool fastMode) {
  fastBlinkMode = fastMode;
}

void turnOffLED() {
  if (!DISABLE_STATUS_LED && g_boardConfig.led_pin >= 0) {
    digitalWrite(g_boardConfig.led_pin, LOW);
    ledState = false;
  }
}

// ======================
// SETUP FUNCTION
// ======================
void setup() {
  // Initialize Serial with timeout to prevent hanging on standalone power
  Serial.begin(115200);
  
  #ifdef CONFIG_IDF_TARGET_ESP32C3
    // ESP32-C3: Uses native USB-CDC, wait briefly but don't hang
    delay(2000); // Longer delay for native USB-CDC initialization
    // Note: ESP32-C3 USB-CDC doesn't support setTxTimeoutMs()
  #elif defined(CONFIG_IDF_TARGET_ESP32C6)
    // ESP32-C6: Don't wait for Serial connection in standalone mode
    delay(1000); // Brief delay for Serial initialization
    // Note: ESP32-C6 USB-Serial/JTAG doesn't support setTxTimeoutMs()
  #else
    // Regular ESP32: Wait for Serial with timeout to prevent hanging on standalone power
    unsigned long serialTimeout = millis() + 3000; // 3 second timeout
    while (!Serial && millis() < serialTimeout) { 
      delay(10);
    }
    delay(1000); // Additional delay for Serial stability
  #endif
  
  delay(500);
  
  #ifdef CONFIG_IDF_TARGET_ESP32C3
    // ESP32-C3 specific: Force USB-CDC to be ready
    Serial.flush();
    delay(100);
    // Try to establish communication
    if (ENABLE_DEBUG_OUTPUT) {
      for (int i = 0; i < 5; i++) {
        Serial.print("ESP32-C3 USB-CDC Test #"); Serial.println(i + 1);
        Serial.flush();
        delay(500);
      }
    }
  #endif

  // === DEEP SLEEP WAKE DETECTION ===
  // Must happen early before any other initialization
  rtc_bootCount++;  // Increment on every boot (cold or warm)

  bool wakeFromDeepSleep = isWakeFromDeepSleep();

  if (wakeFromDeepSleep) {
    Serial.println(F("\n========== WAKE FROM DEEP SLEEP =========="));
    Serial.print(F("Wake reason: ")); Serial.println(getWakeReasonString());
    Serial.print(F("Boot count: ")); Serial.println(rtc_bootCount);
    Serial.print(F("Wake cycle: ")); Serial.println(rtc_wakeCount);
    Serial.print(F("LoRaWAN session valid: ")); Serial.println(rtc_lorawanSessionValid ? "Yes" : "No");
    Serial.println(F("==========================================\n"));
  } else {
    // Cold boot - reset RTC variables
    Serial.println(F("\n========== COLD BOOT =========="));
    Serial.print(F("Wake reason: ")); Serial.println(getWakeReasonString());
    rtc_wakeCount = 0;
    rtc_deepSleepActive = false;
    rtc_lorawanSessionValid = false;
    rtc_lastTxTimestamp = 0;

    // Clear any stale session data on cold boot
    if (ENABLE_DEEP_SLEEP) {
      clearLoRaWANSession();
    }
    Serial.println(F("===============================\n"));
  }

  Serial.println(F("Booting up Multi-Sensor Node..."));

  // Auto-detect board and get configuration
  g_boardConfig = getBoardConfig();
  Serial.print("Detected board: "); 
  Serial.println(g_boardConfig.name);
  Serial.print("LoRa support: "); Serial.println(HAS_LORA() ? "Yes" : "No");
  Serial.print("PMU support: "); Serial.println(HAS_PMU() ? "Yes" : "No");
  Serial.print("GPS support: "); Serial.println(HAS_GPS() ? "Yes" : "No");
  Serial.print("Dual I2C support: "); Serial.println(HAS_DUAL_I2C() ? "Yes" : "No");
  
  // LoRaWAN Configuration Debug Information
  if (HAS_LORA()) {
    Serial.println("\n=== LORAWAN CONFIGURATION DETECTED ===");
    
    // Show disable status first if disabled
    if (DISABLE_LORAWAN) {
      Serial.println("STATUS: DISABLED BY CONFIGURATION (DISABLE_LORAWAN = true)");
      Serial.println("Action: Set DISABLE_LORAWAN = false to enable (may cause crashes)");
      Serial.println("========================================\n");
      // Skip the rest of the LoRaWAN debug info, but continue with setup
    } else {
    
    // Board and radio detection
    Serial.print("Board Type: "); Serial.println(g_boardConfig.name);
    
    #ifdef CONFIG_IDF_TARGET_ESP32S3
      Serial.println("ESP32-S3 Detected -> SX1262 Radio Configuration");
    #elif defined(CONFIG_IDF_TARGET_ESP32C3)
      Serial.println("ESP32-C3 Detected -> SX1276 Radio Configuration (Generic)");
    #elif defined(CONFIG_IDF_TARGET_ESP32C6)
      Serial.println("ESP32-C6 Detected -> SX1276 Radio Configuration (Generic)");
    #else
      Serial.println("ESP32 Classic Detected -> SX1276 Radio Configuration");
    #endif
    
    // Region auto-detection information
    Serial.print("Country Code: "); Serial.println(COUNTRY_CODE);
    Serial.print("Subdivision: "); Serial.println(SUBDIVISION_CODE);

    #if AUTO_DETECT_REGION
      Serial.println("Region Detection: AUTO (based on country code)");
    #else
      Serial.println("Region Detection: MANUAL (user-specified)");
    #endif
    
    Serial.print("LoRaWAN Region: "); Serial.println(LORAWAN_REGION_NAME);
    
    // Frequency band information
    #if defined(LORAWAN_REGION_AU915)
      Serial.println("Frequency Band: 915-928 MHz (AU915 - Australia/New Zealand)");
    #elif defined(LORAWAN_REGION_EU868)
      Serial.println("Frequency Band: 863-870 MHz (EU868 - Europe)");
    #elif defined(LORAWAN_REGION_US915)
      Serial.println("Frequency Band: 902-928 MHz (US915 - United States/Canada)");
    #elif defined(LORAWAN_REGION_AS923)
      Serial.println("Frequency Band: 915-928 MHz (AS923 - Asia Pacific)");
    #elif defined(LORAWAN_REGION_IN865)
      Serial.println("Frequency Band: 865-867 MHz (IN865 - India)");
    #elif defined(LORAWAN_REGION_KR920)
      Serial.println("Frequency Band: 920-923 MHz (KR920 - South Korea)");
    #elif defined(LORAWAN_REGION_CN470)
      Serial.println("Frequency Band: 470-510 MHz (CN470 - China)");
    #endif
    
    // Pin configuration
    Serial.println("LoRa Pin Configuration:");
    Serial.print("   NSS (CS):   GPIO"); Serial.println(g_boardConfig.lora_nss);
    Serial.print("   RST:       GPIO"); Serial.println(g_boardConfig.lora_rst);
    if (g_boardConfig.lora_dio0 >= 0) {
      Serial.print("   DIO0:      GPIO"); Serial.println(g_boardConfig.lora_dio0);
    }
    if (g_boardConfig.lora_dio1 >= 0) {
      Serial.print("   DIO1:      GPIO"); Serial.println(g_boardConfig.lora_dio1);
    }
    if (g_boardConfig.lora_dio2 >= 0) {
      Serial.print("   DIO2:      GPIO"); Serial.println(g_boardConfig.lora_dio2);
    }
    Serial.print("   SPI SCK:   GPIO"); Serial.println(g_boardConfig.lora_sck);
    Serial.print("   SPI MISO:  GPIO"); Serial.println(g_boardConfig.lora_miso);
    Serial.print("   SPI MOSI:  GPIO"); Serial.println(g_boardConfig.lora_mosi);
    
    // Power and capabilities
    if (g_boardConfig.type == BOARD_TBEAM_T3_S3_V12) {
      Serial.println("Radio Power: 22dBm (158mW) - SX1262 High Power");
      Serial.println("Radio Features: Long Range, Low Power, Built-in Antenna Switch");
    } else if (g_boardConfig.type == BOARD_TBEAM_V10 || g_boardConfig.type == BOARD_TBEAM_V11) {
      Serial.println("Radio Power: ~20dBm (100mW) - SX1276 Standard");
      Serial.println("Radio Features: Standard Range, Legacy Compatible");
    } else {
      Serial.println("Radio Power: Variable (depends on external module)");
    }
    
    // TTN credentials status
    Serial.println("The Things Network (TTN) Credentials:");
    Serial.print("   DevEUI: ");
    for (int i = 7; i >= 0; i--) {
      if (USER_DEVEUI[i] < 0x10) Serial.print("0");
      Serial.print(USER_DEVEUI[i], HEX);
    }
    Serial.println();
    
    // Check if using default/placeholder credentials
    bool usingDefaultCredentials = true;
    for (int i = 0; i < 8; i++) {
      if (USER_APPEUI[i] != 0x00 || USER_DEVEUI[i] != 0x9C) {
        usingDefaultCredentials = false;
        break;
      }
    }
    
    if (usingDefaultCredentials) {
      Serial.println("   Status: WARNING - USING PLACEHOLDER CREDENTIALS");
      Serial.println("   Action: Update USER_APPEUI, USER_DEVEUI, USER_APPKEY for TTN");
    } else {
      Serial.println("   Status: Custom credentials configured");
    }
    
    Serial.println("LoRaWAN Status: Ready for initialization");
    Serial.println("========================================\n");
    } // End of enabled LoRaWAN configuration
  } else {
    Serial.println("\nLoRaWAN: Disabled (no LoRa hardware detected)\n");
  }
  
  #ifdef CONFIG_IDF_TARGET_ESP32C3
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("ESP32-C3 Features:");
      Serial.println("  - Native USB-CDC Serial");
      Serial.println("  - Single I2C Controller");
      Serial.println("  - RISC-V Architecture");
    }
  #endif
  
  Serial.println();
  
  // ESP32-C6 Beetle boot issue resolved by USB CDC configuration
  // No additional delays needed
  
  Serial.println("\n=== DEVICE CONFIGURATION ===");
  
  // Initialize status LED (starts in fast blink mode during setup)
  initializeLED();

  generateDeviceID();
  Serial.print("Device ID: "); Serial.println(deviceID);
  
  // Initialize location configuration (loads MQTT overrides or uses firmware defaults)
  initializeLocationConfig();
  Serial.println("==============================\n");
  
  // Memory and storage analysis for different board types
  Serial.println("\n=== MEMORY & STORAGE ANALYSIS ===");
  Serial.println("NOTE: ESP32 memory functions may report inaccurate free space.");
  Serial.println("      Use Arduino IDE compilation output for accurate firmware space.");
  Serial.print("Board: "); Serial.println(g_boardConfig.name);
  
  // Flash storage information
  uint32_t sketchSize = ESP.getSketchSize();
  uint32_t freeSketchSpace = ESP.getFreeSketchSpace();
  
  Serial.print("Sketch size: "); Serial.print(sketchSize); Serial.println(" bytes");
  Serial.print("Free sketch space: "); Serial.print(freeSketchSpace); Serial.println(" bytes");
  
  // On some ESP32 variants, getFreeSketchSpace() returns max partition size instead of free space
  // Let's calculate more accurately based on ESP32 partition schemes
  uint32_t maxSketchSpace = sketchSize + freeSketchSpace;
  uint32_t actualFreeSpace = freeSketchSpace;
  
  // If getFreeSketchSpace seems too large, it's probably reporting max partition size
  if (freeSketchSpace > sketchSize * 2) {
    // Assume standard partition: sketch space is typically 1.3MB on ESP32-C6
    actualFreeSpace = maxSketchSpace - sketchSize;
    Serial.print("Corrected free space: "); Serial.print(actualFreeSpace); Serial.println(" bytes");
  }
  
  Serial.print("Max sketch space: "); Serial.print(maxSketchSpace); Serial.println(" bytes");
  
  float usagePercent = (float)sketchSize / maxSketchSpace * 100;
  Serial.print("Flash usage: "); Serial.print(usagePercent, 1); Serial.println("%");
  Serial.print("Arduino IDE reports: "); Serial.print(sketchSize); Serial.print("/"); Serial.print(maxSketchSpace); 
  Serial.print(" ("); Serial.print(usagePercent, 0); Serial.println("%) - This matches Arduino IDE");
  
  // RAM information  
  Serial.print("Free heap: "); Serial.print(ESP.getFreeHeap()); Serial.println(" bytes");
  Serial.print("Heap size: "); Serial.print(ESP.getHeapSize()); Serial.println(" bytes");
  Serial.print("Min free heap: "); Serial.print(ESP.getMinFreeHeap()); Serial.println(" bytes");
  
  // PSRAM information (if available)
  if (ESP.getPsramSize() > 0) {
    Serial.print("PSRAM size: "); Serial.print(ESP.getPsramSize()); Serial.println(" bytes");
    Serial.print("Free PSRAM: "); Serial.print(ESP.getFreePsram()); Serial.println(" bytes");
  } else {
    Serial.println("PSRAM: Not available");
  }
  
  // Flash filesystem information
  Serial.print("Flash chip size: "); Serial.print(ESP.getFlashChipSize()); Serial.println(" bytes");
  Serial.print("Flash chip speed: "); Serial.print(ESP.getFlashChipSpeed() / 1000000); Serial.println(" MHz");
  
  // Board-specific capabilities
  Serial.println("Board capabilities:");
  Serial.print("  LoRa: "); Serial.println(HAS_LORA() ? "Yes" : "No");
  Serial.print("  PMU: "); Serial.println(HAS_PMU() ? "Yes" : "No");
  Serial.print("  GPS: "); Serial.println(HAS_GPS() ? "Yes" : "No");
  
  // Report available storage space
  Serial.print("Reported available space: "); Serial.print(actualFreeSpace); Serial.println(" bytes");
  Serial.println("(Note: May include non-firmware partitions - see Arduino IDE for firmware space)");
  Serial.println("==============================\n");

  Serial.println("\n=== I2C & SENSOR DETECTION ===");
  
  // Initialize I2C with board-specific pins
  // Use slower I2C speed for better reliability with external power
  Wire.begin(g_boardConfig.sda_pin, g_boardConfig.scl_pin, I2C_FREQUENCY);
  Wire.setTimeOut(500); // 500ms timeout for sensor compatibility (default is 50ms)
  Serial.print("Primary I2C initialized on SDA="); Serial.print(g_boardConfig.sda_pin);
  Serial.print(", SCL="); Serial.print(g_boardConfig.scl_pin);
  Serial.print(" at "); Serial.print(I2C_FREQUENCY); Serial.println(" Hz");

  // Initialize PMU early for T-Beam boards - needed to power the display
  Serial.println("Initializing PMU (AXP2101)...");
  if (!DISABLE_PMU && HAS_PMU()) {
    if (PMU.begin(Wire, g_boardConfig.pmu_address, g_boardConfig.sda_pin, g_boardConfig.scl_pin)) {
      pmuAvailable = true;
      PMU.enableBattVoltageMeasure();
      Serial.println("  AXP2101 PMU initialized successfully");
    } else {
      Serial.println("  AXP2101 PMU init failed");
    }
  } else if (!HAS_PMU()) {
    Serial.println("  Board has no PMU - skipping");
  } else {
    Serial.println("  PMU disabled by configuration");
  }

  // Initialize display early so boot status shows from the start
  initDisplayEarly();
  
  #ifdef CONFIG_IDF_TARGET_ESP32C3
    if (ENABLE_DEBUG_OUTPUT) {
      Serial.println("ESP32-C3 I2C Pin Notes:");
      Serial.println("  Common pin combinations:");
      Serial.println("    GPIO4(SDA) + GPIO5(SCL) - Most common");
      Serial.println("    GPIO8(SDA) + GPIO9(SCL) - Alternative");
      Serial.println("    GPIO1(SDA) + GPIO2(SCL) - Some boards");
      Serial.println("    GPIO6(SDA) + GPIO7(SCL) - Some boards");
      Serial.println("  Check your board's pinout diagram!");
    }
  #endif
  
  // Update LED during setup
  updateStatusLED();

  // Scan all I2C buses for sensors
  updateBootStatus("Scanning I2C...");
  scanI2CBusForSensors();

  updateBootStatus("Init sensors...");
  Serial.println("\n--- Sensor Initialization ---");
  
  Serial.println("Initializing AHT20...");
  if (!DISABLE_AHT20 && sensorLocations.aht20.found) {
    // If AHT20 is on secondary bus and board supports dual I2C, temporarily switch to it
    if (sensorLocations.aht20.bus == 1 && HAS_DUAL_I2C()) {
      Serial.println("AHT20 detected on secondary I2C bus - switching to secondary bus...");
      Wire.end();
      delay(100);
      Wire.begin(g_boardConfig.sda_pin2, g_boardConfig.scl_pin2, 100000);
      delay(100);
    }

    // AHT20 library uses Wire internally, so we pass &Wire after switching pins
    if (aht20.begin(&Wire)) {
      Serial.print("AHT20 sensor initialized on ");
      Serial.print(sensorLocations.aht20.bus == 0 ? "PRIMARY" : "SECONDARY");
      Serial.print(" (pins "); Serial.print(sensorLocations.aht20.bus == 0 ? g_boardConfig.sda_pin : g_boardConfig.sda_pin2);
      Serial.print("/"); Serial.print(sensorLocations.aht20.bus == 0 ? g_boardConfig.scl_pin : g_boardConfig.scl_pin2);
      Serial.print(")");
      Serial.println(" I2C bus.");
      aht20Available = true;
    } else {
      Serial.println("Failed to initialize AHT20.");
    }

    // Switch back to primary bus for other sensors
    if (sensorLocations.aht20.bus == 1 && HAS_DUAL_I2C()) {
      Wire.end();
      delay(100);
      Wire.begin(g_boardConfig.sda_pin, g_boardConfig.scl_pin, 100000);
      delay(100);
      Serial.println("Switched I2C back to primary bus for remaining sensors.");
    }
  } else if (!DISABLE_AHT20) {
    Serial.println("AHT20 not detected during bus scan.");
  } else {
    Serial.println("AHT20 sensor force-disabled.");
  }

  Serial.println("Initializing SCD4x...");
  if (!DISABLE_SCD4X && sensorLocations.scd4x.found) {
    Serial.println("SCD4x: Initializing sensor...");
    delay(1000); // Standard delay for sensor stabilization
    
    TwoWire* scd4xWire = getI2CBus(sensorLocations.scd4x.bus);
    
    Serial.println("SCD4x: Beginning sensor initialization...");
    scd4x.begin(*scd4xWire);
    
    // Test basic I2C communication first
    Serial.println("SCD4x: Testing basic I2C communication...");
    scd4xWire->beginTransmission(SCD4X_I2C_ADDRESS);
    byte i2c_error = scd4xWire->endTransmission();
    if (i2c_error == 0) {
      Serial.println("SCD4x: I2C communication test PASSED");
    } else {
      Serial.print("SCD4x: I2C communication test FAILED with error: ");
      Serial.println(i2c_error);
    }
    
    Serial.println("SCD4x: Stopping any existing periodic measurement...");
    if (!scd4x.stopPeriodicMeasurement()) {
      Serial.println("SCD4x: Note - stopPeriodicMeasurement returned false (sensor may already be stopped)");
    } else {
      Serial.println("SCD4x: Successfully stopped periodic measurement");
    }
    
    delay(500); // Standard delay before starting measurements
    
    Serial.println("SCD4x: Starting periodic measurement...");
    if (!scd4x.startPeriodicMeasurement()) {
      Serial.println("SCD4x: Failed to start periodic measurement");
      Serial.println("SCD4x: Possible causes:");
      Serial.println("  - Sensor not responding to I2C commands");
      Serial.println("  - Sensor still in initialization phase");
      Serial.println("  - Power supply issues affecting sensor");
      
      // Try to recover with a reset
      Serial.println("SCD4x: Attempting sensor reset and retry...");
      delay(1000);
      
      // Reinitialize
      scd4x.begin(*scd4xWire);
      delay(1000);
      
      // Try starting measurement again
      if (!scd4x.startPeriodicMeasurement()) {
        Serial.println("SCD4x: Retry failed - sensor initialization failed - will be disabled");
      } else {
        Serial.println("SCD4x: Retry successful - sensor initialized");
        scd4xAvailable = true;
      }
    } else {
      Serial.print("SCD4x: Started periodic measurement successfully on ");
      Serial.print(sensorLocations.scd4x.bus == 0 ? "PRIMARY" : "SECONDARY");
      Serial.println(" I2C bus.");
      scd4xAvailable = true;
    }
    
    // Final status check
    if (scd4xAvailable) {
      Serial.println("SCD4x: Initialization complete - sensor ready");
      
      // Initialize calibration method tracking
      initializeFactoryCalibration("scd4x");
      
      // Handle Automatic Self-Calibration (ASC) configuration
      if (ENABLE_SCD4X_ASC_BY_DEFAULT) {
        Serial.println("SCD4x: Enabling Automatic Self-Calibration (ASC) by default...");
        Serial.println("SCD4x: ASC is appropriate for this deployment type (rural/suburban with fresh air access)");
        
        // Stop measurements to change ASC setting
        scd4x.stopPeriodicMeasurement();
        delay(500);
        
        // Enable ASC
        if (!scd4x.setAutomaticSelfCalibrationEnabled(true)) {
          Serial.println("SCD4x: WARNING - Failed to enable ASC by default");
          Serial.println("SCD4x: ASC can still be enabled later via MQTT command");
          Serial.println("SCD4x: Calibration tracking will NOT be started without confirmed ASC");
        } else {
          Serial.println("SCD4x: ASC (Automatic Self-Calibration) command sent successfully");
          
          // Verify ASC was actually enabled by reading it back
          delay(200);
          bool ascEnabled = scd4x.getAutomaticSelfCalibrationEnabled();
          
          if (ascEnabled) {
            Serial.println("SCD4x: ASC verification PASSED - Automatic Self-Calibration is ENABLED");
            Serial.println("SCD4x: Sensor will self-calibrate over 7+ days when exposed to fresh air");
            
            // Update calibration method to ASC learning mode
            setCalibrationMethod("scd4x", "asc_learning", "low", "7_day_automatic");
            
            // Calibration period will be started after loading saved state
          } else {
            Serial.println("SCD4x: ASC verification FAILED - ASC is NOT enabled despite successful command");
            Serial.println("SCD4x: This may indicate sensor firmware issues or timing problems");
            Serial.println("SCD4x: Calibration tracking will NOT be started without confirmed ASC");
          }
        }
        
        // Restart measurements
        delay(500);
        if (!scd4x.startPeriodicMeasurement()) {
          Serial.println("SCD4x: Warning - Failed to restart measurement after ASC setup");
        }
      } else {
        Serial.println("SCD4x: ASC (Automatic Self-Calibration) DISABLED by configuration");
        Serial.println("SCD4x: This is correct for urban/indoor deployments without access to ~400ppm fresh air");
        Serial.println("SCD4x: Use manual calibration with reference gas or bush walk method for accuracy");
        Serial.println("SCD4x: Sensor will use factory calibration until manually calibrated via MQTT");
      }
    } else {
      Serial.println("SCD4x: Initialization failed - sensor will be disabled");
    }
    
  } else if (!DISABLE_SCD4X) {
    Serial.println("SCD4x not detected during bus scan.");
  } else {
    Serial.println("SCD4x sensor force-disabled.");
  }
  scd4xAvailable = scd4xAvailable && !DISABLE_SCD4X;

  Serial.println("Initializing SCD30...");
  if (!DISABLE_SCD30 && sensorLocations.scd30.found) {
    Serial.println("SCD30: Initializing sensor...");
    delay(1000); // Standard delay for sensor stabilization

    TwoWire* scd30Wire = getI2CBus(sensorLocations.scd30.bus);

    Serial.println("SCD30: Beginning sensor initialization...");
    if (scd30.begin(*scd30Wire)) {
      Serial.print("SCD30: Sensor initialized successfully on ");
      Serial.print(sensorLocations.scd30.bus == 0 ? "PRIMARY" : "SECONDARY");
      Serial.println(" I2C bus.");
      scd30Available = true;

      // Set measurement interval (2-1800 seconds, default 2)
      scd30.setMeasurementInterval(2);
      Serial.println("SCD30: Measurement interval set to 2 seconds.");

      // Configure automatic self-calibration (ASC) based on deployment
      // SCD30 calls it ASC, same concept as SCD4x
      // Disabled by default for urban deployments
      scd30.setAutoSelfCalibration(false);
      Serial.println("SCD30: Automatic Self-Calibration (ASC) disabled by default.");
      Serial.println("SCD30: Use MQTT commands to enable ASC or perform forced recalibration.");

      // Initialize calibration method tracking
      initializeFactoryCalibration("scd30");

      // Test read to verify sensor is responding
      delay(2500); // Wait for first measurement
      if (scd30.dataAvailable()) {
        float testCO2 = scd30.getCO2();
        float testTemp = scd30.getTemperature();
        float testHum = scd30.getHumidity();
        Serial.println("SCD30: Initial test reading successful:");
        Serial.print("  CO2: "); Serial.print(testCO2, 0); Serial.println(" ppm");
        Serial.print("  Temp: "); Serial.print(testTemp, 2); Serial.println(" °C");
        Serial.print("  Humidity: "); Serial.print(testHum, 2); Serial.println(" %");
      } else {
        Serial.println("SCD30: Note - Initial data not yet available (normal during warmup).");
      }

      Serial.println("SCD30: Initialization complete - sensor ready.");
    } else {
      Serial.println("SCD30: Failed to initialize sensor.");
      Serial.println("SCD30: Possible causes:");
      Serial.println("  - I2C communication error");
      Serial.println("  - Sensor not powered correctly");
      Serial.println("  - Wrong I2C address");
    }
  } else if (!DISABLE_SCD30) {
    Serial.println("SCD30 not detected during bus scan.");
  } else {
    Serial.println("SCD30 sensor force-disabled.");
  }
  scd30Available = scd30Available && !DISABLE_SCD30;

  Serial.println("Initialising CM1106-C...");
  if (!DISABLE_CM1106C && sensorLocations.cm1106c.found) {
    Serial.println("CM1106-C: Initialising sensor...");
    Serial.println("CM1106-C: Waiting for sensor warmup (30 seconds minimum required)...");
    Serial.println("CM1106-C: This is normal - NDIR CO2 sensors need infrared source stabilisation");
    
    // CM1106-C requires minimum 30 seconds warmup before first reading
    // Show progress to user
    for (int i = 1; i <= 30; i++) {
      Serial.print("CM1106-C: Warmup progress: ");
      Serial.print(i);
      Serial.println("/30 seconds");
      delay(1000);
    }
    Serial.println("CM1106-C: Warmup complete - attempting initialisation...");
    
    TwoWire* cm1106cWire = getI2CBus(sensorLocations.cm1106c.bus);
    
    Serial.println("CM1106-C: Beginning sensor initialisation...");
    cm1106c.begin(*cm1106cWire);
    
    // Library calls Wire.begin() which resets our timeout - reconfigure it
    cm1106cWire->setTimeOut(1000);
    Serial.println("CM1106-C: I2C timeout reconfigured to 1000ms after library init");
    
    // Test basic I2C communication
    Serial.println("CM1106-C: Testing basic I2C communication...");
    cm1106cWire->beginTransmission(CM1106C_I2C_ADDRESS);
    byte i2c_error = cm1106cWire->endTransmission();
    if (i2c_error == 0) {
      Serial.println("CM1106-C: I2C communication test PASSED");
    } else {
      Serial.print("CM1106-C: I2C communication test FAILED with error: ");
      Serial.println(i2c_error);
    }
    
    // Test sensor reading
    // CM1106-C needs extra time due to slow internal MCU (per datasheet page 12)
    delay(2000); // 2 second delay for slow MCU
    Serial.println("CM1106-C: Testing sensor reading (this may take a few seconds)...");
    
    // Try reading with error handling
    uint8_t result = 255; // Initialize to error state
    int retry_count = 0;
    const int max_retries = 3;
    
    while (retry_count < max_retries && result != 0) {
      if (retry_count > 0) {
        Serial.print("CM1106-C: Retry attempt ");
        Serial.print(retry_count);
        Serial.println("/3...");
        delay(1000);
      }
      
      // Manual I²C read on first attempt to see what sensor is actually sending
      if (retry_count == 0) {
        Serial.println("CM1106-C: DEBUG - Manual I²C transaction to inspect raw response...");
        cm1106cWire->beginTransmission(CM1106C_I2C_ADDRESS);
        cm1106cWire->write(0x01); // Measure result command
        byte write_result = cm1106cWire->endTransmission();
        Serial.print("CM1106-C: Write result: "); Serial.println(write_result);
        
        delay(500);
        
        int bytesReceived = cm1106cWire->requestFrom(CM1106C_I2C_ADDRESS, (uint8_t)5);
        Serial.print("CM1106-C: Requested 5 bytes, received: ");
        Serial.println(bytesReceived);
        
        if (bytesReceived > 0) {
          Serial.print("CM1106-C: Raw response: ");
          uint8_t rawBuffer[5];
          int idx = 0;
          while (cm1106cWire->available() && idx < 5) {
            rawBuffer[idx] = cm1106cWire->read();
            Serial.print("0x");
            if (rawBuffer[idx] < 16) Serial.print("0");
            Serial.print(rawBuffer[idx], HEX);
            Serial.print(" ");
            idx++;
          }
          Serial.println();
          if (idx > 0) {
            Serial.print("CM1106-C: First byte (should be 0x01): 0x");
            Serial.println(rawBuffer[0], HEX);
          }
        } else {
          Serial.println("CM1106-C: No bytes received from sensor!");
        }
        delay(1000);
      }
      
      result = cm1106c.measure_result();
      retry_count++;
    }
    
    if (result == 0 && cm1106c.co2 > 0) {
      Serial.print("CM1106-C: Initial read successful on ");
      Serial.print(sensorLocations.cm1106c.bus == 0 ? "PRIMARY" : "SECONDARY");
      Serial.println(" I2C bus.");
      Serial.print("  CO2: "); Serial.print(cm1106c.co2); Serial.println(" ppm");
      Serial.print("  Status: 0x"); Serial.println(cm1106c.status, HEX);
      cm1106cAvailable = true;
    } else {
      Serial.print("CM1106-C: All read attempts failed with error code: ");
      Serial.println(result);
      Serial.println("CM1106-C: Sensor may need more warm-up time or have communication issues");
    }
    
    // Final status check
    if (cm1106cAvailable) {
      Serial.println("CM1106-C: Initialisation complete - sensor ready");
      
      // Initialise calibration method tracking
      initializeFactoryCalibration("cm1106c");
      
      // Handle Automatic Baseline Correction (ABC) configuration
      if (ENABLE_CM1106C_ABC_BY_DEFAULT) {
        Serial.println("CM1106-C: Enabling Automatic Baseline Correction (ABC) by default...");
        Serial.println("CM1106-C: ABC is appropriate for this deployment type (rural/suburban with fresh air access)");
        
        // Enable ABC with 15-day cycle at 400ppm baseline (library defaults)
        uint8_t abc_result = cm1106c.auto_zero_setting(0, 15, 400);
        
        if (abc_result == 0) {
          Serial.println("CM1106-C: ABC (Automatic Baseline Correction) enabled successfully");
          Serial.println("CM1106-C: Sensor will self-calibrate over 15+ days when exposed to fresh air");
          
          // Update calibration method to ABC learning mode
          setCalibrationMethod("cm1106c", "abc_learning", "low", "15_day_automatic");
          
          // Calibration period will be started after loading saved state
        } else {
          Serial.print("CM1106-C: WARNING - Failed to enable ABC, error code: ");
          Serial.println(abc_result);
          Serial.println("CM1106-C: ABC can still be enabled later via MQTT command");
        }
      } else {
        Serial.println("CM1106-C: ABC (Automatic Baseline Correction) DISABLED by configuration");
        Serial.println("CM1106-C: This is correct for urban/city deployments without access to ~400ppm fresh air");
        Serial.println("CM1106-C: Disabling ABC now to rely on factory calibration...");
        
        // Explicitly disable ABC (set to close)
        uint8_t abc_result = cm1106c.auto_zero_setting(2, 15, 400);
        
        if (abc_result == 0) {
          Serial.println("CM1106-C: ABC disabled successfully - sensor will use factory calibration");
          Serial.println("CM1106-C: Use manual calibration with reference gas if needed for accuracy");
        } else {
          Serial.print("CM1106-C: WARNING - Failed to disable ABC, error code: ");
          Serial.println(abc_result);
          Serial.println("CM1106-C: Sensor may still have ABC enabled from previous configuration");
        }
      }
    } else {
      Serial.println("CM1106-C: Initialisation failed - sensor will be disabled");
    }
    
  } else if (!DISABLE_CM1106C) {
    Serial.println("CM1106-C not detected during bus scan.");
  } else {
    Serial.println("CM1106-C sensor force-disabled.");
  }
  cm1106cAvailable = cm1106cAvailable && !DISABLE_CM1106C;

  Serial.println("Initializing SGP41...");
  if (!DISABLE_SGP41 && sensorLocations.sgp41.found) {
    TwoWire* sgp41Wire = getI2CBus(sensorLocations.sgp41.bus);
    sgp41.begin(*sgp41Wire);
    Serial.print("SGP41 sensor initialized on ");
    Serial.print(sensorLocations.sgp41.bus == 0 ? "PRIMARY" : "SECONDARY");
    Serial.println(" I2C bus.");
    sgp41Available = true;
    
    // Initialize calibration method tracking
    initializeFactoryCalibration("sgp41");
    
    // Calibration period will be started after loading saved state
  } else if (!DISABLE_SGP41) {
    Serial.println("SGP41 not detected during bus scan.");
  } else {
    Serial.println("SGP41 sensor force-disabled.");
  }
  sgp41Available = sgp41Available && !DISABLE_SGP41;

  Serial.println("Initializing SHT4x...");
  if (!DISABLE_SHT4X && sensorLocations.sht4x.found) {
    TwoWire* sht4xWire = getI2CBus(sensorLocations.sht4x.bus);
    sht4x.begin(*sht4xWire, 0x44);
    delay(10); 
    float temp_test, hum_test;
    uint16_t error = sht4x.measureHighPrecision(temp_test, hum_test);
    if (error == 0) {
      Serial.print("SHT4x sensor initialized and test read successful on ");
      Serial.print(sensorLocations.sht4x.bus == 0 ? "PRIMARY" : "SECONDARY");
      Serial.println(" I2C bus.");
      sht4xAvailable = true;

      // Query serial number and attempt to identify SHT40/41/45 variant
      // Based on empirical testing: SHT41 has upper word 0x151D, SHT40 has 0x0Dxx-0x0Exx range
      // This is NOT officially documented by Sensirion - treat as best-effort detection
      uint32_t sht4xSerial = 0;
      int16_t serialError = sht4x.serialNumber(sht4xSerial);
      if (serialError == 0) {
        uint16_t upperWord = (sht4xSerial >> 16) & 0xFFFF;

        // Attempt auto-detection based on empirical serial number patterns
        String autoDetected = "SHT4x";  // Unknown/fallback
        if (upperWord == 0x151D) {
          autoDetected = "SHT41";
        } else if (upperWord >= 0x0D00 && upperWord <= 0x0FFF) {
          autoDetected = "SHT40";
        } else if (upperWord >= 0x1800 && upperWord <= 0x1FFF) {
          autoDetected = "SHT45";  // Speculative - no test data yet
        }

        // Use configured variant, but warn if it disagrees with auto-detection
        sht4xDetectedVariant = String(SHT4X_VARIANT);

        if (ENABLE_DEBUG_OUTPUT) {
          Serial.printf("  Serial: %lu (0x%08lX), upper=0x%04X\n", sht4xSerial, sht4xSerial, upperWord);
          Serial.printf("  Auto-detected: %s, Configured: %s\n", autoDetected.c_str(), SHT4X_VARIANT);
        }

        if (autoDetected != "SHT4x" && autoDetected != String(SHT4X_VARIANT)) {
          Serial.printf("  WARNING: Auto-detected %s but configured as %s - using configured value\n",
                        autoDetected.c_str(), SHT4X_VARIANT);
          Serial.println("  If this is wrong, update SHT4X_VARIANT in config. SHT40 has known drift issues.");
        } else if (autoDetected == "SHT4x") {
          Serial.printf("  Note: Could not auto-detect variant (unknown serial pattern), using configured: %s\n", SHT4X_VARIANT);
        } else {
          Serial.printf("  Variant confirmed: %s\n", sht4xDetectedVariant.c_str());
        }
      } else {
        Serial.printf("  Could not read serial number (error: %d), using configured variant: %s\n", serialError, SHT4X_VARIANT);
        sht4xDetectedVariant = String(SHT4X_VARIANT);
      }

      // Initialize calibration method tracking
      initializeFactoryCalibration("sht4x");
    } else {
      Serial.print("SHT4x initialization or test read failed. Error: "); Serial.println(error);
    }
  } else if (!DISABLE_SHT4X) {
    Serial.println("SHT4x not detected during bus scan.");
  } else {
    Serial.println("SHT4x sensor force-disabled.");
  }
  sht4xAvailable = sht4xAvailable && !DISABLE_SHT4X;

  Serial.println("Initializing TMP117...");
  if (sensorLocations.tmp117.found) {
    TwoWire* tmp117Wire = getI2CBus(sensorLocations.tmp117.bus);
    if (tmp117.begin(0x48, *tmp117Wire)) {
      Serial.print("TMP117 high-precision temperature sensor initialized on ");
      Serial.print(sensorLocations.tmp117.bus == 0 ? "PRIMARY" : "SECONDARY");
      Serial.println(" I2C bus.");
      tmp117Available = true;

      // Configure for maximum averaging (64 samples) for smoothest readings
      // Conversion takes ~1 second but gives best noise reduction
      // For environmental monitoring with 5-min intervals, this is ideal
      // Averaging modes: 0=none, 1=8 samples, 2=32 samples, 3=64 samples
      tmp117.setConversionAverageMode(3);  // 64 averaged conversions
      Serial.println("  Configured for 64-sample averaging (1s conversion)");

      // Test read
      float testTemp = tmp117.readTempC();
      if (!isnan(testTemp)) {
        Serial.print("  Initial temperature test: ");
        Serial.print(testTemp, 2);
        Serial.println(" °C");
      } else {
        Serial.println("  Initial temperature test failed (NaN)");
      }
    } else {
      Serial.println("TMP117 initialization failed.");
    }
  } else if (!DISABLE_TMP117) {
    Serial.println("TMP117 not detected during bus scan.");
  }
  tmp117Available = tmp117Available && !DISABLE_TMP117;

  Serial.println("Initializing BMP280...");
  if (!DISABLE_BMP280 && sensorLocations.bmp280.found) {
    // Handle BMP280 initialization on the detected bus
    bool foundBMP = false;
    
    // If BMP280 is on secondary bus and board supports dual I2C, temporarily switch default I2C to that bus
    if (sensorLocations.bmp280.bus == 1 && HAS_DUAL_I2C()) {
      Serial.println("BMP280 detected on secondary I2C bus - configuring for isolation mode...");
      // Temporarily end the primary I2C and start with secondary pins
      Wire.end();
      delay(100);
      Wire.begin(g_boardConfig.sda_pin2, g_boardConfig.scl_pin2, 100000);
      delay(100);
    } else if (sensorLocations.bmp280.bus == 1 && !HAS_DUAL_I2C()) {
      Serial.println("BMP280: Ignoring bus assignment - single I2C controller board");
      sensorLocations.bmp280.bus = 0; // Force to primary bus
    }
    
    if (isI2CDeviceAvailable(0x77)) {
        if (bmp280.begin(0x77)) { 
            Serial.print("BMP280 sensor initialized at 0x77 on ");
            Serial.print(sensorLocations.bmp280.bus == 0 ? "PRIMARY" : "SECONDARY");
            Serial.print(" (pins "); Serial.print(sensorLocations.bmp280.bus == 0 ? g_boardConfig.sda_pin : g_boardConfig.sda_pin2);
            Serial.print("/"); Serial.print(sensorLocations.bmp280.bus == 0 ? g_boardConfig.scl_pin : g_boardConfig.scl_pin2);
            Serial.print(")");
            Serial.println(" bus.");
            foundBMP = true;
        }
    }
    if (!foundBMP && isI2CDeviceAvailable(0x76)) {
         if (bmp280.begin(0x76)) {
            Serial.print("BMP280 sensor initialized at 0x76 on ");
            Serial.print(sensorLocations.bmp280.bus == 0 ? "PRIMARY" : "SECONDARY");
            Serial.print(" (pins "); Serial.print(sensorLocations.bmp280.bus == 0 ? g_boardConfig.sda_pin : g_boardConfig.sda_pin2);
            Serial.print("/"); Serial.print(sensorLocations.bmp280.bus == 0 ? g_boardConfig.scl_pin : g_boardConfig.scl_pin2);
            Serial.print(")");
            Serial.println(" bus.");
            foundBMP = true;
        }
    }
    
    if (foundBMP) {
        bmp280Available = true;

        // Perform soft reset to clear any stuck states
        delay(100);

        // Use FORCED mode to ensure fresh readings each time
        // In NORMAL mode, the sensor can return cached values causing stuck readings
        bmp280.setSampling(Adafruit_BMP280::MODE_FORCED,
                        Adafruit_BMP280::SAMPLING_X2,
                        Adafruit_BMP280::SAMPLING_X16,
                        Adafruit_BMP280::FILTER_X4,    // Reduced filter for faster response
                        Adafruit_BMP280::STANDBY_MS_1);

        // Test read to verify sensor is working properly
        // Do this BEFORE switching buses - sensor is still on secondary if that's where it was detected
        delay(500);
        if (bmp280.takeForcedMeasurement()) {
            float test_pressure = bmp280.readPressure();
            if (!isnan(test_pressure) && test_pressure > 0) {
                Serial.println("BMP280 configured and responding correctly.");
                if (ENABLE_DEBUG_OUTPUT) {
                    Serial.print("  Initial pressure test: "); Serial.print(test_pressure/100.0, 2); Serial.println(" hPa");
                }
            } else {
                Serial.println("BMP280 configured but initial test read returned invalid data.");
            }
        } else {
            Serial.println("BMP280 configured but forced measurement failed.");
        }
    } else {
        Serial.println("BMP280 .begin() failed.");
    }

    // Switch back to primary bus AFTER test read is complete
    if (sensorLocations.bmp280.bus == 1 && HAS_DUAL_I2C()) {
      Wire.end();
      delay(100);
      Wire.begin(g_boardConfig.sda_pin, g_boardConfig.scl_pin, 100000);
      delay(100);
      Serial.println("Switched I2C back to primary bus for remaining sensors.");
    }
  } else if (!DISABLE_BMP280) {
    Serial.println("BMP280 not detected during bus scan.");
  } else {
    Serial.println("BMP280 sensor force-disabled.");
  }
  bmp280Available = bmp280Available && !DISABLE_BMP280;

  Serial.println("Initializing BMP3xx (BMP380/BMP388/BMP390/BMP300L)...");
  if (!DISABLE_BMP390 && sensorLocations.bmp390.found) {
    bool foundBMP390 = false;

    // If BMP3xx is on secondary bus and board supports dual I2C, temporarily switch
    if (sensorLocations.bmp390.bus == 1 && HAS_DUAL_I2C()) {
      Serial.println("BMP3xx detected on secondary I2C bus - configuring for isolation mode...");
      Wire.end();
      delay(100);
      Wire.begin(g_boardConfig.sda_pin2, g_boardConfig.scl_pin2, 100000);
      delay(100);
    } else if (sensorLocations.bmp390.bus == 1 && !HAS_DUAL_I2C()) {
      Serial.println("BMP3xx: Ignoring bus assignment - single I2C controller board");
      sensorLocations.bmp390.bus = 0;
    }

    if (isI2CDeviceAvailable(0x77)) {
        if (bmp390.begin_I2C(0x77)) {
            Serial.print("BMP3xx sensor initialized at 0x77 on ");
            Serial.print(sensorLocations.bmp390.bus == 0 ? "PRIMARY" : "SECONDARY");
            Serial.println(" bus.");
            foundBMP390 = true;
        }
    }
    if (!foundBMP390 && isI2CDeviceAvailable(0x76)) {
         if (bmp390.begin_I2C(0x76)) {
            Serial.print("BMP3xx sensor initialized at 0x76 on ");
            Serial.print(sensorLocations.bmp390.bus == 0 ? "PRIMARY" : "SECONDARY");
            Serial.println(" bus.");
            foundBMP390 = true;
        }
    }

    if (foundBMP390) {
        bmp390Available = true;

        // Configure oversampling and filter for high accuracy
        bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp390.setPressureOversampling(BMP3_OVERSAMPLING_16X);
        bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp390.setOutputDataRate(BMP3_ODR_50_HZ);

        // Test read to verify sensor is working
        // Do this BEFORE switching buses - sensor is still on secondary if that's where it was detected
        delay(500);
        if (bmp390.performReading()) {
            Serial.println("BMP3xx configured and responding correctly.");
            if (ENABLE_DEBUG_OUTPUT) {
                Serial.print("  Initial pressure test: "); Serial.print(bmp390.pressure / 100.0, 2); Serial.println(" hPa");
            }
        } else {
            Serial.println("BMP390 configured but initial test read failed.");
        }
    } else {
        Serial.println("BMP390 .begin_I2C() failed.");
    }

    // Switch back to primary bus AFTER test read is complete
    if (sensorLocations.bmp390.bus == 1 && HAS_DUAL_I2C()) {
      Wire.end();
      delay(100);
      Wire.begin(g_boardConfig.sda_pin, g_boardConfig.scl_pin, 100000);
      delay(100);
      Serial.println("Switched I2C back to primary bus for remaining sensors.");
    }
  } else if (!DISABLE_BMP390) {
    Serial.println("BMP390 not detected during bus scan.");
  } else {
    Serial.println("BMP390 sensor force-disabled.");
  }
  bmp390Available = bmp390Available && !DISABLE_BMP390;

  // MS5611 initialization
  Serial.println("Initializing MS5611...");
  if (!DISABLE_MS5611 && sensorLocations.ms5611.found) {
    bool foundMS5611 = false;

    // If MS5611 is on secondary bus, switch to it
    if (sensorLocations.ms5611.bus == 1 && HAS_DUAL_I2C()) {
      Serial.println("MS5611 detected on secondary I2C bus - switching to secondary bus...");
      Wire.end();
      delay(100);
      Wire.begin(g_boardConfig.sda_pin2, g_boardConfig.scl_pin2, 100000);
      delay(100);
    } else if (sensorLocations.ms5611.bus == 1 && !HAS_DUAL_I2C()) {
      Serial.println("MS5611: Ignoring bus assignment - single I2C controller board");
      sensorLocations.ms5611.bus = 0;
    }

    // Try both addresses (0x77 default, 0x76 alternate)
    if (isI2CDeviceAvailable(0x77)) {
      ms5611 = MS5611(0x77);
      if (ms5611.begin()) {
        Serial.print("MS5611 sensor initialized at 0x77 on ");
        Serial.print(sensorLocations.ms5611.bus == 0 ? "PRIMARY" : "SECONDARY");
        Serial.println(" bus.");
        foundMS5611 = true;
      }
    }
    if (!foundMS5611 && isI2CDeviceAvailable(0x76)) {
      ms5611 = MS5611(0x76);
      if (ms5611.begin()) {
        Serial.print("MS5611 sensor initialized at 0x76 on ");
        Serial.print(sensorLocations.ms5611.bus == 0 ? "PRIMARY" : "SECONDARY");
        Serial.println(" bus.");
        foundMS5611 = true;
      }
    }

    if (foundMS5611) {
      ms5611Available = true;
      // Set oversampling for high resolution (OSR_ULTRA_HIGH = 4096)
      ms5611.setOversampling(OSR_ULTRA_HIGH);
      Serial.println("MS5611 configured with ultra-high oversampling.");

      // Print PROM calibration coefficients for diagnostics
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("MS5611 PROM calibration coefficients:");
        for (int i = 0; i < 8; i++) {
          Serial.printf("  C[%d] = %u (0x%04X)\n", i, ms5611.getProm(i), ms5611.getProm(i));
        }
      }

      // Test read to verify sensor is working
      delay(100);
      int result = ms5611.read();
      if (result == MS5611_READ_OK) {
        float testPressure = ms5611.getPressure();
        if (ENABLE_DEBUG_OUTPUT) {
          Serial.println("MS5611 responding correctly.");
          Serial.print("  Initial pressure (mathMode=0): ");
          Serial.print(testPressure, 2);
          Serial.println(" hPa");
        }

        // Auto-detect MS5607 or clones that need factor 2 fix
        // If pressure is unreasonably low (< 700 hPa = higher than Everest), try mathMode=1
        if (testPressure < 700.0) {
          if (ENABLE_DEBUG_OUTPUT) {
            Serial.println("  Pressure too low - trying MS5607/clone mathMode (factor 2 fix)...");
          }
          ms5611.reset(1);  // Apply factor 2 fix for MS5607 or compatible clones
          ms5611.setOversampling(OSR_ULTRA_HIGH);
          delay(100);
          result = ms5611.read();
          if (result == MS5611_READ_OK) {
            testPressure = ms5611.getPressure();
            if (ENABLE_DEBUG_OUTPUT) {
              Serial.print("  Pressure with mathMode=1: ");
              Serial.print(testPressure, 2);
              Serial.println(" hPa");
            }
            if (testPressure >= 700.0 && testPressure <= 1100.0) {
              Serial.println("  MS5611 using MS5607-compatible math (factor 2 fix applied).");
            } else {
              Serial.println("  WARNING: MS5611 pressure out of range, sensor may be faulty.");
            }
          }
        }

        if (ENABLE_DEBUG_OUTPUT) {
          Serial.print("  Temperature: ");
          Serial.print(ms5611.getTemperature(), 2);
          Serial.println(" °C");
        }
      } else {
        Serial.print("MS5611 initial test read failed, error: ");
        Serial.println(result);
      }
    } else {
      Serial.println("MS5611 .begin() failed.");
    }

    // Switch back to primary bus AFTER initialization
    if (sensorLocations.ms5611.bus == 1 && HAS_DUAL_I2C()) {
      Wire.end();
      delay(100);
      Wire.begin(g_boardConfig.sda_pin, g_boardConfig.scl_pin, 100000);
      delay(100);
      Serial.println("Switched I2C back to primary bus for remaining sensors.");
    }
  } else if (!DISABLE_MS5611) {
    Serial.println("MS5611 not detected during bus scan.");
  } else {
    Serial.println("MS5611 sensor force-disabled.");
  }
  ms5611Available = ms5611Available && !DISABLE_MS5611;

  Serial.println("Initializing BME680...");
  if (!DISABLE_BME680 && sensorLocations.bme680.found) {
    bool foundBME = false;
    
    // If BME680 is on secondary bus, switch to it
    if (sensorLocations.bme680.bus == 1 && HAS_DUAL_I2C()) {
      Serial.println("BME680 detected on secondary I2C bus - switching to secondary bus...");
      Wire.end();
      delay(100);
      Wire.begin(g_boardConfig.sda_pin2, g_boardConfig.scl_pin2, 100000);
      delay(100);
    } else if (sensorLocations.bme680.bus == 1 && !HAS_DUAL_I2C()) {
      Serial.println("BME680: Ignoring bus assignment - single I2C controller board");
      sensorLocations.bme680.bus = 0; // Force to primary bus
    }
    
    if (isI2CDeviceAvailable(0x77)) {
        if (bme680.begin(0x77)) {
            Serial.print("BME680 sensor initialised at 0x77 on ");
            Serial.print(sensorLocations.bme680.bus == 0 ? "PRIMARY" : "SECONDARY");
            Serial.println(" bus.");
            foundBME = true;
        }
    }
    if (!foundBME && isI2CDeviceAvailable(0x76)) {
         if (bme680.begin(0x76)) {
            Serial.print("BME680 sensor initialised at 0x76 on ");
            Serial.print(sensorLocations.bme680.bus == 0 ? "PRIMARY" : "SECONDARY");
            Serial.println(" bus.");
            foundBME = true;
        }
    }
    
    // If we switched to secondary bus, switch back to primary for other sensors
    if (sensorLocations.bme680.bus == 1 && HAS_DUAL_I2C()) {
      Wire.end();
      delay(100);
      Wire.begin(g_boardConfig.sda_pin, g_boardConfig.scl_pin, 100000);
      delay(100);
      Serial.println("Switched I2C back to primary bus for remaining sensors.");
    }
    if (foundBME) {
        bme680Available = true;
        
        // Set up oversampling and filter initialization
        bme680.setTemperatureOversampling(BME680_OS_8X);
        bme680.setHumidityOversampling(BME680_OS_2X);
        bme680.setPressureOversampling(BME680_OS_4X);
        bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
        bme680.setGasHeater(320, 150); // 320°C for 150 ms
        
        Serial.println("BME680 configured with gas sensor enabled.");
        
        // Calibration period will be started after loading saved state
        
        // Test read to verify sensor is working properly  
        if (ENABLE_DEBUG_OUTPUT) {
            delay(1000); // BME680 needs more time for first reading
            if (bme680.performReading()) {
                Serial.println("BME680 initial test reading successful:");
                Serial.print("  Test pressure: "); Serial.print(bme680.pressure/100.0, 2); Serial.println(" hPa");
                Serial.print("  Test temperature: "); Serial.print(bme680.temperature, 2); Serial.println(" °C");
            } else {
                Serial.println("BME680 configured but initial test read failed.");
            }
        }
    } else {
        Serial.println("BME680 .begin() failed.");
    }
  } else if (!DISABLE_BME680) {
    Serial.println("BME680 not detected during bus scan.");
  } else {
    Serial.println("BME680 sensor force-disabled.");
  }
  bme680Available = bme680Available && !DISABLE_BME680;

  Serial.println("Initializing INA219...");
  if (!DISABLE_INA219 && sensorLocations.ina219.found) {
    TwoWire* ina219Wire = getI2CBus(sensorLocations.ina219.bus);
    
    // Initialize INA219 with custom calibration
    if (ina219.begin(ina219Wire)) {
      // Try different calibration ranges to find the best one for your setup
      Serial.println("INA219: Testing calibration ranges to find optimal settings...");
      
      // Start with higher precision, lower current range for battery monitoring
      ina219.setCalibration_32V_1A(); // Higher precision for low current measurements
      
      Serial.print("INA219 DC Current Monitor initialized on ");
      Serial.print(sensorLocations.ina219.bus == 0 ? "PRIMARY" : "SECONDARY");
      Serial.print(" I2C bus at address 0x");
      Serial.println(INA219_I2C_ADDRESS, HEX);
      
      // Test reading to verify sensor is working and diagnose calibration
      delay(500); // Give more time for first reading
      
      float testBusVoltage = ina219.getBusVoltage_V();
      float testShuntVoltage = ina219.getShuntVoltage_mV();
      float testCurrent = ina219.getCurrent_mA();
      
      Serial.println("INA219: Initial calibration test (32V_1A range):");
      Serial.print("  Bus Voltage: "); Serial.print(testBusVoltage, 3); Serial.println(" V");
      Serial.print("  Shunt Voltage: "); Serial.print(testShuntVoltage, 3); Serial.println(" mV");
      Serial.print("  Current: "); Serial.print(testCurrent, 1); Serial.println(" mA");
      
      // Check if we're hitting calibration limits (indicating wrong range)
      bool hitLimits = (abs(testShuntVoltage) > 310.0) || (abs(testCurrent) > 990.0);
      
      if (hitLimits) {
        Serial.println("INA219: Measurements hitting limits - switching to higher current range...");
        ina219.setCalibration_32V_2A(); // Switch to higher current range
        
        delay(200);
        testBusVoltage = ina219.getBusVoltage_V();
        testShuntVoltage = ina219.getShuntVoltage_mV();
        testCurrent = ina219.getCurrent_mA();
        
        Serial.println("INA219: Recalibration test (32V_2A range):");
        Serial.print("  Bus Voltage: "); Serial.print(testBusVoltage, 3); Serial.println(" V");
        Serial.print("  Shunt Voltage: "); Serial.print(testShuntVoltage, 3); Serial.println(" mV");
        Serial.print("  Current: "); Serial.print(testCurrent, 1); Serial.println(" mA");
        
        // Check if still hitting limits
        bool stillHitLimits = (abs(testShuntVoltage) > 310.0) || (abs(testCurrent) > 1990.0);
        
        if (stillHitLimits) {
          Serial.println(" INA219: Still hitting limits - sensor may be damaged");
          Serial.println(" This often indicates hardware damage from initial overcurrent.");
          ina219.setCalibration_32V_1A(); // Use moderate range as fallback
        } else {
          Serial.println("INA219: Calibration successful with 32V_2A range");
        }
      } else {
        Serial.println("INA219: Calibration successful with 32V_1A range (higher precision)");
      }
      
      // Additional diagnostic: check if the sensor is actually reading or stuck
      Serial.println("INA219: Additional diagnostics:");
      
      // Try multiple readings to see if values change
      for (int i = 0; i < 3; i++) {
        delay(100);
        float busV = ina219.getBusVoltage_V();
        float shuntV = ina219.getShuntVoltage_mV();
        float currentV = ina219.getCurrent_mA();
        Serial.print("  Reading "); Serial.print(i+1); Serial.print(": Bus=");
        Serial.print(busV, 3); Serial.print("V, Shunt="); Serial.print(shuntV, 3);
        Serial.print("mV, Current="); Serial.print(currentV, 1); Serial.println("mA");
      }
      
      // RAW REGISTER DEBUGGING - bypass library calculations
      Serial.println("INA219: Raw register values (bypassing library):");
      TwoWire* wire = getI2CBus(sensorLocations.ina219.bus);
      
      // Read raw registers directly
      wire->beginTransmission(INA219_I2C_ADDRESS);
      wire->write(0x00); // Configuration register
      wire->endTransmission();
      wire->requestFrom(INA219_I2C_ADDRESS, 2);
      uint16_t configReg = 0;
      if (wire->available() >= 2) {
        configReg = (wire->read() << 8) | wire->read();
      }
      
      wire->beginTransmission(INA219_I2C_ADDRESS);
      wire->write(0x01); // Shunt voltage register
      wire->endTransmission();
      wire->requestFrom(INA219_I2C_ADDRESS, 2);
      uint16_t shuntReg = 0;
      if (wire->available() >= 2) {
        shuntReg = (wire->read() << 8) | wire->read();
      }
      
      wire->beginTransmission(INA219_I2C_ADDRESS);
      wire->write(0x02); // Bus voltage register
      wire->endTransmission();
      wire->requestFrom(INA219_I2C_ADDRESS, 2);
      uint16_t busReg = 0;
      if (wire->available() >= 2) {
        busReg = (wire->read() << 8) | wire->read();
      }
      
      Serial.print("  Config Register: 0x"); Serial.println(configReg, HEX);
      Serial.print("  Shunt Register: 0x"); Serial.print(shuntReg, HEX);
      Serial.print(" (decimal: "); Serial.print(shuntReg); Serial.println(")");
      Serial.print("  Bus Register: 0x"); Serial.print(busReg, HEX);
      Serial.print(" (decimal: "); Serial.print(busReg); Serial.println(")");
      
      // Manual calculation of bus voltage (should be busReg >> 3) * 4mV
      float rawBusVoltage = (busReg >> 3) * 0.004; // 4mV per LSB
      Serial.print("  Raw Bus Voltage Calculation: "); Serial.print(rawBusVoltage, 3); Serial.println(" V");
      
      // Manual calculation of shunt voltage (10μV per LSB)
      int16_t signedShunt = (int16_t)shuntReg;
      float rawShuntVoltage = signedShunt * 0.01; // 10μV per LSB = 0.01mV
      Serial.print("  Raw Shunt Voltage Calculation: "); Serial.print(rawShuntVoltage, 3); Serial.println(" mV");
      
      if (busReg == 0 && shuntReg == 0x7FFF) {
        Serial.println(" Hardware issue: Bus=0, Shunt=max - possible damage");
      } else if (busReg > 0 && rawBusVoltage != testBusVoltage) {
        Serial.println(" Library issue: Raw calculation differs from library");
      } else if (busReg > 0) {
        Serial.println("Hardware working: Raw registers show valid data");
      }
      
      // Check if readings are identical (stuck sensor) or realistic for a battery
      if (testBusVoltage >= 10.0 && testBusVoltage <= 15.0) {
        Serial.println("INA219: Bus voltage is in expected range for 12V battery");
      } else if (testBusVoltage < 0.1) {
        Serial.println(" INA219: Bus voltage near zero - check battery connection to VIN+/VIN-");
      } else {
        Serial.print(" INA219: Unusual bus voltage ("); Serial.print(testBusVoltage, 3);
        Serial.println("V) - verify connections");
      }
      
      if (!isnan(testBusVoltage)) {
        Serial.println("INA219 configured and responding correctly.");
        ina219Available = true;
      } else {
        Serial.println("INA219 configured but initial test read failed.");
      }
    } else {
      Serial.println("INA219 .begin() failed.");
    }
  } else if (!DISABLE_INA219) {
    Serial.println("INA219 not detected during bus scan.");
  } else {
    Serial.println("INA219 sensor force-disabled.");
  }
  ina219Available = ina219Available && !DISABLE_INA219;

  // Display initialization moved to initDisplayEarly() for boot status visibility

  // Initialize boot button for display control
  Serial.println("Initializing Boot Button...");
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  int interruptNum = digitalPinToInterrupt(BOOT_BUTTON_PIN);
  Serial.print("  GPIO"); Serial.print(BOOT_BUTTON_PIN);
  Serial.print(" -> Interrupt #"); Serial.println(interruptNum);
  if (interruptNum != NOT_AN_INTERRUPT) {
    attachInterrupt(interruptNum, bootButtonISR, FALLING);
    Serial.println("  Interrupt attached successfully");
  } else {
    Serial.println("  WARNING: GPIO does not support interrupts! Using polling fallback.");
  }
  Serial.print("  Initial button state: ");
  Serial.println(digitalRead(BOOT_BUTTON_PIN) == HIGH ? "HIGH (not pressed)" : "LOW (pressed)");
  Serial.print("Boot button (GPIO"); Serial.print(BOOT_BUTTON_PIN);
  Serial.println(") configured - press to turn display back on");

  Serial.println("Gas Index Algorithms initialized.");
  Serial.println("=============================\n");
  
  GasIndexAlgorithm_init(&voc_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
  GasIndexAlgorithm_init(&nox_params, GasIndexAlgorithm_ALGORITHM_TYPE_NOX);

  Serial.println("\n=== ADDITIONAL HARDWARE INITIALIZATION ===");

  updateBootStatus("Init GPS...");
  Serial.println("Initializing GPS...");
  if (!DISABLE_GPS && HAS_GPS()) {
    GPSSerial.begin(9600, SERIAL_8N1, g_boardConfig.gps_rx, g_boardConfig.gps_tx);
    Serial.println("GPSSerial (UART1 for GPS) started at 9600 baud.");
    gpsAvailable = true; 
  } else if (!HAS_GPS()) {
    Serial.println("Board has no GPS - skipping GPS initialization.");
  } else {
    Serial.println("GPS is DISABLED by configuration.");
  }
  gpsAvailable = gpsAvailable && !DISABLE_GPS && HAS_GPS();

  #if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
  Serial.println("Initializing UART1 sensors (PMS5003/CM1106-C)...");
    #ifdef CONFIG_IDF_TARGET_ESP32C3
    Serial.println("Note: ESP32-C3 uses UART1 (not UART2) - only one sensor type can be connected at a time");
    #else
    Serial.println("Note: ESP32-C6 uses UART1 (not UART2) - only one sensor type can be connected at a time");
    #endif
  #else
  Serial.println("Initializing UART2 sensors (PMS5003/CM1106-C)...");
  Serial.println("Note: UART2 is shared - only one sensor type can be connected at a time");
  #endif

  if (!DISABLE_PMS5003 || !DISABLE_C8_CO2) {
    // Start the UART (shared between PMS5003 and CM1106-C CO₂)
    PMS5003Serial.begin(9600, SERIAL_8N1, g_boardConfig.pms5003_rx, g_boardConfig.pms5003_tx);
    #if defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6)
    Serial.print("UART1 initialized on pins RX:");
    #else
    Serial.print("UART2 initialized on pins RX:");
    #endif
    Serial.print(g_boardConfig.pms5003_rx);
    Serial.print(", TX:");
    Serial.println(g_boardConfig.pms5003_tx);
    
    // Brief startup time
    delay(1000);
    
    // Clear any initial data in the buffer
    while (PMS5003Serial.available()) {
      PMS5003Serial.read();
    }
    
    // Auto-detect which sensor is connected
    updateBootStatus("Detect UART...");
    Serial.println("Auto-detecting UART2 sensor type...");
    
    // Try PMS5003 first (if not disabled)
    if (!DISABLE_PMS5003) {
      unsigned long testStart = millis();
      while (millis() - testStart < 3000) { // 3 second test window
        if (readPMS5003Data(pms5003Data)) {
          pms5003Available = true;
          Serial.println("PMS5003 sensor detected and configured.");
          Serial.println("PMS5003: Sensor warming up... (readings may be unstable for first 30 seconds)");
          Serial.print("  Initial reading - PM2.5: "); 
          Serial.print(pms5003Data.pm2_5_atmospheric); 
          Serial.println(" μg/m³");
          break;
        }
        delay(500);
      }
    }
    
    // If PMS5003 not found, try CM1106-C CO₂ (if not disabled)
    if (!pms5003Available && !DISABLE_C8_CO2) {
      Serial.println("PMS5003 not found, checking for CM1106-C UART sensor...");
      if (detectC8CO2()) {
        c8co2Available = true;
        Serial.println("CM1106-C UART sensor detected and configured.");
        Serial.print("  Initial reading: ");
        Serial.print(c8co2Data.co2_ppm_alt);  // Bytes 6-7 contain actual CO2
        Serial.println(" ppm");
        
        // Initialize calibration tracking
        initializeFactoryCalibration("c8co2");
      }
    }
    
    // Report results
    if (!pms5003Available && !c8co2Available) {
      Serial.println("No UART2 sensor detected.");
      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("  Expected connections:");
        Serial.println("  PMS5003: TX -> GPIO" + String(g_boardConfig.pms5003_rx) + ", RX -> GPIO" + String(g_boardConfig.pms5003_tx) + ", VCC -> 5V");
        Serial.println("  CM1106-C: CON5 Pin 2 (RX) -> GPIO" + String(g_boardConfig.pms5003_tx) + ", Pin 3 (TX) -> GPIO" + String(g_boardConfig.pms5003_rx) + ", CON4 Pin 6 (+5V) -> 5V");
      }
    }
  } else {
    Serial.println("UART2 sensors DISABLED by configuration.");
  }
  
  pms5003Available = pms5003Available && !DISABLE_PMS5003;
  c8co2Available = c8co2Available && !DISABLE_C8_CO2;
  Serial.println("==========================================\n");

  Serial.println("\n=== NETWORK INITIALIZATION ===");
  setup_wifi();
  if (wifiConnected) {
    // Initialize secure telnet monitor immediately after WiFi connection
    if (ENABLE_SECURE_TELNET) {
      telnetMonitor.begin(TELNET_PASSWORD, ALLOWED_TELNET_IP);
      debugPrintln("Telnet monitor now active - you should see this message!");
    }

    // Initialize syslog client for remote logging (always init, can enable via MQTT)
    syslog.begin(SYSLOG_SERVER, SYSLOG_PORT, getSanitizedHostname().c_str(), "wesense");
    if (ENABLE_SYSLOG) {
      Serial.print("Syslog: Enabled, logging to ");
      Serial.print(SYSLOG_SERVER);
      Serial.print(":");
      Serial.println(SYSLOG_PORT);
    }

    if (ENABLE_MQTT) {
      updateBootStatus("Setting up MQTT...");
      // ESP32-C3 fix: Set longer socket timeout for single-core stability
      espClient.setTimeout(10);  // 10 seconds (default is often too short for C3)
      client.setServer(mqtt_server, mqtt_port);
      client.setKeepAlive(60);   // 60 seconds keepalive (default 15s too aggressive for sensor reading delays)
      client.setCallback(mqttCallback); // Register MQTT callback for incoming commands
      Serial.println("MQTT: Enabled and configured");
    } else {
      Serial.println("MQTT: Disabled by configuration (ENABLE_MQTT = false)");
    }
  }
  Serial.println("================================\n");

  Serial.println("\n=== CALIBRATION LOADING STATUS ===");
  // Load calibration state now that time is synchronized
  debugPrintln("Loading calibration state after time synchronisation...");
  if (wifiConnected) {
    loadCalibrationState();
    calibrationStateLoaded = true; // Mark as loaded to prevent duplicate loading in loop
  } else {
    Serial.println("WiFi not connected - calibration state will be loaded when time sync is available");
    Serial.println("Calibration state loading deferred until time synchronisation available (GPS or WiFi)");
  }
  Serial.println("===================================\n");

  Serial.println("\n=== LORA/LORAWAN INITIALIZATION ===");
  if (HAS_LORA() && !DISABLE_LORAWAN) {
    initializeLoRaWAN();
  } else if (!HAS_LORA()) {
    Serial.println(F("LoRaWAN: Disabled (no LoRa hardware detected)"));
  } else if (DISABLE_LORAWAN) {
    Serial.println(F("LoRaWAN: Disabled by configuration (DISABLE_LORAWAN = true)"));
    Serial.println(F("LoRaWAN: Set DISABLE_LORAWAN = false to enable (may cause crashes on ESP32-S3)"));
  }
  Serial.println("====================================\n");

  // Switch LED to slow blink mode now that setup is complete
  setLEDMode(false); // false = slow blink mode

  // Log deep sleep sensor compatibility warnings
  if (ENABLE_DEEP_SLEEP && isDeepSleepValid()) {
    logDeepSleepSensorWarnings();
  }

  debugPrintln("----------------------------------");
  debugPrintln("Setup complete. Entering loop.");
  debugPrintln("----------------------------------");

  // Log startup to syslog with device info
  syslog.notice("STARTUP device=" + String(deviceID) +
                " heap=" + String(ESP.getFreeHeap()) +
                " board=" + String(g_boardConfig.name));

  // Show ready status before entering main loop
  updateBootStatus("Ready!");
  delay(1000);  // Brief pause to show "Ready!"
  Serial.println("About to enter main loop...");
  Serial.flush();
} 


// ======================
// DISPLAY UPDATE FUNCTIONS
// ======================

// Icon lookup table mapping screen numbers to their icons
const uint8_t* screenIcons[] = {
  icon_environment,   // Screen 0: Environment
  icon_airquality,    // Screen 1: Air Quality
  icon_power,         // Screen 2: Power
  icon_lora,          // Screen 3: LoRa
  icon_network,       // Screen 4: Network
  icon_system,        // Screen 5: System
  icon_sensors,       // Screen 6: Sensors
  icon_debug          // Screen 7: LoRaWAN Debug
};

// Draw navigation bar at bottom of screen showing screen icons
// currentIdx = index into availableScreens array (which screen is selected)
// screenCount = total number of available screens
// availableScreens = array of actual screen numbers
void drawNavigationBar(int currentIdx, int screenCount, int availableScreens[]) {
  if (screenCount <= 1 || display == nullptr) return;

  // Calculate bar position and icon layout
  const int iconSize = WESENSE_ICON_WIDTH;
  const int spacing = NAV_BAR_ICON_SPACING;
  const int totalWidth = screenCount * iconSize + (screenCount - 1) * spacing;
  const int startX = (128 - totalWidth) / 2;  // Center the icons
  const int barY = 64 - NAV_BAR_HEIGHT;       // Bottom of screen

  // Clear the navigation bar area completely (solid black background)
  display->fillRect(0, barY, 128, NAV_BAR_HEIGHT, SSD1306_BLACK);

  // Draw each screen icon
  for (int i = 0; i < screenCount; i++) {
    int screenNum = availableScreens[i];
    int iconX = startX + i * (iconSize + spacing);
    int iconY = barY + 1;  // 1px padding from top of bar

    // Get the icon for this screen
    const uint8_t* icon = screenIcons[screenNum];

    if (i == currentIdx) {
      // Current screen: draw inverted (white background, black icon)
      display->fillRect(iconX - 1, iconY - 1, iconSize + 2, iconSize + 2, SSD1306_WHITE);
      display->drawBitmap(iconX, iconY, icon, iconSize, iconSize, SSD1306_BLACK);
    } else {
      // Other screens: draw normal (black background, white icon)
      display->drawBitmap(iconX, iconY, icon, iconSize, iconSize, SSD1306_WHITE);
    }
  }
}

void updateDisplay() {
  if (!displayAvailable || DISABLE_DISPLAY || display == nullptr) return;
  
  // Check if display should timeout after 10 minutes
  if (!displayTimedOut && millis() - displayStartTime >= DISPLAY_TIMEOUT_MS) {
    displayClear();
    displayRefresh(); // Clear the screen
    displayTimedOut = true;
    Serial.println("OLED Display timed out after 10 minutes - screen turned off");
    return;
  }
  
  // Skip updates if display has timed out
  if (displayTimedOut) return;
  
  static unsigned long lastDisplayUpdate = 0;
  
  // Determine which screens to show based on available sensors
  // Environment: T/H/P sensors
  bool showEnvironmental = (sht4xAvailable || tmp117Available || aht20Available ||
                            bme680Available || bmp280Available || bmp390Available || ms5611Available);
  // Air Quality: CO2, PM, VOC, NOx sensors
  bool showAirQuality = (scd4xAvailable || scd30Available || cm1106cAvailable ||
                         sgp41Available || pms5003Available);
  bool showPowerMonitor = (ina219Available || pmuAvailable);
  bool showLoRaStatus = (HAS_LORA() && !DISABLE_LORAWAN);
  bool showNetwork = true;      // Always show network (WiFi/MQTT)
  bool showSystemStatus = true; // Always show system status
  bool showSensorsList = true;  // Always show sensors list
  bool showLoRaDebug = (HAS_LORA() && !DISABLE_LORAWAN && ENABLE_DEBUG_OUTPUT);  // Extra debug info

  // Build array of available screens
  // Screens: 0=Environment, 1=AirQuality, 2=Power, 3=LoRa, 4=Network, 5=System, 6=Sensors, 7=LoRaDebug
  int availableScreens[9];
  int screenCount = 0;

  if (showEnvironmental) availableScreens[screenCount++] = 0;
  if (showAirQuality) availableScreens[screenCount++] = 1;
  if (showPowerMonitor) availableScreens[screenCount++] = 2;
  if (showLoRaStatus) availableScreens[screenCount++] = 3;
  if (showNetwork) availableScreens[screenCount++] = 4;
  if (showSystemStatus) availableScreens[screenCount++] = 5;
  if (showSensorsList) availableScreens[screenCount++] = 6;
  if (showLoRaDebug) availableScreens[screenCount++] = 7;
  
  // Update display content every 2 seconds
  if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL_MS) {
    lastDisplayUpdate = millis();
    
    displayClear();
    display->setTextSize(1);
    display->setTextColor(1);  // Use 1 for white (compatible with SSD1306 and SH1106)

    // Draw WeSense branding on the left
    display->setCursor(0, 0);
    display->print("WeSense");

    // Draw status icons in header (right side, right-to-left)
    int headerIconX = 120;  // Start from right edge (128 - 8 icon width)

    // WiFi signal strength icon (8x8) - rightmost
    const uint8_t* wifiIcon;
    if (!wifiConnected) {
      wifiIcon = icon_wifi_none;
    } else {
      int rssi = WiFi.RSSI();
      if (rssi >= -50) wifiIcon = icon_wifi_good;
      else if (rssi >= -60) wifiIcon = icon_wifi_fair;
      else if (rssi >= -70) wifiIcon = icon_wifi_weak;
      else wifiIcon = icon_wifi_none;
    }
    display->drawBitmap(headerIconX, 0, wifiIcon, 8, 8, SSD1306_WHITE);
    headerIconX -= 9;

    // TX/RX activity indicators (show for 2 seconds after activity)
    // Only show if the timestamp has been set (not 0)
    unsigned long now = millis();
    bool showTx = (lastMqttTxTime > 0 && now - lastMqttTxTime < ACTIVITY_INDICATOR_MS) ||
                  (lastLoraTxTime > 0 && now - lastLoraTxTime < ACTIVITY_INDICATOR_MS);
    bool showRx = (lastMqttRxTime > 0 && now - lastMqttRxTime < ACTIVITY_INDICATOR_MS) ||
                  (lastLoraRxTime > 0 && now - lastLoraRxTime < ACTIVITY_INDICATOR_MS);

    if (showTx) {
      display->drawBitmap(headerIconX, 0, icon_tx, 8, 8, SSD1306_WHITE);
      headerIconX -= 9;
    }
    if (showRx) {
      display->drawBitmap(headerIconX, 0, icon_rx, 8, 8, SSD1306_WHITE);
      headerIconX -= 9;
    }

    // Draw node name on second line of header (y=8, still in yellow zone)
    // Implements horizontal scrolling for long names
    const char* nodeName = getNodeName();
    int nodeNameLen = strlen(nodeName);

    if (nodeNameLen <= DISPLAY_MAX_CHARS) {
      // Name fits, no scrolling needed
      display->setCursor(0, 8);
      display->print(nodeName);
      hScrollState = SCROLL_WAIT_START;
      hScrollOffset = 0;
    } else {
      // Name is too long, implement scrolling
      hScrollMaxOffset = nodeNameLen - DISPLAY_MAX_CHARS;

      // Update scroll state machine
      unsigned long now = millis();
      switch (hScrollState) {
        case SCROLL_WAIT_START:
          if (hScrollTimer == 0) hScrollTimer = now;
          if (now - hScrollTimer >= SCROLL_WAIT_MS) {
            hScrollState = SCROLL_MOVING;
            hScrollTimer = now;
          }
          break;

        case SCROLL_MOVING:
          if (now - hScrollTimer >= SCROLL_STEP_MS) {
            hScrollOffset++;
            hScrollTimer = now;
            if (hScrollOffset >= hScrollMaxOffset) {
              hScrollState = SCROLL_WAIT_END;
              hScrollTimer = now;
            }
          }
          break;

        case SCROLL_WAIT_END:
          if (now - hScrollTimer >= SCROLL_WAIT_MS) {
            hScrollState = SCROLL_WAIT_START;
            hScrollOffset = 0;
            hScrollTimer = now;
          }
          break;
      }

      // Draw the visible portion of the name
      display->setCursor(0, 8);
      display->print(&nodeName[hScrollOffset]);
    }

    // Draw thin separator line at yellow/blue boundary (for two-color OLEDs)
    display->drawLine(0, 15, 127, 15, 1);

    // Start content below header (in blue zone on two-color displays)
    display->setCursor(0, 18);

    // Get the actual screen to display
    int actualScreen = availableScreens[currentDisplayScreen];

    // Reset vertical scroll when screen changes
    if (actualScreen != lastDisplayScreenForScroll) {
      lastDisplayScreenForScroll = actualScreen;
      vScrollState = SCROLL_WAIT_START;
      vScrollOffset = 0;
      vScrollTimer = 0;
      vScrollMaxOffset = 0;
    }

    switch (actualScreen) {
      case 0: { // Environmental sensors (Temp, Humidity, Pressure)
        display->println("[Environment]");

        // Temperature - large primary value (y=26, after title at y=18)
        if (currentBestReadings.temperature.valid) {
          display->setTextSize(2);
          display->print(currentBestReadings.temperature.value, 1);
          display->setTextSize(1);
          display->print(" C");
        } else {
          display->print("Temp: ---");
        }

        // Move cursor to next line after large text (y=26 + 16 = y=42)
        display->setCursor(0, 42);

        // Humidity and Pressure on same line if both available
        bool hasHumidity = currentBestReadings.humidity.valid;
        bool hasPressure = currentBestReadings.pressure.valid;

        if (hasHumidity && hasPressure) {
          // Compact two-column layout
          display->print("H:");
          display->print(currentBestReadings.humidity.value, 0);
          display->print("%  P:");
          display->print(currentBestReadings.pressure.value, 0);
          display->println("hPa");
        } else {
          if (hasHumidity) {
            display->print("Humidity: ");
            display->print(currentBestReadings.humidity.value, 1);
            display->println(" %");
          }
          if (hasPressure) {
            display->print("Pressure: ");
            display->print(currentBestReadings.pressure.value, 0);
            display->println(" hPa");
          }
        }
        break;
      }

      case 1: { // Air quality (CO2, PM, VOC, NOx)
        display->println("[Air Quality]");

        // CO2 - large primary value
        if (currentBestReadings.co2.valid) {
          display->setTextSize(2);
          display->print((int)currentBestReadings.co2.value);
          display->setTextSize(1);
          display->print(" ppm");
        }

        // Move cursor to next line after large text (y=26 + 16 = y=42)
        display->setCursor(0, 42);

        // PM values on same line if both available
        bool hasPM25 = currentBestReadings.pm2_5.valid;
        bool hasPM10 = currentBestReadings.pm10.valid;
        if (hasPM25 && hasPM10) {
          display->print("PM2.5:");
          display->print(currentBestReadings.pm2_5.value, 0);
          display->print(" PM10:");
          display->println(currentBestReadings.pm10.value, 0);
        } else {
          if (hasPM25) {
            display->print("PM2.5: ");
            display->print(currentBestReadings.pm2_5.value, 0);
            display->println(" ug/m3");
          }
          if (hasPM10) {
            display->print("PM10: ");
            display->print(currentBestReadings.pm10.value, 0);
            display->println(" ug/m3");
          }
        }

        // VOC and NOx on same line if both available
        bool hasVOC = currentBestReadings.voc.valid;
        bool hasNOx = currentBestReadings.nox.valid;
        if (hasVOC && hasNOx) {
          display->print("VOC:");
          display->print((int)currentBestReadings.voc.value);
          display->print(" NOx:");
          display->println((int)currentBestReadings.nox.value);
        } else {
          if (hasVOC) {
            display->print("VOC: ");
            display->println((int)currentBestReadings.voc.value);
          }
          if (hasNOx) {
            display->print("NOx: ");
            display->println((int)currentBestReadings.nox.value);
          }
        }
        break;
      }
        
      case 2: // Power monitoring
        display->println("[Power]");
        
        if (ina219Available) {
          float busV = ina219.getBusVoltage_V();
          float current = ina219.getCurrent_mA();
          float power = ina219.getPower_mW();
          
          display->print("Bus: ");
          display->print(busV, 2);
          display->println(" V");
          
          display->print("Current: ");
          display->print(current, 0);
          display->println(" mA");
          
          display->print("Power: ");
          display->print(power, 0);
          display->println(" mW");
        } else {
          display->println("INA219: Disabled");
        }
        
        if (pmuAvailable) {
          display->print("Battery: ");
          display->print(PMU.getBattVoltage(), 2);
          display->println(" V");
        }
        break;
        
      case 3: // LoRa Status
        display->println("[LoRa]");

        // Connection status with TX/RX indicator
        // Note: RadioLib uses negative codes for some success states:
        //   -1117 = RADIOLIB_LORAWAN_SESSION_RESTORED (success)
        //   -1118 = RADIOLIB_LORAWAN_NEW_SESSION (success)
        if (joinAccepted && (lorawan_last_error >= RADIOLIB_ERR_NONE ||
                             lorawan_last_error == -1117 ||
                             lorawan_last_error == -1118)) {
          if (lorawan_transmitting) display->println("Connected [TX]");
          else if (lorawan_receiving) display->println("Connected [RX]");
          else display->println("Connected");
        } else if (joinAccepted && lorawan_last_error < RADIOLIB_ERR_NONE) {
          display->println("Connection Lost");
        } else if (lorawanJoined) {
          display->println("Joining...");
        } else {
          display->println("Not joined");
        }

        // TX counts and result on same line
        display->print("TX:");
        display->print(lorawan_tx_success);
        display->print("/");
        display->print(lorawan_tx_count);
        if (lorawan_last_error != 0 && lorawan_last_error != RADIOLIB_ERR_NONE) {
          display->print(" E:");
          display->println(lorawan_last_error);
        } else {
          display->println();
        }

        // Next TX or join attempt countdown
        if (joinAccepted && lastLoRaWANTransmission > 0) {
          unsigned long nextTx = (TX_INTERVAL_MS - (millis() - lastLoRaWANTransmission)) / 1000;
          if (nextTx < TX_INTERVAL_MS / 1000) {
            display->print("Next TX: ");
            display->print(nextTx);
            display->println("s");
          }
        } else if (!joinAccepted && lastJoinAttempt > 0) {
          unsigned long nextJoin = (JOIN_RETRY_INTERVAL_MS - (millis() - lastJoinAttempt)) / 1000;
          if (nextJoin < JOIN_RETRY_INTERVAL_MS / 1000) {
            display->print("Next join: ");
            display->print(nextJoin);
            display->println("s");
          }
        }
        break;

      case 4: { // Network status
        display->println("[Network]");

        // WiFi status
        if (wifiConnected) {
          display->print("SSID: ");
          display->println(WiFi.SSID());
          display->print("IP: ");
          display->println(WiFi.localIP());
        } else {
          display->println("WiFi: Disconnected");
        }

        // MQTT status with broker
        display->print("MQTT: ");
        if (client.connected()) {
          display->println("Connected");
          display->println(mqtt_server);
        } else {
          display->println("Disconnected");
        }
        break;
      }

      case 5: { // System status
        display->println("[System]");

        // Show current time
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
          char timeStr[20];
          strftime(timeStr, sizeof(timeStr), "%H:%M:%S %d/%m", &timeinfo);
          display->println(timeStr);
        } else {
          display->println("Time: Not synced");
        }

        // Compact status line: WiFi + MQTT on same line
        display->print("W:");
        display->print(wifiConnected ? "OK" : "--");
        display->print(" M:");
        display->print(client.connected() ? "OK" : "--");
        if (HAS_LORA() && !DISABLE_LORAWAN) {
          display->print(" L:");
          display->println(joinAccepted ? "OK" : "--");
        } else {
          display->println();
        }

        // IP address (if connected)
        if (wifiConnected) {
          display->println(WiFi.localIP());
        }

        // Uptime
        display->print("Up: ");
        unsigned long uptime = millis() / 1000;
        if (uptime < 60) {
          display->print(uptime);
          display->println("s");
        } else if (uptime < 3600) {
          display->print(uptime / 60);
          display->println("m");
        } else {
          display->print(uptime / 3600);
          display->println("h");
        }
        break;
      }

      case 6: { // Sensors list with vertical scrolling
        display->println("[Sensors]");

        // Build list of available sensors
        const char* sensorNames[16];  // Max sensors we might have
        int sensorCount = 0;

        // I2C Sensors
        if (sht4xAvailable) sensorNames[sensorCount++] = "SHT4x (T/H)";
        if (tmp117Available) sensorNames[sensorCount++] = "TMP117 (T)";
        if (aht20Available) sensorNames[sensorCount++] = "AHT20 (T/H)";
        if (bme680Available) sensorNames[sensorCount++] = "BME680 (T/H/P/G)";
        if (bmp280Available) sensorNames[sensorCount++] = "BMP280 (T/P)";
        if (bmp390Available) sensorNames[sensorCount++] = "BMP390 (T/P)";
        if (ms5611Available) sensorNames[sensorCount++] = "MS5611 (T/P)";
        if (scd4xAvailable) sensorNames[sensorCount++] = "SCD4x (CO2/T/H)";
        if (scd30Available) sensorNames[sensorCount++] = "SCD30 (CO2/T/H)";
        if (cm1106cAvailable) sensorNames[sensorCount++] = "CM1106 (CO2)";
        if (sgp41Available) sensorNames[sensorCount++] = "SGP41 (VOC/NOx)";
        if (ina219Available) sensorNames[sensorCount++] = "INA219 (Power)";

        // UART Sensors
        if (pms5003Available) sensorNames[sensorCount++] = "PMS5003 (PM)";
        if (c8co2Available) sensorNames[sensorCount++] = "CM1106-C (CO2)";

        // GPS
        if (gpsAvailable) sensorNames[sensorCount++] = "GPS";

        if (sensorCount == 0) {
          display->println("No sensors found");
        } else {
          // Calculate if scrolling is needed
          // Content area: y=26 (after title) to y=63 = 37 pixels = 4 lines
          const int visibleLines = 4;
          int totalLines = sensorCount;

          if (totalLines <= visibleLines) {
            // All sensors fit, no scrolling needed
            for (int i = 0; i < sensorCount; i++) {
              display->println(sensorNames[i]);
            }
          } else {
            // Need vertical scrolling
            vScrollMaxOffset = totalLines - visibleLines;

            // Update vertical scroll state machine
            unsigned long now = millis();
            switch (vScrollState) {
              case SCROLL_WAIT_START:
                if (vScrollTimer == 0) vScrollTimer = now;
                if (now - vScrollTimer >= SCROLL_WAIT_MS) {
                  vScrollState = SCROLL_MOVING;
                  vScrollTimer = now;
                }
                break;

              case SCROLL_MOVING:
                if (now - vScrollTimer >= SCROLL_STEP_MS * 2) {  // Slower vertical scroll
                  vScrollOffset++;
                  vScrollTimer = now;
                  if (vScrollOffset >= vScrollMaxOffset) {
                    vScrollState = SCROLL_WAIT_END;
                    vScrollTimer = now;
                  }
                }
                break;

              case SCROLL_WAIT_END:
                if (now - vScrollTimer >= SCROLL_WAIT_MS) {
                  vScrollState = SCROLL_WAIT_START;
                  vScrollOffset = 0;
                  vScrollTimer = now;
                }
                break;
            }

            // Draw visible sensors with scroll offset
            for (int i = 0; i < visibleLines && (i + vScrollOffset) < sensorCount; i++) {
              display->println(sensorNames[i + vScrollOffset]);
            }

            // Show scroll indicator if not at end
            if (vScrollOffset < vScrollMaxOffset) {
              display->setCursor(122, 56);  // Bottom right corner
              display->print("v");  // Down arrow indicator
            }
          }
        }
        break;
      }

      case 7: { // LoRaWAN Debug (only shown when ENABLE_DEBUG_OUTPUT)
        display->println("[LoRa Debug]");

        // DevEUI (16 hex chars fits on one line without label)
        for (int i = 0; i < 8; i++) {
          if (USER_DEVEUI[i] < 0x10) display->print("0");
          display->print(USER_DEVEUI[i], HEX);
        }
        display->println();

        // Region and TX count on same line
        #if defined(LORAWAN_REGION_AU915)
          display->print("AU915");
        #elif defined(LORAWAN_REGION_EU868)
          display->print("EU868");
        #elif defined(LORAWAN_REGION_US915)
          display->print("US915");
        #elif defined(LORAWAN_REGION_AS923)
          display->print("AS923");
        #elif defined(LORAWAN_REGION_IN865)
          display->print("IN865");
        #elif defined(LORAWAN_REGION_KR920)
          display->print("KR920");
        #elif defined(LORAWAN_REGION_CN470)
          display->print("CN470");
        #else
          display->print("???");
        #endif

        if (lorawan_tx_count > 0) {
          int successRate = (lorawan_tx_success * 100) / lorawan_tx_count;
          display->print(" TX:");
          display->print(successRate);
          display->print("% ");
          display->print(lorawan_tx_success);
          display->print("/");
          display->println(lorawan_tx_count);
        } else {
          display->println(" TX:0");
        }

        // Last error (if any)
        if (lorawan_last_error != 0 && lorawan_last_error != RADIOLIB_ERR_NONE) {
          display->print("Err: ");
          display->println(lorawan_last_error);
        }
        break;
      }
    }

    // Check if navigation bar should still be shown (auto-hide after 2 seconds)
    if (showNavigationBar && (millis() - lastScreenSwitchTime >= NAV_BAR_HIDE_DELAY_MS)) {
      showNavigationBar = false;
    }

    // Draw navigation bar overlay if visible
    if (showNavigationBar && screenCount > 1) {
      drawNavigationBar(currentDisplayScreen, screenCount, availableScreens);
    }

    displayRefresh();
  }
}

// ======================
// UNIFIED SENSOR READING
// ======================
// Reads ALL sensors in a single pass (~6 seconds total)
// Returns the cycle start timestamp for LoRaWAN (single timestamp mode)
// Individual sensor timestamps are captured in updateSensorReading() for MQTT

uint32_t readAllSensors() {
  // Capture cycle start timestamp (used by LoRaWAN for single-timestamp mode)
  uint32_t cycleStartTime = getCurrentUnixTime();
  lastSensorReadCycleTimestamp = cycleStartTime;

  Serial.println("\n============================================");
  Serial.println("=== UNIFIED SENSOR READ CYCLE START ===");
  Serial.printf("Cycle timestamp: %lu\n", cycleStartTime);
  Serial.println("============================================");

  // Reset sensor hierarchy for this cycle
  resetSensorHierarchy();

  uint16_t errorLoop;

  // --- Temperature & Humidity Sensors ---

  // TMP117 (priority 0 - highest accuracy temperature sensor ±0.1°C)
  // Temperature only - no humidity
  if (tmp117Available) {
    float tmp117_temp = tmp117.readTempC();
    if (!isnan(tmp117_temp) && tmp117_temp > -50.0 && tmp117_temp < 150.0) {
      Serial.println("TMP117:");
      Serial.printf("  Temp: %.3f °C (±0.1°C precision)\n", tmp117_temp);
      if (!DISABLE_TMP117_TEMPERATURE) {
        updateSensorReading("temperature", tmp117_temp, "TMP117", 0, "temperature");
      } else {
        Serial.println("  (Temperature disabled by config)");
      }
    } else {
      Serial.println("TMP117: Read failed or out of range");
    }
  }

  // SHT4x (priority 1 - high accuracy temp & humidity)
  // Note: SHT4x stores reference to correct Wire object from initialization.
  // No bus switching needed - library uses stored Wire/Wire1 reference.
  if (sht4xAvailable) {
    float sht_temp_val = 0, sht_hum_val = 0;
    errorLoop = sht4x.measureHighPrecision(sht_temp_val, sht_hum_val);

    if (errorLoop == 0) {
      Serial.printf("%s:\n", sht4xDetectedVariant.c_str());
      Serial.printf("  Temp: %.2f °C, Humidity: %.2f %%\n", sht_temp_val, sht_hum_val);
      if (!DISABLE_SHT4X_TEMPERATURE) {
        updateSensorReading("temperature", sht_temp_val, sht4xDetectedVariant.c_str(), 1, "temperature");
      } else {
        Serial.println("  (Temperature disabled by config)");
      }
      if (!DISABLE_SHT4X_HUMIDITY) {
        updateSensorReading("humidity", sht_hum_val, sht4xDetectedVariant.c_str(), 1, "humidity");
      } else {
        Serial.println("  (Humidity disabled by config)");
      }
    } else {
      Serial.printf("%s read failed, error: %d\n", sht4xDetectedVariant.c_str(), errorLoop);
    }
  }

  // AHT20 (priority 3 - fallback for temp, but may be only source of humidity)
  // Must read if either temperature OR humidity is needed (humidity may not be available elsewhere)
  if (aht20Available) {
    if (!currentBestReadings.temperature.valid || !currentBestReadings.humidity.valid) {
      // Handle secondary I2C bus if needed
      if (sensorLocations.aht20.bus == 1 && HAS_DUAL_I2C()) {
        Wire.end();
        delay(50);
        Wire.begin(g_boardConfig.sda_pin2, g_boardConfig.scl_pin2, 100000);
        delay(50);
      }

      // Warm up the I2C bus with a ping to AHT20 address before reading
      // This helps with devices that need bus activity to respond reliably
      Wire.beginTransmission(0x38);
      Wire.endTransmission();
      delay(10);

      sensors_event_t aht_humidity_event, aht_temp_event;
      bool aht_success = aht20.getEvent(&aht_humidity_event, &aht_temp_event);

      // If first read fails, try raw I2C read to diagnose
      if (!aht_success) {
        if (ENABLE_DEBUG_OUTPUT) {
          Serial.println("AHT20: Library read failed, trying raw I2C...");

          // Check if device responds at all
          Wire.beginTransmission(0x38);
          byte i2c_error = Wire.endTransmission();
          Serial.print("  I2C ping to 0x38: ");
          if (i2c_error == 0) {
            Serial.println("device responds");

            // Try triggering a measurement manually
            // AHT20 trigger: write 0xAC 0x33 0x00
            Wire.beginTransmission(0x38);
            Wire.write(0xAC);  // Trigger measurement command
            Wire.write(0x33);
            Wire.write(0x00);
            i2c_error = Wire.endTransmission();
            Serial.print("  Trigger measurement: ");
            Serial.println(i2c_error == 0 ? "OK" : "FAILED");

            if (i2c_error == 0) {
              delay(80);  // AHT20 needs ~80ms for measurement

              // Read 7 bytes of data
              uint8_t bytesRead = Wire.requestFrom((uint8_t)0x38, (uint8_t)7);
              Serial.print("  Bytes received: "); Serial.println(bytesRead);

              if (bytesRead >= 6) {
                uint8_t data[7];
                for (int i = 0; i < bytesRead && i < 7; i++) {
                  data[i] = Wire.read();
                }
                Serial.print("  Raw data: ");
                for (int i = 0; i < bytesRead && i < 7; i++) {
                  Serial.print("0x"); Serial.print(data[i], HEX); Serial.print(" ");
                }
                Serial.println();

                // Parse if we got valid data (status byte bit 7 should be 0 when ready)
                if ((data[0] & 0x80) == 0) {
                  // Calculate humidity and temperature from raw data
                  uint32_t humidity_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
                  uint32_t temp_raw = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];

                  float humidity = ((float)humidity_raw / 1048576.0) * 100.0;
                  float temperature = ((float)temp_raw / 1048576.0) * 200.0 - 50.0;

                  Serial.printf("  Calculated: Temp=%.2f°C, Humidity=%.2f%%\n", temperature, humidity);

                  // Use the raw values since library failed
                  aht_temp_event.temperature = temperature;
                  aht_humidity_event.relative_humidity = humidity;
                  aht_success = true;
                } else {
                  Serial.println("  Status indicates busy/not ready");
                }
              }
            }
          } else {
            Serial.print("device NOT responding (error="); Serial.print(i2c_error); Serial.println(")");
          }
        }
      }

      // Switch back to primary bus if we switched
      if (sensorLocations.aht20.bus == 1 && HAS_DUAL_I2C()) {
        Wire.end();
        delay(50);
        Wire.begin(g_boardConfig.sda_pin, g_boardConfig.scl_pin, 100000);
        delay(50);
      }

      if (aht_success) {
        Serial.println("AHT20:");
        Serial.printf("  Temp: %.2f °C, Humidity: %.2f %%\n",
          aht_temp_event.temperature, aht_humidity_event.relative_humidity);
        if (!DISABLE_AHT20_TEMPERATURE) {
          updateSensorReading("temperature", aht_temp_event.temperature, "AHT20", 3, "temperature");
        } else {
          Serial.println("  (Temperature disabled by config)");
        }
        if (!DISABLE_AHT20_HUMIDITY) {
          updateSensorReading("humidity", aht_humidity_event.relative_humidity, "AHT20", 3, "humidity");
        } else {
          Serial.println("  (Humidity disabled by config)");
        }
      } else {
        Serial.println("AHT20: Read failed (after retry)");
      }
    } else {
      Serial.println("AHT20: Skipped (temp and humidity already valid)");
    }
  } else {
    Serial.println("AHT20: Not available");
  }

  // BME680 (priority 5 for T/H, priority 4 for pressure)
  // NOTE: In deep sleep mode, gas resistance is skipped (requires continuous heating)
  // but T/H/P readings are still valid and enabled
  if (bme680Available) {
    if (bme680.performReading()) {
      Serial.println("BME680:");
      Serial.printf("  Temp: %.2f °C, Humidity: %.2f %%\n", bme680.temperature, bme680.humidity);
      float bme_pressure_hPa = bme680.pressure / 100.0;

      // T/H/P readings are always valid (unless individually disabled)
      if (!DISABLE_BME680_TEMPERATURE) {
        updateSensorReading("temperature", bme680.temperature, "BME680", 5, "temperature");
      } else {
        Serial.println("  (Temperature disabled by config)");
      }
      if (!DISABLE_BME680_HUMIDITY) {
        updateSensorReading("humidity", bme680.humidity, "BME680", 5, "humidity");
      } else {
        Serial.println("  (Humidity disabled by config)");
      }
      updateSensorReading("pressure", bme_pressure_hPa, "BME680", 4, "pressure");

      // Gas resistance only valid with continuous operation (skip in deep sleep mode)
      if (!ENABLE_DEEP_SLEEP) {
        Serial.printf("  Pressure: %.2f hPa, Gas: %.2f kOhm\n", bme_pressure_hPa, bme680.gas_resistance / 1000.0);
        // BME680 is the only gas resistance sensor, priority 1
        updateSensorReading("gas_resistance", bme680.gas_resistance / 1000.0, "BME680", 1, "gas_resistance");
      } else {
        Serial.printf("  Pressure: %.2f hPa, Gas: skipped (deep sleep mode)\n", bme_pressure_hPa);
      }
    } else {
      Serial.println("BME680 read failed");
    }
  }

  // --- Pressure Sensor (BMP280) ---
  if (bmp280Available) {
    // Handle secondary I2C bus if needed
    if (sensorLocations.bmp280.bus == 1 && HAS_DUAL_I2C()) {
      Wire.end();
      delay(50);
      Wire.begin(g_boardConfig.sda_pin2, g_boardConfig.scl_pin2, 100000);
      delay(50);
    }

    // Force a fresh measurement to avoid stuck/cached values
    // takeForcedMeasurement() triggers a new reading in FORCED mode
    if (bmp280.takeForcedMeasurement()) {
      float pressure_Pa = bmp280.readPressure();
      float bmp_temp = bmp280.readTemperature();

      // Switch back to primary bus
      if (sensorLocations.bmp280.bus == 1 && HAS_DUAL_I2C()) {
        Wire.end();
        delay(50);
        Wire.begin(g_boardConfig.sda_pin, g_boardConfig.scl_pin, 100000);
        delay(50);
      }

      if (!isnan(pressure_Pa) && !isnan(bmp_temp)) {
        float pressure_hPa = pressure_Pa / 100.0F;
        Serial.println("BMP280:");
        Serial.printf("  Pressure: %.2f hPa, Temp: %.2f °C\n", pressure_hPa, bmp_temp);
        updateSensorReading("pressure", pressure_hPa, "BMP280", 3, "pressure");
        if (!DISABLE_BMP280_TEMPERATURE) {
          updateSensorReading("temperature", bmp_temp, "BMP280", 6, "temperature");
        } else {
          Serial.println("  (Temperature disabled by config)");
        }
      }
    } else {
      // Switch back to primary bus even on failure
      if (sensorLocations.bmp280.bus == 1 && HAS_DUAL_I2C()) {
        Wire.end();
        delay(50);
        Wire.begin(g_boardConfig.sda_pin, g_boardConfig.scl_pin, 100000);
        delay(50);
      }
      Serial.println("BMP280: Forced measurement failed");
    }
  }

  // --- Pressure Sensor (BMP390) - higher precision than BMP280 ---
  if (bmp390Available) {
    // Handle secondary I2C bus if needed
    if (sensorLocations.bmp390.bus == 1 && HAS_DUAL_I2C()) {
      Wire.end();
      delay(50);
      Wire.begin(g_boardConfig.sda_pin2, g_boardConfig.scl_pin2, 100000);
      delay(50);
    }

    if (bmp390.performReading()) {
      // Switch back to primary bus
      if (sensorLocations.bmp390.bus == 1 && HAS_DUAL_I2C()) {
        Wire.end();
        delay(50);
        Wire.begin(g_boardConfig.sda_pin, g_boardConfig.scl_pin, 100000);
        delay(50);
      }

      float pressure_hPa = bmp390.pressure / 100.0F;
      float bmp390_temp = bmp390.temperature;

      // Skip publishing first 2 reads after boot - sensor needs time to stabilize
      if (bmp390WarmupReadsRemaining > 0) {
        Serial.println("BMP390 (WARMUP - not publishing):");
        Serial.printf("  Pressure: %.2f hPa, Temp: %.2f °C (skipping %d more reads)\n",
                      pressure_hPa, bmp390_temp, bmp390WarmupReadsRemaining);
        bmp390WarmupReadsRemaining--;
      } else {
        Serial.println("BMP390:");
        Serial.printf("  Pressure: %.2f hPa, Temp: %.2f °C\n", pressure_hPa, bmp390_temp);
        updateSensorReading("pressure", pressure_hPa, "BMP390", 2, "pressure");
        if (!DISABLE_BMP390_TEMPERATURE) {
          updateSensorReading("temperature", bmp390_temp, "BMP390", 5, "temperature");
        } else {
          Serial.println("  (Temperature disabled by config)");
        }
      }
    } else {
      // Switch back to primary bus even on failure
      if (sensorLocations.bmp390.bus == 1 && HAS_DUAL_I2C()) {
        Wire.end();
        delay(50);
        Wire.begin(g_boardConfig.sda_pin, g_boardConfig.scl_pin, 100000);
        delay(50);
      }
      Serial.println("BMP390 read failed");
    }
  }

  // --- Pressure Sensor (MS5611) - high precision altimeter ---
  if (ms5611Available) {
    // Handle secondary I2C bus if needed
    if (sensorLocations.ms5611.bus == 1 && HAS_DUAL_I2C()) {
      Wire.end();
      delay(50);
      Wire.begin(g_boardConfig.sda_pin2, g_boardConfig.scl_pin2, 100000);
      delay(50);
    }

    int result = ms5611.read();
    if (result == MS5611_READ_OK) {
      // Switch back to primary bus
      if (sensorLocations.ms5611.bus == 1 && HAS_DUAL_I2C()) {
        Wire.end();
        delay(50);
        Wire.begin(g_boardConfig.sda_pin, g_boardConfig.scl_pin, 100000);
        delay(50);
      }

      float pressure_hPa = ms5611.getPressure();  // Already in mBar (= hPa)
      float ms5611_temp = ms5611.getTemperature();

      // Skip publishing first 2 reads after boot - sensor needs time to stabilize
      if (ms5611WarmupReadsRemaining > 0) {
        Serial.println("MS5611 (WARMUP - not publishing):");
        Serial.printf("  Pressure: %.2f hPa, Temp: %.2f °C (skipping %d more reads)\n",
                      pressure_hPa, ms5611_temp, ms5611WarmupReadsRemaining);
        ms5611WarmupReadsRemaining--;
      } else {
        Serial.println("MS5611:");
        Serial.printf("  Pressure: %.2f hPa, Temp: %.2f °C\n", pressure_hPa, ms5611_temp);
        // MS5611 is highest precision pressure sensor - priority 1
        updateSensorReading("pressure", pressure_hPa, "MS5611", 1, "pressure");
        if (!DISABLE_MS5611_TEMPERATURE) {
          updateSensorReading("temperature", ms5611_temp, "MS5611", 5, "temperature");
        } else {
          Serial.println("  (Temperature disabled by config)");
        }
      }
    } else {
      // Switch back to primary bus even on failure
      if (sensorLocations.ms5611.bus == 1 && HAS_DUAL_I2C()) {
        Wire.end();
        delay(50);
        Wire.begin(g_boardConfig.sda_pin, g_boardConfig.scl_pin, 100000);
        delay(50);
      }
      Serial.print("MS5611 read failed, error: ");
      Serial.println(result);
    }
  }

  // --- CO2 Sensor (SCD4x) - takes ~5 seconds ---
  if (scd4xAvailable) {
    bool dataReady = scd4x.getDataReadyStatus();
    if (dataReady && scd4x.readMeasurement()) {
      uint16_t co2 = scd4x.getCO2();
      float scd_temp = scd4x.getTemperature();
      float scd_hum = scd4x.getHumidity();

      if (co2 > 0) {
        Serial.println("SCD4x:");
        Serial.printf("  CO2: %d ppm, Temp: %.2f °C, Humidity: %.2f %%\n", co2, scd_temp, scd_hum);

        updateSensorReading("co2", (float)co2, "SCD4x", 1, "co2");

        // T/H priority based on configuration
        int thPriority = USE_EXTERNAL_SENSORS_FOR_SCD4X_COMPENSATION ? 7 : 1;
        if (!DISABLE_SCD4X_TEMPERATURE) {
          updateSensorReading("temperature", scd_temp, "SCD4x", thPriority, "temperature");
        } else {
          Serial.println("  (Temperature disabled by config)");
        }
        if (!DISABLE_SCD4X_HUMIDITY) {
          updateSensorReading("humidity", scd_hum, "SCD4x", thPriority, "humidity");
        } else {
          Serial.println("  (Humidity disabled by config)");
        }
      }
    } else {
      Serial.println("SCD4x: No data ready or read failed");
    }
  }

  // --- CO2 Sensor (SCD30) ---
  if (scd30Available) {
    if (scd30.dataAvailable()) {
      float co2 = scd30.getCO2();
      float scd30_temp = scd30.getTemperature();
      float scd30_hum = scd30.getHumidity();

      if (co2 > 0) {
        Serial.println("SCD30:");
        Serial.printf("  CO2: %.0f ppm, Temp: %.2f °C, Humidity: %.2f %%\n", co2, scd30_temp, scd30_hum);

        // SCD30 has slightly lower priority than SCD4x (priority 2 vs 1) as SCD4x is newer/more accurate
        updateSensorReading("co2", co2, "SCD30", 2, "co2");

        // T/H priority based on configuration
        int thPriority = USE_EXTERNAL_SENSORS_FOR_SCD30_COMPENSATION ? 7 : 2;
        if (!DISABLE_SCD30_TEMPERATURE) {
          updateSensorReading("temperature", scd30_temp, "SCD30", thPriority, "temperature");
        } else {
          Serial.println("  (Temperature disabled by config)");
        }
        if (!DISABLE_SCD30_HUMIDITY) {
          updateSensorReading("humidity", scd30_hum, "SCD30", thPriority, "humidity");
        } else {
          Serial.println("  (Humidity disabled by config)");
        }
      }
    } else {
      Serial.println("SCD30: No data ready");
    }
  }

  // --- VOC/NOx Sensor (SGP41) ---
  // NOTE: SGP41 requires continuous operation for gas index algorithm
  // In deep sleep mode, readings are inaccurate and MUST be skipped
  if (sgp41Available && !ENABLE_DEEP_SLEEP) {
    // Get compensation values from best available sensors
    float compensationT = currentBestReadings.temperature.valid ? currentBestReadings.temperature.value : 25.0f;
    float compensationRH = currentBestReadings.humidity.valid ? currentBestReadings.humidity.value : 50.0f;

    uint16_t srawVoc = 0, srawNox = 0;
    uint16_t compensationTTicks = static_cast<uint16_t>((compensationT + 45.0f) * 65535.0f / 175.0f);
    uint16_t compensationRHTicks = static_cast<uint16_t>(compensationRH * 65535.0f / 100.0f);

    uint16_t sgpError = sgp41.measureRawSignals(compensationRHTicks, compensationTTicks, srawVoc, srawNox);

    if (sgpError == 0) {
      // Process raw signals through gas index algorithm
      int32_t voc_index = 0, nox_index = 0;
      GasIndexAlgorithm_process(&voc_params, srawVoc, &voc_index);
      GasIndexAlgorithm_process(&nox_params, srawNox, &nox_index);

      Serial.println("SGP41:");
      Serial.printf("  VOC Index: %d, NOx Index: %d\n", voc_index, nox_index);
      Serial.printf("  Raw VOC: %d, Raw NOx: %d\n", srawVoc, srawNox);

      updateSensorReading("voc", (float)voc_index, "SGP41", 1, "voc");
      updateSensorReading("nox", (float)nox_index, "SGP41", 1, "nox");
    } else {
      Serial.printf("SGP41 read failed, error: %d\n", sgpError);
    }
  } else if (sgp41Available && ENABLE_DEEP_SLEEP) {
    // Skip SGP41 in deep sleep mode - gas index algorithm requires continuous operation
    static bool sgp41_warned = false;
    if (!sgp41_warned) {
      Serial.println(F("SGP41: Skipped (deep sleep mode - gas index requires continuous operation)"));
      sgp41_warned = true;
    }
  }

  // --- Particulate Matter (PMS5003) ---
  if (pms5003Available) {
    // PMS5003 reads continuously - grab latest values
    if (readPMS5003Data(pms5003Data)) {
      Serial.println("PMS5003:");
      Serial.printf("  PM1.0: %d, PM2.5: %d, PM10: %d µg/m³\n",
        pms5003Data.pm1_0_atmospheric, pms5003Data.pm2_5_atmospheric, pms5003Data.pm10_atmospheric);

      // Use updateSensorReading to populate currentBestReadings for protobuf encoding
      // PMS5003 is priority 1 for all PM readings (only PM sensor currently supported)
      updateSensorReading("pm1", (float)pms5003Data.pm1_0_atmospheric, "PMS5003", 1, "pm1_0");
      updateSensorReading("pm2_5", (float)pms5003Data.pm2_5_atmospheric, "PMS5003", 1, "pm2_5");
      updateSensorReading("pm10", (float)pms5003Data.pm10_atmospheric, "PMS5003", 1, "pm10");
    } else {
      Serial.println("PMS5003: No data available (read failed)");
    }
  }

  // --- CM1106-C CO2 Sensor (I2C) ---
  if (cm1106cAvailable) {
    uint8_t result = cm1106c.measure_result();
    if (result == 0 && cm1106c.co2 > 0 && cm1106c.co2 < 10000) {
      Serial.println("CM1106-C (I2C):");
      Serial.printf("  CO2: %d ppm\n", cm1106c.co2);

      // CM1106-C I2C - priority 2 (lower than SCD4x=1, higher than UART=3)
      if (!DISABLE_CM1106C_CO2) {
        updateSensorReading("co2", (float)cm1106c.co2, "CM1106-C", 2, "co2");
      } else {
        Serial.println("  (CO2 disabled by config)");
      }
    } else if (result != 0) {
      Serial.printf("CM1106-C (I2C): Read failed, error: %d\n", result);
    }
  }

  // --- CM1106-C CO2 Sensor (UART) ---
  if (c8co2Available) {
    if (readC8CO2Data(c8co2Data)) {
      Serial.println("CM1106-C (UART):");
      Serial.printf("  CO2: %d ppm\n", c8co2Data.co2_ppm_alt);  // Bytes 6-7

      // CM1106-C UART - priority 3 (lower than SCD4x=1, SCD30=2, CM1106-C I2C=2)
      if (c8co2Data.co2_ppm_alt > 0 && c8co2Data.co2_ppm_alt < 10000) {
        if (!DISABLE_C8_CO2_CO2) {
          updateSensorReading("co2", (float)c8co2Data.co2_ppm_alt, "CM1106-C", 3, "co2");
        } else {
          Serial.println("  (CO2 disabled by config)");
        }
      }
    } else {
      Serial.println("CM1106-C (UART): No data available");
    }
  }

  // --- DC Power Monitor (INA219) ---
  if (ina219Available) {
    // Switch to appropriate I2C bus if needed
    if (sensorLocations.ina219.bus == 1 && HAS_DUAL_I2C()) {
      Wire.end();
      delay(50);
      Wire.begin(g_boardConfig.sda_pin2, g_boardConfig.scl_pin2, 100000);
      delay(50);
    }

    float busVoltage = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();
    float power_mW = ina219.getPower_mW();

    // Switch back to primary bus if needed
    if (sensorLocations.ina219.bus == 1 && HAS_DUAL_I2C()) {
      Wire.end();
      delay(50);
      Wire.begin(g_boardConfig.sda_pin, g_boardConfig.scl_pin, 100000);
      delay(50);
    }

    if (!isnan(busVoltage) && !isnan(current_mA) && !isnan(power_mW)) {
      Serial.println("INA219:");
      Serial.printf("  Voltage: %.2f V, Current: %.1f mA, Power: %.1f mW\n",
        busVoltage, current_mA, power_mW);

      updateSensorReading("voltage", busVoltage, "INA219", 1, "voltage");
      updateSensorReading("current", current_mA, "INA219", 1, "current");
      updateSensorReading("power", power_mW, "INA219", 1, "power");
    }
  }

  // --- Battery Monitor (PMU - T-Beam) ---
  if (pmuAvailable) {
    float batteryVoltage = PMU.getBattVoltage() / 1000.0;
    float batteryPercent = PMU.getBatteryPercent();

    Serial.println("PMU Battery:");
    Serial.printf("  Voltage: %.2f V, Level: %.0f%%\n", batteryVoltage, batteryPercent);

    // Battery level is what we track in currentBestReadings
    updateSensorReading("battery", batteryPercent, "AXP2101", 1, "battery_percent");
  }

  Serial.println("============================================");
  Serial.printf("=== SENSOR READ CYCLE COMPLETE ===\n");
  Serial.printf("Duration: ~%lu ms\n", millis() % 10000);  // Approximate
  Serial.printf("Readings captured:\n");
  if (currentBestReadings.temperature.valid)
    Serial.printf("  Temperature: %.1f°C from %s (ts: %lu)\n",
      currentBestReadings.temperature.value, currentBestReadings.temperature.sensor_name.c_str(),
      currentBestReadings.temperature.timestamp);
  if (currentBestReadings.humidity.valid)
    Serial.printf("  Humidity: %.1f%% from %s (ts: %lu)\n",
      currentBestReadings.humidity.value, currentBestReadings.humidity.sensor_name.c_str(),
      currentBestReadings.humidity.timestamp);
  if (currentBestReadings.pressure.valid)
    Serial.printf("  Pressure: %.2fhPa from %s (ts: %lu)\n",
      currentBestReadings.pressure.value, currentBestReadings.pressure.sensor_name.c_str(),
      currentBestReadings.pressure.timestamp);
  if (currentBestReadings.co2.valid)
    Serial.printf("  CO2: %.0fppm from %s (ts: %lu)\n",
      currentBestReadings.co2.value, currentBestReadings.co2.sensor_name.c_str(),
      currentBestReadings.co2.timestamp);
  if (currentBestReadings.voc.valid)
    Serial.printf("  VOC: %.0f from %s (ts: %lu)\n",
      currentBestReadings.voc.value, currentBestReadings.voc.sensor_name.c_str(),
      currentBestReadings.voc.timestamp);
  if (currentBestReadings.nox.valid)
    Serial.printf("  NOx: %.0f from %s (ts: %lu)\n",
      currentBestReadings.nox.value, currentBestReadings.nox.sensor_name.c_str(),
      currentBestReadings.nox.timestamp);
  if (currentBestReadings.voltage.valid)
    Serial.printf("  Voltage: %.2fV from %s (ts: %lu)\n",
      currentBestReadings.voltage.value, currentBestReadings.voltage.sensor_name.c_str(),
      currentBestReadings.voltage.timestamp);
  if (currentBestReadings.current.valid)
    Serial.printf("  Current: %.1fmA from %s (ts: %lu)\n",
      currentBestReadings.current.value, currentBestReadings.current.sensor_name.c_str(),
      currentBestReadings.current.timestamp);
  if (currentBestReadings.power.valid)
    Serial.printf("  Power: %.1fmW from %s (ts: %lu)\n",
      currentBestReadings.power.value, currentBestReadings.power.sensor_name.c_str(),
      currentBestReadings.power.timestamp);
  if (currentBestReadings.battery.valid)
    Serial.printf("  Battery: %.0f%% from %s (ts: %lu)\n",
      currentBestReadings.battery.value, currentBestReadings.battery.sensor_name.c_str(),
      currentBestReadings.battery.timestamp);
  Serial.println("============================================\n");

  return cycleStartTime;
}

// ======================
// MAIN LOOP
// ======================
void loop() {
  // MQTT publish timer (only used when LoRaWAN is disabled)
  // When LoRaWAN is enabled, MQTT publishes happen during LoRaWAN's sensor read cycle
  static unsigned long lastMqttPublishTime = 0;

  // Other timers
  static unsigned long lastNtpResyncTime = 0;
  static unsigned long lastDiagnosticsCheckTime = 0;
  static bool calibrationStateLoaded = false;
  static unsigned long loopDebugCounter = 0;
  static unsigned long lastLoopDebug = 0;
  
  // Debug loop counter every 10 seconds
  loopDebugCounter++;
  if (millis() - lastLoopDebug >= 10000) {
    uint32_t freeHeap = ESP.getFreeHeap();
    unsigned long uptimeSec = millis() / 1000;
    Serial.print("Loop alive - count: "); Serial.print(loopDebugCounter);
    Serial.print(", heap: "); Serial.print(freeHeap);
    Serial.print(", uptime: "); Serial.print(uptimeSec); Serial.println("s");
    Serial.flush();

    // Send to syslog for persistent logging
    syslog.info("loop count=" + String(loopDebugCounter) +
                " heap=" + String(freeHeap) +
                " uptime=" + String(uptimeSec) + "s");

    lastLoopDebug = millis();
    loopDebugCounter = 0;
  }

  // Update status LED (non-blocking)
  updateStatusLED();
  
  // Handle secure telnet monitor connections
  if (ENABLE_SECURE_TELNET && wifiConnected) {
    telnetMonitor.handle();
  }
  
  // Update calibration state (check if calibration period is complete)
  updateCalibrationState();
  
  // Handle boot button press for display control
  // Polling fallback: Check button state directly in case interrupts don't work
  bool currentButtonState = digitalRead(BOOT_BUTTON_PIN);
  if (currentButtonState == LOW && lastBootButtonState == HIGH) {
    // Button was just pressed (falling edge detected via polling)
    bootButtonPressed = true;
    Serial.println("Boot button press detected via polling");
  }
  lastBootButtonState = currentButtonState;

  if (bootButtonPressed) {
    Serial.print("Boot button ISR triggered (count: ");
    Serial.print(bootButtonISRCount);
    Serial.println(")");
    bootButtonPressed = false; // Clear the flag

    // Debounce the button
    if (millis() - lastButtonPress > BUTTON_DEBOUNCE_MS) {
      lastButtonPress = millis();
      Serial.println("Boot button press accepted (passed debounce)");
      
      if (displayAvailable && displayTimedOut) {
        // Turn display back on
        displayTimedOut = false;
        displayStartTime = millis(); // Reset timeout counter
        
        // Show a brief "Button Pressed" message
        displayClear();
        display->setTextSize(1);
        display->setTextColor(1);  // Use 1 for white (compatible with SSD1306 and SH1106)
        display->setCursor(0, 0);
        display->println("Boot Button Pressed");
        display->println("Display Re-enabled");
        display->println("");
        
        // Show current time if available
        struct tm timeinfo;
        if (getLocalTime(&timeinfo)) {
          char timeStr[20];
          strftime(timeStr, sizeof(timeStr), "%H:%M:%S %d/%m", &timeinfo);
          display->print("Time: ");
          display->println(timeStr);
        }
        
        display->print("WiFi: ");
        display->println(wifiConnected ? "OK" : "Disconnected");
        displayRefresh();
        
        Serial.println("OLED Display re-enabled via boot button press");
        
        // Wait a moment to show the message before resuming normal display rotation
        delay(2000);
      } else if (displayAvailable) {
        // Display is already on - advance to next screen
        displayStartTime = millis();
        
        // Get available screen count (same logic as in updateDisplay)
        // Screens: 0=Environment, 1=AirQuality, 2=Power, 3=LoRa, 4=Network, 5=System, 6=Sensors, 7=LoRaDebug
        bool showEnvironmental = (sht4xAvailable || tmp117Available || aht20Available ||
                                  bme680Available || bmp280Available || bmp390Available || ms5611Available);
        bool showAirQuality = (scd4xAvailable || scd30Available || cm1106cAvailable ||
                               sgp41Available || pms5003Available);
        bool showPowerMonitor = (ina219Available || pmuAvailable);
        bool showLoRaStatus = (HAS_LORA() && !DISABLE_LORAWAN);
        bool showNetwork = true;
        bool showSystemStatus = true;
        bool showSensorsList = true;
        bool showLoRaDebug = (HAS_LORA() && !DISABLE_LORAWAN && ENABLE_DEBUG_OUTPUT);

        int screenCount = 0;
        if (showEnvironmental) screenCount++;
        if (showAirQuality) screenCount++;
        if (showPowerMonitor) screenCount++;
        if (showLoRaStatus) screenCount++;
        if (showNetwork) screenCount++;
        if (showSystemStatus) screenCount++;
        if (showSensorsList) screenCount++;
        if (showLoRaDebug) screenCount++;
        
        // Advance to next screen
        currentDisplayScreen = (currentDisplayScreen + 1) % screenCount;

        // Show navigation bar for 2 seconds
        showNavigationBar = true;
        lastScreenSwitchTime = millis();

        Serial.print("Boot button pressed - advancing to screen ");
        Serial.print(currentDisplayScreen + 1);
        Serial.print(" of ");
        Serial.println(screenCount);
      }
    } else {
      Serial.print("Boot button press ignored (debounce: ");
      Serial.print(millis() - lastButtonPress);
      Serial.print("ms < ");
      Serial.print(BUTTON_DEBOUNCE_MS);
      Serial.println("ms)");
    }
  }

  // Update OLED display with current sensor readings
  updateDisplay();
  
  // Check and publish calibration diagnostics less frequently (every 5 seconds instead of every loop)
  if (millis() - lastDiagnosticsCheckTime >= 5000) {
    lastDiagnosticsCheckTime = millis();
    publishCalibrationDiagnostics();
  }
  
  // Handle LoRaWAN transmission (non-blocking)
  if (HAS_LORA() && !DISABLE_LORAWAN) {
    sendLoRaWANData();
  }

  // Handle WiFi reconnection (non-blocking)
  handleWiFiReconnection();
  
  // Handle MQTT (non-blocking)
  if (wifiConnected && ENABLE_MQTT) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
  }
  
  // Small delay to prevent CPU spinning when LoRaWAN is enabled
  // LoRaWAN only sends every 60 seconds, so we don't need to poll constantly
  if (HAS_LORA() && !DISABLE_LORAWAN) {
    delay(10); // 10ms delay reduces CPU usage without impacting responsiveness
  }

  if (gpsAvailable && GPSSerial.available() > 0) {
      while (GPSSerial.available() > 0) { 
        gps.encode(GPSSerial.read());
      }
  }

  // Load calibration state if time is available and we haven't loaded it yet
  // Works with both NTP (WiFi) and GPS time sources
  if (!calibrationStateLoaded && getCurrentUnixTime() > 0) {
    Serial.println("\n=== LOOP: CALIBRATION LOADING ===");
    Serial.println("Time sync now available - loading calibration state...");
    loadCalibrationState();
    calibrationStateLoaded = true;
    Serial.println("==================================\n");
  }

  // Discovery configuration (send once when MQTT connects)
  static bool discoverySent = false;
  if (ENABLE_MQTT && wifiConnected && client.connected() && !discoverySent) {
    Serial.println("\n=== LOOP: HOME ASSISTANT DISCOVERY ===");
    publishAllDiscoveryConfigs();
    discoverySent = true;
    Serial.println("========================================\n");
  }

  // Periodic time resync (every 24 hours) - GPS or NTP based on configuration
  if (millis() - lastNtpResyncTime >= NTP_RESYNC_INTERVAL_MS) {
    lastNtpResyncTime = millis();
    
    bool timeSync = false;
    
    // Try GPS time sync first if preferred (for off-grid operation)
    if (PREFER_GPS_TIME && HAS_GPS()) {
      Serial.print(getTimestampSerial()); Serial.println(" Attempting GPS time sync...");
      timeSync = syncTimeFromGPS();
      if (!timeSync) {
        Serial.print(getTimestampSerial()); Serial.println(" GPS time sync failed - no valid GPS time");
      }
    }
    
    // Fall back to NTP if GPS failed or not preferred
    if (!timeSync && wifiConnected) {
      Serial.print(getTimestampSerial()); Serial.println(" Performing periodic NTP resync...");
      configTime(gmt_offset_sec, daylight_offset_sec, ntp_server, ntp_server2);
      delay(500); // Brief pause for sync
      struct tm timeinfo;
      if (getLocalTime(&timeinfo)) {
        Serial.print(getTimestampSerial()); Serial.print(" NTP resync successful: ");
        Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
        timeSync = true;
      } else {
        Serial.print(getTimestampSerial()); Serial.println(" NTP resync failed");
      }
    }
    
    // If both failed
    if (!timeSync) {
      Serial.print(getTimestampSerial()); Serial.println(" All time sync methods failed - using internal clock");
    }
  }

  // ==============================================
  // MQTT PUBLISH CYCLE (WiFi/MQTT mode)
  // ==============================================
  // Only runs when LoRaWAN is disabled - reads sensors and publishes to MQTT
  // When LoRaWAN is enabled, MQTT publishing happens during LoRaWAN's sensor read cycle
  bool lorawanEnabled = HAS_LORA() && !DISABLE_LORAWAN;

  if (!lorawanEnabled && ENABLE_MQTT && wifiConnected) {
    if (millis() - lastMqttPublishTime >= MQTT_PUBLISH_INTERVAL_MS) {
      lastMqttPublishTime = millis();

      Serial.println(F("\n=== MQTT PUBLISH CYCLE ==="));
      Serial.println(F("Reading sensors for MQTT publish..."));

      // Read all sensors - populates currentBestReadings with per-reading timestamps
      uint32_t cycleTimestamp = readAllSensors();

      // Publish all readings as consolidated v2 protobuf message
      // WiFi version includes per-reading timestamps for accuracy
      // (LoRaWAN omits per-reading timestamps to save bandwidth)
      publishAllSensorsMQTT(cycleTimestamp);

      Serial.printf("MQTT cycle complete. Next publish in %lu seconds.\n",
                    MQTT_PUBLISH_INTERVAL_MS / 1000);

      // Log sensor cycle to syslog
      syslog.info("sensor_cycle complete heap=" + String(ESP.getFreeHeap()));
    }
  }

  // ==============================================
  // GPS STATUS PUBLISHING
  // ==============================================
  // GPS satellite count, fix status, and coordinates are published
  // with internal debouncing (every 30s or on significant change)
  if (gpsAvailable) {
    static unsigned long lastGPSPublishTime = 0;
    static uint8_t lastFixStateGPS = 255;
    static bool gpsEverAcquiredFix = false;

    uint8_t currentFixStateGPS = gps.location.isValid() ? 1 : 0;
    if (gps.location.isValid()) gpsEverAcquiredFix = true;

    bool significantGPSChange = (currentFixStateGPS != lastFixStateGPS);

    if (significantGPSChange ||
        (gpsEverAcquiredFix && (millis() - lastGPSPublishTime > 30000)) ||
        (gps.satellites.isValid() && gps.satellites.value() > 0 && (millis() - lastGPSPublishTime > 15000))) {

      lastGPSPublishTime = millis();
      lastFixStateGPS = currentFixStateGPS;

      if (ENABLE_DEBUG_OUTPUT) {
        Serial.println("\n=== GPS STATUS ===");
        if (gps.satellites.isValid()) {
          Serial.print("  Satellites: "); Serial.println(gps.satellites.value());
        }
        Serial.print("  GPS Fix: "); Serial.println(currentFixStateGPS ? "Yes" : "No");
      }

      // Publish GPS data to MQTT
      if (wifiConnected && client.connected() && ENABLE_MQTT) {
        if (gps.satellites.isValid()) {
          publishSensorData("gps_sats", (float)gps.satellites.value());
        }
        publishSensorData("gps_status", (float)currentFixStateGPS);

        if (gps.location.isValid()) {
          publishSensorData("gps_lat", gps.location.lat());
          publishSensorData("gps_lon", gps.location.lng());
          if (gps.altitude.isValid()) {
            publishSensorData("gps_alt", gps.altitude.meters());
          }
        }
      }
    }
  }
}

// Legacy sensor loop code removed - all sensor reading now handled by readAllSensors()

