#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "esp_system.h"
#include "esp_chip_info.h"

// Board type enumeration
enum BoardType {
    BOARD_UNKNOWN = 0,
    BOARD_ESP32_GENERIC,
    BOARD_ESP32_C3_GENERIC,
    BOARD_ESP32_C6_BEETLE,
    BOARD_ESP32_S3_GENERIC,
    BOARD_TBEAM_V07,
    BOARD_TBEAM_V10,
    BOARD_TBEAM_V11,
    BOARD_TBEAM_T3_S3_V12
};

// Board configuration structure
struct BoardConfig {
    BoardType type;
    const char* name;
    bool hasLoRa;
    bool hasPMU;
    bool hasGPS;
    bool hasDualI2C;  // Whether board supports two I2C controllers
    
    // I2C pins - Primary bus
    int sda_pin;
    int scl_pin;
    
    // Secondary I2C pins (for sensor isolation)
    int sda_pin2;
    int scl_pin2;
    
    // LoRa pins (if present)
    int lora_nss;
    int lora_rst;
    int lora_dio0;
    int lora_dio1;
    int lora_dio2;
    int lora_busy;   // SX1262 BUSY pin
    int lora_sck;
    int lora_miso;
    int lora_mosi;
    
    // GPS pins (if present)
    int gps_rx;
    int gps_tx;
    
    // PMS5003 pins (for detection if connected)
    int pms5003_rx;
    int pms5003_tx;
    
    // PMU I2C address (if present)
    uint8_t pmu_address;
    
    // Built-in LED pin (if present)
    int led_pin;
};

// Forward declaration for user override
extern const bool FORCE_TBEAM_T3_S3_V12;

// Board detection function
BoardType detectBoard() {
    // Check for manual override first
    #ifdef CONFIG_IDF_TARGET_ESP32S3
    if (FORCE_TBEAM_T3_S3_V12) {
        Serial.println("🔧 Manual override: Forcing T-Beam T3 S3 v1.2 configuration");
        return BOARD_TBEAM_T3_S3_V12;
    }
    #endif
    // Method 1: Check for ESP32-C6 compile target
#ifdef CONFIG_IDF_TARGET_ESP32C6
    // If compiling for ESP32-C6, assume it's the Beetle board
    return BOARD_ESP32_C6_BEETLE;
#endif

    // Method 2: Check for ESP32-C3 compile target
#ifdef CONFIG_IDF_TARGET_ESP32C3
    // If compiling for ESP32-C3, assume it's generic C3
    return BOARD_ESP32_C3_GENERIC;
#endif

    // Method 2.5: Check for ESP32-S3 compile target
#ifdef CONFIG_IDF_TARGET_ESP32S3
    Serial.println("🔍 ESP32-S3 detected - checking for T-Beam T3 S3 v1.2...");
    
    // Try T-Beam S3 pins first
    Serial.println("🔍 Testing T-Beam S3 I2C pins (SDA=18, SCL=17)...");
    Wire.begin(18, 17, 100000); // Start with T-Beam S3 pins
    delay(200); // Longer delay for PMU power stabilization
    
    // Check for AXP2101 PMU (T-Beam S3 indicator) with multiple attempts
    bool hasPMU_S3 = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        Wire.beginTransmission(0x34); // AXP2101 address
        uint8_t error = Wire.endTransmission();
        if (error == 0) {
            hasPMU_S3 = true;
            Serial.println("✅ AXP2101 PMU detected at 0x34 - this is a T-Beam T3 S3 v1.2!");
            break;
        } else {
            Serial.print("❌ PMU detection attempt "); Serial.print(attempt + 1);
            Serial.print("/3 failed (error: "); Serial.print(error); Serial.println(")");
            delay(100);
        }
    }
    
    if (hasPMU_S3) {
        return BOARD_TBEAM_T3_S3_V12; // T-Beam T3 S3 1.2
    }

    // PMU not found - try detecting display at 0x3C as alternative T-Beam indicator
    Serial.println("🔍 PMU not responding - checking for display on T-Beam pins...");
    bool hasDisplay_S3 = false;
    for (uint8_t addr : {0x3C, 0x3D}) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            hasDisplay_S3 = true;
            Serial.print("✅ Display detected at 0x");
            Serial.print(addr, HEX);
            Serial.println(" on T-Beam pins - this is a T-Beam T3 S3 v1.2!");
            break;
        }
    }

    Wire.end();  // Release I2C before potential reconfiguration

    if (hasDisplay_S3) {
        return BOARD_TBEAM_T3_S3_V12;
    }

    Serial.println("🔍 No T-Beam T3 S3 indicators found, configuring as generic ESP32-S3...");

    // Try common generic S3 pins
    Wire.begin(8, 9, 100000); // Try common generic S3 I2C pins
    delay(100);
    
    // Also check if there might be a T-Beam with different I2C pins
    Serial.println("🔍 Final check: scanning for any I2C devices...");
    bool foundAnyDevice = false;
    for (int addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("📡 Found I2C device at 0x"); 
            if (addr < 16) Serial.print("0");
            Serial.println(addr, HEX);
            foundAnyDevice = true;
            
            // If we find the PMU at the expected address, it might still be T-Beam
            if (addr == 0x34) {
                Serial.println("🤔 Found AXP2101 PMU at 0x34 - might be T-Beam with different I2C pins");
                Serial.println("⚠️  Treating as generic S3 for now, but check your T-Beam variant");
            }
        }
    }
    
    if (!foundAnyDevice) {
        Serial.println("⚠️  No I2C devices found - I2C bus may need different pins");
    }

    // Clean up I2C before returning - main firmware will reinitialize
    Wire.end();

    Serial.println("📋 Configuring as Generic ESP32-S3 Development Board");
    return BOARD_ESP32_S3_GENERIC; // Generic ESP32-S3 development board
#endif
    
    // Method 3: Try to detect T-Beam based on I2C devices
    Wire.begin(21, 22, 100000); // Start with common T-Beam pins
    delay(100);
    
    // Check for AXP2101 PMU (T-Beam indicator)
    Wire.beginTransmission(0x34); // AXP2101 address
    bool hasPMU = (Wire.endTransmission() == 0);
    
    // Check for common T-Beam sensors at expected addresses
    Wire.beginTransmission(0x77); // BMP280
    bool hasBMP = (Wire.endTransmission() == 0);
    
    Wire.beginTransmission(0x62); // SCD4x
    bool hasSCD = (Wire.endTransmission() == 0);
    
    if (hasPMU) {
        return BOARD_TBEAM_V10; // Assume v1.0 if PMU detected
    } else if (hasBMP || hasSCD) {
        return BOARD_ESP32_GENERIC; // Has sensors but no PMU
    } else {
        return BOARD_ESP32_GENERIC; // Default fallback
    }
}

// Get board configuration
BoardConfig getBoardConfig(BoardType type = BOARD_UNKNOWN) {
    if (type == BOARD_UNKNOWN) {
        type = detectBoard();
    }
    
    BoardConfig config = {};
    
    switch (type) {
        case BOARD_TBEAM_V10:
        case BOARD_TBEAM_V11:
            config.type = type;
            config.name = "T-Beam v1.0/1.1";
            config.hasLoRa = true;   // Built-in SX1276 LoRa module
            config.hasPMU = true;    // Built-in AXP2101 PMU
            config.hasGPS = true;    // Built-in GPS module
            config.hasDualI2C = true; // ESP32 classic supports dual I2C
            config.sda_pin = 21;
            config.scl_pin = 22;
            config.sda_pin2 = 13;  // Alternative I2C bus
            config.scl_pin2 = 14;
            config.lora_nss = 18;
            config.lora_rst = 23;
            config.lora_dio0 = 26;
            config.lora_dio1 = 33;
            config.lora_dio2 = 32;
            config.lora_sck = 5;
            config.lora_miso = 19;
            config.lora_mosi = 27;
            config.gps_rx = 34;
            config.gps_tx = 12;
            config.pms5003_rx = 16; // GPIO16 for PMS5003 TX -> ESP32 RX
            config.pms5003_tx = 17; // GPIO17 for PMS5003 RX -> ESP32 TX
            config.pmu_address = 0x34;
            config.led_pin = 4;  // T-Beam v1.0/1.1 built-in LED
            break;
            
        case BOARD_TBEAM_V07:
            config.type = type;
            config.name = "T-Beam v0.7";
            config.hasLoRa = true;   // Built-in SX1276 LoRa module
            config.hasPMU = true;    // Built-in AXP192 PMU
            config.hasGPS = true;    // Built-in GPS module
            config.hasDualI2C = true; // ESP32 classic supports dual I2C
            config.sda_pin = 21;
            config.scl_pin = 22;
            config.sda_pin2 = 13;  // Alternative I2C bus
            config.scl_pin2 = 14;
            config.lora_nss = 18;
            config.lora_rst = 23;
            config.lora_dio0 = 26;
            config.lora_dio1 = 33;
            config.lora_dio2 = 32;
            config.lora_sck = 5;
            config.lora_miso = 19;
            config.lora_mosi = 27;
            config.gps_rx = 34;
            config.gps_tx = 12;
            config.pms5003_rx = 16; // GPIO16 for PMS5003 TX -> ESP32 RX
            config.pms5003_tx = 17; // GPIO17 for PMS5003 RX -> ESP32 TX
            config.pmu_address = 0x35; // Different PMU address
            config.led_pin = 4;  // T-Beam v0.7 built-in LED
            break;
            
        case BOARD_TBEAM_T3_S3_V12:
            config.type = type;
            config.name = "T-Beam T3 S3 v1.2";
            config.hasLoRa = true;   // Built-in SX1262 LoRa module
            config.hasPMU = true;    // Built-in AXP2101 PMU
            config.hasGPS = false;   // No built-in GPS module (variant dependent)
            config.hasDualI2C = true; // ESP32-S3 supports dual I2C
            config.sda_pin = 18;  // T-Beam S3 I2C SDA (OLED/PMU) - hardwired, not accessible
            config.scl_pin = 17;  // T-Beam S3 I2C SCL (OLED/PMU) - hardwired, not accessible
            config.sda_pin2 = 45; // GPIO 45 (SDA) - free adjacent pin for secondary I2C bus
            config.scl_pin2 = 46; // GPIO 46 (SCL) - free adjacent pin for secondary I2C bus
            // LoRa pins from user's diagram:
            config.lora_nss = 7;     // CS from diagram
            config.lora_rst = 8;     // RST from diagram
            config.lora_dio0 = -1;   // SX1262 doesn't use DIO0
            config.lora_dio1 = 33;   // DIO from diagram
            config.lora_dio2 = -1;   // SX1262 doesn't use DIO2
            config.lora_busy = 34;   // BUSY pin for SX1262
            config.lora_sck = 5;     // SCLK from diagram
            config.lora_miso = 3;    // MISO from diagram
            config.lora_mosi = 6;    // MOSI from diagram
            // GPS pins unused (no GPS module)
            config.gps_rx = -1;      // No GPS module
            config.gps_tx = -1;      // No GPS module
            config.pms5003_rx = 16;  // GPIO16 for PMS5003 TX -> ESP32-S3 RX
            config.pms5003_tx = 15;  // GPIO15 for PMS5003 RX -> ESP32-S3 TX
            config.pmu_address = 0x34; // AXP2101 address
            config.led_pin = 4;      // T-Beam S3 built-in LED
            break;
            
        case BOARD_ESP32_C3_GENERIC:
            config.type = BOARD_ESP32_C3_GENERIC;
            config.name = "ESP32-C3 Generic";
            config.hasLoRa = false;  // No built-in LoRa module
            config.hasPMU = false;   // No built-in PMU
            config.hasGPS = false;   // No built-in GPS
            config.hasDualI2C = false; // ESP32-C3 has only one I2C controller
            // Common ESP32-C3 I2C pins (try different combinations)
            config.sda_pin = 4;   // GPIO4 (very common C3 SDA)
            config.scl_pin = 5;   // GPIO5 (very common C3 SCL)
            config.sda_pin2 = 8;  // GPIO8 (alternative SDA)
            config.scl_pin2 = 9;  // GPIO9 (alternative SCL)
            // LoRa pins unused
            config.lora_nss = -1;
            config.lora_rst = -1;
            config.lora_dio0 = -1;
            config.lora_dio1 = -1;
            config.lora_dio2 = -1;
            config.lora_sck = -1;
            config.lora_miso = -1;
            config.lora_mosi = -1;
            // GPS pins unused
            config.gps_rx = -1;
            config.gps_tx = -1;
            // PMS5003 pins (using UART pins)
            config.pms5003_rx = 20; // GPIO20 (UART1 RX) for PMS5003 TX -> ESP32-C3 RX
            config.pms5003_tx = 21; // GPIO21 (UART1 TX) for PMS5003 RX -> ESP32-C3 TX
            config.pmu_address = 0x00;
            config.led_pin = 8;  // ESP32-C3 commonly uses GPIO8 for built-in LED
            break;
            
        case BOARD_ESP32_C6_BEETLE:
            config.type = BOARD_ESP32_C6_BEETLE;
            config.name = "DFRobot Beetle ESP32-C6 Mini";
            config.hasLoRa = false;  // No built-in LoRa module
            config.hasPMU = false;   // No built-in PMU
            config.hasGPS = false;   // No built-in GPS
            config.hasDualI2C = true; // ESP32-C6 supports dual I2C
            config.sda_pin = 19;  // ESP32-C6 main SDA pin (labeled)
            config.scl_pin = 20;  // ESP32-C6 main SCL pin (labeled)
            config.sda_pin2 = 6;  // ESP32-C6 low-power SDA pin (labeled)
            config.scl_pin2 = 7;  // ESP32-C6 low-power SCL pin (labeled)
            // LoRa pins unused
            config.lora_nss = -1;
            config.lora_rst = -1;
            config.lora_dio0 = -1;
            config.lora_dio1 = -1;
            config.lora_dio2 = -1;
            config.lora_sck = -1;
            config.lora_miso = -1;
            config.lora_mosi = -1;
            // GPS pins unused
            config.gps_rx = -1;
            config.gps_tx = -1;
            // PMS5003 pins (using labeled UART pins)
            config.pms5003_rx = 17; // GPIO17 RX (labeled) for PMS5003 TX -> ESP32-C6 RX
            config.pms5003_tx = 16; // GPIO16 TX (labeled) for PMS5003 RX -> ESP32-C6 TX
            config.pmu_address = 0x00;
            config.led_pin = 15; // ESP32-C6 Beetle built-in LED
            break;
            
        case BOARD_ESP32_S3_GENERIC:
            config.type = BOARD_ESP32_S3_GENERIC;
            config.name = "ESP32-S3 Generic Development Board";
            config.hasLoRa = false;  // No built-in LoRa module
            config.hasPMU = false;   // No built-in PMU
            config.hasGPS = false;   // No built-in GPS
            config.hasDualI2C = true; // ESP32-S3 supports dual I2C
            config.sda_pin = 8;   // GPIO8 - common S3 SDA pin
            config.scl_pin = 9;   // GPIO9 - common S3 SCL pin
            config.sda_pin2 = 17; // GPIO17 - alternative I2C SDA
            config.scl_pin2 = 18; // GPIO18 - alternative I2C SCL
            // LoRa pins unused
            config.lora_nss = -1;
            config.lora_rst = -1;
            config.lora_dio0 = -1;
            config.lora_dio1 = -1;
            config.lora_dio2 = -1;
            config.lora_sck = -1;
            config.lora_miso = -1;
            config.lora_mosi = -1;
            // GPS pins unused
            config.gps_rx = -1;
            config.gps_tx = -1;
            // PMS5003 pins (using UART pins)
            config.pms5003_rx = 44; // GPIO44 (RX0) for PMS5003 TX -> ESP32-S3 RX
            config.pms5003_tx = 43; // GPIO43 (TX0) for PMS5003 RX -> ESP32-S3 TX
            config.pmu_address = 0x00;
            config.led_pin = 48;  // GPIO48 - common built-in LED on S3 boards
            break;
            
        case BOARD_ESP32_GENERIC:
        default:
            config.type = BOARD_ESP32_GENERIC;
            config.name = "ESP32 Generic/WROOM-32";
            config.hasLoRa = false;  // No built-in LoRa module
            config.hasPMU = false;   // No built-in PMU
            config.hasGPS = false;   // No built-in GPS
            config.hasDualI2C = true; // ESP32 classic supports dual I2C
            config.sda_pin = 21;  // D21 (standard I2C SDA)
            config.scl_pin = 22;  // D22 (standard I2C SCL)
            config.sda_pin2 = 13; // D13 (alternative I2C SDA)
            config.scl_pin2 = 14; // D14 (alternative I2C SCL)
            // LoRa pins unused
            config.lora_nss = -1;
            config.lora_rst = -1;
            config.lora_dio0 = -1;
            config.lora_dio1 = -1;
            config.lora_dio2 = -1;
            config.lora_sck = -1;
            config.lora_miso = -1;
            config.lora_mosi = -1;
            // GPS pins unused
            config.gps_rx = -1;
            config.gps_tx = -1;
            // PMS5003 pins - this devkit uses same-to-same labeling (RX->RX, TX->TX)
            config.pms5003_rx = 17; // GPIO17 (RX2) connects to PMS5003 RX 
            config.pms5003_tx = 16; // GPIO16 (TX2) connects to PMS5003 TX
            config.pmu_address = 0x00;
            config.led_pin = 2;  // ESP32 Generic/WROOM-32 built-in LED
            break;
    }
    
    return config;
}

// Global board configuration (set once in setup)
extern BoardConfig g_boardConfig;

// Convenience macros
#define HAS_LORA() (g_boardConfig.hasLoRa)
#define HAS_PMU() (g_boardConfig.hasPMU)
#define HAS_GPS() (g_boardConfig.hasGPS)
#define HAS_DUAL_I2C() (g_boardConfig.hasDualI2C)
// PMS5003 detection is now purely based on UART communication test

#endif // BOARD_CONFIG_H