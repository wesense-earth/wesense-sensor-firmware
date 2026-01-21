# WeSense Sensor Firmware - Automatic Multi-Board Environmental Monitor Firmware

A comprehensive environmental monitoring solution for ESP32-based devices with automatic board detection, multi-sensor support, and global deployment capabilities.

**📖 [View Complete Documentation Wiki](https://gitea.electropositive.net/SkyTrace/SkyTrace-ESP32_SensorArray_Automatic/wiki) ➜**

## 🌟 **Key Features**

### 🔧 **Automatic Hardware Detection**

- **Multi-Board Support**: ESP32 Generic/WROOM-32, T-Beam v0.7/v1.0/v1.1, DFRobot Beetle ESP32-C6 Mini
- **Dynamic Pin Configuration**: Automatic I2C, SPI, GPS, and LoRa pin mapping per board
- **Feature Detection**: Automatically enables/disables LoRa, PMU, GPS based on detected hardware
- **Graceful Degradation**: Continues operation even if specific hardware unavailable

### 🌡️ **Comprehensive Sensor Support**

- **Temperature & Humidity**: SHT4x (primary), AHT20 (fallback), BME680 (multi-sensor)
- **CO2 Monitoring**: SCD4x (primary), SCD30 (fallback) with automatic calibration and dynamic pressure compensation
- **Air Quality**: SGP41 VOC/NOx with temperature/humidity compensation
- **Particulate Matter**: PMS5003 PM1.0/PM2.5/PM10 with particle count analysis
- **Atmospheric Pressure**: BME680, BMP390 (high precision), BMP280 with altitude correction
- **Gas Resistance**: BME680 for VOC estimation and air quality indication
- **Power Management**: AXP2101 PMU with battery monitoring (T-Beam)
- **DC Power Monitoring**: INA219 current/voltage/power monitoring with 0-26V, 3.2A capability
- **Location Services**: GPS with satellite count and fix status

### 🧠 **Intelligent Sensor Hierarchy**

- **Duplicate Detection**: Automatically identifies overlapping sensor capabilities
- **Accuracy-Based Priority**: Always publishes the most accurate reading for each measurement type
- **Smart Fallback**: Uses less accurate sensors only when higher-accuracy ones are unavailable
- **Bandwidth Optimization**: Eliminates redundant data (60-80% reduction) while maintaining sensor functionality

### 📡 **Multi-Protocol Connectivity**

- **WiFi + MQTT**: Real-time sensor data publishing with automatic reconnection
- **LoRaWAN**: Long-range communication for off-grid deployments (OTAA with TTN)
- **GPS Integration**: Location-aware sensor readings for mobile/distributed networks
- **NTP/GPS Time Sync**: Accurate timestamping with automatic daily resync

### 📦 **LoRaWAN Payload Optimization (V2 Protobuf)**

The firmware uses a compact V2 protobuf format that packs all sensor readings into a single message, dramatically reducing payload size compared to sending each sensor separately.

#### Payload Size Comparison

| Sensors            | V1 (Legacy) | V2 (Current) | Savings |
| ------------------ | ----------- | ------------ | ------- |
| 2 (temp, humidity) | ~198 bytes  | ~53 bytes    | **73%** |
| 4 sensors          | ~400 bytes  | ~71 bytes    | **82%** |
| 6 sensors          | ~600 bytes  | ~89 bytes    | **85%** |
| 10 sensors         | ~1000 bytes | ~125 bytes   | **88%** |
| 12 sensors (max)   | ~1200 bytes | ~143 bytes   | **88%** |

#### AU915 Data Rate Compatibility (Australia/NZ)

With 400ms dwell time restrictions, AU915 has the following payload limits:

| Data Rate | Spreading Factor | Max Payload | V2 Sensors That Fit | Range    |
| --------- | ---------------- | ----------- | ------------------- | -------- |
| DR2       | SF10             | 11 bytes    | Header only         | Best     |
| DR3       | SF9              | 53 bytes    | 2 sensors           | Good     |
| DR4       | SF8              | 125 bytes   | 10 sensors          | Medium   |
| DR5       | SF7              | 222 bytes   | 12+ sensors (all)   | Shortest |

**Key insight**: With V2 protobuf, even medium signal strength (DR4) can transmit **all 10 common environmental sensors** in a single uplink!

### 🏠 **Home Assistant Integration**

- **Automatic Discovery**: Zero-configuration sensor registration
- **Device Grouping**: All sensors appear under single device entity
- **Proper Device Classes**: Temperature, humidity, pressure, CO2, battery with correct icons
- **Real-time Updates**: Live sensor data with configurable intervals

### 🎛️ **Remote Command & Control**

- **MQTT Command System**: Extensible remote control via MQTT messages
- **Sensor Calibration**: Remote CO₂ sensor forced recalibration with custom ppm values
- **ASC Control**: Enable/disable SCD4x automatic calibration remotely for global deployments
- **System Management**: Remote restart, factory reset, diagnostic commands

### 🔧 **Advanced Calibration Management**

- **Multi-Sensor Calibration Tracking**: Individual calibration periods for each sensor type
- **Smart Data Suppression**: Automatically suppresses unreliable data during calibration periods
- **Calibration State Persistence**: Survives reboots - calibration progress maintained across power cycles
- **Per-Sensor Timeframes**: SCD4x (7 days), SGP41 (12 hours), BME680 (2 days), PMS5003 (1 hour)
- **Global Testing Mode**: User-controlled data suppression for sensor testing and setup

### 🔐 **Enterprise Features**

- **Secure Network Serial Monitor**: Password-protected telnet access for remote debugging
- **MQTT Authentication**: Username/password authentication with secure credentials
- **Network Resilience**: Automatic WiFi reconnection and MQTT failover
- **Watchdog Protection**: Handles sensor failures gracefully without crashes

## 🔌 **Supported Hardware**

### **ESP32 Boards**

| Board                       | LoRa     | GPS | PMU       | Primary I2C          | Secondary I2C            | Status      |
| --------------------------- | -------- | --- | --------- | -------------------- | ------------------------ | ----------- |
| **ESP32 WROOM-32/DevKit**   | ❌        | ❌   | ❌         | GPIO 21/22 (D21/D22) | GPIO 13/14 (D13/D14)     | ✅ Tested    |
| **T-Beam v0.7**             | ✅ SX1276 | ✅   | ✅ AXP192  | GPIO 21/22           | GPIO 13/14               | ✅ Supported |
| **T-Beam v1.0/1.1**         | ✅ SX1276 | ✅   | ✅ AXP2101 | GPIO 21/22           | GPIO 13/14               | ✅ Supported |
| **DFRobot Beetle ESP32-C6** | ❌        | ❌   | ❌         | GPIO 19/20 (SDA/SCL) | GPIO 6/7 (LP_SDA/LP_SCL) | ✅ Supported |

### **Supported Sensors**

| Sensor      | Measurements                         | Interface | Address/Pins | Notes                                            |
| ----------- | ------------------------------------ | --------- | ------------ | ------------------------------------------------ |
| **SHT4x**   | Temperature, Humidity                | I2C       | 0x44         | High precision, primary T/H sensor               |
| **AHT20**   | Temperature, Humidity                | I2C       | 0x38         | Backup T/H sensor if SHT4x unavailable           |
| **BME680**  | Temperature, Humidity, Pressure, Gas | I2C       | 0x76/0x77    | 4-in-1 environmental sensor with VOC             |
| **SCD4x**   | CO2, Temperature, Humidity           | I2C       | 0x62         | NDIR CO2 sensor with auto-calibration (primary)  |
| **SCD30**   | CO2, Temperature, Humidity           | I2C       | 0x61         | NDIR CO2 sensor with auto-calibration (fallback) |
| **SGP41**   | VOC Index, NOx Index                 | I2C       | 0x59         | Air quality with gas index algorithms            |
| **BMP390**  | Pressure, Temperature                | I2C       | 0x76/0x77    | High precision barometric pressure (±3 Pa)       |
| **BMP280**  | Pressure, Temperature                | I2C       | 0x76/0x77    | Barometric pressure with altitude                |
| **PMS5003** | PM1.0, PM2.5, PM10, Particles        | UART      | GPIO16/17    | Plantower laser particle sensor                  |
| **INA219**  | Voltage, Current, Power              | I2C       | 0x40-0x45    | DC power monitoring (0-26V, 3.2A)                |
| **AXP2101** | Battery, Voltage, Status             | I2C       | 0x34         | Power management (T-Beam only)                   |

## 📋 **Prerequisites**

### **Hardware Requirements**

- ESP32-based board (see supported hardware table)
- Environmental sensors (SHT4x recommended, others optional)
- PMS5003 air quality sensor (optional, requires 5V power supply)
- **Time source**: Either WiFi (for NTP) OR GPS module (for atomic time)
- WiFi network access (required for NTP time sync if no GPS)
- MQTT broker (local or cloud-based)

### **Software Requirements**

- **Arduino IDE** 2.0+ or **PlatformIO**
- **ESP32 Board Package** 2.0.0+
- **Required Libraries** (install via Library Manager)

## 🚀 **Quick Start**

### **1. Install Arduino IDE & ESP32 Support**

```bash
# Download Arduino IDE from: https://www.arduino.cc/en/software

# Add ESP32 board package URL in Arduino IDE:
# File → Preferences → Additional Board Manager URLs:
https://espressif.github.io/arduino-esp32/package_esp32_index.json

# Install ESP32 boards:
# Tools → Board → Board Manager → Search "ESP32" → Install
```

### **2. Install Required Libraries**

Open Arduino IDE and install these libraries via **Tools → Manage Libraries**:

```
RadioLib (for LoRaWAN)
SparkFun SCD4x CO2 Sensor Library
SparkFun SCD30 Arduino Library
Adafruit BMP280 Library
Adafruit BMP3XX Library (for BMP390)
Adafruit BME680 Library
Adafruit AHTX0 Library
Adafruit INA219 Library
PubSubClient (for MQTT)
TinyGPSPlus
XPowersLib (for T-Beam PMU support)
```

**Sensirion Libraries** (install manually):

- Download from [Sensirion GitHub](https://github.com/Sensirion)
- `SensirionI2CSgp41` (VOC/NOx sensor)
- `SensirionI2cSht4x` (Temperature/Humidity sensor)

### **3. Download & Configure Code**

```bash
# Clone or download this repository
git clone https://github.com/your-repo/ESP32_SensorArray_Automatic.git
cd ESP32_SensorArray_Automatic

# Open ESP32_SensorArray_Automatic.ino in Arduino IDE
```

### **4. Basic Configuration**

Edit the configuration section at the top of `ESP32_SensorArray_Automatic.ino`:

```cpp
// WiFi & MQTT Configuration
const char* wifi_ssid = "YourWiFiNetwork";
const char* wifi_password = "YourWiFiPassword";
const char* mqtt_server = "192.168.1.100";  // Your MQTT broker IP
const char* mqtt_user = "mqttuser";
const char* mqtt_password = "mqttpassword";

// Time Configuration
const long gmt_offset_sec = 43200;      // Your timezone offset (43200 = GMT+12)
const int daylight_offset_sec = 3600;   // Daylight saving (3600 = 1 hour)
```

### **5. Upload & Monitor**

1. **Select your board**: Tools → Board → ESP32 Arduino → [Your Board]
2. **Select port**: Tools → Port → [Your ESP32 Port]
3. **Upload**: Click Upload button or Ctrl+U
4. **Monitor**: Tools → Serial Monitor (115200 baud)

## 📈 **Data Output**

### **MQTT Topic Structure (v2)**

```
wesense/v2/{country}/{subdivision}/{device_id}
```

All sensor readings are consolidated into a single v2 protobuf message per publish cycle.

**Example Topic:**

- `wesense/v2/nz/auk/office_301274c0e8fc`

**Subtopics for diagnostics/commands:**

- `wesense/v2/nz/auk/office_301274c0e8fc/diagnostic/...`
- `wesense/v2/nz/auk/office_301274c0e8fc/command/...`
- `wesense/v2/nz/auk/office_301274c0e8fc/status/...`

### **Payload Format (v2 Protobuf)**

The v2 format consolidates all sensor readings into a single efficient protobuf message:

- **WiFi**: ~35 bytes header + ~12-14 bytes per sensor (includes per-reading timestamps)
- **LoRaWAN**: ~35 bytes header + ~7-9 bytes per sensor (uses header timestamp)

See `proto/README.md` for detailed schema documentation.

## 📚 **Documentation**

### **📖 [Complete Documentation Wiki ➜](https://gitea.electropositive.net/SkyTrace/SkyTrace-ESP32_SensorArray_Automatic/wiki)**

For detailed configuration, troubleshooting, and advanced features, see our comprehensive wiki:

**🚀 Getting Started:**

- **[Hardware Setup & Wiring](https://gitea.electropositive.net/SkyTrace/SkyTrace-ESP32_SensorArray_Automatic/wiki/Hardware-Setup)** - Sensor connections and I2C configuration
- **[Configuration Guide](https://gitea.electropositive.net/SkyTrace/SkyTrace-ESP32_SensorArray_Automatic/wiki/Configuration)** - Complete settings reference
- **[Air Quality Ranges](https://gitea.electropositive.net/SkyTrace/SkyTrace-ESP32_SensorArray_Automatic/wiki/Air-Quality-Ranges)** - Understanding sensor readings

**🎛️ Operation & Control:**

- **[MQTT Commands](https://gitea.electropositive.net/SkyTrace/SkyTrace-ESP32_SensorArray_Automatic/wiki/MQTT-Commands)** - Remote control and management
- **[Calibration System](https://gitea.electropositive.net/SkyTrace/SkyTrace-ESP32_SensorArray_Automatic/wiki/Calibration)** - Advanced calibration management
- **[Location Management](https://gitea.electropositive.net/SkyTrace/SkyTrace-ESP32_SensorArray_Automatic/wiki/Location-Management)** - Dynamic positioning
- **[Remote Debug](https://gitea.electropositive.net/SkyTrace/SkyTrace-ESP32_SensorArray_Automatic/wiki/Remote-Debug)** - Network serial monitor

**🔧 Support & Advanced:**

- **[Troubleshooting Guide](https://gitea.electropositive.net/SkyTrace/SkyTrace-ESP32_SensorArray_Automatic/wiki/Troubleshooting)** - Common issues and solutions
- **[Monitoring & Analytics](https://gitea.electropositive.net/SkyTrace/SkyTrace-ESP32_SensorArray_Automatic/wiki/Monitoring)** - InfluxDB, Grafana, data pipelines
- **[Global Deployment](https://gitea.electropositive.net/SkyTrace/SkyTrace-ESP32_SensorArray_Automatic/wiki/Global-Deployment)** - Enterprise-scale networks
- **[Development Guide](https://gitea.electropositive.net/SkyTrace/SkyTrace-ESP32_SensorArray_Automatic/wiki/Development)** - Contributing and extending

## 🔧 **Troubleshooting Quick Reference**

**WiFi Connection Failed**

- Check SSID and password
- Ensure 2.4GHz network (ESP32 doesn't support 5GHz)

**Sensor Not Found**  

- Check sensor wiring (SDA/SCL pins)
- Verify sensor I2C address
- Check 3.3V power supply

**PMS5003 No Data**

- Verify 5V power supply (required, not 3.3V)
- Check UART wiring: PMS5003 TX → ESP32 RX, PMS5003 RX → ESP32 TX

**For detailed troubleshooting**, see **[Troubleshooting Wiki](https://gitea.electropositive.net/SkyTrace/SkyTrace-ESP32_SensorArray_Automatic/wiki/Troubleshooting)**

## 🤝 **Contributing**

Contributions welcome! Please see our **[Development Guide](https://gitea.electropositive.net/SkyTrace/SkyTrace-ESP32_SensorArray_Automatic/wiki/Development)** for details on:

- Adding new sensors
- Adding new boards  
- Code style guidelines
- Testing procedures

## 📄 **License**

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📧 **Support**

- **Issues**: [GitHub Issues](https://github.com/your-repo/ESP32_SensorArray_Automatic/issues)
- **Discussions**: [GitHub Discussions](https://github.com/your-repo/ESP32_SensorArray_Automatic/discussions)
- **Documentation**: [Project Wiki](https://gitea.electropositive.net/SkyTrace/SkyTrace-ESP32_SensorArray_Automatic/wiki)

---

**Built for global environmental monitoring networks • Scales to thousands of devices • Production ready**