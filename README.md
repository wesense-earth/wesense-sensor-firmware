# WeSense Sensor Firmware

**Turn an inexpensive ESP32 into a calibrated environmental sensor that joins the [WeSense](https://wesense.earth) decentralised air-quality network — or runs entirely standalone in Home Assistant.**

Auto-detects your board. Auto-discovers your sensors. Signs every reading. Publishes over WiFi/MQTT or LoRaWAN with no central server in the path.

**📖 Full setup, configuration, and operation guides: [docs.wesense.earth](https://docs.wesense.earth/) ➜**

---

## What you can do with it

- **Contribute to a global air-quality map** — readings flow to [map.wesense.earth](https://map.wesense.earth) and are mirrored across the WeSense P2P network. No paywall, no central API.
- **Run your own indoor or outdoor air monitor** — Home Assistant picks it up automatically via MQTT discovery, or feed it into your own MQTT broker / dashboards.
- **Deploy off-grid** — LoRaWAN with TTN integration means no WiFi required. Solar + LoRa T-Beam = a sensor you can leave anywhere.
- **Build a sensor fleet** — every reading is signed end-to-end with Ed25519, so anyone replicating your data can verify it came from your station.

## Why this firmware

### Plug-and-play hardware
Drop in any supported ESP32 board with any combination of supported sensors and it just works. The firmware probes I2C at boot, identifies what's connected, and configures itself. Add a sensor later? Just plug it in and reboot.

### Best-reading-wins sensor fusion
When two sensors measure the same thing (say, an SHT4x and an AHT20 both reporting temperature), the firmware always publishes the more accurate one. Cheap sensors act as automatic backups for premium ones without spamming the network with duplicates.

### Two transports, one configuration
Same firmware, same `.ino` file: enable WiFi/MQTT for indoor or networked deployments, or LoRaWAN for off-grid. A compact V2 protobuf payload packs all sensors into a single uplink — even on AU915's tightest data rates.

### Honest calibration
Sensors that need to settle (SCD4x CO2 needs 7 days of auto-calibration; SGP41 VOC index needs 12 hours; BME680 baselines over 2 days) are tracked individually. The firmware suppresses unreliable readings during warmup and surfaces calibration state — so you never publish data your sensor isn't confident about. Calibration state survives reboots.

### Remote control over MQTT
Calibrate a CO2 sensor, toggle auto-calibration, restart, or factory-reset over MQTT — no physical access needed. Useful when your station is on the roof.

### Home Assistant ready
MQTT auto-discovery means every sensor shows up as a proper HA entity with the right device class and icon, grouped under a single device. Zero YAML.

### Built to be trustworthy
Every reading carries an Ed25519 signature and a public key. The data is yours, and anyone can verify it without trusting WeSense (or anyone else).

## Supported hardware

### ESP32 Boards

| Board                       | LoRa       | GPS         | PMU       | Primary I2C                     | Secondary I2C            | Status      |
| --------------------------- | ---------- | ----------- | --------- | ------------------------------- | ------------------------ | ----------- |
| **ESP32 WROOM-32/DevKit**   | ❌          | ❌           | ❌         | GPIO 21/22 (D21/D22)            | GPIO 13/14 (D13/D14)     | ✅ Tested    |
| **ESP32-C3 Generic**        | ❌          | ❌           | ❌         | GPIO 4/5                        | GPIO 8/9                 | ✅ Supported |
| **ESP32-S3 Generic DevKit** | ❌          | ❌           | ❌         | GPIO 8/9                        | GPIO 17/18               | ✅ Supported |
| **DFRobot Beetle ESP32-C6** | ❌          | ❌           | ❌         | GPIO 19/20 (SDA/SCL)            | GPIO 6/7 (LP_SDA/LP_SCL) | ✅ Supported |
| **T-Beam v0.7**             | ✅ SX1276   | ✅           | ✅ AXP192  | GPIO 21/22                      | GPIO 13/14               | ✅ Supported |
| **T-Beam v1.0/1.1**         | ✅ SX1276   | ✅           | ✅ AXP2101 | GPIO 21/22                      | GPIO 13/14               | ✅ Supported |
| **T-Beam T3 S3 v1.2**       | ✅ SX1262   | ⚠️ Variant   | ✅ AXP2101 | GPIO 18/17 (hardwired OLED/PMU) | GPIO 45/46               | ✅ Supported |

### Supported Sensors

| Sensor      | Measurements                         | Interface | Address/Pins | Notes                                            |
| ----------- | ------------------------------------ | --------- | ------------ | ------------------------------------------------ |
| **SHT4x**   | Temperature, Humidity                | I2C       | 0x44         | High precision, primary T/H sensor               |
| **AHT20**   | Temperature, Humidity                | I2C       | 0x38         | Backup T/H sensor if SHT4x unavailable           |
| **BME680**  | Temperature, Humidity, Pressure, Gas | I2C       | 0x76/0x77    | 4-in-1 environmental sensor with VOC             |
| **SCD4x**   | CO2, Temperature, Humidity           | I2C       | 0x62         | NDIR CO2 sensor with auto-calibration (primary)  |
| **SCD30**   | CO2, Temperature, Humidity           | I2C       | 0x61         | NDIR CO2 sensor with auto-calibration (fallback) |
| **CM1106-C**| CO2                                  | I2C/UART  | 0x31         | NDIR CO2 sensor with ABC (24h + 15-day cycle)    |
| **SGP41**   | VOC Index, NOx Index                 | I2C       | 0x59         | Air quality with gas index algorithms            |
| **MS5611**  | Pressure, Temperature                | I2C       | 0x77         | High-precision barometric altimeter (priority 1) |
| **BMP390**  | Pressure, Temperature                | I2C       | 0x76/0x77    | High precision barometric pressure (±3 Pa)       |
| **BMP280**  | Pressure, Temperature                | I2C       | 0x76/0x77    | Barometric pressure with altitude                |
| **PMS5003** | PM1.0, PM2.5, PM10, Particles        | UART      | GPIO16/17    | Plantower laser particle sensor                  |
| **INA219**  | Voltage, Current, Power              | I2C       | 0x40-0x45    | DC power monitoring (0-26V, 3.2A)                |
| **AXP2101** | Battery, Voltage, Status             | I2C       | 0x34         | Power management (T-Beam only)                   |

## Getting started

Pick a supported board, wire up whatever sensors you have, flash the firmware, and go. Full walkthroughs live on the docs site:

- 🚀 **[Quick Start](https://docs.wesense.earth/getting-started/quick-start)** — start here
- 🔧 **[Build a WeSense Node](https://docs.wesense.earth/getting-started/build-wesense-node)** — hardware assembly and wiring
- 🛒 **[Recommended Sensors](https://docs.wesense.earth/getting-started/recommended-sensors)** — what to buy
- ⚡ **[Firmware Setup](https://docs.wesense.earth/getting-started/firmware-setup)** — Arduino IDE, libraries, flashing
- ⚙️ **[Firmware Configuration](https://docs.wesense.earth/getting-started/firmware-configuration)** — credentials, sensors, transport
- 📡 **[Managing Your Sensor](https://docs.wesense.earth/getting-started/managing-your-sensor)** — MQTT commands, calibration, remote control
- 🧰 **[Board Configurations](https://docs.wesense.earth/hardware/board-configurations)** — per-board pin maps
- 🏠 **[Home Assistant Integration](https://docs.wesense.earth/getting-started/home-assistant)** — auto-discovery setup

## Contributing

Contributions welcome — new sensors, new boards, bug fixes, docs. See the **[Contributing Code guide](https://docs.wesense.earth/developers/contributing-code)**.

## License

AGPL-3.0 — see [LICENSE](LICENSE).

## Support

- **Issues**: [GitHub Issues](https://github.com/wesense-earth/wesense-sensor-firmware/issues)
- **Discussions**: [GitHub Discussions](https://github.com/wesense-earth/wesense-sensor-firmware/discussions)
- **Documentation**: [docs.wesense.earth](https://docs.wesense.earth/)

---

**Open, decentralised, signed end-to-end.** Built so the air-quality data outlasts any single organisation.
