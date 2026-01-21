/*
 * Protobuf Encoder V2.3 for WeSense Homebrew Beacon
 *
 * Encodes ALL sensor readings into a single compact protobuf message.
 * Uses the v2.3 schema with:
 * - BoardType enum (replaces vendor/product_line/device_type)
 * - node_name for friendly device names
 * - node_info for physical setup descriptions (e.g., "outdoor pole, perspex case")
 * - node_info_url for links to detailed documentation
 * - altitude_cm for elevation data
 * - Split message support (WiFi: full messages, LoRa: readings + metadata)
 *
 * Transport modes:
 * - WiFi: Full SensorReadingV2 with all fields every transmission
 * - LoRa: Minimal SensorReadingV2 for readings + DeviceMetadataV2 on boot/daily
 *
 * Payload sizes:
 * - LoRa minimal (5 sensors): ~49 bytes (fits DR3)
 * - LoRa standard (8 sensors): ~70 bytes (fits DR4)
 * - WiFi full (14 sensors): ~170-330 bytes (depends on node_info usage)
 */

#ifndef PROTOBUF_ENCODER_V2_H
#define PROTOBUF_ENCODER_V2_H

#include "wesense_homebrew_v2.pb.h"
#include "board_config.h"
#include <pb_encode.h>
#include <math.h>

// Forward declarations (provided by main code)
extern uint64_t getMacAsUint64();
extern uint32_t getCurrentUnixTime();
extern double getLatitude();
extern double getLongitude();
extern float getAltitudeMeters();  // Returns altitude in meters (from GPS or config)
extern const char* getNodeName();  // Returns friendly node name (from config)
extern const char* getNodeInfo();  // Returns physical setup description (from config)
extern const char* getNodeInfoUrl();  // Returns URL to detailed documentation (from config)
extern const char* getFirmwareVersion();  // Returns firmware version string
extern BoardConfig g_boardConfig;  // Global board configuration

/**
 * Map local BoardType enum to v2.3 proto BoardType enum
 */
wesense_homebrew_v2_BoardType mapBoardTypeV2(BoardType type) {
    switch (type) {
        case BOARD_TBEAM_V07:
            return wesense_homebrew_v2_BoardType_LILYGO_T_BEAM_V0_7;
        case BOARD_TBEAM_V10:
        case BOARD_TBEAM_V11:
            return wesense_homebrew_v2_BoardType_LILYGO_T_BEAM;
        case BOARD_TBEAM_T3_S3_V12:
            return wesense_homebrew_v2_BoardType_LILYGO_T_BEAM_S3;
        case BOARD_ESP32_GENERIC:
            return wesense_homebrew_v2_BoardType_ESP32_DEVKIT;
        case BOARD_ESP32_S3_GENERIC:
            return wesense_homebrew_v2_BoardType_ESP32_S3_DEVKIT;
        case BOARD_ESP32_C3_GENERIC:
            return wesense_homebrew_v2_BoardType_ESP32_C3_DEVKIT;
        case BOARD_ESP32_C6_BEETLE:
            return wesense_homebrew_v2_BoardType_DFROBOT_ESP32_C6_BEETLE;
        default:
            return wesense_homebrew_v2_BoardType_CUSTOM_HOMEBREW;
    }
}

/**
 * Map string sensor name to v2.1 SensorModel enum
 */
wesense_homebrew_v2_SensorModel mapSensorModelV2(const char* sensorName) {
    String name = String(sensorName);
    name.toLowerCase();

    if (name.indexOf("sht4") >= 0 || name.indexOf("sht40") >= 0) {
        return wesense_homebrew_v2_SensorModel_SHT4X;
    } else if (name.indexOf("aht20") >= 0) {
        return wesense_homebrew_v2_SensorModel_AHT20;
    } else if (name.indexOf("scd4") >= 0) {
        return wesense_homebrew_v2_SensorModel_SCD4X;
    } else if (name.indexOf("scd30") >= 0) {
        return wesense_homebrew_v2_SensorModel_SCD30;
    } else if (name.indexOf("bmp390") >= 0) {
        return wesense_homebrew_v2_SensorModel_BMP390;
    } else if (name.indexOf("bmp280") >= 0) {
        return wesense_homebrew_v2_SensorModel_BMP280;
    } else if (name.indexOf("bme280") >= 0) {
        return wesense_homebrew_v2_SensorModel_BME280;
    } else if (name.indexOf("bme680") >= 0) {
        return wesense_homebrew_v2_SensorModel_BME680;
    } else if (name.indexOf("sgp41") >= 0) {
        return wesense_homebrew_v2_SensorModel_SGP41;
    } else if (name.indexOf("pms5003") >= 0 || name.indexOf("pms") >= 0) {
        return wesense_homebrew_v2_SensorModel_PMS5003;
    } else if (name.indexOf("ina219") >= 0) {
        return wesense_homebrew_v2_SensorModel_INA219;
    } else if (name.indexOf("cm1106") >= 0) {
        return wesense_homebrew_v2_SensorModel_CM1106C;
    } else if (name.indexOf("tmp117") >= 0) {
        return wesense_homebrew_v2_SensorModel_TMP117;
    } else if (name.indexOf("axp") >= 0) {
        return wesense_homebrew_v2_SensorModel_AXP2101;
    }
    return wesense_homebrew_v2_SensorModel_SENSOR_UNKNOWN;
}

/**
 * Map deployment type enum to v2 enum
 */
wesense_homebrew_v2_DeploymentType mapDeploymentTypeV2(DeploymentType type) {
    switch (type) {
        case INDOOR:  return wesense_homebrew_v2_DeploymentType_INDOOR;
        case OUTDOOR: return wesense_homebrew_v2_DeploymentType_OUTDOOR;
        case MIXED:   return wesense_homebrew_v2_DeploymentType_MIXED;
        default:      return wesense_homebrew_v2_DeploymentType_DEPLOYMENT_UNKNOWN;
    }
}

/**
 * Transport mode for the encoder
 * Determines which fields are included in the message
 */
enum TransportMode {
    TRANSPORT_WIFI,   // Full message with all fields
    TRANSPORT_LORA    // Minimal message (device_id, timestamp, measurements only)
};

/**
 * Round sensor value to appropriate decimal places based on reading type.
 * Precision is matched to sensor hardware resolution to reduce noise.
 *
 * Precision by type:
 * - Temperature: 2 decimals (0.01°C - matches BMP/SHT sensor resolution)
 * - Pressure: 2 decimals (0.01 hPa - matches BMP280/BMP390/BME680 resolution)
 * - Humidity: 1 decimal (0.1% - sensor accuracy is ±1.8-3% anyway)
 * - CO2: integer (1 ppm resolution)
 * - VOC/NOx indices: integer (index values)
 * - PM readings: 1 decimal (0.1 µg/m³ resolution)
 * - Particle counts: integer (count values)
 * - Voltage/Current/Power: 2 decimals
 * - Battery: integer (percentage)
 */
float roundSensorValue(wesense_homebrew_v2_ReadingType readingType, float value) {
    int decimals;

    switch (readingType) {
        // 2 decimal places
        case wesense_homebrew_v2_ReadingType_TEMPERATURE:
        case wesense_homebrew_v2_ReadingType_PRESSURE:  // BMP280/BMP390/BME680: 0.01 hPa resolution
        case wesense_homebrew_v2_ReadingType_VOLTAGE:
        case wesense_homebrew_v2_ReadingType_CURRENT:
        case wesense_homebrew_v2_ReadingType_POWER:
        case wesense_homebrew_v2_ReadingType_GAS_RESISTANCE:
            decimals = 2;
            break;

        // 1 decimal place
        case wesense_homebrew_v2_ReadingType_HUMIDITY:
        case wesense_homebrew_v2_ReadingType_PM1:
        case wesense_homebrew_v2_ReadingType_PM2_5:
        case wesense_homebrew_v2_ReadingType_PM10:
            decimals = 1;
            break;

        // Integer values (0 decimal places)
        case wesense_homebrew_v2_ReadingType_CO2:
        case wesense_homebrew_v2_ReadingType_VOC_INDEX:
        case wesense_homebrew_v2_ReadingType_NOX_INDEX:
        case wesense_homebrew_v2_ReadingType_BATTERY_LEVEL:
        case wesense_homebrew_v2_ReadingType_PC_03UM:
        case wesense_homebrew_v2_ReadingType_PC_05UM:
        case wesense_homebrew_v2_ReadingType_PC_10UM:
        case wesense_homebrew_v2_ReadingType_PC_25UM:
        case wesense_homebrew_v2_ReadingType_PC_50UM:
        case wesense_homebrew_v2_ReadingType_PC_100UM:
            decimals = 0;
            break;

        // Default: 2 decimal places for unknown types
        default:
            decimals = 2;
            break;
    }

    float multiplier = pow(10, decimals);
    return round(value * multiplier) / multiplier;
}

/**
 * V2.1 Consolidated Sensor Reading Encoder
 *
 * Encodes ALL sensor readings into a single protobuf message.
 * Transport-aware: WiFi sends all fields, LoRa sends minimal fields.
 *
 * Supports two transport modes:
 * - TRANSPORT_WIFI: All fields (location, board_type, node_name, altitude)
 * - TRANSPORT_LORA: Minimal fields (device_id, timestamp, measurements only)
 *
 * Note: v2.1 removed per-reading timestamps - all readings use the header timestamp.
 *       The timestamp parameter in helper methods is kept for API compatibility but ignored.
 */
class ProtobufEncoderV2 {
public:
    ProtobufEncoderV2() : measurementCount(0), transportMode(TRANSPORT_WIFI) {}

    /**
     * Initialize a new message with device metadata
     * Call this once before adding measurements
     *
     * @param cycleTimestamp Optional timestamp from sensor read cycle.
     *                       If 0, uses getCurrentUnixTime().
     * @param unused Ignored in v2.1 (kept for API compatibility)
     * @param mode Transport mode (TRANSPORT_WIFI or TRANSPORT_LORA)
     */
    void begin(uint32_t cycleTimestamp = 0, bool unused = false,
               TransportMode mode = TRANSPORT_WIFI) {
        (void)unused;  // Suppress unused parameter warning
        transportMode = mode;
        msg = wesense_homebrew_v2_SensorReadingV2_init_zero;
        measurementCount = 0;

        // === REQUIRED FIELDS (always sent) ===
        msg.device_id = getMacAsUint64();
        msg.timestamp = (cycleTimestamp > 0) ? cycleTimestamp : getCurrentUnixTime();

        // === WIFI-ONLY FIELDS (omit for LoRa to save bandwidth) ===
        // LoRa devices send DeviceMetadataV2 separately for caching
        if (transportMode == TRANSPORT_WIFI) {
            // Location (apply GPS privacy if configured)
            double lat = getLatitude();
            double lon = getLongitude();

            #ifdef GPS_PRIVACY_MODE
            switch (GPS_PRIVACY_MODE) {
                case GPS_PRIVACY_REDUCED:
                    lat = round(lat * 100.0) / 100.0;
                    lon = round(lon * 100.0) / 100.0;
                    break;
                case GPS_PRIVACY_OBFUSCATED:
                    lat += (random(-100, 100) / 100000.0);
                    lon += (random(-100, 100) / 100000.0);
                    break;
            }
            #endif

            msg.latitude_e5 = (int32_t)(lat * 100000);
            msg.longitude_e5 = (int32_t)(lon * 100000);

            // Altitude in centimeters (v2.1)
            float altMeters = getAltitudeMeters();
            msg.altitude_cm = (int32_t)(altMeters * 100);

            // Board type (v2.1 - replaces vendor/product_line/device_type)
            msg.board_type = mapBoardTypeV2(g_boardConfig.type);

            // Deployment context
            msg.deployment_type = mapDeploymentTypeV2(DEPLOYMENT_TYPE);

            // Node name (v2.1)
            const char* nodeName = getNodeName();
            if (nodeName && strlen(nodeName) > 0) {
                strncpy(msg.node_name, nodeName, sizeof(msg.node_name) - 1);
                msg.node_name[sizeof(msg.node_name) - 1] = '\0';
            }

            // Node info - physical setup description (v2.2)
            const char* nodeInfo = getNodeInfo();
            if (nodeInfo && strlen(nodeInfo) > 0) {
                strncpy(msg.node_info, nodeInfo, sizeof(msg.node_info) - 1);
                msg.node_info[sizeof(msg.node_info) - 1] = '\0';
            }

            // Node info URL - link to detailed documentation (v2.2)
            const char* nodeInfoUrl = getNodeInfoUrl();
            if (nodeInfoUrl && strlen(nodeInfoUrl) > 0) {
                strncpy(msg.node_info_url, nodeInfoUrl, sizeof(msg.node_info_url) - 1);
                msg.node_info_url[sizeof(msg.node_info_url) - 1] = '\0';
            }

            // Deprecated fields (for backward compatibility with older ingesters)
            msg.vendor = wesense_homebrew_v2_Vendor_WESENSE;
            msg.product_line = wesense_homebrew_v2_ProductLine_HOMEBREW;
            msg.device_type = wesense_homebrew_v2_DeviceType_BEACON;
        }
        // LoRa mode: only device_id, timestamp, and measurements are sent
        // Metadata is sent separately via DeviceMetadataV2 on boot and periodically
    }

    /**
     * Add a sensor measurement to the message
     *
     * @param readingType Type of reading (use wesense_homebrew_v2_ReadingType_*)
     * @param value The measurement value
     * @param sensorName Name of sensor for model detection (e.g., "sht4x")
     * @param timestamp Ignored in v2.1 (kept for API compatibility)
     * @return true if added successfully, false if message is full
     */
    bool addMeasurement(wesense_homebrew_v2_ReadingType readingType,
                        float value,
                        const char* sensorName,
                        uint32_t timestamp = 0) {
        if (measurementCount >= 14) {
            return false;  // Max measurements reached
        }

        msg.measurements[measurementCount].reading_type = readingType;
        msg.measurements[measurementCount].value = roundSensorValue(readingType, value);
        msg.measurements[measurementCount].sensor_model = mapSensorModelV2(sensorName);
        // v2.1: calibration status can be set per-reading if needed
        // For now, leave as default (CALIBRATION_UNKNOWN)

        measurementCount++;
        msg.measurements_count = measurementCount;

        return true;
    }

    // === Core sensor readings ===

    bool addTemperature(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_TEMPERATURE, value, sensorName, timestamp);
    }

    bool addHumidity(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_HUMIDITY, value, sensorName, timestamp);
    }

    bool addCO2(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_CO2, value, sensorName, timestamp);
    }

    bool addPressure(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_PRESSURE, value, sensorName, timestamp);
    }

    // === PM readings ===

    bool addPM1(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_PM1, value, sensorName, timestamp);
    }

    bool addPM25(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_PM2_5, value, sensorName, timestamp);
    }

    bool addPM10(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_PM10, value, sensorName, timestamp);
    }

    // === Extended particle counts (WiFi only, v2.1) ===

    bool addPC03um(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_PC_03UM, value, sensorName, timestamp);
    }

    bool addPC05um(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_PC_05UM, value, sensorName, timestamp);
    }

    bool addPC10um(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_PC_10UM, value, sensorName, timestamp);
    }

    bool addPC25um(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_PC_25UM, value, sensorName, timestamp);
    }

    bool addPC50um(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_PC_50UM, value, sensorName, timestamp);
    }

    bool addPC100um(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_PC_100UM, value, sensorName, timestamp);
    }

    // === Air quality indices ===

    bool addVOC(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_VOC_INDEX, value, sensorName, timestamp);
    }

    bool addNOx(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_NOX_INDEX, value, sensorName, timestamp);
    }

    // === Power monitoring ===

    bool addVoltage(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_VOLTAGE, value, sensorName, timestamp);
    }

    bool addCurrent(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_CURRENT, value, sensorName, timestamp);
    }

    bool addPower(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_POWER, value, sensorName, timestamp);
    }

    bool addBattery(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_BATTERY_LEVEL, value, sensorName, timestamp);
    }

    bool addGasResistance(float value, const char* sensorName, uint32_t timestamp = 0) {
        return addMeasurement(wesense_homebrew_v2_ReadingType_GAS_RESISTANCE, value, sensorName, timestamp);
    }

    /**
     * Encode the message to binary protobuf format
     *
     * @param buffer Output buffer for encoded data
     * @param bufferSize Size of output buffer
     * @return Number of bytes encoded, or 0 on error
     */
    size_t encode(uint8_t* buffer, size_t bufferSize) {
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, bufferSize);

        bool status = pb_encode(&stream, wesense_homebrew_v2_SensorReadingV2_fields, &msg);

        if (!status) {
            Serial.printf("Protobuf V2.1 encoding failed: %s\n", PB_GET_ERROR(&stream));
            return 0;
        }

        return stream.bytes_written;
    }

    uint8_t getMeasurementCount() const { return measurementCount; }

    TransportMode getTransportMode() const { return transportMode; }

    /**
     * Estimate encoded size (for checking against max payload)
     * v2.1: ~50 bytes header (WiFi) or ~15 bytes (LoRa) + ~8 bytes per measurement
     */
    size_t estimateSize() const {
        size_t headerSize = (transportMode == TRANSPORT_WIFI) ? 50 : 15;
        size_t perMeasurement = 8;  // v2.1: no per-reading timestamps
        return headerSize + (measurementCount * perMeasurement);
    }

private:
    wesense_homebrew_v2_SensorReadingV2 msg;
    uint8_t measurementCount;
    TransportMode transportMode;
};


/**
 * DeviceMetadataV2 Encoder (v2.1)
 *
 * For LoRa sensors: encodes device metadata sent on boot and daily.
 * This allows LoRa readings to be minimal while ingesters cache the metadata.
 */
class DeviceMetadataEncoderV2 {
public:
    DeviceMetadataEncoderV2() : calibrationCount(0) {}

    /**
     * Initialize the metadata message
     */
    void begin() {
        msg = wesense_homebrew_v2_DeviceMetadataV2_init_zero;
        calibrationCount = 0;

        // Device identification
        msg.device_id = getMacAsUint64();
        msg.timestamp = getCurrentUnixTime();

        // Location
        msg.latitude_e5 = (int32_t)(getLatitude() * 100000);
        msg.longitude_e5 = (int32_t)(getLongitude() * 100000);
        msg.altitude_cm = (int32_t)(getAltitudeMeters() * 100);

        // Device info
        msg.board_type = mapBoardTypeV2(g_boardConfig.type);
        msg.deployment_type = mapDeploymentTypeV2(DEPLOYMENT_TYPE);

        // Node name
        const char* nodeName = getNodeName();
        if (nodeName && strlen(nodeName) > 0) {
            strncpy(msg.node_name, nodeName, sizeof(msg.node_name) - 1);
            msg.node_name[sizeof(msg.node_name) - 1] = '\0';
        }

        // Firmware version
        const char* fwVersion = getFirmwareVersion();
        if (fwVersion && strlen(fwVersion) > 0) {
            strncpy(msg.firmware_version, fwVersion, sizeof(msg.firmware_version) - 1);
            msg.firmware_version[sizeof(msg.firmware_version) - 1] = '\0';
        }

        // Node info - physical setup description (v2.2)
        const char* nodeInfo = getNodeInfo();
        if (nodeInfo && strlen(nodeInfo) > 0) {
            strncpy(msg.node_info, nodeInfo, sizeof(msg.node_info) - 1);
            msg.node_info[sizeof(msg.node_info) - 1] = '\0';
        }

        // Node info URL - link to detailed documentation (v2.2)
        const char* nodeInfoUrl = getNodeInfoUrl();
        if (nodeInfoUrl && strlen(nodeInfoUrl) > 0) {
            strncpy(msg.node_info_url, nodeInfoUrl, sizeof(msg.node_info_url) - 1);
            msg.node_info_url[sizeof(msg.node_info_url) - 1] = '\0';
        }
    }

    /**
     * Add calibration info for a sensor
     */
    bool addCalibration(wesense_homebrew_v2_SensorModel sensor,
                        wesense_homebrew_v2_CalibrationStatus status,
                        wesense_homebrew_v2_CalibrationMethod method,
                        uint32_t calibrationDate = 0) {
        if (calibrationCount >= 10) {
            return false;
        }

        msg.calibrations[calibrationCount].sensor_model = sensor;
        msg.calibrations[calibrationCount].status = status;
        msg.calibrations[calibrationCount].method = method;
        msg.calibrations[calibrationCount].calibration_date = calibrationDate;

        calibrationCount++;
        msg.calibrations_count = calibrationCount;

        return true;
    }

    /**
     * Encode the metadata message
     */
    size_t encode(uint8_t* buffer, size_t bufferSize) {
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, bufferSize);

        bool status = pb_encode(&stream, wesense_homebrew_v2_DeviceMetadataV2_fields, &msg);

        if (!status) {
            Serial.printf("DeviceMetadataV2 encoding failed: %s\n", PB_GET_ERROR(&stream));
            return 0;
        }

        return stream.bytes_written;
    }

private:
    wesense_homebrew_v2_DeviceMetadataV2 msg;
    uint8_t calibrationCount;
};

#endif // PROTOBUF_ENCODER_V2_H
