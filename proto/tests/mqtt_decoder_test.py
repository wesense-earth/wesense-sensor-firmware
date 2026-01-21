#!/usr/bin/env python3
"""
MQTT Protobuf Decoder Test Script

Subscribes to protobuf topics from ESP32 and decodes messages in real-time.
Use this to verify ESP32 protobuf encoding is working correctly.

Usage:
    ./venv/bin/python3 mqtt_decoder_test.py

Press Ctrl+C to exit.
"""

import sys
import os

# Add parent directory to path so we can import proto module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import paho.mqtt.client as mqtt
import wesense_homebrew_pb2 as pb

# MQTT Configuration (update to match your broker)
MQTT_BROKER = "192.168.43.11"
MQTT_PORT = 1883
MQTT_USER = "mqttuser"
MQTT_PASSWORD = "rubadub32"

# Topic to subscribe to
MQTT_TOPIC = "sensor/wesense/proto/#"

def on_connect(client, userdata, flags, rc):
    """Callback when connected to MQTT broker"""
    if rc == 0:
        print(f"✅ Connected to MQTT broker: {MQTT_BROKER}:{MQTT_PORT}")
        client.subscribe(MQTT_TOPIC)
        print(f"📡 Subscribed to: {MQTT_TOPIC}")
        print("\nWaiting for messages from ESP32...\n")
    else:
        print(f"❌ Connection failed with code {rc}")

def on_message(client, userdata, msg):
    """Callback when message received"""
    print("=" * 70)
    print(f"📥 Received message on: {msg.topic}")
    print(f"   Payload size: {len(msg.payload)} bytes")
    
    try:
        # Decode protobuf
        reading = pb.SensorReading()
        reading.ParseFromString(msg.payload)
        
        # Print decoded values
        print("\n📊 Decoded Protobuf Data:")
        print(f"   Device Taxonomy:")
        print(f"      Vendor: {pb.Vendor.Name(reading.vendor)}")
        print(f"      Product Line: {pb.ProductLine.Name(reading.product_line)}")
        print(f"      Device Type: {pb.DeviceType.Name(reading.device_type)}")
        
        print(f"\n   Measurement:")
        print(f"      Value: {reading.value}")
        print(f"      Reading Type: {pb.ReadingType.Name(reading.reading_type)}")
        print(f"      Sensor Model: {pb.SensorModel.Name(reading.sensor_model)}")
        if reading.unit:
            print(f"      Unit: {reading.unit}")
        
        print(f"\n   Temporal:")
        print(f"      Timestamp: {reading.timestamp}")
        print(f"      Timezone Offset: {reading.timezone_offset_sec}s")
        
        print(f"\n   Device:")
        print(f"      Device ID: 0x{reading.device_id:016X}")
        if reading.board_manufacturer:
            print(f"      Board: {reading.board_manufacturer}")
        if reading.board_model:
            print(f"      Model: {reading.board_model}")
        if reading.firmware_version:
            print(f"      Firmware: {reading.firmware_version}")
        
        print(f"\n   Location:")
        if reading.latitude_e5 != 0 or reading.longitude_e5 != 0:
            print(f"      Latitude: {reading.latitude_e5 / 100000.0:.5f}°")
            print(f"      Longitude: {reading.longitude_e5 / 100000.0:.5f}°")
            print(f"      Location Source: {pb.LocationSource.Name(reading.location_source)}")
        if reading.location_name:
            print(f"      Name: {reading.location_name}")
        
        print(f"\n   Deployment:")
        if reading.deployment_region:
            print(f"      Region: {reading.deployment_region}")
        print(f"      Type: {pb.DeploymentType.Name(reading.deployment_type)}")
        if reading.deployment_location:
            print(f"      Location: {reading.deployment_location}")
        
        print(f"\n   Transport:")
        print(f"      Type: {pb.TransportType.Name(reading.transport_type)}")
        
        if reading.calibration_status != 0:
            print(f"\n   Calibration:")
            print(f"      Status: {pb.CalibrationStatus.Name(reading.calibration_status)}")
            if reading.data_quality != 0:
                print(f"      Data Quality: {pb.DataQuality.Name(reading.data_quality)}")
            if reading.calibration_date != 0:
                print(f"      Calibration Date: {reading.calibration_date}")
        
        print("\n✅ Decode successful!")
        
    except Exception as e:
        print(f"\n❌ Decode failed: {e}")
        import traceback
        traceback.print_exc()
        
        # Print raw bytes for debugging
        print(f"\n🔍 Raw payload (hex):")
        print("   " + msg.payload.hex())
    
    print("=" * 70)
    print()

def on_disconnect(client, userdata, rc):
    """Callback when disconnected"""
    if rc != 0:
        print(f"⚠️  Unexpected disconnection. Code: {rc}")

def main():
    """Main entry point"""
    print("=" * 70)
    print("SkyTrace Protobuf Decoder - MQTT Test")
    print("=" * 70)
    
    # Create MQTT client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect
    
    # Set credentials if needed
    if MQTT_USER and MQTT_PASSWORD:
        client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
    
    try:
        # Connect to broker
        print(f"\n🔌 Connecting to {MQTT_BROKER}:{MQTT_PORT}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        
        # Start listening
        client.loop_forever()
        
    except KeyboardInterrupt:
        print("\n\n⏹  Stopped by user")
        client.disconnect()
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
