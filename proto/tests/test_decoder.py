#!/usr/bin/env python3
"""
Simple test script to verify protobuf decoding works.
Tests with hardcoded binary data before connecting to MQTT.
"""

import sys
import os

# Add parent directory to path so we can import proto module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import wesense_homebrew_pb2 as pb

def test_decode_sample():
    """Test decoding with sample protobuf data"""
    
    # Sample encoded protobuf (will be replaced with real data from ESP32)
    # This is a minimal test message: vendor=1, product_line=1, device_type=1, value=21.5
    test_data = bytes([
        0x08, 0x01,  # vendor = WESENSE (1)
        0x10, 0x01,  # product_line = HOMEBREW (1)
        0x18, 0x01,  # device_type = BEACON (1)
        0x25, 0x00, 0x00, 0xAC, 0x41,  # value = 21.5 (float)
        0x28, 0x01,  # reading_type = TEMPERATURE (1)
        0x30, 0x01,  # sensor_model = SHT4X (1)
    ])
    
    # Decode
    reading = pb.SensorReading()
    reading.ParseFromString(test_data)
    
    # Print decoded values
    print("=== Decoded Protobuf Message ===")
    print(f"Vendor: {pb.Vendor.Name(reading.vendor)}")
    print(f"Product Line: {pb.ProductLine.Name(reading.product_line)}")
    print(f"Device Type: {pb.DeviceType.Name(reading.device_type)}")
    print(f"Value: {reading.value}")
    print(f"Reading Type: {pb.ReadingType.Name(reading.reading_type)}")
    print(f"Sensor Model: {pb.SensorModel.Name(reading.sensor_model)}")
    print(f"\nPayload size: {len(test_data)} bytes")
    
    return True

if __name__ == "__main__":
    try:
        test_decode_sample()
        print("\n✅ Decoder test passed!")
    except Exception as e:
        print(f"\n❌ Decoder test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
