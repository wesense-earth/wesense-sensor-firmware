#!/usr/bin/env python3
"""
MQTT protobuf decoder check (manual tool).

Subscribes to the raw WiFi sensor topic on an MQTT broker and decodes each message as a
SensorReadingV2, so you can confirm an ESP32's protobuf encoding is working.

Connection details come from environment variables. Nothing is hard-coded and there are no
defaults for the credentials: if they are missing the script says so and stops.

    MQTT_BROKER     broker host or IP            (required)
    MQTT_USER       broker username              (required)
    MQTT_PASSWORD   broker password              (required)
    MQTT_PORT       broker port                  (default 1883)
    MQTT_TOPIC      topic filter                 (default wesense/v2/wifi/#)

Usage:
    MQTT_BROKER=... MQTT_USER=... MQTT_PASSWORD=... python3 proto/tests/mqtt_decoder_test.py

Press Ctrl+C to exit.
"""

import os
import sys

# Make the generated module in proto/ importable
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import paho.mqtt.client as mqtt
import wesense_homebrew_v2_pb2 as pb


def require_env(name):
    value = os.environ.get(name)
    if not value:
        sys.exit(f"{name} is not set. See the usage notes at the top of this file.")
    return value


def describe(payload):
    reading = pb.SensorReadingV2()
    reading.ParseFromString(payload)

    lines = [f"   Device ID: 0x{reading.device_id:016X}", f"   Timestamp: {reading.timestamp}"]
    if reading.node_name:
        lines.append(f"   Node name: {reading.node_name}")
    if reading.board_type:
        lines.append(f"   Board: {pb.BoardType.Name(reading.board_type)}")
    if reading.latitude_e5 or reading.longitude_e5:
        lines.append(
            f"   Location: {reading.latitude_e5 / 100000.0:.5f}, {reading.longitude_e5 / 100000.0:.5f}"
        )
    lines.append(f"   Deployment: {pb.DeploymentType.Name(reading.deployment_type)}")
    lines.append(f"   Measurements ({len(reading.measurements)}):")
    for m in reading.measurements:
        lines.append(
            f"      {pb.ReadingType.Name(m.reading_type)} = {m.value:g}"
            f"  [{pb.SensorModel.Name(m.sensor_model)}]"
        )
    return "\n".join(lines)


def main():
    broker = require_env("MQTT_BROKER")
    user = require_env("MQTT_USER")
    password = require_env("MQTT_PASSWORD")
    port = int(os.environ.get("MQTT_PORT", "1883"))
    topic = os.environ.get("MQTT_TOPIC", "wesense/v2/wifi/#")

    def on_connect(client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            sys.exit(f"Connection failed: {reason_code}")
        print(f"Connected to {broker}:{port}, subscribing to {topic}\nWaiting for messages...\n")
        client.subscribe(topic)

    def on_message(client, userdata, msg):
        print("=" * 70)
        print(f"{msg.topic}  ({len(msg.payload)} bytes)")
        try:
            print(describe(msg.payload))
        except Exception as exc:  # decode problems are the thing this tool is for
            print(f"   Decode FAILED: {exc}")
            print(f"   Raw payload (hex): {msg.payload.hex()}")
        print("=" * 70 + "\n")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.username_pw_set(user, password)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port)
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
