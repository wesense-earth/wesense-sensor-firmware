"""
Checks the protobuf message definition that the ESP32 firmware and the decoders share.

Runs without a board or a broker:  python3 -m unittest discover -s proto/tests -v

The messages are built, serialised and parsed back with the installed protobuf library, so this
also shows whether a protobuf upgrade still works with the committed generated code.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import wesense_homebrew_v2_pb2 as pb


def sample_reading():
    reading = pb.SensorReadingV2()
    reading.device_id = 0x24A160FFFE123456
    reading.timestamp = 1_790_000_000
    reading.latitude_e5 = -4_278_900  # -42.789
    reading.longitude_e5 = 17_390_000  # 173.9
    reading.altitude_cm = 1250
    reading.board_type = pb.BoardType.Value(pb.BoardType.keys()[1])
    reading.node_name = "test-node"
    m = reading.measurements.add()
    m.reading_type = pb.ReadingType.Value(pb.ReadingType.keys()[1])
    m.value = 21.5
    m.sensor_model = pb.SensorModel.Value(pb.SensorModel.keys()[1])
    return reading


class RoundTrip(unittest.TestCase):
    def test_full_message_survives_serialise_and_parse(self):
        original = sample_reading()
        parsed = pb.SensorReadingV2()
        parsed.ParseFromString(original.SerializeToString())
        self.assertEqual(parsed, original)
        self.assertEqual(parsed.device_id, 0x24A160FFFE123456)
        self.assertAlmostEqual(parsed.measurements[0].value, 21.5)

    def test_minimal_lora_message_is_small(self):
        # LoRa sensors send only device_id, timestamp and measurements (see the .proto comments)
        reading = pb.SensorReadingV2()
        reading.device_id = 1
        reading.timestamp = 1_790_000_000
        for _ in range(5):
            m = reading.measurements.add()
            m.reading_type = pb.ReadingType.Value(pb.ReadingType.keys()[1])
            m.value = 1.5
        self.assertLess(len(reading.SerializeToString()), 80)

    def test_all_enums_have_a_zero_default(self):
        for enum in (pb.BoardType, pb.ReadingType, pb.SensorModel, pb.DeploymentType):
            self.assertIn(0, enum.values(), f"{enum.DESCRIPTOR.name} must define a 0 value")

    def test_unknown_fields_are_ignored(self):
        # An older decoder must still parse messages from newer firmware that adds fields.
        data = sample_reading().SerializeToString() + b"\xf8\x0f\x01"  # unknown varint field 255
        parsed = pb.SensorReadingV2()
        parsed.ParseFromString(data)
        self.assertEqual(parsed.node_name, "test-node")


if __name__ == "__main__":
    unittest.main()
