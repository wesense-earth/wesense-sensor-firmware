#!/bin/bash
# Regenerate all protobuf files from single source of truth
set -e

echo "=== Regenerating Protobuf Files ==="
echo ""

# Activate venv if it exists
if [ -d "venv" ]; then
    source venv/bin/activate
fi

# Change to proto directory
cd proto

echo "1. Generating C files for ESP32 (nanopb)..."
nanopb_generator wesense_homebrew.proto
echo "   ✓ Created wesense_homebrew.pb.h and .pb.c"

echo ""
echo "2. Generating Python files for decoder..."
protoc --python_out=. wesense_homebrew.proto
echo "   ✓ Created wesense_homebrew_pb2.py"

echo ""
echo "3. Copying Python protobuf to decoder directory..."
if [ -d "wesense-decoder" ]; then
    cp wesense_homebrew_pb2.py wesense-decoder/
    echo "   ✓ Copied to wesense-decoder/"
else
    echo "   ⚠ wesense-decoder directory not found (skipping)"
fi

cd ..

echo ""
echo "4. Copying C files to project root for Arduino IDE..."
cp proto/wesense_homebrew.pb.h proto/wesense_homebrew.pb.c .
echo "   ✓ Copied to project root"

echo ""
echo "5. Cleaning up old skytrace files..."
if [ -f "skytrace_homebrew.pb.h" ]; then
    rm skytrace_homebrew.pb.h skytrace_homebrew.pb.c 2>/dev/null || true
    rm proto/skytrace_homebrew.pb.h proto/skytrace_homebrew.pb.c 2>/dev/null || true
    echo "   ✓ Removed old skytrace_homebrew files"
else
    echo "   ✓ No old files to remove"
fi

echo ""
echo "=== Files Generated ==="
ls -lh proto/wesense_homebrew.pb.* proto/wesense_homebrew_pb2.py
echo ""
echo "Files in project root:"
ls -lh wesense_homebrew.pb.*

echo ""
echo "=== Next Steps ==="
echo "1. Compile and upload ESP32 firmware in Arduino IDE"
echo "2. (Optional) Rebuild decoder: cd proto/wesense-decoder && docker build -t wesense/decoder:latest ."
echo "3. (Optional) Restart decoder on TrueNAS"
