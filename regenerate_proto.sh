#!/bin/bash
# Regenerate the nanopb C files from the single source of truth, proto/wesense_homebrew_v2.proto.
#
# The generated files are committed in two places: proto/ and the repository root (where the
# Arduino IDE picks them up). CI regenerates them and fails if they differ, so run this and commit
# the result whenever the .proto changes or the nanopb generator version changes.
#
# Needs: pip install -r requirements.txt   (provides nanopb_generator)
set -euo pipefail

cd "$(dirname "$0")"

if [ -d "venv" ]; then
    # shellcheck disable=SC1091
    source venv/bin/activate
fi

echo "Generating C files (nanopb) from proto/wesense_homebrew_v2.proto"
# Run from the repository root so the include guards and include paths match the committed files.
nanopb_generator -I . proto/wesense_homebrew_v2.proto

echo "Copying to the repository root for the Arduino IDE"
cp proto/wesense_homebrew_v2.pb.h proto/wesense_homebrew_v2.pb.c .

echo "Done. Review with git diff, then compile and upload the firmware."
