#!/bin/bash
# Compile-tests telnet_password_check.h: the build must fail for an empty or placeholder password
# while telnet is enabled, and succeed otherwise. Needs g++; no Arduino or board.
set -u
cd "$(dirname "$0")/.."
CXX=${CXX:-g++}
fail=0

try() { # <expect ok|fail> <label> <telnet enabled true|false> <password literal>
  local expect=$1 label=$2 enabled=$3 pass=$4 src out
  src=$(mktemp /tmp/telnet_check.XXXXXX.cpp)
  printf '#include "%s/telnet_password_check.h"\nconst bool ENABLE_SECURE_TELNET = %s;\nREQUIRE_TELNET_PASSWORD(ENABLE_SECURE_TELNET, %s);\nint main() { return 0; }\n' "$PWD" "$enabled" "$pass" > "$src"
  if out=$($CXX -std=gnu++11 -fsyntax-only "$src" 2>&1); then got=ok; else got=fail; fi
  rm -f "$src"
  if [ "$got" = "$expect" ]; then echo "PASS  $label"; else echo "FAIL  $label (expected $expect, got $got)"; echo "$out" | head -5; fail=1; fi
  if [ "$expect" = fail ] && [ "$got" = fail ] && ! echo "$out" | grep -q "TELNET_PASS in credentials.h"; then
    echo "FAIL  $label: failed, but not with the helpful message"; fail=1
  fi
}

try fail "placeholder password, telnet on"  true  '"CHANGE_THIS_PASSWORD"'
try fail "empty password, telnet on"        true  '""'
try ok   "own password, telnet on"          true  '"correct-horse-battery"'
try ok   "password that merely starts like the placeholder" true '"CHANGE_THIS_PASSWORD2"'
try ok   "placeholder, telnet off"          false '"CHANGE_THIS_PASSWORD"'
try ok   "empty, telnet off"                false '""'
exit $fail
