#pragma once

// =============================================================================
// WESENSE DISPLAY ICONS
// =============================================================================
// 8x8 bitmap icons for navigation bar and inline display
// Inspired by Meshtastic UI patterns (see firmware/src/graphics/images.h)
// Each row is 8 bits, drawn left to right (MSB = left pixel)
// =============================================================================

// Screen Navigation Icons (8x8)
// =============================

// Thermometer - Environment screen (Temperature/Humidity/Pressure)
const uint8_t icon_environment[] PROGMEM = {
    0b00011000, // ░░░██░░░  bulb top
    0b00100100, // ░░█░░█░░  tube
    0b00100100, // ░░█░░█░░  tube
    0b00100100, // ░░█░░█░░  tube
    0b00100100, // ░░█░░█░░  tube
    0b01011010, // ░█░██░█░  bulb expand
    0b01011010, // ░█░██░█░  mercury
    0b00111100  // ░░████░░  bulb bottom
};

// Cloud with particles - Air Quality screen (CO2/PM/VOC)
const uint8_t icon_airquality[] PROGMEM = {
    0b00111100, // ░░████░░  cloud top
    0b01111110, // ░██████░  cloud middle
    0b11111111, // ████████  cloud bottom
    0b11111111, // ████████  cloud bottom
    0b00000000, // ░░░░░░░░  gap
    0b01010100, // ░█░█░█░░  particles
    0b00101010, // ░░█░█░█░  particles
    0b01010100  // ░█░█░█░░  particles
};

// Battery with lightning - Power screen
const uint8_t icon_power[] PROGMEM = {
    0b01111100, // ░█████░░  battery top
    0b11111110, // ███████░  battery body
    0b10010010, // █░░█░░█░  with bolt
    0b10011110, // █░░████░  lightning
    0b10111100, // █░████░░  lightning
    0b10010010, // █░░█░░█░  with bolt
    0b11111110, // ███████░  battery body
    0b01111100  // ░█████░░  battery bottom
};

// Radio waves - LoRa/LoRaWAN screen
const uint8_t icon_lora[] PROGMEM = {
    0b00000010, // ░░░░░░█░  small wave
    0b00010010, // ░░░█░░█░  medium wave
    0b01010010, // ░█░█░░█░  large wave
    0b01010010, // ░█░█░░█░  antenna
    0b01010010, // ░█░█░░█░  large wave
    0b00010010, // ░░░█░░█░  medium wave
    0b00000010, // ░░░░░░█░  small wave
    0b00000011  // ░░░░░░██  base
};

// WiFi symbol - Network screen
const uint8_t icon_network[] PROGMEM = {
    0b00000000, // ░░░░░░░░
    0b01111110, // ░██████░  outer arc
    0b10000001, // █░░░░░░█
    0b00111100, // ░░████░░  middle arc
    0b01000010, // ░█░░░░█░
    0b00011000, // ░░░██░░░  inner arc
    0b00000000, // ░░░░░░░░
    0b00011000  // ░░░██░░░  dot
};

// Gear/cog - System screen
const uint8_t icon_system[] PROGMEM = {
    0b00100100, // ░░█░░█░░  top teeth
    0b00111100, // ░░████░░  top
    0b11000011, // ██░░░░██  side teeth
    0b01011010, // ░█░██░█░  center ring
    0b01011010, // ░█░██░█░  center ring
    0b11000011, // ██░░░░██  side teeth
    0b00111100, // ░░████░░  bottom
    0b00100100  // ░░█░░█░░  bottom teeth
};

// Chip/sensor - Sensors list screen
const uint8_t icon_sensors[] PROGMEM = {
    0b01010100, // ░█░█░█░░  pins top
    0b00111100, // ░░████░░  chip top
    0b10111110, // █░█████░  chip with pins
    0b10111110, // █░█████░  chip body
    0b10111110, // █░█████░  chip body
    0b10111110, // █░█████░  chip with pins
    0b00111100, // ░░████░░  chip bottom
    0b00101000  // ░░█░█░░░  pins bottom
};

// Bug/Debug - LoRaWAN Debug screen
const uint8_t icon_debug[] PROGMEM = {
    0b01000010, // ░█░░░░█░  antennae
    0b00100100, // ░░█░░█░░  antennae inner
    0b01111110, // ░██████░  head
    0b11011011, // ██░██░██  eyes
    0b01111110, // ░██████░  body top
    0b10111101, // █░████░█  legs
    0b01111110, // ░██████░  body bottom
    0b10000001  // █░░░░░░█  legs
};

// Status/Activity Icons (8x8)
// ===========================

// Up arrow - TX/Transmitting indicator
const uint8_t icon_tx[] PROGMEM = {
    0b00011000, // ░░░██░░░  arrow tip
    0b00111100, // ░░████░░
    0b01111110, // ░██████░
    0b11011011, // ██░██░██  arrow head
    0b00011000, // ░░░██░░░  shaft
    0b00011000, // ░░░██░░░  shaft
    0b00011000, // ░░░██░░░  shaft
    0b00011000  // ░░░██░░░  shaft base
};

// Down arrow - RX/Receiving indicator
const uint8_t icon_rx[] PROGMEM = {
    0b00011000, // ░░░██░░░  shaft top
    0b00011000, // ░░░██░░░  shaft
    0b00011000, // ░░░██░░░  shaft
    0b00011000, // ░░░██░░░  shaft
    0b11011011, // ██░██░██  arrow head
    0b01111110, // ░██████░
    0b00111100, // ░░████░░
    0b00011000  // ░░░██░░░  arrow tip
};

// Exclamation mark - Warning/Error
const uint8_t icon_warning[] PROGMEM = {
    0b00011000, // ░░░██░░░
    0b00111100, // ░░████░░
    0b00111100, // ░░████░░
    0b00011000, // ░░░██░░░
    0b00011000, // ░░░██░░░
    0b00000000, // ░░░░░░░░
    0b00011000, // ░░░██░░░
    0b00011000  // ░░░██░░░
};

// Checkmark - OK/Success
const uint8_t icon_ok[] PROGMEM = {
    0b00000000, // ░░░░░░░░
    0b00000001, // ░░░░░░░█
    0b00000011, // ░░░░░░██
    0b00000110, // ░░░░░██░
    0b11001100, // ██░░██░░
    0b01111000, // ░████░░░
    0b00110000, // ░░██░░░░
    0b00000000  // ░░░░░░░░
};

// X mark - Error/Failed
const uint8_t icon_error[] PROGMEM = {
    0b00000000, // ░░░░░░░░
    0b11000011, // ██░░░░██
    0b01100110, // ░██░░██░
    0b00111100, // ░░████░░
    0b00111100, // ░░████░░
    0b01100110, // ░██░░██░
    0b11000011, // ██░░░░██
    0b00000000  // ░░░░░░░░
};

// Calibrating - hourglass/spinner
const uint8_t icon_calibrating[] PROGMEM = {
    0b11111111, // ████████  top bar
    0b01000010, // ░█░░░░█░  top triangle
    0b00100100, // ░░█░░█░░
    0b00011000, // ░░░██░░░  middle pinch
    0b00011000, // ░░░██░░░
    0b00100100, // ░░█░░█░░
    0b01000010, // ░█░░░░█░  bottom triangle
    0b11111111  // ████████  bottom bar
};

// Connection Status Icons (8x8)
// =============================

// Connected/online
const uint8_t icon_connected[] PROGMEM = {
    0b00111100, // ░░████░░
    0b01000010, // ░█░░░░█░
    0b10011001, // █░░██░░█  happy face
    0b10000001, // █░░░░░░█
    0b10100101, // █░█░░█░█  smile
    0b10011001, // █░░██░░█
    0b01000010, // ░█░░░░█░
    0b00111100  // ░░████░░
};

// Disconnected/offline
const uint8_t icon_disconnected[] PROGMEM = {
    0b00111100, // ░░████░░
    0b01000010, // ░█░░░░█░
    0b10011001, // █░░██░░█  sad face
    0b10000001, // █░░░░░░█
    0b10011001, // █░░██░░█  frown
    0b10100101, // █░█░░█░█
    0b01000010, // ░█░░░░█░
    0b00111100  // ░░████░░
};

// Battery level icons (8x8)
// =========================

// Battery empty (0-10%)
const uint8_t icon_battery_empty[] PROGMEM = {
    0b01111100, // ░█████░░
    0b11111110, // ███████░
    0b10000010, // █░░░░░█░
    0b10000011, // █░░░░░██  nub
    0b10000011, // █░░░░░██  nub
    0b10000010, // █░░░░░█░
    0b11111110, // ███████░
    0b01111100  // ░█████░░
};

// Battery low (10-30%)
const uint8_t icon_battery_low[] PROGMEM = {
    0b01111100, // ░█████░░
    0b11111110, // ███████░
    0b11000010, // ██░░░░█░  1 bar
    0b11000011, // ██░░░░██  nub
    0b11000011, // ██░░░░██  nub
    0b11000010, // ██░░░░█░
    0b11111110, // ███████░
    0b01111100  // ░█████░░
};

// Battery medium (30-60%)
const uint8_t icon_battery_med[] PROGMEM = {
    0b01111100, // ░█████░░
    0b11111110, // ███████░
    0b11110010, // ████░░█░  2 bars
    0b11110011, // ████░░██  nub
    0b11110011, // ████░░██  nub
    0b11110010, // ████░░█░
    0b11111110, // ███████░
    0b01111100  // ░█████░░
};

// Battery high (60-90%)
const uint8_t icon_battery_high[] PROGMEM = {
    0b01111100, // ░█████░░
    0b11111110, // ███████░
    0b11111010, // █████░█░  3 bars
    0b11111011, // █████░██  nub
    0b11111011, // █████░██  nub
    0b11111010, // █████░█░
    0b11111110, // ███████░
    0b01111100  // ░█████░░
};

// Battery full (90-100%)
const uint8_t icon_battery_full[] PROGMEM = {
    0b01111100, // ░█████░░
    0b11111110, // ███████░
    0b11111110, // ███████░  full
    0b11111111, // ████████  nub
    0b11111111, // ████████  nub
    0b11111110, // ███████░
    0b11111110, // ███████░
    0b01111100  // ░█████░░
};

// Battery charging
const uint8_t icon_battery_charging[] PROGMEM = {
    0b01111100, // ░█████░░
    0b11111110, // ███████░
    0b10010010, // █░░█░░█░  lightning
    0b10011111, // █░░█████  bolt
    0b11111001, // █████░░█  bolt
    0b10010010, // █░░█░░█░  lightning
    0b11111110, // ███████░
    0b01111100  // ░█████░░
};

// WiFi signal strength icons (8x8)
// ================================

// WiFi no signal
const uint8_t icon_wifi_none[] PROGMEM = {
    0b00000000, // ░░░░░░░░
    0b00000000, // ░░░░░░░░
    0b00000000, // ░░░░░░░░
    0b00000000, // ░░░░░░░░
    0b00000000, // ░░░░░░░░
    0b00000000, // ░░░░░░░░
    0b00000000, // ░░░░░░░░
    0b00011000  // ░░░██░░░  dot only
};

// WiFi weak signal (1 bar)
const uint8_t icon_wifi_weak[] PROGMEM = {
    0b00000000, // ░░░░░░░░
    0b00000000, // ░░░░░░░░
    0b00000000, // ░░░░░░░░
    0b00000000, // ░░░░░░░░
    0b00011000, // ░░░██░░░  inner arc
    0b00000000, // ░░░░░░░░
    0b00000000, // ░░░░░░░░
    0b00011000  // ░░░██░░░  dot
};

// WiFi fair signal (2 bars)
const uint8_t icon_wifi_fair[] PROGMEM = {
    0b00000000, // ░░░░░░░░
    0b00000000, // ░░░░░░░░
    0b00111100, // ░░████░░  middle arc
    0b01000010, // ░█░░░░█░
    0b00011000, // ░░░██░░░  inner arc
    0b00000000, // ░░░░░░░░
    0b00000000, // ░░░░░░░░
    0b00011000  // ░░░██░░░  dot
};

// WiFi good signal (3 bars)
const uint8_t icon_wifi_good[] PROGMEM = {
    0b01111110, // ░██████░  outer arc
    0b10000001, // █░░░░░░█
    0b00111100, // ░░████░░  middle arc
    0b01000010, // ░█░░░░█░
    0b00011000, // ░░░██░░░  inner arc
    0b00000000, // ░░░░░░░░
    0b00000000, // ░░░░░░░░
    0b00011000  // ░░░██░░░  dot
};

// Icon dimensions
#define WESENSE_ICON_WIDTH 8
#define WESENSE_ICON_HEIGHT 8

// Navigation bar settings
#define NAV_BAR_HEIGHT 10
#define NAV_BAR_ICON_SPACING 2
#define NAV_BAR_HIDE_DELAY_MS 2000
