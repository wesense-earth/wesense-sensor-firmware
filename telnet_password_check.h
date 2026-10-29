// Build-time guard for the telnet monitor password.
//
// The password comes from TELNET_PASS in credentials.h (compiled in; an ESP32 has no environment
// variables). An empty password would let anyone log in by pressing Enter, and the placeholder from
// credentials.h.example is public, so the build fails if either is left in place while telnet is on.
// Written for C++11 (single-expression constexpr) so it works with every ESP32 Arduino core.

#ifndef TELNET_PASSWORD_CHECK_H
#define TELNET_PASSWORD_CHECK_H

constexpr bool telnetStringsEqual(const char* a, const char* b) {
  return *a == *b && (*a == '\0' || telnetStringsEqual(a + 1, b + 1));
}

constexpr bool telnetPasswordUnusable(const char* password) {
  return password[0] == '\0' || telnetStringsEqual(password, "CHANGE_THIS_PASSWORD");
}

#define REQUIRE_TELNET_PASSWORD(telnet_enabled, password)                                      \
  static_assert(!(telnet_enabled) || !telnetPasswordUnusable(password),                        \
                "TELNET_PASS in credentials.h must be your own password (not empty and not "   \
                "CHANGE_THIS_PASSWORD), or set ENABLE_SECURE_TELNET to false")

#endif  // TELNET_PASSWORD_CHECK_H
