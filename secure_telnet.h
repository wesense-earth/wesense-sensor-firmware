// ======================
// SECURE NETWORK SERIAL MONITOR
// ======================
// Production-ready telnet-to-serial bridge with security safeguards

#include <WiFiServer.h>

class SecureTelnetMonitor {
private:
  WiFiServer server;
  WiFiClient client;
  bool authenticated = false;
  bool enabled = false;
  unsigned long connectionTime = 0;
  unsigned long lastActivity = 0;
  String inputBuffer = "";
  
  // Security settings
  unsigned long maxConnectionTime = 86400000;  // Default 24 hours, configurable
  unsigned long idleTimeout = 3600000;         // Default 1 hour, configurable
  static const unsigned long AUTH_TIMEOUT = 30000;         // 30 seconds to authenticate
  static const int MAX_FAILED_ATTEMPTS = 3;
  static const int LOCKOUT_TIME = 300000;                  // 5 minute lockout after failed attempts
  
  int failedAttempts = 0;
  unsigned long lockoutUntil = 0;
  String allowedIP = "";  // Optional: restrict to specific IP
  
  // Simple authentication (you can make this more complex)
  const String PASSWORD = "esp32_debug_2025!";  // Change this!
  
public:
  SecureTelnetMonitor(int port = 2323, unsigned long maxConnTime = 86400000, unsigned long idleTimeoutMs = 3600000) 
    : server(port), maxConnectionTime(maxConnTime), idleTimeout(idleTimeoutMs) {}  // Non-standard port for security
  
  void begin() {
    enabled = true;
    server.begin();
    server.setNoDelay(true);
    Serial.println("🔒 Secure telnet monitor started on port 2323");
    Serial.println("🔒 Authentication required for access");
    Serial.print("🔒 Connect: telnet ");
    Serial.print(WiFi.localIP());
    Serial.println(" 2323");
  }
  
  void setAllowedIP(String ip) {
    allowedIP = ip;
    Serial.println("🔒 Telnet access restricted to IP: " + ip);
  }
  
  void handle() {
    if (!enabled) return;
    
    // Check for lockout period
    if (lockoutUntil > 0 && millis() < lockoutUntil) {
      if (server.hasClient()) {
        WiFiClient tempClient = server.available();
        tempClient.println("🚫 Access temporarily locked due to failed attempts");
        tempClient.stop();
      }
      return;
    } else if (lockoutUntil > 0) {
      lockoutUntil = 0;  // Reset lockout
      failedAttempts = 0;
    }
    
    // Handle new connections
    if (server.hasClient()) {
      if (client && client.connected()) {
        // Already have a client, reject new connection
        WiFiClient newClient = server.available();
        newClient.println("🚫 Only one connection allowed at a time");
        newClient.stop();
        return;
      }
      
      client = server.available();
      String clientIP = client.remoteIP().toString();
      
      // IP restriction check
      if (allowedIP != "" && clientIP != allowedIP) {
        Serial.println("🚫 Telnet connection rejected from unauthorized IP: " + clientIP);
        client.println("🚫 Access denied - unauthorized IP address");
        client.stop();
        return;
      }
      
      Serial.println("🔗 Telnet connection from: " + clientIP);
      connectionTime = millis();
      lastActivity = millis();
      authenticated = false;
      inputBuffer = "";
      
      // Send authentication prompt
      client.println("🔒 ESP32 Secure Serial Monitor");
      client.println("🔒 Authentication required");
      client.print("Password: ");
    }
    
    // Handle existing connection
    if (client && client.connected()) {
      // Check timeouts
      if (millis() - connectionTime > maxConnectionTime) {
        client.println("\n⏰ Maximum connection time exceeded");
        disconnectClient("Max connection time");
        return;
      }
      
      if (millis() - lastActivity > idleTimeout) {
        client.println("\n⏰ Connection timed out due to inactivity");
        disconnectClient("Idle timeout");
        return;
      }
      
      // Handle incoming data
      while (client.available()) {
        char c = client.read();
        lastActivity = millis();
        
        if (!authenticated) {
          handleAuthentication(c);
        } else {
          // Authenticated - handle commands safely
          handleAuthenticatedInput(c);
        }
      }
      
      // Forward Serial output to authenticated client
      if (authenticated) {
        while (Serial.available()) {
          char c = Serial.read();
          client.write(c);
        }
      }
    }
  }
  
private:
  void handleAuthentication(char c) {
    if (c == '\r' || c == '\n') {
      // Check password
      if (inputBuffer.equals(PASSWORD)) {
        authenticated = true;
        client.println("\n✅ Authentication successful");
        client.println("📡 Serial monitor active - Ctrl+] then 'quit' to exit");
        client.println("----------------------------------------");
        Serial.println("🔒 Telnet client authenticated successfully");
        failedAttempts = 0;  // Reset failed attempts on success
      } else {
        failedAttempts++;
        client.println("\n❌ Authentication failed");
        Serial.println("🚫 Telnet authentication failed from " + client.remoteIP().toString());
        
        if (failedAttempts >= MAX_FAILED_ATTEMPTS) {
          client.println("🚫 Too many failed attempts - access locked");
          lockoutUntil = millis() + LOCKOUT_TIME;
          Serial.println("🚫 Telnet access locked due to failed attempts");
          disconnectClient("Max failed attempts");
          return;
        }
        
        client.print("Password (" + String(MAX_FAILED_ATTEMPTS - failedAttempts) + " attempts remaining): ");
      }
      inputBuffer = "";
    } else if (c == 8 || c == 127) { // Backspace
      if (inputBuffer.length() > 0) {
        inputBuffer.remove(inputBuffer.length() - 1);
      }
    } else if (c >= 32 && c <= 126) { // Printable characters only
      inputBuffer += c;
      
      // Prevent buffer overflow
      if (inputBuffer.length() > 50) {
        client.println("\n❌ Input too long");
        disconnectClient("Input overflow");
        return;
      }
    }
    
    // Authentication timeout
    if (!authenticated && millis() - connectionTime > AUTH_TIMEOUT) {
      client.println("\n⏰ Authentication timeout");
      disconnectClient("Auth timeout");
    }
  }
  
  void handleAuthenticatedInput(char c) {
    // For security, only allow very limited input
    if (c == '\r' || c == '\n') {
      inputBuffer.trim();
      
      // Safe commands only
      if (inputBuffer.equals("quit") || inputBuffer.equals("exit")) {
        client.println("👋 Goodbye!");
        disconnectClient("User quit");
      } else if (inputBuffer.equals("help")) {
        client.println("Available commands:");
        client.println("  quit/exit - Disconnect");
        client.println("  help      - Show this help");
        client.println("  status    - Show system status");
      } else if (inputBuffer.equals("status")) {
        showSystemStatus();
      } else if (inputBuffer.length() > 0) {
        client.println("❌ Unknown command: " + inputBuffer);
        client.println("Type 'help' for available commands");
      }
      inputBuffer = "";
    } else if (c == 8 || c == 127) { // Backspace
      if (inputBuffer.length() > 0) {
        inputBuffer.remove(inputBuffer.length() - 1);
      }
    } else if (c >= 32 && c <= 126) { // Printable characters only
      inputBuffer += c;
      
      // Prevent command injection
      if (inputBuffer.length() > 20) {
        client.println("\n❌ Command too long");
        inputBuffer = "";
      }
    }
    // Ignore all other characters (control codes, etc.)
  }
  
  void showSystemStatus() {
    client.println("📊 ESP32 System Status:");
    client.println("  Uptime: " + String(millis() / 1000) + " seconds");
    client.println("  Free heap: " + String(ESP.getFreeHeap()) + " bytes");
    client.println("  WiFi RSSI: " + String(WiFi.RSSI()) + " dBm");
    client.println("  Connection time: " + String((millis() - connectionTime) / 1000) + " seconds");
  }
  
  void disconnectClient(String reason) {
    if (client) {
      Serial.println("🔒 Telnet client disconnected: " + reason);
      client.stop();
    }
    authenticated = false;
    inputBuffer = "";
  }
  
public:
  void disable() {
    enabled = false;
    if (client && client.connected()) {
      client.println("🔒 Serial monitor disabled");
      client.stop();
    }
    server.end();
    Serial.println("🔒 Secure telnet monitor disabled");
  }
  
  bool isConnected() {
    return client && client.connected() && authenticated;
  }
  
  // Safe way to send debug output to both serial and telnet
  void println(String message) {
    Serial.println(message);
    if (isConnected()) {
      client.println(message);
    }
  }
  
  void print(String message) {
    Serial.print(message);
    if (isConnected()) {
      client.print(message);
    }
  }
};