/**
 * ESP32 Modbus Listener
 * 
 * A passive Modbus RTU listener device based on ESP32-C3 Mini that monitors
 * communication on a Modbus bus (e.g. Acrel ADL400 three-phase energy meter),
 * providing traffic monitoring, data parsing, and MQTT publishing.
 * 
 * Hardware:
 *   - ESP32-C3 Mini
 *   - 1x RS485 to TTL adapter (with automatic direction control)
 * 
 * Pin Configuration:
 *   - GPIO2:  RS485 RX (receives bus traffic)
 *   - GPIO8:  Status LED
 * 
 * @author  Mikko
 * @version 1.0.0
 * @date    2026-03-13
 * @license MIT
 */

// =============================================================================
// INCLUDES
// =============================================================================

#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Update.h>

// =============================================================================
// PIN DEFINITIONS
// =============================================================================

#define PIN_RS485_RX    2   // RS485 adapter RX (receives bus traffic)
#define PIN_STATUS_LED  8   // Status LED (built-in on ESP32-C3 Mini)

// =============================================================================
// UART CONFIGURATION
// =============================================================================

#define UART_RS485      1   // UART number for RS485 (UART1)
#define MODBUS_BAUD     9600  // Default Modbus baud rate (configurable)
#define MODBUS_CONFIG   SERIAL_8N1  // 8 data bits, no parity, 1 stop bit

// =============================================================================
// CONFIGURATION STRUCTURE
// =============================================================================

struct Config {
  // WiFi settings
  char wifi_ssid[33];
  char wifi_password[65];
  
  // MQTT settings
  char mqtt_server[65];
  uint16_t mqtt_port;
  char mqtt_username[33];
  char mqtt_password[33];
  
  // Modbus settings
  uint32_t modbus_baud;
  uint8_t modbus_slave_address;
};

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

// Configuration
Config config;
Preferences preferences;

// Network
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
WebServer webServer(80);

// UART for Modbus
HardwareSerial modbusSerial(UART_RS485);

// State
bool mqttConnected = false;
bool apModeActive = false;
unsigned long lastMqttReconnect = 0;
unsigned long lastWifiCheck = 0;
unsigned long bootTime = 0;

// Modbus buffer
#define MODBUS_BUFFER_SIZE 256
uint8_t modbusBuffer[MODBUS_BUFFER_SIZE];
size_t modbusBufferLen = 0;
unsigned long lastModbusActivity = 0;

// Modbus log (circular buffer)
#define LOG_ENTRY_COUNT 50
#define LOG_DATA_SIZE 64

struct LogEntry {
  unsigned long timestamp;    // millis() when received
  uint8_t data[LOG_DATA_SIZE];
  uint8_t length;
  bool crcValid;
  uint8_t slaveAddr;
  uint8_t functionCode;
};

LogEntry modbusLog[LOG_ENTRY_COUNT];
int logHead = 0;              // Next write position
int logCount = 0;             // Number of entries in buffer
unsigned long totalFrames = 0;
unsigned long validFrames = 0;

// Debug counters
bool debugMode = false;       // Enable verbose debug output
unsigned long totalBytesReceived = 0;
unsigned long lastDebugPrint = 0;

// =============================================================================
// ADL400 METER DATA (Real-time values from register 0x0061)
// =============================================================================

struct MeterData {
  // Voltages (V) - scale 0.1
  float voltageA;
  float voltageB;
  float voltageC;
  
  // Currents (A) - scale 0.01
  float currentA;
  float currentB;
  float currentC;
  
  // Active Power (W) - signed
  int16_t activePowerA;
  int16_t activePowerB;
  int16_t activePowerC;
  int16_t activePowerTotal;
  
  // Reactive Power (VAr) - signed
  int16_t reactivePowerA;
  int16_t reactivePowerB;
  int16_t reactivePowerC;
  int16_t reactivePowerTotal;
  
  // Apparent Power (VA)
  uint16_t apparentPowerA;
  uint16_t apparentPowerB;
  uint16_t apparentPowerC;
  uint16_t apparentPowerTotal;
  
  // Power Factor - scale 0.001
  float powerFactorA;
  float powerFactorB;
  float powerFactorC;
  float powerFactorTotal;
  
  // Frequency (Hz) - scale 0.01
  float frequency;
  
  // Timestamp of last update
  unsigned long lastUpdate;
  bool valid;
};

MeterData meter = {0};

// ADL400 CONFIG PARAMETERS (read occasionally)
struct MeterScaling {
  uint16_t pt;               // Potential transformer ratio/coefficient (PT)
  uint16_t ct;               // Current transformer ratio/coefficient (CT)
  unsigned long lastUpdate;
  bool valid;
};

MeterScaling scaling = {0};

// Register access tracking
#define MAX_REGISTER_RANGES 20
struct RegisterRange {
  uint16_t startAddr;
  uint16_t count;
  unsigned long lastSeen;
  unsigned long accessCount;
  bool isNew;       // Flag for newly discovered registers
  bool isExpected;  // Flag for registers decoded by GUI (expected traffic)
};

RegisterRange knownRegisters[MAX_REGISTER_RANGES];
int knownRegisterCount = 0;
int newRegisterAlerts = 0;

// Initialize expected register ranges (those decoded by GUI)
void initExpectedRegisters() {
  // ADL400 real-time data: 0x0061, 23 registers
  knownRegisters[0].startAddr = 0x0061;
  knownRegisters[0].count = 23;
  knownRegisters[0].lastSeen = 0;
  knownRegisters[0].accessCount = 0;
  knownRegisters[0].isNew = false;
  knownRegisters[0].isExpected = true;

  // ADL400 scaling/config: PT (0x008D) and CT (0x008E)
  knownRegisters[1].startAddr = 0x008D;
  knownRegisters[1].count = 2;
  knownRegisters[1].lastSeen = 0;
  knownRegisters[1].accessCount = 0;
  knownRegisters[1].isNew = false;
  knownRegisters[1].isExpected = true;

  knownRegisterCount = 2;
}

// Last request tracking (to match responses)
uint16_t lastRequestAddr = 0;
uint16_t lastRequestCount = 0;
unsigned long lastRequestTime = 0;

// MQTT Home Assistant discovery
bool haDiscoverySent = false;
bool legacyCleanupDone = false;
unsigned long lastMeterPublish = 0;
#define METER_PUBLISH_INTERVAL 5000  // Publish meter values every 5 seconds

// Device ID (derived from MAC address)
char deviceId[13];

// =============================================================================
// FUNCTION DECLARATIONS
// =============================================================================

// Configuration
void loadConfiguration();
void saveConfiguration();
void initializeDefaultConfig();

// Network
void setupWiFi();
void setupAccessPoint();
void checkWiFiConnection();

// MQTT
void setupMqtt();
void mqttReconnect();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void publishStatus();
void publishHADiscovery();
void cleanupLegacyProxyDiscovery();
void publishMeterValues();

// Web Server
void setupWebServer();
void handleRoot();
void handleConfig();
void handleSave();
void handleStatus();
void handleReboot();
void handleUpdate();
void handleOtaUpload();
void handleLog();
void handleLogData();
void handleLogClear();

// Modbus
void setupModbus();
void processModbusListener();
uint16_t calculateCRC16(uint8_t* data, size_t len);
bool validateModbusCRC(uint8_t* data, size_t len);
void addLogEntry(uint8_t* data, size_t len, bool crcValid);
String getFunctionName(uint8_t funcCode);
void parseModbusFrame(uint8_t* data, size_t len);
void parseMeterResponse(uint8_t* data, size_t len);
void trackRegisterAccess(uint16_t startAddr, uint16_t count);

// Web Server - Meter
void handleMeter();
void handleMeterData();

// Utility
void blinkLed(int times, int delayMs);
String getUptimeString();

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  // Initialize serial for debug output
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=================================");
  Serial.println("ESP32 Modbus Listener");
  Serial.println("Version: 1.0.0");
  Serial.println("=================================\n");
  
  bootTime = millis();
  
  // Initialize expected register ranges (decoded by GUI)
  initExpectedRegisters();
  
  // Initialize pins
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, HIGH);  // LED on during setup
  
  // Generate device ID from MAC address
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(deviceId, sizeof(deviceId), "%02x%02x%02x%02x%02x%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print("Device ID: ");
  Serial.println(deviceId);
  
  // Load configuration from flash
  loadConfiguration();
  
  // Setup Modbus serial port
  setupModbus();
  
  // Setup WiFi (station mode or AP mode)
  setupWiFi();
  
  // Setup MQTT
  setupMqtt();
  
  // Setup Web Server
  setupWebServer();
  
  // Setup complete
  digitalWrite(PIN_STATUS_LED, LOW);
  blinkLed(3, 200);
  Serial.println("\nSetup complete!\n");
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
  unsigned long currentMillis = millis();
  
  // Handle web server
  webServer.handleClient();
  
  // Check WiFi connection
  if (currentMillis - lastWifiCheck >= 10000) {
    checkWiFiConnection();
    lastWifiCheck = currentMillis;
  }
  
  // Handle MQTT
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqtt.connected()) {
      if (currentMillis - lastMqttReconnect >= 5000) {
        mqttReconnect();
        lastMqttReconnect = currentMillis;
      }
    } else {
      mqtt.loop();
      
      // Publish meter values periodically
      if (meter.valid && currentMillis - lastMeterPublish >= METER_PUBLISH_INTERVAL) {
        publishMeterValues();
        lastMeterPublish = currentMillis;
      }
    }
  }
  
  // Process Modbus (passive listening)
  processModbusListener();
  
  // Small delay to prevent watchdog issues
  yield();
}

// =============================================================================
// CONFIGURATION FUNCTIONS
// =============================================================================

void loadConfiguration() {
  Serial.println("Loading configuration...");
  
  preferences.begin("modbus-listen", true);  // Read-only
  
  // Check if configuration exists
  if (!preferences.isKey("configured")) {
    preferences.end();
    Serial.println("No configuration found, using defaults.");
    initializeDefaultConfig();
    saveConfiguration();
    return;
  }
  
  // Load WiFi settings
  preferences.getString("wifi_ssid", config.wifi_ssid, sizeof(config.wifi_ssid));
  preferences.getString("wifi_pass", config.wifi_password, sizeof(config.wifi_password));
  
  // Load MQTT settings
  preferences.getString("mqtt_server", config.mqtt_server, sizeof(config.mqtt_server));
  config.mqtt_port = preferences.getUShort("mqtt_port", 1883);
  preferences.getString("mqtt_user", config.mqtt_username, sizeof(config.mqtt_username));
  preferences.getString("mqtt_pass", config.mqtt_password, sizeof(config.mqtt_password));
  
  // Load Modbus settings
  config.modbus_baud = preferences.getUInt("modbus_baud", 9600);
  config.modbus_slave_address = preferences.getUChar("modbus_addr", 1);
  
  preferences.end();
  
  Serial.println("Configuration loaded.");
  Serial.print("  WiFi SSID: ");
  Serial.println(config.wifi_ssid);
  Serial.print("  MQTT Server: ");
  Serial.println(config.mqtt_server);
  Serial.print("  Modbus Baud: ");
  Serial.println(config.modbus_baud);
}

void saveConfiguration() {
  Serial.println("Saving configuration...");
  
  preferences.begin("modbus-listen", false);  // Read-write
  
  // Mark as configured
  preferences.putBool("configured", true);
  
  // Save WiFi settings
  preferences.putString("wifi_ssid", config.wifi_ssid);
  preferences.putString("wifi_pass", config.wifi_password);
  
  // Save MQTT settings
  preferences.putString("mqtt_server", config.mqtt_server);
  preferences.putUShort("mqtt_port", config.mqtt_port);
  preferences.putString("mqtt_user", config.mqtt_username);
  preferences.putString("mqtt_pass", config.mqtt_password);
  
  // Save Modbus settings
  preferences.putUInt("modbus_baud", config.modbus_baud);
  preferences.putUChar("modbus_addr", config.modbus_slave_address);
  
  preferences.end();
  
  Serial.println("Configuration saved.");
}

void initializeDefaultConfig() {
  Serial.println("Initializing default configuration...");
  
  memset(&config, 0, sizeof(config));
  
  // Default WiFi settings (empty = AP mode)
  config.wifi_ssid[0] = '\0';
  config.wifi_password[0] = '\0';
  
  // Default MQTT settings
  config.mqtt_server[0] = '\0';
  config.mqtt_port = 1883;
  config.mqtt_username[0] = '\0';
  config.mqtt_password[0] = '\0';
  
  // Default Modbus settings
  config.modbus_baud = 9600;
  config.modbus_slave_address = 1;
}

// =============================================================================
// NETWORK FUNCTIONS
// =============================================================================

void setupWiFi() {
  if (strlen(config.wifi_ssid) == 0) {
    // No WiFi configured, start in AP mode
    setupAccessPoint();
    return;
  }
  
  Serial.print("Connecting to WiFi: ");
  Serial.println(config.wifi_ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  WiFi.begin(config.wifi_ssid, config.wifi_password);
  
  // Wait for connection with timeout
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    apModeActive = false;
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed, starting AP mode.");
    setupAccessPoint();
  }
}

void setupAccessPoint() {
  Serial.println("Starting Access Point...");
  
  char apName[32];
  snprintf(apName, sizeof(apName), "ModbusListener-%s", &deviceId[8]);
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apName, "modbuslistener");
  apModeActive = true;
  
  Serial.print("AP SSID: ");
  Serial.println(apName);
  Serial.print("AP Password: modbuslistener");
  Serial.print("\nAP IP: ");
  Serial.println(WiFi.softAPIP());
}

void checkWiFiConnection() {
  if (strlen(config.wifi_ssid) == 0) {
    return;  // Running in AP mode
  }

  if (apModeActive) {
    return;  // Keep AP fallback stable; reconnect after reboot or new settings
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, reconnecting...");
    WiFi.reconnect();
  }
}

// =============================================================================
// MQTT FUNCTIONS
// =============================================================================

void setupMqtt() {
  if (strlen(config.mqtt_server) == 0) {
    Serial.println("MQTT server not configured.");
    return;
  }
  
  mqtt.setServer(config.mqtt_server, config.mqtt_port);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(1024);
  
  Serial.print("MQTT configured for: ");
  Serial.print(config.mqtt_server);
  Serial.print(":");
  Serial.println(config.mqtt_port);
}

void mqttReconnect() {
  if (strlen(config.mqtt_server) == 0) {
    return;
  }
  
  Serial.print("Connecting to MQTT...");
  
  char clientId[32];
  snprintf(clientId, sizeof(clientId), "ModbusListener-%s", deviceId);
  
  char willTopic[64];
  snprintf(willTopic, sizeof(willTopic), "modbus_listener/%s/status", deviceId);
  
  bool connected;
  if (strlen(config.mqtt_username) > 0) {
    connected = mqtt.connect(clientId, config.mqtt_username, config.mqtt_password,
                             willTopic, 1, true, "offline");
  } else {
    connected = mqtt.connect(clientId, willTopic, 1, true, "offline");
  }
  
  if (connected) {
    Serial.println("connected!");
    mqttConnected = true;
    
    // Publish online status
    mqtt.publish(willTopic, "online", true);
    
    // Subscribe to command topics
    char cmdTopic[64];
    snprintf(cmdTopic, sizeof(cmdTopic), "modbus_listener/%s/cmd/#", deviceId);
    mqtt.subscribe(cmdTopic);

    // Remove retained discovery payloads from legacy proxy firmware.
    if (!legacyCleanupDone) {
      cleanupLegacyProxyDiscovery();
      legacyCleanupDone = true;
    }
    
    // Send Home Assistant discovery messages
    publishHADiscovery();
    
    // Publish initial status
    publishStatus();
  } else {
    Serial.print("failed, rc=");
    Serial.println(mqtt.state());
    mqttConnected = false;
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  String payloadStr = "";
  for (unsigned int i = 0; i < length; i++) {
    payloadStr += (char)payload[i];
  }
  
  Serial.print("MQTT received [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(payloadStr);
}

void publishStatus() {
  if (!mqtt.connected()) return;
  
  char topic[64];
  char payload[256];
  
  // Publish IP address
  snprintf(topic, sizeof(topic), "modbus_listener/%s/ip", deviceId);
  if (WiFi.status() == WL_CONNECTED) {
    snprintf(payload, sizeof(payload), "%s", WiFi.localIP().toString().c_str());
  } else {
    snprintf(payload, sizeof(payload), "%s", WiFi.softAPIP().toString().c_str());
  }
  mqtt.publish(topic, payload, true);
}

void cleanupLegacyProxyDiscovery() {
  if (!mqtt.connected()) return;

  Serial.println("Cleaning up legacy modbus_proxy discovery topics...");

  const char* sensorIds[] = {
    "voltage_a", "voltage_b", "voltage_c",
    "current_a", "current_b", "current_c",
    "power_a", "power_b", "power_c", "power_total",
    "reactive_a", "reactive_b", "reactive_c", "reactive_total",
    "apparent_a", "apparent_b", "apparent_c", "apparent_total",
    "pf_a", "pf_b", "pf_c", "pf_total",
    "frequency", "pt", "ct"
  };

  char topic[160];
  for (size_t i = 0; i < (sizeof(sensorIds) / sizeof(sensorIds[0])); i++) {
    snprintf(topic, sizeof(topic), "homeassistant/sensor/modbus_proxy_%s/%s/config", deviceId, sensorIds[i]);
    // Empty retained payload deletes retained discovery topic on broker.
    mqtt.publish(topic, "", true);
    delay(10);
  }

  Serial.println("Legacy proxy discovery cleanup published.");
}

// Helper to publish a single HA sensor discovery message
void publishHASensor(const char* sensorId, const char* name, const char* unit, 
                     const char* deviceClass, const char* valueTemplate, const char* icon = nullptr) {
  char topic[128];
  char payload[768];
  
  snprintf(topic, sizeof(topic), "homeassistant/sensor/modbus_listener_%s/%s/config", 
           deviceId, sensorId);
  
  // Build JSON payload
  JsonDocument doc;
  
  char uniqueId[64];
  snprintf(uniqueId, sizeof(uniqueId), "modbus_listener_%s_%s", deviceId, sensorId);
  doc["unique_id"] = uniqueId;
  
  char fullName[64];
  snprintf(fullName, sizeof(fullName), "Energy Meter %s", name);
  doc["name"] = fullName;
  
  char stateTopic[64];
  snprintf(stateTopic, sizeof(stateTopic), "modbus_listener/%s/meter", deviceId);
  doc["state_topic"] = stateTopic;
  doc["value_template"] = valueTemplate;
  
  if (unit && strlen(unit) > 0) {
    doc["unit_of_measurement"] = unit;
  }
  if (deviceClass && strlen(deviceClass) > 0) {
    doc["device_class"] = deviceClass;
  }
  if (icon && strlen(icon) > 0) {
    doc["icon"] = icon;
  }
  
  // State class for statistics
  if (deviceClass && (strcmp(deviceClass, "power") == 0 || 
      strcmp(deviceClass, "voltage") == 0 || 
      strcmp(deviceClass, "current") == 0 ||
      strcmp(deviceClass, "frequency") == 0 ||
      strcmp(deviceClass, "power_factor") == 0)) {
    doc["state_class"] = "measurement";
  }
  
  // Device info
  JsonObject device = doc["device"].to<JsonObject>();
  device["identifiers"][0] = deviceId;
  device["name"] = "Modbus Energy Meter";
  device["model"] = "ADL400";
  device["manufacturer"] = "Acrel";
  device["sw_version"] = "1.0.0";
  
  char availTopic[64];
  snprintf(availTopic, sizeof(availTopic), "modbus_listener/%s/status", deviceId);
  doc["availability_topic"] = availTopic;
  doc["payload_available"] = "online";
  doc["payload_not_available"] = "offline";
  
  size_t payloadLen = measureJson(doc);
  if (payloadLen >= sizeof(payload)) {
    Serial.printf("[MQTT] Discovery payload too large for %s (%u bytes), skipping.\n", sensorId, (unsigned)payloadLen);
    return;
  }

  payloadLen = serializeJson(doc, payload, sizeof(payload));
  mqtt.publish(topic, (uint8_t*)payload, payloadLen, true);
  
  delay(50);  // Small delay to avoid overwhelming MQTT
}

void publishHADiscovery() {
  if (!mqtt.connected()) return;
  
  Serial.println("Publishing Home Assistant discovery...");
  
  // Voltage sensors
  publishHASensor("voltage_a", "Voltage A", "V", "voltage", "{{ value_json.voltage_a }}");
  publishHASensor("voltage_b", "Voltage B", "V", "voltage", "{{ value_json.voltage_b }}");
  publishHASensor("voltage_c", "Voltage C", "V", "voltage", "{{ value_json.voltage_c }}");
  
  // Current sensors
  publishHASensor("current_a", "Current A", "A", "current", "{{ value_json.current_a }}");
  publishHASensor("current_b", "Current B", "A", "current", "{{ value_json.current_b }}");
  publishHASensor("current_c", "Current C", "A", "current", "{{ value_json.current_c }}");
  
  // Active Power sensors
  publishHASensor("power_a", "Active Power A", "W", "power", "{{ value_json.power_a }}");
  publishHASensor("power_b", "Active Power B", "W", "power", "{{ value_json.power_b }}");
  publishHASensor("power_c", "Active Power C", "W", "power", "{{ value_json.power_c }}");
  publishHASensor("power_total", "Active Power Total", "W", "power", "{{ value_json.power_total }}");
  
  // Reactive Power sensors
  publishHASensor("reactive_a", "Reactive Power A", "var", "reactive_power", "{{ value_json.reactive_a }}");
  publishHASensor("reactive_b", "Reactive Power B", "var", "reactive_power", "{{ value_json.reactive_b }}");
  publishHASensor("reactive_c", "Reactive Power C", "var", "reactive_power", "{{ value_json.reactive_c }}");
  publishHASensor("reactive_total", "Reactive Power Total", "var", "reactive_power", "{{ value_json.reactive_total }}");
  
  // Apparent Power sensors
  publishHASensor("apparent_a", "Apparent Power A", "VA", "apparent_power", "{{ value_json.apparent_a }}");
  publishHASensor("apparent_b", "Apparent Power B", "VA", "apparent_power", "{{ value_json.apparent_b }}");
  publishHASensor("apparent_c", "Apparent Power C", "VA", "apparent_power", "{{ value_json.apparent_c }}");
  publishHASensor("apparent_total", "Apparent Power Total", "VA", "apparent_power", "{{ value_json.apparent_total }}");
  
  // Power Factor sensors
  publishHASensor("pf_a", "Power Factor A", "", "power_factor", "{{ value_json.pf_a }}");
  publishHASensor("pf_b", "Power Factor B", "", "power_factor", "{{ value_json.pf_b }}");
  publishHASensor("pf_c", "Power Factor C", "", "power_factor", "{{ value_json.pf_c }}");
  publishHASensor("pf_total", "Power Factor Total", "", "power_factor", "{{ value_json.pf_total }}");
  
  // Frequency
  publishHASensor("frequency", "Grid Frequency", "Hz", "frequency", "{{ value_json.frequency }}");

  // Transformer ratios (configuration/scaling)
  publishHASensor("pt", "PT Ratio", "ratio", "", "{{ value_json.pt }}", "mdi:transmission-tower");
  publishHASensor("ct", "CT Ratio", "ratio", "", "{{ value_json.ct }}", "mdi:current-ac");
  
  Serial.println("Home Assistant discovery published!");
  haDiscoverySent = true;
}

void publishMeterValues() {
  if (!mqtt.connected() || !meter.valid) return;
  
  char topic[64];
  snprintf(topic, sizeof(topic), "modbus_listener/%s/meter", deviceId);
  
  JsonDocument doc;
  
  // Round values for cleaner JSON
  doc["voltage_a"] = round(meter.voltageA * 10) / 10.0;
  doc["voltage_b"] = round(meter.voltageB * 10) / 10.0;
  doc["voltage_c"] = round(meter.voltageC * 10) / 10.0;
  
  doc["current_a"] = round(meter.currentA * 100) / 100.0;
  doc["current_b"] = round(meter.currentB * 100) / 100.0;
  doc["current_c"] = round(meter.currentC * 100) / 100.0;
  
  doc["power_a"] = meter.activePowerA;
  doc["power_b"] = meter.activePowerB;
  doc["power_c"] = meter.activePowerC;
  doc["power_total"] = meter.activePowerTotal;
  
  doc["reactive_a"] = meter.reactivePowerA;
  doc["reactive_b"] = meter.reactivePowerB;
  doc["reactive_c"] = meter.reactivePowerC;
  doc["reactive_total"] = meter.reactivePowerTotal;
  
  doc["apparent_a"] = meter.apparentPowerA;
  doc["apparent_b"] = meter.apparentPowerB;
  doc["apparent_c"] = meter.apparentPowerC;
  doc["apparent_total"] = meter.apparentPowerTotal;
  
  doc["pf_a"] = round(meter.powerFactorA * 1000) / 1000.0;
  doc["pf_b"] = round(meter.powerFactorB * 1000) / 1000.0;
  doc["pf_c"] = round(meter.powerFactorC * 1000) / 1000.0;
  doc["pf_total"] = round(meter.powerFactorTotal * 1000) / 1000.0;
  
  doc["frequency"] = round(meter.frequency * 100) / 100.0;

  // Scaling parameters (PT/CT). These are updated only when the master reads them.
  if (scaling.valid) {
    doc["pt"] = scaling.pt;
    doc["ct"] = scaling.ct;
  }
  
  char payload[768];
  size_t payloadLen = measureJson(doc);
  if (payloadLen >= sizeof(payload)) {
    Serial.printf("[MQTT] Meter payload too large (%u bytes), skipping publish.\n", (unsigned)payloadLen);
    return;
  }

  payloadLen = serializeJson(doc, payload, sizeof(payload));
  mqtt.publish(topic, (uint8_t*)payload, payloadLen, false);
  
  if (debugMode) {
    Serial.printf("[MQTT] Published meter values: Pt=%dW f=%.2fHz\n", 
                  meter.activePowerTotal, meter.frequency);
  }
}

// =============================================================================
// WEB SERVER FUNCTIONS
// =============================================================================

void setupWebServer() {
  webServer.on("/test", HTTP_GET, []() {
    webServer.send(200, "text/plain", "Web server is working!");
  });
  
  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/config", HTTP_GET, handleConfig);
  webServer.on("/save", HTTP_POST, handleSave);
  webServer.on("/status", HTTP_GET, handleStatus);
  webServer.on("/reboot", HTTP_POST, handleReboot);
  webServer.on("/log", HTTP_GET, handleLog);
  webServer.on("/log/data", HTTP_GET, handleLogData);
  webServer.on("/log/clear", HTTP_POST, handleLogClear);
  webServer.on("/meter", HTTP_GET, handleMeter);
  webServer.on("/meter/data", HTTP_GET, handleMeterData);
  webServer.on("/update", HTTP_GET, handleUpdate);
  webServer.on("/update", HTTP_POST, []() {
    String html = Update.hasError()
      ? "<!DOCTYPE html><html><head><meta charset='utf-8'><style>body{font-family:Arial,sans-serif;margin:40px;text-align:center}.err{color:#ea4335}</style></head><body><h1 class='err'>Update Failed!</h1><p>" + String(Update.errorString()) + "</p><p><a href='/update'>Try again</a></p></body></html>"
      : "<!DOCTYPE html><html><head><meta charset='utf-8'><meta http-equiv='refresh' content='10;url=/'><style>body{font-family:Arial,sans-serif;margin:40px;text-align:center}.ok{color:#34a853}</style></head><body><h1 class='ok'>Update Success!</h1><p>Rebooting device...</p></body></html>";
    webServer.send(200, "text/html", html);
    delay(1000);
    ESP.restart();
  }, handleOtaUpload);
  
  webServer.begin();
  Serial.println("Web server started.");
}

void handleRoot() {
  Serial.println("[WEB] handleRoot called");
  
  String ipAddr = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  String wifiStatus = (WiFi.status() == WL_CONNECTED) ? "<span class='badge ok'>Connected</span>" : "<span class='badge warn'>AP Mode</span>";
  String mqttStatus = mqttConnected ? "<span class='badge ok'>Connected</span>" : "<span class='badge err'>Disconnected</span>";
  String uptimeStr = getUptimeString();
  
  webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webServer.send(200, "text/html", "");
  
  webServer.sendContent(F("<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'><title>ESP32 Modbus Listener</title><style>"));
  webServer.sendContent(F("body{font-family:Arial,sans-serif;margin:20px;background:#f0f2f5;color:#333}"));
  webServer.sendContent(F("h1{color:#1a73e8;margin-bottom:10px}"));
  webServer.sendContent(F(".card{background:#fff;border-radius:8px;padding:20px;margin:15px 0;box-shadow:0 2px 4px rgba(0,0,0,0.1)}"));
  webServer.sendContent(F(".badge{display:inline-block;padding:4px 12px;border-radius:12px;font-size:12px;margin:2px}"));
  webServer.sendContent(F(".ok{background:#34a853;color:#fff}.warn{background:#fbbc04;color:#333}.err{background:#ea4335;color:#fff}"));
  webServer.sendContent(F("table{width:100%;border-collapse:collapse;margin:10px 0}"));
  webServer.sendContent(F("th,td{padding:8px 12px;text-align:left;border-bottom:1px solid #e0e0e0}"));
  webServer.sendContent(F("th{background:#f8f9fa;font-weight:600}"));
  webServer.sendContent(F("a.btn{display:inline-block;padding:10px 20px;background:#1a73e8;color:#fff;text-decoration:none;border-radius:4px;margin:5px}"));
  webServer.sendContent(F("a.btn:hover{background:#1557b0}"));
  webServer.sendContent(F("</style></head><body>"));
  webServer.sendContent(F("<h1>ESP32 Modbus Listener</h1>"));
  webServer.sendContent(F("<div class='card'><h2>Status</h2><table>"));
  webServer.sendContent("<tr><th>Device ID</th><td>" + String(deviceId) + "</td></tr>");
  webServer.sendContent("<tr><th>IP Address</th><td>" + ipAddr + "</td></tr>");
  webServer.sendContent("<tr><th>WiFi Status</th><td>" + wifiStatus + "</td></tr>");
  webServer.sendContent("<tr><th>MQTT Status</th><td>" + mqttStatus + "</td></tr>");
  webServer.sendContent("<tr><th>Uptime</th><td>" + uptimeStr + "</td></tr>");
  webServer.sendContent(F("</table></div>"));
  webServer.sendContent(F("<div class='card'><h2>Navigation</h2>"));
  webServer.sendContent(F("<a class='btn' style='background:#34a853' href='/meter'>Live Meter</a>"));
  webServer.sendContent(F("<a class='btn' href='/log'>Modbus Log</a>"));
  webServer.sendContent(F("<a class='btn' href='/config'>Configuration</a>"));
  webServer.sendContent(F("<a class='btn' href='/status'>JSON Status</a>"));
  webServer.sendContent(F("<a class='btn' href='/update'>Firmware Update</a>"));
  webServer.sendContent(F("</div></body></html>"));
  webServer.sendContent("");
  
  Serial.println("[WEB] handleRoot completed");
}

void handleConfig() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset='utf-8'>
  <meta name='viewport' content='width=device-width,initial-scale=1'>
  <title>Configuration - ESP32 Modbus Listener</title>
  <style>
    body{font-family:Arial,sans-serif;margin:20px;background:#f0f2f5;color:#333}
    h1{color:#1a73e8}
    h2{margin-top:20px;padding-top:15px;border-top:1px solid #e0e0e0}
    .card{background:#fff;border-radius:8px;padding:20px;margin:15px 0;box-shadow:0 2px 4px rgba(0,0,0,0.1)}
    label{display:block;margin:10px 0 5px;font-weight:600}
    input[type=text],input[type=password],input[type=number]{width:100%;padding:10px;border:1px solid #ddd;border-radius:4px;box-sizing:border-box}
    button{padding:12px 24px;background:#1a73e8;color:#fff;border:none;border-radius:4px;cursor:pointer;margin:5px}
    button:hover{background:#1557b0}
    button.danger{background:#ea4335}
    button.danger:hover{background:#c62828}
    a{color:#1a73e8}
  </style>
</head>
<body>
  <h1>Configuration</h1>
  <p><a href='/'>&#8592; Back to Dashboard</a></p>
  
  <form method='POST' action='/save'>
    <div class='card'>
      <h2>WiFi Settings</h2>
      <label>SSID</label>
      <input type='text' name='wifi_ssid' value=')" + String(config.wifi_ssid) + R"(' maxlength='32'>
      <label>Password</label>
      <input type='password' name='wifi_pass' value=')" + String(config.wifi_password) + R"(' maxlength='64'>
    </div>
    
    <div class='card'>
      <h2>MQTT Settings</h2>
      <label>Server</label>
      <input type='text' name='mqtt_server' value=')" + String(config.mqtt_server) + R"(' maxlength='64'>
      <label>Port</label>
      <input type='number' name='mqtt_port' value=')" + String(config.mqtt_port) + R"(' min='1' max='65535'>
      <label>Username</label>
      <input type='text' name='mqtt_user' value=')" + String(config.mqtt_username) + R"(' maxlength='32'>
      <label>Password</label>
      <input type='password' name='mqtt_pass' value=')" + String(config.mqtt_password) + R"(' maxlength='32'>
    </div>
    
    <div class='card'>
      <h2>Modbus Settings</h2>
      <label>Baud Rate</label>
      <input type='number' name='modbus_baud' value=')" + String(config.modbus_baud) + R"(' min='1200' max='115200'>
      <label>Slave Address</label>
      <input type='number' name='modbus_addr' value=')" + String(config.modbus_slave_address) + R"(' min='1' max='247'>
    </div>
    
    <div class='card'>
      <button type='submit'>Save Configuration</button>
    </div>
  </form>
  
  <form method='POST' action='/reboot'>
    <div class='card'>
      <h2>Device Control</h2>
      <button type='submit' class='danger'>Reboot Device</button>
    </div>
  </form>
</body>
</html>
)";
  
  webServer.send(200, "text/html", html);
}

void handleSave() {
  // Get WiFi settings
  if (webServer.hasArg("wifi_ssid")) {
    String ssid = webServer.arg("wifi_ssid");
    ssid.trim();
    snprintf(config.wifi_ssid, sizeof(config.wifi_ssid), "%s", ssid.c_str());
  }
  if (webServer.hasArg("wifi_pass")) {
    String wifiPass = webServer.arg("wifi_pass");
    snprintf(config.wifi_password, sizeof(config.wifi_password), "%s", wifiPass.c_str());
  }
  
  // Get MQTT settings
  if (webServer.hasArg("mqtt_server")) {
    strncpy(config.mqtt_server, webServer.arg("mqtt_server").c_str(), sizeof(config.mqtt_server) - 1);
  }
  if (webServer.hasArg("mqtt_port")) {
    config.mqtt_port = webServer.arg("mqtt_port").toInt();
  }
  if (webServer.hasArg("mqtt_user")) {
    strncpy(config.mqtt_username, webServer.arg("mqtt_user").c_str(), sizeof(config.mqtt_username) - 1);
  }
  if (webServer.hasArg("mqtt_pass")) {
    strncpy(config.mqtt_password, webServer.arg("mqtt_pass").c_str(), sizeof(config.mqtt_password) - 1);
  }
  
  // Get Modbus settings
  if (webServer.hasArg("modbus_baud")) {
    config.modbus_baud = webServer.arg("modbus_baud").toInt();
  }
  if (webServer.hasArg("modbus_addr")) {
    config.modbus_slave_address = webServer.arg("modbus_addr").toInt();
  }
  
  // Save configuration
  saveConfiguration();
  
  String html = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset='utf-8'>
  <meta http-equiv='refresh' content='3;url=/'>
  <title>Configuration Saved</title>
  <style>body{font-family:Arial,sans-serif;margin:40px;text-align:center}</style>
</head>
<body>
  <h1>Configuration Saved!</h1>
  <p>Redirecting to dashboard in 3 seconds...</p>
  <p><a href='/'>Click here if not redirected</a></p>
</body>
</html>
)";
  
  webServer.send(200, "text/html", html);
}

void handleStatus() {
  JsonDocument doc;
  
  doc["device_id"] = deviceId;
  doc["uptime_ms"] = millis() - bootTime;
  doc["wifi_connected"] = (WiFi.status() == WL_CONNECTED);
  doc["ip_address"] = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  doc["mqtt_connected"] = mqttConnected;
  doc["modbus_baud"] = config.modbus_baud;
  doc["total_frames"] = totalFrames;
  doc["valid_frames"] = validFrames;
  
  String response;
  serializeJsonPretty(doc, response);
  
  webServer.send(200, "application/json", response);
}

void handleReboot() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset='utf-8'>
  <meta http-equiv='refresh' content='10;url=/'>
  <title>Rebooting...</title>
  <style>body{font-family:Arial,sans-serif;margin:40px;text-align:center}</style>
</head>
<body>
  <h1>Rebooting...</h1>
  <p>The device will restart. Please wait...</p>
</body>
</html>
)";
  
  webServer.send(200, "text/html", html);
  delay(1000);
  ESP.restart();
}

void handleLog() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset='utf-8'>
  <meta name='viewport' content='width=device-width,initial-scale=1'>
  <title>Modbus Log - ESP32 Modbus Listener</title>
  <style>
    body{font-family:Arial,sans-serif;margin:20px;background:#f0f2f5;color:#333}
    h1{color:#1a73e8}
    .card{background:#fff;border-radius:8px;padding:20px;margin:15px 0;box-shadow:0 2px 4px rgba(0,0,0,0.1)}
    .stats{display:flex;gap:20px;flex-wrap:wrap}
    .stat{background:#e8f0fe;padding:15px;border-radius:8px;text-align:center;min-width:100px}
    .stat-value{font-size:24px;font-weight:bold;color:#1a73e8}
    .stat-label{font-size:12px;color:#666}
    table{width:100%;border-collapse:collapse;font-size:13px}
    th,td{padding:8px;text-align:left;border-bottom:1px solid #e0e0e0}
    th{background:#f8f9fa;font-weight:600;position:sticky;top:0}
    .log-container{max-height:500px;overflow-y:auto}
    .hex{font-family:monospace;background:#f5f5f5;padding:2px 4px;border-radius:2px;word-break:break-all}
    .ok{color:#34a853}
    .err{color:#ea4335}
    .func{background:#e3f2fd;padding:2px 6px;border-radius:10px;font-size:11px}
    a{color:#1a73e8}
    button{padding:8px 16px;background:#1a73e8;color:#fff;border:none;border-radius:4px;cursor:pointer;margin:5px}
    button:hover{background:#1557b0}
    button.danger{background:#ea4335}
    button.danger:hover{background:#c62828}
    .controls{display:flex;gap:10px;align-items:center;flex-wrap:wrap}
    select,input[type=checkbox]{margin-left:5px}
    label{display:flex;align-items:center;gap:5px}
  </style>
</head>
<body>
  <h1>Modbus Log</h1>
  <p><a href='/'>&#8592; Back to Dashboard</a></p>
  
  <div class='card'>
    <h2>Statistics</h2>
    <div class='stats'>
      <div class='stat'>
        <div class='stat-value' id='totalFrames'>0</div>
        <div class='stat-label'>Total Frames</div>
      </div>
      <div class='stat'>
        <div class='stat-value' id='validFrames'>0</div>
        <div class='stat-label'>Valid CRC</div>
      </div>
      <div class='stat'>
        <div class='stat-value' id='errorRate'>0%</div>
        <div class='stat-label'>Error Rate</div>
      </div>
      <div class='stat'>
        <div class='stat-value' id='logCount'>0</div>
        <div class='stat-label'>In Buffer</div>
      </div>
      <div class='stat'>
        <div class='stat-value' id='bytesReceived'>0</div>
        <div class='stat-label'>Raw Bytes (debug)</div>
      </div>
    </div>
  </div>
  
  <div class='card'>
    <h2>Controls</h2>
    <div class='controls'>
      <label><input type='checkbox' id='autoRefresh' checked> Auto-refresh</label>
      <select id='refreshRate'>
        <option value='1000'>1 sec</option>
        <option value='2000' selected>2 sec</option>
        <option value='5000'>5 sec</option>
      </select>
      <button onclick='refreshLog()'>Refresh Now</button>
      <form method='POST' action='/log/clear' style='display:inline'>
        <button type='submit' class='danger'>Clear Log</button>
      </form>
    </div>
  </div>
  
  <div class='card'>
    <h2>Recent Messages</h2>
    <div class='log-container'>
      <table>
        <thead>
          <tr>
            <th>Time</th>
            <th>Addr</th>
            <th>Function</th>
            <th>Data (hex)</th>
            <th>Len</th>
            <th>CRC</th>
          </tr>
        </thead>
        <tbody id='logBody'>
          <tr><td colspan='6'>Loading...</td></tr>
        </tbody>
      </table>
    </div>
  </div>
  
  <script>
    var refreshTimer;
    
    function formatTime(ms) {
      var sec = Math.floor(ms / 1000);
      var min = Math.floor(sec / 60);
      var hr = Math.floor(min / 60);
      return String(hr).padStart(2,'0') + ':' + 
             String(min % 60).padStart(2,'0') + ':' + 
             String(sec % 60).padStart(2,'0') + '.' +
             String(ms % 1000).padStart(3,'0');
    }
    
    function refreshLog() {
      fetch('/log/data')
        .then(r => r.json())
        .then(data => {
          document.getElementById('totalFrames').textContent = data.total;
          document.getElementById('validFrames').textContent = data.valid;
          document.getElementById('logCount').textContent = data.count;
          document.getElementById('bytesReceived').textContent = data.bytes;
          var errRate = data.total > 0 ? ((data.total - data.valid) / data.total * 100).toFixed(1) : 0;
          document.getElementById('errorRate').textContent = errRate + '%';
          
          var html = '';
          if (data.entries.length === 0) {
            html = '<tr><td colspan="6" style="text-align:center;color:#666">No messages yet</td></tr>';
          } else {
            data.entries.forEach(function(e) {
              html += '<tr>';
              html += '<td>' + formatTime(e.ts) + '</td>';
              html += '<td>' + e.addr + '</td>';
              html += '<td><span class="func">' + e.func + '</span></td>';
              html += '<td><span class="hex">' + e.hex + '</span></td>';
              html += '<td>' + e.len + '</td>';
              html += '<td class="' + (e.crc ? 'ok' : 'err') + '">' + (e.crc ? '&#10003;' : '&#10007;') + '</td>';
              html += '</tr>';
            });
          }
          document.getElementById('logBody').innerHTML = html;
        })
        .catch(err => console.error('Error:', err));
    }
    
    function updateAutoRefresh() {
      clearInterval(refreshTimer);
      if (document.getElementById('autoRefresh').checked) {
        var rate = parseInt(document.getElementById('refreshRate').value);
        refreshTimer = setInterval(refreshLog, rate);
      }
    }
    
    document.getElementById('autoRefresh').addEventListener('change', updateAutoRefresh);
    document.getElementById('refreshRate').addEventListener('change', updateAutoRefresh);
    
    refreshLog();
    updateAutoRefresh();
  </script>
</body>
</html>
)";
  
  webServer.send(200, "text/html", html);
}

void handleLogData() {
  JsonDocument doc;
  
  doc["total"] = totalFrames;
  doc["valid"] = validFrames;
  doc["count"] = logCount;
  doc["bytes"] = totalBytesReceived;
  doc["debug"] = debugMode;
  
  JsonArray entries = doc["entries"].to<JsonArray>();
  
  // Return entries from newest to oldest
  for (int i = 0; i < logCount; i++) {
    int idx = (logHead - 1 - i + LOG_ENTRY_COUNT) % LOG_ENTRY_COUNT;
    LogEntry& entry = modbusLog[idx];
    
    JsonObject obj = entries.add<JsonObject>();
    obj["ts"] = entry.timestamp;
    obj["addr"] = entry.slaveAddr;
    obj["func"] = getFunctionName(entry.functionCode);
    obj["len"] = entry.length;
    obj["crc"] = entry.crcValid;
    
    // Create hex string
    String hexStr = "";
    for (int j = 0; j < entry.length && j < LOG_DATA_SIZE; j++) {
      if (entry.data[j] < 0x10) hexStr += "0";
      hexStr += String(entry.data[j], HEX);
      if (j < entry.length - 1) hexStr += " ";
    }
    hexStr.toUpperCase();
    obj["hex"] = hexStr;
  }
  
  String response;
  serializeJson(doc, response);
  webServer.send(200, "application/json", response);
}

void handleLogClear() {
  logHead = 0;
  logCount = 0;
  totalFrames = 0;
  validFrames = 0;
  
  webServer.sendHeader("Location", "/log");
  webServer.send(303);
}

// =============================================================================
// LIVE METER PAGE
// =============================================================================

void handleMeter() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset='utf-8'>
  <meta name='viewport' content='width=device-width,initial-scale=1'>
  <title>Live Meter - ESP32 Modbus Listener</title>
  <style>
    body{font-family:Arial,sans-serif;margin:20px;background:#f0f2f5;color:#333}
    h1{color:#1a73e8;text-align:center}
    h2{color:#333;border-bottom:2px solid #1a73e8;padding-bottom:5px;margin-top:0}
    h3{color:#666;margin:15px 0 10px;font-size:14px}
    .card{background:#fff;border-radius:8px;padding:20px;margin:15px auto;max-width:900px;box-shadow:0 2px 4px rgba(0,0,0,0.1)}
    a{color:#1a73e8;text-decoration:none}
    a:hover{text-decoration:underline}
    .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:15px;margin:15px 0}
    .value-box{background:#f8f9fa;border-radius:8px;padding:15px;text-align:center;border-left:4px solid #1a73e8}
    .value-box.phase-a{border-color:#e53935}
    .value-box.phase-b{border-color:#43a047}
    .value-box.phase-c{border-color:#1e88e5}
    .value-box.total{border-color:#8e24aa}
    .value-box .label{font-size:12px;color:#666;text-transform:uppercase}
    .value-box .value{font-size:28px;font-weight:bold;color:#333;margin:5px 0}
    .value-box .unit{font-size:14px;color:#888}
    .status{padding:8px 15px;border-radius:20px;display:inline-block;font-size:12px;margin-bottom:15px}
    .status.online{background:#e8f5e9;color:#2e7d32}
    .status.offline{background:#ffebee;color:#c62828}
    .alert{background:#fff3cd;border:1px solid #ffc107;padding:10px;border-radius:4px;margin:10px 0}
    .alert.new{background:#ffe0e0;border-color:#ff5722}
    .reg-table{width:100%;border-collapse:collapse;margin:10px 0;font-size:13px}
    .reg-table th,.reg-table td{padding:8px;text-align:left;border-bottom:1px solid #eee}
    .reg-table th{background:#f5f5f5;color:#666}
    .reg-table .new{background:#fff8e1}
    .reg-table .expected{background:#e8f5e9}
    .update-time{font-size:12px;color:#888;text-align:center;margin-top:15px}
    .freq-box{background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);color:white;padding:20px;border-radius:8px;text-align:center;margin:15px 0}
    .freq-box .value{font-size:48px;font-weight:bold}
    .freq-box .label{font-size:14px;opacity:0.9}
    .btn{display:inline-block;padding:8px 16px;background:#1a73e8;color:#fff;text-decoration:none;border-radius:4px;margin:5px}
    .btn:hover{background:#1557b0;text-decoration:none}
  </style>
</head>
<body>
  <h1>Live Meter</h1>
  <p style='text-align:center'><a href='/'>&#8592; Back to Dashboard</a> | <a href='/log'>View Raw Log</a></p>
  
  <div class='card'>
    <h2>Connection Status</h2>
    <div id='status' class='status offline'>Waiting for data...</div>
    <div id='lastUpdate' class='update-time'>No data received yet</div>
  </div>
  
  <div class='card'>
    <h2>Frequency</h2>
    <div class='freq-box'>
      <div class='label'>Grid Frequency</div>
      <div class='value'><span id='freq'>--.-</span> <span style='font-size:24px'>Hz</span></div>
    </div>
  </div>

  <div class='card'>
    <h2>Transformer Ratios</h2>
    <div class='grid'>
      <div class='value-box total'>
        <div class='label'>PT</div>
        <div class='value' id='pt'>--</div>
        <div class='unit'>ratio</div>
      </div>
      <div class='value-box total'>
        <div class='label'>CT</div>
        <div class='value' id='ct'>--</div>
        <div class='unit'>ratio</div>
      </div>
    </div>
    <div class='update-time' id='scalingUpdate'>Not read yet</div>
  </div>
  
  <div class='card'>
    <h2>Voltage</h2>
    <div class='grid'>
      <div class='value-box phase-a'>
        <div class='label'>Phase A</div>
        <div class='value' id='vA'>---.-</div>
        <div class='unit'>V</div>
      </div>
      <div class='value-box phase-b'>
        <div class='label'>Phase B</div>
        <div class='value' id='vB'>---.-</div>
        <div class='unit'>V</div>
      </div>
      <div class='value-box phase-c'>
        <div class='label'>Phase C</div>
        <div class='value' id='vC'>---.-</div>
        <div class='unit'>V</div>
      </div>
    </div>
  </div>
  
  <div class='card'>
    <h2>Current</h2>
    <div class='grid'>
      <div class='value-box phase-a'>
        <div class='label'>Phase A</div>
        <div class='value' id='iA'>--.-</div>
        <div class='unit'>A</div>
      </div>
      <div class='value-box phase-b'>
        <div class='label'>Phase B</div>
        <div class='value' id='iB'>--.-</div>
        <div class='unit'>A</div>
      </div>
      <div class='value-box phase-c'>
        <div class='label'>Phase C</div>
        <div class='value' id='iC'>--.-</div>
        <div class='unit'>A</div>
      </div>
    </div>
  </div>
  
  <div class='card'>
    <h2>Active Power</h2>
    <div class='grid'>
      <div class='value-box phase-a'>
        <div class='label'>Phase A</div>
        <div class='value' id='pA'>----</div>
        <div class='unit'>W</div>
      </div>
      <div class='value-box phase-b'>
        <div class='label'>Phase B</div>
        <div class='value' id='pB'>----</div>
        <div class='unit'>W</div>
      </div>
      <div class='value-box phase-c'>
        <div class='label'>Phase C</div>
        <div class='value' id='pC'>----</div>
        <div class='unit'>W</div>
      </div>
      <div class='value-box total'>
        <div class='label'>Total</div>
        <div class='value' id='pT'>----</div>
        <div class='unit'>W</div>
      </div>
    </div>
  </div>
  
  <div class='card'>
    <h2>Reactive Power</h2>
    <div class='grid'>
      <div class='value-box phase-a'>
        <div class='label'>Phase A</div>
        <div class='value' id='qA'>----</div>
        <div class='unit'>VAr</div>
      </div>
      <div class='value-box phase-b'>
        <div class='label'>Phase B</div>
        <div class='value' id='qB'>----</div>
        <div class='unit'>VAr</div>
      </div>
      <div class='value-box phase-c'>
        <div class='label'>Phase C</div>
        <div class='value' id='qC'>----</div>
        <div class='unit'>VAr</div>
      </div>
      <div class='value-box total'>
        <div class='label'>Total</div>
        <div class='value' id='qT'>----</div>
        <div class='unit'>VAr</div>
      </div>
    </div>
  </div>
  
  <div class='card'>
    <h2>Apparent Power</h2>
    <div class='grid'>
      <div class='value-box phase-a'>
        <div class='label'>Phase A</div>
        <div class='value' id='sA'>----</div>
        <div class='unit'>VA</div>
      </div>
      <div class='value-box phase-b'>
        <div class='label'>Phase B</div>
        <div class='value' id='sB'>----</div>
        <div class='unit'>VA</div>
      </div>
      <div class='value-box phase-c'>
        <div class='label'>Phase C</div>
        <div class='value' id='sC'>----</div>
        <div class='unit'>VA</div>
      </div>
      <div class='value-box total'>
        <div class='label'>Total</div>
        <div class='value' id='sT'>----</div>
        <div class='unit'>VA</div>
      </div>
    </div>
  </div>
  
  <div class='card'>
    <h2>Power Factor</h2>
    <div class='grid'>
      <div class='value-box phase-a'>
        <div class='label'>Phase A</div>
        <div class='value' id='pfA'>-.---</div>
        <div class='unit'></div>
      </div>
      <div class='value-box phase-b'>
        <div class='label'>Phase B</div>
        <div class='value' id='pfB'>-.---</div>
        <div class='unit'></div>
      </div>
      <div class='value-box phase-c'>
        <div class='label'>Phase C</div>
        <div class='value' id='pfC'>-.---</div>
        <div class='unit'></div>
      </div>
      <div class='value-box total'>
        <div class='label'>Total</div>
        <div class='value' id='pfT'>-.---</div>
        <div class='unit'></div>
      </div>
    </div>
  </div>
  
  <div class='card'>
    <h2>Register Access Monitor</h2>
    <div id='alertArea'></div>
    <table class='reg-table'>
      <thead>
        <tr><th>Address</th><th>Count</th><th>Accesses</th><th>Last Seen</th><th>Status</th></tr>
      </thead>
      <tbody id='regTable'></tbody>
    </table>
  </div>
  
  <script>
    let deviceUptime = 0;
    
    function formatTime(timestamp) {
      if (!timestamp) return 'Never';
      const sec = Math.floor((deviceUptime - timestamp) / 1000);
      if (sec < 0) return 'Just now';
      if (sec < 60) return sec + 's ago';
      if (sec < 3600) return Math.floor(sec/60) + 'm ago';
      return Math.floor(sec/3600) + 'h ago';
    }
    
    function updateMeter() {
      fetch('/meter/data')
        .then(r => r.json())
        .then(d => {
          deviceUptime = d.uptime;
          
          const statusEl = document.getElementById('status');
          const lastUpEl = document.getElementById('lastUpdate');
          if (d.meter.valid) {
            statusEl.className = 'status online';
            statusEl.textContent = 'Receiving meter data';
            lastUpEl.textContent = 'Last update: ' + formatTime(d.meter.lastUpdate);
          } else {
            statusEl.className = 'status offline';
            statusEl.textContent = 'Waiting for meter data...';
          }
          
          document.getElementById('freq').textContent = d.meter.frequency.toFixed(2);
          document.getElementById('vA').textContent = d.meter.voltageA.toFixed(1);
          document.getElementById('vB').textContent = d.meter.voltageB.toFixed(1);
          document.getElementById('vC').textContent = d.meter.voltageC.toFixed(1);
          document.getElementById('iA').textContent = d.meter.currentA.toFixed(2);
          document.getElementById('iB').textContent = d.meter.currentB.toFixed(2);
          document.getElementById('iC').textContent = d.meter.currentC.toFixed(2);
          document.getElementById('pA').textContent = d.meter.activePowerA;
          document.getElementById('pB').textContent = d.meter.activePowerB;
          document.getElementById('pC').textContent = d.meter.activePowerC;
          document.getElementById('pT').textContent = d.meter.activePowerTotal;
          document.getElementById('qA').textContent = d.meter.reactivePowerA;
          document.getElementById('qB').textContent = d.meter.reactivePowerB;
          document.getElementById('qC').textContent = d.meter.reactivePowerC;
          document.getElementById('qT').textContent = d.meter.reactivePowerTotal;
          document.getElementById('sA').textContent = d.meter.apparentPowerA;
          document.getElementById('sB').textContent = d.meter.apparentPowerB;
          document.getElementById('sC').textContent = d.meter.apparentPowerC;
          document.getElementById('sT').textContent = d.meter.apparentPowerTotal;
          document.getElementById('pfA').textContent = d.meter.powerFactorA.toFixed(3);
          document.getElementById('pfB').textContent = d.meter.powerFactorB.toFixed(3);
          document.getElementById('pfC').textContent = d.meter.powerFactorC.toFixed(3);
          document.getElementById('pfT').textContent = d.meter.powerFactorTotal.toFixed(3);

          if (d.scaling && d.scaling.valid) {
            document.getElementById('pt').textContent = d.scaling.pt;
            document.getElementById('ct').textContent = d.scaling.ct;
            document.getElementById('scalingUpdate').textContent = 'Last read: ' + formatTime(d.scaling.lastUpdate);
          } else {
            document.getElementById('pt').textContent = '--';
            document.getElementById('ct').textContent = '--';
            document.getElementById('scalingUpdate').textContent = 'Not read yet';
          }
          
          let regHtml = '';
          let alertHtml = '';
          for (const r of d.registers) {
            const cls = r.isNew ? 'new' : (r.isExpected ? 'expected' : '');
            let status = 'NEW';
            if (r.isExpected) status = 'Expected';
            else if (!r.isNew) status = 'Known';
            
            regHtml += '<tr class="'+cls+'"><td>0x'+r.startAddr.toString(16).toUpperCase().padStart(4,'0')+'</td>';
            regHtml += '<td>'+r.count+'</td><td>'+r.accessCount+'</td>';
            regHtml += '<td>'+formatTime(r.lastSeen)+'</td>';
            regHtml += '<td>'+status+'</td></tr>';
            
            if (r.isNew) {
              alertHtml += '<div class="alert new">New register access detected: 0x'+r.startAddr.toString(16).toUpperCase().padStart(4,'0')+' ('+r.count+' registers)</div>';
            }
          }
          document.getElementById('regTable').innerHTML = regHtml || '<tr><td colspan="5">No register accesses logged</td></tr>';
          document.getElementById('alertArea').innerHTML = alertHtml;
          
          if (d.newAlerts > 0) {
            document.title = '('+d.newAlerts+') Live Meter - ESP32 Modbus Listener';
          } else {
            document.title = 'Live Meter - ESP32 Modbus Listener';
          }
        })
        .catch(e => {
          document.getElementById('status').className = 'status offline';
          document.getElementById('status').textContent = 'Connection error';
        });
    }
    
    updateMeter();
    setInterval(updateMeter, 1000);
  </script>
</body>
</html>
)";
  
  webServer.send(200, "text/html", html);
}

void handleMeterData() {
  JsonDocument doc;
  
  // Meter values
  JsonObject m = doc["meter"].to<JsonObject>();
  m["valid"] = meter.valid;
  m["lastUpdate"] = meter.lastUpdate;
  m["frequency"] = meter.frequency;
  m["voltageA"] = meter.voltageA;
  m["voltageB"] = meter.voltageB;
  m["voltageC"] = meter.voltageC;
  m["currentA"] = meter.currentA;
  m["currentB"] = meter.currentB;
  m["currentC"] = meter.currentC;
  m["activePowerA"] = meter.activePowerA;
  m["activePowerB"] = meter.activePowerB;
  m["activePowerC"] = meter.activePowerC;
  m["activePowerTotal"] = meter.activePowerTotal;
  m["reactivePowerA"] = meter.reactivePowerA;
  m["reactivePowerB"] = meter.reactivePowerB;
  m["reactivePowerC"] = meter.reactivePowerC;
  m["reactivePowerTotal"] = meter.reactivePowerTotal;
  m["apparentPowerA"] = meter.apparentPowerA;
  m["apparentPowerB"] = meter.apparentPowerB;
  m["apparentPowerC"] = meter.apparentPowerC;
  m["apparentPowerTotal"] = meter.apparentPowerTotal;
  m["powerFactorA"] = meter.powerFactorA;
  m["powerFactorB"] = meter.powerFactorB;
  m["powerFactorC"] = meter.powerFactorC;
  m["powerFactorTotal"] = meter.powerFactorTotal;

  // Scaling parameters (PT/CT)
  JsonObject s = doc["scaling"].to<JsonObject>();
  s["valid"] = scaling.valid;
  s["lastUpdate"] = scaling.lastUpdate;
  s["pt"] = scaling.pt;
  s["ct"] = scaling.ct;
  
  // Register access tracking
  JsonArray regs = doc["registers"].to<JsonArray>();
  for (int i = 0; i < knownRegisterCount; i++) {
    JsonObject r = regs.add<JsonObject>();
    r["startAddr"] = knownRegisters[i].startAddr;
    r["count"] = knownRegisters[i].count;
    r["lastSeen"] = knownRegisters[i].lastSeen;
    r["accessCount"] = knownRegisters[i].accessCount;
    r["isNew"] = knownRegisters[i].isNew;
    r["isExpected"] = knownRegisters[i].isExpected;
  }
  
  doc["newAlerts"] = newRegisterAlerts;
  doc["uptime"] = millis();
  
  String response;
  serializeJson(doc, response);
  webServer.send(200, "application/json", response);
}

// =============================================================================
// MODBUS PARSING AND METER DATA EXTRACTION
// =============================================================================

void trackRegisterAccess(uint16_t startAddr, uint16_t count) {
  // Check if this register range is already known
  for (int i = 0; i < knownRegisterCount; i++) {
    if (knownRegisters[i].startAddr == startAddr && knownRegisters[i].count == count) {
      knownRegisters[i].lastSeen = millis();
      knownRegisters[i].accessCount++;
      return;
    }
  }

  // Coalesce: if this request is fully covered by an existing known/expected range,
  // count it under the covering range rather than creating a duplicate row.
  for (int i = 0; i < knownRegisterCount; i++) {
    uint32_t knownStart = knownRegisters[i].startAddr;
    uint32_t knownEnd = knownStart + knownRegisters[i].count;
    uint32_t reqStart = startAddr;
    uint32_t reqEnd = reqStart + count;

    if (knownRegisters[i].isExpected && reqStart >= knownStart && reqEnd <= knownEnd) {
      knownRegisters[i].lastSeen = millis();
      knownRegisters[i].accessCount++;
      return;
    }
  }
  
  // New register range discovered
  if (knownRegisterCount < MAX_REGISTER_RANGES) {
    knownRegisters[knownRegisterCount].startAddr = startAddr;
    knownRegisters[knownRegisterCount].count = count;
    knownRegisters[knownRegisterCount].lastSeen = millis();
    knownRegisters[knownRegisterCount].accessCount = 1;
    knownRegisters[knownRegisterCount].isNew = true;
    knownRegisters[knownRegisterCount].isExpected = false;
    knownRegisterCount++;
    newRegisterAlerts++;
    
    if (debugMode) {
      Serial.printf("[METER] New register access: 0x%04X, count=%d\n", startAddr, count);
    }
  }
}

void parseMeterResponse(uint8_t* data, size_t len) {
  // Response format: [addr][func][byteCount][data...][crcL][crcH]
  if (len < 5) return;
  
  uint8_t funcCode = data[1];
  if (funcCode != 0x03) return;
  
  uint8_t byteCount = data[2];
  
  // Parse ADL400 real-time data (23 registers = 46 bytes)
  if (lastRequestAddr == 0x0061 && lastRequestCount == 23 && byteCount == 46) {
    uint8_t* d = &data[3];
    
    // Voltages (registers 0-2) - scale 0.1V
    meter.voltageA = ((d[0] << 8) | d[1]) * 0.1;
    meter.voltageB = ((d[2] << 8) | d[3]) * 0.1;
    meter.voltageC = ((d[4] << 8) | d[5]) * 0.1;
    
    // Currents (registers 3-5) - scale 0.01A
    meter.currentA = ((d[6] << 8) | d[7]) * 0.01;
    meter.currentB = ((d[8] << 8) | d[9]) * 0.01;
    meter.currentC = ((d[10] << 8) | d[11]) * 0.01;
    
    // Active Power (registers 6-9) - signed, unit: W
    meter.activePowerA = (int16_t)((d[12] << 8) | d[13]);
    meter.activePowerB = (int16_t)((d[14] << 8) | d[15]);
    meter.activePowerC = (int16_t)((d[16] << 8) | d[17]);
    meter.activePowerTotal = (int16_t)((d[18] << 8) | d[19]);
    
    // Reactive Power (registers 10-13) - signed, unit: VAr
    meter.reactivePowerA = (int16_t)((d[20] << 8) | d[21]);
    meter.reactivePowerB = (int16_t)((d[22] << 8) | d[23]);
    meter.reactivePowerC = (int16_t)((d[24] << 8) | d[25]);
    meter.reactivePowerTotal = (int16_t)((d[26] << 8) | d[27]);
    
    // Apparent Power (registers 14-17) - unsigned, unit: VA
    meter.apparentPowerA = (d[28] << 8) | d[29];
    meter.apparentPowerB = (d[30] << 8) | d[31];
    meter.apparentPowerC = (d[32] << 8) | d[33];
    meter.apparentPowerTotal = (d[34] << 8) | d[35];
    
    // Power Factor (registers 18-21) - scale 0.001
    meter.powerFactorA = ((int16_t)((d[36] << 8) | d[37])) * 0.001;
    meter.powerFactorB = ((int16_t)((d[38] << 8) | d[39])) * 0.001;
    meter.powerFactorC = ((int16_t)((d[40] << 8) | d[41])) * 0.001;
    meter.powerFactorTotal = ((int16_t)((d[42] << 8) | d[43])) * 0.001;
    
    // Frequency (register 22) - scale 0.01Hz
    meter.frequency = ((d[44] << 8) | d[45]) * 0.01;
    
    meter.lastUpdate = millis();
    meter.valid = true;
    
    if (debugMode) {
      Serial.printf("[METER] Va=%.1fV Vb=%.1fV Vc=%.1fV | Ia=%.2fA Ib=%.2fA Ic=%.2fA | Pt=%dW | f=%.2fHz\n",
        meter.voltageA, meter.voltageB, meter.voltageC,
        meter.currentA, meter.currentB, meter.currentC,
        meter.activePowerTotal, meter.frequency);
    }
  }

  // PT/CT scaling parameters (holding registers 0x008D and 0x008E)
  if (lastRequestAddr == 0x008D && lastRequestCount == 2 && byteCount == 4) {
    uint8_t* d = &data[3];
    scaling.pt = (d[0] << 8) | d[1];
    scaling.ct = (d[2] << 8) | d[3];
    scaling.lastUpdate = millis();
    scaling.valid = true;

    if (debugMode) {
      Serial.printf("[METER] Scaling updated: PT=%u CT=%u\n", scaling.pt, scaling.ct);
    }
  } else if (lastRequestAddr == 0x008D && lastRequestCount == 1 && byteCount == 2) {
    uint8_t* d = &data[3];
    scaling.pt = (d[0] << 8) | d[1];
    scaling.lastUpdate = millis();
    scaling.valid = true;

    if (debugMode) {
      Serial.printf("[METER] Scaling updated: PT=%u\n", scaling.pt);
    }
  } else if (lastRequestAddr == 0x008E && lastRequestCount == 1 && byteCount == 2) {
    uint8_t* d = &data[3];
    scaling.ct = (d[0] << 8) | d[1];
    scaling.lastUpdate = millis();
    scaling.valid = true;

    if (debugMode) {
      Serial.printf("[METER] Scaling updated: CT=%u\n", scaling.ct);
    }
  }
}

void parseModbusFrame(uint8_t* data, size_t len) {
  if (len < 5) return;
  
  uint8_t slaveAddr = data[0];
  uint8_t funcCode = data[1];
  
  if (funcCode == 0x03) {
    // Distinguish request from response:
    // Request: [addr][0x03][startHi][startLo][countHi][countLo][crcL][crcH] = exactly 8 bytes
    // Response: [addr][0x03][byteCount][data...][crcL][crcH] = 5 + byteCount bytes
    
    if (len == 8) {
      // This is a request
      uint16_t startAddr = (data[2] << 8) | data[3];
      uint16_t regCount = (data[4] << 8) | data[5];
      
      if (regCount >= 1 && regCount <= 125) {
        lastRequestAddr = startAddr;
        lastRequestCount = regCount;
        lastRequestTime = millis();
        
        trackRegisterAccess(startAddr, regCount);
        
        if (debugMode) {
          Serial.printf("[MODBUS] Request: slave=%d FC=0x03 addr=0x%04X count=%d\n", 
            slaveAddr, startAddr, regCount);
        }
      }
    } 
    else if (len > 5) {
      // This could be a response
      uint8_t byteCount = data[2];
      
      if (len == (size_t)(5 + byteCount)) {
        parseMeterResponse(data, len);
        
        if (debugMode) {
          Serial.printf("[MODBUS] Response: slave=%d FC=0x03 bytes=%d\n", 
            slaveAddr, byteCount);
        }
      }
    }
  }
}

void handleUpdate() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset='utf-8'>
  <meta name='viewport' content='width=device-width,initial-scale=1'>
  <title>Firmware Update - ESP32 Modbus Listener</title>
  <style>
    body{font-family:Arial,sans-serif;margin:20px;background:#f0f2f5;color:#333}
    h1{color:#1a73e8}
    .card{background:#fff;border-radius:8px;padding:20px;margin:15px auto;max-width:500px;box-shadow:0 2px 4px rgba(0,0,0,0.1)}
    input[type=file]{margin:15px 0;padding:10px;border:2px dashed #ddd;border-radius:4px;width:100%;box-sizing:border-box}
    button{padding:12px 24px;background:#1a73e8;color:#fff;border:none;border-radius:4px;cursor:pointer;width:100%}
    button:hover{background:#1557b0}
    button:disabled{background:#ccc;cursor:not-allowed}
    a{color:#1a73e8}
    .info{background:#e8f0fe;padding:10px;border-radius:4px;margin:10px 0;font-size:14px}
    .progress{display:none;margin:15px 0}
    .progress-bar{height:20px;background:#e0e0e0;border-radius:10px;overflow:hidden}
    .progress-fill{height:100%;background:#1a73e8;width:0%;transition:width 0.3s}
    .progress-text{text-align:center;margin-top:5px;font-size:14px}
  </style>
</head>
<body>
  <h1>Firmware Update</h1>
  <p style='text-align:center'><a href='/'>&#8592; Back to Dashboard</a></p>
  
  <div class='card'>
    <h2>Upload Firmware</h2>
    <div class='info'>
      <strong>Current Version:</strong> 1.0.0<br>
      <strong>Free Space:</strong> )" + String(ESP.getFreeSketchSpace()) + R"( bytes
    </div>
    <form method='POST' action='/update' enctype='multipart/form-data' id='uploadForm'>
      <input type='file' name='firmware' id='firmware' accept='.bin' required>
      <div class='progress' id='progress'>
        <div class='progress-bar'><div class='progress-fill' id='progressFill'></div></div>
        <div class='progress-text' id='progressText'>Uploading...</div>
      </div>
      <button type='submit' id='uploadBtn'>Upload Firmware</button>
    </form>
    <div class='info' style='margin-top:15px'>
      <strong>Instructions:</strong><br>
      1. Build your project in PlatformIO<br>
      2. Find firmware.bin in .pio/build/esp32c3/<br>
      3. Select the file and click Upload<br>
      4. Wait for the device to reboot
    </div>
  </div>
  
  <script>
    document.getElementById('uploadForm').addEventListener('submit', function(e) {
      document.getElementById('progress').style.display = 'block';
      document.getElementById('uploadBtn').disabled = true;
      document.getElementById('uploadBtn').textContent = 'Uploading...';
    });
    document.getElementById('firmware').addEventListener('change', function(e) {
      var file = e.target.files[0];
      if (file) {
        document.getElementById('uploadBtn').textContent = 'Upload ' + file.name + ' (' + Math.round(file.size/1024) + ' KB)';
      }
    });
  </script>
</body>
</html>
)";
  
  webServer.send(200, "text/html", html);
}

void handleOtaUpload() {
  HTTPUpload& upload = webServer.upload();
  
  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("OTA Update Start: %s\n", upload.filename.c_str());
    
    // Stop Modbus processing during update
    modbusSerial.end();
    
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Serial.printf("Update Begin Error: %s\n", Update.errorString());
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Serial.printf("Update Write Error: %s\n", Update.errorString());
    }
    digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("OTA Update Success: %u bytes\n", upload.totalSize);
      digitalWrite(PIN_STATUS_LED, LOW);
    } else {
      Serial.printf("Update End Error: %s\n", Update.errorString());
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    Serial.println("OTA Update Aborted");
  }
}

// =============================================================================
// MODBUS FUNCTIONS
// =============================================================================

void setupModbus() {
  Serial.println("Setting up Modbus serial port...");
  
  // Initialize RS485 serial port on UART1
  modbusSerial.begin(config.modbus_baud, MODBUS_CONFIG, PIN_RS485_RX, -1);
  Serial.printf("RS485: RX=GPIO%d @ %lu baud\n", 
                PIN_RS485_RX, config.modbus_baud);
  
  Serial.println("Passive listener mode - waiting for Modbus data...");
}

void processModbusListener() {
  // Passive monitoring: receive all bus traffic via RS485 adapter
  
  unsigned long currentTime = millis();
  
  // Debug: Print status every 5 seconds
  if (debugMode && (currentTime - lastDebugPrint >= 5000)) {
    Serial.printf("[DEBUG] Bytes received: %lu, Frames: %lu, Buffer: %d bytes\n",
                  totalBytesReceived, totalFrames, modbusBufferLen);
    lastDebugPrint = currentTime;
  }
  
  // Check for available data
  int available = modbusSerial.available();
  if (available > 0) {
    if (debugMode && modbusBufferLen == 0) {
      Serial.printf("[DEBUG] Data incoming: %d bytes available\n", available);
    }

    // Budget the amount of UART work per loop iteration to avoid starving WiFi/WebServer.
    int budget = 256;
    while (modbusSerial.available() && budget-- > 0) {
      uint8_t b = modbusSerial.read();
      totalBytesReceived++;
      
      if (modbusBufferLen < MODBUS_BUFFER_SIZE) {
        modbusBuffer[modbusBufferLen++] = b;
      }
      lastModbusActivity = currentTime;

      if ((budget & 0x1F) == 0) {
        delay(0);
      }
    }
  }
  
  // Check for frame complete (3.5 character times of silence)
  // At 9600 baud, 1 character = ~1.04ms, so 3.5 chars ~ 4ms
  unsigned long frameTimeout = (1000000UL / config.modbus_baud) * 11 * 4 / 1000;
  if (frameTimeout < 4) frameTimeout = 4;
  
  if (modbusBufferLen > 0 && (currentTime - lastModbusActivity) > frameTimeout) {
    // Frame complete, process it
    if (debugMode) {
      Serial.printf("\n[DEBUG] Frame complete: %d bytes, timeout=%lums\n", modbusBufferLen, frameTimeout);
      Serial.print("Modbus frame received (");
      Serial.print(modbusBufferLen);
      Serial.print(" bytes): ");
      for (size_t i = 0; i < modbusBufferLen; i++) {
        if (modbusBuffer[i] < 0x10) Serial.print("0");
        Serial.print(modbusBuffer[i], HEX);
        Serial.print(" ");
        if ((i & 0x1F) == 0x1F) delay(0);
      }
      Serial.println();
    }
    
    // Validate CRC and log the frame
    bool crcOk = (modbusBufferLen >= 4 && validateModbusCRC(modbusBuffer, modbusBufferLen));
    
    // Add to log buffer
    addLogEntry(modbusBuffer, modbusBufferLen, crcOk);
    
    // Parse Modbus frame for meter data extraction
    if (crcOk) {
      parseModbusFrame(modbusBuffer, modbusBufferLen);
    }
    
    if (debugMode) {
      if (crcOk) {
        Serial.println("CRC valid");
      } else {
        Serial.println("CRC invalid or frame too short");
      }
    }
    
    // Reset buffer
    modbusBufferLen = 0;
  }
}

uint16_t calculateCRC16(uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc = crc >> 1;
      }
    }
  }
  
  return crc;
}

bool validateModbusCRC(uint8_t* data, size_t len) {
  if (len < 4) return false;
  
  uint16_t calculated = calculateCRC16(data, len - 2);
  uint16_t received = data[len - 2] | (data[len - 1] << 8);
  
  return calculated == received;
}

void addLogEntry(uint8_t* data, size_t len, bool crcValid) {
  LogEntry& entry = modbusLog[logHead];
  
  entry.timestamp = millis() - bootTime;
  entry.length = min((size_t)LOG_DATA_SIZE, len);
  entry.crcValid = crcValid;
  
  memcpy(entry.data, data, entry.length);
  
  entry.slaveAddr = (len > 0) ? data[0] : 0;
  entry.functionCode = (len > 1) ? data[1] : 0;
  
  logHead = (logHead + 1) % LOG_ENTRY_COUNT;
  if (logCount < LOG_ENTRY_COUNT) logCount++;
  totalFrames++;
  if (crcValid) validFrames++;
}

String getFunctionName(uint8_t funcCode) {
  switch (funcCode) {
    case 0x01: return "01 Read Coils";
    case 0x02: return "02 Read Inputs";
    case 0x03: return "03 Read Holding";
    case 0x04: return "04 Read Input Reg";
    case 0x05: return "05 Write Coil";
    case 0x06: return "06 Write Reg";
    case 0x0F: return "0F Write Coils";
    case 0x10: return "10 Write Regs";
    case 0x17: return "17 Read/Write";
    default:
      if (funcCode & 0x80) {
        return String(funcCode, HEX) + " Exception";
      }
      return String(funcCode, HEX) + " Unknown";
  }
}

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

void blinkLed(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_STATUS_LED, HIGH);
    delay(delayMs);
    digitalWrite(PIN_STATUS_LED, LOW);
    delay(delayMs);
  }
}

String getUptimeString() {
  unsigned long sec = (millis() - bootTime) / 1000;
  unsigned long days = sec / 86400;
  unsigned long hours = (sec % 86400) / 3600;
  unsigned long minutes = (sec % 3600) / 60;
  unsigned long seconds = sec % 60;
  
  char buf[32];
  snprintf(buf, sizeof(buf), "%lud %02lu:%02lu:%02lu", days, hours, minutes, seconds);
  return String(buf);
}
