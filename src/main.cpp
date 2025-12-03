#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <esp_wifi.h>
#include <HTTPClient.h>

// Pin Definitions for ESP32-DOIT-DevKit-V1
#define TRIGGER_PIN 2 // GPIO26
#define ECHO_PIN 4    // GPIO27
// RTC I2C Pins
#define RTC_SDA 21 // GPIO21 (SDA)
#define RTC_SCL 22 // GPIO22 (SCL)
// Add new pin definitions for A01NYUB
#define A01_RX 16 // GPIO16
#define A01_TX 17 // GPIO17

// Constants
#define WDT_TIMEOUT 180 // 3 minutes watchdog timeout
#define CONFIG_FILE "/config.json"
#define DATA_FILE "/data.csv"
#define MAX_CLIENTS 10

#define SERIAL_BUFFER_SIZE 20
String serialBuff[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;

// Add this at the top with other global variables
bool rtcAvailable = false;
bool systemInitialized = false;


// Global Variables
WebServer server(80);
RTC_DS3231 rtc;
String serialBuffer = "";
unsigned long lastMeasurementTime = 0;
unsigned long lastDataSyncTime = 0;
float currentWaterLevelBlok = 0.0;
float currentWaterLevelParit = 0.0;
float currentRawDistance = 0.0;
String connectedClients[MAX_CLIENTS];
int numClients = 0;
bool isOnlineMode = false;
bool hasInternetConnection = false;

// Add sensor type enum
enum SensorType
{
    HCSR04_SENSOR,
    A01NYUB_SENSOR
};

// Add operation mode enum
enum OperationMode
{
    OFFLINE_MODE,
    ONLINE_MODE
};

// Configuration structure
struct Config {
    int stationId;
    String stationName;
    unsigned long measurementInterval;
    float calibrationOffset;
    SensorType sensorType;
    float sensorToBottomDistance;
    float sensorToZeroBlokDistance;
    OperationMode operationMode;
    String wifiSSID;
    String wifiPassword;
    String apiEndpoint;
    String apiToken;
    unsigned long dataSyncInterval; // in milliseconds
    
    struct DateTime {
        int year;
        int month;
        int day;
        int hour;
        int minute;
        int second;

        DateTime() : year(2024), month(1), day(1),
                     hour(0), minute(0), second(0) {}
    } dateTime;

    Config() : stationId(1),
               stationName("Default Station"),
               measurementInterval(12000),
               calibrationOffset(0.0),
               sensorType(HCSR04_SENSOR),
               sensorToBottomDistance(100.0),
               sensorToZeroBlokDistance(50.0),
               operationMode(OFFLINE_MODE),
               wifiSSID(""),
               wifiPassword(""),
               apiEndpoint(""),
               apiToken(""),
               dataSyncInterval(3600000) {} // 1 hour default
} config;

unsigned long startTime = 0;
const unsigned long MINIMUM_INTERVAL = 12000; // 12 seconds in milliseconds

// Function declarations
bool setupSPIFFS();
bool setupRTC();
void setupWiFi();
void setupWebServer();
void handleRoot();
void handleGetData();
void handleDeleteData();
void handleSettings();
void handleCalibration();
void handleSetTime();
void handleSerial();
void handleClients();
void handleGetConfig();
void handleCurrentLevel();
void measureWaterLevel();
String getFormattedDateTime();
bool loadConfig();
bool saveConfig();
void initWatchdog();
void resetWatchdog();
void addToSerialBuffer(const String &message);
void handleRestart();
void handleUptime();
void validateMeasurementInterval();
float readA01NYUB();
float readHCSR04();
bool connectToWiFi();
bool checkInternetConnection();
bool sendDataToAPI(float levelBlok, float levelParit, float rawDistance);
void syncStoredData();
String formatDataAsJSON();
void getStorageInfo();
void getDataFileInfo();
void logDataWithManagement(float levelBlok, float levelParit);
void cleanOldData();
void handleStorageInfo();

// Function to get SPIFFS usage information
void getStorageInfo() {
    size_t totalBytes = SPIFFS.totalBytes();
    size_t usedBytes = SPIFFS.usedBytes();
    size_t freeBytes = totalBytes - usedBytes;
    
    addToSerialBuffer("Storage - Total: " + String(totalBytes/1024) + "KB, " +
                     "Used: " + String(usedBytes/1024) + "KB, " +
                     "Free: " + String(freeBytes/1024) + "KB");
}

// Function to get data file size and record count
void getDataFileInfo() {
    if (SPIFFS.exists(DATA_FILE)) {
        File file = SPIFFS.open(DATA_FILE, "r");
        if (file) {
            size_t fileSize = file.size();
            int recordCount = 0;
            
            // Count lines (records)
            while (file.available()) {
                String line = file.readStringUntil('\n');
                if (line.length() > 0 && !line.startsWith("Station")) {
                    recordCount++;
                }
            }
            file.close();
            
            addToSerialBuffer("Data file - Size: " + String(fileSize/1024) + "KB, " +
                             "Records: " + String(recordCount));
        }
    }
}

// Enhanced data logging with file size management
void logDataWithManagement(float levelBlok, float levelParit) {
    // Check available space before writing
    size_t freeBytes = SPIFFS.totalBytes() - SPIFFS.usedBytes();
    
    // If less than 50KB free, clean old data
    if (freeBytes < 50000) {
        cleanOldData();
    }
    
    // Create the data file with headers if it doesn't exist
    if (!SPIFFS.exists(DATA_FILE)) {
        File file = SPIFFS.open(DATA_FILE, "w");
        if (file) {
            file.println("Station ID,Station Name,DateTime,Water Level (Blok) (cm),Water Level (Parit) (cm),Raw Distance (cm)");
            file.close();
        }
    }
    
    File file = SPIFFS.open(DATA_FILE, "a");
    if (!file) {
        addToSerialBuffer("Failed to open data file for logging");
        return;
    }

    String dataString = String(config.stationId) + "," +
                        config.stationName + "," +
                        getFormattedDateTime() + "," +
                        String(levelBlok, 2) + "," +
                        String(levelParit, 2) + "," +
                        String(currentRawDistance, 2) + "\n";

    if (file.print(dataString)) {
        addToSerialBuffer("Data logged successfully");
    } else {
        addToSerialBuffer("Failed to write data");
    }

    file.close();
    
    // Log storage info periodically
    static unsigned long lastStorageInfo = 0;
    if (millis() - lastStorageInfo > 300000) { // Every 5 minutes
        getStorageInfo();
        getDataFileInfo();
        lastStorageInfo = millis();
    }
}

// Function to clean old data when storage gets full
void cleanOldData() {
    addToSerialBuffer("Storage getting full, cleaning old data...");
    
    if (!SPIFFS.exists(DATA_FILE)) {
        return;
    }
    
    File originalFile = SPIFFS.open(DATA_FILE, "r");
    if (!originalFile) {
        return;
    }
    
    // Create temporary file
    File tempFile = SPIFFS.open("/temp.csv", "w");
    if (!tempFile) {
        originalFile.close();
        return;
    }
    
    // Copy header
    String header = originalFile.readStringUntil('\n');
    tempFile.println(header);
    
    // Count total lines first
    int totalLines = 0;
    while (originalFile.available()) {
        originalFile.readStringUntil('\n');
        totalLines++;
    }
    
    // Reset to beginning and skip header
    originalFile.seek(0);
    originalFile.readStringUntil('\n');
    
    // Keep only the latest 50% of records
    int linesToSkip = totalLines / 2;
    int currentLine = 0;
    
    while (originalFile.available()) {
        String line = originalFile.readStringUntil('\n');
        if (currentLine >= linesToSkip) {
            tempFile.println(line);
        }
        currentLine++;
    }
    
    originalFile.close();
    tempFile.close();
    
    // Replace original file with cleaned file
    SPIFFS.remove(DATA_FILE);
    SPIFFS.rename("/temp.csv", DATA_FILE);
    
    addToSerialBuffer("Old data cleaned, kept latest " + String(totalLines - linesToSkip) + " records");
}

// Add storage info endpoint
void handleStorageInfo() {
    size_t totalBytes = SPIFFS.totalBytes();
    size_t usedBytes = SPIFFS.usedBytes();
    size_t freeBytes = totalBytes - usedBytes;
    
    int recordCount = 0;
    size_t dataFileSize = 0;
    
    if (SPIFFS.exists(DATA_FILE)) {
        File file = SPIFFS.open(DATA_FILE, "r");
        if (file) {
            dataFileSize = file.size();
            while (file.available()) {
                String line = file.readStringUntil('\n');
                if (line.length() > 0 && !line.startsWith("Station")) {
                    recordCount++;
                }
            }
            file.close();
        }
    }
    
    JsonDocument doc;
    doc["totalBytes"] = totalBytes;
    doc["usedBytes"] = usedBytes;
    doc["freeBytes"] = freeBytes;
    doc["dataFileSize"] = dataFileSize;
    doc["recordCount"] = recordCount;
    doc["percentUsed"] = (usedBytes * 100) / totalBytes;
    
    String jsonString;
    serializeJson(doc, jsonString);
    server.send(200, "application/json", jsonString);
}

void setup()
{
    // Initialize serial first and wait for it to be ready
    Serial.begin(115200);
    delay(2000); // Give more time for serial to stabilize
    
    Serial.println("\n=== Water Level Logger Starting ===");
    Serial.println("Phase 1: Basic initialization");
    Serial.flush();

    // Initialize watchdog early but with longer timeout
    esp_task_wdt_init(300, true); // 5 minutes timeout during setup
    esp_task_wdt_add(NULL);
    
    // Basic pin setup
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIGGER_PIN, LOW);
    
    Serial.println("Phase 2: SPIFFS initialization");
    Serial.flush();
    
    if (!setupSPIFFS()) {
        Serial.println("CRITICAL: SPIFFS failed - restarting in 5 seconds");
        delay(5000);
        ESP.restart();
    }
    
    Serial.println("Phase 3: Config loading");
    Serial.flush();
    
    if (!loadConfig()) {
        Serial.println("Config load failed, using defaults");
        saveConfig();
    }
    
    Serial.println("Phase 4: WiFi setup");
    Serial.flush();
    
    setupWiFi();
    
    Serial.println("Phase 5: RTC setup (safe mode)");
    Serial.flush();
    
    // This is where the crash was happening - now with extensive protection
    bool rtcSuccess = setupRTC();
    if (rtcSuccess) {
        Serial.println("RTC initialized successfully");
    } else {
        Serial.println("RTC initialization failed - continuing without RTC");
        // Don't restart, just continue
    }
    
    Serial.println("Phase 6: Web server setup");
    Serial.flush();
    
    setupWebServer();
    
    Serial.println("Phase 7: Final initialization");
    Serial.flush();
    
    // Reduce watchdog timeout to normal operation
    esp_task_wdt_deinit();
    initWatchdog(); // This will reinitialize with normal timeout
    
    startTime = millis();
    validateMeasurementInterval();
    systemInitialized = true; // Mark system as fully initialized
    
    Serial.println("=== System Initialization Complete ===");
    if (config.operationMode == OFFLINE_MODE) {
        Serial.printf("OFFLINE MODE - Web interface: http://%s\n", WiFi.softAPIP().toString().c_str());
    } else {
        Serial.printf("ONLINE MODE - Web interface: http://%s\n", WiFi.localIP().toString().c_str());
    }
    
    // Initialize sensor pins based on type
    if (config.sensorType == HCSR04_SENSOR) {
        pinMode(TRIGGER_PIN, OUTPUT);
        pinMode(ECHO_PIN, INPUT);
        Serial.println("HC-SR04 sensor configured");
    } else {
        // Configure A01NYUB but don't initialize Serial2 yet
        Serial.println("A01NYUB sensor configured");
    }
    
    Serial.println("Taking first measurement...");
    Serial.flush();
    
    measureWaterLevel();
    
    Serial.println("Setup completed successfully!");
}

void handleRestart()
{
    server.send(200, "text/plain", "Restarting...");
    delay(1000);
    ESP.restart();
}

void handleUptime()
{
    unsigned long currentMillis = millis();
    unsigned long uptimeSeconds = currentMillis / 1000;
    unsigned long uptimeMinutes = uptimeSeconds / 60;
    unsigned long uptimeHours = uptimeMinutes / 60;
    unsigned long uptimeDays = uptimeHours / 24;

    String uptimeStr = String(uptimeDays) + " days, " +
                       String(uptimeHours % 24) + " hours, " +
                       String(uptimeMinutes % 60) + " minutes, " +
                       String(uptimeSeconds % 60) + " seconds";

    server.send(200, "text/plain", uptimeStr);
}

void validateMeasurementInterval()
{
    if (config.measurementInterval < MINIMUM_INTERVAL)
    {
        config.measurementInterval = MINIMUM_INTERVAL;
        saveConfig();
    }
}

void loop()
{
    server.handleClient();

    unsigned long currentTime = millis();
    
    // Handle measurements
    if (currentTime - lastMeasurementTime >= config.measurementInterval)
    {
        measureWaterLevel();
        lastMeasurementTime = currentTime;
        
        // In online mode, check internet and sync data periodically
        if (config.operationMode == ONLINE_MODE) {
            hasInternetConnection = checkInternetConnection();
        }
    }

    // Handle data synchronization in online mode
    if (config.operationMode == ONLINE_MODE && 
        currentTime - lastDataSyncTime >= config.dataSyncInterval)
    {
        if (hasInternetConnection) {
            syncStoredData();
        }
        lastDataSyncTime = currentTime;
    }

    resetWatchdog();
}

bool setupSPIFFS()
{
    if (!SPIFFS.begin(true))
    {
        Serial.println("SPIFFS Mount Failed");
        return false;
    }

    Serial.println("SPIFFS mounted successfully");
    
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while(file){
        Serial.print("  FILE: ");
        Serial.print(file.name());
        Serial.print("  SIZE: ");
        Serial.println(file.size());
        file = root.openNextFile();
    }
    
    return true;
}

// Completely rewrite the setupRTC function with extensive error checking
bool setupRTC()
{
    Serial.println("Starting RTC setup...");
    Serial.flush();
    
    // Ensure I2C pins are not conflicting with anything
    pinMode(RTC_SDA, INPUT_PULLUP);
    pinMode(RTC_SCL, INPUT_PULLUP);
    delay(100);
    
    // Initialize I2C with explicit error checking
    Serial.println("Initializing I2C...");
    Serial.flush();
    
    // End any existing I2C session first
    Wire.end();
    delay(50);
    
    // Start I2C with specific pins
    bool i2cResult = Wire.begin(RTC_SDA, RTC_SCL);
    if (!i2cResult) {
        Serial.println("I2C initialization failed!");
        rtcAvailable = false;
        return false;
    }
    
    // Set I2C frequency to a safe value
    Wire.setClock(100000); // 100kHz - slower but more reliable
    delay(100);
    
    Serial.println("I2C initialized, attempting RTC begin...");
    Serial.flush();
    
    // Try to initialize RTC with timeout protection
    unsigned long startTime = millis();
    bool rtcBeginResult = false;
    
    // Wrap RTC begin in a timeout
    while (millis() - startTime < 2000) { // 2 second timeout
        rtcBeginResult = rtc.begin();
        if (rtcBeginResult) break;
        delay(100);
    }
    
    if (!rtcBeginResult) {
        Serial.println("RTC begin failed - continuing without RTC");
        rtcAvailable = false;
        Wire.end(); // Clean up I2C
        return false;
    }
    
    Serial.println("RTC begin successful, checking power status...");
    Serial.flush();
    
    // Check RTC power status with error handling
    try {
        if (rtc.lostPower()) {
            Serial.println("RTC lost power, setting default time");
            rtc.adjust(DateTime(2024, 1, 1, 12, 0, 0));
            delay(100);
        }
    } catch (...) {
        Serial.println("Error checking RTC power status");
        rtcAvailable = false;
        return false;
    }
    
    // Verify RTC is actually working by reading time
    try {
        DateTime now = rtc.now();
        Serial.printf("RTC time: %04d-%02d-%02d %02d:%02d:%02d\n", 
                     now.year(), now.month(), now.day(),
                     now.hour(), now.minute(), now.second());
    } catch (...) {
        Serial.println("Error reading RTC time");
        rtcAvailable = false;
        return false;
    }
    
    rtcAvailable = true;
    Serial.println("RTC setup completed successfully");
    return true;
}

void setupWiFi()
{
    if (config.operationMode == OFFLINE_MODE) {
        // Offline mode: Create hotspot
        WiFi.mode(WIFI_AP);
        WiFi.softAP("water_level", "sulungresearch");
        Serial.print("OFFLINE MODE - AP IP address: ");
        Serial.println(WiFi.softAPIP());
        addToSerialBuffer("Started in OFFLINE mode - Hotspot created");
    } else {
        // Online mode: Connect to WiFi
        isOnlineMode = true;
        if (connectToWiFi()) {
            hasInternetConnection = checkInternetConnection();
            addToSerialBuffer("Started in ONLINE mode - Connected to WiFi");
        } else {
            addToSerialBuffer("ONLINE mode failed - No WiFi connection");
        }
    }
}

bool connectToWiFi()
{
    WiFi.mode(WIFI_STA);
    
    // Try custom WiFi first
    if (config.wifiSSID.length() > 0) {
        Serial.println("Connecting to custom WiFi: " + config.wifiSSID);
        WiFi.begin(config.wifiSSID.c_str(), config.wifiPassword.c_str());
        
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nConnected to custom WiFi!");
            Serial.print("IP address: ");
            Serial.println(WiFi.localIP());
            return true;
        }
    }
    
    // Try default WiFi
    Serial.println("\nTrying default WiFi...");
    WiFi.begin("water_level", "w4t3r_l3v3l");
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to default WiFi!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        return true;
    }
    
    Serial.println("\nFailed to connect to any WiFi network");
    return false;
}

bool checkInternetConnection()
{
    if (WiFi.status() != WL_CONNECTED) {
        return false;
    }
    
    HTTPClient http;
    http.begin("http://www.google.com");
    http.setTimeout(5000);
    int httpCode = http.GET();
    http.end();
    
    return (httpCode > 0);
}

void setupWebServer()
{
    server.enableCORS(true);

    server.on("/", HTTP_GET, handleRoot);
    server.on("/getData", HTTP_GET, handleGetData);
    server.on("/deleteData", HTTP_POST, handleDeleteData);
    server.on("/settings", HTTP_POST, handleSettings);
    server.on("/calibration", HTTP_POST, handleCalibration);
    server.on("/setTime", HTTP_POST, handleSetTime);
    server.on("/serial", HTTP_GET, handleSerial);
    server.on("/clients", HTTP_GET, handleClients);
    server.on("/config", HTTP_GET, handleGetConfig);
    server.on("/currentLevel", HTTP_GET, handleCurrentLevel);
    server.on("/restart", HTTP_POST, handleRestart);
    server.on("/uptime", HTTP_GET, handleUptime);
    server.on("/storageInfo", HTTP_GET, handleStorageInfo);

    server.begin();
}

void handleRoot()
{
    if (SPIFFS.exists("/index.html"))
    {
        File file = SPIFFS.open("/index.html", "r");
        if (file)
        {
            server.streamFile(file, "text/html");
            file.close();
        }
        else
        {
            server.send(500, "text/plain", "Error reading file");
        }
    }
    else
    {
        server.send(404, "text/plain", "File not found - index.html missing from SPIFFS");
    }
}

void handleGetData()
{
    if (SPIFFS.exists(DATA_FILE))
    {
        File file = SPIFFS.open(DATA_FILE, "r");
        if (file)
        {
            server.streamFile(file, "text/csv");
            file.close();
        }
        else
        {
            server.send(500, "text/plain", "Error reading data file");
        }
    }
    else
    {
        server.send(404, "text/plain", "No data found");
    }
}

void handleDeleteData()
{
    if (SPIFFS.remove(DATA_FILE))
    {
        File file = SPIFFS.open(DATA_FILE, "w");
        if (file)
        {
            file.println("Station ID,Station Name,DateTime,Water Level (Blok) (cm),Water Level (Parit) (cm),Raw Distance (cm)");
            file.close();
            server.send(200, "text/plain", "Data deleted successfully");
        }
        else
        {
            server.send(500, "text/plain", "Error creating new data file");
        }
    }
    else
    {
        server.send(500, "text/plain", "Failed to delete data");
    }
}

void handleSettings()
{
    if (server.hasArg("sensorType"))
    {
        String sensorTypeStr = server.arg("sensorType");
        config.sensorType = sensorTypeStr.equals("A01NYUB") ? A01NYUB_SENSOR : HCSR04_SENSOR;
    }
    if (server.hasArg("stationId"))
    {
        config.stationId = server.arg("stationId").toInt();
    }
    if (server.hasArg("stationName"))
    {
        config.stationName = server.arg("stationName");
    }
    if (server.hasArg("interval"))
    {
        unsigned long seconds = server.arg("interval").toInt();
        config.measurementInterval = max(MINIMUM_INTERVAL, seconds * 1000UL);
    }
    if (server.hasArg("sensorToBottomDistance"))
    {
        config.sensorToBottomDistance = server.arg("sensorToBottomDistance").toFloat();
    }
    if (server.hasArg("sensorToZeroBlokDistance"))
    {
        config.sensorToZeroBlokDistance = server.arg("sensorToZeroBlokDistance").toFloat();
    }
    if (server.hasArg("operationMode"))
    {
        config.operationMode = server.arg("operationMode").equals("ONLINE") ? ONLINE_MODE : OFFLINE_MODE;
    }
    if (server.hasArg("wifiSSID"))
    {
        config.wifiSSID = server.arg("wifiSSID");
    }
    if (server.hasArg("wifiPassword"))
    {
        config.wifiPassword = server.arg("wifiPassword");
    }
    if (server.hasArg("apiEndpoint"))
    {
        config.apiEndpoint = server.arg("apiEndpoint");
    }
    if (server.hasArg("apiToken"))
    {
        config.apiToken = server.arg("apiToken");
    }
    if (server.hasArg("dataSyncInterval"))
    {
        unsigned long hours = server.arg("dataSyncInterval").toInt();
        config.dataSyncInterval = hours * 3600000UL; // Convert hours to milliseconds
    }

    if (saveConfig())
    {
        if (config.sensorType == HCSR04_SENSOR)
        {
            pinMode(TRIGGER_PIN, OUTPUT);
            pinMode(ECHO_PIN, INPUT);
            Serial2.end();
        }
        else
        {
            pinMode(TRIGGER_PIN, INPUT);
            pinMode(ECHO_PIN, INPUT);
        }
        
        server.send(200, "text/plain", "Settings saved successfully. Restart required for mode changes.");
    }
    else
    {
        server.send(500, "text/plain", "Failed to save settings");
    }
}

void handleCalibration()
{
    if (server.hasArg("offset"))
    {
        config.calibrationOffset = server.arg("offset").toFloat();

        if (saveConfig())
        {
            server.send(200, "text/plain", "Calibration saved successfully");
        }
        else
        {
            server.send(500, "text/plain", "Failed to save calibration");
        }
    }
    else
    {
        server.send(400, "text/plain", "Missing offset parameter");
    }
}

void handleSetTime()
{
    if (server.hasArg("year") && server.hasArg("month") &&
        server.hasArg("day") && server.hasArg("hour") &&
        server.hasArg("minute") && server.hasArg("second"))
    {

        config.dateTime.year = server.arg("year").toInt();
        config.dateTime.month = server.arg("month").toInt();
        config.dateTime.day = server.arg("day").toInt();
        config.dateTime.hour = server.arg("hour").toInt();
        config.dateTime.minute = server.arg("minute").toInt();
        config.dateTime.second = server.arg("second").toInt();

        rtc.adjust(DateTime(
            config.dateTime.year,
            config.dateTime.month,
            config.dateTime.day,
            config.dateTime.hour,
            config.dateTime.minute,
            config.dateTime.second));

        if (saveConfig())
        {
            server.send(200, "text/plain", "Time set successfully");
        }
        else
        {
            server.send(500, "text/plain", "Failed to save time settings");
        }
    }
    else
    {
        server.send(400, "text/plain", "Missing time parameters");
    }
}

void handleSerial()
{
    String output;
    for (int i = 0; i < SERIAL_BUFFER_SIZE; i++)
    {
        int index = (serialBufferIndex + i) % SERIAL_BUFFER_SIZE;
        if (serialBuff[index].length() > 0)
        {
            output += serialBuff[index] + "\n";
        }
    }
    server.send(200, "text/plain", output);
}

void handleClients()
{
    if (config.operationMode == OFFLINE_MODE) {
        wifi_sta_list_t stationList;
        tcpip_adapter_sta_list_t adapterList;

        esp_wifi_ap_get_sta_list(&stationList);
        tcpip_adapter_get_sta_list(&stationList, &adapterList);

        String clientsList = "[";
        for (int i = 0; i < adapterList.num; i++)
        {
            if (i > 0)
                clientsList += ",";
            tcpip_adapter_sta_info_t station = adapterList.sta[i];
            String mac = "";
            for (int j = 0; j < 6; j++)
            {
                if (j > 0)
                    mac += ":";
                if (station.mac[j] < 0x10)
                    mac += "0";
                mac += String(station.mac[j], HEX);
            }
            String ip = String(station.ip.addr & 0xFF) + "." +
                        String((station.ip.addr >> 8) & 0xFF) + "." +
                        String((station.ip.addr >> 16) & 0xFF) + "." +
                        String((station.ip.addr >> 24) & 0xFF);
            clientsList += "\"" + ip + " (MAC: " + mac + ")\"";
        }
        clientsList += "]";
        server.send(200, "application/json", clientsList);
    } else {
        // Online mode - show WiFi connection status
        String status = "[\"WiFi: " + WiFi.localIP().toString() + 
                       " (Internet: " + (hasInternetConnection ? "Yes" : "No") + ")\"]";
        server.send(200, "application/json", status);
    }
}

void handleCurrentLevel()
{
    JsonDocument doc;
    doc["waterLevelBlok"] = currentWaterLevelBlok;
    doc["waterLevelParit"] = currentWaterLevelParit;
    doc["rawDistance"] = currentRawDistance;
    doc["operationMode"] = (config.operationMode == ONLINE_MODE) ? "ONLINE" : "OFFLINE";
    doc["internetConnection"] = hasInternetConnection;
    
    String jsonString;
    serializeJson(doc, jsonString);
    server.send(200, "application/json", jsonString);
}

void handleGetConfig()
{
    JsonDocument doc;

    doc["stationId"] = config.stationId;
    doc["stationName"] = config.stationName;
    doc["measurementInterval"] = config.measurementInterval;
    doc["calibrationOffset"] = config.calibrationOffset;
    doc["sensorType"] = (int)config.sensorType;
    doc["sensorToBottomDistance"] = config.sensorToBottomDistance;
    doc["sensorToZeroBlokDistance"] = config.sensorToZeroBlokDistance;
    doc["operationMode"] = (config.operationMode == ONLINE_MODE) ? "ONLINE" : "OFFLINE";
    doc["wifiSSID"] = config.wifiSSID;
    doc["wifiPassword"] = config.wifiPassword;
    doc["apiEndpoint"] = config.apiEndpoint;
    doc["apiToken"] = config.apiToken;
    doc["dataSyncInterval"] = config.dataSyncInterval / 3600000; // Convert to hours

    DateTime now = rtc.now();
    JsonObject dateTime = doc["dateTime"].to<JsonObject>();
    dateTime["year"] = now.year();
    dateTime["month"] = now.month();
    dateTime["day"] = now.day();
    dateTime["hour"] = now.hour();
    dateTime["minute"] = now.minute();
    dateTime["second"] = now.second();

    String jsonString;
    serializeJson(doc, jsonString);
    server.send(200, "application/json", jsonString);
}

bool loadConfig()
{
    if (!SPIFFS.exists(CONFIG_FILE))
    {
        return false;
    }

    File file = SPIFFS.open(CONFIG_FILE, "r");
    if (!file)
    {
        return false;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error)
    {
        return false;
    }

    config.stationId = doc["stationId"] | 1;
    config.stationName = doc["stationName"].as<String>();
    config.measurementInterval = doc["measurementInterval"] | 12000;
    config.calibrationOffset = doc["calibrationOffset"] | 0.0;
    config.sensorToBottomDistance = doc["sensorToBottomDistance"] | 100.0;
    config.sensorToZeroBlokDistance = doc["sensorToZeroBlokDistance"] | 50.0;
    config.sensorType = (SensorType)(doc["sensorType"] | HCSR04_SENSOR);
    config.operationMode = doc["operationMode"].as<String>().equals("ONLINE") ? ONLINE_MODE : OFFLINE_MODE;
    config.wifiSSID = doc["wifiSSID"].as<String>();
    config.wifiPassword = doc["wifiPassword"].as<String>();
    config.apiEndpoint = doc["apiEndpoint"].as<String>();
    config.apiToken = doc["apiToken"].as<String>();
    config.dataSyncInterval = doc["dataSyncInterval"] | 3600000;

    JsonObject dateTime = doc["dateTime"];
    if (dateTime)
    {
        config.dateTime.year = dateTime["year"] | 2024;
        config.dateTime.month = dateTime["month"] | 1;
        config.dateTime.day = dateTime["day"] | 1;
        config.dateTime.hour = dateTime["hour"] | 0;
        config.dateTime.minute = dateTime["minute"] | 0;
        config.dateTime.second = dateTime["second"] | 0;
    }

    return true;
}

bool saveConfig()
{
    File file = SPIFFS.open(CONFIG_FILE, "w");
    if (!file)
    {
        return false;
    }

    JsonDocument doc;

    doc["stationId"] = config.stationId;
    doc["stationName"] = config.stationName;
    doc["measurementInterval"] = config.measurementInterval;
    doc["calibrationOffset"] = config.calibrationOffset;
    doc["sensorType"] = (int)config.sensorType;
    doc["sensorToBottomDistance"] = config.sensorToBottomDistance;
    doc["sensorToZeroBlokDistance"] = config.sensorToZeroBlokDistance;
    doc["operationMode"] = (config.operationMode == ONLINE_MODE) ? "ONLINE" : "OFFLINE";
    doc["wifiSSID"] = config.wifiSSID;
    doc["wifiPassword"] = config.wifiPassword;
    doc["apiEndpoint"] = config.apiEndpoint;
    doc["apiToken"] = config.apiToken;
    doc["dataSyncInterval"] = config.dataSyncInterval;

    JsonObject dateTime = doc["dateTime"].to<JsonObject>();
    dateTime["year"] = config.dateTime.year;
    dateTime["month"] = config.dateTime.month;
    dateTime["day"] = config.dateTime.day;
    dateTime["hour"] = config.dateTime.hour;
    dateTime["minute"] = config.dateTime.minute;
    dateTime["second"] = config.dateTime.second;
    
    if (serializeJson(doc, file) == 0)
    {
        file.close();
        return false;
    }

    file.close();
    return true;
}

float readA01NYUB()
{
    const int numMeasurements = 30;
    float measurements[numMeasurements];
    int validMeasurements = 0;

    Serial2.begin(9600, SERIAL_8N1, A01_RX, A01_TX);

    for (int i = 0; i < numMeasurements; i++)
    {
        if (Serial2.write(0x01) == 1)
        {
            delay(100);

            if (Serial2.available() >= 4)
            {
                byte response[4];
                Serial2.readBytes(response, 4);

                if (response[0] == 0xFF)
                {
                    int distance = (response[1] << 8) | response[2];
                    if (distance > 0 && distance < 7500)
                    {
                        measurements[validMeasurements++] = distance / 10.0;
                    }
                }
            }
        }
        delay(50);
    }

    Serial2.end();

    if (validMeasurements == 0)
    {
        addToSerialBuffer("Warning: No valid A01NYUB measurements");
        return -1;
    }

    float sum = 0;
    for (int i = 0; i < validMeasurements; i++)
    {
        sum += measurements[i];
    }
    return sum / validMeasurements;
}

float readHCSR04()
{
    const int numMeasurements = 30;
    float measurements[numMeasurements];
    int validMeasurements = 0;

    for (int i = 0; i < numMeasurements; i++)
    {
        digitalWrite(TRIGGER_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIGGER_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_PIN, LOW);

        long duration = pulseIn(ECHO_PIN, HIGH, 30000);
        if (duration > 0)
        {
            float distance = duration * 0.034 / 2;
            measurements[validMeasurements++] = distance;
        }
        delay(50);
    }

    if (validMeasurements == 0)
    {
        addToSerialBuffer("Warning: No valid HCSR04 measurements");
        return -1;
    }

    float sum = 0;
    for (int i = 0; i < validMeasurements; i++)
    {
        sum += measurements[i];
    }
    return sum / validMeasurements;
}

bool sendDataToAPI(float levelBlok, float levelParit, float rawDistance)
{
    if (!hasInternetConnection || config.apiEndpoint.length() == 0) {
        return false;
    }

    HTTPClient http;
    http.begin(config.apiEndpoint);
    http.addHeader("Content-Type", "application/json");
    
    if (config.apiToken.length() > 0) {
        http.addHeader("Authorization", "Bearer " + config.apiToken);
    }

    JsonDocument doc;
    doc["station_name"] = config.stationName;
    doc["idwl"] = config.stationId;
    doc["level_blok"] = levelBlok;
    doc["level_parit"] = levelParit;
    doc["sensor_distance"] = rawDistance;
    doc["datetime"] = getFormattedDateTime();

    String jsonString;
    serializeJson(doc, jsonString);

    int httpResponseCode = http.POST(jsonString);
    http.end();

    if (httpResponseCode > 0 && httpResponseCode < 400) {
        addToSerialBuffer("Data sent to API successfully. Response: " + String(httpResponseCode));
        return true;
    } else {
        addToSerialBuffer("Failed to send data to API. Response: " + String(httpResponseCode));
        return false;
    }
}

void syncStoredData()
{
    if (!hasInternetConnection || config.apiEndpoint.length() == 0) {
        return;
    }

    if (!SPIFFS.exists(DATA_FILE)) {
        return;
    }

    File file = SPIFFS.open(DATA_FILE, "r");
    if (!file) {
        return;
    }

    String csvContent = file.readString();
    file.close();

    if (csvContent.length() <= 100) { // Only headers
        return;
    }

    // Convert CSV to JSON array
    JsonDocument doc;
    JsonArray dataArray = doc["data"].to<JsonArray>();

    String lines[1000]; // Support up to 1000 lines
    int lineCount = 0;
    int startPos = 0;
    int endPos = csvContent.indexOf('\n');

    // Skip header line
    if (endPos > 0) {
        startPos = endPos + 1;
        endPos = csvContent.indexOf('\n', startPos);
    }

    // Parse CSV lines
    while (endPos > 0 && lineCount < 1000) {
        String line = csvContent.substring(startPos, endPos);
        line.trim();
        
        if (line.length() > 0) {
            // Parse CSV line: Station ID,Station Name,DateTime,Water Level (Blok) (cm),Water Level (Parit) (cm),Raw Distance (cm)
            int comma1 = line.indexOf(',');
            int comma2 = line.indexOf(',', comma1 + 1);
            int comma3 = line.indexOf(',', comma2 + 1);
            int comma4 = line.indexOf(',', comma3 + 1);
            int comma5 = line.indexOf(',', comma4 + 1);

            if (comma1 > 0 && comma2 > 0 && comma3 > 0 && comma4 > 0 && comma5 > 0) {
                JsonObject entry = dataArray.add<JsonObject>();
                entry["station_name"] = line.substring(comma1 + 1, comma2);
                entry["idwl"] = line.substring(0, comma1).toInt();
                entry["datetime"] = line.substring(comma2 + 1, comma3);
                entry["level_blok"] = line.substring(comma3 + 1, comma4).toFloat();
                entry["level_parit"] = line.substring(comma4 + 1, comma5).toFloat();
                entry["sensor_distance"] = line.substring(comma5 + 1).toFloat();
            }
        }

        startPos = endPos + 1;
        endPos = csvContent.indexOf('\n', startPos);
        lineCount++;
    }

    if (dataArray.size() == 0) {
        return;
    }

    // Send data to API
    HTTPClient http;
    http.begin(config.apiEndpoint + "/bulk"); // Assume bulk endpoint
    http.addHeader("Content-Type", "application/json");
    
    if (config.apiToken.length() > 0) {
        http.addHeader("Authorization", "Bearer " + config.apiToken);
    }

    String jsonString;
    serializeJson(doc, jsonString);

    int httpResponseCode = http.POST(jsonString);
    http.end();

    if (httpResponseCode > 0 && httpResponseCode < 400) {
        addToSerialBuffer("Bulk data sync successful. Cleared local storage.");
        // Clear the data file after successful sync
        SPIFFS.remove(DATA_FILE);
        File newFile = SPIFFS.open(DATA_FILE, "w");
        if (newFile) {
            newFile.println("Station ID,Station Name,DateTime,Water Level (Blok) (cm),Water Level (Parit) (cm),Raw Distance (cm)");
            newFile.close();
        }
    } else {
        addToSerialBuffer("Bulk data sync failed. Response: " + String(httpResponseCode));
    }
}

void measureWaterLevel()
{
    if (!systemInitialized) {
        Serial.println("System not fully initialized, skipping measurement");
        return;
    }
    
    float distance;

    if (config.sensorType == HCSR04_SENSOR) {
        distance = readHCSR04();
    } else {
        distance = readA01NYUB();
    }

    if (distance >= 0) {
        currentRawDistance = distance;
        
        currentWaterLevelBlok = ((distance - config.sensorToZeroBlokDistance) * -1) + config.calibrationOffset;
        currentWaterLevelParit = (config.sensorToBottomDistance - distance) + config.calibrationOffset;
        
        // In online mode, try to send data directly to API
        bool sentToAPI = false;
        if (config.operationMode == ONLINE_MODE && hasInternetConnection) {
            sentToAPI = sendDataToAPI(currentWaterLevelBlok, currentWaterLevelParit, currentRawDistance);
        }
        
        // If not sent to API or in offline mode, save to local storage
        if (!sentToAPI) {
            logDataWithManagement(currentWaterLevelBlok, currentWaterLevelParit);
        }
        
        String statusMsg = "Raw: " + String(currentRawDistance, 2) + "cm, " +
                          "Blok: " + String(currentWaterLevelBlok, 2) + "cm, " +
                          "Parit: " + String(currentWaterLevelParit, 2) + "cm" +
                          (sentToAPI ? " [API]" : " [LOCAL]");
        
        addToSerialBuffer(statusMsg);
    } else {
        addToSerialBuffer("Measurement failed - sensor error");
    }
}

String getFormattedDateTime()
{
    if (!rtcAvailable || !systemInitialized) {
        // Use system uptime as fallback
        unsigned long uptimeMs = millis();
        unsigned long seconds = uptimeMs / 1000;
        unsigned long minutes = seconds / 60;
        unsigned long hours = minutes / 60;
        
        char timeStr[32];
        sprintf(timeStr, "UPTIME_%02lu:%02lu:%02lu", 
                hours % 24, minutes % 60, seconds % 60);
        return String(timeStr);
    }
    
    try {
        DateTime now = rtc.now();
        if (now.year() < 2020 || now.year() > 2030) {
            // Invalid date, use uptime
            return "INVALID_DATE";
        }
        
        char datetime[20];
        sprintf(datetime, "%04d-%02d-%02d %02d:%02d:%02d",
                now.year(), now.month(), now.day(),
                now.hour(), now.minute(), now.second());
        return String(datetime);
    } catch (...) {
        rtcAvailable = false;
        return "RTC_ERROR";
    }
}

void addToSerialBuffer(const String &message)
{
    String timestampedMessage;
    
    if (systemInitialized) {
        timestampedMessage = getFormattedDateTime() + " - " + message;
    } else {
        timestampedMessage = "BOOT - " + message;
    }
    
    Serial.println(timestampedMessage);
    Serial.flush(); // Ensure message is sent immediately
    
    // Only add to buffer if system is properly initialized
    if (systemInitialized) {
        serialBuff[serialBufferIndex] = timestampedMessage;
        serialBufferIndex = (serialBufferIndex + 1) % SERIAL_BUFFER_SIZE;
    }
}


void initWatchdog()
{
    esp_task_wdt_init(WDT_TIMEOUT, true);
    esp_task_wdt_add(NULL);
    Serial.println("Watchdog initialized");
}

void resetWatchdog()
{
    esp_task_wdt_reset();
}