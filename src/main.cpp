#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "FS.h"
#include "SD.h"
#include <SPI.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <esp_wifi.h>

// Pin Definitions for ESP32-DOIT-DevKit-V1
#define TRIGGER_PIN 2 // GPIO26
#define ECHO_PIN 4    // GPIO27
// SD Card SPI Pins
#define SD_CS 5    // GPIO5 (CS)
#define SD_MOSI 23 // GPIO23 (MOSI)
#define SD_MISO 19 // GPIO19 (MISO)
#define SD_SCK 18  // GPIO18 (SCK)
// RTC I2C Pins
#define RTC_SDA 21 // GPIO21 (SDA)
#define RTC_SCL 22 // GPIO22 (SCL)
// Add new pin definitions for A01NYUB
#define A01_RX 16 // GPIO16
#define A01_TX 17 // GPIO17

// Constants
#define WDT_TIMEOUT 180 // 3 minutes watchdog timeout
#define CONFIG_FILE "/config.json"
#define MAX_CLIENTS 10

#define SERIAL_BUFFER_SIZE 20
String serialBuff[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;

// Global Variables
WebServer server(80);
RTC_DS3231 rtc;
String serialBuffer = "";
unsigned long lastMeasurementTime = 0;
float currentWaterLevelBlok = 0.0;
float currentWaterLevelParit = 0.0;
float currentRawDistance = 0.0;
String connectedClients[MAX_CLIENTS];
int numClients = 0;

// Add sensor type enum
enum SensorType
{
    HCSR04_SENSOR,
    A01NYUB_SENSOR
};

// Configuration structure
struct Config {
    int stationId;
    String stationName;
    unsigned long measurementInterval;
    float calibrationOffset;
    SensorType sensorType;
    float sensorToBottomDistance;  // Distance from sensor to bottom of ditch
    float sensorToZeroBlokDistance; // Distance from sensor to 0 level of block
    
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
               measurementInterval(10000),
               calibrationOffset(0.0),
               sensorType(HCSR04_SENSOR),
               sensorToBottomDistance(100.0),
               sensorToZeroBlokDistance(50.0) {}
} config;

unsigned long startTime = 0;
const unsigned long MINIMUM_INTERVAL = 12000; // 12 seconds in milliseconds

// Function declarations
bool setupSD();
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
void logData(float levelBlok, float levelParit);
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

void addToSerialBuffer(const String &message)
{
    String timestampedMessage = getFormattedDateTime() + " - " + message;
    Serial.println(timestampedMessage); // Print to actual serial for debugging
    serialBuff[serialBufferIndex] = timestampedMessage;
    serialBufferIndex = (serialBufferIndex + 1) % SERIAL_BUFFER_SIZE;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("\nInitializing Water Level Logger...");

    // Initialize pins
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Initialize SPI for SD card
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

    // Initialize I2C for RTC
    Wire.begin(RTC_SDA, RTC_SCL);

    if (!setupSD())
    {
        Serial.println("SD Card initialization failed! Please check wiring.");
        delay(3000);
        ESP.restart();
    }

    if (!loadConfig())
    {
        Serial.println("Failed to load config, using defaults");
        saveConfig();
    }

    setupWiFi();

    if (!setupRTC())
    {
        Serial.println("RTC initialization failed! Please check wiring.");
        delay(3000);
        ESP.restart();
    }

    setupWebServer();
    initWatchdog();

    startTime = millis();
    validateMeasurementInterval();

    Serial.println("System initialized successfully");
    Serial.printf("Access web interface at: http://%s\n", WiFi.softAPIP().toString().c_str());
    measureWaterLevel();

    // Initialize sensor pins based on type
    if (config.sensorType == HCSR04_SENSOR)
    {
        pinMode(TRIGGER_PIN, OUTPUT);
        pinMode(ECHO_PIN, INPUT);
    }
    else
    {
        Serial2.begin(9600, SERIAL_8N1, A01_RX, A01_TX);
        Serial2.end(); // Will be reopened when needed
    }
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
    if (currentTime - lastMeasurementTime >= config.measurementInterval)
    {
        measureWaterLevel();
        lastMeasurementTime = currentTime;
    }

    resetWatchdog();
}

bool setupSD()
{
    if (!SD.begin(SD_CS))
    {
        Serial.println("Card Mount Failed");
        return false;
    }

    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE)
    {
        Serial.println("No SD card attached");
        return false;
    }

    Serial.println("SD Card mounted successfully");
    return true;
}

bool setupRTC()
{
    if (!rtc.begin())
    {
        Serial.println("Couldn't find RTC");
        return false;
    }

    if (rtc.lostPower())
    {
        Serial.println("RTC lost power, lets set the time!");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    return true;
}

void setupWiFi()
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP("water_level", "sulungresearch");
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());
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

    server.begin();
}

void handleRoot()
{
    if (SD.exists("/index.html"))
    {
        File file = SD.open("/index.html", "r");
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
        server.send(404, "text/plain", "File not found");
    }
}

void handleGetData()
{
    if (SD.exists("/data.csv"))
    {
        File file = SD.open("/data.csv", "r");
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
    if (SD.remove("/data.csv"))
    {
        // Create new file with headers
        File file = SD.open("/data.csv", "w");
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
        // Convert seconds to milliseconds and ensure minimum interval
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

    if (saveConfig())
    {
        // Initialize appropriate pins based on new sensor type
        if (config.sensorType == HCSR04_SENSOR)
        {
            pinMode(TRIGGER_PIN, OUTPUT);
            pinMode(ECHO_PIN, INPUT);
            Serial2.end(); // Make sure A01NYUB serial is closed
        }
        else
        {
            // For A01NYUB, we'll initialize Serial2 when needed
            pinMode(TRIGGER_PIN, INPUT); // Set unused pins to input
            pinMode(ECHO_PIN, INPUT);
        }
        server.send(200, "text/plain", "Settings saved successfully");
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
}

void handleCurrentLevel()
{
    JsonDocument doc;
    doc["waterLevelBlok"] = currentWaterLevelBlok;
    doc["waterLevelParit"] = currentWaterLevelParit;
    doc["rawDistance"] = currentRawDistance;
    
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
    if (!SD.exists(CONFIG_FILE))
    {
        return false;
    }

    File file = SD.open(CONFIG_FILE, "r");
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
    config.measurementInterval = doc["measurementInterval"] | 60000;
    config.calibrationOffset = doc["calibrationOffset"] | 0.0;
    config.sensorToBottomDistance = doc["sensorToBottomDistance"] | 100.0;
    config.sensorToZeroBlokDistance = doc["sensorToZeroBlokDistance"] | 50.0;

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
    
    config.sensorType = (SensorType)(doc["sensorType"] | HCSR04_SENSOR);

    return true;
}

bool saveConfig()
{
    File file = SD.open(CONFIG_FILE, "w");
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

// Add A01NYUB reading function
float readA01NYUB()
{
    const int numMeasurements = 30;
    float measurements[numMeasurements];
    int validMeasurements = 0;

    Serial2.begin(9600, SERIAL_8N1, A01_RX, A01_TX);

    for (int i = 0; i < numMeasurements; i++)
    {
        if (Serial2.write(0x01) == 1)
        {               // Trigger measurement
            delay(100); // Wait for response

            if (Serial2.available() >= 4)
            {
                byte response[4];
                Serial2.readBytes(response, 4);

                if (response[0] == 0xFF)
                { // Valid header
                    int distance = (response[1] << 8) | response[2];
                    if (distance > 0 && distance < 7500)
                    {                                                        // Valid range
                        measurements[validMeasurements++] = distance / 10.0; // Convert to cm
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

// Extract HCSR04 code into separate function
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

// Modified measureWaterLevel function with dual calculations
void measureWaterLevel()
{
    float distance;

    if (config.sensorType == HCSR04_SENSOR)
    {
        distance = readHCSR04();
    }
    else
    {
        distance = readA01NYUB();
    }

    if (distance >= 0)
    {
        currentRawDistance = distance;
        
        // Calculate water level for block (blok) using formula:
        // waterlevel(blok) = ((distance between sensor to water - distance between sensor to 0 blok) * -1) + calibration
        currentWaterLevelBlok = ((distance - config.sensorToZeroBlokDistance) * -1) + config.calibrationOffset;
        
        // Calculate water level for ditch (parit) using formula:
        // waterlevel(parit) = (distance between sensor to bottom of ditch - distance between sensor to water) + calibration
        currentWaterLevelParit = (config.sensorToBottomDistance - distance) + config.calibrationOffset;
        
        // Log both measurements
        logData(currentWaterLevelBlok, currentWaterLevelParit);
        
        addToSerialBuffer("Raw Distance: " + String(currentRawDistance) + 
                         "cm, Water Level (Blok): " + String(currentWaterLevelBlok) + 
                         "cm, Water Level (Parit): " + String(currentWaterLevelParit) + "cm");
    }
}

void logData(float levelBlok, float levelParit)
{
    // Create the data file with headers if it doesn't exist
    if (!SD.exists("/data.csv"))
    {
        File file = SD.open("/data.csv", "w");
        if (file)
        {
            file.println("Station ID,Station Name,DateTime,Water Level (Blok) (cm),Water Level (Parit) (cm),Raw Distance (cm)");
            file.close();
        }
    }
    
    File file = SD.open("/data.csv", "a");
    if (!file)
    {
        Serial.println("Failed to open data file for logging");
        return;
    }

    String dataString = String(config.stationId) + "," +
                        config.stationName + "," +
                        getFormattedDateTime() + "," +
                        String(levelBlok, 2) + "," +
                        String(levelParit, 2) + "," +
                        String(currentRawDistance, 2) + "\n";

    if (file.print(dataString))
    {
        Serial.println("Data logged successfully");
    }
    else
    {
        Serial.println("Failed to write data");
    }

    file.close();
}

String getFormattedDateTime()
{
    DateTime now = rtc.now();
    char datetime[20];
    sprintf(datetime, "%04d-%02d-%02d %02d:%02d:%02d",
            now.year(), now.month(), now.day(),
            now.hour(), now.minute(), now.second());
    return String(datetime);
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