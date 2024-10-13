#include <Arduino.h>

// Define modem type before including TinyGsmClient
#define TINY_GSM_MODEM_SIM7000

//#define DEBUG_ERRORS
#include <sys/time.h>
#include <SPIFFS.h>
#include <SSLClient.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>
#include <map>
#include <unordered_set>
#include <ArduinoJson.h>

// Define the CS and INT pins
#define CAN_CS 15  // Choose a free GPIO for CS
#define CAN_INT 35
#define CAN_MOSI 14  // Choose a free GPIO for MOSI
#define CAN_MISO 19  // Choose a free GPIO for MISO
#define CAN_SCK 18   // Choose a free GPIO for SCK

// Example configuration settings
#define SerialMon Serial
#define MODEM_RST 5
#define MODEM_PWRKEY 4
#define MODEM_POWER_ON 23
#define MODEM_TX 27
#define MODEM_RX 26
#define I2C_SDA 21
#define I2C_SCL 22
#define SerialAT Serial1

// analog relay pins
#define DOOR1_WINUP_RELAY 12
#define DOOR1_WINDWN_RELAY 25

#define DOOR2_WINUP_RELAY 32
#define DOOR2_WINDWN_RELAY 33

#define SUNROOF_OPEN_RELAY 34
#define SUNROOF_CLOSE_RELAY 35

#define SUNROOF_TILT_RELAY 36
#define SUNROOF_UNTILT_RELAY 39

// Misc settings
#define WINDOW_UPDOWN_DELAY 5000
#define WINDOW_VENT_DELAY 500

#define CERT_BUF_SIZE 2048  // Adjust this size based on your certificate length
#define PUBLISH_INTERVAL 5000;

#define MQTT_MAX_PACKET_SIZE 512  // Increase packet size if needed
#define MQTT_DEBUG

char ca_cert[CERT_BUF_SIZE];
char client_cert[CERT_BUF_SIZE];
char client_key[CERT_BUF_SIZE];

const char apn[] = "h2g2";
const char gprsUser[] = "";
const char gprsPass[] = "";

// NTP Server
const char* ntpServer = "pool.ntp.org";
const int ntpPort = 123;

// AWS IoT Core settings
const char* awsEndpoint = "a11mldvlf3x3z0-ats.iot.us-west-1.amazonaws.com";
const int port = 8883;

// Define the MQTT topics for the shadow
const char* shadowUpdateTopic = "$aws/things/ESP32_DevModule/shadow/update";

bool isCarlinkSetupSuccessful = false;
bool isConnectedToAWSIoT = false;
int connectCount = 0;
unsigned long lastDisplayErrorsTime = 0;
unsigned long lastPublishTime = 0;
unsigned long publishInterval = 5000;
bool isWindowDown = false;

MCP_CAN CAN(CAN_CS);  // Create CAN object with CS pin

// Serial and modem initialization
// SoftwareSerial SerialAT(12, 13); // RX, TX for the SIM7000G module
TinyGsm modem(SerialAT);
TinyGsmClient baseGsmClient(modem);
SSLClient secureGsmClient(&baseGsmClient);
PubSubClient mqttClient(secureGsmClient);

// Queue for background task queuing and processesing
//QueueHandle_t mqttQueue;
SemaphoreHandle_t canMessagesMutex;
//SemaphoreHandle_t awsConnectredFlagMutex;

// Structure to hold raw CAN data for MQTT publishing
struct CanMessage {
  unsigned long id;
  std::vector<uint8_t> data;
  unsigned long timestamp;
};

// Data structure to store CAN messages
std::map<unsigned long, CanMessage>
  canMessages;

// Define a set of CAN IDs you are interested in
std::unordered_set<unsigned long> canMessageIdsToPublish = { 0x126 };

enum CommandID {
  CMD_READ_DATA = 1,
  CMD_UNLOCK_DOORS,
  CMD_LOCK_DOORS,
  CMD_ROLL_WINDOWS_DOWN,
  CMD_ROLL_WINDOWS_UP,
  CMD_TOGGLE_WINDOW,
  CMD_VENT_WINDOWS,
  CMD_TURN_ON_HVAC,
  CMD_TURN_OFF_HVAC,
  CMD_HONK,
  CMD_OPEN_SUNROOF,
  CMD_CLOSE_SUNROOF,
  CMD_TILT_SUNROOF,
  CMD_UNTILT_SUNROOF,
  CMD_FRONT_DEFROST,
  CMD_REAR_DEFROST,
  CMD_INVALID  // Special value for invalid commands
};

struct Command {
  CommandID id;
  String params;
};

//= Setup functions =============================
void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(100);
  Serial.flush();
  delay(1000);
  Serial.println();
  Serial.println(F("Begin setup..."));

  // relay pins
  pinMode(DOOR1_WINUP_RELAY, OUTPUT);
  pinMode(DOOR1_WINDWN_RELAY, OUTPUT);

  pinMode(DOOR2_WINUP_RELAY, OUTPUT);
  pinMode(DOOR2_WINDWN_RELAY, OUTPUT);

  // pinMode(SUNROOF_OPEN_RELAY, OUTPUT);
  // pinMode(SUNROOF_CLOSE_RELAY, OUTPUT);

  // pinMode(SUNROOF_TILT_RELAY, OUTPUT);
  // pinMode(SUNROOF_UNTILT_RELAY, OUTPUT);

  isCarlinkSetupSuccessful = loadCertificates() && setupModem() && setupAWSIoT() && connectToAWSIoT() && InitializeCANTransceiver();

  canMessagesMutex = xSemaphoreCreateMutex();
  lastPublishTime = millis();

  // Create a task for publishing to MQTT
  xTaskCreatePinnedToCore(
    CANTask,    // Task function
    "CANTask",  // Name of the task
    4096,       // Stack size
    NULL,       // Task input parameter
    1,          // Priority of the task
    NULL,       // Task handle
    0           // Pin task to core 0
  );

  if (isCarlinkSetupSuccessful) {
    Serial.println(F("Carlink online! :) Ready to send and receive data."));
  } else {
    Serial.println(F("Carlink offline! :("));
  }
}

bool loadCertificates() {
  Serial.print(F("Loading certificates..."));
  if (!SPIFFS.begin(true)) {  // 'true' will format the file system if it's corrupted
    Serial.println(F("Failed: Unable to mount SPIFFS."));
    return false;
    delay(100);
  }

  // List files in the root directory
  //listDir(SPIFFS, "/", 0);

  // Read certificates from SPIFFS
  readFileToBuffer("/ca.crt", ca_cert, CERT_BUF_SIZE);
  readFileToBuffer("/client.crt", client_cert, CERT_BUF_SIZE);
  readFileToBuffer("/client.key", client_key, CERT_BUF_SIZE);

  if (!ca_cert || !client_cert || !client_key) {
    Serial.println(F(" Failed: Certificated missing."));
    return false;
  } else {
    Serial.println(F(" Success!"));
    //    Serial.println("AmazonRootCA1:");
    //    Serial.println(ca_cert);
    //    Serial.println();
    //    Serial.println("Client Cert:");
    //    Serial.println(client_cert);
    //    Serial.println();
    //    Serial.println("Client Key:");
    //    Serial.println(client_key);
  }

  return true;
}

bool setupModem() {
  // Initialize Serial1 with appropriate baud rate
  // Start the modem serial communication
  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  pinMode(MODEM_PWRKEY, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWRKEY, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  Serial.print(F("Initializing modem..."));
  modem.restart();
  Serial.println(F(" Success!"));

  syncSystemToNetworkTime();

  Serial.print(F("Connecting to cellular network..."));
  while (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    connectCount++;
    Serial.print(F(" Failed! Retrying..."));
    modem.restart();
  }
  Serial.println(F(" Success!"));
  return true;
}

bool setupAWSIoT() {

  // Load AWS IoT certificates
  secureGsmClient.setCACert(ca_cert);
  secureGsmClient.setCertificate(client_cert);
  secureGsmClient.setPrivateKey(client_key);

  mqttClient.setKeepAlive(60);
  mqttClient.setBufferSize(1024);
  // mqttClient.setTimeout(10000);
  mqttClient.setServer(awsEndpoint, port);
  mqttClient.setCallback(mqttCallback);
  // mqttClient.setDebug(true);

  return true;
}

bool connectToAWSIoT() {
  Serial.println(F("Connecting to AWS IoT Core..."));
  if (!modem.isNetworkConnected()) {
    Serial.println(F("Modem is disconnected, reconnecting..."));
    // Attempt to reconnect
    modem.restart();
    // Re-initiate connection
  }
  while (!mqttClient.connected()) {
    isConnectedToAWSIoT = false;
    if (mqttClient.connect("ESP32_DevModule")) {
      Serial.print(F(" Success! Connected to endpoint: "));
      Serial.println(awsEndpoint);
      mqttClient.subscribe("car/in");
    } else {
      Serial.print(F("Failed! Error state="));
      Serial.print(mqttClient.state());
      Serial.println(". Retrying...");
      delay(5000);
    }
  }
  isConnectedToAWSIoT = true;
  return isConnectedToAWSIoT;
}

bool InitializeCANTransceiver() {

  // Initialize MCP2515 CAN controller at 500kbps speed
  Serial.println(F("Initializing MCP2515 CAN tranceiver..."));
  SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);
  if (!CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println(F(" Failed!"));
    return false;
  }

  // Enable CAN interrupt
  pinMode(CAN_INT, INPUT);
  pinMode(CAN_CS, OUTPUT);
  CAN.setMode(MCP_NORMAL);  // Set to normal mode to start receiving

  // Set mask & filter to only process motor temp (0x126)
  CAN.init_Mask(0, 0, 0x7FF);
  CAN.init_Filt(0, 0, 0x126);
  CAN.init_Mask(1, 0, 0x7FF);
  CAN.init_Filt(1, 0, 0x126);

  return true;
}

//= Run functions ===============================
void loop() {

#ifdef DEBUG_ERRORS
  if (millis() - lastDisplayErrorsTime >= 5000) {
    displayCANErrors();
    lastDisplayErrorsTime = millis();
  }
#endif

  if (canMessages.size() > 0 && millis() - lastPublishTime >= publishInterval && canMessages[0x126].timestamp > lastPublishTime) {

    std::map<unsigned long, CanMessage> snapshot;
    if (xSemaphoreTake(canMessagesMutex, portMAX_DELAY) == pdTRUE) {
      snapshot = canMessages;  //copy the map
      xSemaphoreGive(canMessagesMutex);
    }

    time_t now = time(nullptr);
    float motorTemperature = getMotorTempFromCanMessage(snapshot[0x126]);

    char msg_temp[20];
    char msg_millis[20];
    sprintf(msg_temp, "%.2f", motorTemperature);
    sprintf(msg_millis, "%d", now);
    String payload = "{\"timestsmp\":" + String(now) + ", \"state\":{\"motorTempC\":" + String(motorTemperature) + "}}";
    const char* mqttMsg = payload.c_str();

    if (!mqttClient.connected()) {
      Serial.print(F("Disconnected from AWS IoT. Error state: "));
      Serial.println(mqttClient.state());
      //setupAWSIoT();
      connectToAWSIoT();
    }

    //updateVehicleShadow("closed", 98.2, motorTemperature, motorTemperature);

    //Publish a test message
    Serial.print(F("publishing message: "));
    Serial.print(mqttMsg);

    if (mqttClient.publish("car/out", mqttMsg)) {
      Serial.println(F(" succeeded!"));
    } else {
      Serial.println(F(" failed."));
    }

    lastPublishTime = millis();
  }

  mqttClient.loop();
}

/* Handle processing off CAN BUS messages */
void CANTask(void* pvParameters) {
  delay(3000);
  Serial.println(F("CANTask started..."));
  while (1) {

    // Check for received data
    if (CAN_MSGAVAIL == CAN.checkReceive()) {

      long unsigned int rxId;
      unsigned char len = 0;
      unsigned char rxBuf[8];

      // Read the incoming message
      CAN.readMsgBuf(&rxId, &len, rxBuf);

      // Create a CAN message struct
      // Convert received data to CanMessage
      CanMessage message;
      message.id = rxId;
      message.data.assign(rxBuf, rxBuf + len);
      message.timestamp = millis();

      // Update the message in the map (store the latest message)
      if (xSemaphoreTake(canMessagesMutex, portMAX_DELAY) == pdTRUE) {
        canMessages[rxId] = message;
        xSemaphoreGive(canMessagesMutex);
      }
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);  // Allow other tasks to run
  }
}

/* Handle processing of incoming remote commands */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Serial.print(F("Message arrived ["));
  // Serial.print(topic);
  // Serial.print(F("] "));
  // for (int i = 0; i < length; i++) {
  //   Serial.print((char)payload[i]);
  // }
  // Serial.println();

  // Deserialize json string into json document
  String jsonString = String((char*)payload);
  const size_t capacity = JSON_OBJECT_SIZE(2) + 40;
  DynamicJsonDocument doc(capacity);
  DeserializationError error = deserializeJson(doc, jsonString);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }

  // Check if "cmd" key exists
  if (!doc.containsKey("cmd")) {
    Serial.println(F("Error: 'cmd' key is missing"));
    return;
  }

  int cmdValue = doc["cmd"];
  CommandID commandID = getCommandID(cmdValue);

  // Check if the command is valid
  if (commandID == CMD_INVALID) {
    Serial.print(F("Error: Invalid command received: "));
    Serial.println(cmdValue);
    return;
  }

  // Print the enum name
  Serial.print(F("Command received: "));
  Serial.println(commandIDToString(commandID));

  Command cmd;
  cmd.id = commandID;

  // Check if "params" exists and extract it, or set it to an empty string
  if (doc.containsKey("params")) {
    cmd.params = String(doc["params"].as<const char*>());  // Params found, assign value
  } else {
    cmd.params = "";  // Params not found, assign an empty string
  }

  processCommand(cmd);
}

/* Handle persisting vehicle state to cloud */
void updateVehicleShadow(const char* doorState, float batteryLevel, float temperature, float motorTemp) {
  // Create a JSON document for the shadow update
  StaticJsonDocument<512> doc;
  doc["state"]["reported"]["vehicleState"]["doors"] = doorState;
  doc["state"]["reported"]["vehicleState"]["batteryLevel"] = batteryLevel;
  doc["state"]["reported"]["vehicleState"]["temperature"] = motorTemp;
  doc["state"]["reported"]["vehicleState"]["motorTemp"] = motorTemp;

  // Serialize the JSON document to a string
  char shadowPayload[512];
  serializeJson(doc, shadowPayload);

  Serial.println(shadowPayload);

  // Publish to the shadow update topic
  if (mqttClient.publish(shadowUpdateTopic, shadowPayload)) {
    Serial.println("Shadow update published successfully!");
  } else {
    Serial.println("Shadow update failed.");
  }
}

//= Vehicle Commands ============================
const char* commandIDToString(CommandID cmd) {
  switch (cmd) {
    case CMD_READ_DATA: return "CMD_READ_DATA";
    case CMD_UNLOCK_DOORS: return "CMD_UNLOCK_DOORS";
    case CMD_LOCK_DOORS: return "CMD_LOCK_DOORS";
    case CMD_ROLL_WINDOWS_DOWN: return "CMD_ROLL_WINDOWS_DOWN";
    case CMD_ROLL_WINDOWS_UP: return "CMD_ROLL_WINDOWS_UP";
    case CMD_TOGGLE_WINDOW: return "CMD_TOGGLE_WINDOW";
    case CMD_VENT_WINDOWS: return "CMD_VENT_WINDOWS";
    case CMD_TURN_ON_HVAC: return "CMD_TURN_ON_HVAC";
    case CMD_TURN_OFF_HVAC: return "CMD_TURN_OFF_HVAC";
    case CMD_HONK: return "CMD_HONK";
    case CMD_OPEN_SUNROOF: return "CMD_OPEN_SUNROOF";
    case CMD_CLOSE_SUNROOF: return "CMD_CLOSE_SUNROOF";
    case CMD_TILT_SUNROOF: return "CMD_TILT_SUNROOF";
    case CMD_UNTILT_SUNROOF: return "CMD_UNTILT_SUNROOF";
    case CMD_FRONT_DEFROST: return "CMD_FRONT_DEFROST";
    case CMD_REAR_DEFROST: return "CMD_REAR_DEFROST";
    default: return "UNKNOWN_COMMAND";
  }
}

void rollWindowDown(int doorNum = 1, int ms = WINDOW_UPDOWN_DELAY) {
  int upPin, downPin;

  if (doorNum == 1) {
    upPin = DOOR1_WINUP_RELAY;
    downPin = DOOR1_WINDWN_RELAY;
  } else if (doorNum == 2) {
    upPin = DOOR2_WINUP_RELAY;
    downPin = DOOR2_WINDWN_RELAY;
  } else {
    Serial.println("Unrecognized door.");
    return;
  }

  Serial.print("Rolling window " + String(doorNum) + " down...");
  digitalWrite(upPin, false);
  digitalWrite(downPin, true);
  delay(ms);
  digitalWrite(downPin, false);
  Serial.println(F("done."));
}

void rollWindowUp(int doorNum = 1, int ms = WINDOW_UPDOWN_DELAY) {
  int upPin, downPin;

  if (doorNum == 1) {
    upPin = DOOR1_WINUP_RELAY;
    downPin = DOOR1_WINDWN_RELAY;
  } else if (doorNum == 2) {
    upPin = DOOR2_WINUP_RELAY;
    downPin = DOOR2_WINDWN_RELAY;
  } else {
    Serial.println(F("Unrecognized door."));
    return;
  }

  Serial.print("Rolling window " + String(doorNum) + " up...");
  digitalWrite(downPin, false);
  digitalWrite(upPin, true);
  delay(ms);
  digitalWrite(upPin, false);
  Serial.println(F("done."));
}

void ventWindow(int doorNum = 1, int ms = WINDOW_VENT_DELAY) {
  Serial.print("Venting window " + String(doorNum) + "...");
  rollWindowUp(doorNum);
  delay(1000);
  rollWindowDown(doorNum, ms);
  Serial.println(F("done."));
}

void toggleWindow(int doorNum = 1) {
  Serial.print("Toggling window " + String(doorNum) + " state. Currently: ");
  Serial.println(isWindowDown);
  if (isWindowDown) {
    rollWindowUp(doorNum);
  } else {
    rollWindowDown(doorNum);
  }
  isWindowDown = !isWindowDown;
}

void processCommand(Command cmd) {
  switch (cmd.id) {
    case CMD_ROLL_WINDOWS_DOWN:
      rollWindowDown(getDoorNumFromCommandParams(cmd));
      break;

    case CMD_ROLL_WINDOWS_UP:
      rollWindowUp(getDoorNumFromCommandParams(cmd));
      break;

    case CMD_TOGGLE_WINDOW:
      toggleWindow(getDoorNumFromCommandParams(cmd));
      break;

    case CMD_VENT_WINDOWS:
      ventWindow(getDoorNumFromCommandParams(cmd));
      break;
  }
}

int getDoorNumFromCommandParams(Command cmd) {
  Serial.println(cmd.params);
  return cmd.params.toInt();
}

//= Utility functions ===========================
float getMotorTempFromCanMessage(CanMessage message) {
  // Openinverter CAN mapping for Speedhut Gauge
  // The motor temperature is at offset 32 bits (4 bytes) and is 16 bits (2 bytes) long
  uint16_t rawTemperature = (message.data[5] << 8) | message.data[4];  // Combine two bytes for the temperature value

  // Apply the gain (divide by 10 to get the actual temperature in Celsius)
  float motorTemperature = rawTemperature / 10.0;
  return motorTemperature;
}

// Function to convert an integer to CommandID safely
CommandID getCommandID(int cmdId) {
  if (cmdId >= CMD_READ_DATA && cmdId <= CMD_REAR_DEFROST) {
    return static_cast<CommandID>(cmdId);
  }
  return CMD_INVALID;  // Return invalid command if the value is out of range
}

void displayCANErrors() {
  // Check for errors
  if (CAN.checkError()) {
    Serial.println(F("MCP2515 Error Detected!"));
    Serial.print(F("Errors RX: "));
    Serial.print(CAN.errorCountRX());
    Serial.print(F(", TX: "));
    Serial.println(CAN.errorCountTX());

    uint8_t errorFlag = CAN.getError();
    if (errorFlag & MCP_EFLG_RX1OVR) {
      Serial.println(F("Receive Buffer 1 Overflow Error"));
      //CAN.reset();
    }
    if (errorFlag & MCP_EFLG_RX0OVR) {
      Serial.println(F("Receive Buffer 0 Overflow Error"));
      //CAN.reset();
    }
    if (errorFlag & MCP_EFLG_TXBO) {
      Serial.println(F("Bus-Off Error"));
    }
    if (errorFlag & MCP_EFLG_TXEP) {
      Serial.println(F("Transmit Error-Passive"));
    }
    if (errorFlag & MCP_EFLG_RXEP) {
      Serial.println(F("Receive Error-Passive"));
    }
  } else {
    //Serial.println("No Errors Detected.");
  }
}

void listDir(fs::FS& fs, const char* dirname, uint8_t levels) {
  Serial.printf(F("Listing directory: %s\n"), dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println(F("Failed to open directory"));
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(F("Not a directory"));
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print(F("DIR : "));
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print(F("FILE: "));
      Serial.print(file.name());
      Serial.print(F("  SIZE: "));
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

// Function to read file content into a char array
void readFileToBuffer(const char* path, char* buffer, size_t bufferSize) {
  File file = SPIFFS.open(path, "r");
  if (file) {
    size_t len = file.size();
    if (len > bufferSize - 1) {
      len = bufferSize - 1;
    }
    file.readBytes(buffer, len);
    buffer[len] = '\0';  // Ensure null-termination
    file.close();
  } else {
    Serial.printf("Failed to open file: %s\n", path);
  }
}

void saveStateToFlash(const char* filename, const JsonDocument& jsonDoc) {
  // Open file for writing
  File file = SPIFFS.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  // Serialize the JSON document and write to file
  if (serializeJson(jsonDoc, file) == 0) {
    Serial.println("Failed to write JSON to file");
  } else {
    Serial.println("JSON saved to file successfully");
  }

  // Close the file
  file.close();
}

bool loadStateFromFlash(const char* filename, JsonDocument& jsonDoc) {
  // Open file for reading
  File file = SPIFFS.open(filename, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return false;
  }

  // Parse the JSON document from file
  DeserializationError error = deserializeJson(jsonDoc, file);
  if (error) {
    Serial.print("Failed to read file, error: ");
    Serial.println(error.f_str());
    file.close();
    return false;
  }

  // Close the file
  file.close();
  return true;
}

void syncSystemToNetworkTime() {
  // Send the AT command to enable NITZ
  modem.sendAT("+CLTS=1;&W");  //&W is used to write the state permanently to Modem. So even if the modem is restarted CLTS will be enabled.

  // Verify the command response (wait for "OK")
  if (modem.waitResponse(10000L) == 1) {
    SerialMon.println(F("NITZ enabled successfully."));
  } else {
    SerialMon.println(F("Failed to enable NITZ."));
  }

  modem.sendAT("+CTZU=1");  // Enable automatic time zone update
  if (modem.waitResponse(10000L) == 1) {
    SerialMon.println(F("Automatic time zone update enabled."));
  } else {
    SerialMon.println(F("Failed to enable automatic time zone update."));
  }

  Serial.print(F("Syncing to network time..."));
  String networkTime = "";
  while (networkTime.indexOf("+CCLK:") == -1) {
    // Request the current time from the network
    modem.sendAT("+CCLK?");
    networkTime = modem.stream.readStringUntil('\n');
    if (networkTime.indexOf("+CCLK:") == -1) {
      Serial.print(F("."));
      delay(1000);
    }
  }
  Serial.println("\nNetwork time: " + networkTime);

  // Parse the network time and set the ESP32 system time to match
  int tzOffset = 0;
  struct tm currentTime = parseNetworkTimeWithOffset(networkTime, tzOffset);
  if (currentTime.tm_year > 0) {
    adjustToUTC(currentTime, tzOffset);
    setEsp32Time(currentTime);
  } else {
    Serial.println(F("Failed to parse network time."));
  }
}

// Set the ESP32's internal clock
void setEsp32Time(struct tm t) {
  time_t timeSinceEpoch = mktime(&t);
  struct timeval now = { .tv_sec = timeSinceEpoch, .tv_usec = 0 };
  settimeofday(&now, NULL);
  SerialMon.printf("System time set to UTC: %02d-%02d-%04d %02d:%02d:%02d\n",
                   t.tm_mday, t.tm_mon + 1, t.tm_year + 1900,
                   t.tm_hour, t.tm_min, t.tm_sec);
}

// Parse modem time and set ESP32 time
struct tm parseNetworkTimeWithOffset(String modemTime, int& tzOffset) {
  struct tm t;
  int firstQuote = modemTime.indexOf('"');
  int secondQuote = modemTime.lastIndexOf('"');

  if (firstQuote == -1 || secondQuote == -1 || secondQuote <= firstQuote) {
    SerialMon.println(F("Invalid time format."));
    return t;
  }

  String timeString = modemTime.substring(firstQuote + 1, secondQuote);
  t.tm_year = timeString.substring(0, 2).toInt() + 100;
  t.tm_mon = timeString.substring(3, 5).toInt() - 1;
  t.tm_mday = timeString.substring(6, 8).toInt();
  t.tm_hour = timeString.substring(9, 11).toInt();
  t.tm_min = timeString.substring(12, 14).toInt();
  t.tm_sec = timeString.substring(15, 17).toInt();
  t.tm_isdst = 0;

  String offsetString = timeString.substring(18);
  tzOffset = offsetString.toInt();
  return t;
}

void adjustToUTC(struct tm& t, int tzOffset) {
  // Convert the timezone offset from quarter-hours to seconds
  int offsetInSeconds = tzOffset * 15 * 60;

  // Get time_t (seconds since epoch) from the local time
  time_t localTime = mktime(&t);

  // Adjust by the offset to get UTC time
  time_t utcTime = localTime - offsetInSeconds;

  // Convert the time_t back to struct tm in UTC
  struct tm* utcTm = gmtime(&utcTime);

  // Update the original time structure with the UTC time
  t = *utcTm;
}
