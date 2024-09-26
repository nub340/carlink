#include <Arduino.h>

// Define modem type before including TinyGsmClient
#define TINY_GSM_MODEM_SIM7000

//#define DEBUG_ERRORS

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
#define WINDOW_UP_RELAY 0
#define WINDOW_DOWN_RELAY 25
#define WINDOW_UPDOWN_DELAY 6000
#define WINDOW_VENT_DELAY 600

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

// AWS IoT Core settings
const char* awsEndpoint = "a11mldvlf3x3z0-ats.iot.us-west-1.amazonaws.com";
const int port = 8883;

int connectCount = 0;
unsigned long lastDisplayErrorsTime = 0;
static volatile unsigned long lastPublishTime = 0;
unsigned long publishInterval = 5000;
bool isWindowDown = false;

MCP_CAN CAN(CAN_CS);  // Create CAN object with CS pin

// Serial and modem initialization
// SoftwareSerial SerialAT(12, 13); // RX, TX for the SIM7000G module
TinyGsm modem(SerialAT);
TinyGsmClient base_client(modem);
SSLClient client(&base_client);
PubSubClient mqttClient(client);

// Queue for background task queuing and processesing
//QueueHandle_t mqttQueue;

// Structure to hold raw CAN data for MQTT publishing
struct CanMessage {
  unsigned long id;
  std::vector<uint8_t> data;
};

// Data structure to store CAN messages
std::map<unsigned long, CanMessage> canMessages;

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

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  Serial.flush();
  Serial.println();
  Serial.println(F("Begin setup..."));

  pinMode(0, OUTPUT);
  digitalWrite(0, LOW);

  Serial.println(F("\Configuring connectivity..."));
  if (loadCertificates() && setupModem() && setupAWSIoT() && connectToAWS()) {
    Serial.println(F("Connected to AWS. Ready to send and receive..."));
  } else {
    Serial.println(F("Setup failed!"));
  }

  // Initialize MCP2515 CAN controller at 500kbps speed
  SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println(F("MCP2515 Initialized Successfully!"));
  } else {
    Serial.println(F("Error Initializing MCP2515."));
    while (1)
      ;
  }

  //mqttQueue = xQueueCreate(100, sizeof(std::map<unsigned long, CanMessage>));

  // Create a task for publishing to MQTT
  xTaskCreatePinnedToCore(
    MQTTTask,    // Task function
    "MQTTTask",  // Name of the task
    4096,        // Stack size
    NULL,        // Task input parameter
    1,           // Priority of the task
    NULL,        // Task handle
    0            // Pin task to core 1
  );

  // relay pins
  pinMode(WINDOW_UP_RELAY, OUTPUT);
  pinMode(WINDOW_DOWN_RELAY, OUTPUT);

  // Enable CAN interrupt
  pinMode(CAN_INT, INPUT);
  pinMode(CAN_CS, OUTPUT);
  CAN.setMode(MCP_NORMAL);  // Set to normal mode to start receiving

  // Set mask & filter to only process motor temp (0x126)
  CAN.init_Mask(0, 0, 0x7FF);
  CAN.init_Filt(0, 0, 0x126);
  CAN.init_Mask(1, 0, 0x7FF);
  CAN.init_Filt(1, 0, 0x126);

  lastPublishTime = millis();
}

void loop() {
#ifdef DEBUG_ERRORS
  if (millis() - lastDisplayErrorsTime >= 5000) {
    displayCANErrors();
    lastDisplayErrorsTime = millis();
  }
#endif

  // Check for received data
  if (CAN_MSGAVAIL == CAN.checkReceive()) {

    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];

    // Read the incoming message
    CAN.readMsgBuf(&rxId, &len, rxBuf);

    //if (canMessageIdsToPublish.find(rxId) != canMessageIdsToPublish.end()) {
    // Create a CAN message struct
    // Convert received data to CanMessage
    CanMessage message;
    message.id = rxId;
    message.data.assign(rxBuf, rxBuf + len);

    // Update the message in the map (store the latest message)
    canMessages[rxId] = message;
    //}
  } else {
    //Serial.print(".");
  }

  // enqueue the latest data for publishing if time interval has elapsed
  if (millis() - lastPublishTime >= publishInterval) {
    //enqueueCanMessages();
  }

  mqttClient.loop();
}

// void enqueueCanMessages() {
//   // Create a copy of the map or its entries
//   // for (const auto& entry : canMessages) {
//   //   CanMessage message = entry.second;               // Copy message
//   //   xQueueSend(mqttQueue, &message, portMAX_DELAY);  // Send copy to queue
//   // }
//   std::map<unsigned long, CanMessage> canMessagesSnapshot(canMessages);
//   xQueueSend(mqttQueue, &canMessagesSnapshot, portMAX_DELAY);

//   //canMessages.clear();  // Optionally clear the map if it’s only used for temporary storage
// }

// Task to handle MQTT publishing
void MQTTTask(void* pvParameters) {
  Serial.println(F("MQTTTask"));
  //std::map<unsigned long, CanMessage> snapshot;
  char payload[100];

  while (1) {
    // Wait for new CAN messages in the queue
    //if (xQueueReceive(mqttQueue, &snapshot, portMAX_DELAY) == pdPASS && millis() - lastPublishTime >= publishInterval) {
    if (false && millis() - lastPublishTime >= publishInterval) {

      std::map<unsigned long, CanMessage> snapshot(canMessages);
      CanMessage msg = snapshot[0x126];

      // The motor temperature is at offset 32 bits (4 bytes) and is 16 bits (2 bytes) long
      uint16_t rawTemperature = (msg.data[5] << 8) | msg.data[4];  // Combine two bytes for the temperature value

      // Apply the gain (divide by 10 to get the actual temperature)
      float motorTemperature = rawTemperature / 10.0;

      char msg_temp[20];
      char msg_millis[20];
      sprintf(msg_temp, "%.2f", motorTemperature);
      sprintf(msg_millis, "%d", millis());
      String payload = "{\"motorTemp\":\"" + String(msg_temp) + " °C\",\"millis\":\"" + msg_millis + "\"}";

      if (!mqttClient.connected()) {
        Serial.println(F("reconnecting to MQTT broker..."));
        Serial.println(mqttClient.state());
        yield();
        connectToAWS();
      }

      // Serial.print("Free heap before publish: ");
      // Serial.println(ESP.getFreeHeap());

      //Publish a test message
      Serial.print(F("publishing message: "));
      Serial.println(payload.c_str());
      // if (mqttClient.publish("car/out", payload.c_str())) {
      //   Serial.println(F(" succeeded!"));
      // } else {
      //   Serial.println(F(" failed."));
      // }
      yield();  // Yield CPU control to other tasks

      // Serial.print("Free heap after publish: ");
      // Serial.println(ESP.getFreeHeap());

      lastPublishTime = millis();
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);  // Allow other tasks to run
  }
}

bool loadCertificates() {
  if (!SPIFFS.begin(true)) {  // 'true' will format the file system if it's corrupted
    Serial.println(F("SPIFFS Mount Failed"));
    return false;
    delay(100);
  }

  // List files in the root directory
  listDir(SPIFFS, "/", 0);

  // Read certificates from SPIFFS
  readFileToBuffer("/ca.crt", ca_cert, CERT_BUF_SIZE);
  readFileToBuffer("/client.crt", client_cert, CERT_BUF_SIZE);
  readFileToBuffer("/client.key", client_key, CERT_BUF_SIZE);

  if (!ca_cert || !client_cert || !client_key) {
    Serial.println("Failed to load certificates from flash storage");
    return false;
  } else {
    Serial.println("Successfully loaded certificates");
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

  Serial.println(F("Initializing modem..."));
  modem.restart();

  Serial.println(F("Connecting to cellular network..."));
  while (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    connectCount++;
    Serial.print(F("Failed to connect to cellular network. Fail count: "));
    Serial.println(connectCount);
    Serial.println(F("Restarting modem..."));
    modem.restart();
  }
  Serial.println(F("Connected to cellular network"));
  return true;
}

bool setupAWSIoT() {

  // Load AWS IoT certificates
  client.setCACert(ca_cert);
  client.setCertificate(client_cert);
  client.setPrivateKey(client_key);

  mqttClient.setKeepAlive(60);
  mqttClient.setServer(awsEndpoint, port);
  mqttClient.setCallback(mqttCallback);

  return true;
}

bool connectToAWS() {
  Serial.println(F("Connecting to AWS IoT Core..."));
  while (!mqttClient.connected()) {
    if (mqttClient.connect("ESP32_DevModule")) {
      Serial.print(F("Connected to AWS IoT Core endpoint: "));
      Serial.println(awsEndpoint);
      mqttClient.subscribe("car/in");
    } else {
      Serial.print(F("Failed to connect. Error state="));
      Serial.println(mqttClient.state());
      delay(5000);
    }
    yield();
  }
  return true;
}

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

void rollWindowDown(int ms = WINDOW_UPDOWN_DELAY) {
  Serial.print(F("Rolling window down..."));
  digitalWrite(WINDOW_UP_RELAY, false);
  digitalWrite(WINDOW_DOWN_RELAY, true);
  Serial.println(digitalRead(WINDOW_DOWN_RELAY));
  delay(ms);
  digitalWrite(WINDOW_DOWN_RELAY, false);
  Serial.println(F("done."));
  Serial.println(digitalRead(WINDOW_DOWN_RELAY));
}

void rollWindowUp(int ms = WINDOW_UPDOWN_DELAY) {
  Serial.print(F("Rolling window up..."));
  digitalWrite(WINDOW_DOWN_RELAY, false);
  digitalWrite(WINDOW_UP_RELAY, true);
  Serial.println(digitalRead(WINDOW_UP_RELAY));
  delay(ms);
  digitalWrite(WINDOW_UP_RELAY, false);
  Serial.println(F("done."));
  Serial.println(digitalRead(WINDOW_UP_RELAY));
}

void ventWindow(int ms = WINDOW_VENT_DELAY) {
  Serial.print(F("Venting window..."));
  rollWindowUp();
  rollWindowDown(ms);
}

void processCommand(Command cmd) {
  switch (cmd.id) {
    case CMD_ROLL_WINDOWS_DOWN:
      rollWindowDown();
      break;

    case CMD_ROLL_WINDOWS_UP:
      rollWindowUp();
      break;

    case CMD_TOGGLE_WINDOW:
      toggleWindow();
      break;

    case CMD_VENT_WINDOWS:
      ventWindow();
      break;
  }
}

void toggleWindow() {
  Serial.print(F("Toggling windw state. Currently: "));
  Serial.println(isWindowDown);
  if (isWindowDown) {
    rollWindowUp();
  } else {
    rollWindowDown();
  }
  isWindowDown = !isWindowDown;
}

//= Helpers ==================================================
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
