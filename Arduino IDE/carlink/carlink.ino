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
#define CAN_INT 2
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

MCP_CAN CAN(CAN_CS);  // Create CAN object with CS pin

// Serial and modem initialization
// SoftwareSerial SerialAT(12, 13); // RX, TX for the SIM7000G module
TinyGsm modem(SerialAT);
TinyGsmClient base_client(modem);
SSLClient client(&base_client);
PubSubClient mqttClient(client);

// Queue for background task queuing and processesing
QueueHandle_t mqttQueue;

// Structure to hold raw CAN data for MQTT publishing
struct CanMessage {
  unsigned long id;
  std::vector<uint8_t> data;
};

// Data structure to store CAN messages
std::map<unsigned long, CanMessage> canMessages;

// Define a set of CAN IDs you are interested in
std::unordered_set<unsigned long> canMessageIdsToPublish = { 0x126 };

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  Serial.flush();
  Serial.println();
  Serial.println("Begin setup...");

  pinMode(0, OUTPUT);
  digitalWrite(0, LOW);

  Serial.println("\Configuring connectivity...");
  if (loadCertificates() && setupModem() && setupAWSIoT() && connectToAWS()) {
    Serial.println("Connected to AWS. Ready to send and receive...");
  } else {
    Serial.println("Setup failed!");
  }

  // Initialize MCP2515 CAN controller at 500kbps speed
  SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN_CS);
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515.");
    while (1)
      ;
  }

  mqttQueue = xQueueCreate(10, sizeof(CanMessage));

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
    enqueueCanMessages();
  }

  mqttClient.loop();
}

void enqueueCanMessages() {
  // Create a copy of the map or its entries
  for (const auto& entry : canMessages) {
    CanMessage message = entry.second;               // Copy message
    xQueueSend(mqttQueue, &message, portMAX_DELAY);  // Send copy to queue
  }
  //canMessages.clear();  // Optionally clear the map if it’s only used for temporary storage
}

// Task to handle MQTT publishing
void MQTTTask(void* pvParameters) {
  Serial.println("MQTTTask");
  CanMessage msg;
  char payload[100];

  while (1) {
    // Wait for new CAN messages in the queue
    if (xQueueReceive(mqttQueue, &msg, portMAX_DELAY) == pdPASS && millis() - lastPublishTime >= publishInterval) {

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
        Serial.println("reconnecting to MQTT broker...");
        Serial.println(mqttClient.state());
        yield();
        connectToAWS();
      }

      Serial.print("Free heap before publish: ");
      Serial.println(ESP.getFreeHeap());

      //Publish a test message
      Serial.print("publishing message: ");
      Serial.println(payload.c_str());
      if (mqttClient.publish("car/out", "test")) {
        Serial.println(" succeeded!");
      } else {
        Serial.println(" failed.");
      }
      yield();  // Yield CPU control to other tasks

      Serial.print("Free heap after publish: ");
      Serial.println(ESP.getFreeHeap());

      lastPublishTime = millis();
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);  // Allow other tasks to run
  }
}

bool loadCertificates() {
  if (!SPIFFS.begin(true)) {  // 'true' will format the file system if it's corrupted
    Serial.println("SPIFFS Mount Failed");
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

  Serial.println("Initializing modem...");
  modem.restart();

  Serial.println("Connecting to cellular network...");
  while (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    connectCount++;
    Serial.print("Failed to connect to cellular network. Fail count: ");
    Serial.println(connectCount);
    Serial.println("Restarting modem...");
    modem.restart();
  }
  Serial.println("Connected to cellular network");
  return true;
}

bool setupAWSIoT() {

  // Load AWS IoT certificates
  client.setCACert(ca_cert);
  client.setCertificate(client_cert);
  client.setPrivateKey(client_key);

  mqttClient.setKeepAlive(60);
  mqttClient.setServer(awsEndpoint, port);
  //mqttClient.setCallback(mqttCallback);
  //mqttClient.setDebugOutput(true);

  return true;
}

bool connectToAWS() {
  Serial.println("Connecting to AWS IoT Core...");
  while (!mqttClient.connected()) {
    if (mqttClient.connect("ESP32_DevModule")) {
      Serial.print("Connected to AWS IoT Core endpoint: ");
      Serial.println(awsEndpoint);
      //mqttClient.subscribe("car/in");
    } else {
      Serial.print("Failed to connect. Error state=");
      Serial.println(mqttClient.state());
      delay(5000);
    }
    yield();
  }
  return true;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  rollWindowDown();
}

void displayCANErrors() {
  // Check for errors
  if (CAN.checkError()) {
    Serial.println("MCP2515 Error Detected!");
    Serial.print("Errors RX: ");
    Serial.print(CAN.errorCountRX());
    Serial.print(", TX: ");
    Serial.println(CAN.errorCountTX());

    uint8_t errorFlag = CAN.getError();
    if (errorFlag & MCP_EFLG_RX1OVR) {
      Serial.println("Receive Buffer 1 Overflow Error");
      //CAN.reset();
    }
    if (errorFlag & MCP_EFLG_RX0OVR) {
      Serial.println("Receive Buffer 0 Overflow Error");
      //CAN.reset();
    }
    if (errorFlag & MCP_EFLG_TXBO) {
      Serial.println("Bus-Off Error");
    }
    if (errorFlag & MCP_EFLG_TXEP) {
      Serial.println("Transmit Error-Passive");
    }
    if (errorFlag & MCP_EFLG_RXEP) {
      Serial.println("Receive Error-Passive");
    }
  } else {
    //Serial.println("No Errors Detected.");
  }
}

void rollWindowDown() {
  digitalWrite(0, !digitalRead(0));
}

//= Helpers ==================================================

void listDir(fs::FS& fs, const char* dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
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
