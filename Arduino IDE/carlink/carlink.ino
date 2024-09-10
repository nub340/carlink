#include <Arduino.h>

// Define modem type before including TinyGsmClient
#define TINY_GSM_MODEM_SIM7000

#include <SPIFFS.h>
#include <SSLClient.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>

// Define the CS and INT pins
#define CAN_CS 5
#define CAN_INT 27

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

bool setupSuccessful = false;
char ca_cert[CERT_BUF_SIZE];
char client_cert[CERT_BUF_SIZE];
char client_key[CERT_BUF_SIZE];

const char apn[] = "h2g2";
const char gprsUser[] = "";
const char gprsPass[] = "";

// AWS IoT Core settings
const char* awsEndpoint = "a11mldvlf3x3z0-ats.iot.us-west-1.amazonaws.com";
const int port = 8883;

MCP_CAN CAN(CAN_CS);  // Create CAN object with CS pin

// Serial and modem initialization
// SoftwareSerial SerialAT(12, 13); // RX, TX for the SIM7000G module
TinyGsm modem(SerialAT);
TinyGsmClient base_client(modem);
SSLClient client(&base_client);
PubSubClient mqttClient(client);

void setup() {
  Serial.begin(115200);
  while(!Serial);
  delay(500);

  Serial.println();
  
  // Initialize MCP2515 CAN controller at 500kbps speed
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515.");
    while (1);
  }

  // Enable CAN interrupt
  pinMode(CAN_INT, INPUT);
  CAN.setMode(MCP_NORMAL);  // Set to normal mode to start receiving

  Serial.println("\Configuring connectivity...");
  setupSuccessful = loadCertificates() && setupModem() && setupAWSIoT() && connectToAWS();
  if(setupSuccessful) {
    Serial.println("Setup complete. Running...");
  } else {
    Serial.println("Setup failed!");
  }
}

bool loadCertificates() {
  if (!SPIFFS.begin(true)) {  // 'true' will format the file system if it's corrupted
    Serial.println("SPIFFS Mount Failed");
    return false;;
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
  //modem.restart();

  Serial.print("Connecting to cellular network...");
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println("Failed to connect to cellular network. Restarting...");
    modem.restart();
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) { 
      Serial.println("Failed to connect to cellular network, again.");
    }
    return false;
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
  mqttClient.setCallback(mqttCallback);

  return true;
}

bool connectToAWS() {
  Serial.println("Connecting to AWS IoT Core...");
  while (!mqttClient.connected()) {
    if (mqttClient.connect("ESP32_DevModule")) {
      Serial.println("Connected to AWS IoT Core.");
      mqttClient.subscribe("car/commands");
    } else {
      Serial.print("Failed to connect. Error state=");
      Serial.println(mqttClient.state());
      delay(5000);
    }
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
}

void loop() {

  // Check for received data
  if (!digitalRead(CAN_INT)) {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];

    // Read the data
    CAN.readMsgBuf(&rxId, &len, rxBuf);

    Serial.print("Received data with ID: ");
    Serial.println(rxId, HEX);

    Serial.print("Data: ");
    for (int i = 0; i < len; i++) {
      Serial.print(rxBuf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Publish a test message
    mqttClient.publish("car/responses", "Hello from ESP32 via Cellular");
    Serial.println("published");
  }
  
//  if (!mqttClient.connected()) {
//    connectToAWS();
//  }
  mqttClient.loop();
  delay(1000);
}

//= Helpers ==================================================

void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
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
    buffer[len] = '\0'; // Ensure null-termination
    file.close();
  } else {
    Serial.printf("Failed to open file: %s\n", path);
  }
}


//#include "CarControl.h"
//#include "Commands.h"
//
//void setup() {
//    Serial.begin(115200);
//    // Initialize car control system
//    CarControl::init();
//    // Other setup code
//}
//
//void loop() {
//  if (CarControl::isNetworkConnected()) {
//        // Execute commands or handle communication with the server
//        CarControl::executeCommand(CMD_UNLOCK_DOORS);
//    } else {
//        Serial.println("Network not connected, retrying...");
//        CarControl::connectToNetwork();
//    }
//
//    delay(5000); // Wait 5 seconds
//}
