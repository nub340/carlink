//#include "ConnectivityManager.h"
//
//// Initialize the TinyGsm modem and clients
//TinyGsm modem(SerialAT);
//TinyGsmClient base_client(modem);
//SSLClient ConnectivityManager::client(&base_client);
//PubSubClient ConnectivityManager::mqttClient(client);
//
//bool ConnectivityManager::init() {
//  // Initialize any required components
//  Serial.println("\Cellular Connectivity Initializing.");
//  return loadCertificates() && setupModem() && connectToNetwork() && setupAWSIoT() && connectToAWS();
//}
//
//bool ConnectivityManager::loadCertificates() {
//  if (!SPIFFS.begin(true)) {  // 'true' will format the file system if it's corrupted
//    Serial.println("SPIFFS Mount Failed");
//    return false;;
//  }
//
//  // List files in the root directory
//  listDir(SPIFFS, "/", 0);
//
//  // Read certificates from SPIFFS
//  readFileToBuffer("/ca.crt", ca_cert, CERT_BUF_SIZE);
//  readFileToBuffer("/client.crt", client_cert, CERT_BUF_SIZE);
//  readFileToBuffer("/client.key", client_key, CERT_BUF_SIZE);
//
//  if (!ca_cert || !client_cert || !client_key) {
//    Serial.println("Failed to load certificates from flash storage");
//    return false;
//  } else {
//    Serial.println("Successfully loaded certificates");
////    Serial.println("AmazonRootCA1:");
////    Serial.println(ca_cert);
////    Serial.println();
////    Serial.println("Client Cert:");
////    Serial.println(client_cert);
////    Serial.println();
////    Serial.println("Client Key:");
////    Serial.println(client_key);
//  }
//
//  return true;
//}
//
//bool ConnectivityManager::setupModem() {
//  // Initialize Serial1 with appropriate baud rate
//  // Start the modem serial communication
//  SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
//  delay(3000);
//
//  pinMode(MODEM_PWRKEY, OUTPUT);
//  pinMode(MODEM_POWER_ON, OUTPUT);
//  digitalWrite(MODEM_PWRKEY, HIGH);
//  digitalWrite(MODEM_POWER_ON, HIGH);
//
//  Serial.println("Initializing modem...");
//  //modem.restart();
//
//  Serial.print("Connecting to cellular network...");
//  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
//    Serial.println("Failed to connect to cellular network. Restarting...");
//    modem.restart();
//    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) { 
//      Serial.println("Failed to connect to cellular network, again.");
//    }
//    return false;
//  }
//  Serial.println("Connected to cellular network");
//  return true;
//}
//
//bool ConnectivityManager::connectToNetwork() {
//    SerialMon.print("Connecting to ");
//    SerialMon.print(apn);
//    SerialMon.println(" network...");
//
//    // Set the modem's APN
//    modem.gprsConnect(apn, gprsUser, gprsPass);
//
//    // Wait for network connection
//    while (!modem.isNetworkConnected()) {
//        SerialMon.print(".");
//        delay(1000);
//    }
//
//    SerialMon.println("\nNetwork connected!");
//    return true;
//}
//
//bool ConnectivityManager::setupAWSIoT() {
//
//  // Load AWS IoT certificates
//  client.setCACert(ca_cert);
//  client.setCertificate(client_cert);
//  client.setPrivateKey(client_key);
//
//  char* awsEndpoint = "a11mldvlf3x3z0-ats.iot.us-west-1.amazonaws.com";
//  int port = 8883;
//
//  mqttClient.setServer(awsEndpoint, port);
//  mqttClient.setCallback(mqttCallback);
//
//  return true;
//}
//
//bool ConnectivityManager::connectToAWS() {
//  Serial.println("Connecting to AWS IoT Core...");
//  while (!mqttClient.connected()) {
//    if (mqttClient.connect("ESP32_DevModule")) {
//      Serial.println("Connected to AWS IoT Core.");
//      mqttClient.subscribe("car/commands");
//    } else {
//      Serial.print("Failed to connect. Error state=");
//      Serial.println(mqttClient.state());
//      delay(5000);
//    }
//  }
//  return true;
//}
//
//void ConnectivityManager::mqttCallback(char* topic, byte* payload, unsigned int length) {
//  Serial.print("Message arrived [");
//  Serial.print(topic);
//  Serial.print("] ");
//  for (int i = 0; i < length; i++) {
//    Serial.print((char)payload[i]);
//  }
//  Serial.println();
//}
