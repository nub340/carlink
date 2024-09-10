#include "CarControl.h"

// Initialize the TinyGsm modem and client
//TinyGsm CarControl::modem(SerialAT);
//TinyGsmClient CarControl::client(modem);

void CarControl::init() {
    // Initialize any required components
//    Serial.println("\nCarLink Initializing.");
//
//    // Start the modem
//    SerialMon.println("Initializing modem...");
//    
//    // Initialize Serial1 with appropriate baud rate
//    // Start the modem serial communication
//    SerialAT.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
//    // SerialAT.begin(14400, SERIAL_8N1, MODEM_RX, MODEM_TX);
//    delay(3000);
//
//    pinMode(MODEM_PWRKEY, OUTPUT);
//    pinMode(MODEM_POWER_ON, OUTPUT);
//    digitalWrite(MODEM_PWRKEY, HIGH);
//    digitalWrite(MODEM_POWER_ON, HIGH);
//
//    delay(3000);
//    modem.restart();
//
//    // Connect and test connection. 
//    // Optionally, you can perform additional setup here
//    connectToNetwork();
//    testHTTP();
}

void CarControl::connectToNetwork() {
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
}

bool CarControl::isNetworkConnected() {
    //return modem.isNetworkConnected();
}

bool CarControl::testHTTP() {
//  // Make an HTTP GET request
//  if (client.connect(testHttpServer, testHttpServerPort)) {
//    SerialMon.println("Connected to the server.");
//    client.print(String("GET /get HTTP/1.1\r\n") +
//                 "Host: " + testHttpServer + "\r\n" +
//                 "Connection: close\r\n\r\n");
//
//    // Read the response and print it to the serial monitor
//    while (client.connected() || client.available()) {
//      if (client.available()) {
//        String line = client.readStringUntil('\n');
//        SerialMon.println(line);
//      }
//    }
//    client.stop();
//    SerialMon.println("HTTP Server disconnected.");
//    return true;
//  } else {
//    SerialMon.println("Failed to connect to the HTTP server.");
//    return false;
//  }
}

void CarControl::executeCommand(CommandID commandID) {
    //Commands::executeCommand(commandID);
}
