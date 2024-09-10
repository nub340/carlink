#ifndef CONNECTIVITY_MANAGER_H
#define CONNECTIVITY_MANAGER_H

#include "ConnectivityManagerConfig.h"         // Includes the configuration file with the modem define
#include "utils.h"
#include <SPIFFS.h>
#include <SSLClient.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>

class ConnectivityManager {
  public:
    static bool init();
    static bool loadCertificates();
    static bool setupModem();
    static bool connectToNetwork();
    static bool setupAWSIoT();
    static bool connectToAWS();
    static PubSubClient mqttClient;
    static char ca_cert[CERT_BUF_SIZE];
    static char client_cert[CERT_BUF_SIZE];
    static char client_key[CERT_BUF_SIZE];

  private:
    static SSLClient client;
    
    static void mqttCallback(char* topic, byte* payload, unsigned int length);
};

#endif // CONNECTIVITY_MANAGER_H
