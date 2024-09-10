#ifndef CONFIG_H
#define CONFIG_H

// Define modem type before including TinyGsmClient
#define TINY_GSM_MODEM_SIM7000

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

const char apn[] = "h2g2"; 
const char gprsUser[] = ""; 
const char gprsPass[] = "";

// Test server details
const char testHttpServer[] = "httpbin.org";
const int testHttpServerPort = 80;

// AWS IoT Core settings
//const char* awsEndpoint = "a11mldvlf3x3z0-ats.iot.us-west-1.amazonaws.com";
//const int port = 8883;

#endif // CONFIG_H
