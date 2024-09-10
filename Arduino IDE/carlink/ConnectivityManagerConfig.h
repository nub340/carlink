#ifndef CONNECTIVITY_MANAGER_CONFIG_H
#define CONNECTIVITY_MANAGER_CONFIG_H

// Define modem type before including TinyGsmClient
#define TINY_GSM_MODEM_SIM7000
#define CERT_BUF_SIZE 2048

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

#endif // CONNECTIVITY_MANAGER_CONFIG_H
