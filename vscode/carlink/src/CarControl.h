#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include "Config.h"         // Includes the configuration file with the modem define
#include <TinyGsmClient.h>
#include <Wire.h>
#include <SPI.h>

#include "Commands.h"

class CarControl {
public:
    static void init();
    static void executeCommand(CommandID commandID);
    static void connectToNetwork();
    static bool isNetworkConnected();
    static bool testHTTP();

private:
    static TinyGsm modem;
    static TinyGsmClient client;
};

#endif // CAR_CONTROL_H
