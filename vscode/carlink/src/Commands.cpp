#include "Commands.h"

namespace Commands {

    void unlockDoors() {
        Serial.println("Doors unlocked.");
    }

    void rollWindowsDown() {
        Serial.println("Windows rolled down.");
    }

    void turnOnAC() {
        Serial.println("A/C turned on.");
    }

    CarCommand commandList[] = {
        {CMD_UNLOCK_DOORS, "Unlock Doors", unlockDoors},
        {CMD_ROLL_WINDOWS_DOWN, "Roll Windows Down", rollWindowsDown},
        {CMD_TURN_ON_HVAC, "Turn on A/C", turnOnAC}
    };

    void executeCommand(CommandID commandID) {
        for (const auto& command : commandList) {
            if (command.commandID == commandID) {
                command.execute();
                return;
            }
        }
        Serial.println("Command not found.");
    }
}
