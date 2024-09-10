#ifndef COMMANDS_H
#define COMMANDS_H

#include <Arduino.h>

enum CommandID {
    CMD_READ_DATA = 1,
    CMD_UNLOCK_DOORS,
    CMD_LOCK_DOORS,
    CMD_ROLL_WINDOWS_DOWN,
    CMD_ROLL_WINDOWS_UP,
    CMD_VENT_WINDOWS,
    CMD_TURN_ON_HVAC,
    CMD_TUEN_OFF_HVAC,
    CMD_HONK,
    CMD_OPEN_SUNROOF,
    CMD_CLOSE_SUNROOF,
    CMD_TILT_SUNROOF,
    CMD_UNTILT_SUNROOF,
    CMD_FRONT_DEFROST,
    CMD_REAR_DEFROST
};

typedef void (*CommandFunction)();

struct CarCommand {
    CommandID commandID;
    const char* description;
    CommandFunction execute;
};

namespace Commands {
    void executeCommand(CommandID commandID);
}

#endif // COMMANDS_H
