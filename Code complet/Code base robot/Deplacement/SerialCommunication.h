#ifndef SerialCommunication_h
#define SerialCommunication_h

#include "Arduino.h"

#define SERIALCOM0 Serial
#define SERIALCOM0NUMBER 0
#define SERIALCOM1 SerialUSB
#define SERIALCOM1NUMBER 1

union SerialCom {
    bool flags[11];
    struct {
        bool flagVerbose;
        bool flagDisplayCount;
        bool flagStopTimer;
        bool flagDisplayRpm;
        bool flagDisplaySetpointPidSpeed;
        bool flagExportDataToExcel;
        bool flagDisplayRobotSpeedsOrder;
        bool flagDisplayRobotSpeedsMeasured;
        bool flagDisplayGyroSpeedZ;
        bool flagDisplayGyroAngleZ;
        bool flagDisplayCurrentPosition;
    };
};

void initSerialCommunication(bool serialComNumber, uint32_t baudRate);
void resetSerialCom(SerialCom &mySerialCom);
int serialComAvailable(bool serialComNumber);
int serialComRead(bool serialComNumber);
void serialComPrint(bool serialComNumber, String serialMessage);
void serialComPrintln(bool serialComNumber, String serialMessage);  
void serialComPrintlnDigits(bool serialComNumber, float serialFloatMessage, uint8_t nbDigits);

#endif