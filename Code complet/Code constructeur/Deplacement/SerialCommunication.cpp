#include "SerialCommunication.h"

void initSerialCommunication(bool serialComNumber, uint32_t baudRate) {
        if (serialComNumber == SERIALCOM0NUMBER)
        SERIALCOM0.begin(baudRate);
    else if (serialComNumber == SERIALCOM1NUMBER)
        SERIALCOM1.begin(baudRate);
}

void resetSerialCom(SerialCom &mySerialCom) {
    for (uint8_t i = 0; i < sizeof(mySerialCom.flags); i++)
        mySerialCom.flags[i] = false;
}

int serialComAvailable(bool serialComNumber) {
    int numberAvailableBytes = 0;
    if (serialComNumber == SERIALCOM0NUMBER)
        numberAvailableBytes = SERIALCOM0.available();
    else if (serialComNumber == SERIALCOM1NUMBER)
        numberAvailableBytes = SERIALCOM1.available();
    return numberAvailableBytes;
}

int serialComRead(bool serialComNumber) {
    int incomingByte = -1;      //S'il n'y a pas d'octet disponible, la fonction read() renvoie -1
    if (serialComNumber == SERIALCOM0NUMBER)
        incomingByte = SERIALCOM0.read();
    else if (serialComNumber == SERIALCOM1NUMBER)
        incomingByte = SERIALCOM1.read();
    return incomingByte;
}

void serialComPrint(bool serialComNumber, String serialMessage) {
    if (serialComNumber == SERIALCOM0NUMBER)
        SERIALCOM0.print(serialMessage);
    else if (serialComNumber == SERIALCOM1NUMBER)
        SERIALCOM1.print(serialMessage);
}

void serialComPrintln(bool serialComNumber, String serialMessage) {  //Le serialComNumber vaut soit 0 (SerialUSB) soit 1 (Serial)
    if (serialComNumber == SERIALCOM0NUMBER)
        SERIALCOM0.println(serialMessage);
    else if (serialComNumber == SERIALCOM1NUMBER)
        SERIALCOM1.println(serialMessage);
}

void serialComPrintlnDigits(bool serialComNumber, float serialFloatMessage, uint8_t nbDigits) {  //Le serialComNumber vaut soit 0 (SerialUSB) soit 1 (Serial)
    if (serialComNumber == SERIALCOM0NUMBER)
        SERIALCOM0.println(serialFloatMessage, nbDigits);
    else if (serialComNumber == SERIALCOM1NUMBER)
        SERIALCOM1.println(serialFloatMessage, nbDigits);
}