#include "Communication.h"
#include "Mouvement.h"

void setup() {
    // Fonctions définies dans "Mouvement.c"
    initMotors();
    initFlashStorage();
    initMPU9250();
    /*Mise aux valeurs de départ de toutes les variables, sauf de celles sauvegardées dans la mémoire Flash */
    resetValues();

    // Fonction définie dans "Communication.c"
    initCommunication();  //Initialise la communication avec Serial et SerialUSB
}

void loop() {
    processCommunication();
}