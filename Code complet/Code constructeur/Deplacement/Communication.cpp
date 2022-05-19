#include "Communication.h"

#include "Arduino.h"
#include "Mouvement.h"
#include "SerialCommunication.h"

/* Union de structure regroupant les flags propres a chaque SerialPort */
SerialCom serialCom[] = {{false, false, false, false, false, false, false, false, false, false, false}, {false, false, false, false, false, false, false, false, false, false, false}};

uint8_t motorNumber[] = {4, 4};
uint8_t dataTypeExport = DEFAULTPRESELECTEDDATA;  //Pour avoir une securite (superieur au nombre de types de donnees a exporter)
uint8_t axisData[] = {DEFAULTPRESELECTEDDATA, DEFAULTPRESELECTEDDATA};

void initCommunication() {
    initSerialCommunication(SERIALCOM0NUMBER, 115200);
    initSerialCommunication(SERIALCOM1NUMBER, 115200);
}

void processCommunication() {
    serialComReception(SERIALCOM0NUMBER);
    display(SERIALCOM0NUMBER);
    serialComReception(SERIALCOM1NUMBER);
    display(SERIALCOM1NUMBER);
}

void display(bool serialComNumber) {
    static uint16_t displayTime = 0;
    if ((millis() - displayTime) > DISPLAYINTERVAL) {
        displayTime = millis();
        if (serialCom[serialComNumber].flagDisplayCount)
            displayCount(serialComNumber);
        if (serialCom[serialComNumber].flagDisplayRpm)
            displayRpm(serialComNumber);
        if (serialCom[serialComNumber].flagDisplaySetpointPidSpeed)
            displaySetpointPidSpeed(serialComNumber);
        if (serialCom[serialComNumber].flagExportDataToExcel)
            exportDataToExcel(dataTypeExport, !serialComNumber);  //On exporte les donnees sur l'autre Serial que celui qui a donne l'ordre
        if (serialCom[serialComNumber].flagDisplayRobotSpeedsOrder)
            displayRobotSpeedOrder(serialComNumber);
        if (serialCom[serialComNumber].flagDisplayRobotSpeedsMeasured)
            displayRobotSpeedMeasured(serialComNumber);
        if (serialCom[serialComNumber].flagDisplayGyroSpeedZ)
            displayGyroSpeedZ(serialComNumber);
        if (serialCom[serialComNumber].flagDisplayGyroAngleZ)
            displayGyroAngleZ(serialComNumber);
        if (serialCom[serialComNumber].flagDisplayCurrentPosition)
            displayCurrentPosition(serialComNumber);
    }
}

void displayHelp(uint8_t serialComNumber) {
    serialComPrintln(serialComNumber, "Version 0.1");
    serialComPrintln(serialComNumber, "Ensemble des commandes interpretees par l'Arduino : ");
    serialComPrintln(serialComNumber, String(HELP) + " : Affiche ce menu d'aide");
    serialComPrintln(serialComNumber, String(LISTENING) + String(BEGINLISTENING) + " : Demarre la communication");
    serialComPrintln(serialComNumber, String(LISTENING) + String(FINISHLISTENING) + " : Termine la communication");
    serialComPrintln(serialComNumber, "nombre : Valeur d'un ordre qui sera specifie après avec un caractère");
    serialComPrintln(serialComNumber, String(MOTORNUMBER) + " : numero du moteur avec lequel on souhaite communique (de 0 a " + String(NBMOTORS - 1) + " pour piloter un moteur et " + String(NBMOTORS) + " pour tous les piloter)");
    serialComPrintln(serialComNumber, String(PWMORDER) + " : Envoi un pwm a un ou tous les moteurs (pwm jusqu'a -/+ " + String(PRESCALER) + " envoye avant la lettre)");
    serialComPrintln(serialComNumber, String(DISPLAYCOUNT) + " : Active/desactive l'affichage du count du moteur selectionne precedemment ou de tous les moteurs");
    serialComPrintln(serialComNumber, String(RESETVALUES) + " : Reset toutes les valeurs de tics, vitesses, pwm, position etc");
    serialComPrintln(serialComNumber, String(RESETSTOREDVALUES) + " : Reset toutes les valeurs stockees dans la memoire Flash (Coefficients des Pid, position de depart)");
    serialComPrintln(serialComNumber, String(STOPTIMER) + " : Desactive/active le Timer");
    serialComPrintln(serialComNumber, String(VERBOSE) + " : Active/desactive la verbosite du log");
    serialComPrintln(serialComNumber, String(RPMORDER) + " : Envoi une vitesse en tr/mn a un ou plusieurs moteurs (comprise entre -/+ " + String(MAXRPMORDER) + " tr/mn au niveau de la roue et envoyee avant la lettre)");
    serialComPrintln(serialComNumber, String(DISPLAYRPM) + " : Active/desactive l'affichage de la vitesse de rotation de la roue selectionnee precedemment ou de toutes les roues en tr/mn");
    serialComPrintln(serialComNumber, String(DISPLAYSETPOINTPIDSPEED) + " : Active/desactive l'affichage du setpointPidSpeed du moteur selectionne precedemment ou de tous les moteurs en tic/0.01s");
    serialComPrintln(serialComNumber, String(XAXIS) + "/" + String(YAXIS) + "/" + String(ZAXIS) + " " + String(EXPORTDATATOEXCEL) + " : Exporte les donnees selectionnees precedemment vers Excel");
    serialComPrintln(serialComNumber, "            1 : Export pour l'asservissement en vitesse de rotation des moteurs,");
    serialComPrintln(serialComNumber, "            2 : Export pour l'asservissement en position selon l'axe X/Y/Z selectionne avant la lettre" + String(EXPORTDATATOEXCEL));
    serialComPrintln(serialComNumber, String(PROPORTIONNAL) + "/" + String(INTEGRAL) + "/" + String(DERIVATIVE) + +" " + String(PIDSPEEDTUNING) + " : Reglage du coefficient Kp/ki/kd utilise dans l'asservissement en vitesse de rotation des moteurs");
    serialComPrintln(serialComNumber, String(DISPLAYSPEEDTUNINGS) + " : Affiche les coefficients Kp, Ki, Kd utilises dans l'asservissement en vitesse de rotation des moteurs");
    serialComPrintln(serialComNumber, String(XSPEEDROBOTORDER) + " : Envoi un ordre de vitesse lineaire au robot en mm/s selon son axe X (vitesse entre -/+ " + String(MAXSPEEDX) + " mm/s envoyee avant la lettre)");
    serialComPrintln(serialComNumber, String(YSPEEDROBOTORDER) + " : Envoi un ordre de vitesse lineaire au robot en mm/s selon son axe Y (vitesse entre -/+ " + String(MAXSPEEDY) + " mm/s envoyee avant la lettre)");
    serialComPrintln(serialComNumber, String(THETASPEEDROBOTORDER) + " : Envoi un ordre de vitesse de rotation au robot en °/s selon son axe Z (vitesse entre -/+ " + String(MAXSPEEDTHETA) + " °/s envoyee avant la lettre)");
    serialComPrintln(serialComNumber, String(DISPLAYROBOTSPEEDSORDER) + " : Active/desactive l'affichage des ordres de vitesses du robot selon ses axes X et Y en mm/s et selon Z en °/s");
    serialComPrintln(serialComNumber, String(DISPLAYROBOTSPEEDSMEASURED) + " : Active/desactive l'affichage des vitesses du robot mesurees selon ses axes X et Y en mm/s et selon Z en °/s");
    serialComPrintln(serialComNumber, String(DISPLAYGYROSPEEDZ) + " : Active/desactive l'affichage de la vitesse gyroscopique du robot en °/s selon son axe Z");
    serialComPrintln(serialComNumber, String(DISPLAYGYROANGLEZ) + " : Active/desactive l'affichage de l'angle gyroscopique du robot en ° selon son axe Z");
    serialComPrintln(serialComNumber, String(XAXIS) + "/" + String(YAXIS) + "/" + String(ZAXIS) + " " + String(CURRENTPOSITION) + " : Met a jour la position actuelle du robot selon l'axe X/Y/Z (valeurs en mm ou ° renseignees precedemment)");
    serialComPrintln(serialComNumber, String(CURRENTPOSITION) + String(DISPLAYPOSITION) + " : Active/desactive l'affichage de la position actuelle du robot selon les 3 axes");
    serialComPrintln(serialComNumber, String(XAXIS) + "/" + String(YAXIS) + "/" + String(ZAXIS) + " " + String(STARTINGPOSITION) + " : Choix de la position de depart du robot dans la carte selon l'axe X/Y/Z (valeurs en mm ou ° renseignees precedemment)");
    serialComPrintln(serialComNumber, String(STARTINGPOSITION) + String(DISPLAYPOSITION) + " : Affiche de la position de depart du robot dans la carte selon les 3 axes");
    serialComPrintln(serialComNumber, String(XAXIS) + "/" + String(YAXIS) + "/" + String(ZAXIS) + " " + String(AUTOMODE) + "/" + String(STRAFEMODE) + "/" + String(ORIENTATIONMODE) + " " + String(TARGETPOSITIONORDER) + " : Envoi un ordre de position cible au robot dans la carte selon l'axe X/Y/Z en mode de deplacement automatique/strafe/orientation (valeurs en mm ou ° renseignees precedemment)");
    serialComPrintln(serialComNumber, String(TARGETPOSITIONORDER) + String(DISPLAYPOSITION) + " : Affiche la position cible du robot dans la carte selon les 3 axes");
    serialComPrintln(serialComNumber, String(XAXIS) + "/" + String(YAXIS) + "/" + String(ZAXIS) + String(PROPORTIONNAL) + "/" + String(INTEGRAL) + "/" + String(DERIVATIVE) + " : Reglage du coefficient Kp/Ki/Kd utilise dans l'asservissement en position selon l'axe X/Y/Z");
    serialComPrintln(serialComNumber, String(DISPLAYPIDPOSITIONTUNINGS) + " : Affichage des coefficients Kp, Ki et Kd utilises dans l'asservissement en position selon les 3 axes");
}

void serialComReception(bool serialComNumber) {
    static bool flagListening[] = {false, false};
    static bool flagDisplayListening[] = {false, false};
    static String numberData[] = {"", ""};
    static char textData[] = {0, 0};
    static uint8_t tuningData[] = {DEFAULTPRESELECTEDDATA, DEFAULTPRESELECTEDDATA};
    static uint8_t modeData[] = {AUTOMODENUMBER, AUTOMODENUMBER};
    char SerialMessage;

    while (serialComAvailable(serialComNumber) > 0) {
        if (!flagListening[serialComNumber]) {
            SerialMessage = serialComRead(serialComNumber);
            switch (SerialMessage) {
                case VERBOSE:
                    serialCom[serialComNumber].flagVerbose = !serialCom[serialComNumber].flagVerbose;
                    if (serialCom[serialComNumber].flagVerbose) {
                        serialComPrintln(serialComNumber, "Verbosite activee.");
                        serialComPrintln(serialComNumber, "Tapez: #B pour debuter la communication");
                        if (getCodeRunningForTheFirstTime())
                            serialComPrintln(serialComNumber, "Un nouveau televersement a ete effectue, les coefficients par defaut des PID ont ete selectionnes.");
                    }
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case LISTENING:
                    textData[serialComNumber] = LISTENING;
                    numberData[serialComNumber] = "";
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case BEGINLISTENING:
                    if (textData[serialComNumber] == LISTENING) {
                        flagListening[serialComNumber] = true;
                        initTimer();  //On Initialise le timer au cas ou la communication n'a pas ete terminee precedemment et donc qu'il a ete stopper
                        flagDisplayListening[serialComNumber] = false;
                        if (serialCom[serialComNumber].flagVerbose) {
                            serialComPrintln(serialComNumber, "Communication etablie, vous pouvez maintenant tapez vos ordres");
                            serialComPrintln(serialComNumber, "Initialisation de l'IMU (0: echec, 1: reussite): " + String(getIMUStatus()));
                        }
                    } else
                        serialComPrintln(serialComNumber, "Commande non reconnue, tapez ? pour voir les commandes reconnues !");
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                default:
                    if (!flagDisplayListening[serialComNumber]) {
                        if (serialCom[serialComNumber].flagVerbose)
                            serialComPrintln(serialComNumber, "Tapez: #B pour debuter la communication");
                        flagDisplayListening[serialComNumber] = true;
                    }
                    break;
            }
        } else {
            SerialMessage = serialComRead(serialComNumber);
            switch (SerialMessage) {
                case HELP:
                    displayHelp(serialComNumber);
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case LISTENING:
                    textData[serialComNumber] = LISTENING;
                    numberData[serialComNumber] = "";
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case FINISHLISTENING:
                    if (textData[serialComNumber] == LISTENING) {
                        flagListening[serialComNumber] = false;
                        if (serialCom[serialComNumber].flagVerbose) {
                            serialComPrintln(serialComNumber, "Communication terminee, veuillez la redemarrer pour envoyer a nouveau des ordres.");
                            serialComPrintln(serialComNumber, "Toutes les valeurs on ete remises a 0.");
                        }
                        resetValues();
                        resetSerialCom(serialCom[serialComNumber]);
                    } else
                        serialComPrintln(serialComNumber, "Commande non reconnue, tapez ? pour voir les commandes reconnues !");
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case VERBOSE:
                    serialCom[serialComNumber].flagVerbose = !serialCom[serialComNumber].flagVerbose;
                    if (serialCom[serialComNumber].flagVerbose)
                        serialComPrintln(serialComNumber, "Verbosite activee.");
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case RESETVALUES:
                    resetValues();
                    resetValues();  //Deux fois pour être sûr d'être aux valeurs initiales au cas où il y a eu un gros freinage et donc un leger mouvement après le 1er reset
                    if (serialCom[serialComNumber].flagVerbose)
                        serialComPrintln(serialComNumber, "Toutes les valeurs ont ete remises a leur valeur initiale (sauf celles stockees dans la Flash)");
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case RESETSTOREDVALUES:
                    resetStoredValues();
                    if (serialCom[serialComNumber].flagVerbose)
                        serialComPrintln(serialComNumber, "Toutes les valeurs stockees dans la memoire Flash on ete remises a leur valeur par defaut");
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case STOPTIMER:
                    serialCom[serialComNumber].flagStopTimer = !serialCom[serialComNumber].flagStopTimer;
                    if (serialCom[serialComNumber].flagStopTimer) {
                        stopTimer();
                        if (serialCom[serialComNumber].flagVerbose)
                            serialComPrintln(serialComNumber, "Timer desactive");
                    } else {
                        initTimer();
                        if (serialCom[serialComNumber].flagVerbose)
                            serialComPrintln(serialComNumber, "Timer active");
                    }
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case MOTORNUMBER:
                    motorNumber[serialComNumber] = motorNumberChoice((int8_t)numberData[serialComNumber].toInt(), serialComNumber);
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case PWMORDER:  //Envoi d'un ordre de pwm a un ou plusieurs moteurs
                    motorPwmOrder((int16_t)numberData[serialComNumber].toInt(), serialComNumber);
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case RPMORDER:
                    motorRpmOrder((int16_t)numberData[serialComNumber].toInt(), serialComNumber);
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case PROPORTIONNAL:
                    tuningData[serialComNumber] = KPTUNINGNUMBER;
                    if (axisData[serialComNumber] != DEFAULTPRESELECTEDDATA) {  //On a change la valeur de l'axe donc on veut modifier les coefficients du Pid en position selon cet axe
                        pidPositionTuning(numberData[serialComNumber].toFloat(), axisData[serialComNumber], tuningData[serialComNumber], serialComNumber);
                        numberData[serialComNumber] = "";
                    }
                    textData[serialComNumber] = 0;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case INTEGRAL:
                    tuningData[serialComNumber] = KITUNINGNUMBER;
                    if (axisData[serialComNumber] != DEFAULTPRESELECTEDDATA) {
                        pidPositionTuning(numberData[serialComNumber].toFloat(), axisData[serialComNumber], tuningData[serialComNumber], serialComNumber);
                        numberData[serialComNumber] = "";
                    }
                    textData[serialComNumber] = 0;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case DERIVATIVE:
                    tuningData[serialComNumber] = KDTUNINGNUMBER;
                    if (axisData[serialComNumber] != DEFAULTPRESELECTEDDATA) {
                        pidPositionTuning(numberData[serialComNumber].toFloat(), axisData[serialComNumber], tuningData[serialComNumber], serialComNumber);
                        numberData[serialComNumber] = "";
                    }
                    textData[serialComNumber] = 0;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case PIDSPEEDTUNING:
                    pidSpeedTuning(numberData[serialComNumber].toFloat(), tuningData[serialComNumber], serialComNumber);
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case XSPEEDROBOTORDER:
                    robotSpeedOrder((int16_t)numberData[serialComNumber].toInt(), XAXISNUMBER, serialComNumber);
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case YSPEEDROBOTORDER:
                    robotSpeedOrder((int16_t)numberData[serialComNumber].toInt(), YAXISNUMBER, serialComNumber);
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case THETASPEEDROBOTORDER:
                    robotSpeedOrder((int16_t)numberData[serialComNumber].toInt(), ZAXISNUMBER, serialComNumber);
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case XAXIS:
                    axisData[serialComNumber] = XAXISNUMBER;
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case YAXIS:
                    axisData[serialComNumber] = YAXISNUMBER;
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case ZAXIS:
                    axisData[serialComNumber] = ZAXISNUMBER;
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case STARTINGPOSITION:
                    if (axisData[serialComNumber] != DEFAULTPRESELECTEDDATA) {  //Si c'est celui par defaut, c'est surement que l'on va demander d'afficher la position de depart donc on ne la change pas
                        setStartingPosition((int16_t)numberData[serialComNumber].toInt(), axisData[serialComNumber]);

                        if (serialCom[serialComNumber].flagVerbose)
                            serialComPrintln(serialComNumber, "Position de depart du robot mise a jour selon l'axe " + AXES[axisData[serialComNumber]] + " : " + numberData[serialComNumber] + AXESUNIT[axisData[serialComNumber]]);
                    }
                    textData[serialComNumber] = STARTINGPOSITION;
                    numberData[serialComNumber] = "";
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case CURRENTPOSITION:
                    if (axisData[serialComNumber] != DEFAULTPRESELECTEDDATA) {  //Si c'est celui par defaut, c'est surement que l'on va demander d'afficher la position actuelle donc on ne la change pas
                        setCurrentPosition((int16_t)numberData[serialComNumber].toInt(), axisData[serialComNumber]);

                        if (serialCom[serialComNumber].flagVerbose)
                            serialComPrintln(serialComNumber, "Position mise a jour du robot selon l'axe " + AXES[axisData[serialComNumber]] + " : " + numberData[serialComNumber] + AXESUNIT[axisData[serialComNumber]]);
                    }
                    textData[serialComNumber] = CURRENTPOSITION;
                    numberData[serialComNumber] = "";
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case AUTOMODE:
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    break;
                case STRAFEMODE:
                    modeData[serialComNumber] = STRAFEMODENUMBER;
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    break;
                case ORIENTATIONMODE:
                    modeData[serialComNumber] = ORIENTATIONMODENUMBER;
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    break;
                case TARGETPOSITIONORDER:
                    if (axisData[serialComNumber] != DEFAULTPRESELECTEDDATA) {  //Si c'est celui par defaut, c'est surement que l'on va demander d'afficher la position cible donc on ne la change pas
                        activePidSpeed(true);
                        activeRobotSpeedMode(false);
                        activePositionMode(true);
                        activeBeginTravel(true);
                        setTravelMode(modeData[serialComNumber]);
                        setTargetPosition((int16_t)numberData[serialComNumber].toInt(), axisData[serialComNumber]);

                        if (serialCom[serialComNumber].flagVerbose)
                            serialComPrintln(serialComNumber, "Position cible du robot selon l'axe " + AXES[axisData[serialComNumber]] + " : " + numberData[serialComNumber] + AXESUNIT[axisData[serialComNumber]] + " en mode " + MODES[modeData[serialComNumber]]);
                    }
                    textData[serialComNumber] = TARGETPOSITIONORDER;
                    numberData[serialComNumber] = "";
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    break;

                case DISPLAYCOUNT:
                    serialCom[serialComNumber].flagDisplayCount = !serialCom[serialComNumber].flagDisplayCount;
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case DISPLAYRPM:
                    serialCom[serialComNumber].flagDisplayRpm = !serialCom[serialComNumber].flagDisplayRpm;
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case DISPLAYSETPOINTPIDSPEED:
                    serialCom[serialComNumber].flagDisplaySetpointPidSpeed = !serialCom[serialComNumber].flagDisplaySetpointPidSpeed;
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case DISPLAYSPEEDTUNINGS:
                    if (serialCom[serialComNumber].flagVerbose) {
                        serialComPrint(serialComNumber, "Coefficients du Pid en vitesse de rotation des moteurs : Kp : " + String(getPidSpeedTunings(KPTUNINGNUMBER)) + ", Ki : " + String(getPidSpeedTunings(KITUNINGNUMBER)) + ", Kd : ");
                        serialComPrintlnDigits(serialComNumber, getPidSpeedTunings(KDTUNINGNUMBER), 8);
                    }
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case EXPORTDATATOEXCEL:
                    serialCom[serialComNumber].flagExportDataToExcel = !serialCom[serialComNumber].flagExportDataToExcel;
                    if (serialCom[serialComNumber].flagExportDataToExcel) {
                        dataTypeExport = (uint8_t)numberData[serialComNumber].toInt();
                        serialComPrintln(!serialComNumber, "CLEARSHEET");
                        serialComPrintln(!serialComNumber, "LABEL,Time,Target,Measure");  //Pour exportation de donnees vers Excel: Initialisation des titres des colonnes
                        if (serialCom[serialComNumber].flagVerbose)
                            serialComPrintln(serialComNumber, "Export des donnees selectionnees vers Excel.");
                    } else {
                        if (serialCom[serialComNumber].flagVerbose)
                            serialComPrintln(serialComNumber, "Fin de l'export des donnees vers Excel.");
                    }
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case DISPLAYROBOTSPEEDSORDER:
                    serialCom[serialComNumber].flagDisplayRobotSpeedsOrder = !serialCom[serialComNumber].flagDisplayRobotSpeedsOrder;
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case DISPLAYROBOTSPEEDSMEASURED:
                    serialCom[serialComNumber].flagDisplayRobotSpeedsMeasured = !serialCom[serialComNumber].flagDisplayRobotSpeedsMeasured;
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case DISPLAYGYROSPEEDZ:
                    serialCom[serialComNumber].flagDisplayGyroSpeedZ = !serialCom[serialComNumber].flagDisplayGyroSpeedZ;
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case DISPLAYGYROANGLEZ:
                    serialCom[serialComNumber].flagDisplayGyroAngleZ = !serialCom[serialComNumber].flagDisplayGyroAngleZ;
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case DISPLAYPOSITION:
                    displayPosition(textData[serialComNumber], serialComNumber);
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                case DISPLAYPIDPOSITIONTUNINGS:
                    if (serialCom[serialComNumber].flagVerbose) {
                        serialComPrint(serialComNumber, "Coefficients du Pid en position selon l'axe X : Kp : " + String(getPidPositionTuning(XAXISNUMBER, KPTUNINGNUMBER)) + ", Ki : " + String(getPidPositionTuning(XAXISNUMBER, KITUNINGNUMBER)) + ", Kd : ");
                        serialComPrintlnDigits(serialComNumber, getPidPositionTuning(XAXISNUMBER, KDTUNINGNUMBER), 8);
                        serialComPrint(serialComNumber, "Coefficients du Pid en position selon l'axe Y : Kp : " + String(getPidPositionTuning(YAXISNUMBER, KPTUNINGNUMBER)) + ", Ki : " + String(getPidPositionTuning(YAXISNUMBER, KITUNINGNUMBER)) + ", Kd : ");
                        serialComPrintlnDigits(serialComNumber, getPidPositionTuning(YAXISNUMBER, KDTUNINGNUMBER), 8);
                        serialComPrint(serialComNumber, "Coefficients du Pid en position selon l'axe Z : Kp : " + String(getPidPositionTuning(ZAXISNUMBER, KPTUNINGNUMBER)) + ", Ki : " + String(getPidPositionTuning(ZAXISNUMBER, KITUNINGNUMBER)) + ", Kd : ");
                        serialComPrintlnDigits(serialComNumber, getPidPositionTuning(ZAXISNUMBER, KDTUNINGNUMBER), 8);
                    }
                    numberData[serialComNumber] = "";
                    textData[serialComNumber] = 0;
                    tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                    modeData[serialComNumber] = AUTOMODENUMBER;
                    break;
                default:
                    if ((SerialMessage >= '0' && SerialMessage <= '9') || SerialMessage == '-' || SerialMessage == '.')  //On prend en compte les chiffres negatifs et les floats
                        numberData[serialComNumber] = numberData[serialComNumber] + SerialMessage;
                    else {
                        serialComPrintln(serialComNumber, "Tapez: ? pour voir les commandes reconnues.");
                        numberData[serialComNumber] = "";
                        textData[serialComNumber] = 0;
                        tuningData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                        axisData[serialComNumber] = DEFAULTPRESELECTEDDATA;
                        modeData[serialComNumber] = AUTOMODENUMBER;
                    }
                    break;
            }
        }
    }
}

int8_t motorNumberChoice(int8_t motorNumber, bool serialComNumber) {
    String motorNumberMessage = "";
    if (motorNumber <= NBMOTORS) {
        if (motorNumber == NBMOTORS)
            motorNumberMessage = "Vous pilotez tous les moteurs en même temps.";
        else
            motorNumberMessage = "Vous pilotez le moteur " + String(motorNumber);
    } else {
        motorNumberMessage = "Le numero de moteur specifie auparavant n'est pas compris entre 0 et " + String(NBMOTORS);
        motorNumber = 4;
    }
    if (serialCom[serialComNumber].flagVerbose)
        serialComPrintln(serialComNumber, motorNumberMessage);
    return motorNumber;
}

void motorPwmOrder(int16_t pwmOrder, bool serialComNumber) {
    String motorPwmMessage = "";
    activePidSpeed(false);
    activeRobotSpeedMode(false);
    activePositionMode(false);
    activeBeginTravel(false);
    if (pwmOrder >= -PRESCALER && pwmOrder <= PRESCALER) {
        if (motorNumber[serialComNumber] < NBMOTORS) {
            setMotorPwm(motorNumber[serialComNumber], pwmOrder);
            motorPwmMessage = "Pwm de " + String(pwmOrder) + " envoye au moteur " + String(motorNumber[serialComNumber]);
        } else if (motorNumber[serialComNumber] == NBMOTORS) {
            setSameMotorPwm(pwmOrder);
            motorPwmMessage = "Pwm de " + String(pwmOrder) + " envoye a tous les moteurs.";
        }
    } else
        motorPwmMessage = "Le pwm envoye n'est pas compris entre -/+" + String(PRESCALER);
    if (serialCom[serialComNumber].flagVerbose)
        serialComPrintln(serialComNumber, motorPwmMessage);
}

void motorRpmOrder(int16_t rpmOrder, bool serialComNumber) {
    String motorRpmMessage = "";
    activePidSpeed(true);
    activeRobotSpeedMode(false);
    activePositionMode(false);
    activeBeginTravel(false);
    if (rpmOrder >= -MAXRPMORDER && rpmOrder <= MAXRPMORDER) {
        if (motorNumber[serialComNumber] < NBMOTORS) {
            setMotorRpm(motorNumber[serialComNumber], rpmOrder);
            motorRpmMessage = "Ordre de " + String(rpmOrder) + " tr/mn envoye au moteur " + String(motorNumber[serialComNumber]);
        } else if (motorNumber[serialComNumber] == NBMOTORS) {
            setSameMotorRpm(rpmOrder);
            motorRpmMessage = "Ordre de " + String(rpmOrder) + " tr/mn envoye a tous les moteurs.";
        }
    } else
        motorRpmMessage = "La vitesse de rotation envoyee n'est pas comprise entre -/+" + String(MAXRPMORDER) + " tr/mn !";
    if (serialCom[serialComNumber].flagVerbose)
        serialComPrintln(serialComNumber, motorRpmMessage);
}

void pidSpeedTuning(float tuningValue, uint8_t tuningTypeNumber, bool serialComNumber) {
    String speedTuningsMessage = "";
    if (tuningTypeNumber != DEFAULTPRESELECTEDDATA) {  //Donc si tuningTypeNumber correspond a un numero connu de TuningType
        setPidSpeedTuning(tuningValue, tuningTypeNumber);
        if (tuningTypeNumber == KDTUNINGNUMBER)
            speedTuningsMessage = "Reglage du " + TUNINGTYPES[tuningTypeNumber] + " vitesse de rotation des moteurs a " + String(tuningValue, 8);
        else
            speedTuningsMessage = "Reglage du " + TUNINGTYPES[tuningTypeNumber] + " vitesse de rotation des moteurs a " + String(tuningValue);
    } else
        speedTuningsMessage = "Commande non reconnue, tapez ? pour voir les commandes reconnues !";
    if (serialCom[serialComNumber].flagVerbose)
        serialComPrintln(serialComNumber, speedTuningsMessage);
}

void robotSpeedOrder(int16_t speedOrder, uint8_t axisNumber, bool serialComNumber) {
    String robotSpeedMessage = "";
    activePidSpeed(true);
    activeRobotSpeedMode(true);
    activePositionMode(false);
    activeBeginTravel(false);
    if (speedOrder >= -getMaxRobotSpeeds(axisNumber) && speedOrder <= getMaxRobotSpeeds(axisNumber)) {
        if (axisNumber != DEFAULTPRESELECTEDDATA) {  //Donc si axisNumber correspond a un numero d'axe
            robotSpeedMessage = "Ordre de " + String(speedOrder) + AXESUNIT[axisNumber] + "/s selon l'axe " + AXES[axisNumber];
            setRobotSpeed(speedOrder, axisNumber);
        } else
            robotSpeedMessage = "Commande non reconnue, tapez ? pour voir les commandes reconnues !";
    } else
        robotSpeedMessage = "La vitesse envoyee n'est pas comprise entre -/+" + String(getMaxRobotSpeeds(axisNumber)) + AXESUNIT[axisNumber] + "/s";
    if (serialCom[serialComNumber].flagVerbose)
        serialComPrintln(serialComNumber, robotSpeedMessage);
}

void pidPositionTuning(float tuningValue, uint8_t axisNumber, uint8_t tuningTypeNumber, bool serialComNumber) {
    String positionTuningsMessage = "";
    if (tuningTypeNumber != DEFAULTPRESELECTEDDATA) {
        setPidPositionTuning(tuningValue, axisNumber, tuningTypeNumber);
        if (tuningTypeNumber == KDTUNINGNUMBER)
            positionTuningsMessage = "Reglage du " + TUNINGTYPES[tuningTypeNumber] + " position selon l'axe " + AXES[axisNumber] + " a " + String(tuningValue, 8);
        else
            positionTuningsMessage = "Reglage du " + TUNINGTYPES[tuningTypeNumber] + " position selon l'axe " + AXES[axisNumber] + " a " + String(tuningValue);
    } else
        positionTuningsMessage = "Commande non reconnue, tapez ? pour voir les commandes reconnues !";
    if (serialCom[serialComNumber].flagVerbose)
        serialComPrintln(serialComNumber, positionTuningsMessage);
}

void displayCount(bool serialComNumber) {
    String countMessage = "";
    if (motorNumber[serialComNumber] < NBMOTORS) {
        if (serialCom[serialComNumber].flagVerbose)
            countMessage = String(getCount(motorNumber[serialComNumber])) + " tics comptes pour le moteur " + String(motorNumber[serialComNumber]);
        else
            countMessage = String(DISPLAYCOUNT) + String(motorNumber[serialComNumber]) + SPLITTERORDER + String(getCount(motorNumber[serialComNumber]));
    } else if (motorNumber[serialComNumber] == NBMOTORS) {
        if (serialCom[serialComNumber].flagVerbose)
            countMessage = "Moteur 0 : " + String(getCount(0)) + ", Moteur 1 : " + String(getCount(1)) + ", Moteur 2 : " + String(getCount(2)) + ", Moteur 3 : " + String(getCount(3)) + " tics comptes";
        else
            countMessage = String(DISPLAYCOUNT) + String(motorNumber[serialComNumber]) + SPLITTERORDER + String(getCount(0)) + SPLITTERVALUE + String(getCount(1)) + SPLITTERVALUE + String(getCount(2)) + SPLITTERVALUE + String(getCount(3));
    }
    serialComPrintln(serialComNumber, countMessage);
}

void displayRpm(bool serialComNumber) {
    String rpmMessage = "";
    if (motorNumber[serialComNumber] < NBMOTORS) {
        if (serialCom[serialComNumber].flagVerbose)
            rpmMessage = "Vitesse de " + String(getRpm(motorNumber[serialComNumber])) + " tr/mn mesuree pour la roue " + String(motorNumber[serialComNumber]);
        else
            rpmMessage = String(DISPLAYRPM) + String(motorNumber[serialComNumber]) + SPLITTERORDER + String(getRpm(motorNumber[serialComNumber]));
    } else if (motorNumber[serialComNumber] == NBMOTORS) {
        if (serialCom[serialComNumber].flagVerbose)
            rpmMessage = "Roue 0 : " + String(getRpm(0)) + ", Roue 1 : " + String(getRpm(1)) + ", Roue 2 : " + String(getRpm(2)) + ", Roue 3 : " + String(getRpm(3)) + " tr/mn mesures";
        else
            rpmMessage = String(DISPLAYRPM) + String(motorNumber[serialComNumber]) + SPLITTERORDER + String(getRpm(0)) + SPLITTERVALUE + String(getRpm(1)) + SPLITTERVALUE + String(getRpm(2)) + SPLITTERVALUE + String(getRpm(3));
    }
    serialComPrintln(serialComNumber, rpmMessage);
}

void displaySetpointPidSpeed(bool serialComNumber) {
    String setpointMessage = "";
    if (motorNumber[serialComNumber] < NBMOTORS) {
        if (serialCom[serialComNumber].flagVerbose)
            setpointMessage = "Le SetpointSpeed du moteur "  + String(motorNumber[serialComNumber]) + " est de " + String(getSetpointPidSpeed(motorNumber[serialComNumber])) + " tic/0.01s";
        else
            setpointMessage = String(DISPLAYSETPOINTPIDSPEED) + String(motorNumber[serialComNumber]) + SPLITTERORDER + String(getSetpointPidSpeed(motorNumber[serialComNumber]));
    } else if (motorNumber[serialComNumber] == NBMOTORS) {
        if (serialCom[serialComNumber].flagVerbose)
            setpointMessage = "SetpointSpeed des moteurs M0 : " + String(getSetpointPidSpeed(0)) + ", M1 : " + String(getSetpointPidSpeed(1)) + ", M2 : " + String(getSetpointPidSpeed(2)) + ", M3 : " + String(getSetpointPidSpeed(3)) + " tics/0.01s";
        else
            setpointMessage = String(DISPLAYSETPOINTPIDSPEED) + String(motorNumber[serialComNumber]) + SPLITTERORDER + String(getSetpointPidSpeed(0)) + SPLITTERVALUE + String(getSetpointPidSpeed(1)) + SPLITTERVALUE + String(getSetpointPidSpeed(2)) + SPLITTERVALUE + String(getSetpointPidSpeed(3));
    }
    serialComPrintln(serialComNumber, setpointMessage);
}

void exportDataToExcel(uint8_t dataTypeExport, bool serialComNumberExport) {
    serialComPrint(serialComNumberExport, "DATA," + String(millis()) + ",");
    if (dataTypeExport == EXPORTSPEEDDATA) {
        if (motorNumber[!serialComNumberExport] < NBMOTORS)
            serialComPrintln(serialComNumberExport, String(getSetpointPidSpeed(motorNumber[!serialComNumberExport])) + "," + String(getMotorRotationSpeed(motorNumber[!serialComNumberExport])));
        else
            serialComPrintln(!serialComNumberExport, "Il n'est pas possible d'afficher les donnees demandees pour ce/ces moteur(s) ...");  //On ecrit sur le SerialPort qui a donne l'ordre d'export pour avoir un retour
    } else if (dataTypeExport == EXPORTPOSITIONDATA) {
        if (axisData[serialComNumberExport] != DEFAULTPRESELECTEDDATA)
            serialComPrintln(serialComNumberExport, String(getSetpointPidPosition(axisData[serialComNumberExport])) + "," + String(getCurrentPosition(axisData[serialComNumberExport])));
        else
            serialComPrintln(!serialComNumberExport, "Il n'est pas possible d'afficher les donnees demandees pour cet axe ...");  //On ecrit sur le SerialPort qui a donne l'ordre d'export pour avoir un retour
    } else
        serialComPrintln(!serialComNumberExport, "La valeur envoyee ne correspond a aucun type de donnee, tapez ? pour connaître les types donnees reconnus.");
}

void displayRobotSpeedOrder(bool serialComNumber) {
    String robotSpeedMessage = "";
    if (serialCom[serialComNumber].flagVerbose)
        robotSpeedMessage = "Ordre de vitesses du robot selon ses axes : X : " + String(getRobotSpeedsOrder(XAXISNUMBER)) + " mm/s , Y : " + String(getRobotSpeedsOrder(YAXISNUMBER)) + " mm/s , Z : " + String(getRobotSpeedsOrder(ZAXISNUMBER)) + " °/s";
    else
        robotSpeedMessage = String(DISPLAYROBOTSPEEDSORDER) + SPLITTERORDER + String(getRobotSpeedsOrder(XAXISNUMBER)) + SPLITTERVALUE + String(getRobotSpeedsOrder(YAXISNUMBER)) + SPLITTERVALUE + String(getRobotSpeedsOrder(ZAXISNUMBER));
    serialComPrintln(serialComNumber, robotSpeedMessage);
}

void displayRobotSpeedMeasured(bool serialComNumber) {
    String robotSpeedMessage = "";
    if (serialCom[serialComNumber].flagVerbose)
        robotSpeedMessage = "Vitesses du robot mesurees selon ses axes : X : " + String(getRobotSpeedsMeasured(XAXISNUMBER)) + " mm/s , Y : " + String(getRobotSpeedsMeasured(YAXISNUMBER)) + " mm/s , Z : " + String(getRobotSpeedsMeasured(ZAXISNUMBER)) + " °/s";
    else
        robotSpeedMessage = String(DISPLAYROBOTSPEEDSMEASURED) + SPLITTERORDER + String(getRobotSpeedsMeasured(XAXISNUMBER)) + SPLITTERVALUE + String(getRobotSpeedsMeasured(YAXISNUMBER)) + SPLITTERVALUE + String(getRobotSpeedsMeasured(ZAXISNUMBER));
    serialComPrintln(serialComNumber, robotSpeedMessage);
}

void displayPosition(char positionType, bool serialComNumber) {
    if (positionType == STARTINGPOSITION)  //Il faut avoir renseigne la lettre s avant pour activer l'affichage de la position de depart du robot
        if (serialCom[serialComNumber].flagVerbose)
            serialComPrintln(serialComNumber, "Position de depart du robot selon l'axe X : " + String(getStartingPosition(XAXISNUMBER)) + " mm, Y : " + String(getStartingPosition(YAXISNUMBER)) + " mm, Z : " + String(getStartingPosition(ZAXISNUMBER)) + " °");
        else
            serialComPrintln(serialComNumber, String(STARTINGPOSITION) + SPLITTERORDER + String(getStartingPosition(XAXISNUMBER)) + SPLITTERVALUE + String(getStartingPosition(YAXISNUMBER)) + SPLITTERVALUE + String(getStartingPosition(ZAXISNUMBER)));
    else if (positionType == CURRENTPOSITION)  //Il faut avoir renseigne la lettre c avant pour activer l'affichage de la position actuelle du robot
        serialCom[serialComNumber].flagDisplayCurrentPosition = !serialCom[serialComNumber].flagDisplayCurrentPosition;
    else if (positionType == TARGETPOSITIONORDER)  //Il faut avoir renseigne la lettre t avant pour activer l'affichage de la position cible du robot
        if (serialCom[serialComNumber].flagVerbose)
            serialComPrintln(serialComNumber, "Position cible du robot selon l'axe X : " + String(getTargetPosition(XAXISNUMBER)) + " mm, Y : " + String(getTargetPosition(YAXISNUMBER)) + " mm, Z : " + String(getTargetPosition(ZAXISNUMBER)) + " °");
        else
            serialComPrintln(serialComNumber, String(TARGETPOSITIONORDER) + SPLITTERORDER + String(getTargetPosition(XAXISNUMBER)) + SPLITTERVALUE + String(getTargetPosition(YAXISNUMBER)) + SPLITTERVALUE + String(getTargetPosition(ZAXISNUMBER)));
    else
        serialComPrintln(serialComNumber, "Tapez: ? pour voir les commandes reconnues.");
}

void displayCurrentPosition(bool serialComNumber) {
    String currentPositionMessage = "";
    if (serialCom[serialComNumber].flagVerbose)
        currentPositionMessage = "Position actuelle du robot : x : " + String(getCurrentPosition(XAXISNUMBER)) + " mm, y : " + String(getCurrentPosition(YAXISNUMBER)) + " mm, theta : " + String(getCurrentPosition(ZAXISNUMBER)) + " °";
    else
        currentPositionMessage = String(CURRENTPOSITION) + SPLITTERORDER + String(getCurrentPosition(XAXISNUMBER)) + SPLITTERVALUE + String(getCurrentPosition(YAXISNUMBER)) + SPLITTERVALUE + String(getCurrentPosition(ZAXISNUMBER));
    serialComPrintln(serialComNumber, currentPositionMessage);
}

void displayGyroSpeedZ(bool serialComNumber) {
    String GyroZMessage = "";
    if (serialCom[serialComNumber].flagVerbose)
        GyroZMessage = "Vitesse gyroscopique du robot selon son axe Z : " + String(getGyroSpeedZ()) + " °/s";
    else
        GyroZMessage = String(DISPLAYGYROSPEEDZ) + SPLITTERORDER + String(getGyroSpeedZ());
    serialComPrintln(serialComNumber, GyroZMessage);
}

void displayGyroAngleZ(bool serialComNumber) {
    String GyroZMessage = "";
    if (serialCom[serialComNumber].flagVerbose)
        GyroZMessage = "Angle gyroscopique du robot selon son axe Z : " + String(getGyroAngleZ()) + " °";
    else
        GyroZMessage = String(DISPLAYGYROANGLEZ) + SPLITTERORDER + String(getGyroAngleZ());
    serialComPrintln(serialComNumber, GyroZMessage);
}
