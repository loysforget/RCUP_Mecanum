#include "Mouvement.h"

#include <DueFlashStorage.h>
#include <DueTimer.h>
#include <MPU9250.h>

#include "Arduino.h"
#include "PID.h"

DueFlashStorage dueFlashStorage;
MPU9250 IMU(SPI, PINCHIPSELECT);
float gyroSpeedZ = 0;
float gyroAngleZ = 0;
bool IMUStatus = false;

volatile int16_t count[] = {0, 0, 0, 0};        //Comptage des tics d'encodeur qui est incrémenté à chaque interruption
volatile float rotationSpeed[] = {0, 0, 0, 0};  //Vitesse de rotation extraite de la variable count
volatile bool laststateA[] = {0, 0, 0, 0};      //Etat précédent des sorties A des enncodeurs
//volatile bool laststateB[] = {0, 0, 0, 0};      //Etat précédent des sorties B des enncodeurs (si utilisation des interruptions sur les 2 encodeurs)

bool flagPidSpeedIsActive = true;
bool flagRobotSpeedMode = false;
bool flagCodeRunningForTheFirstTime = 0;
bool flagActiveGyroAngleZ = true;
bool flagBeginTravel = false;
bool flagPositionMode = false;

int16_t pwm[] = {0, 0, 0, 0};

/* Asservissement PID en vitesse */
const PidTunings defaultTuningsPidSpeed = {DEFAULTKPSPEED, DEFAULTKISPEED, DEFAULTKDSPEED};
PidTunings pidSpeedTunings;
volatile float setpointSpeed[NBMOTORS] = {0, 0, 0, 0};
volatile float setpointPidSpeed[NBMOTORS] = {0, 0, 0, 0};
volatile float outputPidSpeed[NBMOTORS] = {0, 0, 0, 0};
PID pidSpeed[NBMOTORS] = {
    PID(&rotationSpeed[0], &outputPidSpeed[0], &setpointPidSpeed[0], pidSpeedTunings.kp, pidSpeedTunings.ki, pidSpeedTunings.kd, P_ON_E, DIRECT, SPEEDMODE),
    PID(&rotationSpeed[1], &outputPidSpeed[1], &setpointPidSpeed[1], pidSpeedTunings.kp, pidSpeedTunings.ki, pidSpeedTunings.kd, P_ON_E, DIRECT, SPEEDMODE),
    PID(&rotationSpeed[2], &outputPidSpeed[2], &setpointPidSpeed[2], pidSpeedTunings.kp, pidSpeedTunings.ki, pidSpeedTunings.kd, P_ON_E, DIRECT, SPEEDMODE),
    PID(&rotationSpeed[3], &outputPidSpeed[3], &setpointPidSpeed[3], pidSpeedTunings.kp, pidSpeedTunings.ki, pidSpeedTunings.kd, P_ON_E, DIRECT, SPEEDMODE)};

/* Structures des vitesses du robot */
const Coordinates maxRobotSpeeds = {MAXSPEEDX, MAXSPEEDY, MAXSPEEDTHETA};
volatile Coordinates robotSpeedsOrder = {0, 0, 0};
volatile Coordinates robotSpeedsMeasured;

/* Structures position du robot */
const Coordinates defaultStartingPosition = {DEFAULTXSTARTINGPOSITION, DEFAULTYSTARTINGPOSITION, DEFAULTTHETASTARTINGANGLE};
Coordinates startingPosition;
volatile Coordinates_float currentPosition_float;
Coordinates targetPosition;
volatile float currentTheta = startingPosition.z;  //Variable tampon qui fluctue et qui permet d'avoir toujours la valeur adaptée de currentPosition_float.z

uint8_t travelMode = AUTOMODENUMBER;

/* Asservissement PID en position */
const PidTunings defaultPidPositionXTunings = {DEFAULTKPPOSITIONX, DEFAULTKIPOSITIONX, DEFAULTKDPOSITIONX};
const PidTunings defaultPidPositionYTunings = {DEFAULTKPPOSITIONY, DEFAULTKIPOSITIONY, DEFAULTKDPOSITIONX};
const PidTunings defaultPidPositionZTunings = {DEFAULTKPANGLETHETA, DEFAULTKIANGLETHETA, DEFAULTKDANGLETHETA};
PidTunings pidPositionXTunings;
PidTunings pidPositionYTunings;
PidTunings pidPositionZTunings;

const PidAxesTunings defaultPidPositionAxesTunings = {defaultPidPositionXTunings, defaultPidPositionYTunings, defaultPidPositionZTunings};
PidAxesTunings pidPositionAxesTunings = {pidPositionXTunings, pidPositionYTunings, pidPositionZTunings};

volatile Coordinates_float setpointPidPosition = {currentPosition_float.x, currentPosition_float.y, currentPosition_float.z};
volatile Coordinates_float outputPidPosition = {0, 0, 0};

PID pidPosition[NBAXIS] = {
    PID(&currentPosition_float.x, &outputPidPosition.x, &setpointPidPosition.x, pidPositionXTunings.kp, pidPositionXTunings.ki, pidPositionXTunings.kd, P_ON_E, DIRECT, POSITIONMODE),
    PID(&currentPosition_float.y, &outputPidPosition.y, &setpointPidPosition.y, pidPositionYTunings.kp, pidPositionYTunings.ki, pidPositionYTunings.kd, P_ON_E, DIRECT, POSITIONMODE),
    PID(&currentPosition_float.z, &outputPidPosition.z, &setpointPidPosition.z, pidPositionZTunings.kp, pidPositionZTunings.ki, pidPositionZTunings.kd, P_ON_E, DIRECT, POSITIONMODE)};

/* Tableau de pointeurs sur des fonctions pour faciliter l'usage des interruptions */
void (*counterEncoderA[])(void) = {counterA0, counterA1, counterA2, counterA3};
//void (*counterEncoderB[])(void) = {counterB0, counterB1, counterB2, counterB3};  //(si utilisation des interruptions sur les 2 encodeurs)

/* Initialisation de tous les moteurs */
void initMotors() {
    for (uint8_t i = 0; i < NBMOTORS; i++) {
        /* Initialisation des entrées et sorties */
        pinMode(PINENCODERA[i], INPUT_PULLUP);
        pinMode(PINENCODERB[i], INPUT_PULLUP);
        pinMode(PINSPEED[i], OUTPUT);
        pinMode(PINDIRECTION[i], OUTPUT);


        /* Initialisation des interruptions */
        attachInterrupt(digitalPinToInterrupt(PINENCODERA[i]), counterEncoderA[i], RISING);  //S'active lorsqu'il y a un front montant sur la pin de l'encodeur A
        //attachInterrupt(digitalPinToInterrupt(PINENCODERA[i]), counterEncoderA[i], CHANGE);  //S'active lorsqu'il y a un changement d'état sur la pin de l'encodeur A
        //attachInterrupt(digitalPinToInterrupt(PINENCODERB[i]), counterEncoderB[i], CHANGE); //(si utilisation des interruptions sur les 2 encodeurs) S'active lorsqu'il y a un changement d'état sur la pin de l'encodeur B
    }

    /* Initialisation de la fréquence du PWM */
    initTimerCounter6();  //Pour les moteurs 0 et 1
    initTimerCounter8();  //Pour les moteurs 2 et 3

    /* Mise des moteurs à vitesse nulle */
    setSameMotorPwm(0);

    initTimer();
    initPidSpeed();
    initPidPosition();
}

/* Calcul de la vitesse du moteur grâce aux interruptions sur l'encodeur A */
/* Si on possède une carte microprogrammée assez puissante ou des encodeurs de moins bonne qualité, on peut en plus utiliser les interruptions sur l'encodeur B */
/*
void counterA(uint8_t motorNumber)
{
  bool stateA = digitalRead(PINENCODERA[motorNumber]);
  (stateA ^ laststateB[motorNumber]) ? count[motorNumber]++ : count[motorNumber]--;
  laststateA[motorNumber] = stateA;
}

void counterB(uint8_t motorNumber)
{
  bool stateB = digitalRead(PINENCODERB[motorNumber]);
  (stateB ^ laststateA[motorNumber]) ? count[motorNumber]-- : count[motorNumber]++;
  laststateB[motorNumber] = stateB;
}


void counterA0() {
    counterA(0);
}
void counterA1() {
    counterA(1);
}
void counterA2() {
    counterA(2);
}
void counterA3() {
    counterA(3);
}

//(si utilisation des interruptions sur les 2 encodeurs)
void counterB0(){
  counterB(0);
}
void counterB1(){
  counterB(1);
}
void counterB2(){
  counterB(2);
}
void counterB3(){
  counterB(3);
}
*/

void counterA0() {
    byte state = REG_PIOC_PDSR;
    ((state >> SHIFT4) & 1) ^ ((state >> SHIFT5) & 1) ? count[0]++ : count[0]--;
}
void counterA1() {
    byte state = REG_PIOC_PDSR;
    ((state >> SHIFT6) & 1) ^ ((state >> SHIFT7) & 1) ? count[1]++ : count[1]--;
}
void counterA2() {
    byte state = REG_PIOC_PDSR;
    ((state >> SHIFT2) & 1) ^ ((state >> SHIFT3) & 1) ? count[2]++ : count[2]--;
}
void counterA3() {
    uint16_t state = REG_PIOC_PDSR;
    ((state >> SHIFT8) & 1) ^ ((state >> SHIFT9) & 1) ? count[3]++ : count[3]--;
}

/* Ecriture dans les registres du TimerCounter 6 pour mettre la fréquence du pwm à 20kHz */
void initTimerCounter6() {
    REG_PMC_PCER1 |= PMC_PCER1_PID33;              // Enable peripheral TC6 (TC2 Channel 0)
    REG_PIOC_ABSR |= PIO_ABSR_P26 | PIO_ABSR_P25;  // Switch the multiplexer to peripheral B for TIOA6 and TIOB6
    REG_PIOC_PDR |= PIO_PDR_P26 | PIO_PDR_P25;     // Disable the GPIO on the corresponding pins

    REG_TC2_CMR0 = TC_CMR_BCPC_SET |            // Set TIOB on counter match with RC0
                   TC_CMR_ACPC_SET |            // Set TIOA on counter match with RC0
                   TC_CMR_BCPB_CLEAR |          // Clear TIOB on counter match with RB0
                   TC_CMR_ACPA_CLEAR |          // Clear TIOA on counter match with RA0
                   TC_CMR_WAVE |                // Enable wave mode
                   TC_CMR_WAVSEL_UP_RC |        // Count up with automatic trigger on RC compare
                   TC_CMR_EEVT_XC0 |            // Set event selection to XC0 to make TIOB an output
                   TC_CMR_TCCLKS_TIMER_CLOCK1;  // Set the timer clock to TCLK1 (MCK/2 = 84MHz/2 = 48MHz)

    REG_TC2_RA0 = 0;          // Load the RA0 register
    REG_TC2_RB0 = 0;          // Load the RB0 register
    REG_TC2_RC0 = PRESCALER;  // Load the RC0 register (diviseur de la fréquence de abse (42MHz) => nouvelle fréquence à 20kHz)

    REG_TC2_CCR0 = TC_CCR_SWTRG | TC_CCR_CLKEN;  // Enable the timer TC6
}

/* Ecriture dans les registres du TimerCounter 8 pour mettre la fréquence du pwm à 20kHz */
void initTimerCounter8() {
    REG_PMC_PCER1 |= PMC_PCER1_PID35;            // Enable peripheral TC8 (TC2 Channel 2)
    REG_PIOD_ABSR |= PIO_ABSR_P8 | PIO_ABSR_P7;  // Switch the multiplexer to peripheral B for TIOA8 and TIOB8
    REG_PIOD_PDR |= PIO_PDR_P8 | PIO_PDR_P7;     // Disable the GPIO on the corresponding pins

    REG_TC2_CMR2 = TC_CMR_BCPC_SET |            // Set TIOB on counter match with RC2
                   TC_CMR_ACPC_SET |            // Set TIOA on counter match with RC2
                   TC_CMR_BCPB_CLEAR |          // Clear TIOB on counter match with RB2
                   TC_CMR_ACPA_CLEAR |          // Clear TIOA on counter match with RA2
                   TC_CMR_WAVE |                // Enable wave mode
                   TC_CMR_WAVSEL_UP_RC |        // Count up with automatic trigger on RC compare
                   TC_CMR_EEVT_XC0 |            // Set event selection to XC0 to make TIOB an output
                   TC_CMR_TCCLKS_TIMER_CLOCK1;  // Set the timer clock to TCLK1 (MCK/2 = 84MHz/2 = 48MHz)

    REG_TC2_RA2 = 0;          // Load the RA2 register
    REG_TC2_RB2 = 0;          // Load the RB2 register
    REG_TC2_RC2 = PRESCALER;  // Load the RC2 register

    REG_TC2_CCR2 = TC_CCR_SWTRG | TC_CCR_CLKEN;  // Enable the timer TC8
}

void initTimer() {
    Timer1.attachInterrupt(timerIsr);
    Timer1.start(TIMERPERIODE_US);  // timer de période 10 000 microsecondes (0.01 sec ou 100 Hz)
}

void initPidSpeed() {
    /* Activation du PID Vitesse*/
    for (uint8_t i = 0; i < NBMOTORS; i++) {
        pidSpeed[i].SetMode(AUTOMATIC);
        pidSpeed[i].SetOutputLimits(-PRESCALER, PRESCALER);  //Correspondant aux limites en PWM
    }
}

void initPidPosition() {
    for (uint8_t i = 0; i < NBAXIS; i++) {
        pidPosition[i].SetMode(AUTOMATIC);
        pidPosition[i].SetOutputLimits(-maxRobotSpeeds.axes[i], maxRobotSpeeds.axes[i]);  //Correspondant aux limites de vitesse selon les axes
    }
}

/* La mémoire Flash est écrasée à chaque nouveau téléversement. Si il y a eu un nouveau téléversement, écrit la configuration par défaut, sinon, récupère les anciennes valeurs  */
void initFlashStorage() {
    flagCodeRunningForTheFirstTime = dueFlashStorage.read(0);  //Lors de la première fois, la case mémoire est différente de zéro

    if (flagCodeRunningForTheFirstTime) {
        /* On met les valeurs par défaut dans les structures */
        for (uint8_t i = 0; i < NBAXIS; i++) {
            pidSpeedTunings.tunings[i] = defaultTuningsPidSpeed.tunings[i];
            startingPosition.axes[i] = defaultStartingPosition.axes[i];
            for (uint8_t j = 0; j < NBAXIS; j++)
                pidPositionAxesTunings.axes[i].tunings[j] = defaultPidPositionAxesTunings.axes[i].tunings[j];
        }

        updatePidSpeedTunings();
        updatePidPositionTunings();

        dueFlashStorage.write(STARTINGADDRESSPIDSPEED, pidSpeedTunings.bytes, sizeof(pidSpeedTunings));            //Ecrit les coefficients par défaut du Pid en vitesse de rotation sur la Flash
        dueFlashStorage.write(STARTINGADDRESSSTARTINGPOSITION, startingPosition.bytes, sizeof(startingPosition));  //Ecrit la position de départ du robot par défaut sur la Flash
        for (uint8_t i = 0; i < NBAXIS; i++)
            dueFlashStorage.write(STARTINGADDRESSPIDPOSITION[i], pidPositionAxesTunings.axes[i].bytes, sizeof(pidPositionAxesTunings.axes[i]));

        // Ecrit 0 à l'adresse 0 pour indiquer que la première fois est passée
        dueFlashStorage.write(0, 0);

    } else {  //On récupère les valeurs stockées dans la Flash
        uint8_t *byteArrayPidSpeed = dueFlashStorage.readAddress(STARTINGADDRESSPIDSPEED);
        memcpy(&pidSpeedTunings, byteArrayPidSpeed, sizeof(pidSpeedTunings));  //Copie le tableau d'octets stockés dans la Flash vers la structure
        uint8_t *byteArrayStartingPosition = dueFlashStorage.readAddress(STARTINGADDRESSSTARTINGPOSITION);
        memcpy(&startingPosition, byteArrayStartingPosition, sizeof(startingPosition));

        for (uint8_t i = 0; i < NBAXIS; i++) {
            uint8_t *byteArrayPidPosition = dueFlashStorage.readAddress(STARTINGADDRESSPIDPOSITION[i]);
            memcpy(&pidPositionAxesTunings.axes[i], byteArrayPidPosition, sizeof(pidPositionAxesTunings.axes[i]));
        }

        updatePidSpeedTunings();
        updatePidPositionTunings();
    }

    /* On met la position de départ renseignée avant le reset ou celle par défaut s'il y a eu un nouveau téléversement */
    for (uint8_t i = 0; i < NBAXIS; i++) {
        currentPosition_float.axes[i] = startingPosition.axes[i];
    }

    gyroAngleZ = startingPosition.z;  //On Initialise gyroAngleZ à la valeur d'angle de départ
}

void initMPU9250() {
    if (IMU.begin() < 0)
        IMUStatus = false;
    else {
        IMUStatus = true;
        // Met la bande passante du filtre passe bas digital à 20Hz
        IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
        // setting SRD to 19 for a 50 Hz update rate
        IMU.setSrd(19);
        // enabling the data ready interrupt
        IMU.enableDataReadyInterrupt();
        // attaching the interrupt to microcontroller pin
        pinMode(PININTERRUPTMPU9250, INPUT);
        attachInterrupt(PININTERRUPTMPU9250, IMUGyroSpeedZ, RISING);
    }
}

/* Met à jour la vitesse angulaire du robot selon l'axe Z en °/s et son angle en °  à chaque interruption*/
void IMUGyroSpeedZ() {
    static uint16_t previousTime = 0;
    uint16_t deltaTime = 0;

    IMU.readSensor();
    gyroSpeedZ = degrees(IMU.getGyroZ_rads());

    if (flagActiveGyroAngleZ) {
        deltaTime = micros() - previousTime;
        gyroAngleZ += gyroSpeedZ * deltaTime * CONVERSION_US_S;  //On intègre la vitesse pour obtenir l'angle
        gyroAngleZ = angleScale(gyroAngleZ);    //On remet l'angle entre -/+ 180 °
        previousTime = micros();
    }
}

void resetValues() {
    setSameMotorPwm(0);
    initTimer();
    for (uint8_t i = 0; i < NBMOTORS; i++) {
        count[i] = 0;
        rotationSpeed[i] = 0;
        laststateA[i] = 0;
        //laststateB[i] = 0;  //(si utilisation des interruptions sur les 2 encodeurs)
        pwm[i] = 0;
        setpointSpeed[i] = 0;
        setpointPidSpeed[i] = 0;
        outputPidSpeed[i] = 0;
    }
    for (uint8_t i = 0; i < NBAXIS; i++) {
        setRobotSpeed(0, i);
        robotSpeedsMeasured.axes[i] = 0;
        setCurrentPosition(startingPosition.axes[i], i);
        setTargetPosition(currentPosition_float.axes[i], i);
        setpointPidPosition.axes[i] = currentPosition_float.axes[i];
        outputPidPosition.axes[i] = 0;
    }

    gyroSpeedZ = 0;
    gyroAngleZ = currentPosition_float.z;
    currentTheta = currentPosition_float.z;
    activePidSpeed(true);
    activeRobotSpeedMode(false);
    flagCodeRunningForTheFirstTime = 0;
    activeGyroAngleZ(false);
    setTravelMode(AUTOMODENUMBER);
    activeBeginTravel(false);
    activePositionMode(false);
}

void resetStoredValues() {
    for (uint8_t i = 0; i < NBAXIS; i++) {
        setPidSpeedTuning(defaultTuningsPidSpeed.tunings[i], i);
        setStartingPosition(defaultStartingPosition.axes[i], i);
        for (uint8_t j = 0; j < NBAXIS; j++)
            setPidPositionTuning(defaultPidPositionAxesTunings.axes[i].tunings[j], i, j);
    }
}

/* Timer qui extrait toutes les 0.1s la valeur de vitesse de rotation calculée grâce au count */
void timerIsr() {
    Coordinates_float deltaToTarget;
    static float beginningTravelAngle = 0;  //Angle au début du mouvement

    for (uint8_t i = 0; i < NBMOTORS; i++) {
        rotationSpeed[i] = (float)count[i];  // vitesse de rotation du moteur en tic/0.01s ; sens avant si vitesse positive (encodeur A avant encodeur B)
        count[i] = 0;
    }

    /* Calcul de la nouvelle position du robot dans le repère du sol et de sa vitesse dans son propre plan */
    currentPosition_robotSpeedsMeasure(rotationSpeed[0], rotationSpeed[1], rotationSpeed[2], rotationSpeed[3]);

    /* On adapte thetaTarget en fonction de l'angle courant pour parcourir le plus court chemin */
    thetaTargetCompute(targetPosition.z);

    /* Mise à jour de la distance qu'il reste à parcourir */
    for (uint8_t i = 0; i < NBAXIS; i++)
        deltaToTarget.axes[i] = targetPosition.axes[i] - currentPosition_float.axes[i];
    float distanceToTarget = sqrt(pow(deltaToTarget.x, 2) + pow(deltaToTarget.y, 2));

    if (flagPidSpeedIsActive) {
        if (flagRobotSpeedMode)
            updateSetpointsSpeed();  //Calcul et mise à l'échelle des setpointPidSpeed en fonction des vitesses robot demandées
        else if (flagPositionMode) {
            if (arrival(distanceToTarget, deltaToTarget.z)) {
                for (uint8_t i = 0; i < NBAXIS; i++)
                    robotSpeedsOrder.axes[i] = 0;
            } else {
                if (flagBeginTravel) {
                    beginningTravelAngle = currentPosition_float.z;  // On change la valeur que lorsque l'on débute un déplacement et une seule fois
                }
                updateSetpointsPidPosition(beginningTravelAngle, deltaToTarget.x, deltaToTarget.y, distanceToTarget);  //Calcul des setpointPidPosition
                activeBeginTravel(false);
                for (uint8_t i = 0; i < NBAXIS; i++)
                    pidPosition[i].Compute();
                updateRobotSpeedsOrder();
            }
            updateSetpointsSpeed();
        }
        for (uint8_t i = 0; i < NBMOTORS; i++) {
            pidSpeed[i].Compute();
            setMotorPwm(i, (int16_t)outputPidSpeed[i]);
        }
    }
}

void activePidSpeed(bool state) {
    flagPidSpeedIsActive = state;
}

void activeRobotSpeedMode(bool state) {
    flagRobotSpeedMode = state;
}

void activeGyroAngleZ(bool state) {
    flagActiveGyroAngleZ = state;
}

void activePositionMode(bool state) {
    flagPositionMode = state;
}

void activeBeginTravel(bool state) {
    flagBeginTravel = state;
}

void stopTimer() {
    Timer1.stop();
}

/* Envoi un même ordre de vitesse et de direction à tous les moteurs */
void setSameMotorPwm(int16_t pwmOrder) {
    for (uint8_t i = 0; i < NBMOTORS; i++)
        setMotorPwm(i, pwmOrder);
}

/* Envoi un ordre de vitesse et de direction à un moteur */
void setMotorPwm(uint8_t motorNumber, int16_t pwmOrder) {
    bool forward = false;
    if (pwmOrder > 0)
        forward = true;
    digitalWrite(PINDIRECTION[motorNumber], !(forward ^ FORWARDDIRECTIONSTATE[motorNumber]));
    setPwm(motorNumber, pwmOrder);
}

void setPwm(uint8_t motorNumber, int16_t pwmOrder) {  //PWM allant de 0 au prescaler (ici de 0 à 2100)
    pwm[motorNumber] = pwmOrder;
    if (pwmOrder == 0) {
        pwmOrder = 1;  //Car en configurant le registre, un pwm de 0 est équivalent à un pwm de 2100
    }
    pwmOrder = abs(pwmOrder);
    switch (motorNumber) {
        case 0:
            REG_TC2_RA0 = pwmOrder;  //Pin 5 (moteur 1)
            break;
        case 1:
            REG_TC2_RA2 = pwmOrder;  //Pin 11 (moteur 2)
            break;
        case 2:
            REG_TC2_RB0 = pwmOrder;  //Pin 4 (moteur 3) (oui c'est bien inversé entre RA0 et RB0)
            break;
        case 3:
            REG_TC2_RB2 = pwmOrder;  //Pin 12 (moteur 4)
            break;
    }
}

void setPidSpeedTuning(float tuning, uint8_t tuningTypeNumber) {
    if (pidSpeedTunings.tunings[tuningTypeNumber] != tuning) {
        pidSpeedTunings.tunings[tuningTypeNumber] = tuning;
        updatePidSpeedTunings();
        dueFlashStorage.write(STARTINGADDRESSPIDSPEED, pidSpeedTunings.bytes, sizeof(pidSpeedTunings));
    }
}

void updatePidSpeedTunings() {
    for (uint8_t i = 0; i < NBMOTORS; i++)
        pidSpeed[i].SetTunings(pidSpeedTunings.kp, pidSpeedTunings.ki, pidSpeedTunings.kd);
}

void setSameMotorRpm(int16_t rpmOrder) {
    for (uint8_t i = 0; i < NBMOTORS; i++)
        setMotorRpm(i, rpmOrder);
}

void setMotorRpm(uint8_t motorNumber, int16_t rpmOrder) {
    setpointPidSpeed[motorNumber] = rpmOrder / TICBYSECONDTORPM;  //Conversion de RPM au niveau de la roue en tic/0.01s au niveau du moteur
}

void setRobotSpeed(int16_t speedRobot, uint8_t axisNumber) {
    robotSpeedsOrder.axes[axisNumber] = speedRobot;
}

/* Calcul et mise à l'échelle des vitesses de rotation et accélérations aux valeurs max*/
void updateSetpointsSpeed() {
    if (robotSpeedsOrder.x == 0 && robotSpeedsOrder.y == 0 && robotSpeedsOrder.z == 0) {
        for (uint8_t i = 0; i < NBMOTORS; i++)
            setpointSpeed[i] = 0;
    } else {

        /* Mise à jour des vitesses de rotation */
        setpointsSpeedCompute();
    }
    /* Mise à l'échelle des setpointSpeed s'il y en a un qui dépasse les capacités moteur en vitesse de rotation ou accélération */
    setpointsSpeedScaleSpeed();
    setpointsSpeedScaleAcceleration();
}


/* Calcul des setpointSpeed pour le PID de chaque moteur, en fonction de la vitesse souhaitée du robot dans son repère */
void setpointsSpeedCompute() {
    setpointSpeed[0] = (robotSpeedsOrder.y / KY + robotSpeedsOrder.x / KX - robotSpeedsOrder.z / KTHETA) * TIMERPERIODE_S;  //en tic/0.01s
    setpointSpeed[1] = (robotSpeedsOrder.y / KY - robotSpeedsOrder.x / KX + robotSpeedsOrder.z / KTHETA) * TIMERPERIODE_S;  //On multiplie par 0.01s car les coefficients sont en mm/tic et °/tic
    setpointSpeed[2] = (robotSpeedsOrder.y / KY - robotSpeedsOrder.x / KX - robotSpeedsOrder.z / KTHETA) * TIMERPERIODE_S;
    setpointSpeed[3] = (robotSpeedsOrder.y / KY + robotSpeedsOrder.x / KX + robotSpeedsOrder.z / KTHETA) * TIMERPERIODE_S;
}

/* Mise à l'échelle proportionnelle des setpointSpeed s'il y en a un qui dépasse les capacités du moteur */
void setpointsSpeedScaleSpeed() {
    float setpointSpeedMax = 0;
    bool flagSetpointSpeedOversized = false;

    for (uint8_t i = 0; i < NBMOTORS; i++) {
        if (abs(setpointSpeed[i]) > MAXROTATIONSPEED) {
            flagSetpointSpeedOversized = true;
            if (abs(setpointSpeed[i]) > setpointSpeedMax)
                setpointSpeedMax = abs(setpointSpeed[i]);
        }
    }
    if (flagSetpointSpeedOversized) {
        float ratio = MAXROTATIONSPEED / setpointSpeedMax;
        for (uint8_t i = 0; i < NBMOTORS; i++)
            setpointSpeed[i] *= ratio;
    } else {
        for (uint8_t i = 0; i < NBMOTORS; i++)
            setpointPidSpeed[i] = setpointSpeed[i];
    }
}

void setpointsSpeedScaleAcceleration() {
    float AccelerationMax = 0;
    bool flagAccelerationOversized = false;

    for (uint8_t i = 0; i < NBMOTORS; i++) {
        float currentRotationAcceleration = setpointSpeed[i] - setpointPidSpeed[i];
        if (abs(currentRotationAcceleration) > MAXROTATIONACCELERATION) {
            flagAccelerationOversized = true;
            if (abs(currentRotationAcceleration) > AccelerationMax)
                AccelerationMax = abs(currentRotationAcceleration);
        }
    }
    if (flagAccelerationOversized) {
        float ratio = MAXROTATIONACCELERATION / AccelerationMax;
        for (uint8_t i = 0; i < NBMOTORS; i++) {
            float scaledAcceleration = (setpointSpeed[i] - setpointPidSpeed[i]) * ratio;
            setpointPidSpeed[i] += scaledAcceleration;  // C'est seulement après tous les calculs que l'on met à jour le setpointPidSpeed pour garder en mémoire la vraie valeur souhaitée en setpointSpeed
        }
    }
}

void setStartingPosition(int16_t position, uint8_t axisNumber) {
    if (startingPosition.axes[axisNumber] != position) {
        if (axisNumber == ZAXISNUMBER)
            startingPosition.z = angleScale((float)position);
        else
            startingPosition.axes[axisNumber] = (float)position;
        dueFlashStorage.write(STARTINGADDRESSSTARTINGPOSITION, startingPosition.bytes, sizeof(startingPosition));
    }
}

void setCurrentPosition(int16_t position, uint8_t axisNumber) {
    if (axisNumber == ZAXISNUMBER) {
        gyroAngleZ = angleScale((float)position);
        currentPosition_float.z = angleScale((float)position);
        currentTheta = angleScale((float)position);
    } else
        currentPosition_float.axes[axisNumber] = (float)position;
}

/* Ecrètage de l'angle en paramètre entre -/+ 180 ° */
float angleScale(float angle) {
    while (angle > 180)
        angle -= 360;
    while (angle < -180)
        angle += 360;
    return angle;
}

void setPidPositionTuning(float tuning, uint8_t axisNumber, uint8_t tuningTypeNumber) {
    if (pidPositionAxesTunings.axes[axisNumber].tunings[tuningTypeNumber] != tuning) {
        pidPositionAxesTunings.axes[axisNumber].tunings[tuningTypeNumber] = tuning;
        updatePidSpeedTunings();
        dueFlashStorage.write(STARTINGADDRESSPIDPOSITION[axisNumber], pidPositionAxesTunings.axes[axisNumber].bytes, sizeof(pidPositionAxesTunings.axes[axisNumber]));
    }
}

void updatePidPositionTunings() {
    for (uint8_t i = 0; i < NBAXIS; i++)
        pidPosition[i].SetTunings(pidPositionAxesTunings.axes[i].kp, pidPositionAxesTunings.axes[i].ki, pidPositionAxesTunings.axes[i].kd);
}

/* Calcul de la nouvelle position du robot en fonction des mesures prises par les encodeurs*/
void currentPosition_robotSpeedsMeasure(int16_t count0, int16_t count1, int16_t count2, int16_t count3) {
    float deltaXRobotTraveled = KX * (count0 - count1 - count2 + count3) / 4;      //en mm
    float deltaYRobotTraveled = KY * (count0 + count1 + count2 + count3) / 4;      //en mm
    float deltaThetaTraveled = KTHETA * (-count0 + count1 - count2 + count3) / 4;  //en °

    currentPositionMeasure(deltaXRobotTraveled, deltaYRobotTraveled, deltaThetaTraveled);
    robotSpeedsMeasure(deltaXRobotTraveled, deltaYRobotTraveled, deltaThetaTraveled);
}

/* Projection de la variation de position dans le plan du sol */
void currentPositionMeasure(float deltaXRobotTraveled, float deltaYRobotTraveled, float deltaThetaTraveled) {
    currentTheta += deltaThetaTraveled;
    currentTheta = angleScale(currentTheta);  //On remet theta de -180° à 180°

    /* On prend soit en compte la valeur d'angle du gyromètre, soit des encodeurs moteurs */
    if (flagActiveGyroAngleZ)
        currentPosition_float.z = gyroAngleZ;
    else
        currentPosition_float.z = currentTheta;
    
    currentPosition_float.x += deltaXRobotTraveled * cos(radians(currentPosition_float.z)) - deltaYRobotTraveled * sin(radians(currentPosition_float.z));
    currentPosition_float.y += deltaYRobotTraveled * cos(radians(currentPosition_float.z)) + deltaXRobotTraveled * sin(radians(currentPosition_float.z));
}

void robotSpeedsMeasure(float deltaXRobotTraveled, float deltaYRobotTraveled, float deltaThetaTraveled) {
    robotSpeedsMeasured.x = deltaXRobotTraveled / TIMERPERIODE_S;
    robotSpeedsMeasured.y = deltaYRobotTraveled / TIMERPERIODE_S;
    robotSpeedsMeasured.z = deltaThetaTraveled / TIMERPERIODE_S;
}

void setTargetPosition(int16_t position, uint8_t axisNumber) {
    if (axisNumber == ZAXISNUMBER)
        thetaTargetCompute(position);
    else
        targetPosition.axes[axisNumber] = position;
}

void thetaTargetCompute(float angle) {
    float thetaTarget = angleScale(angle);                                 //On ramène thetaTarget de -180° à 180°
    thetaTarget = adaptAngleTarget(thetaTarget, currentPosition_float.z);  //On adapte thetaTarget de façon à parcourir le plus court chemin
    targetPosition.z = thetaTarget;
}

void setTravelMode(uint8_t mode) {
    travelMode = mode;
}

/* Choix de l'orientation du robot en fonction du mode sélectionné */
void updateSetpointsPidPosition(float beginningTravelAngle, float deltaXTableToTarget, float deltaYTableToTarget, float distanceToTarget) {
    setpointPidPosition.x = targetPosition.x;
    setpointPidPosition.y = targetPosition.y;

    /* Choix du setpointTheta en fonction du mode de déplacement et de la distance jusqu'à la cible */
    if (distanceToTarget > FINALPHASEDISTANCE) {
        if (travelMode == STRAFEMODENUMBER)
            setpointPidPosition.z = beginningTravelAngle;  //On garde l'angle du début du déplacement
        else if (travelMode == ORIENTATIONMODENUMBER) {
            if (flagBeginTravel)                                                       //On calcul une seule fois l'angle d'orientation pour que le robot soit asservit sur l'angle d'orientation du début du déplacement
                angleChoiceOrientationMode(deltaXTableToTarget, deltaYTableToTarget);  //On s'oriente vers l'angle directeur de la position cible
        } else {                                                                       //On est en mode auto
            if (flagBeginTravel)                                                       //On calcul une seule fois l'angle pour que le robot soit asservit sur l'angle du début du déplacement
                angleChoiceAutoMode(beginningTravelAngle, deltaXTableToTarget, deltaYTableToTarget, distanceToTarget);
        }
    } else  //On est très proche de la cible donc on s'oriente vers l'angle final
        setpointPidPosition.z = targetPosition.z;
}

void angleChoiceOrientationMode(float deltaXTableToTarget, float deltaYTableToTarget) {
    float steeringAngle = -degrees(atan2(deltaXTableToTarget, deltaYTableToTarget));  //On souhaite avoir l'angle par rapport à l'axe Y de la table (on inverse deltaX et deltaY) pour le comparer à l'axe y du robot.
    steeringAngle = adaptAngleTarget(steeringAngle, currentPosition_float.z);         //On adapte steeringAngle de façon à parcourir le plus court chemin
    setpointPidPosition.z = steeringAngle;
}

/* Choix de l'orientation du robot en fonction de sa localisation par rapport à l'objectif */
void angleChoiceAutoMode(float beginningTravelAngle, float deltaXTableToTarget, float deltaYTableToTarget, float distanceToTarget) {
    float steeringAngle = -degrees(atan2(deltaXTableToTarget, deltaYTableToTarget));                                     //On souhaite avoir l'angle par rapport à l'axe Y de la table (on inverse deltaX et deltaY) pour le comparer à l'axe y du robot.
    int16_t deltaSteeringAngleTo90 = deltaSteeringAngleScale((int16_t)steeringAngle, (int16_t)currentPosition_float.z);  //On adapte deltaSteeringAngle pour regarder s'il est compris dans le cone d'angle 2*MAXSTRAFEANGLE autour des angles multiples de 90°

    if ((abs(distanceToTarget) < MAXSTRAFEDISTANCE) || (abs(deltaSteeringAngleTo90) < MAXSTRAFEANGLE))  //On passe en mode strafe si la cible est près du point de départ ou l'orientation est favorable.
        setpointPidPosition.z = beginningTravelAngle;                                                   //Mode strafe
    else {
        setpointPidPosition.z = adaptAngleTarget(steeringAngle, currentPosition_float.z);  // Mode orientation vers la cible
    }
}

/* Adaptation de l'angle cible en fonction de l'angle courant pour parcourir le plus court chemin */
float adaptAngleTarget(float angleTarget, float angle) {
    float deltaAngle = angleTarget - angle;
    if (deltaAngle > 180 && deltaAngle <= 360)
        angleTarget -= 360;
    if (deltaAngle >= -360 && deltaAngle < -180)
        angleTarget += 360;
    return angleTarget;
}

/* On ramène le deltaAngle entre -90 et +90° pour voir si le strAngle est proches des modulos 90°*/
int16_t deltaSteeringAngleScale(int16_t strAngle, int16_t angle) {
    int16_t deltaAngleTo90 = (strAngle - angle) % 90;
    if (deltaAngleTo90 > 45)
        deltaAngleTo90 -= 90;
    else if (deltaAngleTo90 < -45)
        deltaAngleTo90 += 90;
    return deltaAngleTo90;
}

/* Calcul des consignes de vitesses du robot dans son repère */
void updateRobotSpeedsOrder() {
    robotSpeedsOrder.x = outputPidPosition.x * cos(radians(currentPosition_float.z)) + outputPidPosition.y * sin(radians(currentPosition_float.z));
    robotSpeedsOrder.y = -outputPidPosition.x * sin(radians(currentPosition_float.z)) + outputPidPosition.y * cos(radians(currentPosition_float.z));
    robotSpeedsOrder.z = outputPidPosition.z;

    //robotSpeedOrderClipping(xSpeedRobotOrder, ySpeedRobotOrder, thetaSpeedRobotOrder);
}

/* Arrêt des moteurs lorsque l'on est arrivé au point cible */
bool arrival(float distanceToTarget, float deltaThetaToTarget) {
    if (abs(distanceToTarget) < POSITIONTOLERANCE && abs(deltaThetaToTarget) < ANGLETOLERANCE)
        return true;
    else
        return false;
}

int16_t getCount(uint8_t motorNumber) {
    return count[motorNumber];
}

int16_t getRpm(uint8_t motorNumber) {
    return (int16_t)(rotationSpeed[motorNumber] * TICBYSECONDTORPM);
}

bool getCodeRunningForTheFirstTime() {
    return flagCodeRunningForTheFirstTime;
}

float getPidSpeedTunings(uint8_t tuningNumber) {
    return pidSpeedTunings.tunings[tuningNumber];
}

int16_t getSetpointPidSpeed(uint8_t motorNumber) {
    return (int16_t)setpointPidSpeed[motorNumber];
}

int16_t getMotorRotationSpeed(uint8_t motorNumber) {
    return (int16_t)rotationSpeed[motorNumber];
}

uint16_t getMaxRobotSpeeds(uint8_t axisNumber) {
    return maxRobotSpeeds.axes[axisNumber];
}

int16_t getRobotSpeedsOrder(uint8_t axisNumber) {
    return robotSpeedsOrder.axes[axisNumber];
}

int16_t getRobotSpeedsMeasured(uint8_t axisNumber) {
    return robotSpeedsMeasured.axes[axisNumber];
}

int16_t getStartingPosition(uint8_t axisNumber) {
    return startingPosition.axes[axisNumber];
}

int16_t getCurrentPosition(uint8_t axisNumber) {
    return currentPosition_float.axes[axisNumber];
}

bool getIMUStatus() {
    return IMUStatus;
}

int16_t getGyroSpeedZ() {
    return gyroSpeedZ;
}

int16_t getGyroAngleZ() {
    return (int16_t)gyroAngleZ;
}

int16_t getTargetPosition(uint8_t axisNumber) {
    return targetPosition.axes[axisNumber];
}

float getPidPositionTuning(uint8_t axisNumber, uint8_t tuningTypeNumber) {
    return pidPositionAxesTunings.axes[axisNumber].tunings[tuningTypeNumber];
}

int16_t getSetpointPidPosition(uint8_t axisNumber){
    return setpointPidPosition.axes[axisNumber];
}
