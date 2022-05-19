#ifndef Mouvement_h
#define Mouvement_h

#include "Arduino.h"

#define NBMOTORS 4
#define NBAXIS 3

#define XAXISNUMBER 0
#define YAXISNUMBER 1
#define ZAXISNUMBER 2
#define KPTUNINGNUMBER 0
#define KITUNINGNUMBER 1
#define KDTUNINGNUMBER 2

#define SHIFT2 2
#define SHIFT3 3
#define SHIFT4 4
#define SHIFT5 5
#define SHIFT6 6
#define SHIFT7 7
#define SHIFT8 8
#define SHIFT9 9

/* Valeurs du Pid Vitesse par défaut sauvegardées dans la Flash */
#define DEFAULTKPSPEED 5
#define DEFAULTKISPEED 55
#define DEFAULTKDSPEED 1e-7
#define STARTINGADDRESSPIDSPEED 4  //On ne peut commencer à écrire qu'à partir d'une adresse multiple de 4 bytes !!

/* Position de départ par défaut du robot sauvegardées dans la Flash */
#define DEFAULTXSTARTINGPOSITION 0
#define DEFAULTYSTARTINGPOSITION 0
#define DEFAULTTHETASTARTINGANGLE 0     // Doit être compris entre -/+ 180 °
#define STARTINGADDRESSSTARTINGPOSITION 16  //On ne peut commencer à écrire qu'à partir d'une adresse multiple de 4 bytes et décalé le 12 par rapport à l'adresse de début du pidTunings (taille tableau bytes)!!

/* Mode de déplacement vers la position cible */
#define AUTOMODENUMBER 0  //Orientation ou strafe en fonction de la configuration
#define STRAFEMODENUMBER 1
#define ORIENTATIONMODENUMBER 2

/* Coefficients du Pid en position par défaut sauvegardées dans la Flash */
#define DEFAULTKPPOSITIONX 1.8
#define DEFAULTKPPOSITIONY 1.8
#define DEFAULTKPANGLETHETA 1.8
#define DEFAULTKIPOSITIONX 0
#define DEFAULTKIPOSITIONY 0
#define DEFAULTKIANGLETHETA 0
#define DEFAULTKDPOSITIONX 0
#define DEFAULTKDPOSITIONY 0
#define DEFAULTKDANGLETHETA 0
#define STARTINGADDRESSPIDPOSITIONX 24  //On ne peut commencer à écrire qu'à partir d'une adresse multiple de 4 bytes et décalé le 6 par rapport à l'adresse de début du startingPosition (taille tableau bytes)!!
#define STARTINGADDRESSPIDPOSITIONY 36  
#define STARTINGADDRESSPIDPOSITIONZ 48  

/* Union de structure regroupant les coefficients des Pid */
union PidTunings {
    struct {
        float kp;
        float ki;
        float kd;
    };
    float tunings[3];
    uint8_t bytes[12];  //Stockage en uint8_t pour pouvoir stocker dans la mémoire Flash
};

union PidAxesTunings{
    struct{
        PidTunings x;
        PidTunings y;
        PidTunings z;
    };
    PidTunings axes[3];
};

/* Union de structure regroupant des valeurs selon les 3 axes */
union Coordinates {
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    int16_t axes[3];
    uint8_t bytes[6];  //Stockage en uint8_t pour pouvoir stocker dans la mémoire Flash
};

union Coordinates_float {
    struct {
        float x;
        float y;
        float z;
    };
    float axes[3];
};

/* Câblage MPU9250 */
const uint8_t PININTERRUPTMPU9250 = 42;
const uint8_t PINCHIPSELECT = 43;

/* Câblage Moteurs */
//const uint8_t PINENCODERA[] = {24, 27, 22, 29};    //Tableau des sorties A des encodeurs
//const uint8_t PINENCODERB[] = {25, 26, 23, 28};    //Inversion entre encodeurA et encodeurB pour les moteurs 2 et 4 pour une raison de côté droit/gauche du robot
const uint8_t PINENCODERA[] = {36, 39, 34, 41};  //Tableau des sorties A des encodeurs
const uint8_t PINENCODERB[] = {37, 38, 35, 40};
const uint8_t PINSPEED[] = {5, 11, 4, 12};                    //Tableau des pins PWM des moteurs
const uint8_t PINDIRECTION[] = {6, 10, 3, 13};                //Tableau des pins de direction des moteurs
const bool FORWARDDIRECTIONSTATE[] = {HIGH, LOW, HIGH, LOW};  //Etat des pins de direction pour que le robot avance

/* Timer counter */
const uint16_t FREQUENCY = 20000;                                //Fréquence du signal PWM envoyé aux moteurs en Hz
const uint16_t PRESCALER = 42 * 1000000 / FREQUENCY;             //diviseur de la fréquence maximale (42MHz) => nouvelle fréquence à 20kHz
const uint16_t TIMERPERIODE_US = 10000;                          //En microsecondes
const float CONVERSION_US_S = 1e-6;                              //Conversion de microsecondes en secondes
const float TIMERPERIODE_S = TIMERPERIODE_US * CONVERSION_US_S;  //En secondes (=0.01s)

/* Capacités moteur */
const uint16_t MAXRPMORDER = 70;              //En tr/mn (nous l'avons baissé afin que les calculs d'interruptions arrivent à suivre) 70 125
const uint16_t MAXROTATIONSPEED = 110;         //Vitesse de rotation maximale en tic/0.01s  110 285
const uint16_t MAXROTATIONACCELERATION = 60;  //Accélération de rotation maximale en tic/(0.01s)² 60 100

/*Constantes moteur */
const float WHEELRADIUS = 76.2;  //En mm
const float WHEELPERIMETER = 2 * PI * WHEELRADIUS;
const uint8_t GEARREDUCTION = 27;  //Rapport de réduction du réducteur
//const uint16_t TICBYROTATION = 2000;        //Nombre de tics par tour d'arbre moteur (500*2*2 pour le calcul sur les 2 encodeurs et sur les fronts montants et descendants) (si utilisation des interruptions sur les 2 encodeurs)
const uint16_t TICBYMOTORROTATION = 500;                                 //Nombre de tics par tour d'arbre moteur (500 pour le calcul sur l'encodeur A et sur les fronts montants)
const uint16_t TICBYWHEELROTATION = GEARREDUCTION * TICBYMOTORROTATION;  //Nombre de tics par tour de roue
const uint16_t WHEELTOCENTERRADIUS = 324;                                //Distance du centre d'une roue au centre du robot en mm
const float WHEELTODIAGONALEANGLE = 41.5;                                //Angle entre le vecteur vitesse d'une roue (parallèle à l'axe y du robot) et la perpendiculaire d'une diagonale du robot
const float ADAPTATIONX = 0.912;                                         //Coefficient réglable pour avoir la valeur de Ky correspondant au réel
const float ADAPTATIONY = 1.037;                                         //Coefficient réglable pour avoir la valeur de Ky correspondant au réel
const float ADAPTATIONTHETA = 0.98;                                      //Coefficient réglable pour avoir la valeur de Ktheta correspondant au réel

const float KX = WHEELPERIMETER * ADAPTATIONX / (float)TICBYWHEELROTATION;                                                                           // = 0.0323    //Facteur en mm/tic pour le calcul de la variation de position transversal du robot
const float KY = WHEELPERIMETER * ADAPTATIONY / (float)TICBYWHEELROTATION;                                                                           // = 0.0368    //Facteur en mm/tic pour le calcul de la variation de position linéaire du robot
const float KTHETA = WHEELRADIUS * ADAPTATIONTHETA * 360 * cos(radians(WHEELTODIAGONALEANGLE)) / (float)(WHEELTOCENTERRADIUS * TICBYWHEELROTATION);  // = 4.6e-3    //Facteur en °/tic pour le calcul de la variation de rotation du robot

/* Conversions temporelles */
const uint16_t TIMECONVERSION_MN = 60 / TIMERPERIODE_S;                          //Conversion de 0.01s à mn
const float TICBYSECONDTORPM = TIMECONVERSION_MN / (float)(TICBYWHEELROTATION);  //Facteur de conversion d'une vitesse en tic/0.01s en tr/mn

/* Vitesses bridées selon les axes */
const uint16_t MAXSPEEDX = 200;     //En mm/s  On a choisit de la brider à cette valeur 1000
const uint16_t MAXSPEEDY = 200;     //En mm/s  On a choisit de la brider à cette valeur 1000
const uint16_t MAXSPEEDTHETA = 40;  //En °/s   On a choisit de la brider à cette valeur  130

const uint16_t MAXSTRAFEDISTANCE = 1000;  //Distance en mm jusqu'à laquelle le robot se déplace en strafant
const uint8_t MAXSTRAFEANGLE = 10;       //Angle en ° jusqu'au quel le robot se déplace en strafant
const uint8_t FINALPHASEDISTANCE = 250;  //Distance en mm à partir de laquelle le robot s'oriente vers l'angle final
const uint8_t ANGLETOLERANCE = 2;        //Tolérance en ° sur l'orientation du robot
const uint8_t POSITIONTOLERANCE = 2;     //Tolérance en mm sur la position du robot

const uint8_t STARTINGADDRESSPIDPOSITION[] = {STARTINGADDRESSPIDPOSITIONX, STARTINGADDRESSPIDPOSITIONY, STARTINGADDRESSPIDPOSITIONZ};

/* Fonctions "publiques" */
void initMotors();
void initFlashStorage();
void initMPU9250();
void activePidSpeed(bool state);
void activeRobotSpeedMode(bool state);
void activePositionMode(bool state);
void activeBeginTravel(bool state);
void stopTimer();
void resetValues();
void resetStoredValues();

void setSameMotorPwm(int16_t pwmOrder);
void setMotorPwm(uint8_t motorNumber, int16_t pwmOrder);
void setPwm(uint8_t motorNumber, int16_t pwmOrder);
void setPidSpeedTuning(float tuning, uint8_t tuningTypeNumber);
void setSameMotorRpm(int16_t rpmOrder);
void setMotorRpm(uint8_t motorNumber, int16_t rpmOrder);

void setRobotSpeed(int16_t speedRobot, uint8_t axisNumber);
void setStartingPosition(int16_t position, uint8_t axisNumber);
void setCurrentPosition(int16_t position, uint8_t axisNumber);
void setTargetPosition(int16_t position, uint8_t axisNumber);
void setPidPositionTuning(float tuning, uint8_t axisNumber, uint8_t tuningTypeNumber);

int16_t getCount(uint8_t motorNumber);
int16_t getRpm(uint8_t motorNumber);
bool getCodeRunningForTheFirstTime();
float getPidSpeedTunings(uint8_t tuningNumber);
int16_t getSetpointPidSpeed(uint8_t motorNumber);
int16_t getMotorRotationSpeed(uint8_t motorNumber);

uint16_t getMaxRobotSpeeds(uint8_t axisNumber);
int16_t getRobotSpeedsOrder(uint8_t axisNumber);
int16_t getRobotSpeedsMeasured(uint8_t axisNumber);
int16_t getStartingPosition(uint8_t axisNumber);
int16_t getCurrentPosition(uint8_t axisNumber);

bool getIMUStatus();
int16_t getGyroSpeedZ();
int16_t getGyroAngleZ();

int16_t getTargetPosition(uint8_t axisNumber);
float getPidPositionTuning(uint8_t axisNumber, uint8_t tuningTypeNumber);
int16_t getSetpointPidPosition(uint8_t axisNumber);



/* Fonctions "privées" */
void counterA0();
void counterA1();
void counterA2();
void counterA3();
void initTimerCounter6();
void initTimerCounter8();
void initTimer();
void initPidSpeed();
void initPidPosition();
void IMUGyroSpeedZ();
void timerIsr();
void activeGyroAngleZ(bool state);
void updatePidSpeedTunings();
void updateSetpointsSpeed();
void setpointsSpeedCompute();
void setpointsSpeedScaleSpeed();
void setpointsSpeedScaleAcceleration();
float angleScale(float angle);
void updatePidPositionTunings();
void currentPosition_robotSpeedsMeasure(int16_t count0, int16_t count1, int16_t count2, int16_t count3);
void currentPositionMeasure(float deltaXRobotTraveled, float deltaYRobotTraveled, float deltaThetaTraveled);
void robotSpeedsMeasure(float deltaXRobotTraveled, float deltaYRobotTraveled, float deltaThetaTraveled);
void thetaTargetCompute(float angle);
void setTravelMode(uint8_t mode);
void updateSetpointsPidPosition(float beginningTravelAngle, float deltaXTableToTarget, float deltaYTableToTarget, float distanceToTarget);
void angleChoiceOrientationMode(float deltaXTableToTarget, float deltaYTableToTarget);
float adaptAngleTarget(float angleTarget, float angle);
void angleChoiceAutoMode(float beginningTravelAngle, float deltaXTableToTarget, float deltaYTableToTarget, float distanceToTarget);
int16_t deltaSteeringAngleScale(int16_t strAngle, int16_t angle);
void updateRobotSpeedsOrder();
bool arrival(float distanceToTarget, float deltaThetaToTarget);

#endif
