#ifndef Communication_h
#define Communication_h

#include "Arduino.h"

#define LISTENING '#'
#define BEGINLISTENING 'B'
#define FINISHLISTENING 'F'
#define HELP '?'
#define MOTORNUMBER 'M'
#define PWMORDER 'W'
#define DISPLAYCOUNT 'C'
#define VERBOSE 'V'
#define SPLITTERORDER ':'
#define SPLITTERVALUE ','
#define RESETVALUES 'R'
#define RESETSTOREDVALUES 'r'
#define STOPTIMER 'T'
#define RPMORDER 'N'
#define DISPLAYRPM 'n'
#define DISPLAYSETPOINTPIDSPEED 'g'
#define EXPORTDATATOEXCEL 'E'
#define PIDSPEEDTUNING 'k'
#define PROPORTIONNAL 'P'
#define INTEGRAL 'I'
#define DERIVATIVE 'D'
#define DISPLAYSPEEDTUNINGS 'K'
#define XSPEEDROBOTORDER 'X'
#define YSPEEDROBOTORDER 'Y'
#define THETASPEEDROBOTORDER 'Z'
#define DISPLAYROBOTSPEEDSORDER 'v'
#define DISPLAYROBOTSPEEDSMEASURED 'm'
#define DISPLAYGYROSPEEDZ 'G'
#define DISPLAYGYROANGLEZ 'a'
#define CURRENTPOSITION 'c'
#define XAXIS 'x'
#define YAXIS 'y'
#define ZAXIS 'z'
#define DISPLAYPOSITION 'p'
#define STARTINGPOSITION 's'
#define TARGETPOSITIONORDER 't' 
#define AUTOMODE 'A'
#define STRAFEMODE 'S'
#define ORIENTATIONMODE 'O'
#define DISPLAYPIDPOSITIONTUNINGS 'd'



#define DISPLAYINTERVAL 200

#define DEFAULTPRESELECTEDDATA 3    //Sécurité (doit être différent de 0, 1 ou 2)
#define EXPORTSPEEDDATA 1
#define EXPORTPOSITIONDATA 2

const String TUNINGTYPES[] = {"Kp", "Ki", "Kd"};
const String AXES[] = {"X", "Y", "Z"};
const String AXESUNIT[] = {"mm", "mm", "°"};
const String MODES[] = {"Auto", "strafe", "Orientation"};

/* Fonctions "publiques" */
void initCommunication();
void processCommunication();


/* Fonctions "privées" */
void display(bool serialComNumber);
void serialComReception(bool serialComNumber);

int8_t motorNumberChoice(int8_t motorNumber, bool serialComNumber);
void motorPwmOrder(int16_t pwmOrder, bool serialComNumber);
void motorRpmOrder(int16_t rpmOrder, bool serialComNumber);
void pidSpeedTuning(float tuningValue, uint8_t tuningTypeNumber, bool serialComNumber);

void robotSpeedOrder(int16_t speedOrder, uint8_t axisNumber, bool serialComNumber);
void pidPositionTuning(float tuningValue, uint8_t axisNumber, uint8_t tuningTypeNumber, bool serialComNumber);

void displayCount(bool serialComNumber);
void displayRpm(bool serialComNumber);
void displaySetpointPidSpeed(bool serialComNumber);
void exportDataToExcel(uint8_t dataTypeExport, bool serialComNumberExport);

void displayRobotSpeedOrder(bool serialComNumber);
void displayRobotSpeedMeasured(bool serialComNumber);
void displayPosition(char positionType, bool serialComNumber);
void displayCurrentPosition(bool serialComNumber);
void displayGyroSpeedZ(bool serialComNumber);
void displayGyroAngleZ(bool serialComNumber);

#endif
