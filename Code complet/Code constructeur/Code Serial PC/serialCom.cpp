/* ------------------------------------------------------------------------ --
--                                                                          --
--             PC serial port connection object exemple                     --
--              with event-driven communication handle                      --
--                                                                          --
--   Based on the original work from  Thierry Schneider                     --
--                                                                          --
-- ------------------------------------------------------------------------ --
--                                                                          --
--  Filename : serialCom.cpp                                                --
--  Author   : Jonathan QUILLES                                             --
--  Created  : 16/04/2020                                                   --
--  Plateform: Windows 95, 98, NT, 2000, XP (Win32), 7                      --
-- ------------------------------------------------------------------------ --
--                                                                          --
--  This software is given without any warranty. It can be distributed      --
--  free of charge as long as this header remains, unchanged.               --
--                                                                          --
-- ------------------------------------------------------------------------ --
--    Note to Visual C++ users:  Don't forget to compile with the           --
--     "Multithreaded" option in your project settings                      --
--                                                                          --
--         See   Project settings                                           --
--                   |                                                      --
--                   *--- C/C++                                             --
--                          |                                               --
--                          *--- Code generation                            --
--                                       |                                  --
--                                       *---- Use run-time library         --
--                                                     |                    --
--                                                     *---- Multithreaded  --
--                                                                          --
--                                                                          --
--                                                                          --
-- ------------------------------------------------------------------------ */


/* ---------------------------------------------------------------------- */


#include <iostream> // Terminal IO => std cout
#include <sstream> // Stringstreams
#include <math.h> // srt , abs
#include <stdlib.h>  // atoi
#include <string.h> // strcmp
#include <conio.h> // _getch
#include <stdio.h> // printf getchar
#include <time.h> //time

#include "Tserial_event.h" // Tserial 


bool initSerial(Tserial_event *com) {
	// Serial 
	int erreur;
	bool echec = false;
	char txt[32];
	int i = 1;

	do {
		com = new Tserial_event();
		if (com != 0) {
			char  ComPortName[10] = "";
			sprintf(ComPortName, "\\\\.\\COM%i", i);
			com->setManager(SerialEventManager);
			erreur = com->connect(ComPortName, 115200, SERIAL_PARITY_NONE, 8, true);
			if (erreur) {
				printf(ComPortName);
				printf("ERROR : com->connect (%ld)\n", erreur);
				com->disconnect();
				delete com;
				com = 0;
			}
			else {
				printf("Connected on com %i ", i);
			}
			i++;
			if(i>99)
				echec = true;
		} else {
			return false;
		}
	} while (erreur&&!echec);
    if(echec) {
		return false;
	}
	com->setRxSize(1);
	return true;
}

int getTimestamp() {
	return timestamp;
}

time_t timestampSystem = 0;

time_t getTimestampSystem() {
	return timestampSystem;
}


void SerialEventManager(uint32 object, uint32 event) {
	char *buffer;
	int   size;
	Tserial_event *com;

	com = (Tserial_event *)object;
	if (com != 0)
	{
		switch (event)
		{
		case  SERIAL_CONNECTED:
			printf("Connected ! \n");
			break;
		case  SERIAL_DISCONNECTED:
			printf("Disonnected ! \n");
			break;
		case  SERIAL_DATA_SENT:
			printf("Data sent ! \n");
			break;
		case  SERIAL_RING:
			printf("DRING ! \n");
			break;
		case  SERIAL_CD_ON:
			printf("Carrier Detected ! \n");
			break;
		case  SERIAL_CD_OFF:
			printf("No more carrier ! \n");
			break;
		case  SERIAL_DATA_ARRIVAL:
			size = com->getDataInSize();
			buffer = com->getDataInBuffer();
			OnDataArrival(size, buffer);
			com->dataHasBeenRead();
	
			break;
		}
	}
}

void OnDataArrival(int size, char *buffer) {
   decode(buffer[0]);  // only one byte because we specified to send interrupt for each byte	
}

void decode(char c) {
  printf("Receive _%c \n",c);  // putchar(c);
}

int main(int argc, char* argv[]) {
    char c;
	char data[1];
    Tserial_event *com;
	
    if(initSerial(com)) {
		printf("Hello!\n");
		printf("Press space to quit\n");
		com->sendData("Hello World",11);
		do  {
                c = getchar(); // c = getch();
                data[0] = c;
                com->sendData(data, 1);    // sending over uart
				printf("Sent _%c \n",c);  // putchar(c);
        } while (c!=32);  // Space = 32
		
		com->disconnect();
        delete com;
        com = 0;
		
	} else {
		printf("ERROR : No COM port detected!\n");
		printf("Press any key to quit.\n");
		c = getchar();
	}
    return 0;
}

