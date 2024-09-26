
#ifndef INC_ALARM_SYSTEM_H_
#define INC_ALARM_SYSTEM_H_

// INCLUDES
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

// DEFINES
// for enabling and disabling the passive buzzer
// the note played is an E5 plus or minus a few cents
#define ENABLE_BEEP 					1761
#define DISABLE_BEEP 					0

// the lengths at which the beep will sound depending on the event
#define LOCK_COUNTDOWN_BEEP_LEGNTH		330
#define INPUT_BEEP_LENGTH				165
#define OPEN_ON_READY_BEEP_LENGTH		494
#define OPEN_ON_SET_BEEP_LENGTH			41
#define OPEN_ON_SET_SILENT_LENGTH		453

// the amount of times the countdown beep will sound (10 seconds in total, once per second)
#define LOCK_COUNTDOWN_COUNT 			20

// PRIVATE VARIABLES
typedef struct System_State system_state;
enum State { ready, set };
extern char* user_input;
// extern char buffer[5];
extern system_state* alarm_system;

struct System_State {
	char* password;
	enum State state;
};

// MACROS
#define __GET_SYSTEM_STATE  alarm_system->state    //(__HANDLE__)	((__HANDLE__).state)

// FUNCTION PROTOTYPES
void System_Init(void);
bool Check_Password(void);
bool Lock_System(void);
bool Unlock_System(void);
int Change_Password(char* new_password);


#endif /* INC_TELEGRAPH_CONFIGURATIONS_H_ */


