#include "alarm_system.h"

char* user_input;
// char buffer[5];
system_state* alarm_system;

void System_Init(void) {
	alarm_system = (system_state*) calloc(1, sizeof(system_state));
	alarm_system->password = malloc(sizeof(char) * 5);
	user_input = malloc(sizeof(char) * 5);

	alarm_system->state = ready;
	alarm_system->password= "1001";

	user_input = "\0";
}

bool Check_Password(void) {
	if (!strcmp(alarm_system->password, user_input))		return true;
	return false;
}

bool Lock_System(void) {
	if (__GET_SYSTEM_STATE == ready) {
		alarm_system->state = set;
		return true;
	}
	return false;
}

bool Unlock_System(void) {
	if (__GET_SYSTEM_STATE == set && Check_Password()) {
		alarm_system->state = ready;
		return true;
	}
	return false;
}

int Change_Password(char* new_password) {
	if (Check_Password()) {
		alarm_system->password = new_password;
		return 0;
	}
	return 1;
}
