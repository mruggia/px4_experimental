
uint8 FLIFO_STATE_UNDEFINED = 0
uint8 FLIFO_STATE_RSU = 1
uint8 FLIFO_STATE_USD = 2
uint8 FLIFO_STATE_USD_TO_RSU = 3
uint8 FLIFO_STATE_RSU_TO_USD = 4

uint64 timestamp	# time since system start (microseconds)

uint8 state			# current state of the vtol, see FLIFO_STATE
bool is_inv			# flag drone is inverted based on current attitude
