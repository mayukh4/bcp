#ifndef PBOB_H
#define PBOB_H

#include <stdbool.h>

#define NUM_PBOB 3    
#define NUM_RELAYS 6  
#define DIR_TYPE 1    
#define STATE_TYPE 0  
#define DELAY_US 20000  
#define MAXLEN 1024

typedef struct {
    int relay_id;        
    bool state;   
    int toggle;
    int registerAddress;
} Relay; 

typedef struct {
    int handle;
    int num_relays; 
    Relay relays[NUM_RELAYS];
    const char* ip;
    int id;
} RelayController; 

// Function prototypes - match the actual implementations
void handle_ljm_error(int err, const char* operation, int relay_id, const char* function);
int open_labjack(RelayController* ctrl);
void close_labjack(RelayController* ctrl);
int initialize_relays(RelayController* ctrl);  // Changed to int return type
void toggle_relay(RelayController* ctrl, int relay_id);
void set_all_relays(RelayController* ctrl, bool state);
void run_controller(RelayController* ctrl);
int run_pbob();
void set_toggle(int pbob_id, int relay_id);
void* run_pbob_thread(void* arg);
int all_relays_off();

extern RelayController controller[NUM_PBOB];
extern FILE* pbob_log_file;
extern int shutdown_pbob;
extern int pbob_enabled;

#endif
