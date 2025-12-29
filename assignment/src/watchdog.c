/* ======================================================================================
 * SECTION 1: INCLUDES AND GLOBALS
 * Standard headers, process structures, and configuration constants.
 * ====================================================================================== */
#include <stdio.h>
#include <ncurses.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <sys/file.h>
#include <stdarg.h> 
#include <fcntl.h> // <--- CRITICAL: Required for O_NONBLOCK

#include "process_pid.h" 
#include "log.h" 

#define LOG_PATH "logs/watchdog.log"
#define TIMEOUT_US 200000 // 200ms timeout for response
#define CYCLE_DELAY 2     // Seconds between checks

typedef struct {
    pid_t pid;
    char name[32];
    volatile int alive;
} ProcessInfo;

#define MAX_PROCESSES 20
static ProcessInfo process_map[MAX_PROCESSES];
static int process_count = 0;

/* ======================================================================================
 * SECTION 2: HELPER FUNCTIONS
 * Logging wrapper, PID file parsing, and Signal Handling.
 * ====================================================================================== */

// Wrapper to log to both Console (stdout) and File with timestamps
void w_log(const char *format, ...) {
    va_list args;
    char buffer[1024]; 

    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    time_t t = time(NULL);
    struct tm tm_info;
    localtime_r(&t, &tm_info);
    char timebuf[32];
    strftime(timebuf, sizeof(timebuf), "%H:%M:%S", &tm_info);
    
    // Print to console immediately
    printf("[%s] %s\n", timebuf, buffer);
    fflush(stdout); 

    // Print to log file
    logMessage(LOG_PATH, "%s", buffer);
}

// Writes the Watchdog's own PID to the shared file
void publish_my_pid(FILE *fp) {
    fprintf(fp, "%s %d\n", WD_PID_TAG, getpid());
    w_log("[WATCHDOG] PID published securely");
}

// Reads the shared PID file to discover other processes dynamically
void refresh_process_registry() {
    FILE *fp = fopen(PID_FILE_PATH, "r");
    if (!fp) return;
    
    char line[256];
    char tag[128];
    int pid_val;
    
    process_count = 0; 

    while (fgets(line, sizeof(line), fp)) {
        if (sscanf(line, "%s %d", tag, &pid_val) == 2) {
            // Skip own PID
            if (strcmp(tag, WD_PID_TAG) == 0) continue;

            if (process_count < MAX_PROCESSES) {
                process_map[process_count].pid = (pid_t)pid_val;
                process_map[process_count].alive = 0; 
                
                // Map tags to readable names
                if (strcmp(tag, DRONE_PID_TAG) == 0) strcpy(process_map[process_count].name, "DRONE");
                else if (strcmp(tag, BB_PID_TAG) == 0) strcpy(process_map[process_count].name, "BLACKBOARD");
                else if (strcmp(tag, TARGET_PID_TAG) == 0) strcpy(process_map[process_count].name, "TARGET");
                else if (strcmp(tag, OBSTACLE_PID_TAG) == 0) strcpy(process_map[process_count].name, "OBSTACLE");
                else if (strcmp(tag, INPUT_PID_TAG) == 0) strcpy(process_map[process_count].name, "INPUT");
                else strcpy(process_map[process_count].name, "UNKNOWN");
                
                process_count++;
            }
        }
    }
    fclose(fp);
}

// Handler for SIGUSR2 (PONG): Marks a specific process as alive
void pong_handler(int sig, siginfo_t *info, void *context) {
    (void)context;
    if (sig == SIGUSR2) {
        pid_t sender_pid = info->si_pid;
        // Find the process in the map and set alive = 1
        for (int i = 0; i < process_count; i++) {
            if (process_map[i].pid == sender_pid) {
                process_map[i].alive = 1; 
                return;
            }
        }
    }
}

/* ======================================================================================
 * SECTION 3: MAIN SETUP
 * Initialization, Locking, and Configuration.
 * ====================================================================================== */
int main(int argc, char *argv[]) {

    if (argc < 2) return 1;
    int fd_bb_read  = atoi(argv[1]);

    // 1. CRITICAL: SET PIPE TO NON-BLOCKING
    // This prevents the read() call from freezing the Watchdog if no data is present
    fcntl(fd_bb_read, F_SETFL, O_NONBLOCK);

    // Setup Signals
    signal(SIGUSR1, SIG_IGN); // Ignore the signal used for PING
    
    // Setup PONG handler (SIGUSR2)
    struct sigaction sa_pong = {0};
    sa_pong.sa_sigaction = pong_handler;
    sigemptyset(&sa_pong.sa_mask);
    sa_pong.sa_flags = SA_RESTART | SA_SIGINFO; 
    sigaction(SIGUSR2, &sa_pong, NULL);

    // Clean old PID file
    remove(PID_FILE_PATH);

    w_log("[WATCHDOG] Starting... PID: %d", getpid());

    // Securely publish PID with file locking
    FILE *fp_pid = fopen(PID_FILE_PATH, "a");
    if (!fp_pid) {
        w_log("[WATCHDOG] Error opening PID file!");
        exit(1);
    }
    int fd_pid = fileno(fp_pid);
    flock(fd_pid, LOCK_EX); 
    publish_my_pid(fp_pid);
    fflush(fp_pid);
    flock(fd_pid, LOCK_UN);
    fclose(fp_pid);
    
    // Wait for other processes to start and register their PIDs
    w_log("[WATCHDOG] Warm-up phase (4 seconds)...");
    sleep(4); 
    w_log("[WATCHDOG] Warm-up complete. Monitoring started.");

    /* ======================================================================================
     * SECTION 4: MONITORING LOOP
     * The infinite loop that checks system health.
     * ====================================================================================== */
    while (1) {
        char buf[80];
        
        // 1. CHECK FOR QUIT SIGNAL (Non-blocking)
        ssize_t n = read(fd_bb_read, buf, sizeof(buf)-1);
        if(n > 0){
            w_log("[WATCHDOG] Received quit signal. Exiting.");
            break;
        }

        // 2. UPDATE PROCESS LIST
        refresh_process_registry();

        if (process_count == 0) {
            sleep(1);
            continue;
        }

        // 3. SEQUENTIAL HEALTH CHECK
        for(int i=0; i<process_count; i++) {
            
            // A. Reset alive flag
            process_map[i].alive = 0;

            // B. Send PING (SIGUSR1)
            kill(process_map[i].pid, SIGUSR1);

            // C. Wait for response (Fast Polling)
            int elapsed = 0;
            int step = 5000; // 5ms steps
            
            while (process_map[i].alive == 0 && elapsed < TIMEOUT_US) {
                usleep(step);
                elapsed += step;
            }

            // D. Verify Result (Performed AFTER the waiting loop)
            if (process_map[i].alive == 1) {
                // Success: The process responded in time
                w_log("[WATCHDOG] Process %s [PID %d] is responsive!", 
                           process_map[i].name, process_map[i].pid);
            } else {
                // Failure: Timeout reached
                w_log("[WATCHDOG] ALERT! Process %s [PID %d] timed out after %d ms!", 
                           process_map[i].name, process_map[i].pid, elapsed/1000);
                
                w_log("[WATCHDOG] Killing system due to unresponsive process.");
                kill(0, SIGKILL); // Kill the entire process group
                exit(1);
            }
        }
        
        w_log("[WATCHDOG] All %d processes checked. Waiting next cycle...", process_count);
        sleep(CYCLE_DELAY);
    }

    logMessage(LOG_PATH, "[WD] Terminated Successfully");
    return 0;
}