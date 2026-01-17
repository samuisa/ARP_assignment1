/*
 * ======================================================================================
 * MACRO-SECTION 1: HEADERS, CONSTANTS, AND GLOBAL STATE
 * ======================================================================================
 * This section contains standard library inclusions, custom header definitions,
 * global variables for game state management, and internal state monitoring structures.
 */

#include <ncurses.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/select.h> 
#include <sys/file.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>

#include "app_blackboard.h"
#include "app_common.h"
#include "process_pid.h"
#include "log.h"

#define BUFSZ 256
#define OBSTACLE_PERIOD_SEC 5

/* * Internal Process State Enumeration
 * Used to track what the Blackboard is currently doing for logging and debugging purposes.
 */
typedef enum {
    STATE_INIT,                 // Initialization phase
    STATE_IDLE,                 // Waiting in select()
    STATE_PROCESSING_INPUT,     // Handling Keyboard Input
    STATE_UPDATING_MAP,         // Updating Obstacles/Targets logic
    STATE_RENDERING,            // Drawing to Ncurses
    STATE_BROADCASTING          // Sending data via Pipes/Sockets
} BBProcessState;

/* * State Monitor Structure
 * Stores the current state and the timestamp of the last state change.
 */
typedef struct {
    volatile BBProcessState current_state;
    time_t last_state_change;
} BBMonitor;

/* Global Monitoring Instance */
static BBMonitor bb_monitor = {STATE_INIT, 0};

/* Global Game Configuration */
static int current_mode = MODE_STANDALONE;
static int current_role = 0; // 0 = None, 1 = Server, 2 = Client

/* Timing and Optimization Globals */
static struct timespec last_obst_change = {0, 0};
static char last_status[256] = ""; // Caching string to avoid unnecessary redraws

/* Dynamic Game Entities */
static float current_x = 1.0f, current_y = 1.0f; // Local Drone Coordinates
static Point *obstacles = NULL;
static int num_obstacles = 0;
static Point *targets = NULL;
static int num_targets = 0;
static int target_reached = 0;

/* System Handles */
static WINDOW *status_win = NULL;
static pid_t watchdog_pid = -1;

/* Helper Macros */
#define BB_LOG_STATE(msg) \
    logMessage(LOG_PATH, "[BB][%s] %s", state_to_str(bb_monitor.current_state), msg)

/* Function Prototypes */
void reposition_and_redraw(WINDOW **win_ptr, int req_h, int req_w);

/*
 * Helper: Updates the internal state monitor
 */
void set_state(BBProcessState new_state) {
    bb_monitor.current_state = new_state;
    bb_monitor.last_state_change = time(NULL);
}


/*
 * ======================================================================================
 * MACRO-SECTION 2: WATCHDOG INTERACTION & SIGNALS
 * ======================================================================================
 * Functions dedicated to registering the process ID, handling signals, and
 * communicating with the Watchdog process for health checks.
 */

/*
 * Publishes the current PID to a shared file securely.
 */
void publish_my_pid(FILE *fp) {
    fprintf(fp, "%s %d\n", BB_PID_TAG, getpid());
    logMessage(LOG_PATH, "[BB] PID published securely");
}

/*
 * Polls the PID file until the Watchdog's PID is found.
 * Essential for the handshake process in Standalone mode.
 */
void wait_for_watchdog_pid() {
    FILE *fp;
    char line[256], tag[128];
    int pid_temp;
    bool wd_found = false;

    logMessage(LOG_PATH, "[BB] Waiting for Watchdog...");

    while (!wd_found) {
        fp = fopen(PID_FILE_PATH, "r");
        if (fp) {
            while (fgets(line, sizeof(line), fp)) {
                if (sscanf(line, "%s %d", tag, &pid_temp) == 2) {
                    if (strcmp(tag, WD_PID_TAG) == 0) {
                        watchdog_pid = (pid_t)pid_temp;
                        wd_found = true;
                        break;
                    }
                }
            }
            fclose(fp);
        }
        if (!wd_found) usleep(200000); // Sleep 200ms before retry
    }
    logMessage(LOG_PATH, "[BB] Watchdog found (PID %d)", watchdog_pid);
}

/*
 * Signal Handler: Responds to a PING signal (SIGUSR1) from the Watchdog
 * by sending back a PONG signal (SIGUSR2).
 */
void watchdog_ping_handler(int sig) {
    (void)sig; 
    if (watchdog_pid > 0) kill(watchdog_pid, SIGUSR2);
}


/*
 * ======================================================================================
 * MACRO-SECTION 3: NCURSES WINDOW MANAGEMENT
 * ======================================================================================
 * Helper wrappers for Ncurses library functions to create, destroy, and manage
 * terminal windows.
 */

WINDOW* create_window(int height, int width, int starty, int startx) {
    WINDOW *win = newwin(height, width, starty, startx);
    keypad(win, TRUE); // Enable arrow keys
    box(win, 0, 0);    // Draw border
    wnoutrefresh(win);
    return win;
}

void destroy_window(WINDOW *win) {
    if (!win) return;
    werase(win);
    wnoutrefresh(win);
    doupdate();
    delwin(win);
}


/*
 * ======================================================================================
 * MACRO-SECTION 4: RENDERING ENGINE
 * ======================================================================================
 * Functions responsible for drawing game entities (Drone, Obstacles, Targets)
 * onto the Ncurses window.
 */

void draw_background(WINDOW *win) {
    werase(win);
    box(win, 0, 0);
}

void draw_obstacles(WINDOW *win) {
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    for (int i = 0; i < num_obstacles; i++) {
        int ox = obstacles[i].x;
        int oy = obstacles[i].y;
        if (ox > 0 && ox < max_x - 1 && oy > 0 && oy < max_y - 1) {
            wattron(win, COLOR_PAIR(2));
            mvwprintw(win, oy, ox, "O"); 
            wattroff(win, COLOR_PAIR(2));
        }
    }
}

void draw_targets(WINDOW *win) {
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    for (int i = 0; i < num_targets; i++) {
        int tx = targets[i].x;
        int ty = targets[i].y;
        if (tx > 0 && tx < max_x - 1 && ty > 0 && ty < max_y - 1) {
            wattron(win, COLOR_PAIR(3));
            mvwprintw(win, ty, tx, "%d", i + target_reached);
            wattroff(win, COLOR_PAIR(3));
        }
    }
}

void draw_drone(WINDOW *win, float x, float y) {
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    int ix = (int)x;
    int iy = (int)y;

    // Boundary clamping to keep drone inside the box
    if (ix >= max_x - 1) ix = max_x - 2;
    if (iy >= max_y - 1) iy = max_y - 2;
    if (ix < 1) ix = 1;
    if (iy < 1) iy = 1;

    wattron(win, COLOR_PAIR(1));
    mvwprintw(win, iy, ix, "+");
    wattroff(win, COLOR_PAIR(1));
}

/*
 * Master refresh function: Calls all draw sub-routines and refreshes the screen.
 */
void redraw_scene(WINDOW *win) {
    set_state(STATE_RENDERING); 
    draw_background(win);
    
    if(current_mode == MODE_STANDALONE){
        draw_targets(win);
    }
    draw_obstacles(win);
    draw_drone(win, current_x, current_y);

    wnoutrefresh(win);
    wnoutrefresh(status_win);
    doupdate();
}


/*
 * ======================================================================================
 * MACRO-SECTION 5: GAME LOGIC & PHYSICS
 * ======================================================================================
 * Algorithms for collision detection, random generation of entities, and coordinate checks.
 */

int overlaps_target(int x, int y) {
    for (int i = 0; i < num_targets; i++) {
        if (targets[i].x == x && targets[i].y == y) return 1;
    }
    return 0;
}

/*
 * Generates a new position for a specific obstacle index.
 * Ensures no overlap with existing obstacles or targets.
 */
void generate_new_obstacle(int idx, int width, int height) {
    int valid;
    do {
        valid = 1;
        obstacles[idx].x = rand() % (width - 2) + 1;
        obstacles[idx].y = rand() % (height - 2) + 1;

        // Check collision with other obstacles
        for (int i = 0; i < num_obstacles; i++) {
            if (i != idx && obstacles[idx].x == obstacles[i].x && obstacles[idx].y == obstacles[i].y) {
                valid = 0; break;
            }
        }
        // Check collision with targets
        if (valid && overlaps_target(obstacles[idx].x, obstacles[idx].y)) {
            valid = 0;
        }
    } while (!valid);
}

/*
 * Generates a new position for a specific target index.
 * Ensures no overlap with obstacles or other targets.
 */
void generate_new_target(int idx, int width, int height) {
    int valid;
    do {
        valid = 1;
        targets[idx].x = rand() % (width - 2) + 1;
        targets[idx].y = rand() % (height - 2) + 1;

        // Check collision with Obstacles
        for (int i = 0; i < num_obstacles; i++) {
            if (targets[idx].x == obstacles[i].x && targets[idx].y == obstacles[i].y) {
                valid = 0; break;
            }
        }

        // Check collision with other Targets
        if (valid) {
            for (int k = 0; k < num_targets; k++) {
                if (k != idx && targets[k].x == targets[idx].x && targets[k].y == targets[idx].y) {
                    valid = 0; 
                    break;
                }
            }
        }
        
    } while (!valid);

    logMessage(LOG_PATH, "[BB] New target %d position: %d %d", idx, targets[idx].x, targets[idx].y);
}


/*
 * ======================================================================================
 * MACRO-SECTION 6: STATUS BAR & RESIZE HANDLING
 * ======================================================================================
 * Functions to manage the top status bar and handle terminal resize events.
 */

void update_dynamic(float x, float y, float drn_Fx, float drn_Fy, float obst_Fx, float obst_Fy, float wall_Fx, float wall_Fy, float targ_Fx, float targ_Fy)
{
    if (!status_win) return;

    char buffer[256];
    snprintf(buffer, sizeof(buffer),
        "x=%.4f y=%.4f | drn(%.4f %.4f) | obst(%.4f %.4f) | wall(%.4f %.4f) | targ(%.4f %.4f)",
        x, y, drn_Fx, drn_Fy, obst_Fx, obst_Fy, wall_Fx, wall_Fy, targ_Fx, targ_Fy
    );

    // Only update if the text has actually changed
    if (strcmp(buffer, last_status) != 0) {
        strcpy(last_status, buffer);
        werase(status_win);
        mvwprintw(status_win, 0, 0, "%s", buffer);
        wnoutrefresh(status_win);
        doupdate();
    }
}

/*
 * Handles window resizing. 
 * If req_h/req_w are 0, it detects terminal size. Otherwise, it forces a specific size (used in networking).
 */
void reposition_and_redraw(WINDOW **win_ptr, int req_h, int req_w) {
    
    // Auto-detect size if arguments are 0
    if (req_h == 0 || req_w == 0) {
        if (is_term_resized(LINES, COLS)) {
            resize_term(0, 0);
        }
        req_w = COLS;
        req_h = LINES - 1;
    } else {
        // Force specific size (Client syncing with Server)
        resize_term(req_h + 1, req_w);
    }

    int startx = 0;
    int starty = 1;

    if (*win_ptr != NULL) {
        // Attempt resize, recreate if it fails
        if (wresize(*win_ptr, req_h, req_w) == ERR || mvwin(*win_ptr, starty, startx) == ERR) {
            destroy_window(*win_ptr);
            *win_ptr = create_window(req_h, req_w, starty, startx);
            
            wresize(status_win, 1, req_w);
            mvwin(status_win, 0, 0);
        }
    } else {
        *win_ptr = create_window(req_h, req_w, starty, startx);
        status_win = newwin(1, req_w, 0, 0);
    }

    werase(status_win);
    box(*win_ptr, 0, 0);
    redraw_scene(*win_ptr);
    logMessage(LOG_PATH, "[BB] Window Resized to: %dx%d", req_w, req_h);
}


/*
 * ======================================================================================
 * MACRO-SECTION 7: IPC (BROADCASTING & MESSAGING)
 * ======================================================================================
 * Helper functions to package and send messages (Window Size, Positions) 
 * via file descriptors (pipes or sockets).
 */

void send_window_size(WINDOW *win, int fd_drone, int fd_obst, int fd_targ) {
    set_state(STATE_BROADCASTING);
    Message msg;
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    msg.type = MSG_TYPE_SIZE;
    snprintf(msg.data, sizeof(msg.data), "%d %d", max_x, max_y);

    write(fd_drone, &msg, sizeof(msg));
    if(current_mode == MODE_STANDALONE){
        write(fd_obst,  &msg, sizeof(msg));
        write(fd_targ,  &msg, sizeof(msg));
    }
}

void send_window_size_network(WINDOW *win, int fd_network) {
    if (fd_network < 0) return;
    set_state(STATE_BROADCASTING); 
    Message msg;
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);

    msg.type = MSG_TYPE_SIZE;
    snprintf(msg.data, sizeof(msg.data), "%d %d", max_x, max_y);
    write(fd_network, &msg, sizeof(msg));
}

void send_drone_position_network(float x, float y, int fd_network) {
    if (fd_network < 0) return;
    Message net_msg;
    net_msg.type = MSG_TYPE_POSITION; 
    snprintf(net_msg.data, sizeof(net_msg.data), "%f %f", x, y);
    write(fd_network, &net_msg, sizeof(net_msg));
}

void send_resize(WINDOW *win, int fd_drone) {
    set_state(STATE_BROADCASTING);
    Message msg;
    int max_y, max_x;
    getmaxyx(win, max_y, max_x);
    msg.type = MSG_TYPE_SIZE;
    snprintf(msg.data, sizeof(msg.data), "%d %d", max_x, max_y);
    write(fd_drone, &msg, sizeof(msg));
}


/*
 * ======================================================================================
 * MACRO-SECTION 8: MAIN EXECUTION
 * ======================================================================================
 * Entry point. Handles Argument Parsing, Watchdog Synchronization, Ncurses Init,
 * and the main Event Loop (Select Multiplexing).
 */

int main(int argc, char *argv[]) {
    
    // --- ARGUMENT PARSING ---
    if (argc < 14) {
        fprintf(stderr, "[BB] Error: Needed 13 arguments, received %d\n", argc-1);
        return 1;
    }

    int fd_input_read  = atoi(argv[1]);
    int fd_drone_read  = atoi(argv[2]);
    int fd_drone_write = atoi(argv[3]);
    int fd_obst_write  = atoi(argv[4]);
    int fd_obst_read   = atoi(argv[5]);
    int fd_targ_write  = atoi(argv[6]);
    int fd_targ_read   = atoi(argv[7]);
    int fd_wd_write    = atoi(argv[8]);
    current_mode = atoi(argv[9]);
    if (argv[10]) {
        strncpy(server_address, argv[10], sizeof(server_address) - 1);
        server_address[sizeof(server_address) - 1] = '\0';
    }
    int fd_network_write = atoi(argv[11]);
    int fd_network_read = atoi(argv[12]);
    current_role = atoi(argv[13]);

    logMessage(LOG_PATH, "[BB] FDs: input=%d drone=%d obst=%d target=%d wd=%d network=%d", 
    fd_input_read, fd_drone_read, fd_obst_write, fd_targ_write, fd_wd_write, fd_network_read);

    // Ignore SIGPIPE to prevent crash on broken pipes
    signal(SIGPIPE, SIG_IGN);

    // --- WATCHDOG SETUP ---
    struct sigaction sa;
    sa.sa_handler = watchdog_ping_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    sigaction(SIGUSR1, &sa, NULL);

    if (current_mode == MODE_STANDALONE) {
        signal(SIGUSR1, watchdog_ping_handler);
        wait_for_watchdog_pid();
    }

    // Publish PID with file locking
    FILE *fp_pid = fopen(PID_FILE_PATH, "a");
    if (!fp_pid) {
        perror("[BB] Error opening PID file");
        exit(1);
    }

    int fd_pid = fileno(fp_pid);
    flock(fd_pid, LOCK_EX); // Acquire Lock
    publish_my_pid(fp_pid);
    fflush(fp_pid);
    flock(fd_pid, LOCK_UN); // Release Lock
    fclose(fp_pid);

    // --- NCURSES INITIALIZATION ---
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    curs_set(0);
    start_color();
    use_default_colors();
    init_pair(1, COLOR_BLUE, -1);
    init_pair(2, COLOR_RED, -1);
    init_pair(3, COLOR_GREEN, -1);
    refresh();

    // --- WINDOW & PROTOCOL HANDSHAKE ---
    WINDOW *win = NULL;
    status_win = newwin(1, COLS, 0, 0);
    win = create_window(LINES - 1, COLS, 1, 0);
    
    reposition_and_redraw(&win, 0, 0);
    
    // Initial size broadcast
    if (current_mode == MODE_STANDALONE || (current_mode == MODE_NETWORKED && current_role == MODE_SERVER)) {
        send_window_size(win, fd_drone_write, fd_obst_write, fd_targ_write);
    }

    // Network Synchronization Logic
    if (current_mode == MODE_NETWORKED) {
        if (current_role == MODE_SERVER) {
            // Server: Sends dimensions to Client
            send_window_size_network(win, fd_network_write);
        } else {
            // Client: Waits for dimensions from Server
            Message msg;
            ssize_t n = read(fd_network_read, &msg, sizeof(msg));
            if (n > 0 && msg.type == MSG_TYPE_SIZE) {
                int width, height;
                if (sscanf(msg.data, "%d %d", &width, &height) == 2) {
                    
                    // 1. Resize local window to match Server
                    reposition_and_redraw(&win, height, width);
                    
                    // 2. Forward correct size to Local Drone
                    send_window_size(win, fd_drone_write, fd_obst_write, fd_targ_write);
                    
                    logMessage(LOG_PATH, "[BB] Synced size with Server: %dx%d and forwarded to Drone", width, height);
                }
            }
        }
    }

    // Physics variables initialization
    float drn_Fx = 0.0f, drn_Fy = 0.0f;
    float obst_Fx = 0.0f, obst_Fy = 0.0f;
    float wall_Fx = 0.0f, wall_Fy = 0.0f;
    float targ_Fx = 0.0f, targ_Fy = 0.0f;

    logMessage(LOG_PATH, "[BB] Ready and GUI started");

    fd_set readfds;
    struct timeval tv;
    Message msg;

    obstacles = malloc(sizeof(Point)); 
    num_obstacles = 0;
    
    // --- MAIN EVENT LOOP ---
    while (1) {
        set_state(STATE_IDLE); // Reset state before waiting

        // 1. UI Input Handling (Local Keyboard)
        int ch = getch();
        if (ch != ERR) {
            set_state(STATE_PROCESSING_INPUT);
            if (ch == 'q') break;
            if (ch == KEY_RESIZE) {
                reposition_and_redraw(&win, 0, 0);
                send_resize(win, fd_drone_write);
            }
        }

        // 2. Periodic Logic (Obstacle Random Movement)
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        if (current_mode == MODE_STANDALONE && num_obstacles > 0 && now.tv_sec - last_obst_change.tv_sec >= OBSTACLE_PERIOD_SEC) {
            set_state(STATE_UPDATING_MAP); 
            last_obst_change = now;
            
            int idx = rand() % num_obstacles;
            int max_y, max_x;
            getmaxyx(win, max_y, max_x);
            generate_new_obstacle(idx, max_x, max_y);
            
            redraw_scene(win); 

            // Broadcast update
            set_state(STATE_BROADCASTING); 
            Message m;
            m.type = MSG_TYPE_OBSTACLES;
            snprintf(m.data, sizeof(m.data), "%d", num_obstacles);
            write(fd_drone_write, &m, sizeof(m));
            write(fd_drone_write, obstacles, sizeof(Point) * num_obstacles);
        }

        // 3. I/O Multiplexing Configuration (Select)
        FD_ZERO(&readfds);
        FD_SET(fd_input_read, &readfds);
        FD_SET(fd_drone_read, &readfds);
        
        if(current_mode == MODE_STANDALONE){
            FD_SET(fd_obst_read, &readfds);
            FD_SET(fd_targ_read, &readfds); 
        }
        if(current_mode == MODE_NETWORKED){
            FD_SET(fd_network_read, &readfds);
        }
        
        // Calculate max_fd for select()
        int max_fd = fd_input_read;
        if (fd_drone_read > max_fd) max_fd = fd_drone_read;
        if(current_mode == MODE_STANDALONE){
            if (fd_obst_read > max_fd) max_fd = fd_obst_read;
            if (fd_targ_read > max_fd) max_fd = fd_targ_read;
        }
        if(current_mode == MODE_NETWORKED){
            if (fd_network_read > max_fd) max_fd = fd_network_read;
        }
        max_fd += 1;

        tv.tv_sec  = 0;
        tv.tv_usec = 30000; // 30ms Timeout for fluid rendering

        int ret = select(max_fd, &readfds, NULL, NULL, &tv);
        if (ret < 0) {
            if (errno == EINTR) continue;
            break;
        }

        // 4. Input Process Handler
        if (FD_ISSET(fd_input_read, &readfds)) {
            set_state(STATE_PROCESSING_INPUT);
            char buf[80];
            ssize_t n = read(fd_input_read, buf, sizeof(buf)-1);
            if (n > 0) {
                buf[n] = '\0';
                if (buf[0] == 'q'){
                    // Handle Quit Sequence
                    Message quit_msg;
                    quit_msg.type = MSG_TYPE_EXIT;
                    snprintf(msg.data, sizeof(quit_msg), "%s", buf);
                    if(current_mode == MODE_STANDALONE){
                        write(fd_wd_write, &quit_msg, sizeof(Message));
                        write(fd_drone_write, &quit_msg, sizeof(Message));
                        write(fd_obst_write, &quit_msg, sizeof(Message));
                        write(fd_targ_write, &quit_msg, sizeof(Message));
                    }
                    else{
                        write(fd_drone_write, &quit_msg, sizeof(Message));
                        write(fd_network_write, &quit_msg, sizeof(Message));
                    }
                    goto quit;
                }
                logMessage(LOG_PATH_SC, "[BB] Input received: %c", buf[0]);
                
                // Forward keypress to Drone Process
                msg.type = MSG_TYPE_INPUT;
                snprintf(msg.data, sizeof(msg.data), "%s", buf);
                write(fd_drone_write, &msg, sizeof(Message));
            }
        }

        // 5. Network Process Handler
        if(FD_ISSET(fd_network_read, &readfds)){
            if(read(fd_network_read, &msg, sizeof(Message)) > 0){
                switch(msg.type){
                    case MSG_TYPE_DRONE: {
                        // Receiving remote drone position, treating it as an obstacle locally
                        float remote_x, remote_y;
                        if (sscanf(msg.data, "%f %f", &remote_x, &remote_y) == 2) {
                            if (!obstacles) {
                                obstacles = malloc(sizeof(Point));
                            }

                            num_obstacles = 1;
                            obstacles[0].x = (int)remote_x;
                            obstacles[0].y = (int)remote_y;

                            // Clamp values within bounds
                            int max_y, max_x;
                            getmaxyx(win, max_y, max_x);
                            if(obstacles[0].x >= max_x) obstacles[0].x = max_x - 1;
                            if(obstacles[0].y >= max_y - 1) obstacles[0].y = max_y - 2;
                            if(obstacles[0].x < 1) obstacles[0].x = 1;
                            if(obstacles[0].y < 1) obstacles[0].y = 1;

                            // Notify local drone about the "obstacle" (remote drone)
                            Message out_msg;
                            out_msg.type = MSG_TYPE_OBSTACLES;
                            snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_obstacles);
                            write(fd_drone_write, &out_msg, sizeof(Message));
                            write(fd_drone_write, obstacles, sizeof(Point) * num_obstacles);
                            
                            redraw_scene(win);
                        }
                        break;
                    }
                    default: break;
                }
            }
        }


        // 6. Drone Process Handler
        if (FD_ISSET(fd_drone_read, &readfds)) {
            set_state(STATE_UPDATING_MAP);
            if (read(fd_drone_read, &msg, sizeof(msg)) > 0) {
                switch (msg.type) {

                case MSG_TYPE_POSITION:{
                    sscanf(msg.data, "%f %f", &current_x, &current_y);

                    redraw_scene(win); 

                    // Forward position to network if applicable
                    if (current_mode == MODE_NETWORKED) {
                        send_drone_position_network(current_x, current_y, fd_network_write);
                    }
                    
                    // Standalone Mode: Check Collisions with Targets
                    if(current_mode == MODE_STANDALONE){
                        int dx = (int)current_x;
                        int dy = (int)current_y;

                        for (int i = 0; i < num_targets; i++) {
                            if (dx == (int)targets[i].x && dy == (int)targets[i].y) {
                                
                                // Logic for Sequential Target Collection
                                if(i == 0){
                                    logMessage(LOG_PATH, "[BB] Expected target reached");
                                    // Shift array (remove target 0)
                                    for (int j = i; j < num_targets - 1; j++) targets[j] = targets[j + 1];
                                    target_reached++;
                                    num_targets--;
                                    
                                    // Broadcast new target list
                                    set_state(STATE_BROADCASTING);
                                    Message out_msg;
                                    out_msg.type = MSG_TYPE_TARGETS;
                                    snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_targets);
                                    write(fd_drone_write, &out_msg, sizeof(out_msg));
                                    write(fd_drone_write, targets, sizeof(Point) * num_targets);
                                }
                                else if(i != 0){
                                    // Wrong target hit: Respawn it elsewhere
                                    logMessage(LOG_PATH, "[BB] Not expected target reached");
                                    targets[i].x = 0;
                                    targets[i].y = 0;

                                    int max_y, max_x;
                                    getmaxyx(win, max_y, max_x);
                                    generate_new_target(i, max_x, max_y);

                                    set_state(STATE_BROADCASTING);
                                    Message out_msg;
                                    out_msg.type = MSG_TYPE_TARGETS;
                                    snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_targets);
                                    write(fd_drone_write, &out_msg, sizeof(out_msg));
                                    write(fd_drone_write, targets, sizeof(Point) * num_targets);
                                }
                                

                                // Win Condition
                                if (num_targets == 0) {
                                    logMessage(LOG_PATH, "[BB] ALL TARGETS CLEARED");
                                    Message out_msg;
                                    out_msg.type = MSG_TYPE_OBSTACLES;
                                    snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_obstacles);
                                    write(fd_targ_write, &out_msg, sizeof(out_msg));
                                    write(fd_targ_write, obstacles, sizeof(Point) * num_obstacles);
                                }
                                redraw_scene(win);
                                break; 
                            }
                        }
                    }
                    break;
                }

                case MSG_TYPE_FORCE: {
                    // Update force values for the UI status bar
                    if (sscanf(msg.data, "%f %f %f %f %f %f %f %f",
                            &drn_Fx, &drn_Fy, &obst_Fx, &obst_Fy, &wall_Fx, &wall_Fy, &targ_Fx, &targ_Fy) == 8) {
                        update_dynamic(current_x, current_y, drn_Fx, drn_Fy, obst_Fx, obst_Fy, wall_Fx, wall_Fy, targ_Fx, targ_Fy);
                    }
                    break;
                }
                default: break;
                }
            }
        }

        // 7. Obstacle Process Handler
        if (FD_ISSET(fd_obst_read, &readfds)) {
            set_state(STATE_UPDATING_MAP);
            if (read(fd_obst_read, &msg, sizeof(msg)) > 0 && msg.type == MSG_TYPE_OBSTACLES) {
                int count;
                sscanf(msg.data, "%d", &count);
                if (count > 0) {
                    free(obstacles);
                    obstacles = malloc(sizeof(Point) * count);
                    read(fd_obst_read, obstacles, sizeof(Point) * count);
                    num_obstacles = count;
                    
                    logMessage(LOG_PATH, "[BB] received %d obstacles", num_obstacles);
                    
                    // Distribute obstacles to Drone & Target Processes
                    set_state(STATE_BROADCASTING);
                    Message out_msg;
                    out_msg.type = MSG_TYPE_OBSTACLES;
                    snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_obstacles);      
                    
                    write(fd_drone_write, &out_msg, sizeof(out_msg));
                    write(fd_drone_write, obstacles, sizeof(Point) * num_obstacles);
                    
                    write(fd_targ_write, &out_msg, sizeof(out_msg));
                    write(fd_targ_write, obstacles, sizeof(Point) * num_obstacles);
                }
                redraw_scene(win);
            }
        }

        // 8. Target Process Handler
        if (FD_ISSET(fd_targ_read, &readfds)) {
            set_state(STATE_UPDATING_MAP);
            if (read(fd_targ_read, &msg, sizeof(msg)) > 0 && msg.type == MSG_TYPE_TARGETS) {
                int count;
                sscanf(msg.data, "%d", &count);
                if (count > 0) {
                    free(targets);
                    targets = malloc(sizeof(Point) * count);
                    read(fd_targ_read, targets, sizeof(Point) * count);
                    num_targets = count;

                    // Distribute targets to Drone & Obstacle Processes
                    set_state(STATE_BROADCASTING);
                    Message out_msg;
                    out_msg.type = MSG_TYPE_TARGETS;
                    snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_targets);
                    
                    write(fd_drone_write, &out_msg, sizeof(out_msg));
                    write(fd_drone_write, targets, sizeof(Point) * num_targets);
                    
                    write(fd_obst_write, &out_msg, sizeof(out_msg));
                    write(fd_obst_write, targets, sizeof(Point) * num_targets);
                }
                redraw_scene(win);
            }
        }

        usleep(10000); // Brief sleep to prevent 100% CPU usage loop
    }

    // --- CLEANUP ---
    quit:
    destroy_window(win);
    free(obstacles);
    endwin();
    return 0;
}