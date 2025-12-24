#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include "log.h"
#include "app_common.h"

#define LOG_PATH_WD "logs/watchdog.log"
#define MAX_PROC 5
#define WD_TIMEOUT_SEC 4

typedef struct {
    pid_t pid;
    char name[32];
    struct timespec last_hb;
    int timeout_sec;
} watched_proc_t;

static watched_proc_t procs[MAX_PROC];
static int num_procs = 0;

static volatile sig_atomic_t bb_alive = 0;
static struct timespec last_bb_heartbeat;

void heartbeat_handler(int sig, siginfo_t *info, void *ctx) {
    (void)sig;
    (void)ctx;

    pid_t sender = info->si_pid;

    for (int i = 0; i < num_procs; i++) {
        if (procs[i].pid == sender) {
            clock_gettime(CLOCK_MONOTONIC, &procs[i].last_hb);
            return;
        }
    }
}


pid_t receive_pid(int fd_read){
    Message msg;
    ssize_t n;
    pid_t pid = -1;

    do {
        n = read(fd_read, &msg, sizeof(msg));
    } while(n < 0 && errno == EINTR);

    if(n <= 0){
        perror("[WD] read watchdog PID");
        exit(1);
    }

    if(msg.type == MSG_TYPE_PID){
        sscanf(msg.data, "%d", &pid);
        //logMessage(LOG_PATH, "[WD] Watchdog PID received: %d", pid_wd);
    } else {
        //logMessage(LOG_PATH, "[WD] Unexpected message type from watchdog: %d", msg.type);
    }

    return pid;
}

void send_pid(int fd_write){
    pid_t pid = getpid();
    Message msg;
    msg.type = MSG_TYPE_PID;
    snprintf(msg.data, sizeof(msg.data), "%d", pid);
    write(fd_write, &msg, sizeof(msg));
}

/* ================= MAIN WATCHDOG ================= */
int main(int argc, char *argv[]) {

    if(argc < 10) {
        fprintf(stderr, "Usage: %s <fd_proc_read> [<fd_proc_read> ...]\n", argv[0]);
        exit(EXIT_FAILURE);
    }

    int fd_input_read  = atoi(argv[1]);
    int fd_drone_read  = atoi(argv[2]);
    int fd_bb_read     = atoi(argv[3]);
    int fd_obst_read   = atoi(argv[4]);
    int fd_targ_read   = atoi(argv[5]);

    int fd_input_write = atoi(argv[6]);
    int fd_drone_write = atoi(argv[7]);
    int fd_bb_write    = atoi(argv[8]);
    int fd_obst_write  = atoi(argv[9]);
    int fd_targ_write  = atoi(argv[10]);

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = heartbeat_handler;
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    sigemptyset(&sa.sa_mask);

    if (sigaction(SIGUSR1, &sa, NULL) < 0) {
        perror("sigaction");
        exit(1);
    }



    pid_t pid_input = receive_pid(fd_input_read);
        logMessage(LOG_PATH_WD, "[WD] Input process pid received: %d", pid_input);

    pid_t pid_drone = receive_pid(fd_drone_read);
        logMessage(LOG_PATH_WD, "[WD] Drone process pid received: %d", pid_drone);

    pid_t pid_bb    = receive_pid(fd_bb_read);
        logMessage(LOG_PATH_WD, "[WD] Blackboard process pid received: %d", pid_bb);

    pid_t pid_obst  = receive_pid(fd_obst_read);
        logMessage(LOG_PATH_WD, "[WD] Obstacle process pid received: %d", pid_obst);

    pid_t pid_targ  = receive_pid(fd_targ_read);
        logMessage(LOG_PATH_WD, "[WD] Target process pid received: %d", pid_targ);

    clock_gettime(CLOCK_MONOTONIC, &procs[0].last_hb);
    procs[0] = (watched_proc_t){ pid_input, "input", procs[0].last_hb, 5 };

    clock_gettime(CLOCK_MONOTONIC, &procs[1].last_hb);
    procs[1] = (watched_proc_t){ pid_drone, "drone", procs[1].last_hb, 2 };

    clock_gettime(CLOCK_MONOTONIC, &procs[2].last_hb);
    procs[2] = (watched_proc_t){ pid_bb, "blackboard", procs[2].last_hb, 3 };

    clock_gettime(CLOCK_MONOTONIC, &procs[3].last_hb);
    procs[3] = (watched_proc_t){ pid_obst, "obstacle", procs[3].last_hb, 4 };

    clock_gettime(CLOCK_MONOTONIC, &procs[4].last_hb);
    procs[4] = (watched_proc_t){ pid_targ, "target", procs[4].last_hb, 6 };

    num_procs = MAX_PROC;


    send_pid(fd_input_write);
        logMessage(LOG_PATH_WD, "[WD] PID sent to input process: %d", getpid());

    send_pid(fd_drone_write);
        logMessage(LOG_PATH_WD, "[WD] PID sent to drone process: %d", getpid());

    send_pid(fd_bb_write);
        logMessage(LOG_PATH_WD, "[WD] PID sent to blackboard process: %d", getpid());

    send_pid(fd_obst_write);
        logMessage(LOG_PATH_WD, "[WD] PID sent to obstacle process: %d", getpid());

    send_pid(fd_targ_write);
        logMessage(LOG_PATH_WD, "[WD] PID sent to target process: %d", getpid());


    clock_gettime(CLOCK_MONOTONIC, &last_bb_heartbeat);

    while (1) {
        sleep(1);

        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);

        for (int i = 0; i < num_procs; i++) {

            long diff = now.tv_sec - procs[i].last_hb.tv_sec;

            logMessage(LOG_PATH_WD,
                    "[WD] SIGNAL RECEIVED BY! %s (pid=%d) last hb %ld sec ago",
                    procs[i].name, procs[i].pid, diff);

            if (diff > procs[i].timeout_sec) {

                logMessage(LOG_PATH_WD,
                    "[WD] TIMEOUT! %s (pid=%d) last hb %ld sec ago",
                    procs[i].name, procs[i].pid, diff);

                /* KILL TUTTI */
                for (int j = 0; j < num_procs; j++) {
                    logMessage(LOG_PATH_WD,
                        "[WD] Killing %s (pid=%d)",
                        procs[j].name, procs[j].pid);
                    kill(procs[j].pid, SIGKILL);
                }

                exit(1);
            }
        }

    }


    return 0;
}
