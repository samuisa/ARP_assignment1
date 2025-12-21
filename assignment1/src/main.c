#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include "log.h"
#include "app_common.h"

/* ========================================================= */

void ensureLogsDir(void) {
    struct stat st;

    if (stat("logs", &st) == -1) {
        if (mkdir("logs", 0700) == -1) {
            perror("mkdir logs");
            exit(1);
        }
    }

    FILE *logf = fopen(LOG_PATH, "w");
    if (!logf) {
        perror("fopen log file");
        exit(1);
    }
    fclose(logf);
}

/* ========================================================= */

int main(void) {

    ensureLogsDir();
    logMessage(LOG_PATH, "[MAIN] PROGRAM STARTED");

    /* ================= PIPE APPLICATIVE ================= */

    int pipe_input_blackboard[2];
    int pipe_blackboard_drone[2];
    int pipe_drone_blackboard[2];
    int pipe_blackboard_obstacle[2];
    int pipe_obstacle_blackboard[2];
    int pipe_blackboard_target[2];
    int pipe_target_blackboard[2];

    if (pipe(pipe_input_blackboard) == -1 ||
        pipe(pipe_blackboard_drone) == -1 ||
        pipe(pipe_drone_blackboard) == -1 ||
        pipe(pipe_blackboard_obstacle) == -1 ||
        pipe(pipe_obstacle_blackboard) == -1 ||
        pipe(pipe_blackboard_target) == -1 ||
        pipe(pipe_target_blackboard) == -1) {

        perror("pipe");
        logMessage(LOG_PATH, "[MAIN] ERROR creating pipes");
        exit(1);
    }

    logMessage(LOG_PATH, "[MAIN] Pipes created successfully");

    pid_t pid_watchdog = fork();

    if (pid_watchdog == 0) {

        execlp("./exec/watchdog", "./exec/watchdog", NULL);

        perror("exec watchdog");
        exit(1);
    }

    logMessage(LOG_PATH, "[MAIN] Watchdog started (pid=%d)", pid_watchdog);


    /* ================= PROCESSO INPUT ================= */

    pid_t pid_input = fork();
    if (pid_input == 0) {

        logMessage(LOG_PATH, "[MAIN] Input started (pid=%d)", getpid());

        close(pipe_input_blackboard[0]);

        close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
        close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
        close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
        close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

        char fd_out_str[16];
        char p_watchdog[16];

        snprintf(fd_out_str, sizeof(fd_out_str), "%d", pipe_input_blackboard[1]);
        snprintf(p_watchdog,  sizeof(p_watchdog),  "%d", pid_watchdog);


        execlp("konsole", "konsole", "-e",
               "./exec/input", fd_out_str, 
               p_watchdog, "WATCHDOG", NULL);

        perror("exec input");
        exit(1);
    }

    /* ================= PROCESSO OBSTACLE ================= */

    pid_t pid_obst = fork();
    if (pid_obst == 0) {

        logMessage(LOG_PATH, "[MAIN] Obstacle started (pid=%d)", getpid());

        close(pipe_blackboard_obstacle[1]);

        close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
        close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
        close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

        char fd_in_str[16], fd_out_str[16];
        char p_watchdog[16];

        snprintf(fd_in_str, sizeof(fd_in_str), "%d", pipe_blackboard_obstacle[0]);
        snprintf(fd_out_str, sizeof(fd_out_str), "%d", pipe_obstacle_blackboard[1]);
        snprintf(p_watchdog,  sizeof(p_watchdog),  "%d", pid_watchdog);

        execlp("./exec/obstacle", "./exec/obstacle",
               fd_in_str, fd_out_str,
               p_watchdog, "WATCHDOG", NULL);

        perror("exec obstacle");
        exit(1);
    }

    /* ================= PROCESSO TARGET ================= */

    pid_t pid_target = fork();
    if (pid_target == 0) {

        logMessage(LOG_PATH, "[MAIN] Target started (pid=%d)", getpid());

        close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]);

        close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
        close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
        close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
        close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);

        char fd_in_str[16], fd_out_str[16];
        char p_watchdog[16];
        snprintf(fd_in_str, sizeof(fd_in_str), "%d", pipe_blackboard_target[0]);
        snprintf(fd_out_str, sizeof(fd_out_str), "%d", pipe_target_blackboard[1]);
        snprintf(p_watchdog,  sizeof(p_watchdog),  "%d", pid_watchdog);

        execlp("./exec/target", "./exec/target",
               fd_in_str, fd_out_str, 
               p_watchdog, "WATCHDOG", NULL);

        perror("exec target");
        exit(1);
    }

    /* ================= PROCESSO BLACKBOARD ================= */

    pid_t pid_blackboard = fork();
    if (pid_blackboard == 0) {

        logMessage(LOG_PATH, "[MAIN] Blackboard started (pid=%d)", getpid());

        close(pipe_input_blackboard[1]);
        close(pipe_drone_blackboard[1]);
        close(pipe_blackboard_drone[0]);
        close(pipe_blackboard_obstacle[0]);
        close(pipe_obstacle_blackboard[1]);
        close(pipe_blackboard_target[0]);
        close(pipe_target_blackboard[1]);

        char fd_in_input[16], fd_in_drone[16];
        char fd_out_drone[16], fd_out_obst[16], fd_out_target[16];
        char fd_in_obst[16], fd_in_target[16];
        char p_watchdog[16];

        snprintf(fd_in_input, sizeof(fd_in_input), "%d", pipe_input_blackboard[0]);
        snprintf(fd_in_drone, sizeof(fd_in_drone), "%d", pipe_drone_blackboard[0]);
        snprintf(fd_out_drone, sizeof(fd_out_drone), "%d", pipe_blackboard_drone[1]);
        snprintf(fd_out_obst, sizeof(fd_out_obst), "%d", pipe_blackboard_obstacle[1]);
        snprintf(fd_in_obst, sizeof(fd_in_obst), "%d", pipe_obstacle_blackboard[0]);
        snprintf(fd_out_target, sizeof(fd_out_target), "%d", pipe_blackboard_target[1]);
        snprintf(fd_in_target, sizeof(fd_in_target), "%d", pipe_target_blackboard[0]);
        snprintf(p_watchdog,  sizeof(p_watchdog),  "%d", pid_watchdog);

        execlp("konsole", "konsole", "-e",
               "./exec/blackboard",
               fd_in_input, fd_in_drone,
               fd_out_drone, fd_out_obst,
               fd_in_obst, fd_out_target,
               fd_in_target, 
               p_watchdog, "WATCHDOG", NULL);

        perror("exec blackboard");
        exit(1);
    }

    /* ================= PROCESSO DRONE ================= */

    pid_t pid_drone = fork();
    if (pid_drone == 0) {

        logMessage(LOG_PATH, "[MAIN] Drone started (pid=%d)", getpid());

        close(pipe_blackboard_drone[1]);
        close(pipe_drone_blackboard[0]);

        close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
        close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
        close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
        close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
        close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

        char fd_in_str[16], fd_out_str[16];
        char p_watchdog[16];
        snprintf(fd_in_str, sizeof(fd_in_str), "%d", pipe_blackboard_drone[0]);
        snprintf(fd_out_str, sizeof(fd_out_str), "%d", pipe_drone_blackboard[1]);
        snprintf(p_watchdog,  sizeof(p_watchdog),  "%d", pid_watchdog);

        execlp("./exec/drone", "./exec/drone",
               fd_in_str, fd_out_str, 
               p_watchdog, "WATCHDOG", NULL);

        perror("exec drone");
        exit(1);

    }


    union sigval v;

    v.sival_int = pid_input;
    sigqueue(pid_watchdog, SIGUSR2, v);

    v.sival_int = pid_drone;
    sigqueue(pid_watchdog, SIGUSR2, v);

    v.sival_int = pid_blackboard;
    sigqueue(pid_watchdog, SIGUSR2, v);

    v.sival_int = pid_obst;
    sigqueue(pid_watchdog, SIGUSR2, v);

    v.sival_int = pid_target;
    sigqueue(pid_watchdog, SIGUSR2, v);




    /* ================= PARENT ================= */

    close(pipe_input_blackboard[0]); close(pipe_input_blackboard[1]);
    close(pipe_blackboard_drone[0]); close(pipe_blackboard_drone[1]);
    close(pipe_drone_blackboard[0]); close(pipe_drone_blackboard[1]);
    close(pipe_blackboard_obstacle[0]); close(pipe_blackboard_obstacle[1]);
    close(pipe_obstacle_blackboard[0]); close(pipe_obstacle_blackboard[1]);
    close(pipe_blackboard_target[0]); close(pipe_blackboard_target[1]);
    close(pipe_target_blackboard[0]); close(pipe_target_blackboard[1]);

    logMessage(LOG_PATH,
        "[MAIN] All processes running (input=%d drone=%d bb=%d obst=%d targ=%d wd=%d)",
        pid_input, pid_drone, pid_blackboard, pid_obst, pid_target, pid_watchdog);

    int status;
    pid_t wpid;
    while ((wpid = wait(&status)) > 0) {
        logMessage(LOG_PATH,
                   "[MAIN] Child pid=%d exited (status=%d)", wpid, status);
    }

    logMessage(LOG_PATH, "[MAIN] PROGRAM EXIT");
    return 0;
}
