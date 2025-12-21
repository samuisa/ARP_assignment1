#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include "log.h"

#define LOG_PATH "logs/watchdog.log"

#define MAX_PROC 10
#define T 5   // secondi

typedef struct {
    pid_t pid;
    bool responded;
    char name[32];
} watched_proc_t;

watched_proc_t procs[MAX_PROC];
int num_procs = 0;

FILE *logfile;

/* ================= SIGNAL HANDLER ================= */

void sigusr1_handler(int sig, siginfo_t *info, void *ctx) {
    pid_t sender = info->si_pid;

    for (int i = 0; i < num_procs; i++) {
        if (procs[i].pid == sender) {
            procs[i].responded = true;

            time_t now = time(NULL);
            logMessage(LOG_PATH,
                    "[%ld] Watchdog: risposta da %s (pid=%d)\n",
                    now, procs[i].name, sender);
            //fflush(LOG_PATH);
            return;
        }
    }
}

/* ================= MAIN ================= */

int main(int argc, char *argv[]) {

    if (argc < 3 || argc % 2 == 0) {
        fprintf(stderr,
                "Usage: %s <pid name> [<pid name> ...]\n",
                argv[0]);
        exit(EXIT_FAILURE);
    }

    /* Apri file di log */
    logfile = fopen("logs/watchdog.log", "w");
    if (!logfile) {
        perror("fopen");
        exit(EXIT_FAILURE);
    }

    /* Registra processi */
    for (int i = 1; i < argc; i += 2) {
        procs[num_procs].pid = atoi(argv[i]);
        strncpy(procs[num_procs].name, argv[i + 1], 31);
        procs[num_procs].responded = false;
        num_procs++;
    }

    /* Installa handler SIGUSR1 */
    struct sigaction sa;
    sa.sa_sigaction = sigusr1_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_SIGINFO;
    sigaction(SIGUSR1, &sa, NULL);

    logMessage(LOG_PATH, "Watchdog avviato\n");
    //fflush(logfile);

    /* ================= CICLO WATCHDOG ================= */

    while (1) {

        logMessage(LOG_PATH, "PORCODIO");

        /* reset stato */
        for (int i = 0; i < num_procs; i++)
            procs[i].responded = false;

        sleep(T);

        /* verifica risposte */
        for (int i = 0; i < num_procs; i++) {
            if (!procs[i].responded) {
                time_t now = time(NULL);
                logMessage(LOG_PATH,
                        "[%ld] ALERT: processo %s (pid=%d) NON responsivo\n",
                        now, procs[i].name, procs[i].pid);
                //fflush(logfile);
            }
        }
    }
}
