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

static watched_proc_t procs[MAX_PROC];
static int num_procs = 0;

/* aggiornata SOLO dal signal handler */
static volatile sig_atomic_t last_sender = -1;

/* ================= SIGNAL HANDLER ================= */

static void sigusr1_handler(int sig, siginfo_t *info, void *ctx) {
    (void)sig;
    (void)ctx;

    if (info) {
        last_sender = info->si_pid;
    }
}

/* ================= MAIN ================= */

int main(int argc, char *argv[]) {

    /* LOG DI AVVIO GARANTITO */
    logMessage(LOG_PATH, "Watchdog process started (argc=%d)", argc);

    if (argc < 5) {
        logMessage(LOG_PATH,
                   "ERRORE: argomenti non validi. Usage: %s <pid name> [<pid name> ...]",
                   argv[0]);
        fprintf(stderr,
                "Usage: %s <pid name> [<pid name> ...]\n",
                argv[0]);
        exit(EXIT_FAILURE);
    }

    /* Registra processi */
    for (int i = 1; i < argc && num_procs < MAX_PROC; i += 2) {
        procs[num_procs].pid = atoi(argv[i]);

        if (procs[num_procs].pid <= 0) {
            logMessage(LOG_PATH,
                       "PID non valido: '%s'", argv[i]);
            continue;
        }

        strncpy(procs[num_procs].name,
                argv[i + 1],
                sizeof(procs[num_procs].name) - 1);
        procs[num_procs].name[sizeof(procs[num_procs].name) - 1] = '\0';
        procs[num_procs].responded = false;

        logMessage(LOG_PATH,
                   "Registrato processo: %s (pid=%d)",
                   procs[num_procs].name,
                   procs[num_procs].pid);

        num_procs++;
    }

    if (num_procs == 0) {
        logMessage(LOG_PATH, "NESSUN processo valido da monitorare. Uscita.");
        exit(EXIT_FAILURE);
    }

    /* Installa handler SIGUSR1 */
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = sigusr1_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_SIGINFO;

    if (sigaction(SIGUSR1, &sa, NULL) == -1) {
        logMessage(LOG_PATH,
                   "ERRORE sigaction(SIGUSR1): %s", strerror(errno));
        exit(EXIT_FAILURE);
    }

    logMessage(LOG_PATH,
               "Watchdog avviato correttamente (%d processi monitorati)",
               num_procs);

    /* ================= CICLO WATCHDOG ================= */

    while (1) {

            logMessage(LOG_PATH, "PORCODIO");


        /* Reset stato */
        for (int i = 0; i < num_procs; i++)
            procs[i].responded = false;

        sleep(T);

        /* Gestione risposta */
        if (last_sender != -1) {
            bool found = false;
            for (int i = 0; i < num_procs; i++) {
                if (procs[i].pid == last_sender) {
                    procs[i].responded = true;
                    logMessage(LOG_PATH,
                               "Heartbeat ricevuto da %s (pid=%d)",
                               procs[i].name,
                               last_sender);
                    found = true;
                    break;
                }
            }

            if (!found) {
                logMessage(LOG_PATH,
                           "SIGUSR1 ricevuto da PID sconosciuto (%d)",
                           last_sender);
            }

            last_sender = -1;
        }

        /* Verifica processi non responsivi */
        for (int i = 0; i < num_procs; i++) {
            if (!procs[i].responded) {
                logMessage(LOG_PATH,
                           "ALERT: processo %s (pid=%d) NON responsivo",
                           procs[i].name,
                           procs[i].pid);
            }
        }
    }
}
