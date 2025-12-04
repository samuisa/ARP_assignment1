#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <sys/select.h>

#include "app_common.h"
#include "log.h"

static Point *obstacles = NULL;
static int num_obstacles = 0;

/*--------------------------------------------------------------------
  Genera un numero di ostacoli proporzionale alla dimensione della grid
---------------------------------------------------------------------*/
Point* generate_targets(int width, int height,
                        Point* obstacles, int num_obstacles,
                        int* num_out) {

    int total_cells = (width - 2) * (height - 2);
    int count = (int) round(PERC_TARG * total_cells);
    if (count < 1) count = 1;

    Point* arr = malloc(sizeof(Point) * count);
    if (!arr) {
        logMessage(LOG_PATH, "ERRORE malloc target: %s", strerror(errno));
        exit(1);
    }

    srand(time(NULL));

    for (int i = 0; i < count; i++) {
        int valid;
        do {
            valid = 1;
            arr[i].x = rand() % (width - 2) + 1;
            arr[i].y = rand() % (height - 2) + 1;

            // evita sovrapposizioni tra target
            for (int j = 0; j < i; j++) {
                if (arr[i].x == arr[j].x && arr[i].y == arr[j].y) {
                    valid = 0;
                    break;
                }
            }

            // evita sovrapposizioni con ostacoli
            for (int j = 0; j < num_obstacles && valid; j++) {
                if (arr[i].x == obstacles[j].x &&
                    arr[i].y == obstacles[j].y) {
                    valid = 0;
                    break;
                }
            }

        } while (!valid);
    }

    logMessage(LOG_PATH, "[TARG] Generati %d target per grid %dx%d",
               count, width, height);

    *num_out = count;
    return arr;
}

/*--------------------------------------------------------------------
   PROCESSO PRINCIPALE TARGETS
---------------------------------------------------------------------*/
int main(int argc, char *argv[]) {

    if (argc < 3) {
        fprintf(stderr, "Usage: %s <pipe_read_fd> <pipe_write_fd>\n", argv[0]);
        return 1;
    }

    int fd_in  = atoi(argv[1]);
    int fd_out = atoi(argv[2]);

    logMessage(LOG_PATH, "[TARG] Avviato (fd_in=%d, fd_out=%d)", fd_in, fd_out);

    int win_width = 0;
    int win_height = 0;


    while (1) {

        fd_set set;
        FD_ZERO(&set);
        FD_SET(fd_in, &set);

        int ret = select(fd_in + 1, &set, NULL, NULL, NULL);
        if (ret < 0) {
            logMessage(LOG_PATH, "[TARG] ERRORE select(): %s", strerror(errno));
            continue;
        }

        if (FD_ISSET(fd_in, &set)) {

            Message msg;
            ssize_t n = read(fd_in, &msg, sizeof(msg));

            if (n == 0) {
                logMessage(LOG_PATH, "[TARG] Pipe chiusa da blackboard, termino.");
                break;
            }

            if (n < 0) {
                logMessage(LOG_PATH, "[TARG] ERRORE read(): %s", strerror(errno));
                continue;
            }

            if (n != sizeof(msg)) {
                logMessage(LOG_PATH, "[TARG] WARNING read parziale (%zd bytes instead of %zu)", n, sizeof(msg));
                continue;
            }

            /*----------------------------------------------------------------
               RICEVO LA DIMENSIONE DELLA FINESTRA
            ----------------------------------------------------------------*/
            if (msg.type == MSG_TYPE_SIZE) {

                if (sscanf(msg.data, "%d %d", &win_width, &win_height) != 2) {
                    logMessage(LOG_PATH, "[TARG] ERRORE: dimensioni finestra non valide: '%s'", msg.data);
                    continue;
                }

                logMessage(LOG_PATH, "[TARG] Ricevute dimensioni finestra %dx%d", win_width, win_height);
            }

            if (msg.type == MSG_TYPE_OBSTACLES){

                int count = 0;
                if (sscanf(msg.data, "%d", &count) == 1 && count > 0) {

                    // libera eventuali ostacoli precedenti
                    if (obstacles != NULL) {
                        free(obstacles);
                        obstacles = NULL;
                    }

                    obstacles = malloc(sizeof(Point) * count);
                    if (!obstacles) {
                        logMessage(LOG_PATH, "[TARG] ERRORE allocazione ostacoli");
                        break;
                    }

                    ssize_t n_read = read(fd_in, obstacles, sizeof(Point) * count);
                    if ((size_t)n_read != sizeof(Point) * (size_t)count) {

                        logMessage(LOG_PATH, "[TARG] ERRORE lettura ostacoli (%zd bytes)", n_read);

                        free(obstacles);
                        obstacles = NULL;
                        num_obstacles = 0;
                        continue;
                    }

                    num_obstacles = count;

                    logMessage(LOG_PATH, "[TARG] Ricevuti %d ostacoli", num_obstacles);

                    // -------------------------------------------------
                    // ORA che ho sia finestra che ostacoli → genero target
                    // -------------------------------------------------
                    if (win_width > 0 && win_height > 0) {

                        int num_targ = 0;
                        Point* arr = generate_targets(
                            win_width, win_height,
                            obstacles, num_obstacles,
                            &num_targ
                        );

                        Message out_msg;
                        out_msg.type = MSG_TYPE_TARGETS;
                        snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_targ);

                        write(fd_out, &out_msg, sizeof(out_msg));
                        write(fd_out, arr, sizeof(Point) * num_targ);

                        logMessage(LOG_PATH, "[TARG] Inviati %d target a blackboard", num_targ);

                        free(arr);
                    } else {
                        logMessage(LOG_PATH, "[TARG] WARNING: dimensioni finestra non ancora ricevute");
                    }
                }
            }
        }
    }

    close(fd_in);
    close(fd_out);

    logMessage(LOG_PATH, "[TARG] Terminato correttamente.");
    return 0;
}
