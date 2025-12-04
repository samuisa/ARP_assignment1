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

/*--------------------------------------------------------------------
  Genera un numero di ostacoli proporzionale alla dimensione della grid
---------------------------------------------------------------------*/
Point* generate_obstacles(int width, int height, int* num_out) {

    int total_cells = (width - 2) * (height - 2);
    int count = (int) round(PERC_OBST * total_cells);
    if (count < 1) count = 1;

    Point* arr = malloc(sizeof(Point) * count);
    if (!arr) {
        logMessage(LOG_PATH, "ERRORE malloc ostacoli: %s", strerror(errno));
        exit(1);
    }

    srand(time(NULL));

    for (int i = 0; i < count; i++) {
        int valid;
        do {
            valid = 1;
            arr[i].x = rand() % (width - 2) + 1;
            arr[i].y = rand() % (height - 2) + 1;
            /* Evita duplicati */
            for (int j = 0; j < i; j++) {
                if (arr[i].x == arr[j].x && arr[i].y == arr[j].y) {
                    valid = 0;
                    break;
                }
            }
        } while (!valid);
    }

    logMessage(LOG_PATH, "[OBST] Generati %d ostacoli per grid %dx%d",
               count, width, height);

    *num_out = count;
    return arr;
}

/*--------------------------------------------------------------------
   PROCESSO PRINCIPALE OBSTACLES
---------------------------------------------------------------------*/
int main(int argc, char *argv[]) {

    if (argc < 3) {
        fprintf(stderr, "Usage: %s <pipe_read_fd> <pipe_write_fd>\n", argv[0]);
        return 1;
    }

    int fd_in  = atoi(argv[1]);   // riceve MSG_TYPE_SIZE
    int fd_out = atoi(argv[2]);   // invia MSG_TYPE_OBSTACLES + array

    logMessage(LOG_PATH, "[OBST] Avviato (fd_in=%d, fd_out=%d)", fd_in, fd_out);


    while (1) {

        fd_set set;
        FD_ZERO(&set);
        FD_SET(fd_in, &set);

        int ret = select(fd_in + 1, &set, NULL, NULL, NULL);
        if (ret < 0) {
            logMessage(LOG_PATH, "[OBST] ERRORE select(): %s", strerror(errno));
            continue;
        }

        if (FD_ISSET(fd_in, &set)) {

            Message msg;
            ssize_t n = read(fd_in, &msg, sizeof(msg));

            if (n == 0) {
                logMessage(LOG_PATH, "[OBST] Pipe chiusa da blackboard, termino.");
                break;
            }

            if (n < 0) {
                logMessage(LOG_PATH, "[OBST] ERRORE read(): %s", strerror(errno));
                continue;
            }

            if (n != sizeof(msg)) {
                logMessage(LOG_PATH,
                           "[OBST] WARNING read parziale (%zd bytes instead of %zu)",
                           n, sizeof(msg));
                continue;
            }

            /*----------------------------------------------------------------
               RICEVO LA DIMENSIONE DELLA FINESTRA
            ----------------------------------------------------------------*/
            if (msg.type == MSG_TYPE_SIZE) {

                int width, height;
                if (sscanf(msg.data, "%d %d", &width, &height) != 2) {
                    logMessage(LOG_PATH,
                               "[OBST] ERRORE: dimensioni finestra non valide: '%s'",
                               msg.data);
                    continue;
                }

                /* Genera ostacoli in base alle dimensioni */
                int num_obst = 0;
                Point* arr = generate_obstacles(width, height, &num_obst);

                /*----------------------------------------------------------------
                   INVIO:  1) messaggio con numero ostacoli
                           2) array raw di struct Point
                ----------------------------------------------------------------*/
                Message out_msg;
                out_msg.type = MSG_TYPE_OBSTACLES;
                snprintf(out_msg.data, sizeof(out_msg.data), "%d", num_obst);

                write(fd_out, &out_msg, sizeof(out_msg));
                write(fd_out, arr, sizeof(Point) * num_obst);

                logMessage(LOG_PATH,
                           "[OBST] Inviati %d ostacoli a blackboard (width=%d height=%d)",
                           num_obst, width, height);

                free(arr);
            }

            /*if(msg.type == MSG_TYPE_POSITION){
                ssize_t n = read(fd_in, msg, sizeof(msg))
                if(n>0){
                    sscanf(msg.data, "%d %d", current_x, current_y);
                    for(int i = 0; i<num_obst; i++){
                        if(current_x == obstacles[i].x || current_y == obstacles[i].y){
                            compute_force(current_x, current_y);
                        }
                    }
                }
            }*/
        }
    }

    close(fd_in);
    close(fd_out);

    logMessage(LOG_PATH, "[OBST] Terminato correttamente.");
    return 0;
}
