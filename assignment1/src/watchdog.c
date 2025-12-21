#include <ncurses.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include "app_blackboard.h"
#include "app_common.h"
#include "log.h"

int main(int argc, char *argv[]) {

    if (argc < 11) {
        fprintf(stderr, "Usage: %s <pipe_fd_input_write> <pipe_fd_input_read> <fd_drone_write> <fd_drone_read> <fd_blackboard_write> <fd_blackboard_read> <fd_obst_write> <fd_obst_read> <fd_targ_read> <fd_targ_write>\n", argv[0]);
        return 1;
    }

    int fd_input_write  = atoi(argv[1]);
    int fd_input_read  = atoi(argv[2]);
    int fd_drone_write = atoi(argv[3]);
    int fd_drone_read  = atoi(argv[4]);
    int fd_blackboard_write  = atoi(argv[5]);
    int fd_blackboard_read  = atoi(argv[6]);
    int fd_obst_write  = atoi(argv[7]);
    int fd_obst_read   = atoi(argv[8]);
    int fd_targ_write  = atoi(argv[9]);
    int fd_targ_read   = atoi(argv[10]);


}