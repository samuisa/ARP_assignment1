#include <ncurses.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "app_common.h"

static int current_x = 1;
static int current_y = 1;
static float Fx = 0.0f;
static float Fy = 0.0f;


int main(int argc, char *argv[]) {
    if(argc < 3){
        fprintf(stderr, "Usage: %s <fd_out> <fd_in>\n", argv[0]);
        return 1;
    }

    int fd_out = atoi(argv[1]);
    int fd_in = atoi(argv[2]);
    int ch;
    int x,y;
    char msg[2];
    Message msg;

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);

    getmaxyx(stdscr, y, x);
    WINDOW *win_legend = newwin(((y/2)-1), ((x/2)-1), 0, 0);
    WINDOW *win_dynamic = newwin(((y/2)-1), ((x/2)-1), ((y/2)+1), ((x/2)+1));

    printw("Controllo drone - Premi q per uscire\n");
    refresh();

    while (1) {
        ch = getch();

        if (ch == ERR) {
            usleep(10000);
            continue;
        }

        msg[0] = (char)ch;
        msg[1] = '\0';

        write(fd_out, msg, 2);

        if (ch == 'q')
            break;

        ssize_t n = read(fd_in, &msg, sizeof(msg));
        if (n > 0) {
            switch (msg.type) {
                case MSG_TYPE_POSITION: {
                    if (sscanf(msg.data, "%d %d", &current_x, &current_y) == 2) {
                        update_dynamic(win_dynamic, current_x, current_y);
                    }
                    break;
                }

                case MSG_TYPE_FORCE: {
                    if (sscanf(msg.data, "%f %f", &Fx, &Fy) == 2) {
                        update_dynamic(win_dynamic, Fx, Fy);
                    }
                    break;
                }

                default:
                    break;
                
            }
            
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            perror("read fd_drone_read");
            break;
        }
    }

    endwin();
    close(fd_out);
    return 0;
}
