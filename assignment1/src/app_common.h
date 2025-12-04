#ifndef APP_COMMON_H
#define APP_COMMON_H

#define MSG_TYPE_SIZE      1
#define MSG_TYPE_OBSTACLES 2
#define MSG_TYPE_INPUT     3
#define MSG_TYPE_EXIT      4
#define MSG_TYPE_DRONE     5
#define MSG_TYPE_POSITION  6
#define MSG_TYPE_OBST_FORCE 7
#define MSG_TYPE_TARGETS   8
#define MSG_TYPE_FORCE     9

#define PERC_OBST          0.005
#define PERC_TARG          0.001

#define LOG_PATH "logs/system.log"

// ----- DRONE DYNAMIC -----

#define M 1
#define K 1
#define DT 0.01
#define MAX_FORCE 10
#define EPSILON 1e-6
#define rho 5
#define eta 0.8

// ----- MODEL STRUCTURES -----

typedef struct {
    int type;
    char data[80];
} Message;

typedef struct {
int x;
int y;
} Point;

typedef struct {
    float x, y;
    float x_1, x_2;
    float y_1, y_2;
    float Fx, Fy;
} Drone;

#endif