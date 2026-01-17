/* * ======================================================================================
 * MACRO-SECTION 1: HEADERS, CONSTANTS, AND GLOBAL STATE
 * ======================================================================================
 * This section defines the core data structures, state enums for the protocol,
 * and global buffers used for non-blocking communication.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/select.h> 
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdarg.h>

#include "app_common.h"
#include "log.h"

#define BUFSZ 1024 

/* * Rotation angle for coordinate transformation. 
 * If non-zero, the view is rotated between Local and Virtual space.
 */
static float alpha = 0.0f;

/* * Network Protocol State Machine.
 * The protocol follows a strict sequence: Command -> Data -> Acknowledgment.
 */
typedef enum {
    // SERVER STATES
    SV_SEND_CMD_DRONE,   // Tell client: "I am sending drone data"
    SV_SEND_DATA_DRONE,  // Send actual coordinates
    SV_WAIT_DOK,         // Wait for Client to acknowledge drone data
    SV_SEND_CMD_OBST,    // Tell client: "Send me your obstacle data"
    SV_WAIT_DATA_OBST,   // Wait for Client to reply with data
    
    // CLIENT STATES
    CL_WAIT_COMMAND,     // Idle, waiting for Server instruction
    CL_WAIT_DRONE_DATA,  // Server said "drone", waiting for coords
    CL_SEND_OBST_DATA,   // Server said "obst", sending my coords
    CL_WAIT_POK          // Wait for Server to acknowledge my data
} NetState;

static NetState net_state;
static int net_fd = -1;

/* * Buffer structure for Non-Blocking I/O.
 * Accumulates partial reads until a full newline-terminated message is found.
 */
typedef struct {
    char data[BUFSZ];
    int len;
} SocketBuffer;

static SocketBuffer sock_buf = { .len = 0 };

/* Cached local positions to be sent over the network */
static float my_last_x = 0.0f;
static float my_last_y = 0.0f;


/* * ======================================================================================
 * MACRO-SECTION 2: UTILITY AND MATHEMATICS
 * ======================================================================================
 * Helper functions for debugging, coordinate transformation, and file descriptor config.
 */

/* Debug helper: Converts State Enum to String */
const char* state_to_str(NetState s) {
    switch(s) {
        case SV_SEND_CMD_DRONE: return "SV_SEND_CMD_DRONE";
        case SV_SEND_DATA_DRONE: return "SV_SEND_DATA_DRONE";
        case SV_WAIT_DOK: return "SV_WAIT_DOK";
        case SV_SEND_CMD_OBST: return "SV_SEND_CMD_OBST";
        case SV_WAIT_DATA_OBST: return "SV_WAIT_DATA_OBST";
        case CL_WAIT_COMMAND: return "CL_WAIT_COMMAND";
        case CL_WAIT_DRONE_DATA: return "CL_WAIT_DRONE_DATA";
        case CL_SEND_OBST_DATA: return "CL_SEND_OBST_DATA";
        case CL_WAIT_POK: return "CL_WAIT_POK";
        default: return "UNKNOWN";
    }
}

/* * Converts Local Coordinates (Screen pixels) to Virtual Coordinates (Shared World).
 * Applies rotation if alpha != 0.
 */
void local_to_virt(float lx, float ly, float *vx, float *vy) { 
    float x = lx;
    float y = ly; 

    if(alpha != 0.0f) {
        float cos_a = cosf(alpha);
        float sin_a = sinf(alpha);
        float temp_x = x * cos_a - y * sin_a;
        float temp_y = x * sin_a + y * cos_a;
        x = temp_x;
        y = temp_y;
    }
    *vx = x;
    *vy = y;
}

/* * Converts Virtual Coordinates (Shared World) back to Local Coordinates (Screen pixels).
 */
void virt_to_local(float vx, float vy, float *lx, float *ly) { 
    float x = vx;
    float y = vy;

    if(alpha != 0.0f) {
        float cos_a = cosf(-alpha);
        float sin_a = sinf(-alpha);
        float temp_x = x * cos_a - y * sin_a;
        float temp_y = x * sin_a + y * cos_a;
        x = temp_x;
        y = temp_y;
    }
    *lx = x;
    *ly = y;
}

/* Sets a file descriptor to Non-Blocking mode used for select() multiplexing */
void set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags != -1) fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    logMessage(LOG_PATH_SC, "[NET] FD %d set to non-blocking", fd);
}


/* * ======================================================================================
 * MACRO-SECTION 3: LOW-LEVEL I/O AND PARSING
 * ======================================================================================
 * Functions handling raw socket reads/writes, ensuring strict newline delimitation.
 */

/* * Formats and sends a message ensuring strict Protocol adherence.
 * The protocol requires every message to end with '\n'.
 */
void send_msg(int fd, const char *fmt, ...) {
    char buf[BUFSZ];
    va_list args;
    
    // 1. Format string into buffer
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf) - 2, fmt, args); // Reserve space for \n and \0
    va_end(args);

    int len = strlen(buf);
    
    // 2. Log raw data before modification
    logMessage(LOG_PATH_SC, "[NET-OUT] Sending raw data: '%s'", buf);

    // 3. FORCE NEWLINE: The protocol relies on \n to detect end of message
    if (len == 0 || buf[len-1] != '\n') {
        buf[len] = '\n';
        buf[len+1] = '\0'; 
        len++;
    }

    // 4. Write to socket
    ssize_t sent = write(fd, buf, len);
    
    if (sent < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EPIPE)
            logMessage(LOG_PATH_SC, "[NET] ERROR sending: %s", strerror(errno));
    }
}

/* * Reads raw bytes from the socket into the persistent buffer `sock_buf`.
 * Returns 1 if data read, 0 if buffer full, -1 if connection closed.
 */
int read_socket_chunk(int fd) {
    if (sock_buf.len >= BUFSZ - 1) {
        logMessage(LOG_PATH_SC, "[NET-ERR] Buffer full! Cannot read more.");
        return 0; 
    }
    
    ssize_t n = read(fd, sock_buf.data + sock_buf.len, BUFSZ - 1 - sock_buf.len);
    if (n > 0) {
        sock_buf.len += n;
        sock_buf.data[sock_buf.len] = '\0';
        return 1; 
    }
    if (n == 0) logMessage(LOG_PATH_SC, "[NET-IN] Connection closed by peer (read 0).");
    return (n == 0) ? -1 : 0;
}

/* * Extracts a single line from `sock_buf` based on the newline delimiter.
 * Returns 1 if a line was found and extracted, 0 otherwise.
 */
int get_line_from_buffer(char *out_line, int max_len) {
    char *newline_ptr = strchr(sock_buf.data, '\n');
    
    if (newline_ptr) {
        // Calculate line length excluding the newline
        int line_len = newline_ptr - sock_buf.data;
        
        if (line_len >= max_len) line_len = max_len - 1;
        
        // Copy the line
        memcpy(out_line, sock_buf.data, line_len);
        out_line[line_len] = '\0'; // Null-terminate for C string safety
        
        logMessage(LOG_PATH_SC, "[NET-PARSE] Extracted line (via \\n): '%s'", out_line);

        // Shift remaining data in buffer to the front
        int remaining = sock_buf.len - (newline_ptr - sock_buf.data) - 1;
        memmove(sock_buf.data, newline_ptr + 1, remaining);
        sock_buf.len = remaining;
        sock_buf.data[sock_buf.len] = '\0';
        return 1;
    }
    return 0;
}

/* * Blocking Read (Used only during initial Handshake).
 * Reads byte-by-byte until a newline is found.
 */
int read_line_blocking(int fd, char *out, int out_sz) {
    int pos = 0; char c;
    while (pos < out_sz - 1) {
        if (read(fd, &c, 1) <= 0) return -1;
        if (c == '\n') break; 
        out[pos++] = c;
    }
    out[pos] = '\0'; 
    logMessage(LOG_PATH_SC, "[HANDSHAKE] Blocking read: '%s'", out);
    return pos;
}


/* * ======================================================================================
 * MACRO-SECTION 4: CONNECTION AND HANDSHAKE
 * ======================================================================================
 * Functions to initialize sockets (Bind/Listen or Connect) and perform the
 * initial strict protocol handshake to sync Client/Server.
 */

int init_server(int port) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1; setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET; a.sin_addr.s_addr = INADDR_ANY; a.sin_port = htons(port);
    
    if (bind(s, (struct sockaddr*)&a, sizeof(a)) < 0) {
        logMessage(LOG_PATH_SC, "[NET-ERR] Bind failed: %s", strerror(errno));
        return -1;
    }
    listen(s, 1);
    logMessage(LOG_PATH_SC, "[NET-SRV] Waiting for connection on port %d...", port);
    
    struct sockaddr_in cli;
    socklen_t len = sizeof(cli);
    int client_fd = accept(s, (struct sockaddr*)&cli, &len);
    if (client_fd >= 0) {
        logMessage(LOG_PATH_SC, "[NET-SRV] Accepted connection from %s", inet_ntoa(cli.sin_addr));
    }
    return client_fd;
}

int init_client(const char *addr, int port) {
    int s = socket(AF_INET, SOCK_STREAM, 0);
    struct sockaddr_in a = {0};
    a.sin_family = AF_INET; a.sin_port = htons(port);
    inet_pton(AF_INET, addr, &a.sin_addr);
    
    logMessage(LOG_PATH_SC, "[NET-CLI] Connecting to %s:%d ...", addr, port);
    while (connect(s, (struct sockaddr*)&a, sizeof(a)) < 0) {
        logMessage(LOG_PATH_SC, "[NET-CLI] Retry in 1s...");
        sleep(1);
    }
    logMessage(LOG_PATH_SC, "[NET-CLI] Connected!");
    return s;
}

/* Helpers for Blackboard Communication */
void send_window_size(int fd_out, int w, int h) {
    Message msg; msg.type = MSG_TYPE_SIZE;
    snprintf(msg.data, sizeof(msg.data), "%d %d", w, h);
    write(fd_out, &msg, sizeof(msg));  
    logMessage(LOG_PATH_SC, "[BB-OUT] Sent Window Size: %d %d", w, h);
}

void receive_window_size(int fd_in, int *w, int *h){
    Message msg;
    if (read(fd_in, &msg, sizeof(msg)) > 0) {
        sscanf(msg.data, "%d %d", w, h);
        logMessage(LOG_PATH_SC, "[BB-IN] Received Window Size: %d %d", *w, *h);
    }
}

void update_local_position(int fd_in) {
    Message msg;
    while (read(fd_in, &msg, sizeof(msg)) > 0) {
        sscanf(msg.data, "%f %f", &my_last_x, &my_last_y);
    }
}

/* * The Handshake Logic:
 * 1. Server sends "ok" -> Client confirms with "ook".
 * 2. Server sends "size W H" -> Client confirms with "sok W H".
 * 3. Client adapts local window size to match Server.
 */
int protocol_handshake(int mode, int fd, int *w, int *h, int fd_bb_out) {
    char buf[BUFSZ];
    logMessage(LOG_PATH_SC, "[HANDSHAKE] Start Mode: %s", mode == MODE_SERVER ? "SERVER" : "CLIENT");
    
    if (mode == MODE_SERVER) {
        send_msg(fd, "ok"); 
        if (read_line_blocking(fd, buf, sizeof(buf)) <= 0 || strcmp(buf, "ook") != 0) {
            logMessage(LOG_PATH_SC, "[HANDSHAKE] Error: Expected 'ook', got '%s'", buf);
            return -1;
        }
        send_msg(fd, "size %d %d", *w, *h); 
        if (read_line_blocking(fd, buf, sizeof(buf)) <= 0 || sscanf(buf, "sok %d %d", w, h) != 2) {
             logMessage(LOG_PATH_SC, "[HANDSHAKE] Error: Expected 'sok', got '%s'", buf);
             return -1;
        }
    } else {
        if (read_line_blocking(fd, buf, sizeof(buf)) <= 0 || strcmp(buf, "ok") != 0) {
            logMessage(LOG_PATH_SC, "[HANDSHAKE] Error: Expected 'ok', got '%s'", buf);
            return -1;
        }
        send_msg(fd, "ook"); 
        if (read_line_blocking(fd, buf, sizeof(buf)) <= 0 || sscanf(buf, "size %d %d", w, h) != 2) {
             logMessage(LOG_PATH_SC, "[HANDSHAKE] Error: Expected 'size', got '%s'", buf);
             return -1;
        }
        send_window_size(fd_bb_out, *w, *h);
        send_msg(fd, "sok %d %d", *w, *h); 
    }
    
    // Set initial state based on Role
    net_state = (mode == MODE_SERVER) ? SV_SEND_CMD_DRONE : CL_WAIT_COMMAND;
    logMessage(LOG_PATH_SC, "[HANDSHAKE] Done. State: %s", state_to_str(net_state));
    return 0;
}


/* * ======================================================================================
 * MACRO-SECTION 5: MAIN LOGIC LOOP (STATE MACHINE)
 * ======================================================================================
 * The core loop that multiplexes between Network I/O and Blackboard I/O using select().
 * Implements the NetState state machine.
 */

void network_loop(int mode, int fd_bb_in, int fd_bb_out) {
    char net_line[BUFSZ];
    
    // Logic Variables
    float rx, ry; 
    float vx, vy; 
    float remote_x, remote_y;
    Message msg; 
    fd_set read_fds;
    struct timeval timeout; 

    set_nonblocking(net_fd);
    set_nonblocking(fd_bb_in);

    while (1) {
        // --- 1. Prepare Select ---
        FD_ZERO(&read_fds);
        FD_SET(net_fd, &read_fds);
        FD_SET(fd_bb_in, &read_fds);
        
        int max_fd = (net_fd > fd_bb_in) ? net_fd : fd_bb_in;
        int has_buf = (strchr(sock_buf.data, '\n') != NULL);
        
        // If we already have a full line in buffer, immediate timeout (0), else wait briefly
        timeout.tv_sec = 0;
        timeout.tv_usec = has_buf ? 0 : 2000; 

        if (select(max_fd + 1, &read_fds, NULL, NULL, &timeout) < 0 && errno != EINTR) {
             logMessage(LOG_PATH_SC, "[NET-ERR] Select failed: %s", strerror(errno));
             break;
        }

        // --- 2. Handle Inputs ---
        
        // Read local position from Blackboard
        if (FD_ISSET(fd_bb_in, &read_fds)) update_local_position(fd_bb_in);

        // Read raw data from Network into buffer
        if (FD_ISSET(net_fd, &read_fds)) {
            if (read_socket_chunk(net_fd) == -1) {
                logMessage(LOG_PATH_SC, "[NET] Socket closed.");
                goto exit_loop;
            }
        }

        // --- 3. Process State Machine ---
        int state_changed;
        do {
            state_changed = 0;
            if (mode == MODE_SERVER) {
                switch (net_state) {
                    case SV_SEND_CMD_DRONE:
                        logMessage(LOG_PATH_SC, "[SV] >> Sending 'drone'");
                        send_msg(net_fd, "drone");
                        net_state = SV_SEND_DATA_DRONE;
                        state_changed = 1; 
                        break;
                    case SV_SEND_DATA_DRONE:
                        // Convert Local to Virtual coords for transmission
                        local_to_virt(my_last_x, my_last_y, &vx, &vy);
                        send_msg(net_fd, "%f %f", vx, vy);
                        net_state = SV_WAIT_DOK;
                        break;
                    case SV_WAIT_DOK:
                        if (get_line_from_buffer(net_line, sizeof(net_line))) {
                            if (sscanf(net_line, "dok %f %f", &rx, &ry) == 2) {
                                logMessage(LOG_PATH_SC, "[SV] << ACK 'dok'");
                                net_state = SV_SEND_CMD_OBST;
                                state_changed = 1;
                            } else if (strcmp(net_line, "q") == 0) goto exit_loop;
                        }
                        break;
                    case SV_SEND_CMD_OBST:
                        send_msg(net_fd, "obst");
                        net_state = SV_WAIT_DATA_OBST;
                        break;
                    case SV_WAIT_DATA_OBST:
                        if (get_line_from_buffer(net_line, sizeof(net_line))) {
                            if (sscanf(net_line, "%f %f", &rx, &ry) == 2) {
                                logMessage(LOG_PATH_SC, "[SV] << Obst Data");
                                msg.type = MSG_TYPE_DRONE;
                                // Convert Remote Virtual -> Local for display
                                virt_to_local(rx, ry, &remote_x, &remote_y);
                                
                                // Forward to Blackboard
                                snprintf(msg.data, sizeof(msg.data), "%f %f", remote_x, remote_y);
                                write(fd_bb_out, &msg, sizeof(msg));
                                
                                send_msg(net_fd, "pok %f %f", rx, ry);
                                net_state = SV_SEND_CMD_DRONE;
                                state_changed = 1; 
                            }
                        }
                        break;
                    default: break; 
                }
            } else { // CLIENT LOGIC
                switch (net_state) {
                    case CL_WAIT_COMMAND:
                        if (get_line_from_buffer(net_line, sizeof(net_line))) {
                            if (strcmp(net_line, "drone") == 0) {
                                net_state = CL_WAIT_DRONE_DATA;
                                state_changed = 1;
                            } else if (strcmp(net_line, "obst") == 0) {
                                net_state = CL_SEND_OBST_DATA;
                                state_changed = 1;
                            } else if (strcmp(net_line, "q") == 0) {
                                send_msg(net_fd, "qok");
                                goto exit_loop;
                            }
                        }
                        break;
                    case CL_WAIT_DRONE_DATA:
                        if (get_line_from_buffer(net_line, sizeof(net_line))) {
                            if (sscanf(net_line, "%f %f", &rx, &ry) == 2) {
                                msg.type = MSG_TYPE_DRONE;
                                // Convert Remote Virtual -> Local for display
                                virt_to_local(rx, ry, &remote_x, &remote_y);
                                
                                // Forward to Blackboard
                                snprintf(msg.data, sizeof(msg.data), "%f %f", remote_x, remote_y);
                                write(fd_bb_out, &msg, sizeof(msg));
                                
                                send_msg(net_fd, "dok %f %f", rx, ry);
                                net_state = CL_WAIT_COMMAND;
                            }
                        }
                        break;
                    case CL_SEND_OBST_DATA:
                        // Convert Local -> Virtual for transmission
                        local_to_virt(my_last_x, my_last_y, &vx, &vy);
                        send_msg(net_fd, "%f %f", vx, vy);
                        net_state = CL_WAIT_POK;
                        break;
                    case CL_WAIT_POK:
                        if (get_line_from_buffer(net_line, sizeof(net_line))) {
                            if (sscanf(net_line, "pok %f %f", &rx, &ry) == 2) {
                                net_state = CL_WAIT_COMMAND;
                                state_changed = 1; 
                            }
                        }
                        break;
                    default: break; 
                }
            }
        } while (state_changed);
    }

exit_loop:
    if (net_fd >= 0) close(net_fd);
    logMessage(LOG_PATH_SC, "[NET] Loop finished.");
}


/* * ======================================================================================
 * MACRO-SECTION 6: MAIN ENTRY POINT
 * ======================================================================================
 * Arguments parsing, Signal setup, and Initialization.
 */

int main(int argc, char *argv[]) {
    signal(SIGPIPE, SIG_IGN); // Prevent crash on broken pipe
    if (argc < 6) return 1;

    // Parse Arguments
    int fd_bb_in = atoi(argv[1]);   
    int fd_bb_out = atoi(argv[2]);  
    int mode = atoi(argv[3]);
    const char *addr = (argc > 4) ? argv[4] : "127.0.0.1";
    int port = atoi(argv[5]);
    int w = 100, h = 100;

    // Initialize Connection
    if (mode == MODE_SERVER) {
        // Server needs the Window Size from Blackboard to send to Client
        receive_window_size(fd_bb_in, &w, &h);
        net_fd = init_server(port);
    } else {
        net_fd = init_client(addr, port);
    }

    // Perform Handshake
    if (net_fd < 0 || protocol_handshake(mode, net_fd, &w, &h, fd_bb_out) < 0) {
        logMessage(LOG_PATH_SC, "[NET-FATAL] Init Failed.");
        return 1;
    }

    // Start Main Loop
    network_loop(mode, fd_bb_in, fd_bb_out);
    return 0;
}