#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>

void error(char *msg){
    perror(msg);
    exit(1);
}

void send_msg(int fd, const char *msg) {
    write(fd, msg, strlen(msg));
}

void recv_msg(int fd, char *buf) {
    bzero(buf, 256);
    read(fd, buf, 255);
}

int main(int argc, char *argv[]){
    int sockfd;
    struct sockaddr_in serv_addr;
    struct hostent *server;
    char buffer[256];

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    server = gethostbyname("localhost");

    bzero(&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    memcpy(&serv_addr.sin_addr.s_addr,
           server->h_addr,
           server->h_length);
    serv_addr.sin_port = htons(5000);

    connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));

    /* HANDSHAKE */
    recv_msg(sockfd, buffer);    // ok
    send_msg(sockfd, "ook\n");

    /* SIZE */
    recv_msg(sockfd, buffer);    // size l h
    send_msg(sockfd, "sok\n");

    /* LOOP */
    while (1) {
        /* DRONE */
        recv_msg(sockfd, buffer);  // drone
        recv_msg(sockfd, buffer);  // x y
        send_msg(sockfd, "dok\n");

        /* OBSTACLE */
        recv_msg(sockfd, buffer);  // obst
        send_msg(sockfd, "30 40\n");
        recv_msg(sockfd, buffer);  // pok

        /* QUIT */
        recv_msg(sockfd, buffer);  // q
        send_msg(sockfd, "qok\n");
        break;
    }

    close(sockfd);
    return 0;
}
