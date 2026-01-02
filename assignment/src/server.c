#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
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
    int sockfd, newsockfd;
    socklen_t clilen;
    char buffer[256];

    struct sockaddr_in serv_addr, cli_addr;

    int bbfd, clfd;

    char size_msg[256];

    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    bbfd = accept(sockfd, NULL, NULL);   // BLACKBOARD
    clfd = accept(sockfd, NULL, NULL);   // CLIENT

    bzero(&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(5000);

    bind(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
    listen(sockfd, 5);

    printf("Server pronto\n");

    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd, (struct sockaddr *)&cli_addr, &clilen);

    /* HANDSHAKE */
    send_msg(newsockfd, "ok\n");
    recv_msg(newsockfd, buffer);   // ook

    /* SIZE */
    recv_msg(bbfd, size_msg);
    send_msg(clfd, size_msg);
    recv_msg(clfd, buffer);

    /* LOOP */
    while (1) {
        /* DRONE */
        send_msg(newsockfd, "drone\n");
        send_msg(newsockfd, "10 20\n");
        recv_msg(newsockfd, buffer);  // dok

        /* OBSTACLE */
        send_msg(newsockfd, "obst\n");
        recv_msg(newsockfd, buffer);  // x y
        send_msg(newsockfd, "pok\n");

        /* QUIT */
        send_msg(newsockfd, "q\n");
        recv_msg(newsockfd, buffer);  // qok
        break;
    }

    close(newsockfd);
    close(sockfd);
    return 0;
}
