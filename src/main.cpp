#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>

#include "egm.pb.h"

#define PORT 6510
#define BUFFER_SIZE 2048

int main() {
    int sockfd;
    char buffer[BUFFER_SIZE];
    struct sockaddr_in servaddr, cliaddr;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation failed");
        return -1;
    }

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        perror("Bind failed");
        return -1;
    }

    std::cout << "DEBUG: Bridge listening on UDP port " << PORT << "..." << std::endl;

    // debug loop, to test if we are receiving data
    while (true) 
    {
        socklen_t len = sizeof(cliaddr);
        ssize_t n = recvfrom(sockfd, buffer, BUFFER_SIZE, MSG_WAITALL, (struct sockaddr *)&cliaddr, &len);
        if (n < 0) {
            perror("Receive failed");
            continue;
        }
        buffer[n] = '\0';
        std::cout << "DEBUG: Received data from client" << std::endl;

        // Parse egm message
        abb::egm::EgmRobot msg; // Use the namespace defined in egm.pb.h
        if (msg.ParseFromArray(buffer, n)) {
            if (msg.has_header() && msg.header().has_seqno()) {
                std::cout << "SeqNo: " << msg.header().seqno() << std::endl;
            }
        }
    }
}