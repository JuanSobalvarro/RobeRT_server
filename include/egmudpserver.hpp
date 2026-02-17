#pragma once

#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>

#include "egm.pb.h"

namespace robert
{
class EGMUDPServer {
public:
    EGMUDPServer(int port);
    ~EGMUDPServer();
    void start();
    void stop();
private:
    int sockfd;
    bool running;
    void handleClient();
};
}
