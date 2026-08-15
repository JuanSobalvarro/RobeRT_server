#include <iostream>

#include "sock_comm.hpp"

namespace robert::sock_comm {

bool initialize() {
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        std::cerr << "[SOCK_COMM] WSAStartup failed.\n";
        return false;
    }
#endif
    return true;
}

void cleanup() {
#ifdef _WIN32
    WSACleanup();
#endif
}

void close_socket(socket_t fd) {
    if (fd == INVALID_SOCKET_FD) return;

#ifdef _WIN32
    closesocket(fd);
#else
    close(fd);
#endif
}

bool set_timeouts(socket_t fd, int timeout_ms) {
    if (fd == INVALID_SOCKET_FD) return false;

#ifdef _WIN32
    // Windows expects a DWORD representing milliseconds passed as a const char*
    DWORD timeout = static_cast<DWORD>(timeout_ms);
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout)) < 0)
        return false;
    if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout)) < 0)
        return false;
#else
    // POSIX expects a struct timeval passed as a const void*
    struct timeval tv{};
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
        return false;
    if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0)
        return false;
#endif

    return true;
}

socket_t connect_tcp(const std::string& host, int port, int timeout_ms) {
    struct addrinfo hints{}, *res = nullptr;
    hints.ai_family = AF_INET;       // force ipv4
    hints.ai_socktype = SOCK_STREAM; // tcp
    hints.ai_protocol = IPPROTO_TCP; // tcp

    std::string port_str = std::to_string(port);

    if (getaddrinfo(host.c_str(), port_str.c_str(), &hints, &res) != 0) {
        return INVALID_SOCKET_FD;
    }

    socket_t fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd == INVALID_SOCKET_FD) {
        freeaddrinfo(res);
        return INVALID_SOCKET_FD;
    }

    set_timeouts(fd, timeout_ms);

    if (connect(fd, res->ai_addr, res->ai_addrlen) < 0) {
        close_socket(fd);
        freeaddrinfo(res);
        return INVALID_SOCKET_FD;
    }

    freeaddrinfo(res);
    return fd;
}

std::string resolve_host(const std::string& host) {
    struct addrinfo hints{}, *res = nullptr;
    hints.ai_family = AF_INET;       // force ipv4
    hints.ai_socktype = SOCK_STREAM; // tcp
    hints.ai_protocol = IPPROTO_TCP; // tcp

    if (getaddrinfo(host.c_str(), nullptr, &hints, &res) != 0) {
        return "";
    }

    char ip[INET_ADDRSTRLEN];
    inet_ntop(res->ai_family, res->ai_addr, ip, INET_ADDRSTRLEN);
    freeaddrinfo(res);
    return ip;
}

} // namespace robert::sock_comm
