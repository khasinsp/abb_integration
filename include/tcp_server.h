#include <bits/stdc++.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/tcp.h>
#include <unistd.h>

#define BUFFER_SIZE 1024
#define SERVER_IP "192.168.0.50"

class TCPServer {
    public: 
        TCPServer(std::string host, unsigned short port) : local_host_(host), local_port_(port) {

        }

        ~TCPServer() {
            close(sockfd_);
        }

        virtual void start_() {

            struct sockaddr_in serv_addr;
            memset(&serv_addr, 0, sizeof(serv_addr));
            serv_addr.sin_family = AF_INET;
            serv_addr.sin_port = htons(local_port_);

            if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr) <= 0) {
                perror("Invalid address (not supported)");
                return;
            }

            sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
            if (sockfd_ < 0) {
                perror("Error opening socket");
                return;
            }

            optval = 1;
            setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void *) &optval, sizeof(int));
            memset(&serveraddr_, 0, sizeof(serveraddr_));
            serveraddr_.sin_family = AF_INET;
            serveraddr_.sin_addr.s_addr = htonl(INADDR_ANY);
            serveraddr_.sin_port = htons(local_port_);
            if (bind(sockfd_, (struct sockaddr *) & serveraddr_, sizeof(serveraddr_)) < 0) {
                perror("Error on binding socket");
                return;
            }
            clientlen_ = sizeof(clientaddr_);

            // listen(sockfd_, 1);

            int flag = 1;
            int result = setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));
            if (result < 0) {
                perror("Error disabling Nagle-Algorithm");
                return;
            }

            if (connect(sockfd_, (struct sockaddr*) &serv_addr, sizeof(serv_addr)) < 0) {
                perror("Connection failed");
                return;
            }

            std::cout << "Connection established" << std::endl;
        }

        virtual void setToNonblockingMode_() {
            int flags_client = fcntl(sockfd_, F_GETFL, 0);
            if (flags_client == -1) {
                perror("fnctl(F_GETFL) failed on sockfd_");
                close(sockfd_);
                return;
            }

            flags_client |= O_NONBLOCK;
            if (fcntl(sockfd_, F_SETFL, flags_client) == -1) {
                perror("fnctl(F_SETFL) failed on sockfd_");
                close(sockfd_);
                return;
            }

        }

        virtual ssize_t send_(std::vector<uint8_t> &buffer) {
            ssize_t bytes = 0;
            bytes = send(sockfd_, buffer.data(), buffer.size(), 0);
            if (bytes < 0) {
                perror("Error in send()");
            }
            return bytes;
        }

        virtual ssize_t recv_(std::string &buffer) {
            char new_buffer[BUFFER_SIZE];
            memset(new_buffer, 0, BUFFER_SIZE);
            ssize_t bytes = recv(sockfd_, new_buffer, BUFFER_SIZE, 0);

            if (bytes > 0) {
                buffer = std::string(new_buffer, bytes);
            }

            return bytes;
        }
    
    private:
        std::string local_host_;
        unsigned short local_port_;
        
        int sockfd_;
        socklen_t clientlen_;
        struct sockaddr_in serveraddr_;
        struct sockaddr_in clientaddr_;
        char buffer_[BUFFER_SIZE];
        int optval;

};
