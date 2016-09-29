#ifndef _SOCKETSERVER_H_
#define _SOCKETSERVER_H_

#include <netinet/in.h>
#include <string>

class SocketServer
{
public:
    SocketServer(const int port);
    ~SocketServer();
    bool establish_connection(void);
    bool send_message(const std::string& msg) const;
    std::string getNextLine(); // get message stream until \n

private:
    int sockfd, connectfd;  // socket file descriptors
    int portno;             // port number
    struct sockaddr_in serv_addr;
    std::string receivedStream;

    bool open_connection(void);
    void close_connection(void);
    std::string getNextMessage() const;
};

#endif /* _SOCKETSERVER_H_ */
