#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <communication/socketserver.h>

SocketServer::SocketServer(const int port)
: sockfd(-1), connectfd(-1), portno(port), receivedStream("")
{ }

SocketServer::~SocketServer() { close_connection(); }

bool
SocketServer::establish_connection(void) { return open_connection(); }

bool
SocketServer::open_connection(void)
{
    // Socket anlegen
    // Domain: AF_INIT (Internetadresse fuer Heterogene Systeme)
    // Type: SOCK_STREAM (fuer TCP/IP)
    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (-1 == sockfd)
    {
        printf("ERROR opening socket\n");
        return false;
    }

    // serv_addr - Struktur mit 0 initialisieren
    bzero((char *) &serv_addr, sizeof(serv_addr));

    // Socketaddresse-Struct initialisieren
    serv_addr.sin_family = AF_INET;

    // Portnummer des Servers (htons fuer Konvertierung in Networkbyteorder)
    serv_addr.sin_port = htons(portno);

    // IP-Adresse des Servers: IP des eigenen hosts (INADDR_ANY)
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    // TCP_NODELAY flag setzen
    int flag = 1;
    setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));

    // Socket an Socketaddresse binden
    if (-1 == bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)))
    {
        printf("ERROR on binding.\nPlease wait until release or try another port.\n");
        close(sockfd);
        return false;
    }

    // Listen (maximal fuenf Prozesse kann in der Warteschleife auf Verbindung warten)
    if (-1 == listen(sockfd, 5))
    {
        printf("ERROR listening.\n");
        close(sockfd);
        return false;
    }

    // Warten auf eine Client-Connection
    connectfd = accept(sockfd, NULL, NULL);

    if (0 > connectfd)
    {
        printf("ERROR on accept.\n");
        close(sockfd);
        return false;
    }

    // Connection established
    return true;
}

void
SocketServer::close_connection(void)
{
    if (-1 != connectfd)
    {
        if (-1 == shutdown(connectfd, SHUT_RDWR))
        {
            printf("FIXME: can not shutdown socket.\n");
            close(connectfd);
            close(sockfd);
            return;
        } else printf("Socket shut down.\n");

        close(connectfd);
    }
    close(sockfd);
    printf("Socket closed.\n");
}

bool SocketServer::send_message(const std::string& msg) const
{
    int n = write(connectfd, msg.c_str(), msg.length());
    if (n < 0) {
        printf("ERROR writing to socket.\n");
        return false;
    }
    return true;
}

std::string SocketServer::getNextMessage() const
{
    #define MSGLEN 8192
    char buffer[MSGLEN];
    bzero(buffer, MSGLEN); // Buffer leeren

    // vom Socket lesen (blockiert, bis was ankommt)
    int n = read(connectfd, buffer, MSGLEN); // TODO: make non-blocking read
    if (n < 0)
    {
        printf("ERROR reading from socket\n");
        return std::string("");
    }

    if (0 == n)
    {
        printf("Reading no more bytes from socket. Exiting.\n");
        return std::string("EXIT\n");
    }

    return std::string(buffer);
}

std::string SocketServer::getNextLine(void)
{
    std::string::size_type pos;
    std::string ret;

    while((pos = receivedStream.find("\n", 0)) == std::string::npos)
    {
        receivedStream += getNextMessage();
    }
    ret = receivedStream.substr(0, pos);
    receivedStream.erase(0, pos+1);

    return ret;
}


/* fin */

