#ifndef UTILS_SOCKET_UTILS_H
#define UTILS_SOCKET_UTILS_H

#ifndef WIN32
#include <sys/socket.h>
#else
#include <WinSock2.h>
#endif

///Starts up the network library, if needed.  Necessary for Windows.
bool SocketStart();

///Closes the network library, if needed.  Necessary for Windows.
bool SocketStop();

///Parses an address of the form http://servername, ftp://servername,
///tcp://servername:port, or udp://servername:port into the protocol,
///host, and port.
///
///Caller must ensure that protocol and host are large enough to handle the 
///items.  Simple way of doing this is to allocate to size strlen(addr).
bool ParseAddr(const char* addr,char* protocol,char* host,int& port);

///Create a client socket.  Returns the socket file descriptor or -1 otherwise.
///If it's a valid file descriptor, it must be closed using close(fd)
int Connect(const char* addr);

///Create a server socket.  Returns the socket file descriptor or -1 otherwise.
///If block is false, subsequent Accept calls will not block.
///
///After binding, need to call listen() and accept() (Accept() is ok too)
///If it's a valid file descriptor, it must be closed using close(fd).
int Bind(const char* addr,bool block=true);

///Accepts a connection on the server socket, returns the connection socket
///file descriptor.
int Accept(int sockfd);

///Sets a socket to nonblocking mode
void SetNonblock(int sockfd);

#endif

