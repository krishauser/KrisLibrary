#ifndef UTILS_SOCKET_UTILS_H
#define UTILS_SOCKET_UTILS_H

#ifndef _WIN32
#include <sys/socket.h>
typedef int SOCKET;
const static int INVALID_SOCKET = -1;
#else
#include <WinSock2.h>
#endif

///Parses an address of the form http://servername, ftp://servername,
///tcp://servername:port, or udp://servername:port into the protocol,
///host, and port.
///
///Caller must ensure that protocol and host are large enough to handle the 
///items.  Simple way of doing this is to allocate to size strlen(addr).
bool ParseAddr(const char* addr,char* protocol,char* host,int& port);

///Create a client socket.  Returns the socket file descriptor or INVALID_SOCKET otherwise.
///If it's a valid file descriptor, it must be closed using CloseSocket(fd)
SOCKET Connect(const char* addr);

///Create a server socket.  Returns the socket file descriptor or INVALID_SOCKET otherwise.
///If block is false, subsequent Accept calls will not block.
///
///After binding, need to call listen() and accept() (Accept() is ok too)
///If it's a valid file descriptor, it must be closed using CloseSocket(fd).
SOCKET Bind(const char* addr,bool block=true);

///Accepts a connection on the server socket, returns the connection socket
///file descriptor.
SOCKET Accept(SOCKET sockfd);

///Accepts a connection on the server socket, returns the connection socket
///file descriptor.  This version uses a timeout
SOCKET Accept(SOCKET sockfd,double timeout);

///Sets a socket to nonblocking mode
void SetNonblock(SOCKET sockfd,bool enabled=true);

///Cross platform socket close
void CloseSocket(SOCKET sockfd);

///Sets a socket to TCP_NODELAY mode (disables Nagle's algorithm)
void SetNodelay(SOCKET sockfd,bool enabled=true);

///Returns true if data exists to read from
bool ReadAvailable(SOCKET socketfd);

///Returns true if the socket is not busy sending data
bool WriteAvailable(SOCKET socketfd);

///Returns true if the socket is in an exception state
bool HasException(SOCKET socketfd);

#endif

