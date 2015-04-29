#include "socketutils.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
//BSD socket stuff
#ifndef _WIN32
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netdb.h> 
#endif


#ifdef _WIN32
typedef int socklen_t;

struct WSASocketGlobal
{
public:
  bool started,failed;
  WSASocketGlobal() : started(false),failed(false) {}
  ~WSASocketGlobal() {
    if(started) {
      printf("Shutting down the Winsock 2.2 dll\n");
      WSACleanup();
    }
  }
  bool Init() {
    WORD wVersionRequested;
    WSADATA wsaData;
    int err;

/* Use the MAKEWORD(lowbyte, highbyte) macro declared in Windef.h */
    wVersionRequested = MAKEWORD(2, 2);

    err = WSAStartup(wVersionRequested, &wsaData);
    if (err != 0) {
        /* Tell the user that we could not find a usable */
        /* Winsock DLL.                                  */
        printf("WSAStartup failed with error: %d\n", err);
		failed = true;
        return false;
    }

/* Confirm that the WinSock DLL supports 2.2.*/
/* Note that if the DLL supports versions greater    */
/* than 2.2 in addition to 2.2, it will still return */
/* 2.2 in wVersion since that is the version we      */
/* requested.                                        */

    if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2) {
        /* Tell the user that we could not find a usable */
        /* WinSock DLL.                                  */
        printf("Could not find a usable version of Winsock.dll\n");
        WSACleanup();
		failed = true;
        return false;
    }
    else {
        printf("The Winsock 2.2 dll was found successfully\n");
	started = true;
	return true;
    }
  }
};

static WSASocketGlobal gWSASocketObject;

bool EnsureSocketStarted() 
{
  if(gWSASocketObject.failed) return false;
  if(gWSASocketObject.started) return true;  
  return gWSASocketObject.Init();
}

#else

bool EnsureSocketStarted() { return true; }

#endif //_WIN32

///Caller must ensure that protocol and host are large enough to handle the 
///items.  Simple way of doing this is to allocate to size strlen(addr)
bool ParseAddr(const char* addr,char* protocol,char* host,int& port)
{
  const char* pos=strstr(addr,"://");
  if(pos == NULL) return false;
  //parse protocol
  int spos = pos-addr;
  strncpy(protocol,addr,spos);
  protocol[spos] = 0;
  //parse address and port
  pos += 3;
  spos += 3;
  const char* colonpos = strstr(pos,":");
  if(colonpos==NULL) {
    strcpy(host,pos);
  }
  else {
    strncpy(host,pos,colonpos-pos);
    host[colonpos-pos]=0;
  }
  port = -1;
  //default http port
  if(strcmp(protocol,"http")==0)
    port = 80;
  //default ftp port
  if(strcmp(protocol,"ftp")==0)
    port = 21;

  if(colonpos != NULL) {
    //parse port
    colonpos ++;
    char* endptr;
    long int res = strtol(colonpos,&endptr,0);
    if(res==0 && endptr==colonpos) {
      fprintf(stderr,"ParseAddr: address did not contain valid port\n");
      return false;
    }
    if(res < 0 || res > 0xffff) {
      fprintf(stderr,"ParseAddr: address did not contain valid port\n");
      return false;
    }
    port = (int)res;
  }

  if(port < 0) {
    fprintf(stderr,"ParseAddr: address did not contain valid port\n");
    return false;
  }
  return true;
}


SOCKET Connect(const char* addr)
{
  if(!EnsureSocketStarted()) return INVALID_SOCKET;

  char* protocol = new char[strlen(addr)];
  char* host = new char[strlen(addr)];
  int port;
  if(!ParseAddr(addr,protocol,host,port)) {
    fprintf(stderr,"Connect: Error parsing address %s\n",addr);
    delete [] protocol;
    delete [] host;
    return INVALID_SOCKET;
  }

  struct sockaddr_in serv_addr;
  struct hostent *server;
  
  int sockettype = SOCK_STREAM;
  if(0==strcmp(protocol,"udp")) {
    sockettype = SOCK_DGRAM;
  }
  delete [] protocol;
	  
  SOCKET sockfd = socket(AF_INET, sockettype, 0);
  if (sockfd == INVALID_SOCKET) {
    fprintf(stderr,"Connect: Error creating socket\n");
    delete [] host;
    return INVALID_SOCKET;
  }
  server = gethostbyname(host);
  if (server == NULL) {
    fprintf(stderr,"Connect: Error, no such host %s:%d\n",host,port);
    CloseSocket(sockfd);
    delete [] host;
    return INVALID_SOCKET;
  }
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  memcpy(&serv_addr.sin_addr.s_addr,
	 server->h_addr,
	 server->h_length);
  serv_addr.sin_port = htons(port);

  if (connect(sockfd,(sockaddr*)&serv_addr,sizeof(serv_addr)) < 0) {
    fprintf(stderr,"Connect: Connect to server %s:%d failed\n",host,port);
    perror("  Connect error");
    CloseSocket(sockfd);
    delete [] host;
    return INVALID_SOCKET;
  }

  return sockfd;
}


SOCKET Bind(const char* addr,bool block)
{
  if(!EnsureSocketStarted()) return INVALID_SOCKET;

  char* protocol = new char[strlen(addr)];
  char* host = new char[strlen(addr)];
  int port;
  if(!ParseAddr(addr,protocol,host,port)) {
    fprintf(stderr,"Error parsing address %s\n",addr);
    delete [] protocol;
    delete [] host;
    return INVALID_SOCKET;
  }

  struct sockaddr_in serv_addr;
  struct hostent *server;
  
  int sockettype = SOCK_STREAM;
  if(0==strcmp(protocol,"udp")) {
    sockettype = SOCK_DGRAM;
  }
  delete [] protocol;
	  
  SOCKET sockfd = socket(AF_INET, sockettype, 0);
  if (sockfd == INVALID_SOCKET) {
    fprintf(stderr,"File::Open: Error creating socket\n");
    delete [] host;
    return INVALID_SOCKET;
  }
  if(!block)  {
	  SetNonblock(sockfd);
  }

  server = gethostbyname(host);
  if (server == NULL) {
    fprintf(stderr,"File::Open: Error, no such host %s:%d\n",host,port);
    CloseSocket(sockfd);
    delete [] host;
    return INVALID_SOCKET;
  }
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  memcpy(&serv_addr.sin_addr.s_addr,
	 server->h_addr,
	 server->h_length);
  serv_addr.sin_port = htons(port);

  if (bind(sockfd,(sockaddr*)&serv_addr,sizeof(serv_addr)) < 0) {
    fprintf(stderr,"File::Open: Bind server to %s:%d failed\n",host,port);
    perror("");
    CloseSocket(sockfd);
    delete [] host;
    return INVALID_SOCKET ;
  }
  delete [] host;
  return sockfd;
}

SOCKET Accept(SOCKET sockfd)
{
  if(!EnsureSocketStarted()) return INVALID_SOCKET;

  struct sockaddr_in cli_addr;
  socklen_t clilen = sizeof(cli_addr);
  SOCKET clientsocket = accept(sockfd, (struct sockaddr *)&cli_addr, 
			    &clilen);
  return clientsocket;
}

SOCKET Accept(SOCKET sockfd,double timeout)
{
  if(!EnsureSocketStarted()) return INVALID_SOCKET;

  fd_set rfds;
  FD_ZERO(&rfds);
  FD_SET(sockfd, &rfds);

  timeval tv;
  double secs = floor(timeout);
  tv.tv_sec = (int)secs;
  tv.tv_usec = (timeout-secs)*1000000;

  int result = select(sockfd + 1, &rfds, (fd_set *) 0, (fd_set *) 0, &tv);
  if(result > 0) {
    struct sockaddr_in cli_addr;
    socklen_t clilen = sizeof(cli_addr);
    SOCKET clientsocket = accept(sockfd, (struct sockaddr *)&cli_addr, 
				 &clilen);
    return clientsocket;
  }
  else  {
    if(result < 0) {
      printf("Error using select()\n");
    }
     //always here, even if i connect from another application
    return INVALID_SOCKET;
  }
}


void SetNonblock(SOCKET sockfd)
{
#ifdef _WIN32
	u_long iMode=1;
	ioctlsocket(sockfd,FIONBIO,&iMode);
#else
    fcntl(sockfd,F_SETFL,FNDELAY);
#endif //_WIN32
}


void CloseSocket(SOCKET sockfd)
{
#ifdef _WIN32
	shutdown(sockfd,SD_BOTH);
	closesocket(sockfd);
#else
	shutdown(sockfd,SHUT_RDWR);
	close(sockfd);
#endif
}


bool ReadAvailable(SOCKET socketfd)
{ 
  fd_set fds;
  timeval tv;
  // clear the set ahead of time
  FD_ZERO(&fds);
  
  // add our descriptors to the set
  FD_SET(socketfd, &fds);

#ifdef _WIN32
  int n = 0; //this parameter is ignored
#else
  int n = socketfd + 1;
#endif 

  // wait until either socket has data ready to be recv()d (timeout 10.5 secs)
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  int rv = select(n, &fds, NULL, NULL, &tv);

  if (rv == -1) {
    perror("select"); // error occurred in select()
    return false;
  } else if (rv == 0) {
    return false;
  } else {
    if(FD_ISSET(socketfd, &fds))
      return true;
    else
      printf("ReadAvailable: weird, select returned 1 but the FD set is not set\n");
    return false;
  }
}

bool WriteAvailable(SOCKET socketfd)
{ 
  fd_set fds;
  timeval tv;
  // clear the set ahead of time
  FD_ZERO(&fds);
  
  // add our descriptors to the set
  FD_SET(socketfd, &fds);

#ifdef _WIN32
  int n = 0; //this parameter is ignored
#else
  int n = socketfd + 1;
#endif 

  // wait until either socket has data ready to be recv()d (timeout 10.5 secs)
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  int rv = select(n, NULL, &fds, NULL, &tv);

  if (rv == -1) {
    perror("select"); // error occurred in select()
    return false;
  } else if (rv == 0) {
    return false;
  } else {
    if(FD_ISSET(socketfd, &fds))
      return true;
    else
      printf("WriteAvailable: weird, select returned 1 but the FD set is not set\n");
    return false;
  }
}

bool HasException(SOCKET socketfd)
{ 
  fd_set fds;
  timeval tv;
  // clear the set ahead of time
  FD_ZERO(&fds);
  
  // add our descriptors to the set
  FD_SET(socketfd, &fds);

#ifdef _WIN32
  int n = 0; //this parameter is ignored
#else
  int n = socketfd + 1;
#endif 

  // wait until either socket has data ready to be recv()d (timeout 10.5 secs)
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  int rv = select(n, NULL, NULL, &fds, &tv);

  if (rv == -1) {
    perror("select"); // error occurred in select()
    return false;
  } else if (rv == 0) {
    return false;
  } else {
    if(FD_ISSET(socketfd, &fds))
      return true;
    else
      printf("HasException: weird, select returned 1 but the FD set is not set\n");
    return false;
  }
}
