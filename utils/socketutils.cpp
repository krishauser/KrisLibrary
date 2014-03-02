#include "socketutils.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
//BSD socket stuff
#ifndef WIN32
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <fcntl.h>
#else
#endif //WIN32

#ifdef WIN32
bool SocketStart()
{
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
        return 1;
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
        return false;
    }
    else
        printf("The Winsock 2.2 dll was found okay\n");
	return true;
}

int close(int fd)
{
	return closesocket(fd);
}

#else

bool SocketStart() { return true; }
bool SocketStop() { return true; }

#endif //WIN32

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


int Connect(const char* addr)
{
  char* protocol = new char[strlen(addr)];
  char* host = new char[strlen(addr)];
  int port;
  if(!ParseAddr(addr,protocol,host,port)) {
    fprintf(stderr,"Error parsing address %s\n",addr);
    delete [] protocol;
    delete [] host;
    return -1;
  }

  struct sockaddr_in serv_addr;
  struct hostent *server;
  
  int sockettype = SOCK_STREAM;
  if(0==strcmp(protocol,"udp")) {
    sockettype = SOCK_DGRAM;
  }
  delete [] protocol;
	  
  int sockfd = socket(AF_INET, sockettype, 0);
  if (sockfd < 0) {
    fprintf(stderr,"File::Open: Error creating socket\n");
    delete [] host;
    return -1;
  }
  server = gethostbyname(host);
  if (server == NULL) {
    fprintf(stderr,"File::Open: Error, no such host %s:%d\n",host,port);
    close(sockfd);
    sockfd = -1;
    delete [] host;
    return -1;
  }
  memset(&serv_addr, 0, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  memcpy(&serv_addr.sin_addr.s_addr,
	 server->h_addr,
	 server->h_length);
  serv_addr.sin_port = htons(port);

  if (connect(sockfd,(sockaddr*)&serv_addr,sizeof(serv_addr)) < 0) {
    fprintf(stderr,"File::Open: Connect client to %s:%d failed\n",host,port);
    perror("");
    close(sockfd);
    delete [] host;
    return -1;
  }

  return sockfd;
}


int Bind(const char* addr,bool block)
{
  char* protocol = new char[strlen(addr)];
  char* host = new char[strlen(addr)];
  int port;
  if(!ParseAddr(addr,protocol,host,port)) {
    fprintf(stderr,"Error parsing address %s\n",addr);
    delete [] protocol;
    delete [] host;
    return -1;
  }

  struct sockaddr_in serv_addr;
  struct hostent *server;
  
  int sockettype = SOCK_STREAM;
  if(0==strcmp(protocol,"udp")) {
    sockettype = SOCK_DGRAM;
  }
  delete [] protocol;
	  
  int sockfd = socket(AF_INET, sockettype, 0);
  if (sockfd < 0) {
    fprintf(stderr,"File::Open: Error creating socket\n");
    delete [] host;
    return -1;
  }
  if(!block)  {
	  SetNonblock(sockfd);
  }

  server = gethostbyname(host);
  if (server == NULL) {
    fprintf(stderr,"File::Open: Error, no such host %s:%d\n",host,port);
    close(sockfd);
    delete [] host;
    return -1;
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
    close(sockfd);
    delete [] host;
    return -1 ;
  }
  delete [] host;
  return sockfd;
}

int Accept(int sockfd)
{
  struct sockaddr_in cli_addr;
  int clilen = sizeof(cli_addr);
  int clientsocket = accept(sockfd, (struct sockaddr *)&cli_addr, 
			    &clilen);
  return clientsocket;
}

void SetNonblock(int sockfd)
{
#ifdef WIN32
	u_long iMode=1;
	ioctlsocket(sockfd,FIONBIO,&iMode);
#else
    fcntl(sockfd,F_SETFL,FNDELAY);
#endif //WIN32
}
