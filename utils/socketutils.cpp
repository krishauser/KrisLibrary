#include "socketutils.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
//BSD socket stuff
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <fcntl.h>
#ifndef WIN32
#include <unistd.h>
#endif

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
  if(!block) 
    fcntl(sockfd,F_SETFL,FNDELAY);

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
  fcntl(sockfd,F_SETFL,FNDELAY);
}
