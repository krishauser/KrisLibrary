#include "myfile.h"
#include "utils.h"
#include <stdlib.h>
#include <stdio.h>
#include <memory.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <utils/socketutils.h>
#include <utils/threadutils.h>
#ifndef _WIN32
#include <unistd.h>
#endif

//platform-specific defines

#ifdef _WIN32

inline FILE_POINTER FileOpen(const char* fn, int openmode)
{
	if(openmode & FILEREAD)
	{
		if(openmode & FILEWRITE)
		        return CreateFile(fn, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
		else
		        return CreateFile(fn, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	}
	else if(openmode & FILEWRITE)
	{
		return CreateFile(fn, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	}
	abort();
}

inline void FileClose(FILE_POINTER x)
{
  CloseHandle(x);
}

inline int FilePosition(FILE_POINTER x)
{
  return SetFilePointer(x, 0, 0, FILE_CURRENT);
}

inline bool FileSeek(FILE_POINTER x, int p, int from)
{
  return (SetFilePointer(x, p, 0, from) != 0);
}

inline int FileLength(FILE_POINTER x) {
  return GetFileSize(x, NULL);
}

inline bool FileRead(FILE_POINTER x, void* d, int size)
{
  DWORD s;
  if(!ReadFile(x, d, size, &s, NULL))
    return false;
  return (size == s);
}

inline bool FileWrite(FILE_POINTER x, const void* d, int size)
{
  DWORD s;
  if(!WriteFile(x, d, size, &s, NULL))
    return false;
  return (size == s);
}

inline int SocketClose(int fd) { shutdown(fd,SD_BOTH); return closesocket(fd); }

inline int SocketRead(int fd,char* data,int size) { return recv(fd,data,size,0); }

inline int SocketWrite(int fd,const char* data,int size) { return send(fd,data,size,0); }

#else

inline FILE_POINTER FileOpen(const char* fn, int openmode) {
  if(openmode & FILEREAD) {
    if(openmode & FILEWRITE)
      return fopen(fn,"r+b");
    else
      return fopen(fn,"rb");
  }
  else {
    return fopen(fn,"wb");
  }
}

inline void FileClose(FILE_POINTER x)
{
  fclose(x);
}

inline int FilePosition(FILE_POINTER x)
{
  return ftell(x);
}

inline bool FileSeek(FILE_POINTER x, int p, int from)
{
  return fseek(x,p,from);
}

inline int FileLength(FILE_POINTER x)
{
  long pos=ftell(x);
  fseek(x,0,SEEK_END);
  long len=ftell(x);
  fseek(x,pos,SEEK_SET);
  return len;
}

inline bool FileRead(FILE_POINTER x, void* d, int size)
{
  return (int)fread(d,1,size,x)==size;
}

inline bool FileWrite(FILE_POINTER x, const void* d, int size)
{
  return (int)fwrite(d,1,size,x)==size;
}

inline int SocketClose(int fd) { shutdown(fd,SHUT_RDWR); return close(fd); }

inline int SocketRead(int fd,char* data,int size) { return read(fd,data,size); }

inline int SocketWrite(int fd,const char* data,int size) { return write(fd,data,size); }

#endif




enum { MODE_NONE,
       MODE_MYFILE, //internally managed file
       MODE_EXTFILE, //external file reference
       MODE_MYDATA, //internally managed data
       MODE_EXTDATA, //external data reference
       MODE_TCPSOCKET, //TCP socket
       MODE_UDPSOCKET //UDP socket
};

File::File()
:mode(0),srctype(MODE_NONE),
 file(INVALID_FILE_POINTER),
 datafile(NULL),datapos(0),datasize(0),
 socket(0)
{
}

File::~File()
{
	Close();
}

void File::Close()
{
        if(srctype == MODE_MYFILE && file != INVALID_FILE_POINTER) FileClose(file);
	if(srctype == MODE_MYDATA && datafile != NULL) free(datafile);
	if((srctype == MODE_TCPSOCKET || srctype==MODE_UDPSOCKET) && file > 0) SocketClose(socket);

	srctype = MODE_NONE;
	mode = 0;
	file = INVALID_FILE_POINTER;
	datafile = NULL;
	datapos = 0;
	datasize = 0;
	socket = 0;
}

bool File::OpenData(void* data, int size, int openmode)
{
	Close();

	if(!data)
		return false;
	if(size < 0)
		return false;

	srctype = MODE_EXTDATA;
	if(openmode == 0)
		return false;

	datafile = (unsigned char*)data;
	datapos = 0;
	datasize = size;
	mode = openmode;
	return true;
}

bool File::OpenData(int openmode)
{
	Close();

	srctype = MODE_MYDATA;
	if(openmode == 0)
		return false;

	ResizeDataBuffer(64);
	mode = openmode;
	return true;
}

void* File::FileObjectPointer()
{
  if(srctype == MODE_MYDATA || srctype == MODE_EXTDATA)
    return datafile;
  else if(srctype == MODE_TCPSOCKET || mode == MODE_UDPSOCKET) {
    if(socket == INVALID_SOCKET) return NULL;
    return &socket;
  }
  if(file == INVALID_FILE_POINTER) return NULL;
  return &file;
}

void File::ResizeDataBuffer(int size)
{
  assert(srctype == MODE_MYDATA);
	unsigned char* olddata=datafile;
	datafile=(unsigned char*)malloc(size);
	memcpy(datafile,olddata,datasize);
	free(olddata);
	datasize = size;
}

unsigned char* File::GetDataBuffer() const
{
  if(srctype == MODE_MYDATA || srctype == MODE_EXTDATA)
    return datafile;
  return NULL;
}


bool File::OpenTCPSocket(SOCKET sockfd)
{
  Close();
  if(sockfd == 0) {
    fprintf(stderr,"File::Open: socket file descriptor 0  is incompatible\n");
    return false;
  }

  socket = sockfd;
  srctype = MODE_TCPSOCKET;
  //can read and write to sockets
  mode = FILEREAD | FILEWRITE;
  return true;
}

bool File::OpenUDPSocket(SOCKET sockfd)
{
  Close();
  if(sockfd == 0) {
    fprintf(stderr,"File::Open: socket file descriptor 0  is incompatible\n");
    return false;
  }

  socket = sockfd;
  srctype = MODE_UDPSOCKET;
  //can read and write to sockets
  mode = FILEREAD | FILEWRITE;
  return true;
}

bool File::Open(const char* fn, int openmode)
{
	Close();

	if(openmode == 0)
		return false;

	const char* addrpos=strstr(fn,"://");
	if(addrpos != NULL) {
	  int socketsrctype = MODE_TCPSOCKET;
	  if(strstr(fn,"udp://") != NULL)
	    socketsrctype = MODE_UDPSOCKET;
	  if(openmode == FILESERVER) {
	    SOCKET sockfd = Bind(fn);
	    listen(sockfd,1);
	    SOCKET clientsocket = Accept(sockfd);
	    if(clientsocket == INVALID_SOCKET) {
	      fprintf(stderr,"File::Open: Accept connection to client on %s failed\n",fn);
	      perror("");
	      SocketClose(sockfd);
	      return false;
	    }
	    if(clientsocket == 0) {
	      fprintf(stderr,"File::Open: Accept connection returned a 0 file descriptor, this is incompatible\n");
	      SocketClose(clientsocket);
	      SocketClose(sockfd);
	      return false;
	    }
	    socket = clientsocket;
	    srctype = socketsrctype;
	    //can read and write to sockets
	    mode = FILEREAD | FILEWRITE;
	    SocketClose(sockfd);
	    printf("File::Open server socket %s succeeded\n",fn);
	    return true;
	  }
	  else {
	    SOCKET sockfd = Connect(fn);
	    if (sockfd == INVALID_SOCKET) {
	      fprintf(stderr,"File::Open: Connect client to %s failed\n",fn);
	      perror("");
	      return false;
	    }	    
	    if(sockfd == 0) {
	      fprintf(stderr,"File::Open: socket connect returned a 0 file descriptor, this is incompatible\n");
	      SocketClose(sockfd);
	      return false;
	    }
	    socket = sockfd;
	    srctype = socketsrctype;
	    //can read and write to sockets
	    mode = FILEREAD | FILEWRITE;
	    printf("File::Open client socket %s succeeded\n",fn);
	    return true;
	  }
	}

	file=FileOpen(fn,openmode);

	if(file == INVALID_FILE_POINTER)
		return false;
	srctype = MODE_MYFILE;
	mode = openmode;
	return true;
}


bool File::Open(FILE_POINTER f, int openmode)
{
	Close();

	srctype = MODE_EXTFILE;
	if(openmode == 0)
		return false;

	file = f;
	mode = openmode;
	return true;
}

int File::Position() const
{
	switch(srctype)
	{
	case MODE_MYFILE:
	case MODE_EXTFILE:
	        return FilePosition(file);
	case MODE_MYDATA:
	case MODE_EXTDATA:
		return datapos;
	case MODE_TCPSOCKET:
	case MODE_UDPSOCKET:
	  if(socket == INVALID_SOCKET) return -1;
	  return 0;
	}
	return -1;
}

bool File::Seek(int p, int from)
{
	switch(srctype)
	{
	case MODE_MYFILE:
	case MODE_EXTFILE:
	       return FileSeek(file,p,from);
		break;
	case MODE_MYDATA:
	case MODE_EXTDATA:
		switch (from)
		{
		//case SEEK_CUR:
		case FILESEEKCURRENT:
			if(datapos + p >= datasize || datapos + p < 0)
				return false;
			datapos += p;
			break;
		//case SEEK_SET:
		case FILESEEKSTART:
			if(p >= datasize || p < 0)
				return false;
			datapos = p;
			break;
		//case SEEK_END:
		case FILESEEKEND:
			if(datasize + p < 0 || p > 0)
				return false;
			datapos = datasize + p;
			break;
		}
	case MODE_TCPSOCKET:
	case MODE_UDPSOCKET:
	  return false;
	}
	return true;
}

int File::Length() const
{
	switch(srctype)
	{
	case MODE_MYFILE:
	case MODE_EXTFILE:
	        return FileLength(file);
	case MODE_MYDATA:
	case MODE_EXTDATA:
		return datasize;
	}
	return -1;
}

bool File::ReadData(void* d, int size)
{
  if(size < 0)
    fprintf(stderr,"File::ReadData: invalid size %d\n",size);
	if(mode & FILEREAD)
	{
		switch(srctype)
		{
		case MODE_MYFILE:
		case MODE_EXTFILE:
			return FileRead(file,d,size);
		case MODE_MYDATA:
		case MODE_EXTDATA:
			if(datapos + size > datasize)
				return false;
			memcpy(d, datafile+datapos, size);
			datapos += size;
			return true;
		case MODE_TCPSOCKET:
		case MODE_UDPSOCKET:
		  {
		    char* buffer = (char*)d;
		    int totalread = 0;
		    while(totalread < size) {
		      int n=SocketRead(socket,buffer+totalread,size-totalread);
		      if(n == 0) {
			printf("File(socket): socketRead returned 0, connection shutdown\n");
			return false;
		      }
		      if(n < 0) {
			if(errno==EWOULDBLOCK) {
			  ThreadSleep(0.001);
			  //just spin?
			  continue;
			}
			perror("Unhandled error in socket read");
			return false;
		      }
		      totalread += n;
		    }
		    assert(totalread == size);
		    return true;
		  }
		}
	}
	return false;
}

bool File::WriteData(const void* d, int size)
{
	if(mode & FILEWRITE)
	{
		switch(srctype)
		{
		case MODE_MYFILE:
		case MODE_EXTFILE:
			return FileWrite(file,d,size);
		case MODE_MYDATA:		//resize if buffer's not big enough
			if(datapos + size > datasize) {
				int a=datapos+size,b=datasize*2;
				ResizeDataBuffer(Max(a,b));
			}
			memcpy(datafile+datapos, d, size);
			datapos += size;
			return true;
		case MODE_EXTDATA:
			if(datapos + size > datasize)
				return false;
			memcpy(datafile+datapos, d, size);
			datapos += size;
			return true;
		case MODE_TCPSOCKET:
		case MODE_UDPSOCKET:
		  {
		    const char* msg = (const char*)d;
		    int totalsent = 0;
		    while(totalsent < size) {
		      int n = SocketWrite(socket,msg+totalsent,size-totalsent);
		      if(n < 0) {
			perror("File(socket) SocketWrite");
			return false;
		      }
		      if(n == 0) {
			printf("File(socket): SocketWrite returned %d, what does it mean?\n",n);
			ThreadSleep(0.001);
		      }
		      totalsent += n;
		    }
		    assert(totalsent == size);
		    return true;
		  }
		}
	}
	return false;
}



int ReadChar(FILE_POINTER file)
{
	char c;
	if(!FileRead(file, &c, 1)) return EOF;
	return c;
}

bool File::ReadString(char* str, int bufsize)
{
	if(mode & FILEREAD)
	{
		int i,c;
		switch(srctype)
		{
		case MODE_MYFILE:
		case MODE_EXTFILE:
			for(i=0; i<bufsize; i++)
			{
				c = ReadChar(file);
				if(c==EOF) {
				  if(i != 0) fprintf(stderr,"File::ReadString hit end of file without finding null character\n");
				  return false;
				}
				str[i]=c;
				if(c==0)
					return true;
			}
			fprintf(stderr,"File::ReadString string length is greater than buffer size %d\n",bufsize);
			break;
		case MODE_MYDATA:
		case MODE_EXTDATA:
			for(i=0; i<bufsize; i++)
			{
			  if(datapos >= datasize) {
			    fprintf(stderr,"File::ReadString ran past end of internal buffer without finding null character\n");
			    return false;
			  }
			  str[i]=datafile[datapos];
			  datapos++;
			  if(str[i]==0)
			    return true;
			}
			fprintf(stderr,"File::ReadString string length is greater than buffer size %d\n",bufsize);
			break;
		case MODE_TCPSOCKET:
		case MODE_UDPSOCKET:
		  {
		    int slen;
		    if(!ReadData(&slen,4)) {
		      fprintf(stderr,"File::ReadString read length failed\n");
		      return false;
		    }
		    if(slen < 0) {
		      fprintf(stderr,"File::ReadString read length %d is negative\n",slen);
		      return false;
		    }
		    if(slen+1 > bufsize) {
		      fprintf(stderr,"File::ReadString read length %d is greater than buffer size %d\n",slen,bufsize);
		      return false;
		    }
		    if(!ReadData(str,slen)) {
		      fprintf(stderr,"File::ReadString read string failed\n");
		      return false;
		    }
		    str[slen] = 0;
		    return true;
		  }
		default:
		  fprintf(stderr,"File::ReadString: unknown file type %d\n",srctype);
		  break;
		}
	}
	else
	  fprintf(stderr,"File::ReadString: file not in FILEREAD mode\n");
	return false;
}

bool File::WriteString(const char* str)
{
  switch(srctype) {
  case MODE_TCPSOCKET:
  case MODE_UDPSOCKET:
    if(strlen(str) > 0xffffffff) {
      fprintf(stderr,"File::WriteString: string must be no longer than 2^32\n");
      return false;
    }
    else {
      assert(sizeof(int)==4);
      int slen = (int)strlen(str);
      if(!WriteData(&slen,4)) {
	return false;
      }
      return WriteData(str,slen);
    }
    break;
  default:
    return WriteData(str, (int)(strlen(str)+1));
  }
}

bool File::IsOpen() const
{
  if(srctype == MODE_TCPSOCKET || mode == MODE_UDPSOCKET) {
    if(socket == INVALID_SOCKET) return false;
    return true;
  }
  if(file == INVALID_FILE_POINTER) return false;
  return true;
}

bool File::ReadAvailable(int numbytes) const
{
  if(!IsOpen()) return false;
  if(!(mode & FILEREAD)) return false;
  switch(srctype) {
  case MODE_MYFILE:
  case MODE_EXTFILE:
  case MODE_MYDATA:
  case MODE_EXTDATA:
    return Position()+numbytes <= Length();
  case MODE_TCPSOCKET:
  case MODE_UDPSOCKET:
    return ::ReadAvailable(socket);
  default:
    return false;
  }
}

bool File::WriteAvailable(int numbytes) const
{
  if(!IsOpen()) return false;
  if(!(mode & FILEREAD)) return false;
  switch(srctype) {
  case MODE_MYFILE:
  case MODE_EXTFILE:
  case MODE_MYDATA:
    return true;
  case MODE_EXTDATA:
    return Position()+numbytes <= Length();
  case MODE_TCPSOCKET:
  case MODE_UDPSOCKET:
    return ::WriteAvailable(socket);
  default:
    return false;
  }
}



_DEFINE_READ_WRITE_FILE_BASIC(bool);
_DEFINE_READ_WRITE_FILE_BASIC(char);
_DEFINE_READ_WRITE_FILE_BASIC(signed char);
_DEFINE_READ_WRITE_FILE_BASIC(unsigned char);
_DEFINE_READ_WRITE_FILE_BASIC(short);
_DEFINE_READ_WRITE_FILE_BASIC(unsigned short);
_DEFINE_READ_WRITE_FILE_BASIC(int);
_DEFINE_READ_WRITE_FILE_BASIC(unsigned int);
_DEFINE_READ_WRITE_FILE_BASIC(long);
_DEFINE_READ_WRITE_FILE_BASIC(unsigned long);
_DEFINE_READ_WRITE_FILE_BASIC(long long);
_DEFINE_READ_WRITE_FILE_BASIC(unsigned long long);
_DEFINE_READ_WRITE_FILE_BASIC(float);
_DEFINE_READ_WRITE_FILE_BASIC(double);
_DEFINE_READ_WRITE_FILE_BASIC(long double);
