#include <KrisLibrary/Logger.h>
#include "File.h"
#include "utils.h"
#include "errors.h"
#include <stdlib.h>
#include <stdio.h>
#include <memory.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <KrisLibrary/utils/socketutils.h>
#include <KrisLibrary/utils/threadutils.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
typedef HANDLE FILE_POINTER;
#define INVALID_FILE_POINTER INVALID_HANDLE_VALUE
typedef UINT_PTR SOCKET;

#else

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
typedef FILE *FILE_POINTER;
#define INVALID_FILE_POINTER NULL
typedef int SOCKET;

#endif // _WIN32

struct FileImpl
{
	FILE_POINTER file;
	unsigned char* datafile;
	int datapos;
	int datasize;
	SOCKET socket; 
};

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

inline int SocketClose(SOCKET fd) { shutdown(fd,SD_BOTH); return closesocket(fd); }

inline int SocketRead(SOCKET fd,char* data,int size) { return recv(fd,data,size,0); }

inline int SocketWrite(SOCKET fd,const char* data,int size) { return send(fd,data,size,0); }

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
 impl(new FileImpl)
{
    impl->file = INVALID_FILE_POINTER;
    impl->datafile = NULL;
    impl->datapos = 0;
    impl->datasize = 0;
    impl->socket = INVALID_SOCKET;
}

File::~File()
{
	Close();
	delete impl;
}

void File::Close()
{
    if(srctype == MODE_MYFILE && impl->file != INVALID_FILE_POINTER) FileClose(impl->file);
	if(srctype == MODE_MYDATA && impl->datafile != NULL) free(impl->datafile);
	if((srctype == MODE_TCPSOCKET || srctype==MODE_UDPSOCKET) && impl->socket != INVALID_SOCKET) SocketClose(impl->socket);

	srctype = MODE_NONE;
	mode = 0;
	impl->file = INVALID_FILE_POINTER;
	impl->datafile = NULL;
	impl->datapos = 0;
	impl->datasize = 0;
	impl->socket = INVALID_SOCKET;
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

	impl->datafile = (unsigned char*)data;
	impl->datapos = 0;
	impl->datasize = size;
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
	Assert(IsOpen());
	return true;
}

void* File::FileObjectPointer()
{
  if(srctype == MODE_MYDATA || srctype == MODE_EXTDATA)
    return impl->datafile;
  else if(srctype == MODE_TCPSOCKET || mode == MODE_UDPSOCKET) {
    if(impl->socket == INVALID_SOCKET) return NULL;
    return &impl->socket;
  }
  if(impl->file == INVALID_FILE_POINTER) return NULL;
  return &impl->file;
}

void File::ResizeDataBuffer(int size)
{
  assert(srctype == MODE_MYDATA);
	unsigned char* olddata=impl->datafile;
	impl->datafile=(unsigned char*)malloc(size);
	if(!impl->datafile) FatalError("Memory allocation error, size %d\n",size);
	memcpy(impl->datafile,olddata,impl->datasize);
	free(olddata);
	impl->datasize = size;
}

unsigned char* File::GetDataBuffer() const
{
  if(srctype == MODE_MYDATA || srctype == MODE_EXTDATA)
    return impl->datafile;
  return NULL;
}


bool File::OpenTCPSocket(SOCKET sockfd)
{
  Close();
  if(sockfd == 0) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"File::Open: socket file descriptor 0  is incompatible");
    return false;
  }

  impl->socket = sockfd;
  srctype = MODE_TCPSOCKET;
  //can read and write to sockets
  mode = FILEREAD | FILEWRITE;
  return true;
}

bool File::OpenUDPSocket(SOCKET sockfd)
{
  Close();
  if(sockfd == 0) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"File::Open: socket file descriptor 0  is incompatible");
    return false;
  }

  impl->socket = sockfd;
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
	      LOG4CXX_ERROR(KrisLibrary::logger(),"File::Open: Accept connection to client on "<<fn);
	      perror("");
	      SocketClose(sockfd);
	      return false;
	    }
	    if(clientsocket == 0) {
	      LOG4CXX_ERROR(KrisLibrary::logger(),"File::Open: Accept connection returned a 0 file descriptor, this is incompatible");
	      SocketClose(clientsocket);
	      SocketClose(sockfd);
	      return false;
	    }
	    impl->socket = clientsocket;
	    srctype = socketsrctype;
	    //can read and write to sockets
	    mode = FILEREAD | FILEWRITE;
	    SocketClose(sockfd);
	    LOG4CXX_INFO(KrisLibrary::logger(),"File::Open server socket "<<fn);
	    return true;
	  }
	  else {
	    SOCKET sockfd = Connect(fn);
	    if (sockfd == INVALID_SOCKET) {
	      LOG4CXX_ERROR(KrisLibrary::logger(),"File::Open: Connect client to "<<fn);
	      perror("");
	      return false;
	    }	    
	    if(sockfd == 0) {
	      LOG4CXX_ERROR(KrisLibrary::logger(),"File::Open: socket connect returned a 0 file descriptor, this is incompatible");
	      SocketClose(sockfd);
	      return false;
	    }
	    impl->socket = sockfd;
	    srctype = socketsrctype;
	    //can read and write to sockets
	    mode = FILEREAD | FILEWRITE;
	    LOG4CXX_INFO(KrisLibrary::logger(),"File::Open client socket "<<fn);
	    return true;
	  }
	}

	impl->file=FileOpen(fn,openmode);

	if(impl->file == INVALID_FILE_POINTER)
		return false;
	srctype = MODE_MYFILE;
	mode = openmode;
	return true;
}


bool File::Open(void* f, int openmode)
{
	Close();

	srctype = MODE_EXTFILE;
	if(openmode == 0)
		return false;

	impl->file = (FILE*)f;
	mode = openmode;
	return true;
}

int File::Position() const
{
	switch(srctype)
	{
	case MODE_MYFILE:
	case MODE_EXTFILE:
	        return FilePosition(impl->file);
	case MODE_MYDATA:
	case MODE_EXTDATA:
		return impl->datapos;
	case MODE_TCPSOCKET:
	case MODE_UDPSOCKET:
	  if(impl->socket == INVALID_SOCKET) return -1;
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
	       return FileSeek(impl->file,p,from);
		break;
	case MODE_MYDATA:
	case MODE_EXTDATA:
		switch (from)
		{
		//case SEEK_CUR:
		case FILESEEKCURRENT:
			if(impl->datapos + p >= impl->datasize || impl->datapos + p < 0)
				return false;
			impl->datapos += p;
			break;
		//case SEEK_SET:
		case FILESEEKSTART:
			if(p >= impl->datasize || p < 0)
				return false;
			impl->datapos = p;
			break;
		//case SEEK_END:
		case FILESEEKEND:
			if(impl->datasize + p < 0 || p > 0)
				return false;
			impl->datapos = impl->datasize + p;
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
	        return FileLength(impl->file);
	case MODE_MYDATA:
	case MODE_EXTDATA:
		return impl->datasize;
	}
	return -1;
}

bool File::ReadData(void* d, int size)
{
  if(size < 0)
        LOG4CXX_ERROR(KrisLibrary::logger(),"File::ReadData: invalid size "<<size);
	if(mode & FILEREAD)
	{
		switch(srctype)
		{
		case MODE_MYFILE:
		case MODE_EXTFILE:
			return FileRead(impl->file,d,size);
		case MODE_MYDATA:
		case MODE_EXTDATA:
			if(impl->datapos + size > impl->datasize)
				return false;
			memcpy(d, impl->datafile+impl->datapos, size);
			impl->datapos += size;
			return true;
		case MODE_TCPSOCKET:
		case MODE_UDPSOCKET:
		  {
		    char* buffer = (char*)d;
		    int totalread = 0;
		    while(totalread < size) {
		      int n=SocketRead(impl->socket,buffer+totalread,size-totalread);
		      if(n == 0) {
			LOG4CXX_INFO(KrisLibrary::logger(),"File(socket): socketRead returned 0, connection shutdown");
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
			return FileWrite(impl->file,d,size);
		case MODE_MYDATA:		//resize if buffer's not big enough
			if(impl->datapos + size > impl->datasize) {
				int a=impl->datapos+size,b=impl->datasize*2;
				ResizeDataBuffer(Max(a,b));
			}
			memcpy(impl->datafile+impl->datapos, d, size);
			impl->datapos += size;
			return true;
		case MODE_EXTDATA:
			if(impl->datapos + size > impl->datasize)
				return false;
			memcpy(impl->datafile+impl->datapos, d, size);
			impl->datapos += size;
			return true;
		case MODE_TCPSOCKET:
		case MODE_UDPSOCKET:
		  {
		    const char* msg = (const char*)d;
		    int totalsent = 0;
		    while(totalsent < size) {
		      int n = SocketWrite(impl->socket,msg+totalsent,size-totalsent);
		      if(n < 0) {
			perror("File(socket) SocketWrite");
			return false;
		      }
		      if(n == 0) {
			LOG4CXX_INFO(KrisLibrary::logger(),"File(socket): SocketWrite returned "<<n<<", what does it mean?");
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
		int i;
		switch(srctype)
		{
		case MODE_MYFILE:
		case MODE_EXTFILE:
			for(i=0; i<bufsize; i++)
			{
				int c = ReadChar(impl->file);
				if(c==EOF) {
				  if(i != 0) LOG4CXX_ERROR(KrisLibrary::logger(),"File::ReadString hit end of file without finding null character");
				  return false;
				}
				str[i]=c;
				if(c==0)
					return true;
			}
			LOG4CXX_ERROR(KrisLibrary::logger(),"File::ReadString string length is greater than buffer size "<<bufsize);
			break;
		case MODE_MYDATA:
		case MODE_EXTDATA:
			for(i=0; i<bufsize; i++)
			{
			  if(impl->datapos >= impl->datasize) {
			    LOG4CXX_ERROR(KrisLibrary::logger(),"File::ReadString ran past end of internal buffer without finding null character");
			    return false;
			  }
			  str[i]=impl->datafile[impl->datapos];
			  impl->datapos++;
			  if(str[i]==0)
			    return true;
			}
			LOG4CXX_ERROR(KrisLibrary::logger(),"File::ReadString string length is greater than buffer size "<<bufsize);
			break;
		case MODE_TCPSOCKET:
		case MODE_UDPSOCKET:
		  {
		    int slen;
		    if(!ReadData(&slen,4)) {
		      LOG4CXX_ERROR(KrisLibrary::logger(),"File::ReadString read length failed");
		      return false;
		    }
		    if(slen < 0) {
		      LOG4CXX_ERROR(KrisLibrary::logger(),"File::ReadString read length "<<slen);
		      return false;
		    }
		    if(slen+1 > bufsize) {
		      LOG4CXX_ERROR(KrisLibrary::logger(),"File::ReadString read length "<<slen<<" is greater than buffer size "<<bufsize);
		      return false;
		    }
		    if(!ReadData(str,slen)) {
		      LOG4CXX_ERROR(KrisLibrary::logger(),"File::ReadString read string failed");
		      return false;
		    }
		    str[slen] = 0;
		    return true;
		  }
		default:
		  		  LOG4CXX_ERROR(KrisLibrary::logger(),"File::ReadString: unknown file type "<<srctype);
		  break;
		}
	}
	else
	  LOG4CXX_ERROR(KrisLibrary::logger(),"File::ReadString: file not in FILEREAD mode");
	return false;
}

bool File::WriteString(const char* str)
{
  switch(srctype) {
  case MODE_TCPSOCKET:
  case MODE_UDPSOCKET:
    if(strlen(str) > 0xffffffff) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"File::WriteString: string must be no longer than 2^32");
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
  if(srctype == MODE_TCPSOCKET || srctype == MODE_UDPSOCKET) {
    if(impl->socket == INVALID_SOCKET) return false;
    return true;
  }
  if(srctype == MODE_MYDATA || srctype == MODE_EXTDATA)
    return (impl->datafile != NULL);
  if(impl->file == INVALID_FILE_POINTER) return false;
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
    return ::ReadAvailable(impl->socket);
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
    return ::WriteAvailable(impl->socket);
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
