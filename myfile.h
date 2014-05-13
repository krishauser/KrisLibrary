#ifndef BASIC_FILE_H
#define BASIC_FILE_H

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
typedef HANDLE FILE_POINTER;
#define INVALID_FILE_POINTER INVALID_HANDLE_VALUE
typedef HANDLE SOCKET;

#else

#include <stdlib.h>
#include <stdio.h>
typedef FILE *FILE_POINTER;
#define INVALID_FILE_POINTER NULL
typedef int SOCKET;
#endif // WIN32

///Flags sent to Open: read or write mode (may be bitwise or-ed together)
#define FILEREAD 0x1
#define FILEWRITE 0x2
///Flags sent to Open for socket connections: client mode or server mode.
#define FILECLIENT 0x1
#define FILESERVER 0x2
///Flags sent to Seek
#define FILESEEKSTART 0
#define FILESEEKCURRENT 1
#define FILESEEKEND 2

/** @ingroup Standard
 * @brief A cross-platform class for reading/writing binary data.
 *
 * Can be configured to read from files on disk, memory buffers, or sockets.
 *
 * To read/write to memory buffers, you may either provide a fixed-size buffer
 * to OpenData(buf,size,mode) or just call OpenData(mode), which allocates
 * a buffer for writing to.  GetDataBuffer can then be used to retrieve the
 * buffer.
 *
 * To open a socket, provide an address in the form http://servername,
 * ftp://servername, tcp://servername:port or udp://servername:port to Open. 
 * If opened in FILESERVER mode, the Open call will block until a single
 * connection is accepted. 
 * 
 * Seek and Length do not work in sockets.  Position returns 0 if the socket
 * is open and -1 otherwise.
 *
 * Read/WriteString operate differently in sockets.  Rather than providing a 
 * null-terminated string, the message's first 4 bytes are the length of the
 * string (interpreted as an integer) and the remaining bytes are the string.
 */
class File
{
public:
	File();
	~File();
	bool Open(const char*, int openmode = FILEREAD | FILEWRITE);
	bool Open(FILE_POINTER, int openmode = FILEREAD | FILEWRITE);
	bool OpenData(void* buf, int size, int openmode = FILEREAD | FILEWRITE);
	bool OpenData(int openmode = FILEREAD | FILEWRITE);
	bool OpenTCPSocket(SOCKET sockfd);
	bool OpenUDPSocket(SOCKET sockfd);
	void Close();

	bool Seek(int, int from = FILESEEKCURRENT);
	int Position() const;
	int Length();

	bool ReadData(void*, int size);
	bool WriteData(const void*, int size);

	bool ReadString(char*, int bufsize);
	bool WriteString(const char*);

	void* GetDataBuffer() const;
	void ResizeDataBuffer(int size);

private:
	int mode;		//file read/write mode
	int srctype;	//data source mode (file,data,etc)
	
	FILE_POINTER file;
	unsigned char* datafile;
	int datapos;
	int datasize;
	SOCKET socket; 
};

/** \file myfile.h
 * \ingroup Standard
 * \brief A unified interface for reading/writing binary data to file.
 *
 * A consistent interface for reading/writing data is given by the
 * template methods ReadFile() and WriteFile().  A class can be read/written
 * to file in two ways: 1) Read/WriteFile() can be overloaded
 * to read/write the data directly, or 2) use the default Read/WriteFile(), 
 * but define the following methods in the class:
 * \code
 *   bool Read(File&)
 *   bool Write(File&) const
 * \endcode
 *
 * For structs of contiguous data, the macro 
 * _DEFINE_READ_WRITE_FILE_BASIC(T) will overload the
 * the Read/WriteFile functions to automatically read 
 * the data from file contiguously for objects of type T.
 */

template <class type>
bool ReadFile(File& f, type& t) { return t.Read(f); }
template <class type>
bool WriteFile(File& f, const type& t) { return t.Write(f); }

template <class type>
bool ReadArrayFile(File& f, type* t, int n)
{
  for(int i=0;i<n;i++)
    if(!ReadFile(f,t[i])) return false;
  return true;
}

template <class type>
bool WriteArrayFile(File& f, const type* t, int n)
{
  for(int i=0;i<n;i++)
    if(!WriteFile(f,t[i])) return false;
  return true;
}

#define _DECLARE_READ_WRITE_FILE_BASIC(type) \
  template <> bool ReadFile(File& f, type& t); \
  template <> bool WriteFile(File& f, const type& t); \
  template <> bool ReadArrayFile(File& f, type* t,int n); \
  template <> bool WriteArrayFile(File& f, const type* t,int n); 

#define _DEFINE_READ_WRITE_FILE_BASIC(type) \
  template <> bool ReadFile(File& f, type& t) \
  { return f.ReadData(&t, sizeof(t)); } \
  template <> bool WriteFile(File& f, const type& t) \
  { return f.WriteData(&t, sizeof(t)); } \
  template <> bool ReadArrayFile(File& f, type* t,int n) \
  { return f.ReadData(t, sizeof(type)*n); } \
  template <> bool WriteArrayFile(File& f, const type* t,int n) \
  { return f.WriteData(t, sizeof(type)*n); }


_DECLARE_READ_WRITE_FILE_BASIC(bool);
_DECLARE_READ_WRITE_FILE_BASIC(char);
_DECLARE_READ_WRITE_FILE_BASIC(signed char);
_DECLARE_READ_WRITE_FILE_BASIC(unsigned char);
_DECLARE_READ_WRITE_FILE_BASIC(short);
_DECLARE_READ_WRITE_FILE_BASIC(unsigned short);
_DECLARE_READ_WRITE_FILE_BASIC(int);
_DECLARE_READ_WRITE_FILE_BASIC(unsigned int);
_DECLARE_READ_WRITE_FILE_BASIC(long);
_DECLARE_READ_WRITE_FILE_BASIC(unsigned long);
_DECLARE_READ_WRITE_FILE_BASIC(long long);
_DECLARE_READ_WRITE_FILE_BASIC(unsigned long long);
_DECLARE_READ_WRITE_FILE_BASIC(float);
_DECLARE_READ_WRITE_FILE_BASIC(double);
_DECLARE_READ_WRITE_FILE_BASIC(long double);

#endif
