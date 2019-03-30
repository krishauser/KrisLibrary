#ifndef BASIC_FILE_H
#define BASIC_FILE_H

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

#ifdef _WIN32
#include <basetsd.h>
typedef UINT_PTR SOCKET;
#else
typedef int SOCKET;
#endif // _WIN32

//opaque pointer
struct FileImpl;

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
  ///Non-copyable
  File(const File&) = delete;
  const File& operator = (const File&) = delete;
  ///Opens a named file on the hard drive with the given open mode
	bool Open(const char*, int openmode = FILEREAD | FILEWRITE);
	///Connects this File object to a previously opened FILE object
	bool Open(void*, int openmode = FILEREAD | FILEWRITE);
	///Connects this File object to a memory buffer
	bool OpenData(void* buf, int size, int openmode = FILEREAD | FILEWRITE);
	///Creates a new memory buffer to be written to / read from via this
	///File object.
	bool OpenData(int openmode = FILEREAD | FILEWRITE);
	///Connects this File object to a socket, using TCP transmission
	bool OpenTCPSocket(SOCKET sockfd);
	///Connects this File object to a socket, using UDP transmission
	bool OpenUDPSocket(SOCKET sockfd);
	///Closes this File object
	void Close();

	///Seeks the current read/write position by the given amount, from the 
	///given pointer (may be any of the FILESEEK___ constants).  Returns
	///false if the file object does not support seeking, or the amount
	///is invalid (before or after the file contents)
	bool Seek(int amount, int from = FILESEEKCURRENT);
	///Returns the position of the read/write position, or -1 if the file
	///object is a stream (does not support positioning).
	int Position() const;
	///Returns the total length of the data, or -1 if this file object is
	///a stream.
	int Length() const;

	bool ReadData(void*, int size);
	bool WriteData(const void*, int size);

	///Reads a null-terminated string of at most bufsize characters.
	bool ReadString(char*, int bufsize);
	///Writes a null-terminated string.
	bool WriteString(const char*);

	///Returns true if the file object is open
	bool IsOpen() const;
	///In an internally managed memory buffer created via OpenData(),
	///resizes the buffer to the given size.
	void ResizeDataBuffer(int size);
	///Provides low level access to the memory buffer, or NULL if it's
	///not a data buffer
	unsigned char* GetDataBuffer() const; 
	///Provides low level access to the file object.  If it's a file,
	///returns a pointer to the FILE_POINTER object. If it's a data buffer,
	///returns the data buffer pointe.r  If it's a socket, returns a
	///pointer to the SOCKET object.  If the object is not open, returns
	///NULL.
	void* FileObjectPointer(); 
	///Returns true if you can read up to numbytes bytes on the file object
	bool ReadAvailable(int numbytes=1) const;
	///Returns true if you can write up to numbytes bytes on the file
	///object
	bool WriteAvailable(int numbytes=1) const;

private:
	int mode;		//file read/write mode
	int srctype;	//data source mode (file,data,etc)
	
	FileImpl* impl;
};

/** \file File.h
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
