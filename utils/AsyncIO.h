#ifndef UTILS_ASYNC_IO
#define UTILS_ASYNC_IO

#include <string>
#include <vector>
#include <list>
#include <memory>
#include <string.h>
#include <KrisLibrary/Logger.h>
#include <KrisLibrary/File.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/errors.h>
#include <KrisLibrary/math/math.h>
#include "threadutils.h"

/** @brief Asynchronous reader with queue.
 *
 * Stores the last message and the newest queueMax messages from some other process / subclass.
 * 
 * Usage is to call Newest() or New() to get the latest message(s). These will 
 * flush the queue.
 * 
 * To do a non-blocking check for new messages without flushing the queue,
 * call UnreadCount().  To peek to the latest, call PeekLast()
 */
class AsyncReaderQueue
{
 public:
  ///This will keep only the newest queueMax messages
  AsyncReaderQueue(size_t queueMax=1000);
  virtual ~AsyncReaderQueue() {}
  ///Called by subclass to add a message onto the queue
  void OnRead(const std::string& msg);
  void OnRead_NoLock(const std::string& msg);

  ///Resets the queue and history
  virtual void Reset();
  ///Do some work to read messages from sender -- must be done by subclass
  virtual void Work() {}
  int MessageCount() { return (int)msgCount; }
  int UnreadCount();
  std::string PeekNewest();
  std::string Newest() { msgQueue.clear(); return PeekNewest(); }
  std::vector<std::string> New();

  Mutex mutex;
  size_t queueMax;
  size_t msgCount;
  std::string msgLast;
  std::list<std::string> msgQueue;
  size_t numDroppedMsgs;
};

/** @brief Asynchronous writer with queue.
 *
 * Sends the latest queueMax messages.
 *
 * The usage is to call SendMessage().  The writer will then somehow
 * send it to a receiver (as implemented by the subclass or some external
 * monitor).
 */
class AsyncWriterQueue 
{
 public:
  ///This will keep only the newest recvQueueMax messages sent, and will only
  ///allow an overflow of sendQueueMax messages.  Typical usage may be 1 and 1
  ///(only keep and send newest messages).
  AsyncWriterQueue(size_t queueMax=1000);
  virtual ~AsyncWriterQueue() {}

  ///Called by subclass to see whether there's a message to send
  bool WriteAvailable() const { return !msgQueue.empty(); }
  ///Called by subclass to get the next message to deliver to the destination
  std::string OnWrite();
  std::string OnWrite_NoLock();

  ///Resets the queue and history
  virtual void Reset();
  ///Do some work to write messages to receiver -- must be done by subclass
  virtual void Work() {}
  void Send(const std::string& msg);
  int SentCount() { return msgCount+msgQueue.size(); }
  int DeliveredCount() { return (int)msgCount; }

  Mutex mutex;
  size_t queueMax;
  size_t msgCount;
  std::list<std::string> msgQueue;
  size_t numDroppedMsgs;
};

/** @brief Asynchronous reader/writer with queues.
 */
class AsyncPipeQueue
{
 public:
  AsyncPipeQueue(size_t recvQueueSize=1000,size_t writeQueueSize=1000);
  virtual void Reset() { reader.Reset(); writer.Reset(); }
  virtual void Work() {}

  ///Interfaces that subclasses should use in Work()
  ///Called by subclass to add a message onto the queue
  void OnRead(const std::string& msg) { return reader.OnRead(msg); }
  void OnRead_NoLock(const std::string& msg) { return reader.OnRead_NoLock(msg); }
  ///Called by subclass to see whether there's a message to send
  bool WriteAvailable() const { return writer.WriteAvailable(); }
  ///Called by subclass to get the next message to send to the destination
  std::string OnWrite() { return writer.OnWrite(); }
  std::string OnWrite_NoLock() { return writer.OnWrite_NoLock(); }

  ///Receive functions
  int MessageCount() { return reader.MessageCount(); }
  int UnreadCount() { return reader.UnreadCount(); }
  std::string PeekNewest() { return reader.PeekNewest(); }
  std::vector<std::string> New() { return reader.New(); }
  std::string Newest() { return reader.Newest(); }

  ///Send functions
  void Send(const std::string& msg) { writer.Send(msg); }
  int SentCount() { return writer.SentCount(); }
  int DeliveredCount() { return writer.DeliveredCount(); }

  AsyncReaderQueue reader;
  AsyncWriterQueue writer;
};

/** @brief A base class for a transport protocol for
 * unstructured data.
 */
class TransportBase
{
 public:
  TransportBase() {}
  virtual ~TransportBase() {}
  ///Subclasses -- can the object read?
  virtual bool Start() { return true; }
  virtual bool Stop() { return true; }
  ///Subclasses -- can the object read?
  virtual bool ReadReady() { return true; }
  ///Subclasses -- can the object write?
  virtual bool WriteReady() { return true; }
  ///Subclasses -- in thread, do some I/O and processing (which may block), return the payload
  ///message.
  ///If NULL is returned, there is some error and the thread quits
  virtual const std::string* DoRead() { return NULL; }
  ///Subclasses -- in thread, do some I/O and processing (which may block)
  ///If false is returned, there is some error and the caller should quit or
  ///reopen the connection.
  virtual bool DoWrite(const char* msg,int length) { return false; }

  ///Subclasses -- in thread, do some I/O and processing (which may block)
  ///If false is returned, there is some error and the caller should quit or
  ///reopen the connection.
  ///(Helper function -- raw C strings)
  virtual bool DoWrite(const char* msg) { return DoWrite(msg,strlen(msg)); }
  ///Subclasses -- in thread, do some I/O and processing (which may block)
  ///If false is returned, there is some error and the caller should quit or
  ///reopen the connection.
  ///(Helper function -- raw strings)
  virtual bool DoWrite(const std::string& msg) { return DoWrite(msg.c_str(),msg.length()); }
};


/** @brief A transport protocol that uses STL streams.
 *
 * Stream format is controlled by the format member.  By default uses
 * IntLengthPrepended.
 *
 * - IntLengthPrepended: prepends the integer length of the string.
 * - NullTerminated: simply writes NULL-terminated strings (messages
 *   must not contain 0 bytes in them!)
 * - Ascii: writes raw strings, possibly quoted, separated by an endline '\n'.
 * - Base64: writes Base64 encoded raw messages, separated by an endline '\n'.
 */
class StreamTransport : public TransportBase
{
 public:
  std::istream* in;
  std::ostream* out;
  enum Format { IntLengthPrepended, NullTerminated, Ascii, Base64 };
  Format format;
  std::string buffer;

  StreamTransport(std::istream& in);
  StreamTransport(std::ostream& out);
  StreamTransport(std::istream& in,std::ostream& out);
  virtual ~StreamTransport() {}
  virtual bool ReadReady();
  virtual bool WriteReady();
  virtual const std::string* DoRead();
  virtual bool DoWrite(const char* msg,int length);
};


/** @brief A transport protocol that connects as a client to the given address. */
class SocketClientTransport : public TransportBase
{
 public:
  ///Opens a new client socket on the given address
  SocketClientTransport(const char* addr);
  ///Initializes with an already open socket
  SocketClientTransport(const char* addr,SOCKET socket);
  virtual bool ReadReady();
  virtual bool WriteReady();
  virtual bool Start();
  virtual bool Stop();
  ///Reads a string (4 byte length + data). Note: blocking
  virtual const std::string* DoRead();
  ///Writes a string (4 byte length + data). Note: blocking.
  virtual bool DoWrite (const char* str,int length);

  std::string addr;
  File socket;
  Mutex mutex;
  std::string buf;
};

/** @brief A transport protocol that hosts a client and sends messages to
 * the clients after they connect . */
class SocketServerTransport : public TransportBase
{
 public:
  SocketServerTransport(const char* addr,int maxclients=1);
  ~SocketServerTransport();
  virtual bool Start();
  virtual bool Stop();
  virtual bool ReadReady();
  virtual bool WriteReady();
  ///Reads a string (4 byte length + data) from all clients. Note: blocking
  virtual const std::string* DoRead();
  ///Writes a string (4 byte length + data) to all clients. Note: blocking.
  virtual bool DoWrite (const char* str,int length);

  std::string addr;
  int serversocket;
  int maxclients;
  Mutex mutex;
  std::vector<std::unique_ptr<File> > clientsockets;
  int currentclient;
  std::string buf;
};

/** @brief An synchronous reader/writer.
 * User/subclass will initialize the transport protocol (usually blocking I/O)
 * by setting the transport member.
 *
 * User will call a work loop:
 *   pipe.Start();
 *   while(not done) pipe.Work(); 
 *   pipe.Stop();
 *
 * The read / write queues are asynchronous so it is safe to call SendMessage,
 * NewestMessage, NewMessages from another thread.
 */
class SyncPipe : public AsyncPipeQueue
{
 public:
  SyncPipe();
  virtual ~SyncPipe();
  virtual void Reset();
  virtual void Work();
  ///Subclasses: override these to implement custom starting and stopping routines
  virtual bool Start();
  virtual void Stop();
  inline bool Connected() { return initialized && (transport->WriteReady() || transport->ReadReady()); }
  inline bool WriteReady() { return initialized && transport->WriteReady(); }
  inline bool ReadReady() { return initialized && transport->ReadReady(); }

  std::unique_ptr<TransportBase> transport;

  bool initialized;
  Timer timer;
  double lastReadTime,lastWriteTime;
};


/** @brief An asynchronous reader that uses multithreading.
 * User/subclass will initialize the transport protocol (usually blocking I/O)
 * by setting the transport member.
 *
 * If no read has occurred for 'timeout' seconds, then the thread will quit
 *
 * Stop() or destructor will quit the read thread.
 */
class AsyncReaderThread : public AsyncReaderQueue
{
 public:
  AsyncReaderThread(double timeout=Math::Inf);
  virtual ~AsyncReaderThread();
  virtual void Reset();
  virtual void Work() { LOG4CXX_ERROR(KrisLibrary::logger(),"No need to call Work on AsyncReaderThread"); }
  ///Subclasses: override these to implement custom starting and stopping routines
  virtual bool Start();
  virtual void Stop();

  std::unique_ptr<TransportBase> transport;

  bool initialized;
  Thread thread;
  double timeout;

  Timer timer;
  double lastReadTime;
};

/** @brief An asynchronous read/writer that uses multithreading.
 * Subclass will define the transport protocol by setting the transport member
 * (usually blocking I/O).
 * 
 * If no read/write has occurred for 'timeout' seconds, then the thread will quit
 *
 * Stop() or destructor will quit the read/write threads.
 */
class AsyncPipeThread : public AsyncPipeQueue
{
 public:
  AsyncPipeThread(double timeout=Math::Inf);
  virtual ~AsyncPipeThread();
  virtual void Reset();
    virtual void Work() { LOG4CXX_ERROR(KrisLibrary::logger(),"No need to call Work on AsyncReaderThread"); }
  ///Subclasses: override these to implement custom starting and stopping routines
  virtual bool Start();
  virtual void Stop();
  inline bool Connected() { return initialized && (transport->WriteReady() || transport->ReadReady()); }
  inline bool WriteReady() { return initialized && transport->WriteReady(); }
  inline bool ReadReady() { return initialized && transport->ReadReady(); }

  std::unique_ptr<TransportBase> transport;

  bool initialized;
  Thread workerThread;
  double timeout;

  Timer timer;
  Mutex mutex;
  double lastReadTime;
  double lastWriteTime;
};


/** @brief Launches a thread for asynchronous reading from a socket.
 * 
 * Calling thread can safely use the AsyncReaderQueue methods e.g., 
 * NewMessageCount, NewMessage, NewestMessage while the thread is running.
 */
class SocketReadWorker : public AsyncReaderThread
{
 public:
  SocketReadWorker(const char* addr,bool asServer=false,double timeout=Math::Inf)
    :AsyncReaderThread(timeout)
  {
    if(asServer)
      transport.reset(new SocketServerTransport(addr));
    else
      transport.reset(new SocketClientTransport(addr));
  }
};

/** @brief Launches a thread for asynchronous bidirectional communication with
 * a socket.
 * 
 * Calling thread can safely use the AsyncPipeQueue methods e.g., 
 * SendMessage, NewMessageCount, NewMessage, NewestMessage while the thread 
 * is running.
 */
class SocketPipeWorker : public AsyncPipeThread
{
 public:
  SocketPipeWorker(const char* addr,bool asServer=false,double timeout=Math::Inf)
    :AsyncPipeThread(timeout)
  {
    if(asServer)
      transport.reset(new SocketServerTransport(addr));
    else
      transport.reset(new SocketClientTransport(addr));
  }

  SocketPipeWorker(const char* addr,SOCKET socket,double timeout=Math::Inf)
    :AsyncPipeThread(timeout)
  {
    transport.reset(new SocketClientTransport(addr,socket));
  }
};

#endif
