#ifndef UTILS_ASYNC_IO
#define UTILS_ASYNC_IO

#include <myfile.h>
#include <string>
#include <vector>
#include <deque>
#include <Timer.h>
#include <errors.h>
#include <math/math.h>
#include "SmartPointer.h"
#include "threadutils.h"
using namespace std;

/** @brief Asynchronous reader base class.
 * 
 * Message storage is implemented by subclass. It may store an internal queue of messages.
 * @sa AsyncReaderQueue
 *
 * Usage is to call NewMessages() or NewestMessage() to get the latest message. This will also
 * flush the queue.
 * 
 * To do a non-blocking check for new messages without flushing the queue, call NewMessageCount().
 *
 * Event-driven readers can be implemented by overloading the Work() class.  The user must
 * then periodically call Work() to allow the reader to process messages.
 */
class AsyncReader
{
 public:
  AsyncReader() {}
  virtual ~AsyncReader() {}
  virtual void Reset() {}
  virtual void Work() {}
  virtual int MessageCount() { return 0; }
  virtual int NewMessageCount() { return 0; }
  virtual string LastMessage() { return ""; }
  virtual vector<string> NewMessages() { return vector<string>(); }
  virtual string NewestMessage() { if(NewMessageCount()>0) return NewMessages().back(); return LastMessage(); }
};

/** @brief Asynchronous writer base class.
 * 
 * Message queuing can be implemented in the subclass.
 * @sa AsyncWriterQueue.
 *
 * The usage is to call SendMessage().  The writer will then somehow
 * send it to a receiver (as implemented by the subclass).
 *
 * Event-driven writers can be implemented by overloading the Work() class.  The user must
 * then periodically call Work() to allow the reader to process messages.
 */
class AsyncWriter
{
 public:
  AsyncWriter() {}
  virtual ~AsyncWriter() {}
  virtual void Reset() { }
  virtual void Work() {}
  virtual void SendMessage(const string& msg) {}
  virtual int SentMessageCount() { return 0; }
  virtual int DeliveredMessageCount() { return 0; }
};


/** @brief Asynchronous reader with queue.
 *
 * stores the last message and the newest queueMax messages.
 */
class AsyncReaderQueue : public AsyncReader
{
 public:
  ///This will keep only the newest queueMax messages
  AsyncReaderQueue(size_t queueMax=1000);
  virtual ~AsyncReaderQueue() {}
  ///Called by subclass to add a message onto the queue
  void OnRead(const string& msg);

  ///Resets the queue and history
  virtual void Reset();
  virtual int MessageCount() { return (int)msgCount; }
  virtual int NewMessageCount() { return (int)msgQueue.size(); }
  virtual string LastMessage() { return msgLast; }
  virtual vector<string> NewMessages();

  size_t queueMax;
  size_t msgCount;
  string msgLast;
  deque<string> msgQueue;
};

/** @brief Asynchronous writer with queue.
 *
 * Sends the latest queueMax messages.
 */
class AsyncWriterQueue : public AsyncWriter
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
  string OnWrite();

  virtual void Reset();
  virtual void SendMessage(const string& msg);
  virtual int SentMessageCount() { return msgCount+msgQueue.size(); }
  virtual int DeliveredMessageCount() { return (int)msgCount; }

  size_t queueMax;
  size_t msgCount;
  deque<string> msgQueue;
};

/** @brief Asynchronous reader/writer with queues.
 */
class AsyncPipeQueue
{
 public:
  AsyncPipeQueue(size_t recvQueueSize=1000,size_t writeQueueSize=1000);
  virtual void Reset() { reader.Reset(); writer.Reset(); }
  virtual void Work() { reader.Work(); writer.Work(); }

  ///Interfaces that subclasses should use in Work()
  ///Called by subclass to add a message onto the queue
  void OnRead(const string& msg) { return reader.OnRead(msg); }
  ///Called by subclass to see whether there's a message to send
  bool WriteAvailable() const { return writer.WriteAvailable(); }
  ///Called by subclass to get the next message to send to the destination
  string OnWrite() { return writer.OnWrite(); }

  ///Receive functions
  int MessageCount() { return reader.MessageCount(); }
  int NewMessageCount() { return reader.NewMessageCount(); }
  string LastMessage() { return reader.LastMessage(); }
  vector<string> NewMessages() { return reader.NewMessages(); }
  string NewestMessage() { return reader.NewestMessage(); }

  ///Send functions
  void SendMessage(const string& msg) { writer.SendMessage(msg); }
  int SentMessageCount() { return writer.SentMessageCount(); }
  int DeliveredMessageCount() { return writer.DeliveredMessageCount(); }


  AsyncReaderQueue reader;
  AsyncWriterQueue writer;
};

class AsyncTransport
{
 public:
  AsyncTransport() {}
  virtual ~AsyncTransport() {}
  ///Subclasses -- can the object read?
  virtual bool Start() { return true; }
  virtual bool Stop() { return true; }
  ///Subclasses -- can the object read?
  virtual bool ReadReady() { return true; }
  ///Subclasses -- can the object write?
  virtual bool WriteReady() { return true; }
  ///Subclasses -- in thread, do some I/O and processing (which may block), return the payload message.
  ///If NULL is returned, there is some error and the thread quits
  virtual const char* DoRead() { return NULL; }
  ///Subclasses -- in thread, do some I/O and processing (which may block)
  ///If false is returned, there is some error and the thread quits
  virtual bool DoWrite(const char* msg) { return false; }
};

/** @brief A transport protocol that connects as a client to the given address. */
class SocketClientTransport : public AsyncTransport
{
 public:
  SocketClientTransport(const char* addr);
  virtual bool Start();
  virtual bool Stop();
  ///Reads a string (4 byte length + data). Note: blocking
  virtual const char* DoRead();
  ///Writes a string (4 byte length + data). Note: blocking.
  virtual bool DoWrite (const char* str);

  string addr;
  File socket;
  char buf[4096];
};

/** @brief A transport protocol that hosts a client and sends messages to
 * the clients after they connect . */
class SocketServerTransport : public AsyncTransport
{
 public:
  SocketServerTransport(const char* addr,int maxclients=1);
  virtual bool Start();
  virtual bool Stop();
  virtual bool ReadReady();
  virtual bool WriteReady();
  ///Reads a string (4 byte length + data) from all clients. Note: blocking
  virtual const char* DoRead();
  ///Writes a string (4 byte length + data) to all clients. Note: blocking.
  virtual bool DoWrite (const char* str);

  string addr;
  int serversocket;
  int maxclients;
  vector<SmartPointer<File> > clientsockets;
  int currentclient;
  char buf[4096];
};


///An asynchronous reader that uses multithreading.
///Subclass will define what that particular process is (usually blocking I/O)
///by overloading the Callback() function.
///If no read has occurred for 'timeout' seconds, then the thread will quit
class AsyncReaderThread : public AsyncReaderQueue
{
 public:
  AsyncReaderThread(double timeout=Math::Inf);
  virtual ~AsyncReaderThread();
  virtual void Reset();
  virtual void Work() { AssertNotReached(); }
  ///Subclasses: override these to implement custom starting and stopping routines
  virtual bool Start();
  virtual void Stop();

  SmartPointer<AsyncTransport> transport;

  bool initialized;
  Thread thread;
  double timeout;

  Timer timer;
  Mutex mutex;
  double lastReadTime;
};

///An asynchronous reader that uses multithreading.
///Subclass will define what that particular process is (usually blocking I/O)
///by overloading the Callback() function.
///If no read/write has occurred for 'timeout' seconds, then the thread will quit
class AsyncPipeThread : public AsyncPipeQueue
{
 public:
  AsyncPipeThread(double timeout=Math::Inf);
  virtual ~AsyncPipeThread();
  virtual void Reset();
  ///Subclasses: override these to implement custom starting and stopping routines
  virtual bool Start();
  virtual void Stop();

  SmartPointer<AsyncTransport> transport;

  bool initialized;
  Thread readThread,writeThread;
  double timeout;

  Timer timer;
  Mutex mutex;
  double lastReadTime;
  double lastWriteTime;
};


class SocketReadWorker : public AsyncReaderThread
{
 public:
  SocketReadWorker(const char* addr,bool asServer=false,double timeout=Math::Inf)
    :AsyncReaderThread(timeout)
  {
    if(asServer)
      transport = new SocketServerTransport(addr);
    else
      transport = new SocketClientTransport(addr);
  }
};

class SocketPipeWorker : public AsyncPipeThread
{
 public:
  SocketPipeWorker(const char* addr,bool asServer=false,double timeout=Math::Inf)
    :AsyncPipeThread(timeout)
  {
    if(asServer)
      transport = new SocketServerTransport(addr);
    else
      transport = new SocketClientTransport(addr);
  }
};

#endif
