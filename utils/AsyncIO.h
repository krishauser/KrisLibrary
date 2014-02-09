#ifndef UTILS_ASYNC_IO
#define UTILS_ASYNC_IO

#include <myfile.h>
#include <string>
#include <vector>
#include <deque>
#include <Timer.h>
#include <errors.h>
#include <math/math.h>
#ifndef WIN32
#include <pthread.h>
#endif //WIN32
using namespace std;

/** @brief Asynchronous reader base class.
 * 
 * Message writer is implemented by subclass. It may store an internal queue of messages.
 *
 * Message reader will call NewMessages() or NewestMessage() to get the latest message. This will also flush the queue.
 * To do a non-blocking check for new messages without flushing the queue, call NewMessageCount().
 *
 * Event-driven readers can be implemented by overloading the Work() class.
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

///stores the last messages and the newest queueMax messages
class AsyncReaderQueue : public AsyncReader
{
 public:
  ///This will keep only the newest queueMax messages
  AsyncReaderQueue(size_t queueMax=1000);
  ///Called by writer subclass to add a message onto the queue
  void Enqueue(const string& msg);
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


#ifndef WIN32

///An asynchronous reader that uses multithreading.
///Subclass will define what that particular process is (usually blocking I/O)
///by overloading the Callback() function.
///If no read has occurred for 'timeout' seconds, then the thread will quit
class AsyncReaderThread : public AsyncReaderQueue
{
 public:
  AsyncReaderThread(double timeout=Math::Inf);
  ~AsyncReaderThread();
  virtual void Reset();
  virtual void Work() { AssertNotReached(); }
  ///Subclasses: override these to implement custom starting and stopping routines
  virtual bool Start();
  virtual void Stop();

  ///Subclasses -- in thread, do some I/O and processing (which may block), return the payload message.
  ///If NULL is returned, there is some error and the thread quits
  virtual const char* Callback() { return NULL; }

  bool initialized;
  pthread_t thread;
  double timeout;

  Timer timer;
  pthread_mutex_t mutex;
  string payload;
  bool newMessage;
  double lastReadTime;
};


///Lets you read the last message sent to a socket.
class SocketReadWorker : public AsyncReaderThread
{
 public:
  SocketReadWorker(const char* addr,double timeout=Math::Inf);
  virtual bool Start();
  virtual void Stop();
  virtual const char* Callback();

  string addr;
  File socket;
  char buf[4096];
};

#endif //WIN32

#endif
