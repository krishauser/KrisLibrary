#include <KrisLibrary/Logger.h>
#include "AsyncIO.h"
#include "socketutils.h"
#include "ioutils.h"
#include "stringutils.h"
#include <iostream>
#ifndef _WIN32
#include <unistd.h>
#endif
using namespace std;

AsyncReaderQueue::AsyncReaderQueue(size_t _queueMax)
  :queueMax(_queueMax),msgCount(0),numDroppedMsgs(0)
{}

void AsyncReaderQueue::OnRead(const string& msg)
{
  ScopedLock lock(mutex);
  OnRead_NoLock(msg);
}

void AsyncReaderQueue::OnRead_NoLock(const string& msg)
{
  while(msgQueue.size()>=queueMax) {
    msgQueue.pop_front();
    numDroppedMsgs++;
    if(numDroppedMsgs % 1000 == 1) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"AsyncReaderQueue: Warning, dropped "<<(int)numDroppedMsgs<<" messages, ask your sender to reduce the send rate\n");
    }
  }
  msgQueue.push_back(msg);
  msgLast = msg;
  msgCount += 1;
}

void AsyncReaderQueue::Reset()
{
  ScopedLock lock(mutex);
  msgCount=0;
  msgLast="";
  msgQueue.clear(); 
}

int AsyncReaderQueue::UnreadCount()
{
  ScopedLock lock(mutex);
  return (int)msgQueue.size();
}

string AsyncReaderQueue::PeekNewest()
{
  ScopedLock lock(mutex);
  return msgLast;
}

vector<string> AsyncReaderQueue::New()
{
  ScopedLock lock(mutex);
  if(msgQueue.empty()) { msgLast=""; return vector<string>(); }
  vector<string> res(msgQueue.size());
  copy(msgQueue.begin(),msgQueue.end(),res.begin());
  msgLast=res.back(); 
  msgQueue.clear();
  return res;
}

AsyncWriterQueue::AsyncWriterQueue(size_t _queueMax)
  :queueMax(_queueMax),msgCount(0),numDroppedMsgs(0)
{}


string AsyncWriterQueue::OnWrite()
{
  ScopedLock lock(mutex);
  return OnWrite_NoLock();
}

string AsyncWriterQueue::OnWrite_NoLock()
{
  if(msgQueue.empty()) return "";
  string res = msgQueue.front();
  if(msgQueue.size() >= queueMax) {
    LOG4CXX_INFO(KrisLibrary::logger(),"AsyncWriterQueue: Message queue overflowing!\n");
    LOG4CXX_INFO(KrisLibrary::logger(),"   solution: slow down rate of sending via Send\n");
  }
  while(msgQueue.size()>=queueMax) {
    msgQueue.pop_front();
    numDroppedMsgs++;
    if(numDroppedMsgs % 1000 == 1) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"AsyncWriterQueue: Warning, dropped "<<(int)numDroppedMsgs<<" messages, slow down the rate of sending via SendMessage\n");
    }
  }
  msgQueue.pop_front();
  msgCount += 1;
  return res;
}

void AsyncWriterQueue::Reset()
{
  ScopedLock lock(mutex);
  msgQueue.clear();
  msgCount = 0;
}

void AsyncWriterQueue::Send(const string& msg)
{
  ScopedLock lock(mutex);
  //prevent message queue from growing too big
  while(msgQueue.size()>=queueMax) {
    msgQueue.pop_front();
  }
  msgQueue.push_back(msg);
}


AsyncPipeQueue::AsyncPipeQueue(size_t _recvQueueMax,size_t _sendQueueMax)
  :reader(_recvQueueMax),writer(_sendQueueMax)
{}


SyncPipe::SyncPipe()
  :AsyncPipeQueue(),initialized(false),lastReadTime(-1),lastWriteTime(-1)
{
}

SyncPipe::~SyncPipe()
{
  if (transport) transport->Stop();
}

void SyncPipe::Reset()
{
  Stop();
  AsyncPipeQueue::Reset();
  Start();
}

bool SyncPipe::Start() 
{ 
  if(!transport) return false;
  if(!transport->Start()) return false;
  initialized = true; 
  return true; 
}

void SyncPipe::Stop() { 
  if(!transport) return;
  transport->Stop();
  initialized = false; 
}

void SyncPipe::Work()
{
  bool readerr=false,writeerr=false;
  if(transport->ReadReady()) {
    const string* res = transport->DoRead();
    if(res) {
      if((*res)[0] != 0) {
	AsyncPipeQueue::OnRead(*res);
	lastReadTime = timer.ElapsedTime();
      }
    }
    else
      readerr = true;
  }
  if(transport->WriteReady()) {
    string str = AsyncPipeQueue::OnWrite();
    if(!str.empty()) {
      lastWriteTime = timer.ElapsedTime();
      if(!transport->DoWrite(str))
	writeerr = true;
    }
  }
  if(readerr) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"SyncPipe::Work(): An error occurred during reading\n");
  }
  if(writeerr) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"SyncPipe::Work(): An error occurred during writing\n");
  }
}


AsyncReaderThread::AsyncReaderThread(double _timeout)
  :AsyncReaderQueue(),initialized(false),timeout(_timeout),lastReadTime(-1)
{
}

AsyncReaderThread::~AsyncReaderThread()
{
  if (initialized) {
    double oldtimeout = timeout;
    //signal that the thread should stop
    timeout = 0;
    //wait for the thread to stop
    ThreadJoin(thread);
  }
}

void AsyncReaderThread::Reset()
{
  Stop();
  AsyncReaderQueue::Reset();
  Start();
}

void* read_worker_thread_func(void * ptr)
{
  AsyncReaderThread* data = reinterpret_cast<AsyncReaderThread*>(ptr);
  while(data->timer.ElapsedTime() < data->lastReadTime + data->timeout) {
    const string* res = data->transport->DoRead();
    if(!res) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"AsyncReaderThread: abnormal termination, read failed\n");
      data->transport->Stop();
      data->initialized = false;
      return NULL;
    }
    if((*res)[0] == 0) continue;

    {
      ScopedLock lock(data->mutex);     
      data->OnRead_NoLock(*res);
      data->lastReadTime = data->timer.ElapsedTime();
    }
  }
  if(data->timeout != 0)
        LOG4CXX_ERROR(KrisLibrary::logger(),"AsyncReaderThread: quitting due to timeout\n");
  return NULL;
}


bool AsyncReaderThread::Start()
{
  if(!initialized) {
    if(!transport) return false;
    if(!transport->Start()) return false;
    lastReadTime = 0;
    thread = ThreadStart(read_worker_thread_func,this);
    initialized = true;
  }
  return true;
}

void AsyncReaderThread::Stop()
{
  if(initialized) {
    double oldtimeout = timeout;
    //signal that the thread should stop
    timeout = 0;
    //wait for the thread to stop
    ThreadJoin(thread);
    transport->Stop();
    initialized = false;
    //restore the old timeout
    timeout = oldtimeout;
  }
}




AsyncPipeThread::AsyncPipeThread(double _timeout)
  :AsyncPipeQueue(),initialized(false),timeout(_timeout),lastReadTime(-1),lastWriteTime(-1)
{
}

AsyncPipeThread::~AsyncPipeThread()
{
  if (initialized) {
    timeout = 0;
    ThreadJoin(workerThread);
    transport->Stop();
  }
}

void AsyncPipeThread::Reset()
{
  Stop();
  AsyncPipeQueue::Reset();
  Start();
}

void* pipe_worker_thread_func(void * ptr)
{
  int iters=0;
  AsyncPipeThread* data = reinterpret_cast<AsyncPipeThread*>(ptr);
  while(data->initialized) {
    double t = data->timer.ElapsedTime();
    if(t >= data->lastReadTime + data->timeout && t >= data->lastWriteTime + data->timeout) {
      //explicit close or timeout
      return NULL;
    }
    if(data->transport->ReadReady()) {
      const string* res = data->transport->DoRead();
      if(!res) {
		LOG4CXX_ERROR(KrisLibrary::logger(),"AsyncPipeThread: abnormal termination, read failed\n");
	data->transport->Stop();
	data->initialized = false;
	return NULL;
      } 
      if((*res)[0] != 0) { //nonempty string
	ScopedLock lock(data->mutex);     
	//don't do _NoLock: the read queue needs locking
	data->OnRead(*res);
	data->lastReadTime = data->timer.ElapsedTime();
	//mutex unlocked
      }
    }
    if(data->transport->WriteReady()) {
      string send;
      bool done = false;
      //suppose multiple items are present... send all of them without yielding the thread
      while(!done) {
        {
  	ScopedLock lock(data->mutex);
  	//don't do _NoLock: the write queue needs locking
  	send = data->OnWrite();
  	data->lastWriteTime = data->timer.ElapsedTime();
  	//mutex unlocked
        }
        if(send.empty()) 
          done = true;
        else {
  	if(!data->transport->DoWrite(send)) {
  	    	  LOG4CXX_ERROR(KrisLibrary::logger(),"AsyncPipeThread: abnormal termination, write failed\n");
  	  data->transport->Stop();
  	  return NULL;
  	}
        }
      }
    }
    else {
      if(data->WriteAvailable()) {
	if(iters % 100 == 0)
	  	  LOG4CXX_ERROR(KrisLibrary::logger(),"AsyncPipeThread: Data is ready to send, waiting for transport to be ready\n");
	iters++;

	ThreadSleep(0.01);
      }
    }
    ThreadYield();
    //ThreadSleep(0.001);
  }
  return NULL;
}

bool AsyncPipeThread::Start()
{
  if(!transport) return false;
  if(!initialized) {
    if(!transport->Start()) return false;
    lastReadTime = lastWriteTime = 0;
    initialized = true;
    workerThread = ThreadStart(pipe_worker_thread_func,this);
  }
  return true;
}

void AsyncPipeThread::Stop()
{
  if(initialized) {
    timeout = 0;
    ThreadJoin(workerThread);
    transport->Stop();
    initialized = false;
  }
}

StreamTransport::StreamTransport(std::istream& _in)
  :in(&_in),out(NULL),format(IntLengthPrepended)
{}

StreamTransport::StreamTransport(std::ostream& _out)
  :in(NULL),out(&_out),format(IntLengthPrepended)
{}

StreamTransport::StreamTransport(std::istream& _in,std::ostream& _out)
  :in(&_in),out(&_out),format(IntLengthPrepended)
{}

bool StreamTransport::ReadReady()
{
  return (in != NULL) && !in->fail();
}

bool StreamTransport::WriteReady()
{
  return (out != NULL) && !out->fail();
}

const string* StreamTransport::DoRead()
{
  if(!in) return NULL;
  buffer = "";
  switch(format) {
  case IntLengthPrepended:
    {
      char buf[4097];
      int len;
      in->read(reinterpret_cast<char*>(&len),sizeof(len));
      if(!*in) return NULL;
      while((int)buffer.length() < len) {
	int size = Max(4096,len-(int)buffer.length());
	in->read(buf,size);
	if(!*in) return NULL;
	buf[size] = 0;
	buffer += buf;
      }
    }
    break;
  case NullTerminated:
    {
      int c;
      while((c = in->get()) != EOF) {
	if(!(*in)) return NULL;
	buffer += c;
      }
    }
    break;
  case Ascii:
    if(!SafeInputString(*in,buffer)) return NULL;
  
    break;
  case Base64:
    {
      string base64;
      (*in) >> base64;
      if(!(*in)) return NULL;
      buffer = FromBase64(base64);
    }
    break;
  }
  return &buffer;
}

bool StreamTransport::DoWrite(const char* msg,int length)
{
  if(!out) return false;
  switch(format) {
  case IntLengthPrepended:
    {
      out->write(reinterpret_cast<const char*>(&length),sizeof(length));
      out->write(msg,length);
    }
    break;
  case NullTerminated:
    {
      out->write(msg,length);
      char c=0;
      out->write(&c,1);
    }
    break;
  case Ascii:
        if(msg[length] != 0) { LOG4CXX_ERROR(KrisLibrary::logger(),"StreamTransport: not writing a NULL-terminated string, Ascii mode\n"); return false; }
    SafeOutputString(*out,buffer);
    (*out) << '\n';
    break;
  case Base64:
    {
      string base64 = ToBase64(msg,length);
      (*out) << base64 << '\n';
    }
    break;
  }
  if(!*out) return false;
  return true;
}

SocketClientTransport::SocketClientTransport(const char* _addr)
  :addr(_addr)
{

}

SocketClientTransport::SocketClientTransport(const char* _addr,SOCKET sock)
  :addr(_addr)
{
  socket.OpenTCPSocket(sock);
  buf.reserve(4096);
}

bool SocketClientTransport::ReadReady()
{
  return socket.ReadAvailable();
}

bool SocketClientTransport::WriteReady()
{
  return socket.WriteAvailable();
}

bool ReadIntPrependedString(File& file,std::string& buf)
{
  int slen;
  if(!file.ReadData(&slen,4)) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Socket::ReadString read length failed\n");
    return false;
  }
  if(slen < 0) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"ReadIntPrependedString read length "<<slen);
    return false;
  }
  buf.resize(slen);
  if(!file.ReadData(&buf[0],slen)) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"ReadIntPrependedString read string failed\n");
    return false;
  }
  return true;
}

const string* SocketClientTransport::DoRead()
{
  ScopedLock lock(mutex);
  if(!ReadIntPrependedString(socket,buf)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"SocketClientTransport: Error reading string on "<<addr<<"...");
    return NULL;
  }
  return &buf;
}

bool SocketClientTransport::Start()
{
  LOG4CXX_INFO(KrisLibrary::logger(),"SocketClientTransport: Creating socket on "<<addr<<"...");
  bool opened = false;
  if(socket.IsOpen()) opened = true;
  while(!opened) {
    if(!socket.Open(addr.c_str(),FILECLIENT)) {
            LOG4CXX_ERROR(KrisLibrary::logger(),"SocketClientTransport: Unable to connect to "<<addr.c_str());
      ThreadSleep(1);
    }
    else
      opened=true;
  }
  void* ptr = socket.FileObjectPointer();
  if(ptr == NULL) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"SocketClientTransport: uh... File returns NULL socket?\n");
    return false;
  }
  SetNodelay(*((SOCKET*)ptr));
  return true;
}

bool SocketClientTransport::Stop()
{
  ScopedLock lock(mutex);
  if(socket.IsOpen()) {
    LOG4CXX_INFO(KrisLibrary::logger(),"SocketClientTransport: Closing "<<addr);
    socket.Close();
  }
  return true;
}

bool SocketClientTransport::DoWrite(const char* str,int length)
{
  ScopedLock lock(mutex);
  assert(sizeof(int)==4);
  if(!socket.WriteData(&length,4)) 
    return false;
  return socket.WriteData(str,length);
}


SocketServerTransport::SocketServerTransport(const char* _addr,int _maxclients)
  :addr(_addr),serversocket(-1),maxclients(_maxclients),currentclient(-1)
{
  buf.reserve(4096);
}

SocketServerTransport::~SocketServerTransport()
{
  Stop();
}

bool SocketServerTransport::Start()
{
  serversocket = Bind(addr.c_str(),true);
  if(serversocket < 0) {
        LOG4CXX_ERROR(KrisLibrary::logger(),"Unable to bind server socket to address "<<addr.c_str());
    return false;
  }
  listen(serversocket,maxclients);
  return true;
}

bool SocketServerTransport::Stop()
{
  ScopedLock lock(mutex);
  for(size_t i=0;i<clientsockets.size();i++)
    clientsockets[i] = NULL;
  clientsockets.resize(0);
  CloseSocket(serversocket);
  return true;
}

bool SocketServerTransport::ReadReady()
{
  return true;
  //return !clientsockets.empty();
}

bool SocketServerTransport::WriteReady()
{
  return !clientsockets.empty();
}

const string* SocketServerTransport::DoRead()
{
  ScopedLock lock(mutex);
  if((int)clientsockets.size() < maxclients) {
    SOCKET clientsock = Accept(serversocket,5.0);
    if(clientsock != INVALID_SOCKET) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Accepted new client on "<<addr.c_str());
      SetNodelay(clientsock);
      clientsockets.push_back(std::unique_ptr<File>(new File));
      clientsockets.back()->OpenTCPSocket(clientsock);
    }
  }
  if(clientsockets.empty()) {
    //tolerant of failed clients
    buf.resize(0);
    return &buf;
  }

  int iters=0;
  currentclient = (currentclient+1) % clientsockets.size();
  while(!clientsockets.empty()) {
    //loop through the sockets
    if(!clientsockets[currentclient]->ReadAvailable()) {
      currentclient = (currentclient+1) % clientsockets.size();
      //if we've looped through all clients, break
      iters++;
      if(iters == (int)clientsockets.size()) break;
      continue;
    }
    if(ReadIntPrependedString(*clientsockets[currentclient],buf)) {
      return &buf;
    }
    //close the client
    LOG4CXX_INFO(KrisLibrary::logger(),"SocketServerTransport: Lost client "<<currentclient);
    clientsockets[currentclient] = NULL;
    clientsockets[currentclient].reset(clientsockets.back().release());
    clientsockets.resize(clientsockets.size()-1);
    if(clientsockets.empty()) {
      currentclient = -1;
      break;
    }
    currentclient = currentclient % clientsockets.size();
  }
  //should we be tolerant of failed clients?
  buf.resize(0);
  return &buf;
  return NULL;
}

bool SocketServerTransport::DoWrite(const char* str,int length)
{
  ScopedLock lock(mutex);
  if((int)clientsockets.size() < maxclients) {
    SOCKET clientsock = Accept(serversocket,5.0);
    if(clientsock != INVALID_SOCKET) {
      LOG4CXX_INFO(KrisLibrary::logger(),"Accepted new client on "<<addr.c_str());
      SetNodelay(clientsock);
      clientsockets.push_back(std::unique_ptr<File>(new File));
      clientsockets.back()->OpenTCPSocket(clientsock);
    }
  }
  if(clientsockets.empty()) {
    //tolerant of failed clients
    return true;
  }

  for(size_t i=0;i<clientsockets.size();i++) {
    if(!clientsockets[i]->WriteData(&length,4) || !clientsockets[i]->WriteData(str,length)) {
      LOG4CXX_INFO(KrisLibrary::logger(),"SocketServerTransport: Lost client "<<(int)i);
      //close the client
      clientsockets[i] = NULL;
      clientsockets[i].reset(clientsockets.back().release());
      clientsockets.resize(clientsockets.size()-1);
      i--;
    }
  }
  //should we be tolerant of failed clients?
  return true;
  if(clientsockets.empty()) return false;
  return true;
}
