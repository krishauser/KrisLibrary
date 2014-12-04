#include "AsyncIO.h"
#include "socketutils.h"
#include <iostream>
#ifndef WIN32
#include <unistd.h>
#endif

AsyncReaderQueue::AsyncReaderQueue(size_t _queueMax)
  :queueMax(_queueMax),msgCount(0)
{}

void AsyncReaderQueue::OnRead(const string& msg)
{
  ScopedLock lock(mutex);
  OnRead_NoLock(msg);
}

void AsyncReaderQueue::OnRead_NoLock(const string& msg)
{
  while(msgQueue.size()>=queueMax)
    msgQueue.pop_front();
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

int AsyncReaderQueue::NewMessageCount()
{
  ScopedLock lock(mutex);
  return (int)msgQueue.size();
}

string AsyncReaderQueue::LastMessage()
{
  ScopedLock lock(mutex);
  return msgLast;
}

vector<string> AsyncReaderQueue::NewMessages()
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
  :queueMax(_queueMax),msgCount(0)
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
  while(msgQueue.size()>=queueMax)
    msgQueue.pop_front();
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

void AsyncWriterQueue::SendMessage(const string& msg)
{
  ScopedLock lock(mutex);
  msgQueue.push_back(msg);
}


AsyncPipeQueue::AsyncPipeQueue(size_t _recvQueueMax,size_t _sendQueueMax)
  :reader(_recvQueueMax),writer(_sendQueueMax)
{}

AsyncReaderThread::AsyncReaderThread(double _timeout)
  :AsyncReaderQueue(1),initialized(false),timeout(_timeout),lastReadTime(-1)
{
}

AsyncReaderThread::~AsyncReaderThread()
{
  Stop();
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
    const char* res = data->transport->DoRead();
    if(!res) {
      fprintf(stderr,"AsyncReaderThread: abnormal termination, read failed\n");
      return NULL;
    }
    if(res[0] == 0) continue;

    {
      ScopedLock lock(data->mutex);     
      data->OnRead_NoLock(res);
      data->lastReadTime = data->timer.ElapsedTime();
    }
  }
  if(data->timeout != 0)
    fprintf(stderr,"AsyncReaderThread: quitting due to timeout\n");
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
  :AsyncPipeQueue(1),initialized(false),timeout(_timeout),lastReadTime(-1)
{
}

AsyncPipeThread::~AsyncPipeThread()
{
  Stop();
}

void AsyncPipeThread::Reset()
{
  Stop();
  AsyncPipeQueue::Reset();
  Start();
}

void* pipe_read_worker_thread_func(void * ptr)
{
  AsyncPipeThread* data = reinterpret_cast<AsyncPipeThread*>(ptr);
  while(data->timer.ElapsedTime() < data->lastReadTime + data->timeout) {
    const char* res = data->transport->DoRead();
    if(!res) {
      fprintf(stderr,"AsyncReaderThread: abnormal termination\n");
      data->Stop();
      return NULL;
    } 
    if(res[0] == 0) continue;
    
    {
      ScopedLock lock(data->mutex);     
	  //don't do _NoLock: the read queue needs locking
      data->OnRead(res);
      data->lastReadTime = data->timer.ElapsedTime();
      //mutex unlocked
    }
    ThreadYield();
  }
  return NULL;
}

void* pipe_write_worker_thread_func(void * ptr)
{
  AsyncPipeThread* data = reinterpret_cast<AsyncPipeThread*>(ptr);
  while(data->timer.ElapsedTime() < data->lastWriteTime + data->timeout) {
    string send;
    {
      ScopedLock lock(data->mutex);
	  //don't do _NoLock: the write queue needs locking
      send = data->OnWrite();
      data->lastWriteTime = data->timer.ElapsedTime();
      //mutex unlocked
    }
    if(!send.empty()) {
      if(!data->transport->DoWrite(send.c_str())) {
	fprintf(stderr,"AsyncPipeThread: abnormal termination\n");
	data->Stop();
	return NULL;
      }
    }
    ThreadYield();
  }
  return NULL;
}


bool AsyncPipeThread::Start()
{
  if(!transport) return false;
  if(!initialized) {
    if(!transport->Start()) return false;
    lastReadTime = lastWriteTime = 0;
    readThread = ThreadStart(pipe_read_worker_thread_func,this);
    writeThread = ThreadStart(pipe_write_worker_thread_func,this);
    initialized = true;
  }
  return true;
}

void AsyncPipeThread::Stop()
{
  if(initialized) {
    timeout = 0;
    ThreadJoin(readThread);
    ThreadJoin(writeThread);
    transport->Stop();
    initialized = false;
  }
}


SocketClientTransport::SocketClientTransport(const char* _addr)
  :addr(_addr)
{
}

const char* SocketClientTransport::DoRead()
{
  ScopedLock(mutex);
  if(!socket.ReadString(buf,4096)) {
    cout<<"SocketReadWorker: Error reading string..."<<endl;
    return NULL;
  }
  return buf;
}

bool SocketClientTransport::Start()
{
  cout<<"SocketClientTransport: Creating socket on "<<addr<<"..."<<endl;
  bool opened = false;
  while(!opened) {
    if(!socket.Open(addr.c_str(),FILECLIENT)) {
      fprintf(stderr,"SocketTransport: Unable to open address... waiting\n");
      ThreadSleep(1);
    }
    else
      opened=true;
  }
  //ThreadSleep(1);
  return true;
}

bool SocketClientTransport::Stop()
{
  cout<<"SocketClientTransport: Stopping."<<endl;
  ScopedLock(mutex);
  socket.Close();
  return true;
}

bool SocketClientTransport::DoWrite(const char* str)
{
  ScopedLock(mutex);
  return socket.WriteString(str);
}


SocketServerTransport::SocketServerTransport(const char* _addr,int _maxclients)
  :addr(_addr),serversocket(-1),maxclients(_maxclients),currentclient(-1)
{
}

SocketServerTransport::~SocketServerTransport()
{
  Stop();
}

bool SocketServerTransport::Start()
{
  serversocket = Bind(addr.c_str(),true);
  if(serversocket < 0) {
    fprintf(stderr,"Unable to bind server socket to address %s\n",addr.c_str());
    return false;
  }
  listen(serversocket,maxclients);
  return true;
}

bool SocketServerTransport::Stop()
{
  ScopedLock(mutex);
  for(size_t i=0;i<clientsockets.size();i++)
    clientsockets[i] = NULL;
  clientsockets.resize(0);
  CloseSocket(serversocket);
  return true;
}

bool SocketServerTransport::ReadReady()
{
  return !clientsockets.empty();
}

bool SocketServerTransport::WriteReady()
{
  return !clientsockets.empty();
}

const char* SocketServerTransport::DoRead()
{
  ScopedLock(mutex);
  if((int)clientsockets.size() < maxclients) {
    SOCKET clientsock = Accept(serversocket,5.0);
    if(clientsock != INVALID_SOCKET) {
      printf("Accepted new client on %s\n",addr.c_str());
      clientsockets.push_back(new File);
      clientsockets.back()->OpenTCPSocket(clientsock);
    }
  }
  if(clientsockets.empty()) {
    //tolerant of failed clients
    buf[0] = 0;
    return buf;
  }

  currentclient = (currentclient+1) % clientsockets.size();
  while(!clientsockets.empty()) {
    //loop through the sockets
    if(clientsockets[currentclient]->ReadString(buf,4096))
      return buf;
    //close the client
    printf("SocketServer: Lost client %d\n",currentclient);
    clientsockets[currentclient] = NULL;
    clientsockets[currentclient] = clientsockets.back();
    clientsockets.resize(clientsockets.size()-1);
    if(clientsockets.empty()) {
      currentclient = -1;
      break;
    }
    currentclient = currentclient % clientsockets.size();
  }
  //should we be tolerant of failed clients?
  buf[0] = 0;
  return buf;
  return NULL;
}

bool SocketServerTransport::DoWrite(const char* str)
{
  ScopedLock(mutex);
  if((int)clientsockets.size() < maxclients) {
    SOCKET clientsock = Accept(serversocket,5.0);
    if(clientsock != INVALID_SOCKET) {
      printf("Accepted new client on %s\n",addr.c_str());
      clientsockets.push_back(new File);
      clientsockets.back()->OpenTCPSocket(clientsock);
    }
  }
  if(clientsockets.empty()) {
    //tolerant of failed clients
    return true;
  }

  for(size_t i=0;i<clientsockets.size();i++) {
    if(!clientsockets[i]->WriteString(str)) {
      printf("SocketServer: Lost client %d\n",i);
      //close the client
      clientsockets[i] = NULL;
      clientsockets[i] = clientsockets.back();
      clientsockets.resize(clientsockets.size()-1);
      i--;
    }
  }
  //should we be tolerant of failed clients?
  return true;
  if(clientsockets.empty()) return false;
  return true;
}
