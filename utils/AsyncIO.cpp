#include "AsyncIO.h"
#include "socketutils.h"
#include <iostream>

AsyncReaderQueue::AsyncReaderQueue(size_t _queueMax)
  :queueMax(_queueMax),msgCount(0)
{}

void AsyncReaderQueue::OnRead(const string& msg)
{
  while(msgQueue.size()>=queueMax)
    msgQueue.pop_front();
  msgQueue.push_back(msg);
  msgCount += 1;
}

void AsyncReaderQueue::Reset()
{
  msgCount=0;
  msgLast="";
  msgQueue.clear(); 
}

vector<string> AsyncReaderQueue::NewMessages()
{
  vector<string> res(msgQueue.begin(),msgQueue.end());
  msgLast=res.back(); 
  msgQueue.clear();
  return res;
}

AsyncWriterQueue::AsyncWriterQueue(size_t _queueMax)
  :queueMax(_queueMax),msgCount(0)
{}


string AsyncWriterQueue::OnWrite()
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
  msgQueue.clear();
  msgCount = 0;
}

void AsyncWriterQueue::SendMessage(const string& msg)
{
  msgQueue.push_back(msg);
}


AsyncPipeQueue::AsyncPipeQueue(size_t _recvQueueMax,size_t _sendQueueMax)
  :reader(_recvQueueMax),writer(_sendQueueMax)
{}


#ifndef WIN32

AsyncReaderThread::AsyncReaderThread(double _timeout)
  :AsyncReaderQueue(1),initialized(false),timeout(_timeout),lastReadTime(-1)
{
  mutex = PTHREAD_MUTEX_INITIALIZER;
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
      fprintf(stderr,"AsyncReaderThread: abnormal termination\n");
      return NULL;
    }
    
    pthread_mutex_lock(&data->mutex);
    data->OnRead(res);
    data->lastReadTime = data->timer.ElapsedTime();
    pthread_mutex_unlock(&data->mutex);
  }
  return NULL;
}


bool AsyncReaderThread::Start()
{
  if(!initialized) {
    if(!transport) return false;
    if(!transport->Start()) return false;
    lastReadTime = 0;
    pthread_create(&thread,NULL,read_worker_thread_func,this);
    initialized = true;
  }
  return true;
}

void AsyncReaderThread::Stop()
{
  if(initialized) {
    timeout = 0;
    pthread_join(thread,NULL);
    transport->Stop();
    initialized = false;
  }
}




AsyncPipeThread::AsyncPipeThread(double _timeout)
  :AsyncPipeQueue(1),initialized(false),timeout(_timeout),lastReadTime(-1)
{
  mutex = PTHREAD_MUTEX_INITIALIZER;
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
    if(!data->transport->ReadReady()) {
      pthread_yield();
    }
    else {
      const char* res = data->transport->DoRead();
      if(!res) {
	fprintf(stderr,"AsyncReaderThread: abnormal termination\n");
	return NULL;
      }    
      pthread_mutex_lock(&data->mutex);
      data->OnRead(res);
      data->lastReadTime = data->timer.ElapsedTime();
      pthread_mutex_unlock(&data->mutex);
    }
  }
  return NULL;
}

void* pipe_write_worker_thread_func(void * ptr)
{
  AsyncPipeThread* data = reinterpret_cast<AsyncPipeThread*>(ptr);
  while(data->timer.ElapsedTime() < data->lastWriteTime + data->timeout) {
    //do the writing
    if(!data->transport->WriteReady()) {
      pthread_yield();
    }
    else {
      string send;
      pthread_mutex_lock(&data->mutex);
      if(data->WriteAvailable()) {
	 send = data->OnWrite();
	 data->lastWriteTime = data->timer.ElapsedTime();
      }
      pthread_mutex_unlock(&data->mutex);
      if(!send.empty()) {
	if(!data->transport->DoWrite(send.c_str())) {
	  fprintf(stderr,"AsyncPipeThread: abnormal termination\n");
	  return NULL;
	}
      }
      else
	pthread_yield();
    }    
  }
  return NULL;
}


bool AsyncPipeThread::Start()
{
  if(!transport) return false;
  if(!initialized) {
    if(!transport->Start()) return false;
    lastReadTime = lastWriteTime = 0;
    pthread_create(&readThread,NULL,pipe_read_worker_thread_func,this);
    pthread_create(&readThread,NULL,pipe_write_worker_thread_func,this);
    initialized = true;
  }
  return true;
}

void AsyncPipeThread::Stop()
{
  if(initialized) {
    timeout = 0;
    pthread_join(readThread,NULL);
    pthread_join(writeThread,NULL);
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
      sleep(1);
    }
    else
      opened=true;
  }
  sleep(1);
  return true;
}

bool SocketClientTransport::Stop()
{
  cout<<"SocketTransport: Destroying socket:"<<endl;
  socket.Close();
  cout<<"SocketTransport: Done destroying socket:"<<endl;
  return true;
}

bool SocketClientTransport::DoWrite(const char* str)
{
  return socket.WriteString(str);
}


SocketServerTransport::SocketServerTransport(const char* _addr,int _maxclients)
  :addr(_addr),serversocket(-1),maxclients(_maxclients),currentclient(-1)
{
}

bool SocketServerTransport::Start()
{
  serversocket = Bind(addr.c_str(),true);
  if(serversocket < 0) {
    fprintf(stderr,"Unable to bind to address %s\n",addr.c_str());
    return false;
  }
  listen(serversocket,maxclients);
  return true;
}

bool SocketServerTransport::Stop()
{
  for(size_t i=0;i<clientsockets.size();i++)
    clientsockets[i] = NULL;
  clientsockets.resize(0);
  close(serversocket);
  return true;
}

bool SocketServerTransport::ReadReady()
{
  if((int)clientsockets.size() < maxclients) {
    int clientsock = Accept(serversocket);
    if(clientsock >= 0) {
      printf("Accepted new client on %s\n",addr.c_str());
      clientsockets.push_back(new File);
      clientsockets.back()->OpenTCPSocket(clientsock);
    }
  }
  return !clientsockets.empty();
}

bool SocketServerTransport::WriteReady()
{
  return !clientsockets.empty();
}

const char* SocketServerTransport::DoRead()
{
  if(clientsockets.empty()) return NULL;
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
    if(clientsockets.empty()) break;
    currentclient = currentclient % clientsockets.size();
  }
  //should we be tolerant of failed clients?
  buf[0] = 0;
  return buf;
  return NULL;
}

bool SocketServerTransport::DoWrite(const char* str)
{
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

#endif // WIN32