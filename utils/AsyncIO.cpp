#include "AsyncIO.h"
#include <iostream>

AsyncReaderQueue::AsyncReaderQueue(size_t _queueMax)
  :queueMax(_queueMax),msgCount(0)
{}

void AsyncReaderQueue::Enqueue(const string& msg)
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


AsyncReaderThread::AsyncReaderThread(double _timeout)
  :AsyncReaderQueue(1),initialized(false),timeout(_timeout),lastReadTime(-1)
{
  mutex = PTHREAD_MUTEX_INITIALIZER;
}

AsyncReaderThread::~AsyncReaderThread()
{
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
    const char* res = data->Callback();
    if(!res) {
      fprintf(stderr,"AsyncReaderThread: abnormal termination\n");
      return NULL;
    }
    
    pthread_mutex_lock(&data->mutex);
    data->Enqueue(res);
    pthread_mutex_unlock(&data->mutex);
  }
  return NULL;
}


bool AsyncReaderThread::Start()
{
  if(!initialized) {
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
    initialized = false;
  }
}


SocketReadWorker::SocketReadWorker(const char* _addr,double _timeout)
  :AsyncReaderThread(_timeout),addr(_addr)
{
}

const char* SocketReadWorker::Callback()
{
  if(!socket.ReadString(buf,4096)) {
    cout<<"SocketReadWorker: Error reading string..."<<endl;
    return NULL;
  }
  return buf;
}

bool SocketReadWorker::Start()
{
  cout<<"SocketReadWorker: Creating subscriber socket..."<<endl;
  bool opened = false;
  while(!opened) {
    if(!socket.Open(addr.c_str(),FILEREAD)) {
      fprintf(stderr,"SocketReadWorker: Unable to open address... waiting");
      sleep(1);
    }
    else
      opened=true;
  }
  sleep(1);
  return AsyncReaderThread::Start();
}

void SocketReadWorker::Stop()
{
  AsyncReaderThread::Stop();
  cout<<"SocketReadWorker: Destroying socket:"<<endl;
  socket.Close();
  cout<<"SocketReadWorker: Done destroying socket:"<<endl;
}
