#ifndef THREAD_UTILS_H
#define THREAD_UTILS_H

#ifndef USE_BOOST_THREADS
#ifndef USE_PTHREADS
#ifdef WIN32
#define USE_BOOST_THREADS 1
#define USE_PTHREADS 0
#else
#define USE_BOOST_THREADS 0
#define USE_PTHREADS 1
#endif
#endif //USE_PTHREADS
#endif //USE_BOOST_THREADS

#if USE_BOOST_THREADS
#include <boost/thread.hpp>
typedef boost::thread Thread;
typedef boost::mutex Mutex;
typedef boost::mutex::scoped_lock ScopedLock;
inline Thread ThreadStart(void* (*fn)(void*),void* data=NULL) { return boost::thread(fn,data); }
inline void ThreadJoin(Thread& thread) { thread.join(); }
inline void ThreadYield() { boost::this_thread::yield(); }

#endif //USE_BOOST_THREADS

#if USE_PTHREADS
#include <pthread.h>
typedef pthread_t Thread;
inline Thread ThreadStart(void* (*fn)(void*),void* data=NULL) {
	pthread_t thread;
	pthread_create(&thread,NULL,fn,data);
	return thread;
}
inline void ThreadJoin(Thread& thread) { pthread_join(thread,NULL); }
inline void ThreadYield() { pthread_yield(); }
struct Mutex
{
	Mutex() { mutex = PTHREAD_MUTEX_INITIALIZER; }
	pthread_mutex_t mutex;
};

struct ScopedLock
{
  ScopedLock(Mutex& _mutex) :mutex(_mutex) { pthread_mutex_lock(&mutex.mutex);  }
  ~ScopedLock() { pthread_mutex_unlock(&mutex.mutex); }
  Mutex& mutex;
};

#endif //USE_PTHREADS

#ifdef WIN32
inline void ThreadSleep(double duration) { Sleep(int(duration*1000)); }
#else
#include <unistd.h>
inline void ThreadSleep(double duration) { usleep(int(duration*1000000)); }
#endif

#endif //THREAD_UTILS_H
