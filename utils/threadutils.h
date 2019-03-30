#ifndef THREAD_UTILS_H
#define THREAD_UTILS_H

#ifndef USE_CPP_THREADS
#ifndef USE_BOOST_THREADS
#ifndef USE_PTHREADS
#ifdef _WIN32
#if (_MSC_VER>=1700)
#define USE_CPP_THREADS 1
#define USE_BOOST_THREADS 0
#define USE_PTHREADS 0
#else
#define USE_CPP_THREADS 0
#define USE_BOOST_THREADS 1
#define USE_PTHREADS 0
#endif
#else
#define USE_CPP_THREADS 0
#define USE_BOOST_THREADS 0
#define USE_PTHREADS 1
#endif
#endif //USE_PTHREADS
#endif //USE_BOOST_THREADS
#endif //USE_CPP_THREADS


#if USE_CPP_THREADS
#include <thread>
#include <condition_variable>
#include <mutex>
#include <iostream>
typedef std::thread Thread;
struct Mutex
{
  Mutex() { }
  ~Mutex() { mutex.~mutex(); }
  void lock() { mutex.lock(); }
  bool trylock() { return (mutex.try_lock() == 0); }
  void unlock() { mutex.unlock(); }
  std::mutex mutex;
};
struct ScopedLock{
  explicit ScopedLock(Mutex& _mutex) :mutex(_mutex) { mutex.lock(); }
  ~ScopedLock() { mutex.unlock(); }
  Mutex& mutex;
};
typedef std::condition_variable Condition;
inline Thread ThreadStart(void* (*fn)(void*), void* data = NULL) { return std::thread(fn, data); }
inline void ThreadJoin(Thread& thread) { thread.join(); }
inline void ThreadYield() { std::this_thread::yield(); }

#endif //USE_CPP_THREADS

#if USE_BOOST_THREADS
#include <boost/thread.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
typedef boost::thread Thread;
typedef boost::mutex Mutex;
typedef boost::mutex::scoped_lock ScopedLock;
typedef boost::interprocess::interprocess_condition Condition;
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
  ~Mutex() { pthread_mutex_destroy(&mutex); }
  void lock() { pthread_mutex_lock(&mutex);  }
  bool trylock() { return (pthread_mutex_lock(&mutex) == 0); }
  void unlock() { pthread_mutex_unlock(&mutex);  }
  pthread_mutex_t mutex;
};

struct ScopedLock
{
  ScopedLock(Mutex& _mutex) :mutex(_mutex) { pthread_mutex_lock(&mutex.mutex);  }
  ~ScopedLock() { pthread_mutex_unlock(&mutex.mutex); }
  Mutex& mutex;
};

struct Condition
{
  Condition() { cond = PTHREAD_COND_INITIALIZER; }
  ~Condition() { pthread_cond_destroy(&cond); }
  void wait(ScopedLock& lock) { pthread_cond_wait(&cond,&lock.mutex.mutex); }
  void notify_one() { pthread_cond_signal(&cond); }
  void notify_all() { pthread_cond_broadcast(&cond); }

  pthread_cond_t cond;
};

#endif //USE_PTHREADS

#ifdef WIN32
void ThreadSleep(double duration);
#else
#include <unistd.h>
inline void ThreadSleep(double duration) { usleep(int(duration*1000000)); }
#endif

#endif //THREAD_UTILS_H
