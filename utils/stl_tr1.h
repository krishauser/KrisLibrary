#ifndef UTILS_CROSS_PLATFORM_UNORDERED_MAP_H
#define UTILS_CROSS_PLATFORM_UNORDERED_MAP_H

//this file helps take care of the cross-platform, cross-compiler differences
//in the STL TR1

#if defined(_MSC_VER)
  #include <unordered_set>
  #include <unordered_map>
  #if _MSC_VER >= 1700
    #define USE_TR1_NAMESPACE 0
  #else
    #define USE_TR1_NAMESPACE 1
  #endif
#elif defined(__APPLE__)
  #include <Availability.h>
  #if __MAC_OS_X_VERSION_MIN_REQUIRED < 1070
    #define USE_TR1_NAMESPACE 1
    #include <tr1/unordered_set>
    #include <tr1/unordered_map>
  #else
    #define USE_TR1_NAMESPACE 0
    #include <unordered_set>
    #include <unordered_map>
  #endif
#else
  #include <tr1/unordered_set>
  #include <tr1/unordered_map>
  #define USE_TR1_NAMESPACE 1
#endif //defined(_MSC_VER) || defined(__APPLE__)

//aliases for the unordered_set, unordered_map, and hash types
#if USE_TR1_NAMESPACE
    #define TR1_NAMESPACE std::tr1
#else
    #define TR1_NAMESPACE std
#endif

#define UNORDERED_SET_NAMESPACE TR1_NAMESPACE
#define UNORDERED_MAP_NAMESPACE TR1_NAMESPACE
#define HASH_NAMESPACE TR1_NAMESPACE
#define UNORDERED_SET_TEMPLATE TR1_NAMESPACE::unordered_set
#define UNORDERED_MAP_TEMPLATE TR1_NAMESPACE::unordered_map
#define HASH_TEMPLATE TR1_NAMESPACE::hash

//helpers for defining new elements of the tr1 namespace
#if USE_TR1_NAMESPACE
#define BEGIN_TR1_NAMESPACE namespace std { namespace tr1 {
#define END_TR1_NAMESPACE } /* namespace std */ } /* namespace tr1 */
#else
#define BEGIN_TR1_NAMESPACE namespace std { 
#define END_TR1_NAMESPACE } /* namespace std */
#endif //USE_TR1_NAMESPACE

#endif // UTILS_CROSS_PLATFORM_UNORDERED_MAP_H
