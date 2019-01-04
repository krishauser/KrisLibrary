#include "SignalHandler.h"
#include <assert.h>
#include <map>
#include <list>
#include <signal.h>
#include <stdlib.h>
using namespace std;

typedef void (*SIGNAL_PROC) (int);
typedef list<SignalHandler*> HandlerList;

map<int,SIGNAL_PROC> initialHandlers;
map<int,list<SignalHandler*> > sigHandlers;

void EraseHandler(HandlerList& l,SignalHandler* h)
{
  list<SignalHandler*>::iterator i,p;
  for(i=l.begin();i!=l.end();i++) {
    if(*i==h) {
      p=i; p--;
      l.erase(i);
      i=p;
    }
  }
}

bool HasHandler(int signum)
{
  return sigHandlers.count(signum) != 0 && !sigHandlers[signum].empty();
}



void theSignalHandlerProc(int signum)
{
  assert(sigHandlers.count(signum) != 0);
  assert(!sigHandlers[signum].empty());
  sigHandlers[signum].back()->OnRaise(signum);
}



SignalHandler::~SignalHandler()
{
  for(map<int,list<SignalHandler*> >::iterator i=sigHandlers.begin();i!=sigHandlers.end();i++) {
    EraseHandler(i->second,this);
    if(i->second.empty()) 
      signal(i->first,initialHandlers[i->first]);
  }
}

void SignalHandler::SetCurrent(int signum)
{
  SIGNAL_PROC prevSignalProc = signal(signum,theSignalHandlerProc);
  if(prevSignalProc == SIG_IGN) {
    signal(signum,SIG_IGN);
    return;
  }
  if(!HasHandler(signum)) {
    initialHandlers[signum] = prevSignalProc;
  }
  sigHandlers[signum].push_back(this);
}

bool SignalHandler::IsCurrent(int signum) const
{
  if(!HasHandler(signum)) return false;
  return (this == sigHandlers[signum].back());
}

void SignalHandler::UnsetCurrent(int signum)
{
  assert(HasHandler(signum));
  assert(sigHandlers[signum].back() == this);
  sigHandlers[signum].pop_back();
  if(sigHandlers[signum].empty()) {
    signal(signum,initialHandlers[signum]);
    initialHandlers[signum]=NULL;
  }
}

SignalHandler* SignalHandler::GetCurrent(int signum)
{
  if(!HasHandler(signum)) return NULL;
  return sigHandlers[signum].back();
}
