#include <KrisLibrary/Logger.h>
#include "Trace.h"
#include <KrisLibrary/File.h>
#include <iostream>
#include <utils.h>
#include <stdarg.h>
#include <string.h>
#include <errors.h>
using namespace std;

_DECLARE_READ_WRITE_FILE_BASIC(StatCollector)

_DEFINE_READ_WRITE_FILE_BASIC(StatCollector)



#define MAXBUF 4096

void Indent(std::ostream& out,int indent)
{
  for(int i=0;i<indent;i++) out<<' ';
}


TraceItem::TraceItem()
:call(NULL)
{}

TraceFunctionCall::TraceFunctionCall()
:type(NULL),parent(NULL),calltime(0),endtime(0)
{}

TraceFunctionCall::TraceFunctionCall(TraceFunction* t,TraceFunctionCall* p)
:type(t),parent(p),calltime(0),endtime(0)
{}

TraceFunctionCall::~TraceFunctionCall()
{
  ClearChildren();
}

void TraceFunctionCall::ClearChildren()
{
  for(size_t i=0;i<children.size();i++)
    SafeDelete(children[i].call);
  children.clear();
}

Trace::Trace()
{
  cur = &root;
}

Trace::~Trace()
{
  Clear();
}

void Trace::Clear()
{
  timer.Reset();
  root.ClearChildren();
  funcs.clear();
  cur = &root;
}

void Trace::ResetTrace()
{
  if(cur != &root) {
    LOG4CXX_WARN(KrisLibrary::logger(),"Trace::ResetLoop: Warning, there looks like an unended call in the trace log");
    abort();
  }
  timer.Reset();
  root.ClearChildren();
  cur = &root;
}

bool Trace::Load(const char* fn)
{
  Clear();
  File f;
  if(!f.Open(fn,FILEREAD)) return false;
  size_t n;
  char buf[MAXBUF];

  if(!ReadFile(f,n)) return false;
  TraceFunction tf;
  for(size_t i=0;i<n;i++) {
    if(!f.ReadString(buf,MAXBUF)) return false;
    tf.name = buf;
    if(!ReadFile(f,tf.stats)) return false;
    funcs.push_back(tf);
  }

  if(!ReadFile(f,n)) return false;
  root.children.resize(n);
  for(size_t i=0;i<root.children.size();i++)
    if(!LoadIter(f,root.children[i],&root)) return false;
  if(!ReadFile(f,root.calltime)) return false;
  if(!ReadFile(f,root.endtime)) return false;
  return true; 
}

bool Trace::LoadIter(File& f,TraceItem& item,TraceFunctionCall* parent)
{
  char type;
  char buf[MAXBUF];
  if(!ReadFile(f,type)) return false;
  if(type == 'c') {
    if(!f.ReadString(buf,MAXBUF)) return false;
    TraceFunction* func = FindFunction(buf);
    item.call = new TraceFunctionCall(func,parent);
    TraceFunctionCall* call = item.call;
    if(!f.ReadString(buf,MAXBUF)) return false;
    call->args = buf;
    size_t n;
    if(!ReadFile(f,n)) return false;
    call->children.resize(n);
    for(size_t i=0;i<n;i++)
      if(!LoadIter(f,call->children[i],call)) return false;
    if(!f.ReadString(buf,MAXBUF)) return false;
    call->ret = buf;
    if(!ReadFile(f,call->calltime)) return false;
    if(!ReadFile(f,call->endtime)) return false;
    return true;
  }
  else if(type == 'x') {
    if(!f.ReadString(buf,MAXBUF)) return false;
    item.text = buf;
    return true;
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"Trace::Load(): Wrong type, got "<<type);
  return false;
}

bool Trace::Save(const char* fn) 
{
  File f;
  if(!f.Open(fn,FILEWRITE)) return false;
  size_t n=funcs.size();
  if(!WriteFile(f,n)) return false;
  for(list<TraceFunction>::iterator i=funcs.begin();i!=funcs.end();i++) {
    TraceFunction* func = &(*i);
    if(!f.WriteString(func->name.c_str())) return false;
    if(!WriteFile(f,func->stats)) return false;
  }

  n=root.children.size();
  if(!WriteFile(f,n)) return false;
  for(size_t i=0;i<root.children.size();i++)
    if(!SaveIter(f,root.children[i])) return false;
  if(!WriteFile(f,root.calltime)) return false;
  if(!WriteFile(f,root.endtime)) return false;
  return true; 
}

bool Trace::SaveIter(File& f,const TraceItem& item)
{
  if(item.call) {
    WriteFile(f,'c');
    TraceFunctionCall* call = item.call;
    Assert(call->type != NULL);
    if(!f.WriteString(call->type->name.c_str())) return false;
    if(!f.WriteString(call->args.c_str())) return false;
    size_t n=call->children.size();
    if(!WriteFile(f,n)) return false;
    for(size_t i=0;i<call->children.size();i++)
      if(!SaveIter(f,call->children[i])) return false;
    if(!f.WriteString(call->ret.c_str())) return false;
    if(!WriteFile(f,call->calltime)) return false;
    if(!WriteFile(f,call->endtime)) return false;
    return true;
  }
  else {
    WriteFile(f,'x');
    return f.WriteString(item.text.c_str());
  }
}

void Trace::DumpStats(ostream& out) const
{
  for(list<TraceFunction>::const_iterator i=funcs.begin();i!=funcs.end();i++) {
    out<<i->name;
    Indent(out,40-i->name.length());
    out<<i->stats.n<<" calls, "<<i->stats.sum<<" s, range ["<<i->stats.xmin<<","<<i->stats.xmax<<"]"<<endl;
  }
}

void Trace::DumpTrace(ostream& out) const
{
  DumpIter(out,&root,-2);
}

void Trace::DumpIter(ostream& out,const TraceFunctionCall* call,int indent) const
{
  if(call->type) {
    Indent(out,indent); out<<call->type->name<<"("<<call->args<<") : "<<call->calltime<<endl;
  }
  for(size_t i=0;i<call->children.size();i++) {
    const TraceItem& item = call->children[i];
    if(item.call)
      DumpIter(out,item.call,indent+2);
    else {
      Indent(out,indent+2); out<<item.text<<endl;
    }
  }
  if(call->type) {
    Indent(out,indent); 
    if(!call->ret.empty())
      out<<"~"<<call->type->name<<" ("<<call->ret<<") : "<<call->endtime<<endl;
  }
  if(call == cur) {
    Indent(out,indent); out<<"*** current position ***"<<endl;
  }
}

void Trace::CallFmt(const char* function,const char* fmt,...)
{
  char buf [MAXBUF];
  va_list args;
	va_start(args, fmt);
#ifdef _WIN32
	_vsnprintf(buf, MAXBUF, fmt, args);
#else
  vsnprintf(buf, MAXBUF, fmt, args);
#endif
  va_end(args);
  Call(function,buf);
}

void Trace::Call(const char* function,const char* args)
{
  TraceFunction* func = FindFunction(function);
  if(!func) {
    funcs.resize(funcs.size()+1);
    func = &funcs.back();
    func->name = function;
  }
  TraceItem item;
  item.call = new TraceFunctionCall(func,cur);
  if(args) item.call->args = args;
  cur->children.push_back(item);
  BeginCall(item.call);
  cur = item.call;
}

void Trace::EndCallFmt(const char* function,const char* fmt,...)
{
  char buf [MAXBUF];
  va_list args;
	va_start(args, fmt);
#ifdef _WIN32
	_vsnprintf(buf, MAXBUF, fmt, args);
#else
  vsnprintf(buf, MAXBUF, fmt, args);
#endif
  va_end(args);
  EndCall(function,buf);
}

void Trace::EndCall(const char* function,const char* ret)
{
  TraceFunction* func = FindFunction(function);
  if(!func) {
    DumpTrace(cout);
    FatalError("Trace::EndCall(): attempted to end a nonexistent function");
  }
  TraceFunctionCall* pcall=FindParentIter(cur,func);
  if(!pcall) {
    DumpTrace(cout);
    FatalError("Trace::EndCall(): Fatal error, attempted to end a non-parent function");
  }
  while(cur != pcall) {
    cur->ret = "implicit";
    EndCall(cur);
    cur = cur->parent;
  }
  if(ret) cur->ret = ret;
  else cur->ret = "void";
  EndCall(cur);
  cur = cur->parent;
}

void Trace::Log(const char* txt)
{
  TraceItem item; item.text = txt;
  cur->children.push_back(item);
}

TraceFunction* Trace::FindFunction(const char* func)
{
  //really stupid, linear time algorithm for now
  for(list<TraceFunction>::iterator i=funcs.begin();i!=funcs.end();i++)
    if(0 == strcmp(i->name.c_str(),func)) return &(*i);
  return NULL;
}

TraceFunctionCall* Trace::FindParentIter(TraceFunctionCall* call,TraceFunction* func)
{
  if(!call) return NULL;
  if(call->type == func) return call;
  return FindParentIter(call->parent,func);
}

void Trace::BeginCall(TraceFunctionCall* call)
{
  call->calltime = timer.ElapsedTime();
}

void Trace::EndCall(TraceFunctionCall* call)
{
  call->endtime = timer.ElapsedTime();
  call->type->stats<<(call->endtime-call->calltime);
}
