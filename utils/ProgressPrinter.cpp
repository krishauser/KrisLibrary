#include <KrisLibrary/Logger.h>
#include "ProgressPrinter.h"
#include <iostream>
#include <math.h>
using namespace std;

ProgressPrinter::ProgressPrinter(ostream& _out,int _max,int _increments)
  :out(_out),max(_max),increments(_increments),iter(0)
{
}

ProgressPrinter::ProgressPrinter(int _max,int _increments)
  :out(cout),max(_max),increments(_increments),iter(0)
{
}

void ProgressPrinter::Update()
{
  Update(iter+1);
}

void ProgressPrinter::Update(int _iter)
{
  iter=_iter;
  if((iter*increments) / max != ((iter-1)*increments) / max) {
    Print(float(iter)/float(max));
  }
}

void ProgressPrinter::Done()
{
  iter=0;
  out<<"100%"<<endl;
}

void ProgressPrinter::Print(float fraction)
{
  int n=int(floorf(fraction*100.0f+0.5f));
  out<<n<<"%..."; out.flush();
}

