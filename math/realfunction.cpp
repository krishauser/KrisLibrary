#include "realfunction.h"
using namespace Math;

std::string ComposeFunction::Label() const
{
  std::string sf=f->Label(),sg=g->Label();
  std::string str=sf; 
  str+="(";
  str += sg;
  str += "(t))";
  return str;
}

void ComposeFunction::PreEval(Real t) 
{ 
  g->PreEval(t);
  gt = g->Eval(t);
  f->PreEval(gt);
}

Real ComposeFunction::Eval(Real t) { return f->Eval(gt); }
Real ComposeFunction::Deriv(Real t) { return f->Deriv(gt)*g->Deriv(t); }
Real ComposeFunction::Deriv2(Real t) { return f->Deriv2(gt)*Sqr(g->Deriv(t)) + f->Deriv(gt)*g->Deriv2(t); }

std::string AddFunction::Label() const
{
  std::string sf=f->Label(),sg=g->Label();
  std::string str=sf;
  str += "+";
  str += sg;
  return str;
}

void AddFunction::PreEval(Real t)
{
  f->PreEval(t); 
  g->PreEval(t);
}

Real AddFunction::Eval(Real t) { return f->Eval(t)+g->Eval(t); }
Real AddFunction::Deriv(Real t) {  return f->Deriv(t)+g->Deriv(t); }
Real AddFunction::Deriv2(Real t) {  return f->Deriv2(t)+g->Deriv2(t); } 

std::string MulFunction::Label() const
{
  std::string sf=f->Label(),sg=g->Label();
  std::string str=sf;
  str += "*";
  str += sg;
  return str;
}

void MulFunction::PreEval(Real t)
{
  f->PreEval(t); 
  g->PreEval(t); 
  ft = f->Eval(t);
  gt = g->Eval(t);
}

Real MulFunction::Eval(Real t) 
{ 
  return ft*gt;
}

Real MulFunction::Deriv(Real t)
{
  return f->Deriv(t)*gt+ft*g->Deriv(t); 
}

Real MulFunction::Deriv2(Real t)
{
  return f->Deriv2(t)*gt+Two*f->Deriv(t)*g->Deriv(t)+ft*g->Deriv2(t); 
}

