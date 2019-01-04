#include <KrisLibrary/Logger.h>
#include "SimpleParser.h"
#include <iostream>
#include <string>
#include <stdio.h>

SimpleParser::SimpleParser(istream& _in)
  :in(_in),lineno(1)
{}

bool SimpleParser::EatSpace()
{
  int c;
  while((c=in.peek()) != EOF) {
    if(!isspace(c))
      return true;
    c=in.get();
  }
  return true;
}

bool SimpleParser::ReadLine(string& str)
{
  //read until comment characters or eof
  str.erase();
  int c;
  while((c=in.peek()) != EOF) {
    if(!in) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleParser: Error while reading line!");
      return false;
    }
    if(c == '\\') { //read literal (or skip endline)
      c=in.get();
      c=in.peek();
      if(c == '\r') {  //carriage return, this may be a DOS-style file
	c = in.get();
	c = in.peek();
      }
      if(c == EOF) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleParser: literal character \\ before end of file");
	return false;
      }
      else if(c == '\n') { } //skip
      else str += c;
    }
    else if(c == '\n') {
      return true;
    }
    else str += c;
    c=in.get();
  }
  LOG4CXX_INFO(KrisLibrary::logger(),"SimpleParser: Reached end of file, line \""<<str<<"\"");
  return true;
}

bool SimpleParser::Read()
{
  int c;
  int mode=0;  //0 whitespace/idle, 1 comment, 2 token, 3 punct
  string str;
  while((c=in.peek()) != EOF) {
    if(!in) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleParser: Error while reading characters!");
      return false;
    }
    switch(mode) {
    case 0:
      if(IsSpace(c)) {  //continue
      }
      else if(IsComment(c)) mode=1;
      else if(IsToken(c)) { str+=c; mode=2;  }
      else if(IsPunct(c)) { str+=c; mode=3; }
      else {
	LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleParser: Illegal character "<<(char)c);
	return false;
      }
      break;
    case 1:   //comment
      if(c=='\n') 
	mode=0;
      break;
    case 2:    //reading tokens
      if(IsToken(c)) {str+=c;}
      else {
	Result res=InputToken(str);
	if(res == Stop) return true;
	else if(res == Error) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleParser: Error on token "<<str.c_str());
	  return false;
	}
	str.erase();
	if(c == '\n') {
	  Result res=InputEndLine();
	  if(res==Stop) return true;
	  else if(res == Error) {
	    LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleParser: Error on endline at line "<<lineno);
	    return false;
	  }
	  lineno++;
	  mode=0;
	}
	else if(IsSpace(c)) mode=0;
	else if(IsComment(c)) mode=1;
	else if(IsPunct(c)) { str+=c; mode=3;	}
	else {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleParser: Illegal character "<<(char)c);
	  return false;
	}
      }
      break;
    case 3:
      if(IsPunct(c)) {str+=c;}
      else {
	Result res=InputPunct(str);
	if(res == Stop) return true;
	else if(res == Error) {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleParser: Error on token "<<str.c_str());
	  return false;
	}
	str.erase();
	if(IsSpace(c)) mode=0;
	else if(IsComment(c)) mode=1;
	else if(IsToken(c)) { str+=c; mode=2; }
	else {
	  LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleParser: Illegal character "<<(char)c);
	  return false;
	}
      }
      break;
    }
    if(c == '\n') {
      Result res=InputEndLine();
      if(res==Stop) return true;
      else if(res == Error) {
	LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleParser: Error on endline at line "<<lineno);
	return false;
      }
      lineno++;
    }
    c = in.get();
  }
  if(!str.empty()) {
    Result res=InputToken(str);
    if(res == Stop) return true;
    else if(res == Error) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleParser: Error on token "<<str.c_str());
      return false;
    }
    else {
      LOG4CXX_ERROR(KrisLibrary::logger(),"SimpleParser: Incomplete read at EOF, string "<<str.c_str());
      return false;
    }
  }
  in.get();
  assert(!in);
  return true;
}

