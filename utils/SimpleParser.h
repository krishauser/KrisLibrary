#ifndef UTILS_SIMPLE_PARSER_H
#define UTILS_SIMPLE_PARSER_H

#include <iosfwd>
#include <ctype.h>
#include <assert.h>
#include <string>
using namespace std;

/** @ingroup Utils
 * @brief A simple class to break files into tokens, punctuation, comments, 
 * and whitespace.
 *
 * Non-comment lines are broken up into tokens and punctuation, possibly
 * separated by whitespace.  Tokens and punctuation are contiguous blocks
 * of token and punctuation characters.
 *
 * Comments take up an entire line.  The format is whitespace, followed by
 * comment char, and consumes characters up to the endline.
 * 
 * Whitespace, comment, token, and punctuation characters are determined by
 * the overrideable IsSpace, IsComment, IsToken, and IsPunct methods.
 * By default, tokens are alphanumeric characters, whitespaces are spaces,
 * tabs, and newlines, and comments start with '#'.  Punctuation is everything
 * else.
 *
 * Callbacks are defined by overriding InputToken, InputPunct, and
 * InputEndline in a subclass.  They can return Continue to continue
 * reading the file, Stop to stop reading the file (with success), and Error
 * to stop reading the file with failiure.
 */
class SimpleParser
{
public:
  enum Result { Continue, Stop, Error };
  SimpleParser(istream& in);
  virtual ~SimpleParser() {}
  virtual bool IsSpace(char c) const { return (bool)isspace(c); }
  virtual bool IsComment(char c) const { return c=='#'; }
  virtual bool IsToken(char c) const { return (bool)isalnum(c); }
  virtual bool IsPunct(char c) const { return !IsSpace(c) && !IsComment(c) && !IsToken(c); }
  virtual Result InputToken(const string& word)=0;
  virtual Result InputPunct(const string& punct)=0;
  virtual Result InputEndLine()=0;

  bool Read();

  //called by sub classes on error, stop
  bool EatSpace();
  bool ReadLine(string& str);

  istream& in;
  int lineno;
};

#endif
