#include "MatrixEquationPrinter.h"
#include <iostream>
#include "ASCIIShade.h"
#include <string.h>
#include <assert.h>
#include <iomanip>
#include <vector>
using namespace Math;
using namespace std;


#define kMatrixPrecision 4

EquationTerm::EquationTerm()
  :matrix(NULL),vector(NULL),text(NULL),scalar(0),
   transpose(false),ASCIIshade(false)
{}

int EquationTerm::Height() const
{
  if(matrix)
    return (transpose? matrix->n : matrix->m);
  else if(vector)
    return (transpose? 1 : vector->n);
  else if(text)
    return 1;
  else return 1;
}

int EquationTerm::Width() const
{
  if(matrix)
    return (transpose? matrix->m : matrix->n) * (kMatrixPrecision+1) + 2;
  else if(vector)
    return (transpose? vector->n : 1) * (kMatrixPrecision+1) + 2;
  else if(text)
    return (int)strlen(text);
  else return (kMatrixPrecision);
}

void EquationTerm::PrintLine(int line,ostream& out) const
{
  if(line < 0 || line >= Height()) {
    int w = Width();
    for(int i=0;i<w;i++) out<<' ';
    return;
  }
  streamsize p = out.precision();
  streamsize w = out.width();
  if(matrix) {
    if(ASCIIshade) {
      if(transpose) {
	Vector v;
	matrix->getColRef(line,v);
	OutputASCIIShade(out,v,matrix->maxAbsElement());
      }
      else {
	Vector v;
	matrix->getRowRef(line,v);
	OutputASCIIShade(out,v,matrix->maxAbsElement());
      }
    }
    else {
      out<<"[";
      out.precision(kMatrixPrecision);
      if(transpose) {
	for(int i=0;i<matrix->m;i++)
	  out<<setw(kMatrixPrecision)<<left<<(*matrix)(i,line)<<" ";
      }
      else {
	for(int j=0;j<matrix->n;j++)
	  out<<setw(kMatrixPrecision)<<left<<(*matrix)(line,j)<<" ";
      }
      out.precision(p);
      out.width(w);
      out<<"]";
    }
  }
  else if(vector) {
    if(ASCIIshade) {
      if(transpose) {
	assert(line == 0);
	OutputASCIIShade(out,*vector);
      }
      else {
	OutputASCIIShade(out,(*vector)(line) / vector->maxAbsElement());
      }
    }
    else {
      out<<"[";
      out.precision(kMatrixPrecision);
      out.width(kMatrixPrecision);
      if(transpose) {
	assert(line == 0);
	for(int i=0;i<vector->n;i++)
	  out<<setw(kMatrixPrecision)<<left<<(*vector)(i)<<" ";
      }
      else {
	out<<setw(kMatrixPrecision)<<left<<(*vector)(line)<<" ";
      }
      out.precision(p);
      out.width(w);
      out<<"]";
    }
  }
  else if(text) {
    out<<text;
  }
  else {
    out.precision(kMatrixPrecision);
    out<<setw(kMatrixPrecision)<<left<<scalar;
    out.precision(p);
    out.width(w);
  }
}

void MatrixEquationPrinter::PushMatrix(const Matrix& x)
{
  EquationTerm term;
  term.matrix = &x;
  terms.push_back(term);
}

void MatrixEquationPrinter::PushMatrixTranspose(const Matrix& x)
{
  EquationTerm term;
  term.matrix = &x;
  term.transpose = true;
  terms.push_back(term);
}

void MatrixEquationPrinter::PushVector(const Vector& x)
{
  EquationTerm term;
  term.vector = &x;
  terms.push_back(term);
}

void MatrixEquationPrinter::PushText(const char* x)
{
  EquationTerm term;
  term.text = x;
  terms.push_back(term);
}

void MatrixEquationPrinter::PushScalar(Real x)
{
  EquationTerm term;
  term.scalar = x;
  terms.push_back(term);
}


void MatrixEquationPrinter::Print(std::ostream& out) const
{
  int maxHeight=0;
  size_t numTerms = terms.size();
  vector<int> heights(numTerms,0);
  vector<int> wOffsets(numTerms,0);

  list<EquationTerm>::const_iterator i;
  size_t n=0;
  for(i=terms.begin();i!=terms.end();i++,n++) {
    heights[n] = i->Height();
    if(heights[n] > maxHeight) maxHeight = heights[n];
    if(n+1 < numTerms)
      wOffsets[n+1] = wOffsets[n] + i->Width();
  }

  //each term n begins on line (maxHeight-height[n])/2,
  //ends on (maxHeight+height[n])/2

  for(int j=0;j<maxHeight;j++) {
    n=0;
    for(i=terms.begin();i!=terms.end();i++,n++) {
      int line = j - (maxHeight-heights[n])/2;
      i->PrintLine(line,out);
      out<<" ";
    }
    out<<endl;
  }
}

void MatrixEquationPrinter::Clear()
{
  terms.clear();
}
