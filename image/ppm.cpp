#include <KrisLibrary/Logger.h>
#include "ppm.h"
#include "image.h"
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include <vector>
using namespace std;

bool WritePPM_RGB_ASCII(unsigned char image[],int m,int n,const char* file)
{
  FILE* f = fopen(file,"w");
  if(!f) {
    return false;
  }
  fprintf(f,"P3\n#%s\n",file);
  fprintf(f,"%d %d\n",m,n);
  fprintf(f,"255\n");
  int k=0;
  for(int i=0;i<m;i++) {
    for(int j=0;j<n;j++,k+=3) {
      fprintf(f,"%d %d %d  ",image[k],image[k+1],image[k+2]);
    }
    fprintf(f,"\n");
  }
  fclose(f);
  return true;
}

bool WritePPM_Grayscale_ASCII(unsigned char image[],int m,int n,const char* file)
{
  FILE* f = fopen(file,"w");
  if(!f) {
    return false;
  }
  fprintf(f,"P2\n#%s\n",file);
  fprintf(f,"%d %d\n",m,n);
  fprintf(f,"255\n");
  int k=0;
  for(int i=0;i<m;i++) {
    for(int j=0;j<n;j++,k++) {
      fprintf(f,"%d ",image[k]);
    }
    fprintf(f,"\n");
  }
  fclose(f);
  return true;
}

bool WritePPM_RGB_Binary(unsigned char image[],int m,int n,const char* file)
{
  FILE* f = fopen(file,"wb");
  if(!f) {
    return false;
  }
  fprintf(f,"P6\n#%s\n",file);
  fprintf(f,"%d %d\n",m,n);
  fprintf(f,"255\n");
  fwrite(image,m*n*3,1,f);
  fprintf(f,"\n");
  fclose(f);
  return true;
}

bool WritePPM_Grayscale_Binary(unsigned char image[],int m,int n,const char* file)
{
  FILE* f = fopen(file,"wb");
  if(!f) {
    return false;
  }
  fprintf(f,"P5\n#%s\n",file);
  fprintf(f,"%d %d\n",m,n);
  fprintf(f,"255\n");
  fwrite(image,m*n,1,f);
  fprintf(f,"\n");
  fclose(f);
  return true;
}

#define MAX_LEN 1024

string input(const vector<char>& data,size_t& readpos,bool& eof)
{
  string res;
  if(eof) return res;
  int state=0;
  while(readpos < data.size()) {
    if(state == 0) {
      if(data[readpos]=='#') {
        state = 2;
      }
      else if(!isspace(data[readpos])) {
        res += data[readpos];
        state = 1;
      }
    }
    else if(state == 1) {
      if(isspace(data[readpos])) {
        return res;
      }
      else if(data[readpos]=='#') {
        return res;
      }
      else 
        res += data[readpos];
    }
    else if(state == 2) { //comment before reading
      if(data[readpos] == '\n')
        state = 0;
    }
    readpos++;
  }
  eof = true;
  return res;
}

int input_int(const vector<char>& data,size_t& readpos,bool& eof)
{
  string str = input(data,readpos,eof);
  stringstream ss(str);
  int x;
  ss>>x;
  if(ss.bad()) {
    printf("Couldn't input integer from %s\n",str.c_str());
  }
  return x;
}

bool ReadPPM_RGB(unsigned char** image,int* m,int* n,const char* file)
{
  if(image == NULL || m == NULL || n == NULL) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_RGB: Invalid parameters\n");
    return false;
  }
  FILE* f = fopen(file,"rb");
  if(!f) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_RGB: Couldnt open file "<<file);
    return false;
  }
  vector<char> data;
  char buf[MAX_LEN];
  int num_read=0;
  while((num_read=fread(buf,1,MAX_LEN,f))==MAX_LEN) {
    data.insert(data.end(),buf+0,buf+MAX_LEN);
  }
  if(ferror(f)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_RGB: error reading PPM file\n");
    fclose(f);
    return false;
  }
  else {
    data.insert(data.end(),buf+0,buf+num_read);
  }
  fclose(f);

  string tok;
  size_t readpos = 0;
  bool eof = 0;
  tok = input(data,readpos,eof);
  if(tok.length() != 2) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_RGB: PPM file doesn't begin with PX\n");
    return false;
  }
  if(tok[0] != 'P') {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_RGB: PPM file doesn't begin with PX\n");
    return false;
  }
  if(tok[1] == '6') {
    //RGB binary
    *m = input_int(data,readpos,eof);
    *n = input_int(data,readpos,eof);
    if(*m <= 0 || *n<=0) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_RGB: PPM file has invalid size "<<*m<<" "<<*n);
      return false;
    }
    int maxdepth = input_int(data,readpos,eof);
    if(maxdepth != 255) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_RGB: PPM file is not 24 bit\n");
      return false;      
    }
    readpos++;
    int size=(*m)*(*n)*3;
    if(readpos+size > data.size()) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_RGB: error reading PPM file?\n");
      delete [] *image;
      *image = NULL;
      return false;
    }
    *image = new unsigned char[size];
    copy(data.begin()+readpos,data.begin()+readpos+size,(*image));
    //done!
    return true;
  }
  else if(tok[1] == '3') {
    //RGB ASCII
    *m = input_int(data,readpos,eof);
    *n = input_int(data,readpos,eof);
    if(*m <= 0 || *n<=0) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_RGB: PPM file has invalid size "<<*m<<" "<<*n);
      return false;
    }
    int maxdepth = input_int(data,readpos,eof);
    if(maxdepth > 255) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_RGB: PPM file is more than 24 bit\n");
      return false;      
    }
    *image = new unsigned char[(*m)*(*n)*3];
    int k=0;
    for(int i=0;i<*m;i++) {
      for(int j=0;j<*n*3;j++,k++) {
        (*image)[k] = input_int(data,readpos,eof);
      }
    }
    if(eof) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_RGB: error reading PPM file?\n");
      delete [] *image;
      *image = NULL;
      return false;
    }
    //done!
    return true;
  }
  else {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_RGB: PPM file isn't RGB format\n");
    return false;
  }
  return true;
}

bool ReadPPM_Grayscale(unsigned char** image,int* m,int* n,const char* file)
{
  if(image == NULL || m == NULL || n == NULL) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_Grayscale: Invalid parameters\n");
    return false;
  }
  FILE* f = fopen(file,"rb");
  if(!f) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_Grayscale: Couldnt open file "<<file);
    return false;
  }
  vector<char> data;
  char buf[MAX_LEN];
  int num_read=0;
  while((num_read=fread(buf,1,MAX_LEN,f))==MAX_LEN) {
    data.insert(data.end(),buf+0,buf+MAX_LEN);
  }
  if(ferror(f)) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_Grayscale: error reading PPM file\n");
    fclose(f);
    return false;
  }
  else {
    data.insert(data.end(),buf+0,buf+num_read);
  }
  fclose(f);

  string tok;
  size_t readpos = 0;
  bool eof = 0;
  tok = input(data,readpos,eof);
  if(tok.length() != 2) {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_Grayscale: PPM file doesn't begin with PX\n");
    return false;
  }
  if(tok[0] != 'P') {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_Grayscale: PPM file doesn't begin with PX\n");
    return false;
  }
  if(tok[1] == '5') {
    //RGB binary
    *m = input_int(data,readpos,eof);
    *n = input_int(data,readpos,eof);
    if(*m <= 0 || *n<=0) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_Grayscale: PPM file has invalid size "<<*m<<" "<<*n);
      return false;
    }
    int maxdepth = input_int(data,readpos,eof);
    if(maxdepth != 255) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_Grayscale: PPM file is not 8 bit\n");
      return false;      
    }
    readpos++;
    int size=(*m)*(*n);
    if(readpos+size > data.size()) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_Grayscale: error reading PPM file?\n");
      delete [] *image;
      *image = NULL;
      return false;
    }
    *image = new unsigned char[size];
    copy(data.begin()+readpos,data.begin()+readpos+size,(*image));
    //done!
    return true;
  }
  else if(tok[1] == '2') {
    //RGB ASCII
    *m = input_int(data,readpos,eof);
    *n = input_int(data,readpos,eof);
    if(*m <= 0 || *n<=0) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_Grayscale: PPM file has invalid size "<<*m<<" "<<*n);
      return false;
    }
    int maxdepth = input_int(data,readpos,eof);
    if(maxdepth > 255) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_Grayscale: PPM file is more than 8 bit\n");
      return false;      
    }
    *image = new unsigned char[(*m)*(*n)];
    int k=0;
    for(int i=0;i<*m;i++) {
      for(int j=0;j<*n;j++,k++) {
        (*image)[k] = input_int(data,readpos,eof);
      }
    }
    if(eof) {
      LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_Grayscale: error reading PPM file?\n");
      delete [] *image;
      *image = NULL;
      return false;
    }
    //done!
    return true;
  }
  else {
    LOG4CXX_ERROR(KrisLibrary::logger(),"ReadPPM_Grayscale: PPM file isn't RGB format\n");
    return false;
  }
  return true;
}


bool ImportImagePPM(const char* fn, Image& img)
{
  int w,h;
  if(ReadPPM_RGB(&img.data,&w,&h,fn)) {
    img.w = w;
    img.h = h;
    img.format = Image::R8G8B8;
    return true;
  }
  return false;
}
