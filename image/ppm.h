#ifndef IMAGE_PPM_H
#define IMAGE_PPM_H

#include "image.h"

bool WritePPM_RGB_ASCII(unsigned char image[],int m,int n,const char* file);
bool WritePPM_Grayscale_ASCII(unsigned char image[],int m,int n,const char* file);
bool WritePPM_RGB_Binary(unsigned char image[],int m,int n,const char* file);
bool WritePPM_Grayscale_Binary(unsigned char image[],int m,int n,const char* file);

bool ReadPPM_RGB(unsigned char** image,int* m,int* n,const char* file);
bool ReadPPM_Grayscale(unsigned char** image,int* m,int* n,const char* file);

bool ExportImagePPM(const char* fn, Image& img,bool binary=false);

#endif
