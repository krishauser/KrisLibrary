#ifndef IMAGE_OPENCV_CONVERT_H
#define IMAGE_OPENCV_CONVERT_H

#include <opencv2/core/mat.hpp>
#include "image.h"

///Converts an Image to an OpenCV Mat
inline cv::Mat toMat(const Image& img)
{
    int cvfmt = 0;
    int channels = 0;
    int bytes = 0;
    switch(img.format) {
    case Image::None:
        return cv::Mat();
    case Image::R8G8B8:
        cvfmt = CV_8UC3;
        channels = 3;
        bytes = 1;
        break;
    case Image::A8R8G8B8:
        cvfmt = CV_8UC4;
        channels = 4;
        bytes = 1;
        break;
    case Image::R5G6B5:
    case Image::X1R5G5B5:
        {
        Image cvt;
        cvt.initialize(img.w,img.h,Image::R8G8B8);
        img.blit(cvt);
        return toMat(cvt);
        }
    case Image::A8:
        cvfmt = CV_8U;
        channels = 1;
        bytes = 1;
        break;
    case Image::FloatRGB:
        cvfmt = CV_32FC3;
        channels = 3;
        bytes = 4;
        break;
    case Image::FloatRGBA:
        cvfmt = CV_32FC4;
        channels = 4;
        bytes = 4;
        break;
    case Image::FloatA:
        cvfmt = CV_32F;
        channels = 1;
        bytes = 4;
        break;
    }
    cv::Mat res(img.w,img.h,cvfmt);
    if(channels == 1) {
        memcpy(res.data,img.data,img.w*img.h*channels*bytes);
    }
    else {
        if(bytes == 4) {
            const float* in = (float*)img.data;
            float* out = (float*)res.data;    
            for(int i=0;i<img.w*img.h;i++) {
                if(channels==3) {
                    out[0] = in[2];
                    out[1] = in[1];
                    out[2] = in[0];
                }
                else if(channels==4) {
                    out[0] = in[2];
                    out[1] = in[1];
                    out[2] = in[0];
                    out[3] = in[3];
                }
                in += channels;
                out += channels;
            }
        }
        else {
            assert(bytes==1);
            const unsigned char* in = img.data;
            unsigned char* out = res.data;
            for(int i=0;i<img.w*img.h;i++) {
                if(channels==3) {
                    out[0] = in[2];
                    out[1] = in[1];
                    out[2] = in[0];
                }
                else if(channels==4) {
                    out[0] = in[2];
                    out[1] = in[1];
                    out[2] = in[0];
                    out[3] = in[3];
                }
                in += channels;
                out += channels;
            }
        }
    }
    return res;
}

///Converts an OpenCV Mat to an Image, returning true if successful
inline bool fromMat(const cv::Mat& mat,Image& img)
{
    int channels = mat.channels();
    if(channels != 1 && channels != 3 && channels != 4) {
        //invalid # of channels?
        return false;
    }
    if(mat.depth()==CV_8U) {
        Image::PixelFormat imgformat = (channels == 1 ? Image::A8 : (channels == 3? Image::R8G8B8 : Image::A8R8G8B8));
        img.initialize(mat.rows,mat.cols,imgformat);
        if(channels == 1) 
            memcpy(img.data,mat.data,img.w*img.h*channels);
        else {
            const unsigned char* in = mat.data;
            unsigned char* out = img.data;
            for(int i=0;i<img.w*img.h;i++) {
                if(channels==3) {
                    out[0] = in[2];
                    out[1] = in[1];
                    out[2] = in[0];
                }
                else if(channels==4) {
                    out[0] = in[2];
                    out[1] = in[1];
                    out[2] = in[0];
                    out[3] = in[3];
                }
                in += channels;
                out += channels;
            }
        }
        return true;
    }
    else if(mat.depth() == CV_32F) {
        Image::PixelFormat imgformat = (channels == 1 ? Image::FloatA : (channels == 3? Image::FloatRGB : Image::FloatRGBA));
        img.initialize(mat.rows,mat.cols,imgformat);
        if(channels == 1) 
            memcpy(img.data,mat.data,img.w*img.h*channels*4);
        else {
            const float* in = (float*)mat.data;
            float* out = (float*)img.data;
            for(int i=0;i<img.w*img.h;i++) {
                if(channels==3) {
                    out[0] = in[2];
                    out[1] = in[1];
                    out[2] = in[0];
                }
                else if(channels==4) {
                    out[0] = in[2];
                    out[1] = in[1];
                    out[2] = in[0];
                    out[3] = in[3];
                }
                in += channels;
                out += channels;
            }
        }
        return true;
    }
    else {
        cv::Mat m2;
        mat.convertTo(m2, CV_8UC(channels));
        return fromMat(m2,img);
    }
}

#endif //IMAGE_OPENCV_CONVERT_H