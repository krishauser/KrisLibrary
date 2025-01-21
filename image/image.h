#ifndef IMAGE_IMAGE_H
#define IMAGE_IMAGE_H

class File;

/** @brief A simple image class.
 *
 */
class Image
{
public:
  Image();
  Image(const Image& img);
  Image(Image&& rhs);
  virtual ~Image();
  
  const Image& operator = (const Image&);

  enum PixelFormat {
    None,
    R8G8B8,       //24 bit, 1 byte per channel, red in first byte, green in 2nd, blue in 3rd
    B8G8R8,       //24 bit, 1 byte per channel, blue in first byte, green in 2nd, red in 3rd
    R8G8B8A8,     //32 bit, 1 byte per channel, red in first byte ... alpha in 4th
    B8G8R8A8,     //32 bit, 1 byte per channel, blue in first byte ... alpha in 4th
    R5G6B5,       //16 bit, green has 6 bits
    R5G5B5X1,     //15 bit, 1 padding
    A8,           //8 bits of alpha / grayscale
    A16,          //16 bits of alpha / grayscale
    FloatRGB,     //floating point rgb, with r in the first 4 bytes, g in the 2nd 4, and b in the 3rd 4
    FloatRGBA,    //floating point rgba, with r in the first 4 bytes ... alpha in the 4rd 4 bytes
    FloatA,       //floating point alpha
  };
  static bool isValidFormat(PixelFormat);
  static unsigned int pixelFormatSize(PixelFormat);

  ///Returns -1 on failure, returns size of image on success
  virtual int initialize(int w, int h, PixelFormat format);
  virtual void unload();

  bool Read(const char*);
  bool Write(const char*) const;
  virtual bool Read(File& f);
  virtual bool Write(File& f) const;

  ///Fills in all pixels with the given color
  void clear(int dat = 0);
  ///copies this image, with rect (sx,sy,w,h), onto the dest at dx, dy. 
  ///Default arguments just copy the image to the destination format, which
  ///makes this an easy way to convert formats.
  void blit(Image& dest, int sx = 0, int sy = 0, int w = -1, int h = -1, int dx = 0, int dy = 0) const;

  inline unsigned int pixelSize() const { return pixelFormatSize(format); }  /// < # of bytes per pixel
  inline unsigned int pixelBPP() const { return pixelSize()<<3; }   /// < # of bits per pixel
  unsigned int pixelChannels() const;  /// < # of channels per pixel
  inline unsigned int pitch() const { return pixelSize()*w; }   /// < # of bytes per row
  unsigned char* getData(int x, int y) const;   ///< x is the number of pixels from left to right, y is from top to bottom
  ///Retrieves pixel color as an array of channels in range [0,1]. 
  ///out must point to a buffer with pixelChannels() elements.
  ///
  ///For 1-channel formats, out[0] is set to that value. For 3-channel
  ///formats, the channels 0,1,2 correspond to R,G,B values.  For 4-channel
  ///formats, the channels 0,1,2,3 correspond to R,G,B,A values.
  void getNormalizedColor(int x,int y,float* out) const;
  ///Sets the color of a pixel from an array of channels in range [0,1].
  ///col must point to a buffer with pixelChannels() elements.
  ///
  ///For 1-channel formats, col[0] is that value. For 3-channel
  ///formats, the channels 0,1,2 correspond to R,G,B values.  For 4-channel
  ///formats, the channels 0,1,2,3 correspond to R,G,B,A values.
  void setNormalizedColor(int x,int y,const float* col);

  PixelFormat format;      ///<format of image
  unsigned short w,h;      ///<width and height, in pixels

  ///the image data, stored from left to right, top to bottom
  unsigned char* data;
  unsigned int num_bytes;
};

class ImageMipmapped : public Image
{
public:
  ImageMipmapped();
  explicit ImageMipmapped(const Image&);
  virtual ~ImageMipmapped();

  const ImageMipmapped& operator = (const Image&);
  const ImageMipmapped& operator = (const ImageMipmapped&);

  void createMipmaps();      //creates mipmaps from the first texture level
  virtual void unload();

  virtual bool Read(File& f);

  //mipmap images, starting at 1 after the main image
  unsigned char** mipmap_data;
  unsigned int num_mipmap_levels;
};

typedef unsigned char color_channel_t;

struct Pixel {
  void add(const Pixel& a, const Pixel& b);
  void sub(const Pixel& a, const Pixel& b);
  void mul(const Pixel& a, const Pixel& b);
  void mix(const Pixel& a, const Pixel& b, float u);
  color_channel_t r,g,b,a;
};

class ImageOperator
{
public:
  ImageOperator();
  explicit ImageOperator(const Image&);
  ~ImageOperator();

  void initialize(int w, int h);
  void resize(int w, int h);
  void unload();
  void clear();
  void clear(const Pixel&);

  const ImageOperator& operator = (const Image&);
  const ImageOperator& operator = (const ImageOperator&);
  void output(Image&, Image::PixelFormat fmt) const;

  void stretchBlit(ImageOperator& dest) const;
  void stretchBlitBilinear(ImageOperator& dest) const;

  Pixel& getPixel(int x, int y);
  const Pixel& getPixel(int x, int y) const;
  void sampleNearest(float x, float y, Pixel& out) const;
  void sampleLinear(float x, float y, Pixel& out) const;

  Pixel* pixels;
  unsigned short w,h;
  unsigned int num_pixels;
};

#endif
