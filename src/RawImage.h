/* 
 * File:   Image.h
 * Author: lubos
 *
 * Created on September 21, 2012, 12:26 PM
 */

#ifndef RAW_IMAGE_H
#define RAW_IMAGE_H

#include <cstdio>
#include <iostream>
#include <vector>
#include <cmath>
#include <sensor_msgs/Image.h>

#include "Utils.h"
#include "Pixel.h"

struct Point
{
    int y;
    int x;
};

struct Line
{
    std::vector<Point> points;
};

struct LineVector
{
    int v1;
    int v2;
};

struct DetectInput
{
    int vecY;
    int vecX;
    int chY; int chX;
    int posY;
    int posX;    
};

struct SobelInput
{
    int y1;
    int x1;
    int y2;
    int x2;    
};

#define LINE_LENGTH_TRESHOLD  150
#define COLOR_TRESHOLD 		100

class RawImage 
{
private:
    friend class DetectLineAlgorithm;
    Pixel<double>*** m_imageMatrix; //pod dva nejde:)
    long             m_height;  
    long             m_width;     
    Pixel<double>*   m_baseColor;

    static short readShort(FILE* infile);
    static long  readLong(FILE* infile);
    static void  skipBytes(FILE* infile, int numChars);
    
    static void  writeLong(long data, FILE* outfile);
    static void  writeShort(short data, FILE* outfile);

    static unsigned char convertDoubleToUnsignedChar(double x);  
    
    void allocateImage();
    
public:
    RawImage();
    RawImage(const char* filename);
    RawImage(const sensor_msgs::Image::ConstPtr& img);
    RawImage(int y, int x); // Initialize a blank bitmap of this size.
    ~RawImage();

    bool loadBmpFile(const char *filename); // Loads the bitmap from the specified file
    bool writeBmpFile(const char* filename); // Write the bitmap to the specified file

    long getHeight() const { return m_height; }
    long getWidth()  const { return m_width;  }

    long         getRowBytes() const  { return ((3 * m_width + 3) >> 2) << 2; }
    const void* getImageData() const { return (void*) m_imageMatrix; }

    inline Pixel<double>* getPixel(long x, long y) const;    
    inline double getPixelChannelValue(long x, long y, unsigned int channel) const;    

    void setPixel(long y, long x, double red, double green, double blue);
    void setPixel(long y, long x, unsigned char red, unsigned char green, unsigned char blue); 
    void setPixel(long y, long x, Pixel<double>* pixel) const;

    bool isLoaded() const { return (m_imageMatrix != NULL); }
    void cleanUp();
    
    void blur();
    void sobel();
    void parallelSobel();
    void prewitt();
    void findPlacesWithTheSimilarColor(unsigned char r, unsigned char g, unsigned char b, int tolerance = 20);
    Line* detectLines();

    inline Line* findCorrectLine(int vecY, int vecX, int chY, int chX, int posY, int posX);

    LineVector* getDirectionOfLine(Line* line);
    
    friend std::ostream& operator<<(std::ostream& out, const RawImage& img);
    
    Pixel<double>* convertXyzToLuv(Pixel<double>* xyz);
    Pixel<double>* convertRgbToXyz(Pixel<double>* rgb);
    Pixel<double>* convertRgbToLuv(Pixel<double>* rgb);
    
    double colourDifference(Pixel<double>* rgb1, Pixel<double>* rgb2);            
};



#endif // RGBIMAGE_H



