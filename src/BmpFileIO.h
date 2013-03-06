/* 
 * File:   BmpFileIO.h
 * Author: lubos
 *
 * Created on January 21, 2013, 6:58 AM
 */

#ifndef BMPFILEIO_H
#define	BMPFILEIO_H

#include <cstdio>
#include <stdexcept>

#include "Image.h"

template <class T>
class BmpFileIO
{
private:
    static BmpFileIO<T>* m_instance;
    
    static bool m_destroyed;

    FILE* m_file;

private:

    BmpFileIO() {}

    static void onDestroy();

    short readShort();
    
    long readLong();
    
    void skipBytes(int numChars);    

    void writeLong(long data);
    
    void writeShort(short data);
    
public:
    
    virtual ~BmpFileIO() {}

    static BmpFileIO* getInstance();
    
    Image<T>* loadFile(const char* filename);    

    void writeFile(const char* filename, Image<T>* image);    
};

template <class T>
BmpFileIO<T>* BmpFileIO<T>::m_instance = NULL;

template <class T>
bool BmpFileIO<T>::m_destroyed = false;

template <class T>
BmpFileIO<T>* BmpFileIO<T>::getInstance()
{
    if (m_instance == NULL && !m_destroyed)
    {
        m_instance = new BmpFileIO();
        atexit(&onDestroy);
    }
    return m_instance;
}

template <class T>
void BmpFileIO<T>::onDestroy()
{
    SAFE_DELETE(m_instance);
    m_instance = NULL;
    m_destroyed = true;
}

template <class T>
short BmpFileIO<T>::readShort()
{
    // read a 16 bit integer
    unsigned char lowByte, hiByte;
    lowByte = fgetc(m_file); // Read the low order byte (little endian form)
    hiByte = fgetc(m_file); // Read the high order byte

    // Pack together
    short ret = hiByte;
    ret <<= 8;
    ret |= lowByte;
    return ret;
}

template <class T>
long BmpFileIO<T>::readLong()
{
    // Read in 32 bit integer
    unsigned char byte0, byte1, byte2, byte3;

    byte0 = fgetc(m_file); // Read bytes, low order to high order
    byte1 = fgetc(m_file);
    byte2 = fgetc(m_file);
    byte3 = fgetc(m_file);

    // Pack together
    long ret = byte3;
    ret <<= 8;
    ret |= byte2;
    ret <<= 8;
    ret |= byte1;
    ret <<= 8;
    ret |= byte0;
    return ret;
}

template <class T>
void BmpFileIO<T>::writeLong(long data)
{
    // Read in 32 bit integer
    unsigned char byte0, byte1, byte2, byte3;
    byte0 = static_cast<unsigned char> (data & 0x000000ff); // low first than high
    byte1 = static_cast<unsigned char> ((data >> 8) & 0x000000ff);
    byte2 = static_cast<unsigned char> ((data >> 16) & 0x000000ff);
    byte3 = static_cast<unsigned char> ((data >> 24) & 0x000000ff);

    fputc(byte0, m_file);
    fputc(byte1, m_file);
    fputc(byte2, m_file);
    fputc(byte3, m_file);
}

template <class T>
void BmpFileIO<T>::writeShort(short data)
{
    // Read in 32 bit integer
    unsigned char byte0, byte1;
    byte0 = data & 0x000000ff; // low first than high
    byte1 = (data >> 8) & 0x000000ff;

    fputc(byte0, m_file);
    fputc(byte1, m_file);
}

template <class T>
void BmpFileIO<T>::skipBytes(int numChars)
{
    for (int i = 0; i < numChars; i++)
    {
        fgetc(m_file);
    }
}

template <class T>
Image<T>* BmpFileIO<T>::loadFile(const char* filename)
{
    m_file = fopen(filename, "rb");
    if (!m_file)
    {
        //throw std::runtime_error("Unable to open file: " + filename);
        throw std::runtime_error("Unable to open file: ...");
    }

    unsigned int width = 0;
    unsigned int height = 0;

    bool fileFormatOK = false;
    int bChar = fgetc(m_file);
    int mChar = fgetc(m_file);

    if (bChar == 'B' && mChar == 'M')
    {
        skipBytes(16); //skip header 16bytes
        width = readLong();
        height = readLong();
        skipBytes(2);
        int bitsPerPixel = readShort();
        skipBytes(24); // Skip info 24bytes

        if (width > 0 && width <= 100000
                && height > 0 && height <= 100000
                && bitsPerPixel == 24 && !feof(m_file)) //:)
        {
            fileFormatOK = true;
        }
    }

    if (!fileFormatOK)
    {
        fclose(m_file);
        //throw std::runtime_error("Not a valid 24-bit bitmap file: " + filename);
        throw std::runtime_error("Not a valid 24-bit bitmap file: ...");
    }

    Image<T>* image = new Image<T > (width, height);

    Pixel<T>* pixel = NULL;
    for (unsigned int i = 0; i < height; i++)
    {
        for (unsigned int j = 0; j < width; j++)
        {            
            pixel = new PixelRGB<T > ();
            pixel->r = fgetc(m_file); // Blue color value
            pixel->g = fgetc(m_file); // Green color value
            pixel->b = fgetc(m_file); // Red color value            
            image->setPixel(i, j, pixel);
        }
    }

    if (feof(m_file))
    {
        fclose(m_file);
        //throw std::runtime_error("Premature end of file: " + filename);
        throw std::runtime_error("Premature end of file: ...");
    }

    fclose(m_file);
    return image;
}

template <class T>
void BmpFileIO<T>::writeFile(const char* filename, Image<T>* image)
{
    m_file = fopen(filename, "wb");
    if (!m_file)
    {
        //throw std::runtime_error(stderr, "Unable to open file: %s\n", filename);
        throw std::runtime_error("Unable to open file: ...");
    }

    unsigned int width = image->getWidth();
    unsigned int height = image->getHeight();

    //prepare head
    fputc('B', m_file);
    fputc('M', m_file);
    int rowLen = image->getRowBytes();
    writeLong(40 + 14 + height * rowLen); // Length of file
    writeShort(0); // Reserved for future use
    writeShort(0);
    writeLong(40 + 14); // Offset to pixel data
    writeLong(40); // header length
    writeLong(width); // width in pixels
    writeLong(height); // height in pixels (pos for bottom up)
    writeShort(1); // number of planes
    writeShort(24); // bits per pixel
    writeLong(0); // no compression
    writeLong(0); // not used if no compression
    writeLong(0); // Pixels per meter
    writeLong(0); // Pixels per meter
    writeLong(0); // unused for 24 bits/pixel
    writeLong(0); // unused for 24 bits/pixel

    //store data
    Pixel<T>* pixel = NULL;
    for (unsigned int i = 0; i < height; i++)
    {
        for (unsigned int j = 0; j < width; j++)
        {
            pixel = image->getPixel(i, j);
            fputc((unsigned char)pixel->r, m_file); // Red color value
            fputc((unsigned char)pixel->g, m_file); // Green color value
            fputc((unsigned char)pixel->b, m_file); // Blue color value            
        }
    }
    fclose(m_file);
}

#endif	/* BMPFILEIO_H */

