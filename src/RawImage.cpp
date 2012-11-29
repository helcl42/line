#include "RawImage.h"

RawImage::RawImage()
{
    m_height = 0;
    m_width = 0;
    m_imageMatrix = NULL;    
    m_baseColor = new Pixel<double>();
    m_baseColor->g = 0;
    m_baseColor->b = 0;
    m_baseColor->r = 0;
}

RawImage::RawImage(int width, int height)
{
    if (width > 0 && height > 0)
    {
        m_width = width;
        m_height = height;

        allocateImage();
    }
    m_baseColor = new Pixel<double>();
    m_baseColor->g = 0;
    m_baseColor->b = 0;
    m_baseColor->r = 0;
}

RawImage::RawImage(const sensor_msgs::Image::ConstPtr& img)
{
    if (img->width > 0 && img->height > 0)
    {
        m_width = img->width;
        m_height = img->height;

        std::vector<unsigned char> data = img->data;

        allocateImage();

        int index = 0;
        Pixel<double>* pixel = NULL;
        for (int i = m_height - 1; i >= 0; --i)
        {
            for (int j = m_width - 1; j >= 0; --j, index += 3)
            {
                pixel = new Pixel<double>();
                pixel->r = data[index];
                pixel->g = data[index + 1];
                pixel->b = data[index + 2];
                m_imageMatrix[i][j] = pixel;
            }
        }
        //SAFE_DELETE(pixel);
    }
    m_baseColor = new Pixel<double>();
    m_baseColor->g = 0;
    m_baseColor->b = 0;
    m_baseColor->r = 0;
}

RawImage::RawImage(const char* filename) : m_imageMatrix(NULL), m_height(0), m_width(0)
{
    loadBmpFile(filename);
    m_baseColor = new Pixel<double>();
    m_baseColor->g = 0;
    m_baseColor->b = 0;
    m_baseColor->r = 0;
}

RawImage::~RawImage()
{
    cleanUp();
}

inline void RawImage::cleanUp()
{
    for (unsigned int i = 0; i < m_height; ++i)
    {
        for (unsigned int j = 0; j < m_width; ++j)
            SAFE_DELETE(m_imageMatrix[i][j]);
        SAFE_DELETE_ARRAY(m_imageMatrix[i]);
    }
    SAFE_DELETE_ARRAY(m_imageMatrix);

    m_height = 0;
    m_width = 0;
    m_imageMatrix = NULL;
    m_baseColor = NULL;
}

void RawImage::allocateImage()
{
    m_imageMatrix = new Pixel<double>** [m_height];
    for (int i = 0; i < m_height; ++i)
        m_imageMatrix[i] = new Pixel<double>* [m_width];
}

bool RawImage::loadBmpFile(const char* filename)
{
    cleanUp();

    FILE* infile = fopen(filename, "rb");
    if (!infile)
    {
        fprintf(stderr, "Unable to open file: %s\n", filename);
        return false;
    }

    bool fileFormatOK = false;
    int bChar = fgetc(infile);
    int mChar = fgetc(infile);

    if (bChar == 'B' && mChar == 'M')
    {
        skipBytes(infile, 16); //skip header 16bytes
        m_width = readLong(infile);
        m_height = readLong(infile);
        skipBytes(infile, 2);
        int bitsPerPixel = readShort(infile);
        skipBytes(infile, 24); // Skip info 24bytes

        if (m_width > 0 && m_width <= 100000
                && m_height > 0 && m_height <= 100000
                && bitsPerPixel == 24 && !feof(infile)) //:)
        {
            fileFormatOK = true;
        }
    }

    if (!fileFormatOK)
    {
        cleanUp();
        fprintf(stderr, "Not a valid 24-bit bitmap file: %s.\n", filename);
        fclose(infile);
        return false;
    }

    //prepare memory
    allocateImage();

    if (!m_imageMatrix)
    {
        fprintf(stderr, "Unable to allocate memory for %ld x %ld pixels: %s.\n", m_height, m_width, filename);
        cleanUp();
        fclose(infile);
        return false;
    }

    Pixel<double>* pixel = NULL;
    for (int i = 0; i < m_height; i++)
    {
        for (int j = 0; j < m_width; j++)
        {
            pixel = new Pixel<double>();
            pixel->r = fgetc(infile); // Blue color value
            pixel->g = fgetc(infile); // Green color value
            pixel->b = fgetc(infile); // Red color value            
            m_imageMatrix[i][j] = pixel;
        }
    }

    if (feof(infile))
    {
        fprintf(stderr, "Premature end of file: %s.\n", filename);
        cleanUp();
        fclose(infile);
        return false;
    }

    fclose(infile);
    return true;
}

short RawImage::readShort(FILE* infile)
{
    // read a 16 bit integer
    unsigned char lowByte, hiByte;
    lowByte = fgetc(infile); // Read the low order byte (little endian form)
    hiByte = fgetc(infile); // Read the high order byte

    // Pack together
    short ret = hiByte;
    ret <<= 8;
    ret |= lowByte;
    return ret;
}

long RawImage::readLong(FILE* infile)
{
    // Read in 32 bit integer
    unsigned char byte0, byte1, byte2, byte3;
    byte0 = fgetc(infile); // Read bytes, low order to high order
    byte1 = fgetc(infile);
    byte2 = fgetc(infile);
    byte3 = fgetc(infile);

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

void RawImage::skipBytes(FILE* infile, int numChars)
{
    for (int i = 0; i < numChars; i++)
    {
        fgetc(infile);
    }
}

bool RawImage::writeBmpFile(const char* filename)
{
    FILE* outfile = fopen(filename, "wb");
    if (!outfile)
    {
        fprintf(stderr, "Unable to open file: %s\n", filename);
        return false;
    }

    //prepare head
    fputc('B', outfile);
    fputc('M', outfile);
    int rowLen = getRowBytes();
    writeLong(40 + 14 + m_height * rowLen, outfile); // Length of file
    writeShort(0, outfile); // Reserved for future use
    writeShort(0, outfile);
    writeLong(40 + 14, outfile); // Offset to pixel data
    writeLong(40, outfile); // header length
    writeLong(m_width, outfile); // width in pixels
    writeLong(m_height, outfile); // height in pixels (pos for bottom up)
    writeShort(1, outfile); // number of planes
    writeShort(24, outfile); // bits per pixel
    writeLong(0, outfile); // no compression
    writeLong(0, outfile); // not used if no compression
    writeLong(0, outfile); // Pixels per meter
    writeLong(0, outfile); // Pixels per meter
    writeLong(0, outfile); // unused for 24 bits/pixel
    writeLong(0, outfile); // unused for 24 bits/pixel

    //store data
    Pixel<double>* pixel = NULL;
    for (int i = 0; i < m_height; i++)
    {
        for (int j = 0; j < m_width; j++)
        {
            pixel = m_imageMatrix[i][j];
            fputc(static_cast<unsigned char> (pixel->r), outfile); // Red color value
            fputc(static_cast<unsigned char> (pixel->g), outfile); // Green color value
            fputc(static_cast<unsigned char> (pixel->b), outfile); // Blue color value            
        }
    }

    fclose(outfile);
    return true;
}

void RawImage::writeLong(long data, FILE* outfile)
{
    // Read in 32 bit integer
    unsigned char byte0, byte1, byte2, byte3;
    byte0 = static_cast<unsigned char> (data & 0x000000ff); // low first than high
    byte1 = static_cast<unsigned char> ((data >> 8) & 0x000000ff);
    byte2 = static_cast<unsigned char> ((data >> 16) & 0x000000ff);
    byte3 = static_cast<unsigned char> ((data >> 24) & 0x000000ff);

    fputc(byte0, outfile);
    fputc(byte1, outfile);
    fputc(byte2, outfile);
    fputc(byte3, outfile);
}

void RawImage::writeShort(short data, FILE* outfile)
{
    // Read in 32 bit integer
    unsigned char byte0, byte1;
    byte0 = data & 0x000000ff; // low first than high
    byte1 = (data >> 8) & 0x000000ff;

    fputc(byte0, outfile);
    fputc(byte1, outfile);
}

void RawImage::setPixel(long y, long x, double red, double green, double blue)
{
    setPixel(y, x, convertDoubleToUnsignedChar(red), convertDoubleToUnsignedChar(green), convertDoubleToUnsignedChar(blue));
}

void RawImage::setPixel(long y, long x, unsigned char red, unsigned char green, unsigned char blue)
{
    if (y < m_height && y >= 0 && x < m_width && x >= 0)
    {
        Pixel<double>* thePixel = getPixel(y, x);
        thePixel->r = red;
        thePixel->g = green;
        thePixel->b = blue;
    }
}

void RawImage::setPixel(long y, long x, Pixel<double>* pixel) const
{
    if (y < m_height && y >= 0 && x < m_width && x >= 0)
    {
        Pixel<double>* thePixel = getPixel(y, x);
        thePixel->r = pixel->r;
        thePixel->g = pixel->g;
        thePixel->b = pixel->b;
    }
}

unsigned char RawImage::convertDoubleToUnsignedChar(double x)
{
    if (x >= 1.0)
    {
        return (unsigned char) 255;
    } else if (x <= 0.0)
    {
        return (unsigned char) 0;
    } else
    {
        return static_cast<unsigned char> (x * 255.0); // Rounds down
    }
}

Pixel<double>* RawImage::getPixel(long y, long x) const
{
    Pixel<double>* ret = NULL;
    //if (x < m_width && x >= 0 && y < m_height && y >= 0)
    {
        ret = m_imageMatrix[y][x];
    }

    return ret;
}

double RawImage::getPixelChannelValue(long y, long x, unsigned int channel) const
{
    //if (x < m_width && x >= 0 && y < m_height && y >= 0)
    {
        if (channel < 4)
        {
            switch (channel) {
                case 0:
                    return m_imageMatrix[y][x]->r;
                case 1:
                    return m_imageMatrix[y][x]->g;
                case 2:
                    return m_imageMatrix[y][x]->b;
            }
        }
    }

    return 0.0;
}

Pixel<double>* RawImage::convertXyzToLuv(Pixel<double>* XYZ)
{
    Pixel<double>* Luv = new Pixel<double>();

    double Xr = 1.0 / 3;
    double Yr = 1.0 / 3;
    double Zr = 1.0 / 3;

    double yr, us, vs, usr, vsr, e, k;

    yr = XYZ->g / Yr;
    us = 4 * XYZ->r / (XYZ->r + 15 * XYZ->g + 3 * XYZ->b);
    vs = 9 * XYZ->g / (XYZ->r + 15 * XYZ->g + 3 * XYZ->b);
    usr = 4 * Xr / (Xr + 15 * Yr + 2 * Zr);
    vsr = 9 * Yr / (Xr + 15 * Yr + 3 * Zr);
    e = 216.0 / 24389;
    k = 24389.0 / 27;

    if (yr > e)
        Luv->r = 116 * pow(yr, 1.0 / 3) - 16;
    else
        Luv->r = k * yr;

    Luv->g = 13.0 * Luv->r * (us - usr);
    Luv->b = 13.0 * Luv->r * (vs - vsr);
    return Luv;
}

Pixel<double>* RawImage::convertRgbToXyz(Pixel<double>* rgb)
{
    Pixel<double>* XYZ = new Pixel<double>();
    Pixel<double> temp;

    if (rgb->r <= 0.04045)
        temp.r = rgb->r / 12.92;
    else
        temp.r = pow((rgb->r + 0.055) / 1.055, 2.4);

    if (rgb->g <= 0.04045)
        temp.g = rgb->g / 12.92;
    else
        temp.g = pow((rgb->g + 0.055) / 1.055, 2.4);

    if (rgb->b <= 0.04045)
        temp.b = rgb->b / 12.92;
    else
        temp.b = pow((rgb->b + 0.055) / 1.055, 2.4);

    XYZ->r = 0.4124 * temp.r + 0.3576 * temp.g + 0.1805 * temp.b;
    XYZ->g = 0.2126 * temp.r + 0.7152 * temp.g + 0.0722 * temp.b;
    XYZ->b = 0.0193 * temp.r + 0.1192 * temp.g + 0.9505 * temp.b;
    return XYZ;
}

Pixel<double>* RawImage::convertRgbToLuv(Pixel<double>* rgb)
{
    Pixel<double>* xyzPixel;
    Pixel<double>* luvPixel;
    xyzPixel = convertRgbToXyz(rgb);
    luvPixel = convertXyzToLuv(xyzPixel);
    SAFE_DELETE(xyzPixel);
    return luvPixel;
}

double RawImage::colourDifference(Pixel<double>* rgb1, Pixel<double>* rgb2)
{
    Pixel<double>* temp1;
    Pixel<double>* temp2;
    double diff = 0.0;

    temp1 = convertRgbToLuv(rgb1);
    temp2 = convertRgbToLuv(rgb2);

    diff = sqrt(
            (temp1->r - temp2->r) * (temp1->r - temp2->r) +
            (temp1->g - temp2->g) * (temp1->g - temp2->g) +
            (temp1->b - temp2->b) * (temp1->b - temp2->b));

    SAFE_DELETE(temp1);
    SAFE_DELETE(temp2);

    return diff;
}

void RawImage::blur()
{
    const double m = 1.0 / 9;
    double result;

    Pixel<double>* pixel = new Pixel<double>();

    for (int y = 1; y < m_height - 1; y++)
    {
        for (int x = 1; x < m_width - 1; x++)
        {
            for (int ch = 0; ch < 3; ch++)
            {
                result = m * getPixelChannelValue(y - 1, x - 1, ch) +
                        m * getPixelChannelValue(y - 1, x, ch) +
                        m * getPixelChannelValue(y - 1, x + 1, ch) +
                        m * getPixelChannelValue(y, x - 1, ch) +
                        m * getPixelChannelValue(y, x, ch) +
                        m * getPixelChannelValue(y, x + 1, ch) +
                        m * getPixelChannelValue(y + 1, x - 1, ch) +
                        m * getPixelChannelValue(y + 1, x, ch) +
                        m * getPixelChannelValue(y + 1, x + 1, ch);

                switch (ch) {
                    case 0:
                        pixel->r = result;
                        break;
                    case 1:
                        pixel->g = result;
                        break;
                    case 2:
                        pixel->b = result;
                        break;
                }
            }
            setPixel(y, x, pixel);
        }
    }
    SAFE_DELETE(pixel);
}

void RawImage::sobel()
{
    double min = 1.0;
    double max = 0.0;

    double* buffer = new double[m_height * m_width];

    for (unsigned int i = 1; i < m_height - 1; ++i)
    {
        for (unsigned int j = 1; j < m_width - 1; ++j)
        {
            double gx =
                    1.0 * colourDifference(getPixel(i - 1, j - 1), getPixel(i + 1, j - 1)) +
                    2.0 * colourDifference(getPixel(i - 1, j), getPixel(i + 1, j)) +
                    1.0 * colourDifference(getPixel(i - 1, j + 1), getPixel(i + 1, j + 1));

            double gy =
                    1.0 * colourDifference(getPixel(i - 1, j - 1), getPixel(i - 1, j + 1)) +
                    2.0 * colourDifference(getPixel(i, j - 1), getPixel(i, j + 1)) +
                    1.0 * colourDifference(getPixel(i + 1, j - 1), getPixel(i + 1, j + 1));

            double val = pow(gx * gx + gy * gy, 0.5);

            if (val > max) max = val;
            if (val < min) min = val;

            buffer[i * m_width + j] = val;
        }
    }

    Pixel<double>* pixel = new Pixel<double>();

    for (int y = 1; y < m_height - 1; y++)
    {
        for (int x = 1; x < m_width - 1; x++)
        {
            double val = (buffer[y * m_width + x] - min) / (max - min) * 255;

            if (val > COLOR_TRESHOLD)
            {
                pixel->r = 255;
                pixel->g = 255;
                pixel->b = 255;
            }
            else
            {
                pixel->r = 0;
                pixel->g = 0;
                pixel->b = 0;
            }

            setPixel(y, x, pixel);
        }
    }
    SAFE_DELETE(pixel);
    SAFE_DELETE_ARRAY(buffer);
}

void RawImage::prewitt()
{
    double min = 1.0;
    double max = 0.0;

    double* buffer = new double[m_height * m_width];

    for (unsigned int i = 2; i < m_height - 2; ++i)
    {
        for (unsigned int j = 2; j < m_width - 2; ++j)
        {

            double gi =
                    2.0 * colourDifference(getPixel(i - 2, j - 2), getPixel(i + 2, j - 2)) +
                    2.0 * colourDifference(getPixel(i - 2, j - 1), getPixel(i + 2, j - 1)) +
                    2.0 * colourDifference(getPixel(i - 2, j), getPixel(i + 2, j)) +
                    2.0 * colourDifference(getPixel(i - 2, j + 1), getPixel(i + 2, j + 1)) +
                    2.0 * colourDifference(getPixel(i - 2, j + 2), getPixel(i + 2, j + 2));

            double gj =
                    1.0 * colourDifference(getPixel(i - 1, j - 2), getPixel(i + 1, j - 2)) +
                    1.0 * colourDifference(getPixel(i - 1, j - 1), getPixel(i + 1, j - 1)) +
                    1.0 * colourDifference(getPixel(i - 1, j), getPixel(i + 1, j)) +
                    1.0 * colourDifference(getPixel(i - 1, j + 1), getPixel(i + 1, j + 1)) +
                    1.0 * colourDifference(getPixel(i - 1, j + 2), getPixel(i + 1, j + 2));

            double gk =
                    1.0 * colourDifference(getPixel(i - 2, j - 1), getPixel(i - 2, j + 1)) +
                    1.0 * colourDifference(getPixel(i - 1, j - 1), getPixel(i - 1, j + 1)) +
                    1.0 * colourDifference(getPixel(i, j - 1), getPixel(i, j + 1)) +
                    1.0 * colourDifference(getPixel(i + 1, j - 1), getPixel(i + 1, j + 1)) +
                    1.0 * colourDifference(getPixel(i + 2, j - 1), getPixel(i + 2, j + 1));

            double gl =
                    2.0 * colourDifference(getPixel(i - 2, j - 2), getPixel(i - 2, j + 2)) +
                    2.0 * colourDifference(getPixel(i - 1, j - 2), getPixel(i - 1, j + 2)) +
                    2.0 * colourDifference(getPixel(i, j - 2), getPixel(i, j + 2)) +
                    2.0 * colourDifference(getPixel(i + 1, j - 2), getPixel(i - 1, j + 2)) +
                    2.0 * colourDifference(getPixel(i + 2, j - 2), getPixel(i - 2, j + 2));

            double val = pow(gi * gi + gj * gj + gk * gk + gl * gl, 0.5);

            if (val > max) max = val;
            if (val < min) min = val;

            buffer[i * m_width + j] = val;
        }
    }

    Pixel<double>* pixel = new Pixel<double>();

    for (int y = 0; y < m_height; y++)
    {
        for (int x = 0; x < m_width; x++)
        {
            double val = (buffer[y * m_width + x] - min) / (max - min) * 255;
            pixel->r = val;
            pixel->g = val;
            pixel->b = val;
            setPixel(y, x, pixel);
        }
    }
    SAFE_DELETE(pixel);

    SAFE_DELETE_ARRAY(buffer);
}

void RawImage::findPlacesWithTheSimilarColor(unsigned char red, unsigned char green, unsigned char blue, int interval)
{
    //create pixels with chosen color
    Pixel<double> pixelMinus;
    Pixel<double> pixelPlus;
    Pixel<double>* pixel = NULL;    

    pixelMinus.r = static_cast<double> (blue > interval ? blue - interval : 0);
    pixelMinus.g = static_cast<double> (green > interval ? green - interval : 0);
    pixelMinus.b = static_cast<double> (red > interval ? red - interval : 0);

    pixelPlus.r = static_cast<double> (blue + interval < 255 ? blue + interval : 255);
    pixelPlus.g = static_cast<double> (green + interval < 255 ? green + interval : 255);
    pixelPlus.b = static_cast<double> (red + interval < 255 ? red + interval : 255);

    if (red > green)
    {
        if (blue > red)
        {
            m_baseColor->r = 255;
            m_baseColor->g = 0;
            m_baseColor->b = 0;
        }
        else
        {
            m_baseColor->r = 0;
            m_baseColor->g = 0;
            m_baseColor->b = 255;
        }
    } 
    else
    {
        if (green > blue)
        {
            m_baseColor->r = 0;
            m_baseColor->g = 255;
            m_baseColor->b = 0;
        } else
        {
            m_baseColor->r = 255;
            m_baseColor->g = 0;
            m_baseColor->b = 0;
        }
    }

    for (unsigned int i = 0; i < m_height; ++i)
    {
        for (unsigned int j = 0; j < m_width; ++j)
        {
            pixel = m_imageMatrix[i][j];

            if (pixel->r <= pixelPlus.r && pixel->r >= pixelMinus.r //:)
                    && pixel->g <= pixelPlus.g && pixel->g >= pixelMinus.g
                    && pixel->b <= pixelPlus.b && pixel->b >= pixelMinus.b)
            {
                pixel->r = m_baseColor->r;
                pixel->g = m_baseColor->g;
                pixel->b = m_baseColor->b;
            }
            else
            {
                pixel->r = 0;
                pixel->g = 0;
                pixel->b = 0;
            }
        }
    }
}

inline Line* RawImage::findCorrectLine(int vecY, int vecX, int chY, int chX, int posY, int posX)
{
    Pixel<double>* pixel = NULL;
    Line* line = new Line();

    int countOfFails = 0;
    int vectorY = vecY;
    int vectorX = vecX;

    Point point;
    point.y = posY;
    point.x = posX;
    line->points.push_back(point);

    while (posY > 1 && posX > 1 && posY < m_height - 2 && posX < m_width - 2)
    {
        posY += vectorY;
        posX += vectorX;

        pixel = m_imageMatrix[posY][posX];
        if(pixel->r >= m_baseColor->r && pixel->g >= m_baseColor->g && pixel->b >= m_baseColor->b)
        //if (pixel->b > 250)
        {
            if (countOfFails > 0)
            {
                vectorY = vecY;
                vectorX = vecX;
            }
            countOfFails = 0;
            point.y = posY;
            point.x = posX;
            line->points.push_back(point);
        }
        else
        {
            if (countOfFails > 0)
                return line;

            //change direction and recover positons
            posY -= vectorY;
            posX -= vectorX;
            countOfFails++;
            vectorY = chY;
            vectorX = chX;
        }
    }
    return line;
}

Line* RawImage::detectLines()
{
    Pixel<double>* pixel = NULL;
    Line* line = NULL;
    Line* line2 = NULL;
    std::vector<Line*> lines;
    unsigned int maxLineSize = 0;

    for (unsigned int i = 1; i < m_height - 1; ++i)
    {
        for (unsigned int j = 1; j < m_width - 1; ++j)
        {
            pixel = m_imageMatrix[i][j];

            if(pixel->r >= m_baseColor->r && pixel->g >= m_baseColor->g && pixel->b >= m_baseColor->b)
            //if (pixel->b > 250)
            {
                line = findCorrectLine(1, 0, 0, 1, i, j);
                line2 = findCorrectLine(-1, 0, 0, 1, i, j);

                if (line != NULL && line2 != NULL)
                {
                    if (line->points.size() > line2->points.size())
                    {
                        if (line->points.size() > LINE_LENGTH_TRESHOLD)
                        {
                            lines.push_back(line);
                            SAFE_DELETE(line2);
                            break;
                        }
                    }
                    else
                    {
                        if (line2->points.size() > LINE_LENGTH_TRESHOLD)
                        {
                            lines.push_back(line2);
                            SAFE_DELETE(line);
                            break;
                        }
                    }
                } 
                else if (line != NULL && line2 == NULL)
                {
                    if (line->points.size() > LINE_LENGTH_TRESHOLD)
                    {
                        lines.push_back(line);
                        break;
                    }
                }
                else if (line == NULL && line2 != NULL)
                {
                    if (line2->points.size() > LINE_LENGTH_TRESHOLD)
                    {
                        lines.push_back(line2);
                        break;
                    }
                }
            }
        }
    }

    if (lines.size() == 0)
        return NULL;

    Line* ret = new Line();

    for (unsigned int i = 0, tempSize = 0; i < lines.size(); i++)
    {
        tempSize = lines[i]->points.size();
        if (tempSize > maxLineSize)
        {
            line = lines[i];
            maxLineSize = tempSize;
        }
    }

    Point linePoint;
    for (unsigned int i = 0; i < line->points.size(); i++)
    {
        linePoint = line->points[i];
        pixel = m_imageMatrix[linePoint.y][linePoint.x];
        pixel->b = 0;
        pixel->g = 0;
        pixel->r = 255;
        ret->points.push_back(linePoint);
    }

    for (unsigned int i = 0; i < lines.size(); ++i)
    {
        SAFE_DELETE(lines[i]);
    }
    lines.clear();

    return ret;
}

LineVector* RawImage::getDirectionOfLine(Line* line)
{
    if (line != NULL)
    {
        Point start = line->points.front();
        Point end = line->points.back();

        LineVector* foundVector = new LineVector();
        foundVector->v1 = end.x - start.x;
        foundVector->v2 = end.y - start.y;

        std::cout << "vec(" << foundVector->v1 << ", " << foundVector->v2 << ") |" << line->points.size() << "|" << std::endl;

        return foundVector;
    } 
    else
    {
        return NULL;
    }
}

std::ostream& operator<<(std::ostream& out, const RawImage& img)
{
    out << "Image height: " << img.m_height << " width: " << img.m_width << "\n";

    for (unsigned int i = 0; i < img.m_height; ++i)
    {
        for (unsigned int j = 0; j < img.m_width; ++j)
            out << *img.m_imageMatrix[i][j];
        out << "\n\n";
    }
    out << "\n\n";
    return out;
}




