#ifndef IMAGE_H_INCLUDED
#define IMAGE_H_INCLUDED


/* PPM image object */

typedef unsigned char byte;

class Image {
    int width = 0;
    int height = 0;
    byte *data;

public:

    // load from PPM file
    Image(const char* filename)
    {
        FILE* f = fopen(filename, "rb");
        if (!f) dsError("Cannot open image file '%s'", filename);

        // read in header
        if (fgetc(f) != 'P' || fgetc(f) != '6')
            dsError("Image file '%s' is not a binary PPM (no P6 header)", filename);

        skipWhiteSpace(filename, f);

        // read in image parameters
        width = readNumber(filename, f);
        skipWhiteSpace(filename, f);
        height = readNumber(filename, f);
        skipWhiteSpace(filename, f);
        int max_value = readNumber(filename, f);

        // check values
        if (width < 1 || height < 1)
            dsError("Bad image size'%s'", filename);
        if (max_value != 255)
            dsError("Image file '%s' must have color range of 255", filename);

        // read either nothing, LF (10), or CR,LF (13,10)
        int c = fgetc(f);
        if (c == 10) {
            // LF
        }
        else if (c == 13) {
            // CR
            c = fgetc(f);
            if (c != 10) ungetc(c, f);
        }
        else ungetc(c, f);

        // read in rest of data
        data = new byte[width * height * 3];
        if (fread(data, width * height * 3, 1, f) != 1)
            dsError("Cannot read data from image file '%s'", filename);

        fclose(f);
    }


    ~Image() { delete[] data; }


    // skip over white space and comments in a stream.
    void skipWhiteSpace(const char* filename, FILE* f)
    {
        for(;;) {
            const int c = fgetc(f);
            if (c == EOF)
                dsError("Unexpected end of file in '%s'", filename);

            // skip comments
            if (c == '#') {
                int d = 0;
                do {
                    d = fgetc(f);
                    if (d == EOF)
                        dsError("Unexpected end of file in '%s'", filename);
                } while (d != '\n');
                continue;
            }

            if (c > ' ') {
                ungetc(c, f);
                return;
            }
        }
    }


    // read a number from a stream, this return 0 if there is none (that's okay
    // because 0 is a bad value for all PPM numbers anyway).

    int readNumber(const char* filename, FILE* f)
    {
        int n=0;
        for(;;) {
            int c = fgetc(f);
            if (c==EOF)
                dsError("Unexpected end of file in '%s'",filename);
            if (c >= '0' && c <= '9')
                n = n * 10 + (c - '0');
            else {
                ungetc(c,f);
                return n;
            }
        }
    }


    int get_width() const { return width; }
    int get_height() const { return height; }
    byte *get_data() const { return data; }
};

#endif // IMAGE_H_INCLUDED
