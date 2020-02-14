#ifndef TEXTURE_H_INCLUDED
#define TEXTURE_H_INCLUDED

#include <X11/Xlib.h>
#include <string>

/* This class is based on the original code by Russell L. Smith
 * with only minimal changes.
 */

class Texture {
    Image image;
    GLuint name = 0;

public:
    Texture (std::string const& filename)
    : image(filename.c_str())
    {
        glGenTextures (1, &name);
        glBindTexture (GL_TEXTURE_2D, name);

        // set pixel unpacking mode
        glPixelStorei (GL_UNPACK_SWAP_BYTES , 0);
        glPixelStorei (GL_UNPACK_ROW_LENGTH , 0);
        glPixelStorei (GL_UNPACK_ALIGNMENT  , 1);
        glPixelStorei (GL_UNPACK_SKIP_ROWS  , 0);
        glPixelStorei (GL_UNPACK_SKIP_PIXELS, 0);

        // glTexImage2D (GL_TEXTURE_2D, 0, 3, image->get_width(), image->get_height(), 0,
        //               GL_RGB, GL_UNSIGNED_BYTE, image->get_data());
        gluBuild2DMipmaps(GL_TEXTURE_2D, 3, image.get_width(), image.get_height(),
                          GL_RGB, GL_UNSIGNED_BYTE, image.get_data() );

        // set texture parameters - will these also be bound to the texture???
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

        glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
    }

    ~Texture() {
        glDeleteTextures(1, &name);
    }

    void bind(bool modulate = false)
    {
        glBindTexture(GL_TEXTURE_2D, name);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, modulate ? GL_MODULATE : GL_DECAL);
    }
};



#endif /* TEXTURE_H_INCLUDED */
