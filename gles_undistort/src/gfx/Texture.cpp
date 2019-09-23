#include "Texture.h"

namespace Gfx
{

TextureData createTexture(int width, int height, void* data)
{
    TextureData td;
    glGenTextures(1, &td.id);
    glBindTexture(GL_TEXTURE_2D, td.id);
    td.width = width;
    td.height = height;
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, td.width, td.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    return td;
}

TextureData updateTexture(TextureData &td, int width, int height, void* data)
{
    glBindTexture(GL_TEXTURE_2D, td.id);
    if (width == td.width && height == td.height)
    {
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, td.width, td.height, GL_RGBA, GL_UNSIGNED_BYTE, data);
    }
    else
    {
        td.width = width;
        td.height = height;
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, td.width, td.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    }
    return td;
}

void deleteTexture(TextureData &td)
{
    glDeleteTextures(1, &td.id);
    td.width = td.height = 0;
    td.id = 0;
}

}
