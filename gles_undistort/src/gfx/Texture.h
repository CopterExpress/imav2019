#ifndef GLESIMAGEPROCESSING_TEXTURE_H
#define GLESIMAGEPROCESSING_TEXTURE_H

#include <GLES3/gl3.h>

namespace Gfx
{

struct TextureData
{
    GLuint id;
    GLsizei width, height;
};

TextureData createTexture(int width, int height, void* data);

TextureData updateTexture(TextureData& td, int width, int height, void* data);

void deleteTexture(TextureData &td);

}

#endif //GLESIMAGEPROCESSING_TEXTURE_H
