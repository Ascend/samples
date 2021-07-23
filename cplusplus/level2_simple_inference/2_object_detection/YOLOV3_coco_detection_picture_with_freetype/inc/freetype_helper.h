#ifndef __FREETYPE_HELPER_H__
#define __FREETYPE_HELPER_H__

#include <unistd.h>
#include <string>
#include <memory>
#include "acl/acl.h"
#include "utils.h"

void SetPixel(ImageData& image, int x, int y, const YUVColor &color);
void RenderText(ImageData& image, int x, int y, const std::string &text, const YUVColor *color);

#endif //__FREETYPE_HELPER_H__
