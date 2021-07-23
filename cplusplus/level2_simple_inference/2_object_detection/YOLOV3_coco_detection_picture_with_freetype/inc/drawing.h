#ifndef __DRAWING__H__
#define __DRAWING__H__

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <string>
#include "freetype_helper.h"

void DrawText(ImageData& image, int x, int y, const std::string &text, const YUVColor &color);
void DrawRect(ImageData& image, int x1, int y1, int x2, int y2, const YUVColor &color,
                int lineWidth);
#endif //__DRAWING__H__
