#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdint.h>
#include <string>
#include "drawing.h"
#include <stdio.h>

void DrawText(ImageData& image, int x, int y, const std::string &text, const YUVColor &color) {
    RenderText(image, x, y, text, &color);
}

void DrawRect(ImageData& image, int x1, int y1, int x2, int y2, const YUVColor &color,
              int lineWidth) {
    if (x1 > x2) {
        std::swap(x1, x2);
    }

    if (y1 > y2) {
        std::swap(y1, y2);
    }

    int i, j;
    int i_bound, j_bound;
    int i_start, j_start;
    int width = (int)image.width;
    int height = (int)image.height;

    j_bound = std::min(height, y1 + lineWidth);
    i_bound = std::min(width - 1, x2);

    for (j = y1; j < j_bound; ++j) {
        for (i = x1; i <= i_bound; ++i) {
            SetPixel(image, i, j, color);
        }
    }

    j_start = std::max(0, y2 - lineWidth + 1);
    j_bound = std::min(height - 1, y2);

    i_start = std::max(0, x1);
    i_bound = std::min(width - 1, x2);

    for (j = j_start; j <= j_bound; ++j) {
        for (i = i_start; i <= i_bound; ++i) {
            SetPixel(image, i, j, color);
        }
    }

    i_bound = std::min(width, x1 + lineWidth);
    j_bound = std::min(height - 1, y2);

    for (i = x1; i < i_bound; ++i) {
        for (j = y1; j <= j_bound; ++j) {
            SetPixel(image, i, j, color);
        }
    }

    i_start = std::max(0, (x2 - lineWidth + 1));
    i_bound = std::min(width - 1, x2);

    j_start = std::max(0, y1);
    j_bound = std::min(height - 1, y2);

    for (i = i_start; i <= i_bound; ++i) {
        for (j = j_start; j <= j_bound; ++j) {
            SetPixel(image, i, j, color);
        }
    }
    }
