#include <codecvt>
#include <exception>
#include <iostream>
#include <locale>
#include <mutex>
#include <vector>

#include "freetype_helper.h"
#include "utils.h"

extern "C" {

#include <ft2build.h>
#include FT_FREETYPE_H
const char *FreeTypeErrorMessage(FT_Error err) {
#undef FTERRORS_H_
#define FT_ERRORDEF(e, v, s)                                                   \
    case e:                                                                      \
        return s;
#define FT_ERROR_START_LIST switch (err) {
#define FT_ERROR_END_LIST }
#include FT_ERRORS_H
    return "(Unknown error)";
}
}

#define CHECK_FREETYPE(x)                                                      \
{                                                                            \
    auto error = x;                                                            \
    if (error) {                                                               \
      auto err_msg = FreeTypeErrorMessage(error);                              \
      std::cerr << "FreeType Error: " << err_msg << std::endl;                 \
      throw std::runtime_error(err_msg);                                       \
    }                                                                          \
}

class GlpyhContext {
public:
static GlpyhContext &GetInstance() {
        static thread_local GlpyhContext ctx;
        return ctx;
}

int DrawChar(char32_t ch, int x, int y, const YUVColor *color,
                ImageData& image) {
    FT_UInt glyph_idx = 0;
    FT_Face face;
    int font_idx = 0;
    int font_list_size = faces.size();
    for (font_idx = 0; font_idx < font_list_size; ++font_idx) {
        face = faces[font_idx];
        glyph_idx = FT_Get_Char_Index(face, ch);
        CHECK_FREETYPE(FT_Load_Glyph(face, glyph_idx, FT_LOAD_RENDER));
        // if glyph_idx is 0 that means symbol not found,
        // try next font!
        if (glyph_idx > 0) {
            break;
        }
    }

    CHECK_FREETYPE(FT_Render_Glyph(face->glyph, FT_RENDER_MODE_NORMAL));

    FT_GlyphSlot slot = face->glyph;

    int h = image.width;
    int w = image.height;

    int image_y = y - slot->bitmap_top + font_size;

    for (int i = 0; i < slot->bitmap.rows && image_y < h; ++i, ++image_y) {
        int image_x = x + slot->bitmap_left;
        for (int j = 0; j < slot->bitmap.width && image_x < w; ++j, ++image_x) {
            auto bitmap_val = slot->bitmap.buffer[i * slot->bitmap.width + j];
            if (bitmap_val > 0) {
                SetPixel(image, image_x, image_y, *color);
            }
        }
    }
    return slot->advance.x >> 6;
}

private:
GlpyhContext() {
    std::vector<const char *> font_list{
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf", // For CJK
        "/usr/share/fonts/truetype/noto/NotoMono-Regular.ttf" // For latin and
                                                              // digits
    };

    for (const char *font_path : font_list) {
        FT_Face face;
        CHECK_FREETYPE(FT_Init_FreeType(&library));
        CHECK_FREETYPE(FT_New_Face(library, font_path, 0, &face));

        CHECK_FREETYPE(FT_Set_Pixel_Sizes(face, font_size, font_size));
        faces.push_back(face);
    }
}

    int font_size{20};
    FT_Library library;
    std::vector<FT_Face> faces;
    std::mutex mtx;
};

void SetPixel(ImageData& image, int x, int y, const YUVColor &color) {
    *(image.data.get() + y * image.width + x) = color.y;
    uint8_t *uv_offset = image.data.get() + image.width * image.height + (y / 2) * image.width + x / 2 * 2;
    uv_offset[0] = color.u;
    uv_offset[1] = color.v;
}

// TODO: cache glyphs
void RenderText(ImageData& image, int x, int y, const std::string &text, const YUVColor *color) {
    auto &ctx = GlpyhContext::GetInstance();
    std::wstring_convert<std::codecvt_utf8<char32_t>, char32_t> cvt;
    std::u32string utf32_str = cvt.from_bytes(text);
    for (auto u32char : utf32_str) {
        x += ctx.DrawChar(u32char, x, y, color, image);
    }
}
