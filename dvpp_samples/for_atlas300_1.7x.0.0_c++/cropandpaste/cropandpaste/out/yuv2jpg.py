from PIL import Image
import sys

def yuv420_to_rgb888(width, height, yuv):
    # function requires both width and height to be multiples of 4
    if (width % 4) or (height % 4):
        raise Exception("width and height must be multiples of 4")
    rgb_bytes = bytearray(width*height*3)

    red_index = 0
    green_index = 1
    blue_index = 2
    y_index = 0
    for row in range(0,height):
        u_index = width * height + (row//2)*(width//2)
        v_index = u_index + (width*height)//4
        for column in range(0,width):
            Y = yuv[y_index]
            U = yuv[u_index]
            V = yuv[v_index]
            C = (Y - 16) * 298
            D = U - 128
            E = V - 128
            R = (C + 409*E + 128) // 256
            G = (C - 100*D - 208*E + 128) // 256
            B = (C + 516 * D + 128) // 256
            R = 255 if (R > 255) else (0 if (R < 0) else R)
            G = 255 if (G > 255) else (0 if (G < 0) else G)
            B = 255 if (B > 255) else (0 if (B < 0) else B)
            rgb_bytes[red_index] = R
            rgb_bytes[green_index] = G
            rgb_bytes[blue_index] = B
            u_index += (column % 2)
            v_index += (column % 2)
            y_index += 1
            red_index += 3
            green_index += 3
            blue_index += 3
    return rgb_bytes

def testConversion(source, dest,w,h):
    print("opening file")
    f = open(source, "rb")
    yuv = f.read()
    f.close()
    print("read file")
    rgb_bytes = yuv420_to_rgb888(w,h, yuv)
    # cProfile.runctx('yuv420_to_rgb888(1920,1088, yuv)', {'yuv420_to_rgb888':yuv420_to_rgb888}, {'yuv':yuv})
    print("finished conversion. Creating image object")
    img = Image.frombytes("RGB", (w,h), bytes(rgb_bytes))
    print("Image object created. Starting to save")
    img.save(dest, "JPEG")
    img.close()
    print("Save completed")
#testConversion("E:\\output.yuv","E:\\output.jpg",224,224)
if __name__ == '__main__':
    testConversion("output.yuv","output.jpg",224,224)
