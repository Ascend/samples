/*
 * Copyright(C) 2020. Huawei Technologies Co.,Ltd. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _CBASE64_H_
#define _CBASE64_H_

namespace {
    const int SHIFT_NUMBER_2 = 2;
    const int SHIFT_NUMBER_4 = 4;
    const int SHIFT_NUMBER_6 = 6;
    const int SHIFT_NUMBER_8 = 8;
    const int SHIFT_NUMBER_12 = 12;
    const int SHIFT_NUMBER_16 = 16;
    const int SHIFT_NUMBER_18 = 18;

    const int EACH_STEP_SIZE = 4;

    const int NORMAL_NUMBER_1 = 1;
    const int NORMAL_NUMBER_2 = 2;
    const int NORMAL_NUMBER_3 = 3;
}

class CBase64 {
public:
    CBase64() = default;

    ~CBase64() = default;

    /*
     * base64 encode
     * @param data input data
     * @param dataSize  data size
     * @return base64 string
     */
    static std::string Encode(const std::string &buffer, int dataSize)
    {
        // coding table
        const char EncodeTable[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

        std::string result;

        const char *data = buffer.c_str();
        unsigned char tmp[EACH_STEP_SIZE] = {0};
        int lineLength = 0;
        const int turnBufferLength = 3;
        const int maxLineLength = 76;
        for (int i = 0; i < (int) (dataSize / turnBufferLength); i++) {
            tmp[NORMAL_NUMBER_1] = *data++;
            tmp[NORMAL_NUMBER_2] = *data++;
            tmp[NORMAL_NUMBER_3] = *data++;
            result += EncodeTable[tmp[NORMAL_NUMBER_1] >> SHIFT_NUMBER_2];
            result += EncodeTable[((tmp[NORMAL_NUMBER_1] << SHIFT_NUMBER_4) |
                                   (tmp[NORMAL_NUMBER_2] >> SHIFT_NUMBER_4)) & 0x3F];
            result += EncodeTable[((tmp[NORMAL_NUMBER_2] << SHIFT_NUMBER_2) |
                                   (tmp[NORMAL_NUMBER_3] >> SHIFT_NUMBER_6)) & 0x3F];
            result += EncodeTable[tmp[NORMAL_NUMBER_3] & 0x3F];
            if (lineLength += EACH_STEP_SIZE, lineLength == maxLineLength) {
                lineLength = 0;
            }
        }

        int Mod = dataSize % turnBufferLength;
        if (Mod == NORMAL_NUMBER_1) {
            tmp[NORMAL_NUMBER_1] = *data++;
            result += EncodeTable[(tmp[NORMAL_NUMBER_1] & 0xFC) >> SHIFT_NUMBER_2];
            result += EncodeTable[((tmp[NORMAL_NUMBER_1] & 0x03) << SHIFT_NUMBER_4)];
            result += "==";
        } else if (Mod == NORMAL_NUMBER_2) {
            tmp[NORMAL_NUMBER_1] = *data++;
            tmp[NORMAL_NUMBER_2] = *data++;
            result += EncodeTable[(tmp[NORMAL_NUMBER_1] & 0xFC) >> SHIFT_NUMBER_2];
            result += EncodeTable[((tmp[NORMAL_NUMBER_1] & 0x03) << SHIFT_NUMBER_4) |
                                  ((tmp[NORMAL_NUMBER_2] & 0xF0) >> SHIFT_NUMBER_4)];
            result += EncodeTable[((tmp[NORMAL_NUMBER_2] & 0x0F) << SHIFT_NUMBER_2)];
            result += "=";
        }

        return result;
    }

    /*
     * base64 decode
     * @param data base64 encoded string
     * @param dataSize data size
     * @param OutByte
     * @return
     */
    static std::string Decode(const std::string &buffer, int dataSize, int &outSize)
    {
        const char decodeTable[] = {
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            62, // '+'
            0, 0, 0,
            63, // '/'
            52, 53, 54, 55, 56, 57, 58, 59, 60, 61, // '0'-'9'
            0, 0, 0, 0, 0, 0, 0,
            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
            13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, // 'A'-'Z'
            0, 0, 0, 0, 0, 0,
            26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38,
            39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, // 'a'-'z'
        };

        const char *data = buffer.c_str();

        std::string result;
        int nValue;
        int i = 0;
        while (i < dataSize) {
            if (*data != '\r' && *data != '\n') {
                nValue = decodeTable[(unsigned char) (*data++)] << SHIFT_NUMBER_18;
                nValue += decodeTable[(unsigned char) (*data++)] << SHIFT_NUMBER_12;
                result += (nValue & 0x00FF0000) >> SHIFT_NUMBER_16;
                outSize++;
                if (*data == '=') {
                    i += EACH_STEP_SIZE;
                    continue;
                }

                nValue += decodeTable[(unsigned char) (*data++)] << SHIFT_NUMBER_6;
                result += (nValue & 0x0000FF00) >> SHIFT_NUMBER_8;
                outSize++;
                if (*data != '=') {
                    nValue += decodeTable[(unsigned char) (*data++)];
                    result += nValue & 0x000000FF;
                    outSize++;
                }
                i += EACH_STEP_SIZE;
            } else {
                data++;
                i++;
            }
        }
        return result;
    }
};

#endif
