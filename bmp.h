#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#pragma pack(push, 1)
typedef struct {
    uint16_t bfType;
    uint32_t bfSize;
    uint16_t bfReserved1;
    uint16_t bfReserved2;
    uint32_t bfOffBits;
} BITMAPFILEHEADER;

typedef struct {
    uint32_t biSize;
    int32_t  biWidth;
    int32_t  biHeight;
    uint16_t biPlanes;
    uint16_t biBitCount;
    uint32_t biCompression;
    uint32_t biSizeImage;
    int32_t  biXPelsPerMeter;
    int32_t  biYPelsPerMeter;
    uint32_t biClrUsed;
    uint32_t biClrImportant;
} BITMAPINFOHEADER;
#pragma pack(pop)

void save_mono_bmp(const char* filename, uint8_t* buffer, int width, int height) {
    FILE* file = fopen(filename, "wb");
    if (!file) return;

    BITMAPFILEHEADER fileHeader;
    BITMAPINFOHEADER infoHeader;

    int row_padded = (width + 31) / 32 * 4;
    uint32_t imageSize = row_padded * height;

    fileHeader.bfType = 0x4D42;
    fileHeader.bfSize = 54 + imageSize;
    fileHeader.bfReserved1 = 0;
    fileHeader.bfReserved2 = 0;
    fileHeader.bfOffBits = 54;

    infoHeader.biSize = 40;
    infoHeader.biWidth = width;
    infoHeader.biHeight = -height;
    infoHeader.biPlanes = 1;
    infoHeader.biBitCount = 1;
    infoHeader.biCompression = 0;
    infoHeader.biSizeImage = imageSize;
    infoHeader.biXPelsPerMeter = 0;
    infoHeader.biYPelsPerMeter = 0;
    infoHeader.biClrUsed = 0;
    infoHeader.biClrImportant = 0;

    fwrite(&fileHeader, sizeof(BITMAPFILEHEADER), 1, file);
    fwrite(&infoHeader, sizeof(BITMAPINFOHEADER), 1, file);

    uint8_t* row = (uint8_t*)malloc(row_padded);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < row_padded; x++) {
            if (x < width / 8) {
                row[x] = buffer[y * (width / 8) + x];
            } else {
                row[x] = 0;
            }
        }
        fwrite(row, row_padded, 1, file);
    }
    free(row);
    fclose(file);
}

void save_rgb565_bmp(const char* filename, uint16_t* buffer, int width, int height) {
    FILE* file = fopen(filename, "wb");
    if (!file) return;

    BITMAPFILEHEADER fileHeader;
    BITMAPINFOHEADER infoHeader;

    int row_padded = width * 3;
    uint32_t imageSize = row_padded * height;

    fileHeader.bfType = 0x4D42;
    fileHeader.bfSize = 54 + imageSize;
    fileHeader.bfReserved1 = 0;
    fileHeader.bfReserved2 = 0;
    fileHeader.bfOffBits = 54;

    infoHeader.biSize = 40;
    infoHeader.biWidth = width;
    infoHeader.biHeight = -height;
    infoHeader.biPlanes = 1;
    infoHeader.biBitCount = 24;
    infoHeader.biCompression = 0;
    infoHeader.biSizeImage = imageSize;
    infoHeader.biXPelsPerMeter = 0;
    infoHeader.biYPelsPerMeter = 0;
    infoHeader.biClrUsed = 0;
    infoHeader.biClrImportant = 0;

    fwrite(&fileHeader, sizeof(BITMAPFILEHEADER), 1, file);
    fwrite(&infoHeader, sizeof(BITMAPINFOHEADER), 1, file);

    uint8_t* row = (uint8_t*)malloc(row_padded);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            uint16_t pixel = buffer[y * width + x];
            int idx = x * 3;
            row[idx] = (pixel & 0x1F) << 3;
            row[idx + 1] = ((pixel >> 5) & 0x3F) << 2;
            row[idx + 2] = ((pixel >> 11) & 0x1F) << 3;
        }
        fwrite(row, row_padded, 1, file);
    }
    free(row);
    fclose(file);
}
