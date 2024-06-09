#include <string.h>

// WAV文件头结构体
typedef struct {
    char     chunkID[4];     // "RIFF"
    uint32_t chunkSize;      // 文件大小
    char     format[4];      // "WAVE"
    char     subchunk1ID[4]; // "fmt "
    uint32_t subchunk1Size;  // 子块1大小
    uint16_t audioFormat;    // 音频格式
    uint16_t numChannels;    // 声道数
    uint32_t sampleRate;     // 采样率
    uint32_t byteRate;       // 码率
    uint16_t blockAlign;     // 数据块对齐
    uint16_t bitsPerSample;  // 位深度
    char     subchunk2ID[4]; // "data"
    uint32_t subchunk2Size;  // 子块2大小
} WavHeader_t;

// void *writeBuffer = NULL;

FILE* wav_audio_start(char *filename, uint32_t sample_rate, uint16_t bits_per, uint16_t numChannels) {
    // writeBuffer = malloc(8192);
    FILE *file = fopen(filename, "wb");
    if (file == NULL) {
        printf("Error opening file for writing.\n");
        return NULL;
    }

    WavHeader_t header;
    strncpy(header.chunkID, "RIFF", 4);
    header.chunkSize = 0;  // Later updated
    strncpy(header.format, "WAVE", 4);
    strncpy(header.subchunk1ID, "fmt ", 4);
    header.subchunk1Size = 16;
    header.audioFormat = 1;
    header.numChannels = numChannels;
    header.sampleRate = sample_rate;
    header.bitsPerSample = bits_per;
    header.byteRate = sample_rate * numChannels * bits_per / 8;
    header.blockAlign = numChannels * bits_per / 8;
    strncpy(header.subchunk2ID, "data", 4);
    header.subchunk2Size = 0;

    fwrite(&header, sizeof(header), 1, file);
    // free(writeBuffer);

    return file;
}

void wav_audio_write(void *inbuf, size_t len, size_t *bytes_written, FILE *file) {
    *bytes_written = fwrite(inbuf, sizeof(char), len, file);
}

void wav_audio_close(FILE *file) {
    // 获取文件当前大小
    fseek(file, 0, SEEK_END);
    uint32_t fileSize = ftell(file);
    
    // 计算 chunkSize 和 subchunk2Size
    uint32_t chunkSize = fileSize - 8;
    uint32_t subchunk2Size = fileSize - 44;
    
    // 写入 chunkSize
    fseek(file, 4, SEEK_SET);
    fwrite(&chunkSize, sizeof(chunkSize), 1, file);
    
    // 写入 subchunk2Size
    fseek(file, 40, SEEK_SET);
    fwrite(&subchunk2Size, sizeof(subchunk2Size), 1, file);
    
    // 关闭文件
    fclose(file);
}

