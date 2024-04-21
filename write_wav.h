#ifndef WAV_WRITER_H
#define WAV_WRITER_H

#include <stdint.h>
#include <stdio.h>

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

// 初始化WAV文件
FILE* wav_audio_start(char *filename, uint32_t sample_rate, uint16_t bits_per, uint16_t numChannels) {
    FILE *file = fopen(filename, "wb");
    if (file == NULL) {
        printf("Error opening file for writing.\n");
        return NULL;
    }

    // 填充WAV文件头
    WavHeader_t header;
    strncpy(header.chunkID, "RIFF", 4);
    strncpy(header.format, "WAVE", 4);
    strncpy(header.subchunk1ID, "fmt ", 4);
    header.subchunk1Size = 16;  // PCM格式
    header.audioFormat = 1;     // PCM格式
    header.numChannels = numChannels;     // 单声道
    header.sampleRate = sample_rate;
    header.bitsPerSample = bits_per;
    header.byteRate = sample_rate * bits_per / 8;
    header.blockAlign = bits_per / 8;
    strncpy(header.subchunk2ID, "data", 4);
    header.subchunk2Size = 0;   // 之后会更新为实际数据大小

    // 写入WAV文件头
    fwrite(&header, sizeof(header), 1, file);

    return file;
}

// 将缓冲区写入WAV文件
void wav_audio_write(void *inbuf, size_t len, size_t *bytes_written, FILE *file) {
    // 写入数据
    *bytes_written = fwrite((uint8_t *)inbuf, sizeof(uint8_t), len, file);

    // 更新subchunk2Size字段
    fseek(file, 40, SEEK_SET);  // subchunk2Size偏移量为40
    uint32_t subchunk2Size = ftell(file) - 44;  // 实际数据大小
    fwrite(&subchunk2Size, sizeof(uint32_t), 1, file);

    // 回到文件末尾
    fseek(file, 0, SEEK_END);
}

// 关闭WAV文件
void wav_audio_close(FILE *file) {
    fclose(file);
}

#endif /* WAV_WRITER_H */
