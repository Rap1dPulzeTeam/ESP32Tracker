#include <Arduino.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
// #include <pwm_audio.h>
#include "pwm_audio.h"
#include <driver/gpio.h>
#include <math.h>
#include <esp_log.h>
#include "ssd1306.h"
#include "vol_table.h"
#include <string.h>
#include "esp32-hal-cpu.h"
#include <Adafruit_ST7735.h>

#include "esp_spiffs.h"
#include <dirent.h>
#include "listfile.h"
#include "NULL_MOD.h"
#include "driver/i2s.h"
#include "driver/gpio.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "write_wav.h"
#include "esp_dsp.h"
#include "ESPRotary.h"
#include "audioTypedef.h"
#include <Adafruit_MPR121.h>

#define MOUNT_POINT "/sdcard"

#define TFT_DC 3
#define TFT_CS 10
#define TFT_RST 8
uint8_t *tracker_data;
void fillMidRect(uint8_t w, uint8_t h, uint16_t color);
void drawMidRect(uint8_t w, uint8_t h, uint16_t color);
void setMidCusr(uint8_t w, uint8_t h, int8_t ofst);
void read_pattern_table();
void read_wave_info();
void comp_wave_ofst();
#define CHL_NUM 4
#define TRACKER_ROW 64
uint8_t NUM_PATTERNS;
#define BUFFER_PATTERNS 2
#define NUM_ROWS 64
#define NUM_CHANNELS 4
#define PATTERN_SIZE (NUM_ROWS * NUM_CHANNELS * 4)
#define BUFF_SIZE 1024

bool recMod = false;
FILE *export_wav_file; // WAV文件指针
size_t export_wav_written = 0; // 已写入的数据量
void *export_wav_buffer;
uint8_t inputOct = 2;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Adafruit_MPR121 touchPad = Adafruit_MPR121();
ESPRotary rotary;

typedef enum {
    KEY_IDLE,
    KEY_ATTACK,
    KEY_RELEASE
} key_status_t;

typedef struct {
    uint8_t num;
    key_status_t status;
} key_event_t;

QueueHandle_t xTouchPadQueue;
QueueHandle_t xOptionKeyQueue;

#define readOptionKeyEvent xQueueReceive(xOptionKeyQueue, &optionKeyEvent, 0)
#define readTouchPadKeyEvent xQueueReceive(xTouchPadQueue, &touchPadEvent, 0)

#define KEY_UP 1
#define KEY_DOWN 2
#define KEY_L 3
#define KEY_R 4
#define KEY_OK 5
#define KEY_SPACE 6
#define KEY_A 7
#define KEY_B 8

class  aFrameBuffer : public Adafruit_GFX {
  public:
    uint16_t lcd_buffer[40960];
    aFrameBuffer(int16_t w, int16_t h): Adafruit_GFX(w, h)
    {
    // lcd_buffer = (uint16_t*)malloc(2 * h * w);
        for (int i = 0; i < h * w; i++)
        lcd_buffer[i] = 0;
    }
    void drawPixel( int16_t x, int16_t y, uint16_t color)
    {
        if (x > 159)
            return;
        if (x < 0)
            return;
        if (y > 127)
            return;
        if (y < 0)
            return;
        lcd_buffer[x + y * _width] = color;
    }

    void display()
    {
        tft.setAddrWindow(0, 0, 160, 128);
        digitalWrite(TFT_DC, HIGH);
        digitalWrite(TFT_CS, LOW);
        SPI.beginTransaction(SPISettings(79000000, MSBFIRST, SPI_MODE0));
        for (uint16_t i = 0; i < 160 * 128; i++)
        {
            SPI.transfer16(lcd_buffer[i]);
        }
        SPI.endTransaction();
        digitalWrite(TFT_CS, HIGH);
    }
};

aFrameBuffer frame(160, 128);

#define SMP_RATE 44100
#define SMP_BIT 8
float *buffer_ch[NUM_CHANNELS];
void* buffer;
float time_step = 1.0 / SMP_RATE;
bool enbLine = false;
bool enbCos = false;
bool enbCubic = false;

size_t wrin;
uint8_t stp;
bool display_stat = true;

uint8_t part_table[128];
int8_t part_point = 0;
int8_t row_point = 0;
char song_name[20];
char samp_name[33][22];

#define BASE_FREQ 8267
bool dispRedy = false;
bool playStat = false;
uint32_t wave_MAX_L = 0;
uint32_t wave_MAX_R = 0;
uint32_t wave_MAX_L_OUT = 0;
uint32_t wave_MAX_R_OUT = 0;
bool MAX_COMP_FINISH = false;

bool enbFltr[4] = {false, false, false, false};
bool enbDelay[4] = {false, false, false, false};

float cutOffFreq[4] = {SMP_RATE/2, SMP_RATE/2, SMP_RATE/2, SMP_RATE/2};

const float patch_table[16] = {3546836.0, 3572537.835745873, 3598425.9175884672, 3624501.595143772, 3650766.227807657, 3677221.184826728, 3703867.8453697185, 3730707.5985993897,
                         3347767.391694687, 3372026.6942439806, 3396461.7896998064, 3421073.9519300302, 3445864.464033491, 3470834.61840689, 3495985.7168121682, 3521319.0704443706};

#define hexToDecimalTens(num) (((num) >> 4) & 0x0F)
#define hexToDecimalOnes(num) ((num) & 0x0F)
#define hexToRow(num) (hexToDecimalTens(num) * 10 + hexToDecimalOnes(num))
#define freq_up(base_freq, n) ((base_freq) * powf(2.0f, ((n) / 12.0f)))

#define drawMidRect(w, h, color) \
    frame.drawRect(80 - ((w) >> 1), 64 - ((h) >> 1), (80 + ((w) >> 1)) - (80 - ((w) >> 1)), (64 + ((h) >> 1)) - (64 - ((h) >> 1)), (color))

#define fillMidRect(w, h, color) \
    frame.fillRect(80 - ((w) >> 1), 64 - ((h) >> 1), (80 + ((w) >> 1)) - (80 - ((w) >> 1)), (64 + ((h) >> 1)) - (64 - ((h) >> 1)), (color))

#define setMidCusr(w, h, ofst) \
    frame.setCursor(80 - ((w) >> 1) + (ofst), 64 - ((h) >> 1) + (ofst))

inline int clamp(int value, int min, int max) {
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}
float lmt = 0;
uint16_t lmtP = 0;
float comper = 1;

void limit(float a) {
    lmt += 2 + (a > 0 ? a : -a) * 15 / 512.0f;
    lmtP++;
    comper = lmt / lmtP;
    if (lmtP >= 4096) {
        lmt = 0;
        lmtP = 0;
    }
}

#define DELAY_BUFFER_SIZE 8192

float delayBuffer[4][DELAY_BUFFER_SIZE];
int delayWriteIndex[4] = {0, 0, 0, 0};

void initDelayBuffer() {
    for (int i = 0; i < DELAY_BUFFER_SIZE; ++i) {
        delayBuffer[0][i] = 0.0f;
        delayBuffer[1][i] = 0.0f;
        delayBuffer[2][i] = 0.0f;
        delayBuffer[3][i] = 0.0f;
    }
}

float audioDelay(float inputSample, int delayLength, float decayRate, float dryMix, float wetMix, uint8_t chl) {
    if (delayLength >= DELAY_BUFFER_SIZE) {
        delayLength = DELAY_BUFFER_SIZE - 1;
    }

    int readIndex = delayWriteIndex[chl] - delayLength;
    if (readIndex < 0) {
        readIndex += DELAY_BUFFER_SIZE;
    }

    float outputSample = delayBuffer[chl][readIndex];
    delayBuffer[chl][delayWriteIndex[chl]] = inputSample + decayRate * outputSample;
    delayWriteIndex[chl] = (delayWriteIndex[chl] + 1) & (DELAY_BUFFER_SIZE - 1);
    return dryMix * inputSample + wetMix * outputSample;
}

// 适用于每次输入单个样本点的一阶低通滤波器
static float lastOutputs[NUM_CHANNELS] = {0};

float lowPassFilterSingleSample(float sample, int chl, float cutoffFreqIn, uint16_t sampleRate) {
    if (chl < 0 || chl >= NUM_CHANNELS) {
        printf("Channel index out of range.\n");
        return sample;
    }
    if ((uint16_t)cutoffFreqIn == sampleRate || cutoffFreqIn <= 0) return sample;
    float alpha = cutoffFreqIn / (cutoffFreqIn + sampleRate);
    return lastOutputs[chl] = alpha * sample + (1 - alpha) * lastOutputs[chl];
}

// 读取文件并存储到动态分配的内存中
uint8_t* read_file(const char* path, long* file_size) {
    FILE* file = fopen(path, "rb"); // 以二进制只读模式打开文件
    if (!file) {
        printf("Failed to open file for reading\n");
        return NULL;
    }

    // 获取文件大小
    fseek(file, 0, SEEK_END);
    *file_size = ftell(file);
    fseek(file, 0, SEEK_SET);

    // 动态分配内存来存储文件内容
    uint8_t* buffer = (uint8_t*)realloc(buffer, *file_size);
    if (!buffer) {
        printf("Failed to allocate memory\n");
        fclose(file);
        return NULL;
    }

    // 读取文件内容到动态分配的内存中
    size_t bytes_read = fread(buffer, 1, *file_size, file);
    if (bytes_read != *file_size) {
        printf("Failed to read file\n");
        fclose(file);
        free(buffer);
        return NULL;
    }

    fclose(file);
    return buffer;
}

long read_tracker_file(const char* path) {
    printf("NOW READ FILE %s\n", path);
    // free(tracker_data);
    long file_size;
    if (tracker_data != NULL) {
        free(tracker_data);
        printf("FREE MEM\n");
        tracker_data = NULL;
    }
    tracker_data = read_file(path, &file_size);
    printf("READ FINISH\n");
    if ((tracker_data == NULL) || (file_size == 0)) {
        printf("READ %s ERROR! MALLOC FAILED! FILE SIZE IS %ld\n", path, file_size);
        free(tracker_data);
        tracker_data = NULL;
        return 0;
    } else {
        if ((tracker_data[1080] != 0x4D) ||
            (tracker_data[1081] != 0x2E) ||
            (tracker_data[1082] != 0x4B) ||
            (tracker_data[1083] != 0x2E)) {
            printf("READ %s ERROR! NOT A M.K. MOD FILE! HEAD=%c%c%c%c\n", path, tracker_data[1080], tracker_data[1081], tracker_data[1082], tracker_data[1083]);
            free(tracker_data);
            tracker_data = NULL;
            return -1;
        }
        printf("READ %s FINISH! FILE SIZE IS %ld\n", path, file_size);
    }
    return file_size;
}

bool new_tracker_file() {
    // free(tracker_data);
    // tracker_data = (uint8_t *)malloc(2108);
    tracker_data = (uint8_t *)realloc(tracker_data, 2108);
    if (tracker_data == NULL) {
        printf("CREATE NEW FILE ON MEM FAILED!\n");
        return 1;
    }
    for (uint16_t p = 0; p < 2108; p++) {
        tracker_data[p] = tracker_null[p];
    }
    printf("CREATE NEW FILE ON MEM FINISH!\n");
    return 0;
}
void comp(void *arg);
void load(void *arg);
// **************************INIT*END****************************

char ten[24];
int8_t vol[CHL_NUM] = {0, 0, 0, 0};
int16_t period[4] = {0, 0, 0, 0};
float frq[4] = {0, 0, 0, 0};
float data_index[CHL_NUM] = {0, 0, 0, 0};
bool view_mode = false;
// uint16_t data_index_int[CHL_NUM] = {0};
uint8_t smp_num[CHL_NUM] = {0, 0, 0, 0};

int16_t temp;
uint16_t wave_info[33][5];
uint32_t wav_ofst[34];
bool mute[4] = {false, false, false, false};

// OSC START -----------------------------------------------------
void display(void *arg) {
    SSD1306_t dev;
    i2c_master_init(&dev, 1, 2, -1);
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);
    // bool bs = 0;
    ssd1306_display_text(&dev, 2, (char *)"LOADING....", 14, false);
    vTaskDelay(2);
    for (;;) {
        vTaskDelay(2);
        if (dispRedy) {
            break;
        }
    }
    for (;;) {
        uint8_t x;
        uint8_t volTemp;
        uint8_t addr[4];
        if (view_mode) {
            if (MAX_COMP_FINISH) {
                MAX_COMP_FINISH = false;
                ssd1306_clear_buffer(&dev);
                ssd1306_display_text(&dev, 0, "VIEW MODE", 10, false);
                for (uint8_t i = 0; i < 16; i++) {
                    _ssd1306_line(&dev, 0, i+16, wave_MAX_L_OUT>>8, i+16, false);
                }
                for (uint8_t i = 0; i < 16; i++) {
                    _ssd1306_line(&dev, 0, i+34, wave_MAX_R_OUT>>8, i+34, false);
                }
                ssd1306_show_buffer(&dev);
            } else {
                vTaskDelay(1);
            }
        } else {
            for (uint8_t contr = 0; contr < 4; contr++) {
                ssd1306_clear_buffer(&dev);
                sprintf(ten, " %d", esp_get_free_heap_size());
                // sprintf(ten, "  %2d %2d>%2d", row_point, showPart, part_table[showPart], comper);
                addr[0] = data_index[0] * (32.0f / wave_info[smp_num[0]][0]);
                addr[1] = data_index[1] * (32.0f / wave_info[smp_num[1]][0]);
                addr[2] = data_index[2] * (32.0f / wave_info[smp_num[2]][0]);
                addr[3] = data_index[3] * (32.0f / wave_info[smp_num[3]][0]);
                // ssd1306_display_text(&dev, 7, tet, 16, false);
                ssd1306_display_text(&dev, 0, (char *)"CH1 CH2 CH3 CH4", 16, false);
                ssd1306_display_text(&dev, 6, ten, 16, false);
                if (!mute[0]) {
                for (x = 0; x < 32; x++) {
                    _ssd1306_line(&dev, x, 32, x, (uint8_t)(((buffer_ch[0][(x + (contr * 128)) * 2]) / 256) + 32)&63, false);
                    // _ssd1306_pixel(&dev, x, ((buffer_ch[0][(x + (contr * 128)) * 2]) / 256) + 32, false);
                    if (period[0]) {
                        _ssd1306_pixel(&dev, x, (uint8_t)(period[0] * (64.0f / 743.0f))&63, false);
                    }
                    // printf("DISPLAY %d\n", roundf(period[0] * (64.0f / 743.0f)));
                    volTemp = (vol[0]/2) % 64;
                    _ssd1306_line(&dev, addr[0], 8, addr[0], 47, false);
                    if (x < volTemp) {
                        _ssd1306_line(&dev, x, 58, x, 63, false);
                    }
                }} else {
                    _ssd1306_line(&dev, 0, 0, 31, 63, false);
                    _ssd1306_line(&dev, 31, 0, 0, 63, false);
                }
                if (!mute[1]) {
                for (x = 32; x < 64; x++) {
                    _ssd1306_line(&dev, x, 32, x, (uint8_t)(((buffer_ch[1][(x + (contr * 128)) * 2]) / 256) + 32)&63, false);
                    // _ssd1306_pixel(&dev, x, ((buffer_ch[1][((x-32) + (contr * 128)) * 2]) / 256) + 32, false);
                    if (period[1]) {
                        _ssd1306_pixel(&dev, x, (uint8_t)(period[1] * (64.0f / 743.0f))&63, false);
                    }
                    volTemp = vol[1]/2;
                    _ssd1306_line(&dev, addr[1]+32, 8, addr[1]+32, 47, false);
                    if (x - 32 < volTemp) {
                        _ssd1306_line(&dev, x, 58, x, 63, false);
                    }
                }} else {
                    _ssd1306_line(&dev, 32, 0, 63, 63, false);
                    _ssd1306_line(&dev, 63, 0, 32, 63, false);
                }
                if (!mute[2]) {
                for (x = 64; x < 96; x++) {
                    _ssd1306_line(&dev, x, 32, x, (uint8_t)(((buffer_ch[2][(x + (contr * 128)) * 2]) / 256) + 32)&63, false);
                    // _ssd1306_pixel(&dev, x, ((buffer_ch[2][((x-64) + (contr * 128)) * 2]) / 256) + 32, false);
                    if (period[2]) {
                        _ssd1306_pixel(&dev, x, (uint8_t)(period[2] * (64.0f / 743.0f))&63, false);
                    }
                    volTemp = vol[2]/2;
                    _ssd1306_line(&dev, addr[2]+64, 8, addr[2]+64, 47, false);
                    if (x - 64 < volTemp) {
                        _ssd1306_line(&dev, x, 58, x, 63, false);
                    }
                }} else {
                    _ssd1306_line(&dev, 64, 0, 95, 63, false);
                    _ssd1306_line(&dev, 95, 0, 64, 63, false);
                }
                if (!mute[3]) {
                for (x = 96; x < 128; x++) {
                    _ssd1306_line(&dev, x, 32, x, (uint8_t)(((buffer_ch[3][(x + (contr * 128)) * 2]) / 256) + 32)&63, false);
                    // _ssd1306_pixel(&dev, x, ((buffer_ch[3][((x-96) + (contr * 128)) * 2]) / 256) + 32, false);
                    if (period[3]) {
                        _ssd1306_pixel(&dev, x, (uint8_t)(period[3] * (64.0f / 743.0f))&63, false);
                    }
                    volTemp = vol[3]/2;
                    _ssd1306_line(&dev, addr[3]+96, 8, addr[3]+96, 47, false);
                    if (x - 96 < volTemp) {
                        _ssd1306_line(&dev, x, 58, x, 63, false);
                    }
                }} else {
                    _ssd1306_line(&dev, 96, 0, 127, 63, false);
                    _ssd1306_line(&dev, 127, 0, 96, 63, false);
                }
                ssd1306_show_buffer(&dev);
                vTaskDelay(1);
            }
        }
    }
}
// OSC END -------------------------------------------------------

inline void read_part_data(uint8_t* tracker_data, uint8_t pattern_index, uint8_t row_index, uint8_t channel_index, uint16_t part_data[4]) {
    uint8_t* pattern_data = tracker_data + 1084 + pattern_index * NUM_ROWS * NUM_CHANNELS * 4;

    uint16_t byte_index = row_index * NUM_CHANNELS * 4 + channel_index * 4;
    // Byte structure for tracker data:
    // +----------------+--------------+----------------+-------------+
    // |    byte 1      |    byte 2    |     byte 3     |    byte 4   |
    // |----------------|--------------|----------------|-------------|
    // | 0000       0000|00000000      | 0000       0000|00000000     |
    // |________________|______________|________________|_____________|
    // | Upper four     | 12 bits for  | Lower four     | Effect      |
    // | bits of sample | note period. | bits of sample | command.    |
    // | number.        |              | number.        |             |
    // +----------------+--------------+----------------+-------------+

    uint8_t byte1 = pattern_data[byte_index];
    uint8_t byte2 = pattern_data[byte_index + 1];
    uint8_t byte3 = pattern_data[byte_index + 2];
    uint8_t byte4 = pattern_data[byte_index + 3];

    // part_data[ROW][CHAN][NOTE_DATA]
    //                          |
    //                          |- 0: Amiga period
    //                          |- 1: Sample number
    //                          |- 2: Effect1 (Eee)
    //                          +- 3: Effect2 (eEE)
    part_data[0] = ((byte1 & 0x0F) << 8) | byte2;
    part_data[1] = (byte1 & 0xF0) | (byte3 >> 4);
    part_data[2] = (byte3 & 0x0F);
    part_data[3] = byte4;
}

void write_part_data(uint8_t *tracker_data, uint8_t pattern_index, uint8_t chl, uint8_t row, uint8_t row_data[4]) {
    uint8_t* pattern_data = tracker_data + 1084 + pattern_index * NUM_ROWS * NUM_CHANNELS * 4;

    uint16_t byte_index = row * NUM_CHANNELS * 4 + chl * 4;

    pattern_data[byte_index] = (row_data[1] & 0xF0) | ((row_data[0] >> 8) & 0x0F);
    pattern_data[byte_index + 1] = row_data[0] & 0xFF;
    pattern_data[byte_index + 2] = (row_data[1] << 4) | (row_data[2] & 0x0F);
    pattern_data[byte_index + 3] = row_data[3];
}

// AUDIO DATA COMP START ----------------------------------------
uint8_t arpNote[2][4] = {0};
float arpFreq[3][4];
int8_t lastVol[4];
float sin_index;
inline float make_data(float freq, uint8_t vole, uint8_t chl, bool isLoop, uint16_t loopStart, uint16_t loopLen, uint32_t smp_start, uint16_t smp_size, bool isline, bool isCos, bool isCubic) {
    // 如果音量为0，频率小于等于0，或该通道被静音，则返回0
    if (vole == 0 || freq <= 0 || mute[chl]) {
        return 0;
    }

    // 更新数据索引，根据频率和采样率计算新的数据索引
    data_index[chl] += freq / SMP_RATE;

    // 如果启用了循环并且数据索引超出循环范围，则重置数据索引到循环起点
    if (isLoop && data_index[chl] >= (loopStart + loopLen)) {
        data_index[chl] = loopStart;
    // 如果未启用循环且数据索引超出样本大小，则将音量设置为0并返回0
    } else if (!isLoop && data_index[chl] >= smp_size) {
        vol[chl] = 0;
        return 0;
    }

    // 获取当前索引位置的采样值，并根据音量调整
    int16_t sample1 = ((int8_t)tracker_data[(int)data_index[chl] + smp_start] * (vole << 1));
    float result;

    // 确保只能启用一种插值算法
    if ((isline && isCos) || (isline && isCubic) || (isCos && isCubic)) {
        // 如果多种插值算法都启用，抛出异常或设置默认行为（这里选择禁用线性和余弦插值，只启用三次样条插值）
        isline = false;
        isCos = false;
    }

    // 如果启用了线性插值且当前索引小于样本大小的最后一个索引，则进行线性插值
    if (isline && (int)data_index[chl] < smp_size - 1) {
        // 获取下一个索引位置的采样值，并根据音量调整
        int16_t sample2 = ((int8_t)tracker_data[(int)data_index[chl] + 1 + smp_start] * (vole << 1));
        // 计算当前索引的小数部分
        float frac = data_index[chl] - (int)data_index[chl];
        // 使用线性插值计算结果
        result = (1.0f - frac) * sample1 + frac * sample2;
    }
    // 如果启用了余弦插值且当前索引小于样本大小的最后一个索引，则进行余弦插值
    else if (isCos && (int)data_index[chl] < smp_size - 1) {
        // 获取下一个索引位置的采样值，并根据音量调整
        int16_t sample2 = ((int8_t)tracker_data[(int)data_index[chl] + 1 + smp_start] * (vole << 1));
        // 计算当前索引的小数部分
        float frac = data_index[chl] - (int)data_index[chl];
        // 计算余弦插值因子
        float f_t = (1 - cos(M_PI * frac)) / 2;
        // 使用余弦插值计算结果
        result = (1.0f - f_t) * sample1 + f_t * sample2;
    }
    // 如果启用了三次样条插值且当前索引小于样本大小的倒数第三个索引，则进行三次样条插值
    else if (isCubic && (int)data_index[chl] < smp_size - 2) {
        // 获取周围四个采样值，并根据音量调整
        int16_t y0 = ((int8_t)tracker_data[(int)data_index[chl] - 1 + smp_start] * (vole << 1));
        int16_t y1 = sample1;
        int16_t y2 = ((int8_t)tracker_data[(int)data_index[chl] + 1 + smp_start] * (vole << 1));
        int16_t y3 = ((int8_t)tracker_data[(int)data_index[chl] + 2 + smp_start] * (vole << 1));
        // 计算当前索引的小数部分
        float frac = data_index[chl] - (int)data_index[chl];
        // 使用三次样条插值公式计算结果
        result = y1 + 0.5f * frac * (y2 - y0 + frac * (2.0f * y0 - 5.0f * y1 + 4.0f * y2 - y3 + frac * (3.0f * (y1 - y2) + y3 - y0)));
    }
    else {
        // 如果未启用任何插值，则直接使用当前采样值
        result = sample1;
    }

    // 如果启用了滤波器，则对结果应用低通滤波
    if (enbFltr[chl]) {
        result = lowPassFilterSingleSample(result, chl, cutOffFreq[chl], SMP_RATE);
    }
    // 如果启用了延迟效果，则对结果应用延迟处理
    if (enbDelay[chl]) {
        result = audioDelay(result, 2048, 0.23f, 0.8f, 0.2f, chl);
    }
    
    // 返回处理后的结果
    return result;
}
// AUDIO DATA COMP END ------------------------------------------

// float frq[CHL_NUM] = {0};

uint16_t part_buffer[4];
bool loadOk = true;

bool skipToNextPart = false;
uint8_t skipToAnyPart = false;
bool lcdOK = true;
uint8_t BPM = 125;
uint8_t SPD = 6;
/*
bool keyOK = false;
bool keyUP = false;
bool keyDOWN = false;
bool keyL = false;
bool keyR = false;
bool keySpace = false;
*/
int8_t ChlPos = 0;
// bool ChlMenu = false;
int8_t ChlMenuPos = 0;
bool windowsClose = true;
int8_t MenuPos = 2;
TaskHandle_t COMP;
TaskHandle_t LOAD;
bool partUP = false;
bool partDOWN = false;

#define NUM_RANGES 3
#define NOTES_PER_RANGE 12
uint16_t midi_period_table[NUM_RANGES][NOTES_PER_RANGE] = {
    // C-1 to B-1
    {856, 808, 762, 720, 678, 640, 604, 570, 538, 508, 480, 453},
    // C-2 to B-2
    {428, 404, 381, 360, 339, 320, 302, 285, 269, 254, 240, 226},
    // C-3 to B-3
    {214, 202, 190, 180, 170, 160, 151, 143, 135, 127, 120, 113}
};
typedef struct {
    const int frequency;
    const char *note_name;
} Note;
Note period_table[NUM_RANGES][NOTES_PER_RANGE] = {
    // C-1 to B-1
    {{856, "C-1"}, {808, "C#1"}, {762, "D-1"}, {720, "D#1"}, {678, "E-1"}, {640, "F-1"},
        {604, "F#1"}, {570, "G-1"}, {538, "G#1"}, {508, "A-1"}, {480, "A#1"}, {453, "B-1"}},
    // C-2 to B-2
    {{428, "C-2"}, {404, "C#2"}, {381, "D-2"}, {360, "D#2"}, {339, "E-2"}, {320, "F-2"},
        {302, "F#2"}, {285, "G-2"}, {269, "G#2"}, {254, "A-2"}, {240, "A#2"}, {226, "B-2"}},
    // C-3 to B-3
    {{214, "C-3"}, {202, "C#3"}, {190, "D-3"}, {180, "D#3"}, {170, "E-3"}, {160, "F-3"},
        {151, "F#3"}, {143, "G-3"}, {135, "G#3"}, {127, "A-3"}, {120, "A#3"}, {113, "B-3"}}
};
const char* findNote(int frequency) {
    for (int i = 0; i < NUM_RANGES; ++i) {
        for (int j = 0; j < NOTES_PER_RANGE; ++j) {
            if (period_table[i][j].frequency == frequency) {
                return period_table[i][j].note_name; // 找到对应的音符，返回音符名
            }
        }
    }
    return "???";
}
const char* fileSelect(const char* root_path) {
    playStat = false;
    char path[256];
    uint8_t path_depth = 0;
    int16_t SelPos = 0;
    uint8_t pg = 0;
    FileInfo* files = NULL;
    frame.drawFastHLine(0, 9, 160, 0xe71c);
    sprintf(path, "%s", root_path);
    printf("%s\n", path);
    int count = list_directory(path, &files);
    bool emyDir = false;
    key_event_t optionKeyEvent;
    for (;;) {
        char* showBuf;
        frame.fillRect(0, 10, 160, 118, ST7735_BLACK);
        frame.setCursor(0, 11);
        frame.printf("OPEN FILE IN\n");
        // frame.setCursor(0, 20);
        if (count < 0) {
            frame.printf("Failed to list directory.\n");
            return "FAIL";
        }
        showBuf = shortenFileName(path, 12);
        // shortenFileName(root_path, 12, showBuf);
        frame.printf("%s: %d FILE/DIR\n", showBuf, count);
        free(showBuf);
        frame.fillRect(0, ((SelPos%10)*10)+28, 160, 11, 0x528a);
        frame.drawFastHLine(0, 27, 160, 0xa6bf);
        frame.setCursor(frame.getCursorX(), frame.getCursorY()+3);
        frame.drawFastVLine(134, 28, 100, 0xa6bf);
        if (count) {
            pg = SelPos / 10;
            for (uint16_t i = 0; i < 10; i++) {
                uint16_t showPos = i+(pg*10);
                if (showPos >= count) {
                    break;
                }
                showBuf = shortenFileName(files[showPos].name, 22);
                frame.printf("%s", showBuf);
                frame.setCursor(138, frame.getCursorY());
                frame.printf("%s\n", files[showPos].is_directory ? "DIR" : "FIL");
                frame.setCursor(frame.getCursorX(), frame.getCursorY()+2);
                free(showBuf);
            }
        } else {            
            frame.printf("Empty directory\n");
            frame.setCursor(frame.getCursorX(), frame.getCursorY()+2);
            frame.printf("..");
            SelPos = 1;
            emyDir = true;
        }
        playStat = false;
        frame.display();
        vTaskDelay(2);
        if (readOptionKeyEvent == pdTRUE) { if (optionKeyEvent.status == KEY_ATTACK) {
            switch (optionKeyEvent.num) {
            case KEY_L:
                if (path_depth > 0) {
                    for (uint16_t i = 0; i < count; i++) {
                        free(files[i].name);
                    }
                    free(files);
                    files = NULL;
                    char *lastSlash = strrchr(path, '/'); // 查找最后一个斜杠
                    if (lastSlash != NULL) {
                        *lastSlash = '\0'; // 将最后一个斜杠替换为字符串结束符
                    }
                    emyDir = false;
                    count = list_directory(path, &files);
                    printf("%s\n", path);
                    path_depth--;
                    SelPos = 0;
                }
                break;
            case KEY_UP:
                SelPos--;
                if (SelPos < 0) {
                    SelPos = count-1;
                }
                break;
            case KEY_DOWN:
                SelPos++;
                if (SelPos > count-1) {
                    SelPos = 0;
                }
                break;
            }
            if (optionKeyEvent.num == KEY_OK) {
                if (emyDir) {
                    char *lastSlash = strrchr(path, '/'); // 查找最后一个斜杠
                    if (lastSlash != NULL) {
                        *lastSlash = '\0'; // 将最后一个斜杠替换为字符串结束符
                    }
                    emyDir = false;
                    count = list_directory(path, &files);
                    printf("%s\n", path);
                    SelPos = 0;
                    path_depth--;
                    continue;
                }
                if (files[SelPos].is_directory) {
                    if (strlen(path)+strlen(files[SelPos].name) > 255) {
                        printf("PATH TO LONG\n");
                        continue;
                    }
                    path_depth++;
                    printf("PATH %p\n", path);
                    sprintf(path, "%s/%s", path, files[SelPos].name);
                    // strcat(path, "/");
                    // strcat(path, files[SelPos].name);
                    for (uint16_t i = 0; i < count; i++) {
                        free(files[i].name);
                    }
                    free(files);
                    files = NULL;
                    count = list_directory(path, &files);
                    printf("%s\n", path);
                    SelPos = 0;
                } else {
                    break;
                }
            }
        }}
    }
    fillMidRect(80, 20, 0x4208);
    drawMidRect(80, 20, ST7735_WHITE);
    setMidCusr(80, 20, 7);
    frame.setTextColor(0x7bcf);
    frame.printf("READING...");
    setMidCusr(80, 20, 6);
    frame.setTextColor(ST7735_WHITE);
    frame.printf("READING...");
    frame.display();
    char* full_path = (char*)malloc(strlen(files[SelPos].name) + strlen(path) + 2);
    sprintf(full_path, "%s/%s", path, files[SelPos].name);
    for (uint16_t i = 0; i < count; i++) {
        free(files[i].name);
    }
    free(files);
    files = NULL;
    return full_path;
}

inline void MainReDraw() {
    frame.fillScreen(ST7735_BLACK);
    frame.setTextWrap(true);
    frame.setTextSize(1);
    frame.setTextColor(0x2945);
    frame.setCursor(2, 2);
    frame.print("ESP32Tracker");
    frame.setTextColor(0x7bcf);
    frame.setCursor(1, 1);
    frame.print("ESP32Tracker      libchara");
    frame.setTextColor(ST7735_WHITE);
    frame.setCursor(0, 0);
    frame.print("ESP32Tracker");
    frame.setTextColor(ST7735_WHITE);
    frame.setTextSize(0);
}

inline void fileOpt() {
    key_event_t optionKeyEvent;
    for (;;) {
        long ret = read_tracker_file(fileSelect("/sdcard"));
        if (ret == -1) {
            fillMidRect(148, 20, 0x4208);
            drawMidRect(148, 20, ST7735_WHITE);
            setMidCusr(148, 20, 7);
            frame.setTextColor(0x7bcf);
            frame.printf("THIS IS NOT A MOD FILE!");
            setMidCusr(148, 20, 6);
            frame.setTextColor(ST7735_WHITE);
            frame.printf("THIS IS NOT A MOD FILE!");
            frame.display();
            while(readOptionKeyEvent != pdTRUE) {
                vTaskDelay(4);
            }
            MainReDraw();
        } else if (ret == 0) {
            fillMidRect(148, 20, 0x4208);
            drawMidRect(148, 20, ST7735_WHITE);
            setMidCusr(148, 20, 7);
            frame.setTextColor(0x7bcf);
            frame.printf("THIS FILE IS TOO LARGE!");
            setMidCusr(148, 20, 6);
            frame.setTextColor(ST7735_WHITE);
            frame.printf("THIS FILE IS TOO LARGE!");
            frame.display();
            while(readOptionKeyEvent != pdTRUE) {
                vTaskDelay(4);
            }
            MainReDraw();
        } else {
            break;
        }
    }
}

void loadFileInit() {
    part_point = 0;
    row_point = 0;
    data_index[0] = data_index[1] = data_index[2] = data_index[3] = 0;
    mute[0] = mute[1] = mute[2] = mute[3] = 0;
    period[0] = period[1] = period[2] = period[3] = 0;
    frq[0] = frq[1] = frq[2] = frq[3] = 0;
    fileOpt();
    read_pattern_table();
    read_wave_info();
    comp_wave_ofst();
    windowsClose = true;
    MainReDraw();
    frame.drawFastHLine(0, 9, 160, 0xe71c);
    frame.drawFastHLine(0, 18, 160, 0xe71c);
    frame.fillRect(113, 19, 47, 7, 0xa514);
    frame.drawFastVLine(112, 19, 24, ST7735_WHITE);
    frame.fillRect(0, 10, 160, 8, 0x2104);
    row_point = 0;
}

void newFileInit() {
    part_point = 0;
    row_point = 0;
    data_index[0] = data_index[1] = data_index[2] = data_index[3] = 0;
    mute[0] = mute[1] = mute[2] = mute[3] = 0;
    period[0] = period[1] = period[2] = period[3] = 0;
    frq[0] = frq[1] = frq[2] = frq[3] = 0;
    new_tracker_file();
    read_pattern_table();
    read_wave_info();
    comp_wave_ofst();
    windowsClose = true;
    MainReDraw();
    frame.drawFastHLine(0, 9, 160, 0xe71c);
    frame.drawFastHLine(0, 18, 160, 0xe71c);
    frame.fillRect(113, 19, 47, 7, 0xa514);
    frame.drawFastVLine(112, 19, 24, ST7735_WHITE);
    frame.fillRect(0, 10, 160, 8, 0x2104);
    row_point = 0;
}

uint16_t showTmpNote;
uint8_t showTmpSamp;
uint8_t showTmpEFX1;
uint8_t showTmpEFX2_1;
uint8_t showTmpEFX2_2;

void windowsMenu(const char *title, uint8_t current_option, uint8_t total_options, uint8_t Xlen, ...) {
    va_list args;
    va_start(args, total_options);
    uint8_t OriginX = frame.getCursorX();
    uint8_t OriginY = frame.getCursorY();
    uint8_t Ylen = ((total_options)*11)+18;
    fillMidRect(Xlen, Ylen, 0x4208);
    drawMidRect(Xlen, Ylen, ST7735_WHITE);
    frame.setTextColor(0x7bcf);
    setMidCusr(Xlen, Ylen, 3);
    frame.printf("%s", title);
    frame.setTextColor(ST7735_WHITE);
    setMidCusr(Xlen, Ylen, 2);
    frame.printf("%s\n", title);
    setMidCusr(Xlen, Ylen, 2);
    uint8_t CXTmp = frame.getCursorX()+4;
    frame.setCursor(CXTmp, frame.getCursorY()+14);
    frame.fillRect(frame.getCursorX()-2, (frame.getCursorY()-2)+((current_option-1)*11), Xlen - 10, 11, 0x7bef);
    for (uint8_t q = 0; q < total_options; q++) {
        frame.printf("%s\n", va_arg(args, const char *));
        frame.setCursor(CXTmp, frame.getCursorY()+3);
    }
    frame.setCursor(OriginX, OriginY);
}

int8_t windowsMenuBlocking(const char *title, uint8_t total_options, uint8_t opt_init_val, uint8_t Xlen, ...) {
    key_event_t optionKeyEvent;
    va_list args;
    va_start(args, total_options);
    uint8_t OriginX = frame.getCursorX();
    uint8_t OriginY = frame.getCursorY();
    uint8_t Ylen = ((total_options+1)*11)+18;
    int8_t current_option = opt_init_val;
    char * menuStr[total_options];
    for (uint8_t i = 0; i < total_options; i++) {
        menuStr[i] = (char *)va_arg(args, const char *);
    }
    for (;;) {
        fillMidRect(Xlen, Ylen, 0x4208);
        drawMidRect(Xlen, Ylen, ST7735_WHITE);
        frame.setTextColor(0x7bcf);
        setMidCusr(Xlen, Ylen, 3);
        frame.printf("%s", title);
        frame.setTextColor(ST7735_WHITE);
        setMidCusr(Xlen, Ylen, 2);
        frame.printf("%s\n", title);
        setMidCusr(Xlen, Ylen, 2);
        uint8_t CXTmp = frame.getCursorX()+4;
        frame.setCursor(CXTmp, frame.getCursorY()+14);
        frame.fillRect(frame.getCursorX()-2, (frame.getCursorY()-2)+((current_option-1)*11), Xlen - 10, 11, 0x7bef);
        for (uint8_t q = 0; q < total_options; q++) {
            frame.printf("%s\n", menuStr[q]);
            frame.setCursor(CXTmp, frame.getCursorY()+3);
        }
        frame.printf("Close");
        if (readOptionKeyEvent == pdTRUE) { if (optionKeyEvent.status == KEY_ATTACK) {
            switch (optionKeyEvent.num)
            {
            case KEY_UP:
                current_option--;
                if (current_option < 1) current_option = total_options+1;
                break;
            case KEY_DOWN:
                current_option++;
                if (current_option > total_options+1) current_option = 1;
                break;
            }
            if (optionKeyEvent.num == 5) {
                frame.setCursor(OriginX, OriginY);
                if (current_option == total_options+1) break;
                else return current_option - 1;
            }
        }}
        frame.display();
        vTaskDelay(2);
    }
    frame.setCursor(OriginX, OriginY);
    return -1;
}

void MainPage() {
    frame.drawFastHLine(0, 9, 160, 0xe71c);
    frame.drawFastHLine(0, 18, 160, 0xe71c);
    frame.fillRect(113, 19, 47, 7, 0xa514);
    frame.drawFastVLine(112, 19, 24, ST7735_WHITE);
    frame.fillRect(0, 10, 160, 8, 0x2104);
    uint8_t sideMenu = 0;
    uint8_t fileMenu = 0;
    uint16_t viewTmp[4];
    key_event_t optionKeyEvent;
    BaseType_t PerStat = pdFALSE;
    bool fnA = false;
    for (;;) {
        frame.setCursor(0, 10);
        frame.print(song_name);
        if (windowsClose) {
            windowsClose = false;
            frame.fillRect(0, 24, 160, 30, ST7735_BLACK);
            frame.fillRect(113, 19, 47, 7, 0xa514);

            // SIDE MENU
            frame.drawFastHLine(0, 40, 112, 0xe71c);
            frame.drawFastVLine(37, 40, 15, 0xe71c);
            frame.drawFastVLine(74, 40, 15, 0xe71c);
            frame.drawFastVLine(0, 40, 15, 0xe71c);
            frame.drawFastVLine(111, 40, 15, 0xe71c);

            // TRACKER LINE
            frame.drawFastHLine(0, 54, 160, ST7735_WHITE);

            // PAT VIEW LINE
            frame.drawFastVLine(112, 19, 35, ST7735_WHITE);

            // TRACKER CHL LINE
            frame.drawFastHLine(0, 55, 160, 0xa514);
            frame.drawFastVLine(156, 56, 72, 0x4a51);
            frame.drawFastVLine(157, 56, 72, 0x6b7e);
            frame.drawFastVLine(158, 56, 72, 0xad7f);
            frame.drawFastVLine(159, 56, 72, 0xa514);

            frame.drawFastVLine(12, 56, 72, 0x4a51);
            frame.drawFastVLine(13, 56, 72, 0x6b7e);
            frame.drawFastVLine(14, 56, 72, 0xad7f);

            frame.drawFastVLine(48, 56, 72, 0x4a51);
            frame.drawFastVLine(49, 56, 72, 0x6b7e);
            frame.drawFastVLine(50, 56, 72, 0xad7f);

            frame.drawFastVLine(84, 56, 72, 0x4a51);
            frame.drawFastVLine(85, 56, 72, 0x6b7e);
            frame.drawFastVLine(86, 56, 72, 0xad7f);

            frame.drawFastVLine(120, 56, 72, 0x4a51);
            frame.drawFastVLine(121, 56, 72, 0x6b7e);
            frame.drawFastVLine(122, 56, 72, 0xad7f);
            printf("ReDraw\n");
        }
        //----------------------MAIN PAGE-------------------------
        if (ChlPos == 0 && !sideMenu) {
            frame.fillRect(0, 56, 13, 72, 0x630c);
        } else {
            frame.fillRect(0, 56, 13, 72, 0x2945);
        }
        for (uint8_t x = 1; x < 5; x++) {
            if (ChlPos == x) {
                frame.fillRect(((x-1)*36)+15, 56, 33, 72, 0x630c);
            } else if (mute[x-1]) {
                frame.fillRect(((x-1)*36)+15, 56, 33, 72, 0xa514);
            } else {
                frame.fillRect(((x-1)*36)+15, 56, 33, 72, 0x2945);
            }
        }

        frame.drawRect(0, 87, 160, 9, 0xd69a);

        frame.setCursor(116, 29);
        frame.fillRect(115, 28, 21, 25, 0x2104);
        if (sideMenu == 4) {
            frame.fillRect(137, 28, 21, 25, 0x630c);
        } else {
            frame.fillRect(137, 28, 21, 25, 0x2104);
        }
        frame.drawRect(114, 27, 45, 27, 0xa514);
        frame.fillRect(115, 36, 43, 9, 0x7bcf);
        frame.drawRect(113, 26, 47, 29, ST7735_WHITE);
        frame.drawFastVLine(136, 28, 25, 0xd69a);

        for (int8_t i = -1; i < 2; i++) {
            if ((part_point+i >= 0) && (part_point+i < NUM_PATTERNS)) {
                frame.printf("%3d %3d\n", part_point+i, part_table[part_point+i]);
            } else {
                frame.printf("\n");
            }
            frame.setCursor(116, frame.getCursorY());
        }
        frame.setCursor(0, 56);
        for (int8_t i = -4; i < 5; i++) {
            if ((row_point+i < 64) && (row_point+i >= 0)) {
                frame.setTextColor(0xf7be);
                frame.printf("%2d", row_point+i);
                for (uint8_t chl = 0; chl < 4; chl++) {
                    read_part_data(tracker_data, part_table[part_point], row_point+i, chl, viewTmp);
                    showTmpNote = viewTmp[0];
                    showTmpSamp = viewTmp[1];

                    frame.setCursor(frame.getCursorX()+4, frame.getCursorY());
                    if (showTmpNote) {
                        frame.setTextColor(0xa51f);
                        frame.printf("%s", findNote(showTmpNote));
                    } else {
                        frame.setTextColor(0x52aa);
                        frame.printf("...");
                    }
                    frame.setCursor(frame.getCursorX()+2, frame.getCursorY());
                    if (showTmpSamp) {
                        frame.setTextColor(0x2c2a);
                        frame.printf("%2d", showTmpSamp);
                    } else {
                        frame.setTextColor(0x52aa);
                        frame.printf("..");
                    }
                }
                frame.printf("\n");
            } else {
                frame.printf("\n");
            }
        }
        read_part_data(tracker_data, part_table[part_point], row_point, ChlPos-1, viewTmp);
        frame.setTextColor(ST7735_WHITE);

        frame.fillRect(0, 19, 112, 21, 0x3186);
        frame.setCursor(1, 20);
        frame.printf("BPM: %3d  OCT: %2d", BPM, inputOct);

        frame.setCursor(1, 28);
        frame.printf("SPD: %2d   SMP: %2d", SPD, ChlPos > 0 ? smp_num[ChlPos-1] : 0);

        PerStat = readOptionKeyEvent;

        if (PerStat == pdTRUE && optionKeyEvent.num == KEY_A) {
            if (optionKeyEvent.status == KEY_ATTACK) fnA = true;
            else fnA = false;
        }

        // printf("FnA STAT %d\n", fnA);

        if (PerStat == pdTRUE && optionKeyEvent.num == KEY_SPACE) {
            PerStat = pdFALSE;
            if (!playStat) row_point = 0;
            playStat = !playStat;
        }

        if (sideMenu) {
            for (uint8_t i = 0; i < 3; i++) {
                if (sideMenu-1 == i) {
                    frame.fillRect(1+(i*37), 41, 36, 13, 0x630c);
                } else {
                    frame.fillRect(1+(i*37), 41, 36, 13, 0x2945);
                }
            }

            if (fileMenu) {
                // sideMenu = 2;
                windowsMenu("FILE", fileMenu, 5, 80, "New", "Open", "Save", "Setting", "Close");
                if (PerStat == pdTRUE) if (optionKeyEvent.status == KEY_ATTACK) {
                    PerStat = pdFALSE;
                    switch (optionKeyEvent.num)
                    {
                    case KEY_UP:
                        fileMenu--;
                        if (fileMenu < 1) fileMenu = 5;
                        break;
                    
                    case KEY_DOWN:
                        fileMenu++;
                        if (fileMenu > 5) fileMenu = 1;
                        break;
                    }
                    if (optionKeyEvent.num == KEY_OK) {
                        printf("KEY_OK PUSHED!\n");
                        if (fileMenu == 5) {
                            fileMenu = 0;
                            windowsClose = true;
                        }
                        if (fileMenu == 4) {
                            MenuPos = 3;
                            break;
                        }
                        if (fileMenu == 3) {
                            playStat = false;
                            recMod = true;
                            printf("SAVE!\n");
                            vTaskDelay(8);
                            // memset(buffer, 0, 6892 * sizeof(int16_t));
                            char *export_wave_name = (char*)malloc(strlen(song_name)+32);
                            sprintf(export_wave_name, "/sdcard/%s_record.wav", song_name);
                            export_wav_file = wav_audio_start(export_wave_name, 44100, 16, 2);
                            vTaskDelay(8);
                            free(export_wave_name);
                            playStat = true;
                        }
                        if (fileMenu == 2) {
                            playStat = false;
                            vTaskDelay(16);
                            sideMenu = 0;
                            fileMenu = 0;
                            loadFileInit();
                        }
                        if (fileMenu == 1) {
                            playStat = false;
                            vTaskDelay(16);
                            sideMenu = 0;
                            fileMenu = 0;
                            newFileInit();
                        }
                    }
                }
            }

            if (PerStat == pdTRUE) if (optionKeyEvent.status == KEY_ATTACK) {
                PerStat = pdFALSE;
                switch (optionKeyEvent.num)
                {
                case KEY_DOWN:
                    if (sideMenu == 4) partDOWN = true;
                    break;

                case KEY_UP:
                    if (sideMenu == 4) partUP = true;
                    break;

                case KEY_L:
                    sideMenu--;
                    if (sideMenu < 1) sideMenu = 4;
                    break;

                case KEY_R:
                    sideMenu++;
                    if (sideMenu > 4) sideMenu = 1;
                    break;
                }
                // printf("S2 %d\n", optionKeyEvent.num);
                if (optionKeyEvent.num == KEY_OK) {
                    printf("KEYOK OK!\n");
                    if (sideMenu == 3) {
                        printf("SAMP\n");
                    }
                    if (sideMenu == 2) {
                        // mute[ChlPos-1] = !mute[ChlPos-1];
                        fileMenu = 1;
                    }
                    if (sideMenu == 1) {
                        printf("CLOSE SIDEMENU\n");
                        sideMenu = 0;
                        windowsClose = true;
                    }
                }
            }
        }

        frame.setCursor(6, 44);
        frame.printf("CHAN");
        if (!fileMenu) {
        frame.setCursor(44, 44);
        frame.printf("FILE");
        frame.setCursor(82, 44);
        frame.printf("SAMP");}

        if (ChlMenuPos) {
            char menuShow[13];
            sprintf(menuShow, "CHL%d OPTION", ChlPos);
            windowsMenu(menuShow, ChlMenuPos, 3, 90, mute[ChlPos-1] ? "unMute" : "Mute", "CHL Editer", "Close");
            if (PerStat == pdTRUE) if (optionKeyEvent.status == KEY_ATTACK) {
                PerStat = pdFALSE;
                switch (optionKeyEvent.num)
                {
                case KEY_UP:
                    ChlMenuPos--;
                    if (ChlMenuPos < 1) ChlMenuPos = 3;
                    break;
                
                case KEY_DOWN:
                    ChlMenuPos++;
                    if (ChlMenuPos > 3) ChlMenuPos = 1;
                    break;

                case KEY_L:
                    ChlPos--;
                    if (ChlPos < 1) ChlPos = 4;
                    break;

                case KEY_R:
                    ChlPos++;
                    if (ChlPos > 4) ChlPos = 1;
                    break;
                }
                if (optionKeyEvent.num == KEY_OK) {
                    if (ChlMenuPos == 3) {
                        ChlMenuPos = 0;
                        windowsClose = true;
                    }
                    if (ChlMenuPos == 1) mute[ChlPos-1] = !mute[ChlPos-1];
                    if (ChlMenuPos == 2) {
                        MenuPos = 1;
                        break;
                    }
                }
            }
        }

        //----------------------MAIN PAGH-------------------------
        //----------------------KEY STATUS------------------------
        if (!ChlMenuPos && !sideMenu) {
            if (PerStat == pdTRUE) if (optionKeyEvent.status == KEY_ATTACK) {
                PerStat = pdFALSE;
                if (optionKeyEvent.num == KEY_OK) {
                    if (ChlPos) ChlMenuPos = 1;
                    else sideMenu = 1;
                }
                switch (optionKeyEvent.num)
                {
                case KEY_UP:
                    if (fnA) inputOct++;
                    else row_point--;
                    break;
                
                case KEY_DOWN:
                    if (fnA) inputOct--;
                    else row_point++;
                    break;

                case KEY_L:
                    ChlPos--;
                    if (ChlPos < 0) ChlPos = 4;
                    break;
                
                case KEY_R:
                    ChlPos++;
                    if (ChlPos > 4) ChlPos = 0;
                }
            }
        }
        frame.display();
    }
}

void ChlEdit() {
    frame.setTextSize(2);
    frame.setCursor(0, 12);
    frame.printf("CHL%d", ChlPos);
    frame.setTextSize(0);
    frame.drawFastVLine(77, 0, 128, 0x4a51);
    frame.drawFastVLine(78, 0, 128, 0x6b7e);
    frame.drawFastVLine(79, 0, 128, 0xad7f);
    int8_t EditPos = 0;
    uint16_t viewTmp[4];
    bool enbRec = false;
    key_event_t optionKeyEvent;
    for (;;) {
        frame.fillRect(0, 40, 77, 88, ST7735_BLACK);
        frame.fillRect(0, 40, vol[ChlPos-1], 8, (((vol[ChlPos-1]>>1) << 11) | (clamp(vol[ChlPos-1], 0, 63) << 5) | (vol[ChlPos-1]>>1)));//0x8578);
        frame.setTextColor(0xef9d);
        frame.setCursor(0, 49);
        frame.printf("VOLE:%d\nPROD:%d\nFREQ:%.1f\n", vol[ChlPos-1], period[ChlPos-1], frq[ChlPos-1]);
        frame.setTextColor(0x8410);
        frame.printf("SAMP INFO\nNAME:\n%s\nNUM: %d\nLEN: %d\nPAT: %d\nVOL: %d", samp_name[smp_num[ChlPos-1]], smp_num[ChlPos-1], wave_info[smp_num[ChlPos-1]][0], wave_info[smp_num[ChlPos-1]][1], wave_info[smp_num[ChlPos-1]][2]);

        vTaskDelay(2);

        // HIGH LIGHT
        if (EditPos == 0) {
            frame.fillRect(80, 0, 13, 128, 0x528a);
            frame.fillRect(80, 55, 13, 9, 0x7bcf);
        } else {
            frame.fillRect(80, 0, 13, 128, 0x2945);
            frame.fillRect(80, 55, 13, 9, 0x528a);
        }
        if (EditPos == 1) {
            frame.fillRect(96, 0, 23, 128, 0x528a);
            frame.fillRect(96, 55, 23, 9, 0x7bcf);
        } else {
            frame.fillRect(96, 0, 23, 128, 0x2945);
            frame.fillRect(96, 55, 23, 9, 0x528a);
        }
        if (EditPos == 2) {
            frame.fillRect(119, 0, 18, 128, 0x528a);
            frame.fillRect(119, 55, 18, 9, 0x7bcf);
        } else {
            frame.fillRect(119, 0, 18, 128, 0x2945);
            frame.fillRect(119, 55, 18, 9, 0x528a);
        }
        if (EditPos == 3) {
            frame.fillRect(137, 0, 9, 128, 0x528a);
            frame.fillRect(137, 55, 9, 9, 0x7bcf);
        } else {
            frame.fillRect(137, 0, 9, 128, 0x2945);
            frame.fillRect(137, 55, 9, 9, 0x528a);
        }
        if (EditPos == 4) {
            frame.fillRect(146, 0, 13, 128, 0x528a);
            frame.fillRect(146, 55, 13, 9, 0x7bcf);
        } else {
            frame.fillRect(146, 0, 13, 128, 0x2945);
            frame.fillRect(146, 55, 13, 9, 0x528a);
        }

        frame.setCursor(81, 0);
        for (int8_t i = -7; i < 9; i++) {
            if ((row_point+i < 64) && (row_point+i >= 0)) {
                frame.setTextColor(0xf7be);
                frame.printf("%2d", row_point+i);
                read_part_data(tracker_data, part_table[part_point], row_point+i, ChlPos-1, viewTmp);
                
                showTmpNote = viewTmp[0];
                showTmpSamp = viewTmp[1];
                showTmpEFX1 = viewTmp[2];
                showTmpEFX2_1 = hexToDecimalTens(viewTmp[3]);
                showTmpEFX2_2 = hexToDecimalOnes(viewTmp[3]);
                
                frame.setCursor(frame.getCursorX()+5, frame.getCursorY());
                if (showTmpNote) {
                    frame.setTextColor(0xcdff);
                    frame.printf("%s ", findNote(showTmpNote));
                } else {
                    frame.setTextColor(0x52aa);
                    frame.printf("... ");
                }
                if (showTmpSamp) {
                    frame.setTextColor(0x97f2);
                    frame.printf("%2d ", showTmpSamp);
                } else {
                    frame.setTextColor(0x52aa);
                    frame.printf(".. ");
                }
                if (showTmpEFX1 || showTmpEFX2_1 || showTmpEFX2_2) {
                    frame.setTextColor(0xfc8f);
                    frame.printf("%1X", showTmpEFX1);
                    frame.setTextColor(0xff14);
                    frame.printf("%1X%1X", showTmpEFX2_1, showTmpEFX2_2);
                } else {
                    frame.setTextColor(0x52aa);
                    frame.printf("...");
                }
                frame.printf("\n");
                frame.setCursor(81, frame.getCursorY());
            } else {
                frame.printf("\n");
                frame.setCursor(81, frame.getCursorY());
            }
        }
        frame.drawFastVLine(93, 0, 128, 0x9492);
        frame.drawFastVLine(94, 0, 128, 0xb5b6);
        frame.drawFastVLine(95, 0, 128, 0xad7f);
        frame.drawFastVLine(159, 0, 128, 0xc618);
        frame.display();
        if (readOptionKeyEvent == pdTRUE) if (optionKeyEvent.status == KEY_ATTACK) {
            if (optionKeyEvent.num == KEY_OK) {
                MenuPos = 0;
                break;
            }
            switch (optionKeyEvent.num)
            {
            case KEY_UP:
                row_point--;
                break;
            
            case KEY_DOWN:
                row_point++;
                break;
            
            case KEY_L:
                EditPos--;
                if (EditPos < 0) EditPos = 4;
                break;
            
            case KEY_R:
                EditPos++;
                if (EditPos > 4) EditPos = 0;
                break;

            case KEY_SPACE:
                playStat = !playStat;
                break;
            }
        }
        vTaskDelay(1);
    }
}

int parseWavHeader(FILE* file, WavHeader_t* header) {

    fread(header, sizeof(WavHeader_t), 1, file);

    // 检查文件格式
    if (header->chunkID[0] != 'R' || header->chunkID[1] != 'I' || 
        header->chunkID[2] != 'F' || header->chunkID[3] != 'F' ||
        header->format[0] != 'W' || header->format[1] != 'A' ||
        header->format[2] != 'V' || header->format[3] != 'E') {
        printf("Error: Not a valid WAV file.\n");
        fclose(file);
        return -1;
    }

    return 0;
}

void wav_player() {
    key_event_t optionKeyEvent;
    playStat = false;
    FILE *wave_file;
    size_t bytes_read;
    view_mode = true;
    vTaskDelay(64);
    buffer = realloc(buffer, 10240);
    buffer16BitStro = (audio16BitStro*)buffer;
    memset(buffer, 0, BUFF_SIZE * sizeof(int16_t));
    const char *wave_file_name = fileSelect("/sdcard");
    wave_file = fopen(wave_file_name, "rb");
    WavHeader_t header;
    parseWavHeader(wave_file, &header);
    printf("WAV File Information:\n");
    printf("Sample Rate: %u Hz\n", header.sampleRate);
    printf("Channels: %u\n", header.numChannels);
    printf("Bits Per Sample: %u\n", header.bitsPerSample);
    size_t writeing;
    i2s_zero_dma_buffer(I2S_NUM_0);
    i2s_set_clk(I2S_NUM_0, header.sampleRate, header.bitsPerSample, (i2s_channel_t)header.numChannels);
    frame.fillScreen(ST7735_BLACK);
    frame.setTextColor(ST7735_WHITE);
    frame.setCursor(0, 0);
    frame.printf("PLAYING...\n\n\n\n\n");
    frame.setTextColor(0x52aa);
    frame.printf("WAV File Information:\nSample Rate: %u Hz\nBits Per Sample: %u\nChannels: %u", header.sampleRate, header.bitsPerSample, header.numChannels);
    frame.setTextColor(ST7735_WHITE);
    frame.display();
    float i2s_vol = 1.0f;
    uint16_t read_p;
    uint8_t refs_p = 0;
    while (!feof(wave_file)) {
        read_p = fread(buffer16BitStro, 1, 10240, wave_file);
        if (refs_p > 1) {
            refs_p = 0;
            frame.fillRect(0, 10, 128, 20, ST7735_BLACK);
            frame.setCursor(0, 10);
            frame.printf("POS(INT8)=%d\nTIME(S)=%f", ftell(wave_file), ftell(wave_file)/(float)(header.sampleRate*4));
            frame.display();
        }
        for (uint16_t s = 0; s < 2560; s++) {
            wave_MAX_L += abs(buffer16BitStro[s].dataL);
            wave_MAX_R += abs(buffer16BitStro[s].dataR);
            buffer16BitStro[s].dataL *= i2s_vol;
            buffer16BitStro[s].dataR *= i2s_vol;
        }
        wave_MAX_L_OUT = wave_MAX_L / 2560;
        wave_MAX_L = 0;
        wave_MAX_R_OUT = wave_MAX_R / 2560;
        wave_MAX_R = 0;
        MAX_COMP_FINISH = true;
        i2s_write(I2S_NUM_0, buffer, 10240, &writeing, portMAX_DELAY);
        printf("%d %d L=%5d R=%5d\n", writeing, read_p, wave_MAX_L_OUT, wave_MAX_R_OUT);
        if (readOptionKeyEvent == pdTRUE) if (optionKeyEvent.status == KEY_ATTACK) {
            switch (optionKeyEvent.num)
            {
            case KEY_L:
                fseek(wave_file, -10240*16, SEEK_CUR);
                break;
            
            case KEY_R:
                fseek(wave_file, 10240*16, SEEK_CUR);
                break;

            case KEY_UP:
                i2s_vol += 0.05f;
                printf("%.2f\n", i2s_vol);
                break;

            case KEY_DOWN:
                i2s_vol -= 0.05f;
                printf("%.2f\n", i2s_vol);
                break;
            }
            if (optionKeyEvent.num == KEY_OK) {
                i2s_vol = 1.0f;
                break;
            }
        }
        refs_p++;
        vTaskDelay(1);
    }
    vTaskDelay(64);
    buffer = realloc(buffer, BUFF_SIZE * sizeof(audio16BitStro));
    buffer16BitStro = (audio16BitStro*)buffer;
    vTaskDelay(64);
    memset(buffer, 0, BUFF_SIZE * sizeof(audio16BitStro));
    fclose(wave_file);
    vTaskDelay(16);
    i2s_zero_dma_buffer(I2S_NUM_0);
    i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_CHAN_16BIT, I2S_CHANNEL_STEREO);
    MenuPos = 0;
    view_mode = false;
    vTaskDelay(64);
}

void filterSetting() {
    MainReDraw();
    int8_t fltrPos = 0;
    frame.drawFastHLine(0, 9, 160, 0xe71c);
    frame.fillRect(0, 10, 160, 118, ST7735_BLACK);
    frame.setCursor(1, 11);
    frame.setTextSize(2);
    frame.fillRect(0, 10, 160, 16, 0x39c7);
    frame.printf("FILTERS\n");
    frame.setTextSize(0);
    frame.drawFastHLine(0, 26, 160, 0xa6bf);
    key_event_t optionKeyEvent;
    for (;;) {
        frame.fillRect(0, 27, 160, 107, ST7735_BLACK);
        frame.setCursor(0, 31);
        frame.setTextSize(2);
        if (fltrPos) frame.fillRect(0, (fltrPos*26)+26, 160, 25, 0x528a);
        else frame.fillRect(0, (fltrPos*26)+27, 160, 24, 0x528a);
        frame.drawFastVLine(48, 27, 101, ST7735_WHITE);
        for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
            frame.printf("CHL%d\n", i+1);
            frame.setCursor(0, frame.getCursorY()+10);
            frame.drawFastHLine(0, frame.getCursorY()-6, 160, ST7735_WHITE);
        }
        frame.setTextSize(0);
        frame.setCursor(52, 31);
        for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
            frame.printf("STATUS: %s\n", enbFltr[i] ? "ON" : "OFF");
            frame.setCursor(52, frame.getCursorY());
            frame.printf("CUTOFF: %.1fHz\n", cutOffFreq[i]);
            frame.setCursor(52, frame.getCursorY()+10);
        }
        frame.display();
        vTaskDelay(4);
        if (readOptionKeyEvent == pdTRUE) if (optionKeyEvent.status == KEY_ATTACK) {
            if (optionKeyEvent.num == KEY_L) {
                fltrPos--;
                if (fltrPos < 0) {MenuPos = 3; break;};
            }
            switch (optionKeyEvent.num)
            {
            case KEY_R:
                fltrPos++;
                if (fltrPos > 3) fltrPos = 0;
                break;
            
            case KEY_UP:
                cutOffFreq[fltrPos]+=500;
                break;

            case KEY_DOWN:
                cutOffFreq[fltrPos]-=500;
                break;
            
            case KEY_OK:
                enbFltr[fltrPos] = !enbFltr[fltrPos];
                break;
            }
        }
    }
}

void Setting() {

    const uint8_t SETTING_NUM = 4;
    key_event_t optionKeyEvent;

    const char *menuStr[SETTING_NUM] = {"Interpolation", "Filter Setting", "WAV Player", "Close"};
    uint8_t optPos = 0;
    frame.drawFastHLine(0, 9, 160, 0xe71c);
    frame.fillRect(0, 10, 160, 118, ST7735_BLACK);
    frame.setCursor(1, 11);
    frame.setTextSize(2);
    frame.fillRect(0, 10, 160, 16, 0x39c7);
    frame.printf("SETTINGS\n");
    frame.setTextSize(0);
    frame.drawFastHLine(0, 26, 160, 0xa6bf);
    for (;;) {
        frame.fillRect(0, 27, 160, 107, ST7735_BLACK);
        frame.setCursor(0, 29);
        frame.fillRect(0, (optPos*10)+27, 160, 11, 0x528a);
        for (uint8_t i = 0; i < SETTING_NUM; i++) {
            frame.printf("%s\n", menuStr[i]);
            frame.setCursor(0, frame.getCursorY()+2);
        }
        frame.display();
        vTaskDelay(2);
        if (readOptionKeyEvent == pdTRUE) if (optionKeyEvent.status == KEY_ATTACK) {
            if (optionKeyEvent.num == KEY_OK) {
                if (optPos == 0) {
                    int8_t menuRtrn = windowsMenuBlocking(menuStr[optPos], 4, enbCubic ? 3 : (enbCos ? 2 : (enbLine ? 1 : (!enbCos && !enbLine && !enbCubic ? 4 : -1))), 90, "Linear", "Cosine", "Cubic Spline", "OFF");
                    switch (menuRtrn)
                    {
                    case 0:
                        enbLine = true;
                        enbCubic = enbCos = false;
                        break;
                    
                    case 1:
                        enbCos = true;
                        enbCubic = enbLine = false;
                        break;

                    case 2:
                        enbCubic = true;
                        enbCos = enbLine = false;
                        break;

                    case 3:
                        enbCubic = enbCos = enbLine = false;
                        break;
                    }
                    printf("MENU RETURN %d\n", menuRtrn);
                }
                if (optPos == 1) {MenuPos = 4; break;}
                if (optPos == 2) wav_player();
                if (optPos == SETTING_NUM - 1) {MenuPos = 0; break;}
            }
            switch (optionKeyEvent.num)
            {
            case KEY_UP:
                optPos--;
                break;
            
            case KEY_DOWN:
                optPos++;
                break;
            }
        }
    }
}

void display_lcd(void *arg) {
    tft.initR(INITR_BLACKTAB);
    tft.setRotation(3);
    tft.fillScreen(ST7735_BLACK);
    MainReDraw();
    lcdOK = false;
    MenuPos = 0;
    read_pattern_table();
    read_wave_info();
    comp_wave_ofst();
    part_point = 0;
    loadOk = true;
    xTaskCreate(&comp, "Play", 9000, NULL, 5, &COMP);
    // xTaskCreatePinnedToCore(&load, "Load", 2048, NULL, 3, &LOAD, 0);
    vTaskDelay(32);
    for (;;) {
        if (MenuPos == 0) {
            MainReDraw();
            windowsClose = true;
            MainPage();
            vTaskDelay(4);
        } else if (MenuPos == 1) {
            MainReDraw();
            ChlEdit();
            vTaskDelay(4);
        } else if (MenuPos == 3) {
            MainReDraw();
            Setting();
            vTaskDelay(4);
        }
        else if (MenuPos == 4) {
            MainReDraw();
            filterSetting();
            vTaskDelay(4);
        }
        vTaskDelay(4);
    }
}
bool TestNote = false;
uint8_t chlMap[CHL_NUM] = {/*L*/0, 3, /*R*/1, 2};
// COMP TASK START ----------------------------------------------
void comp(void *arg) {
    uint8_t tick_time = 0;
    uint8_t tick_speed = 6;
    uint8_t arp_p = 0;
    bool enbArp[4] = {false};
    uint8_t volUp[4] = {0};
    uint8_t volDown[4] = {0};
    uint16_t portToneSpeed[4] = {0};
    uint16_t portToneTemp[4] = {0};
    uint16_t portToneSource[4] = {0};
    uint16_t portToneTarget[4] = {0};
    bool enbPortTone[4] = {false};
    uint16_t lastNote[4] = {false};
    bool enbSlideUp[4] = {false};
    uint8_t SlideUp[4] = {false};
    bool enbSlideDown[4] = {false};
    uint8_t SlideDown[4] = {false};
    bool enbVibrato[4] = {false};
    uint8_t VibratoSpeed[4];
    uint8_t VibratoDepth[4];
    uint8_t VibratoPos[4] = {32};
    bool enbTremolo[4] = {false};
    uint8_t TremoloPos[4] = {32};
    uint8_t TremoloSpeed[4] = {0};
    uint8_t TremoloDepth[4] = {0};
    int VibratoItem[4] = {0};
    uint16_t Mtick = 0;
    uint16_t TICK_NUL = roundf(SMP_RATE / (125 * 0.4f));
    buffer = calloc(BUFF_SIZE, sizeof(audio16BitStro));
    // buffer = realloc(buffer, TICK_NUL * sizeof(audio16BitStro));
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
        buffer_ch[ch] = (float *)calloc(BUFF_SIZE, sizeof(float));
    }
    buffer16BitStro = (audio16BitStro*)buffer;
    uint8_t volTemp[4];
    uint16_t OfstCfg[4];
    uint8_t rowLoopStart = 0;
    int8_t rowLoopCont = 0;
    bool enbRowLoop = false;
    printf("READ!\n");
    vTaskDelay(128);
    uint8_t chl;
    bool enbRetrigger[4] = {false};
    // uint8_t RetriggerPos[4] = {1};
    uint8_t RetriggerConfig[4] = {0};
    int16_t audio_tempL;
    int16_t audio_tempR;
    int8_t atkTick[4] = {0};
    int8_t cutTick[4] = {0};
    int8_t skipToRow = 0;
    uint16_t buffPtr = 0;
    dispRedy = true;
    for(;;) {
        // printf("READ!\n");
        if (playStat) {
            SPD = 6;
            tick_time = 0;
            TICK_NUL = roundf(SMP_RATE / (125 * 0.4f));
            Mtick = 0;
            // buffer = realloc(buffer, TICK_NUL * sizeof(audio16BitStro));
            for (;;) {
                for(chl = 0; chl < 4; chl++) {
                    if (wave_info[smp_num[chl]][4] > 2) {
                        buffer_ch[chl][buffPtr] = make_data(frq[chl], vol[chl], chl, true, wave_info[smp_num[chl]][3]<<1, wave_info[smp_num[chl]][4]<<1, wav_ofst[smp_num[chl]], wave_info[smp_num[chl]][0], enbLine, enbCos, enbCubic);
                    } else {
                        buffer_ch[chl][buffPtr] = make_data(frq[chl], vol[chl], chl, false, 0, 0, wav_ofst[smp_num[chl]], wave_info[smp_num[chl]][0], enbLine, enbCos, enbCubic);
                    }
                }
                buffer16BitStro[buffPtr].dataL = (int16_t)
                        roundf(buffer_ch[chlMap[0]][buffPtr]
                                + buffer_ch[chlMap[1]][buffPtr]);
                buffer16BitStro[buffPtr].dataR = (int16_t)
                        roundf(buffer_ch[chlMap[2]][buffPtr]
                                + buffer_ch[chlMap[3]][buffPtr]);
                Mtick++;
                buffPtr++;
                if (buffPtr >= BUFF_SIZE) {
                    buffPtr = 0;
                    if (recMod) {
                        wav_audio_write(buffer, BUFF_SIZE*sizeof(audio16BitStro), &wrin, export_wav_file);
                        printf("WRIN %d\n", wrin);
                    } else {
                        i2s_write(I2S_NUM_0, buffer, BUFF_SIZE*sizeof(audio16BitStro), &wrin, portMAX_DELAY);
                        // printf("WRIN %d\n", wrin);
                    }
                }
                if (Mtick == TICK_NUL) {
                    // pwm_audio_write((uint8_t*)&buffer, Mtick, &wrin, 64);
                    Mtick = 0;
                    tick_time++;
                    arp_p++;
                    if (tick_time != tick_speed) {
                        for(chl = 0; chl < 4; chl++) {
                            if (enbSlideUp[chl]) {
                                // printf("SLIDE UP %d", period[chl]);
                                period[chl] -= SlideUp[chl];
                                enbPortTone[chl] = false;
                                // printf(" - %d = %d\n", SlideUp[chl], period[chl]);
                            }
                            if (enbSlideDown[chl]) {
                                // printf("SLIDE DOWN %d", period[chl]);
                                period[chl] += SlideDown[chl];
                                enbPortTone[chl] = false;
                                // printf(" + %d = %d\n", SlideDown[chl], period[chl]);
                            }
                            vol[chl] += (volUp[chl] - volDown[chl]);
                            vol[chl] = (vol[chl] > 64) ? 64 : (vol[chl] < 1) ? 0 : vol[chl];

                            VibratoItem[chl] = enbVibrato[chl] ? (sine_table[VibratoPos[chl]] * VibratoDepth[chl]) / 128 : 0;
                            VibratoPos[chl] = enbVibrato[chl] ? (VibratoPos[chl] + VibratoSpeed[chl]) & 63 : 0;

                            if (enbTremolo[chl]) {
                                int tremoloValue = (sine_table[TremoloPos[chl]] * TremoloDepth[chl]) / 128;
                                vol[chl] += tremoloValue;
                                vol[chl] = (vol[chl] > 64) ? 64 : (vol[chl] < 1) ? 0 : vol[chl];
                                // printf("TRE %3d  TREPOS %2d  TRESPD %1d  TREDPH %1d  VOL %3d\n", tremoloValue, TremoloPos[chl], TremoloSpeed[chl], TremoloDepth[chl], vol[chl]);
                                TremoloPos[chl] = (TremoloPos[chl] + TremoloSpeed[chl]) & 63;
                            } else {
                                TremoloPos[chl] = 32;
                            }

                            if (enbPortTone[chl]) {
                                int portToneSpeedValue = portToneSpeed[chl];
                                if (portToneSource[chl] > portToneTarget[chl]) {
                                    period[chl] -= portToneSpeedValue;
                                    period[chl] = (period[chl] < portToneTarget[chl]) ? portToneTarget[chl] : period[chl];
                                    // printf("SIL %d + %d\n", portToneTemp[chl], portToneSpeed[chl]);
                                } else if (portToneSource[chl] < portToneTarget[chl]) {
                                    period[chl] += portToneSpeedValue;
                                    period[chl] = (period[chl] > portToneTarget[chl]) ? portToneTarget[chl] : period[chl];
                                    // printf("SID %d - %d\n", portToneTemp[chl], portToneSpeed[chl]);
                                }
                                // printf("PORTTONE %d to %d. speeed=%d\n", portToneSource[chl], portToneTarget[chl], portToneSpeed[chl]);
                            }
                            if (tick_time == cutTick[chl]) {
                                vol[chl] = 0;
                                cutTick[chl] = 0;
                            }
                        }
                    } else if (tick_time == tick_speed) {
                        tick_time = 0;
                        for (chl = 0; chl < 4; chl++) {
                            read_part_data(tracker_data, part_table[part_point], row_point, chl, part_buffer);
                            if (part_buffer[2] == 13) {
                                if (part_buffer[3]) {
                                    skipToRow = hexToRow(part_buffer[3]);
                                    printf("SKIP TO NEXT PART'S %d ROW\n", skipToRow);
                                }
                                skipToNextPart = true;
                            }

                            if (part_buffer[2] == 11) {
                                if (part_buffer[3]) {
                                    skipToAnyPart = part_buffer[3];
                                }
                            }

                            if (part_buffer[1]) {
                                smp_num[chl] = part_buffer[1];
                                vol[chl] = wave_info[smp_num[chl]][2];
                                lastVol[chl] = vol[chl];
                            }

                            if (part_buffer[0]) {
                                if (part_buffer[2] == 3
                                    || part_buffer[2] == 5) {
                                    enbPortTone[chl] = true;
                                    if (part_buffer[0]) {
                                        portToneTarget[chl] = part_buffer[0];
                                        portToneSource[chl] = lastNote[chl];
                                        lastNote[chl] = part_buffer[0];
                                    }
                                    if (enbSlideDown[chl] || enbSlideUp[chl]) {
                                        portToneSource[chl] = period[chl];
                                    }
                                    // printf("PT TARGET SET TO %d. SOURCE IS %d\n", portToneTarget[chl], portToneSource[chl]);
                                    if ((part_buffer[3])
                                            && (part_buffer[2] != 5)) {
                                        portToneSpeed[chl] = part_buffer[3];
                                    }
                                } else {
                                    data_index[chl] = 0;
                                    lastNote[chl] = part_buffer[0];
                                    period[chl] = lastNote[chl];
                                    vol[chl] = lastVol[chl];
                                    enbPortTone[chl] = false;
                                }
                                if (!(part_buffer[2] == 12) 
                                    && part_buffer[1]) {
                                    vol[chl] = wave_info[smp_num[chl]][2];
                                    lastVol[chl] = vol[chl];
                                }
                                if (part_buffer[1]) {
                                    smp_num[chl] = part_buffer[1];
                                }
                            }

                            if (part_buffer[2] == 10
                                    || part_buffer[2] == 6
                                        || part_buffer[2] == 5) {
                                if (part_buffer[2] == 10) {
                                    enbPortTone[chl] = false;
                                }
                                if (part_buffer[2] == 5) {
                                    enbPortTone[chl] = true;
                                }
                                volUp[chl] = hexToDecimalTens(part_buffer[3]);
                                volDown[chl] = hexToDecimalOnes(part_buffer[3]);
                                // printf("VOL+=%d -=%d\n", volUp[chl], volDown[chl]);
                            } else {
                                volUp[chl] = volDown[chl] = 0;
                            }

                            if (part_buffer[2] == 1) {
                                SlideUp[chl] = part_buffer[3];
                                // printf("SET SLIDEUP IS %d\n", part_buffer[3]);
                                enbSlideUp[chl] = true;
                            } else {
                                enbSlideUp[chl] = false;
                            }

                            if (part_buffer[2] == 2) {
                                SlideDown[chl] = part_buffer[3];
                                // printf("SET SLIDEDOWN IS %d\n", part_buffer[3]);
                                enbSlideDown[chl] = true;
                            } else {
                                enbSlideDown[chl] = false;
                            }

                            if (part_buffer[2] == 4
                                    || part_buffer[2] == 6) {
                                enbVibrato[chl] = true;
                                if ((part_buffer[2] == 4)) {
                                    if (hexToDecimalTens(part_buffer[3])) {
                                        VibratoSpeed[chl] = hexToDecimalTens(part_buffer[3]);
                                    }
                                    if (hexToDecimalOnes(part_buffer[3])) {
                                        VibratoDepth[chl] = hexToDecimalOnes(part_buffer[3]);
                                    }
                                    // printf("VIBRATO SPD %d DPH %d\n", VibratoSpeed[chl], VibratoDepth[chl]);
                                }
                                // VibratoPos[chl] = 0;
                            } else {
                                enbVibrato[chl] = false;
                            }

                            if (part_buffer[2] == 7) {
                                enbTremolo[chl] = true;
                                if (hexToDecimalTens(part_buffer[3])) {
                                    TremoloSpeed[chl] = hexToDecimalTens(part_buffer[3]);
                                }
                                if (hexToDecimalOnes(part_buffer[3])) {
                                    TremoloDepth[chl] = hexToDecimalOnes(part_buffer[3]);
                                }
                                // TremoloPos[chl] = 0;
                                // printf("TREMOLO SPD %d DPH %d\n", VibratoSpeed[chl], VibratoDepth[chl]);
                            } else {
                                enbTremolo[chl] = false;
                            }

                            if (part_buffer[2] == 9) {
                                OfstCfg[chl] = part_buffer[3] << 8;
                                // printf("SMP OFST %d\n", OfstCfg[chl]);
                            }

                            if (part_buffer[2] == 12) {
                                vol[chl] = part_buffer[3];
                                lastVol[chl] = vol[chl];
                                enbPortTone[chl] = false;
                            }
                            enbRetrigger[chl] = false;
                            if (part_buffer[2] == 14) {
                                uint8_t decimalTens = hexToDecimalTens(part_buffer[3]);
                                vol[chl] += (decimalTens == 10) ? hexToDecimalOnes(part_buffer[3]) : ((decimalTens == 11) ? -hexToDecimalOnes(part_buffer[3]) : 0);
                                vol[chl] = (vol[chl] > 64) ? 64 : ((vol[chl] < 1) ? 0 : vol[chl]);
                                period[chl] += (decimalTens == 2) ? hexToDecimalOnes(part_buffer[3]) : ((decimalTens == 1) ? -hexToDecimalOnes(part_buffer[3]) : 0);
                                if (decimalTens == 9) {
                                    enbRetrigger[chl] = true;
                                    RetriggerConfig[chl] = hexToDecimalOnes(part_buffer[3]);
                                    // printf("RETRIGGER %d\n", RetriggerConfig[chl]);
                                    volTemp[chl] = vol[chl];
                                } else if (decimalTens == 6) {
                                    if (hexToDecimalOnes(part_buffer[3]) == 0) {
                                        rowLoopStart = row_point;
                                        // printf("SET LOOP START %d\n", rowLoopStart);
                                    } else {
                                        if (rowLoopCont == 0) {
                                            rowLoopCont = hexToDecimalOnes(part_buffer[3]);
                                            // printf("SET LOOP CONT %d\n", rowLoopCont);
                                        } else {
                                            rowLoopCont--;
                                            // printf("LOOP CONT - 1 %d\n", rowLoopCont);
                                        }
                                        if (rowLoopCont > 0) {
                                            // printf("ROWLOOP %d\n", rowLoopCont);
                                            enbRowLoop = true;
                                        }
                                    }
                                } else if (decimalTens == 12) {
                                    cutTick[chl] = hexToDecimalOnes(part_buffer[3]);
                                }
                                // printf("LINE VOL %s TO %d\n", (decimalTens == 10) ? "UP" : ((decimalTens == 11) ? "DOWN" : "UNCHANGED"), vol[chl]);
                            }

                            if (part_buffer[2] == 15) {
                                if (part_buffer[3] < 32) {
                                    tick_speed = part_buffer[3];
                                    SPD = part_buffer[3];
                                    // printf("SPD SET TO %d\n", tick_speed);
                                } else {
                                    BPM = part_buffer[3];
                                    TICK_NUL = roundf(SMP_RATE / (BPM * 0.4f));
                                    // printf("MTICK SET TO %d\n", TICK_NUL);
                                }
                            }

                            if ((!part_buffer[2]) && part_buffer[3]) {
                                arp_p = 0;

                                arpNote[0][chl] = hexToDecimalTens(part_buffer[3]);
                                arpNote[1][chl] = hexToDecimalOnes(part_buffer[3]);

                                arpFreq[0][chl] = patch_table[wave_info[smp_num[chl]][1]] / period[chl];
                                arpFreq[1][chl] = freq_up(arpFreq[0][chl], arpNote[0][chl]);
                                arpFreq[2][chl] = freq_up(arpFreq[0][chl], arpNote[1][chl]);
                                // printf("ARP CTRL %d %d %f %f %f\n", arpNote[0][chl], arpNote[1][chl], arpFreq[0][chl], arpFreq[1][chl], arpFreq[2][chl]);
                                enbArp[chl] = true;
                            } else {
                                if (enbArp[chl]) {
                                    frq[chl] = arpFreq[0][chl];
                                    enbArp[chl] = false;
                                }
                            }
                            if (vol[chl]) {
                                lastVol[chl] = vol[chl];
                            }
                        }
                        if (partUP) {partUP = false; part_point--; if (part_point < 0) part_point = NUM_PATTERNS-1;}
                        if (partDOWN) {partDOWN = false; part_point++; if (part_point >= NUM_PATTERNS) part_point = 0;}
                        row_point++;
                        if (enbRowLoop) {
                            row_point = rowLoopStart;
                            enbRowLoop = false;
                            printf("SKIP TO %d\n", rowLoopStart);
                        }
                        if ((row_point > 63) || skipToNextPart || skipToAnyPart) {
                            if (recMod) {
                                fflush(export_wav_file);
                            }
                            if (skipToRow) {
                                row_point = skipToRow;
                                skipToRow = 0;
                            } else {
                                row_point = 0;
                            }
                            part_point++;
                            if (part_point >= NUM_PATTERNS) {
                                part_point = 0;
                                if (recMod) {
                                    wav_audio_close(export_wav_file);
                                    recMod = false;
                                }
                            }
                            if (skipToAnyPart) {
                                part_point = skipToAnyPart;
                                printf("SKIP TO %d\n", part_point);
                                skipToAnyPart = false;
                                // part_point++;
                            }
                            skipToNextPart = false;
                            printf("NOW %d\n", part_point);
                        }
                    }
                    for (chl = 0; chl < 4; chl++) {
                        if (period[chl] != 0) {
                            frq[chl] = patch_table[wave_info[smp_num[chl]][1]] / (float)(period[chl] + VibratoItem[chl]);
                        } else {
                            frq[chl] = 0;
                        }
                        if (enbRetrigger[chl]) {
                            // printf("RETPOS %d\n", RetriggerPos[chl]);
                            // if (RetriggerPos[chl] > RetriggerConfig[chl]) {
                            if (!(tick_time % RetriggerConfig[chl])) {
                                // printf("EXEC RET %d\n", RetriggerPos[chl]);
                                data_index[chl] = 0;
                                vol[chl] = volTemp[chl];
                            }
                        }
                        if (OfstCfg[chl]) {
                            data_index[chl] = OfstCfg[chl];
                            OfstCfg[chl] = 0;
                        }
                        if (arp_p > 2) {arp_p = 0;}
                        if (enbArp[chl]) {
                            frq[chl] = arpFreq[arp_p][chl];
                        }
                    }
                    vTaskDelay(1);
                }
                if (!playStat) {
                    for (uint8_t s = 0; s < 4; s++) {
                        vol[s] = 0;
                        period[s] = 0;
                        frq[s] = 0;
                    }
                    i2s_zero_dma_buffer(I2S_NUM_0);
                    vTaskDelay(1);
                    tick_speed = 6;
                    BPM = 125;
                    TICK_NUL = roundf(SMP_RATE / (BPM * 0.4f));
                    data_index[0] = data_index[1] = data_index[2] = data_index[3] = 0;
                    Mtick = 0;
                    if (recMod) {
                        wav_audio_close(export_wav_file);
                        recMod = false;
                        playStat = false;
                    }
                    break;
                }
            }
        } else {
            loadOk = false;
            if (!view_mode) {
                for (uint16_t i = 0; i < BUFF_SIZE; i++) {
                    if (wave_info[smp_num[0]][4] > 2) {
                        buffer_ch[0][i] = buffer_ch[1][i] = buffer_ch[2][i] = buffer_ch[3][i] = make_data(frq[0], vol[0], 0, true, wave_info[smp_num[0]][3]<<1, wave_info[smp_num[0]][4]<<1, wav_ofst[smp_num[0]], wave_info[smp_num[0]][0], false, false, false);
                    } else {
                        buffer_ch[0][i] = buffer_ch[1][i] = buffer_ch[2][i] = buffer_ch[3][i] = make_data(frq[0], vol[0], 0, false, 0, 0, wav_ofst[smp_num[0]], wave_info[smp_num[0]][0], false, false, false);
                    }
                    buffer16BitStro[i].dataL = (int16_t)
                            (buffer_ch[chlMap[0]][i]
                                    + buffer_ch[chlMap[1]][i]);
                    buffer16BitStro[i].dataR = (int16_t)
                            (buffer_ch[chlMap[2]][i]
                                    + buffer_ch[chlMap[3]][i]);
                }
            }
            if (TestNote) {
                TestNote = false;
                if (period[0]) {
                    printf("TEST CHL0 OFF\n");
                    data_index[0] = 0;
                    period[0] = 0;
                    vol[0] = 0;
                } else {
                    printf("TEST CHL0 428\n");
                    data_index[0] = 0;
                    period[0] = 428;
                    vol[0] = 64;
                }
            }
            if (row_point > 63 || partDOWN) {
                if (!partDOWN) {
                    row_point = 0;
                }
                partDOWN = false;
                part_point++;
                if (part_point >= NUM_PATTERNS) {
                    part_point = 0;
                }
            }
            if (row_point < 0 || partUP) {
                if (!partUP) {
                    row_point = 63;
                }
                partUP = false;
                part_point--;
                if (part_point < 0) {
                    part_point = NUM_PATTERNS-1;
                }
            }
            if (playStat) {
                for (uint8_t s = 0; s < 4; s++) {
                    vol[s] = 0;
                    period[s] = 0;
                    frq[s] = 0;
                }
            }
            // pwm_audio_write((uint8_t*)&buffer, BUFF_SIZE, &wrin, 64);
            if (!view_mode) {
                frq[0] = patch_table[wave_info[smp_num[0]][1]] / period[0];
                i2s_write(I2S_NUM_0, buffer, BUFF_SIZE*sizeof(audio16BitStro), &wrin, portMAX_DELAY);
                vTaskDelay(4);
            } else {
                vTaskDelay(32);
            }
        }
    }
}
// COMP TASK END ------------------------------------------------
FILE *f;
/*
void load(void *arg) {
    while(true) {
        if (part_buffer_point == 0 && loadOk) {
            printf("LOADING BUF1\n");
            if (part_point >= NUM_PATTERNS) {
                part_point = 0;
            }
            read_part_data((uint8_t*)tracker_data, part_table[part_point], part_buffer[1]);
            part_point++;
            loadOk = false;
        } else if (part_buffer_point == 1 && loadOk) {
            printf("LOADING BUF0\n");
            if (part_point >= NUM_PATTERNS) {
                part_point = 0;
            }
            read_part_data((uint8_t*)tracker_data, part_table[part_point], part_buffer[0]);
            part_point++;
            loadOk = false;
        }
        vTaskDelay(64);
    }
}
*/

void read_pattern_table() {
    for (uint8_t i = 0; i < 128; i++) {
       part_table[i] = 0;
    }
    for (uint8_t p = 0; p < 20; p++) {
        song_name[p] = tracker_data[p];
    }
    NUM_PATTERNS = tracker_data[950];
    for (uint8_t i = 0; i < NUM_PATTERNS; i++) {
        part_table[i] = tracker_data[952 + i];
        printf("%d ", part_table[i]);
    }
    printf("\n");
}

int find_max(int size) {
    if (size <= 0) {
        return -1;
    }
    int max = part_table[0];
    for (int i = 1; i < size; i++) {
        if (part_table[i] > max) {
            max = part_table[i];
        }
    }
    return max;
}

void read_wave_info() {
    for (uint8_t i = 1; i < 33; i++) {
        uint8_t* sample_data = (uint8_t*)tracker_data + 20 + (i - 1) * 30;
        for (uint8_t p = 0; p < 22; p++) {
            samp_name[i][p] = sample_data[p];
        }
        wave_info[i][0] = ((sample_data[22] << 8) | sample_data[23]) * 2;  // 采样长度
        wave_info[i][1] = sample_data[24];  // 微调值
        wave_info[i][2] = sample_data[25];  // 音量
        wave_info[i][3] = (sample_data[26] << 8) | sample_data[27];  // 重复点
        wave_info[i][4] = (sample_data[28] << 8) | sample_data[29];  // 重复长度
        // ESP_LOGI("WAVE INFO", "NUM=%d LEN=%d PAT=%d VOL=%d LOOPSTART=%d LOOPLEN=%d TRK_MAX=%d", i, wave_info[i][0], wave_info[i][1], wave_info[i][2], wave_info[i][3], wave_info[i][4], find_max(NUM_PATTERNS)+1);
    }
}

void comp_wave_ofst() {
    for (uint8_t i = 0; i < 34; i++) {
        wav_ofst[i] = 0;
    }
    wav_ofst[1] = 1084 + ((find_max(NUM_PATTERNS)+1) * 1024);
    printf("FIND MAX %d OFST %ld", find_max(NUM_PATTERNS), wav_ofst[1]);
/*
    for (uint8_t i = 2; i < 33; i++) {
        printf("%d %d\n", i, wav_ofst[i]);
        wav_ofst[i] += wave_info[i][0];
        wav_ofst[i+1] = wav_ofst[i];
    }
*/
    for (uint8_t i = 1; i < 33; i++) {
        // printf("1 %ld %ld %d\n", wav_ofst[i+1], wav_ofst[i], wave_info[i+1][0]);
        wav_ofst[i+1] += (wav_ofst[i] + wave_info[i][0]);
    }
}
/*
void read_wave_data(uint8_t (*wave_info)[5], uint8_t* tracker_data, uint8_t** wave_data) {
    for (int i = 0; i < 32; i++) {
        uint16_t sample_length = wave_info[i][0] * 2;
        wave_data[i] = (uint8_t*)malloc(sample_length * sizeof(uint8_t));
        if (wave_data[i] == NULL) {
            ESP_LOGE("WAVE READ", "MEMRY MALLOC FAIL!");
            exit(EXIT_FAILURE);
        }
        for (int j = 0; j < sample_length; j++) {
            wave_data[i][j] = tracker_data[20 + i * 30 + j];
        }
    }
}
*/

#define TAG "TAG"
const char* root_path = "/spiffs";

void rotate(ESPRotary& rotary) {
    printf("ROTARY %d\n", (int)rotary.getDirection());
    if ((uint8_t)rotary.getDirection() == 255) {
        //keyUP = true;
    }
    if ((uint8_t)rotary.getDirection() == 1) {
        //keyDOWN = true;
    }
}

void IRAM_ATTR setKeyOK() {
    // keyOK = true;
}

void refesMpr121(void *arg) {
    uint16_t lasttouched = 0;
    uint16_t currtouched = 0;
    Wire1.begin(15, 16);
    touchPad.begin(0x5B, &Wire1);
    key_event_t touchPadEvent;
    for (;;) {
        currtouched = touchPad.touched();
        for (uint8_t i=0; i<12; i++) {
            // it if *is* touched and *wasnt* touched before, alert!
            if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
                touchPadEvent.num = i;
                touchPadEvent.status = KEY_ATTACK;
                if (xQueueSend(xTouchPadQueue, &touchPadEvent, portMAX_DELAY) != pdPASS) {
                    printf("WARNING: TOUCHPAD QUEUE LOSS A EVENT!\n");
                } else {
                    printf("INFO: TOUCHPAD SUCCESSFULLY SENT A EVENT. NUM=%d STATUS=ATTACK", touchPadEvent.num);
                }
            }
            if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
                touchPadEvent.num = i;
                touchPadEvent.status = KEY_RELEASE;
                if (xQueueSend(xTouchPadQueue, &touchPadEvent, portMAX_DELAY) != pdPASS) {
                    printf("WARNING: TOUCHPAD QUEUE LOSS A EVENT!\n");
                } else {
                    printf("INFO: TOUCHPAD SUCCESSFULLY SENT A EVENT. NUM=%d STATUS=RELEASE", touchPadEvent.num);
                }
            }
        }
        lasttouched = currtouched;
        vTaskDelay(32);
    }
}

void input(void *arg) {
    pinMode(21, INPUT_PULLUP);
    rotary.begin(38, 39, 2);
    rotary.setChangedHandler(rotate);
    // rotary.setLeftRotationHandler(showDirection);
    // rotary.setRightRotationHandler(showDirection);
    Serial.begin(115200);
    attachInterrupt(21, setKeyOK, RISING);
    key_event_t optionKeyEvent;
    bool fnA = false;
    bool fnB = false;
    while (true) {
        if (Serial.available() > 0) {
            uint16_t received = Serial.read();
            printf("INPUT: %d\n", received);
            if (received == 32) {
                optionKeyEvent.num = KEY_SPACE;
                optionKeyEvent.status = KEY_ATTACK;
                if (xQueueSend(xOptionKeyQueue, &optionKeyEvent, portMAX_DELAY) != pdPASS) {
                    printf("WARNING: OPTIONKEY QUEUE LOSS A EVENT!\n");
                } else {
                    printf("INFO: OPTIONKEY SUCCESSFULLY SENT A EVENT. NUM=%d STATUS=ATTACK\n", optionKeyEvent.num);
                }
            }
            if (received == 119) {
                optionKeyEvent.num = KEY_UP;
                optionKeyEvent.status = KEY_ATTACK;
                if (xQueueSend(xOptionKeyQueue, &optionKeyEvent, portMAX_DELAY) != pdPASS) {
                    printf("WARNING: OPTIONKEY QUEUE LOSS A EVENT!\n");
                } else {
                    printf("INFO: OPTIONKEY SUCCESSFULLY SENT A EVENT. NUM=%d STATUS=ATTACK\n", optionKeyEvent.num);
                }
            }
            if (received == 115) {
                optionKeyEvent.num = KEY_DOWN;
                optionKeyEvent.status = KEY_ATTACK;
                if (xQueueSend(xOptionKeyQueue, &optionKeyEvent, portMAX_DELAY) != pdPASS) {
                    printf("WARNING: OPTIONKEY QUEUE LOSS A EVENT!\n");
                } else {
                    printf("INFO: OPTIONKEY SUCCESSFULLY SENT A EVENT. NUM=%d STATUS=ATTACK\n", optionKeyEvent.num);
                }
            }
            if (received == 97) {
                optionKeyEvent.num = KEY_L;
                optionKeyEvent.status = KEY_ATTACK;
                if (xQueueSend(xOptionKeyQueue, &optionKeyEvent, portMAX_DELAY) != pdPASS) {
                    printf("WARNING: OPTIONKEY QUEUE LOSS A EVENT!\n");
                } else {
                    printf("INFO: OPTIONKEY SUCCESSFULLY SENT A EVENT. NUM=%d STATUS=ATTACK\n", optionKeyEvent.num);
                }
            }
            if (received == 100) {
                optionKeyEvent.num = KEY_R;
                optionKeyEvent.status = KEY_ATTACK;
                if (xQueueSend(xOptionKeyQueue, &optionKeyEvent, portMAX_DELAY) != pdPASS) {
                    printf("WARNING: OPTIONKEY QUEUE LOSS A EVENT!\n");
                } else {
                    printf("INFO: OPTIONKEY SUCCESSFULLY SENT A EVENT. NUM=%d STATUS=ATTACK\n", optionKeyEvent.num);
                }
            }
            if (received == 108) {
                optionKeyEvent.num = KEY_OK;
                optionKeyEvent.status = KEY_ATTACK;
                if (xQueueSend(xOptionKeyQueue, &optionKeyEvent, portMAX_DELAY) != pdPASS) {
                    printf("WARNING: OPTIONKEY QUEUE LOSS A EVENT!\n");
                } else {
                    printf("INFO: OPTIONKEY SUCCESSFULLY SENT A EVENT. NUM=%d STATUS=ATTACK\n", optionKeyEvent.num);
                }
            }
            if (received == 122) {
                optionKeyEvent.num = KEY_A;
                optionKeyEvent.status = fnA ? KEY_RELEASE : KEY_ATTACK;
                fnA = !fnA;
                if (xQueueSend(xOptionKeyQueue, &optionKeyEvent, portMAX_DELAY) != pdPASS) {
                    printf("WARNING: OPTIONKEY QUEUE LOSS A EVENT!\n");
                } else {
                    printf("INFO: OPTIONKEY SUCCESSFULLY SENT A EVENT. NUM=%d STATUS=%s\n", optionKeyEvent.num, optionKeyEvent.status == KEY_ATTACK ? "ATTACK" : "RELEASE");
                }
            }
            if (received == 49) {
                printf("INPUT SAMP NUM:\n");
                for (;;) {
                    if (Serial.available() > 0) {
                        uint16_t received = Serial.read();
                        smp_num[0] = received - 48;
                        printf("CHL0's SMP_NUM=%d NAME:%s\n", smp_num[0], samp_name[smp_num[0]]);
                        break;
                    }
                    vTaskDelay(8);
                }
            }
            if (received == 50) {
                TestNote = true;
            }
            if (received == 107) {
                vTaskPrioritySet(NULL, 5);
                FileInfo* files = NULL;
                int count = list_directory("/sdcard", &files);
                if (count < 0) {
                    printf("Failed to list directory.\n");
                }

                printf("Total files and directories: %d\n", count);
                for (int i = 0; i < count; i++) {
                    printf("%s - %s\n", files[i].name, files[i].is_directory ? "Directory" : "File");
                    free(files[i].name);
                }
                free(files);
                vTaskPrioritySet(NULL, 2);
            }
            if (received == 109) {
                printf("FREE MEM: %d\n", esp_get_free_heap_size());
            }
        }
        vTaskDelay(2);
        rotary.loop();
    }
}

void setup()
{
    xTouchPadQueue = xQueueCreate(8, sizeof(key_event_t));
    xOptionKeyQueue = xQueueCreate(8, sizeof(key_event_t));
    initDelayBuffer();
    esp_err_t ret;
    xTaskCreatePinnedToCore(&display_lcd, "tracker_ui", 8192, NULL, 5, NULL, 1);
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "ffat",
        .max_files = 2,
        .format_if_mount_failed = true
    };
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 2,
        .allocation_unit_size = 8 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = "/sdcard";
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    slot_config.clk = GPIO_NUM_48;
    slot_config.cmd = GPIO_NUM_47;
    slot_config.d0 = GPIO_NUM_45;
    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret == ESP_OK) {
        printf("SDCARD mounted\n");
        sdmmc_card_print_info(stdout, card);
    } else {
        printf("SDCARD NOT EXIST OR ERROR! SKIP MOUNT!\n");
    }

    printf("esp_err_t ret = esp_vfs_spiffs_register(&conf);\n");
    ret = esp_vfs_spiffs_register(&conf);
    if (ret == ESP_OK) {
        printf("SPIFFS mounted!\n");
    } else {
        printf("SPIFFS ERROR! NOW FORMAT! PLEASE WAIT...\n");
    }
    static const int i2s_num = 0; // i2s port number
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 512,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    static const i2s_pin_config_t pin_config = {
        .bck_io_num = 42,
        .ws_io_num = 40,
        .data_out_num = 41,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    printf("I2S INSTALL %d\n", i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL));
    printf("I2S SETPIN %d\n", i2s_set_pin(I2S_NUM_0, &pin_config));
    i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_CHAN_16BIT, I2S_CHANNEL_STEREO);
    i2s_zero_dma_buffer(I2S_NUM_0);
    new_tracker_file();
    xTaskCreatePinnedToCore(&display, "wave_view", 8192, NULL, 5, NULL, 0);
    vTaskDelay(256);
    xTaskCreatePinnedToCore(&refesMpr121, "MPR121", 4096, NULL, 0, NULL, 0);
    xTaskCreatePinnedToCore(&input, "input", 4096, NULL, 2, NULL, 0);
    printf("MAIN EXIT\n");
}

void loop() {
    printf("DELETE LOOP\n");
    vTaskDelete(NULL);
}