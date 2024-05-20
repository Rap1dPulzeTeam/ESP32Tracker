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
#include "abcd.h"
#include "esp32/clk.h"

#define MOUNT_POINT "/sdcard"

#define TFT_DC 3
#define TFT_CS 10
#define TFT_RST 8
uint8_t tracker_data_header[1084];
uint8_t **tracker_data_pattern = NULL;
int8_t *tracker_data_sample[33];
uint8_t *tracker_data_total;
// uint8_t *tracker_data;
void read_pattern_table(uint8_t head_data[1084]);
void read_samp_info(uint8_t head_data[1084]);
// void comp_wave_ofst();
#define CHL_NUM 4
#define TRACKER_ROW 64
uint8_t NUM_PATTERNS;
#define BUFFER_PATTERNS 2
#define NUM_ROWS 64
#define NUM_CHANNELS 4
#define PATTERN_SIZE (NUM_ROWS * NUM_CHANNELS * 4)
#define BUFF_SIZE 1024

bool recMod = false;
FILE *export_wav_file;
size_t export_wav_written = 0;
void *export_wav_buffer;
uint8_t inputOct = 2;
bool editMod = false;
uint16_t limitStats;
bool fromOtherPage = false;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Adafruit_MPR121 touchPad = Adafruit_MPR121();
ESPRotary rotary;
bool RtyL = false;
bool RtyR = false;
bool RtyB = false;
static uint8_t pat_max = 0;

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
#define KEY_C 9
#define LCD_BK 9

class aFrameBuffer : public Adafruit_GFX {
  public:
    uint16_t lcd_buffer[40960];
    aFrameBuffer(int16_t w, int16_t h): Adafruit_GFX(w, h) {
        printf("DISPLAY READLY!\n");
    }
    IRAM_ATTR void drawPixel(int16_t x, int16_t y, uint16_t color)
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

    void display() {tft.drawRGBBitmap(0, 0, lcd_buffer, 160, 128);}
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
int8_t vol[CHL_NUM] = {0, 0, 0, 0};

size_t wrin;
uint8_t stp;
static bool display_stat = true;

uint8_t part_table[128];
int8_t part_point = 0;
int8_t row_point = 0;
char song_name[20];

#define BASE_FREQ 8267
bool dispRedy = false;
bool playStat = false;
uint32_t wave_MAX_L = 0;
uint32_t wave_MAX_R = 0;
uint32_t wave_MAX_L_OUT = 0;
uint32_t wave_MAX_R_OUT = 0;
bool MAX_COMP_FINISH = false;

bool enbFltr[4] = {false, false, false, false};
bool enbDelay[4] = {true, true, true, true};

float cutOffFreq[4] = {SMP_RATE/2, SMP_RATE/2, SMP_RATE/2, SMP_RATE/2};

const float patch_table[16] = {3546836.0f, 3572537.835745873f, 3598425.9175884672f, 3624501.595143772f, 3650766.227807657f, 3677221.184826728f, 3703867.8453697185f, 3730707.5985993897f,
                         3347767.391694687f, 3372026.6942439806f, 3396461.7896998064f, 3421073.9519300302f, 3445864.464033491f, 3470834.61840689f, 3495985.7168121682f, 3521319.0704443706f};

#define hexToDecimalTens(num) (((num) >> 4) & 0x0F)
#define hexToDecimalOnes(num) ((num) & 0x0F)
#define hexToRow(num) (hexToDecimalTens(num) * 10 + hexToDecimalOnes(num))
#define freq_up(base_freq, n) ((base_freq) * powf(2.0f, ((n) / 12.0f)))

#define drawMidRect(w, h, color) \
    frame.drawRect(80 - ((w) >> 1), 64 - ((h) >> 1), (80 + ((w) >> 1)) - (80 - ((w) >> 1)), (64 + ((h) >> 1)) - (64 - ((h) >> 1)), (color))

#define drawMidRectOfst(w, h, color, ofstX, ofstY) \
    frame.drawRect(80 - ((w) >> 1) + (ofstX), 64 - ((h) >> 1) + (ofstY), \
                   (w), (h), (color))

#define fillMidRect(w, h, color) \
    frame.fillRect(80 - ((w) >> 1), 64 - ((h) >> 1), (80 + ((w) >> 1)) - (80 - ((w) >> 1)), (64 + ((h) >> 1)) - (64 - ((h) >> 1)), (color))

#define fillMidRectOfst(w, h, color, ofstX, ofstY) \
    frame.fillRect(80 - ((w) >> 1) + (ofstX), 64 - ((h) >> 1) + (ofstY), \
                   (w), (h), (color))

#define setMidCusr(w, h, ofst) \
    frame.setCursor(80 - ((w) >> 1) + (ofst), 64 - ((h) >> 1) + (ofst))

#define setMidCusrOfst(w, h, ofstX, ofstY) \
    frame.setCursor(80 - ((w) >> 1) + (ofstX), 64 - ((h) >> 1) + (ofstY))

inline int clamp(int value, int min, int max) {
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}

class Animation {
public:
    float startX, startY;
    float endX, endY;
    int16_t currentX, currentY;
    uint16_t step;
    uint16_t maxSteps;

    void initAnimation(float x1, float y1, float x2, float y2, uint16_t maxStep) {
        startX = x1;
        startY = y1;
        endX = x2;
        endY = y2;
        currentX = startX;
        currentY = startY;
        step = 0;
        maxSteps = maxStep;
    }

    inline int16_t getAnimationX() {
        return currentX;
    }

    inline int16_t getAnimationY() {
        return currentY;
    }

    void nextAnimation(float k) {
        if (step < maxSteps) {
            float t = (float)step / (float)maxSteps;
            float scale = 1.0f - expf(-k * t);  // 使用指数衰减函数
            
            currentX = roundf(startX * (1 - scale) + endX * scale);
            currentY = roundf(startY * (1 - scale) + endY * scale);

            step++;
        }
    }
};

void backlightCtrl(void *arg) {
    printf("BACKLIGHT START\n");
    // analogWriteFrequency(22050);
    for (uint16_t i = 0; i < 256; i++) {
        analogWrite(LCD_BK, i);
        vTaskDelay(2);
        // printf("BK %d\n", i);
    }
    printf("BACKLIGHT END\n");
    vTaskDelete(NULL);
}
void backlightCtrlFast(void *arg) {
    printf("BACKLIGHT FAST START\n");
    // analogWriteFrequency(22050);
    for (uint16_t i = 0; i < 256; i++) {
        analogWrite(LCD_BK, i);
        vTaskDelay(1);
        // printf("BK %d\n", i);
    }
    printf("BACKLIGHT FAST END\n");
    vTaskDelete(NULL);
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

typedef struct {
    char name[22];
    uint16_t len;
    uint8_t finetune;
    uint8_t vol;
    uint16_t loopStart;
    uint16_t loopLen;
} samp_info_t;

samp_info_t samp_info[33];
uint32_t wav_ofst[34];

bool mute[4] = {false, false, false, false};

#define DELAY_BUFFER_SIZE 4096

float delayBuffer[6][4096];
int delayWriteIndex[6] = {0, 0, 0, 0, 0, 0};

void initDelayBuffer() {
    for (int i = 0; i < DELAY_BUFFER_SIZE; ++i) {
        delayBuffer[0][i] = 0.0f;
        delayBuffer[1][i] = 0.0f;
        delayBuffer[2][i] = 0.0f;
        delayBuffer[3][i] = 0.0f;
        delayBuffer[4][i] = 0.0f;
        delayBuffer[5][i] = 0.0f;
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

int8_t read_tracker_file(const char* path) {
    // Free previously allocated memory for patterns
    for (uint8_t i = 0; i < pat_max; i++) {
        if (tracker_data_pattern && tracker_data_pattern[i]) {
            printf("FREE PAT #%d ADRS %p\n", i, tracker_data_pattern[i]);
            free(tracker_data_pattern[i]);
            tracker_data_pattern[i] = NULL;
        }
    }
    if (tracker_data_pattern) {
        printf("PAT ADRS %p\n", tracker_data_pattern);
        free(tracker_data_pattern);
        tracker_data_pattern = NULL;
    }

    // Free previously allocated memory for samples
    for (uint8_t i = 0; i < 33; i++) {
        if (tracker_data_sample[i]) {
            printf("FREE SMP #%d ADRS %p\n", i, tracker_data_sample[i]);
            free(tracker_data_sample[i]);
            tracker_data_sample[i] = NULL;
        }
    }

    printf("NOW READ FILE %s\n", path);
    FILE *file = fopen(path, "rb");
    if (!file) {
        printf("Failed to open file %s\n", path);
        return -1;
    }

    size_t readRelly = fread(tracker_data_header, 1, 1084, file);
    printf("READ HEADER FINISH\n");
    if (readRelly < 1084) {
        printf("READ %s ERROR! FILE TOO SMALL! FILE SIZE IS %zu\n", path, readRelly);
        fclose(file);
        return -1;
    }

    if ((tracker_data_header[1080] != 0x4D) ||
        (tracker_data_header[1081] != 0x2E) ||
        (tracker_data_header[1082] != 0x4B) ||
        (tracker_data_header[1083] != 0x2E)) {
        printf("READ %s ERROR! NOT A M.K. MOD FILE! HEAD=%c%c%c%c\n", path, 
            tracker_data_header[1080], tracker_data_header[1081], 
            tracker_data_header[1082], tracker_data_header[1083]);
        fclose(file);
        return -1;
    }

    read_pattern_table(tracker_data_header);
    read_samp_info(tracker_data_header);
    pat_max = find_max(NUM_PATTERNS) + 1;
    tracker_data_pattern = (uint8_t**)malloc(pat_max * sizeof(uint8_t*));
    if (!tracker_data_pattern) {
        printf("UNKNOWN MALLOC FAIL...\n");
        fclose(file);
        return -2;
    }

    printf("HEAD NOW FILE POS %ld\n", ftell(file));
    for (uint8_t i = 0; i < pat_max; i++) {
        tracker_data_pattern[i] = (uint8_t*)malloc(1024);
        if (!tracker_data_pattern[i]) {
            printf("READ PATTERN MALLOC FAIL! EXITING...\n");
            for (uint8_t j = 0; j < i; j++) {
                free(tracker_data_pattern[j]);
            }
            free(tracker_data_pattern);
            fclose(file);
            return -2;
        }
        fread(tracker_data_pattern[i], 1, 1024, file);
    }

    printf("PAT NOW FILE POS %ld\n", ftell(file));
    for (uint8_t i = 1; i < 33; i++) {
        if (samp_info[i].len) {
            tracker_data_sample[i] = (int8_t*)malloc(samp_info[i].len << 1);
            if (!tracker_data_sample[i]) {
                printf("READ SAMPLE MALLOC FAIL! EXITING...\n");
                for (uint8_t j = 0; j < pat_max; j++) {
                    free(tracker_data_pattern[j]);
                }
                free(tracker_data_pattern);
                for (uint8_t j = 0; j <= i; j++) {
                    free(tracker_data_sample[j + 1]);
                }
                fclose(file);
                return -2;
            }
            fread(tracker_data_sample[i], 2, samp_info[i].len, file);
            printf("READ SMP #%d SIZE: %d STATUS %p\n", i + 1, samp_info[i].len<<1, tracker_data_sample[i]);
        }
    }

    printf("FINISH NOW FILE POS %ld\n", ftell(file));
    printf("READ %s FINISH!\n", path);
    fclose(file);
    return 0;
}

bool new_tracker_file() {

    for (uint8_t i = 0; i < pat_max; i++) {
        if (tracker_data_pattern && tracker_data_pattern[i]) {
            printf("FREE PAT #%d ADRS %p\n", i, tracker_data_pattern[i]);
            free(tracker_data_pattern[i]);
            tracker_data_pattern[i] = NULL;
        }
    }
    if (tracker_data_pattern) {
        printf("PAT ADRS %p\n", tracker_data_pattern);
        free(tracker_data_pattern);
        tracker_data_pattern = NULL;
    }

    // Free previously allocated memory for samples
    for (uint8_t i = 0; i < 33; i++) {
        if (tracker_data_sample[i]) {
            printf("FREE SMP #%d ADRS %p\n", i, tracker_data_sample[i]);
            free(tracker_data_sample[i]);
            tracker_data_sample[i] = NULL;
        }
    }

    if (tracker_data_header == NULL) {
        printf("CREATE NEW FILE ON MEM FAILED!\n");
        return 1;
    }
    for (uint16_t p = 0; p < 1084; p++) {
        tracker_data_header[p] = tracker_null[p];
    }
    read_pattern_table(tracker_data_header);
    read_samp_info(tracker_data_header);
    pat_max = find_max(NUM_PATTERNS)+1;
    tracker_data_pattern = (uint8_t**)malloc(pat_max*sizeof(uint8_t*));
    for (uint8_t i = 0; i < pat_max; i++) {
        tracker_data_pattern[i] = (uint8_t*)malloc(1024);
        if (tracker_data_pattern[i] == NULL) {printf("READ PATTERN MALLOC FAIL! EXITNG...\n"); exit(-1);}
        for (uint16_t j = 0; j < 1024; j++) {
            tracker_data_pattern[i][j] = 0;
        }
    }
    for (uint8_t i = 0; i < 32; i++) {
        //if (wave_info[i][0]) fread(tracker_data_sample[i], 1, wave_info[i+1][0], file);
    }
    printf("CREATE NEW FILE ON MEM FINISH!\n");
    return 0;
}

void comp(void *arg);
void load(void *arg);
// **************************INIT*END****************************

int16_t period[4] = {0, 0, 0, 0};
float frq[4] = {0, 0, 0, 0};
float data_index[CHL_NUM] = {0, 0, 0, 0};
bool view_mode = false;
uint8_t smp_num[CHL_NUM] = {0, 0, 0, 0};
float global_vol = 0.5f;

int16_t temp;

bool master_delay = false;
bool master_limit = true;
float stroMix = 0.45f;

// Limit
typedef struct {
    int16_t threshold;
    int16_t soft_knee;
} Limiter;

void initLimiter(Limiter *limiter, int16_t threshold, float soft_knee) {
    limiter->threshold = threshold;
    limiter->soft_knee = soft_knee;
}

static inline int16_t audioLimit(Limiter *limiter, int16_t inputSample, uint16_t *limitOutComp) {
    int16_t absInputSample = inputSample > 0 ? inputSample : -inputSample;

    if (absInputSample > limiter->threshold) {
        *limitOutComp = absInputSample - limiter->threshold;
        float normalizedExcess = (float)(absInputSample - limiter->threshold) / limiter->soft_knee;
        float compressedExcess = tanhf(normalizedExcess) * limiter->soft_knee;
        int16_t compressedSample = limiter->threshold + (int16_t)compressedExcess;
        return inputSample > 0 ? compressedSample : -compressedSample;
    } else {
        *limitOutComp = 0;
        return inputSample;
    }
}

Limiter limiter;

int8_t audio_master_write(const void *src, size_t size, size_t len) {
    for (uint16_t i = 0; i < len; i++) {
        if (size == 4) {
            buffer16BitStro[i].dataL *= global_vol;
            buffer16BitStro[i].dataR *= global_vol;
        } else {
            return -1;
        }
        if (master_limit) {
            buffer16BitStro[i].dataL = audioLimit(&limiter, buffer16BitStro[i].dataL, &limitStats);
            buffer16BitStro[i].dataR = audioLimit(&limiter, buffer16BitStro[i].dataR, &limitStats);
        }
        if (master_delay) {
            buffer16BitStro[i].dataL = audioDelay(buffer16BitStro[i].dataL, 4096, 0.4f, 0.8f, 0.2f, 4);
            buffer16BitStro[i].dataR = audioDelay(buffer16BitStro[i].dataR, 4096, 0.4f, 0.8f, 0.2f, 5);
        }
        int16_t inputL = buffer16BitStro[i].dataL;
        int16_t inputR = buffer16BitStro[i].dataR;
        buffer16BitStro[i].dataL = (int16_t)((1.0f - stroMix) * inputL + stroMix * inputR);
        buffer16BitStro[i].dataR = (int16_t)((1.0f - stroMix) * inputR + stroMix * inputL);
        wave_MAX_L += abs(buffer16BitStro[i].dataL);
        wave_MAX_R += abs(buffer16BitStro[i].dataR);
    }
    wave_MAX_L_OUT = wave_MAX_L / 2560;
    wave_MAX_L = 0;
    wave_MAX_R_OUT = wave_MAX_R / 2560;
    wave_MAX_R = 0;
    MAX_COMP_FINISH = true;
    i2s_write(I2S_NUM_0, src, len*size, &wrin, portMAX_DELAY);
    // printf("L=%5d R=%5d\n", wave_MAX_L_OUT, wave_MAX_R_OUT);
    return 0;
}

// OSC START -----------------------------------------------------
void display(void *arg) {
    char ten[24];
    char one[24];
    SSD1306_t dev;
    i2c_master_init(&dev, 1, 2, -1);
    ssd1306_init(&dev, 128, 64);
    ssd1306_clear_screen(&dev, false);
    ssd1306_contrast(&dev, 0xff);
    ssd1306_display_text(&dev, 0, (char *)"Welcome to", 11, false);
    ssd1306_display_text(&dev, 1, (char *)"ESP32Tracker", 13, false);
    ssd1306_display_text(&dev, 2, (char *)"BOOTING....", 12, false);
    vTaskDelay(32);
    for (;;) {
        vTaskDelay(32);
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
                sprintf(ten, limitStats ? "LIMIT: %5dOVER" : "LIMIT: NO ACTIVE", limitStats);
                ssd1306_clear_buffer(&dev);
                ssd1306_display_text(&dev, 0, "VIEW MODE", 10, false);
                ssd1306_display_text(&dev, 1, ten, 16, false);
                for (uint8_t i = 0; i < 16; i++) {
                    _ssd1306_line(&dev, 0, i+20, wave_MAX_L_OUT>>8, i+20, false);
                }
                for (uint8_t i = 0; i < 16; i++) {
                    _ssd1306_line(&dev, 0, i+38, wave_MAX_R_OUT>>8, i+38, false);
                }
                ssd1306_show_buffer(&dev);
            } else {
                vTaskDelay(1);
            }
        } else {
            for (uint8_t contr = 0; contr < 4; contr++) {
                ssd1306_clear_buffer(&dev);
                sprintf(ten, " %2d %2d>%2d %d", row_point, part_point, part_table[part_point], limitStats ? limitStats : (uint8_t)(stroMix*100));
                // sprintf(ten, "%d   %d   %d   %d   %d   %d\n", channelActive[0], channelActive[1], channelActive[2], channelActive[3], channelActive[4], channelActive[5]);
                // sprintf(one, "%3d %3d %3d %3d %3d %3d\n", sigBase[0], sigBase[1], sigBase[2], sigBase[3], sigBase[4], sigBase[5]);
                addr[0] = (uint8_t)(data_index[0] * (16.0f / samp_info[smp_num[0]].len)) & 31;
                addr[1] = (uint8_t)(data_index[1] * (16.0f / samp_info[smp_num[1]].len)) & 31;
                addr[2] = (uint8_t)(data_index[2] * (16.0f / samp_info[smp_num[2]].len)) & 31;
                addr[3] = (uint8_t)(data_index[3] * (16.0f / samp_info[smp_num[3]].len)) & 31;
                // printf("%d %d %d %d\n", addr[0], addr[1], addr[2], addr[3]);
                ssd1306_display_text(&dev, 0, "CH1 CH2 CH3 CH4", 16, false);
                ssd1306_display_text(&dev, 6, ten, 16, false);
                // ssd1306_display_text(&dev, 5, one, 16, false);
                if (!mute[0]) {
                for (x = 0; x < 32; x++) {
                    _ssd1306_line(&dev, x, 32, x, (uint8_t)(((buffer_ch[0][(x + (contr * 128)) * 2]) / 256) + 32)&63, false);
                    if (period[0]) {
                        _ssd1306_pixel(&dev, x, (uint8_t)(period[0] * (64.0f / 743.0f))&63, false);
                    }
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

inline void read_part_data(uint8_t** tracker_data, uint8_t pattern_index, uint8_t row_index, uint8_t channel_index, uint16_t part_data[4]) {
    uint8_t* pattern_data = tracker_data[pattern_index];

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

void write_part_data(uint8_t **tracker_data, uint8_t pattern_index, uint8_t chl, uint8_t row, uint16_t row_data[4]) {
    uint8_t* pattern_data = tracker_data[pattern_index];

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
inline float make_data(float freq, uint8_t vole, uint8_t chl, bool isLoop, uint16_t loopStart, uint16_t loopLen, uint8_t smp_num, uint16_t smp_size, bool isline, bool isCos, bool isCubic) {
    if (vole == 0 || freq <= 0 || mute[chl] || tracker_data_sample[smp_num] == NULL) {
        if (enbDelay[chl]) return audioDelay(0, 2048, 0.23f, 0.8f, 0.2f, chl);
        else return 0;
    }

    data_index[chl] += freq / SMP_RATE;

    if (isLoop) {
        while (data_index[chl] >= (loopStart + loopLen)) {
            data_index[chl] -= loopLen;
        }
    } else if (data_index[chl] >= smp_size) {
        vol[chl] = 0;
        return 0;
    }

    int idx = (int)data_index[chl];
    int16_t sample1 = (tracker_data_sample[smp_num][idx] * (vole << 1));
    float result;

    if ((isline && isCos) || (isline && isCubic) || (isCos && isCubic)) {
        isCubic = false;
        isline = false;
        isCos = false;
    }

    if (isline) {
        int nextIdx = idx + 1;
        if (nextIdx >= smp_size) {
            nextIdx = isLoop ? loopStart : 0;
        }
        int16_t sample2 = (tracker_data_sample[smp_num][nextIdx] * (vole << 1));
        float frac = data_index[chl] - idx;
        result = (1.0f - frac) * sample1 + frac * sample2;
    } else if (isCos) {
        int nextIdx = idx + 1;
        if (nextIdx >= smp_size) {
            nextIdx = isLoop ? loopStart : 0;
        }
        int16_t sample2 = (tracker_data_sample[smp_num][nextIdx] * (vole << 1));
        float frac = data_index[chl] - idx;
        float f_t = (1 - cosf(M_PI_F * frac)) / 2;
        result = (1.0f - f_t) * sample1 + f_t * sample2;
    } else if (isCubic) {
        int prevIdx = idx - 1;
        int nextIdx1 = idx + 1;
        int nextIdx2 = idx + 2;
        if (prevIdx < 0) {
            prevIdx = isLoop ? loopStart + loopLen - 1 : 0;
        }
        if (nextIdx1 >= smp_size) {
            nextIdx1 = isLoop ? loopStart : 0;
        }
        if (nextIdx2 >= smp_size) {
            nextIdx2 = isLoop ? loopStart + (nextIdx2 - smp_size) : 0;
        }
        int16_t y0 = (tracker_data_sample[smp_num][prevIdx] * (vole << 1));
        int16_t y1 = sample1;
        int16_t y2 = (tracker_data_sample[smp_num][nextIdx1] * (vole << 1));
        int16_t y3 = (tracker_data_sample[smp_num][nextIdx2] * (vole << 1));
        float frac = data_index[chl] - idx;
        result = y1 + 0.5f * frac * (y2 - y0 + frac * (2.0f * y0 - 5.0f * y1 + 4.0f * y2 - y3 + frac * (3.0f * (y1 - y2) + y3 - y0)));
    } else {
        result = sample1;
    }

    if (enbFltr[chl]) {
        result = lowPassFilterSingleSample(result, chl, cutOffFreq[chl], SMP_RATE);
    }
    if (enbDelay[chl]) {
        result = audioDelay(result, 2048, 0.23f, 0.8f, 0.2f, chl);
    }

    return result;
}

// AUDIO DATA COMP END ------------------------------------------

// float frq[CHL_NUM] = {0};

uint16_t part_buffer[4];

bool skipToNextPart = false;
uint8_t skipToAnyPart = false;
bool lcdOK = false;
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
int8_t ChlMenuPos_last = 0;
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
    const uint16_t frequency;
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
    int low, high, mid;
    for (int i = 0; i < NUM_RANGES; ++i) {
        low = 0;
        high = NOTES_PER_RANGE - 1;
        while (low <= high) {
            mid = low + (high - low) / 2; // 计算中间位置
            if (period_table[i][mid].frequency == frequency) {
                return period_table[i][mid].note_name; // 找到音符，返回音符名
            } else if (period_table[i][mid].frequency < frequency) {
                high = mid - 1; // 查找左半部分
            } else {
                low = mid + 1; // 查找右半部分
            }
        }
    }
    return "???"; // 没有找到对应的音符
}

int main() {
    const char *note = findNote(808); // 测试查找频率为808的音符
    printf("Note: %s\n", note);
    return 0;
}

char* fileSelect(const char* root_path) {
    playStat = false;
    char path[256];
    uint8_t path_depth = 0;
    int16_t SelPos = 0;
    int16_t SelPos_last = 0;
    uint8_t pg = 0;
    FileInfo* files = NULL;
    frame.drawFastHLine(0, 9, 160, 0xe71c);
    sprintf(path, "%s", root_path);
    printf("%s\n", path);
    int count = list_directory(path, &files);
    bool emyDir = false;
    key_event_t optionKeyEvent;

    bool CurChange = true;
    uint8_t AnimStep = 0;

    Animation AnimMenu = Animation();

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

        if (CurChange) {
            if (!AnimStep) {
                AnimMenu.initAnimation(0, ((SelPos_last)*10)+28, 0, ((SelPos%10)*10)+28, 16);
                printf("INIT! STARTX=%.1f STARTY=%.1f ENDX=%.1f ENDY=%.1f\n", AnimMenu.startX, AnimMenu.startY, AnimMenu.endX, AnimMenu.endY);
            }
            // printf("STARTX=%.1f STARTY=%.1f ENDX=%.1f ENDY=%.1f X=%d Y=%d\n", startX, startY, endX, endY, getAnimationX(), getAnimationY());
            frame.fillRect(AnimMenu.getAnimationX(), AnimMenu.getAnimationY(), 160, 11, 0x528a);
            AnimMenu.nextAnimation(10);
            AnimStep++;
            if (AnimStep >= 16) {
                CurChange = false;
                AnimStep = 0;
            }
        } else {
            frame.fillRect(0, ((SelPos%10)*10)+28, 160, 11, 0x528a);
        }

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
                SelPos_last = SelPos % 10;
                SelPos--;
                if (SelPos < 0) {
                    SelPos = count-1;
                }
                CurChange = true;
                AnimStep = 0;
                break;
            case KEY_DOWN:
                SelPos_last = SelPos % 10;
                SelPos++;
                if (SelPos > count-1) {
                    SelPos = 0;
                }
                CurChange = true;
                AnimStep = 0;
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
    uint16_t *snap = (uint16_t*)malloc(160*128*sizeof(uint16_t));
    memcpy(snap, frame.lcd_buffer, 160*128*sizeof(uint16_t));
    Animation Anim = Animation();
    Anim.initAnimation(0, -80, 0, 0, 32);
    for (uint8_t i = 0; i < 26; i++) {
        frame.drawRGBBitmap(0, 0, snap, 160, 128);
        fillMidRectOfst(80, 20, 0x4208, Anim.getAnimationX(), Anim.getAnimationY());
        drawMidRectOfst(80, 20, ST7735_WHITE, Anim.getAnimationX(), Anim.getAnimationY());
        setMidCusrOfst(80, 20, Anim.getAnimationX()+7, Anim.getAnimationY()+7);
        // printf("GET ANIM(INT) %d %d %d\n", i, 148, 20);
        frame.setTextColor(0x7bcf);
        frame.printf("READING...");
        setMidCusrOfst(80, 20, Anim.getAnimationX()+6, Anim.getAnimationY()+6);
        frame.setTextColor(ST7735_WHITE);
        frame.printf("READING...");
        frame.display();
        Anim.nextAnimation(8);
    }
    free(snap);
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
        char *fileName = fileSelect("/sdcard");
        int8_t ret = read_tracker_file(fileName);
        free(fileName);
        uint16_t *snap = (uint16_t*)malloc(160*128*sizeof(uint16_t));
        memcpy(snap, frame.lcd_buffer, 160*128*sizeof(uint16_t));
        Animation Anim0 = Animation();
        Animation Anim1 = Animation();
        if (ret == -1) {
            Anim0.initAnimation(0, -80, 0, 0, 32);
            Anim1.initAnimation(0, 0, 0, -80, 32);
            for (uint8_t i = 0; i < 24; i++) {
                frame.drawRGBBitmap(0, 0, snap, 160, 128);
                fillMidRectOfst(148, 20, 0x4208, Anim0.getAnimationX(), Anim0.getAnimationY());
                drawMidRectOfst(148, 20, ST7735_WHITE, Anim0.getAnimationX(), Anim0.getAnimationY());
                setMidCusrOfst(148, 20, Anim0.getAnimationX()+7, Anim0.getAnimationY()+7);
                // printf("GET ANIM(INT) %d %d %d\n", i, 148, 20);
                frame.setTextColor(0x7bcf);
                frame.printf("THIS IS NOT A MOD FILE!");
                setMidCusrOfst(148, 20, Anim0.getAnimationX()+6, Anim0.getAnimationY()+6);
                frame.setTextColor(ST7735_WHITE);
                frame.printf("THIS IS NOT A MOD FILE!");
                frame.display();
                // printf("%d ANIM %d\n", i, Anim0.getAnimationY());
                Anim0.nextAnimation(8);
            }
            while(readOptionKeyEvent != pdTRUE) {
                vTaskDelay(4);
            }
            for (uint8_t i = 0; i < 24; i++) {
                frame.drawRGBBitmap(0, 0, snap, 160, 128);
                fillMidRectOfst(148, 20, 0x4208, Anim1.getAnimationX(), Anim1.getAnimationY());
                drawMidRectOfst(148, 20, ST7735_WHITE, Anim1.getAnimationX(), Anim1.getAnimationY());
                setMidCusrOfst(148, 20, Anim1.getAnimationX()+7, Anim1.getAnimationY()+7);
                // printf("GET ANIM(INT) %d %d %d\n", i, 148, 20);
                frame.setTextColor(0x7bcf);
                frame.printf("THIS IS NOT A MOD FILE!");
                setMidCusrOfst(148, 20, Anim1.getAnimationX()+6, Anim1.getAnimationY()+6);
                frame.setTextColor(ST7735_WHITE);
                frame.printf("THIS IS NOT A MOD FILE!");
                frame.display();
                // printf("%d ANIM %d\n", i, Anim1.getAnimationY());
                Anim1.nextAnimation(8);
            }
            MainReDraw();
        } else if (ret == -2) {
            Anim0.initAnimation(0, -80, 0, 0, 32);
            Anim1.initAnimation(0, 0, 0, -80, 32);
            for (uint8_t i = 0; i < 24; i++) {
                frame.drawRGBBitmap(0, 0, snap, 160, 128);
                fillMidRectOfst(148, 20, 0x4208, Anim0.getAnimationX(), Anim0.getAnimationY());
                drawMidRectOfst(148, 20, ST7735_WHITE, Anim0.getAnimationX(), Anim0.getAnimationY());
                setMidCusrOfst(148, 20, Anim0.getAnimationX()+7, Anim0.getAnimationY()+7);
                // printf("GET ANIM(INT) %d %d %d\n", i, 148, 20);
                frame.setTextColor(0x7bcf);
                frame.printf("THIS FILE IS TOO LARGE!");
                setMidCusrOfst(148, 20, Anim0.getAnimationX()+6, Anim0.getAnimationY()+6);
                frame.setTextColor(ST7735_WHITE);
                frame.printf("THIS FILE IS TOO LARGE!");
                frame.display();
                printf("%d ANIM %d\n", i, Anim0.getAnimationY());
                Anim0.nextAnimation(8);
            }
            while(readOptionKeyEvent != pdTRUE) {
                vTaskDelay(4);
            }
            for (uint8_t i = 0; i < 24; i++) {
                frame.drawRGBBitmap(0, 0, snap, 160, 128);
                fillMidRectOfst(148, 20, 0x4208, Anim1.getAnimationX(), Anim1.getAnimationY());
                drawMidRectOfst(148, 20, ST7735_WHITE, Anim1.getAnimationX(), Anim1.getAnimationY());
                setMidCusrOfst(148, 20, Anim1.getAnimationX()+7, Anim1.getAnimationY()+7);
                // printf("GET ANIM(INT) %d %d %d\n", i, 148, 20);
                frame.setTextColor(0x7bcf);
                frame.printf("THIS FILE IS TOO LARGE!");
                setMidCusrOfst(148, 20, Anim1.getAnimationX()+6, Anim1.getAnimationY()+6);
                frame.setTextColor(ST7735_WHITE);
                frame.printf("THIS FILE IS TOO LARGE!");
                frame.display();
                printf("%d ANIM %d\n", i, Anim1.getAnimationY());
                Anim1.nextAnimation(8);
            }
            MainReDraw();
        } else {
            Animation Anim = Animation();
            Anim.initAnimation(0, 0, 0, 128, 32);
            for (uint8_t i = 0; i < 30; i++) {
                frame.fillScreen(ST7735_BLACK);
                frame.drawRGBBitmap(0, Anim.getAnimationY(), snap, 160, 128);
                // printf("%d ANIM %d\n", i, Anim.getAnimationY());
                Anim.nextAnimation(9);
                frame.display();
                analogWrite(LCD_BK, 255-(i<<3));
            }
            xTaskCreatePinnedToCore(backlightCtrlFast, "BACKLIGHT++", 2048, NULL, 1, NULL, 0);
            free(snap);
            fromOtherPage = true;
            break;
        }
        free(snap);
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
    read_pattern_table(tracker_data_header);
    read_samp_info(tracker_data_header);
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
    read_pattern_table(tracker_data_header);
    read_samp_info(tracker_data_header);
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

void windowsMenu(const char *title, uint8_t current_option, uint8_t current_last, uint8_t total_options, uint8_t Xlen, Animation& menuAnim, bool CurChange, uint8_t animStep, ...) {
    va_list args;
    va_start(args, Xlen);
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

    if (CurChange) {
        if (!animStep) {
            menuAnim.initAnimation(frame.getCursorX()-2, (frame.getCursorY()-2)+((current_last-1)*11),
                frame.getCursorX()-2, (frame.getCursorY()-2)+((current_option-1)*11), 16);
            printf("INIT! STARTX=%.1f STARTY=%.1f ENDX=%.1f ENDY=%.1f\n", menuAnim.startX, menuAnim.startY, menuAnim.endX, menuAnim.endY);
        }
        // printf("STARTX=%.1f STARTY=%.1f ENDX=%.1f ENDY=%.1f X=%d Y=%d\n", menuAnim.startX, menuAnim.startY, menuAnim.endX, menuAnim.endY, menuAnim.getAnimationX(), menuAnim.getAnimationY());
        frame.fillRect(menuAnim.getAnimationX(), menuAnim.getAnimationY(), Xlen - 10, 11, 0x7bef);
        menuAnim.nextAnimation(12);
    } else {
        frame.fillRect(frame.getCursorX()-2, (frame.getCursorY()-2)+((current_option-1)*11), Xlen - 10, 11, 0x7bef);
    }

    for (uint8_t q = 0; q < total_options; q++) {
        frame.printf("%s\n", va_arg(args, const char *));
        frame.setCursor(CXTmp, frame.getCursorY()+3);
    }

    frame.setCursor(OriginX, OriginY);
}

int8_t windowsMenuBlocking(const char *title, uint8_t total_options, uint8_t opt_init_val, uint8_t Xlen, ...) {
    key_event_t optionKeyEvent;
    va_list args;
    va_start(args, Xlen);
    uint8_t OriginX = frame.getCursorX();
    uint8_t OriginY = frame.getCursorY();
    uint8_t Ylen = ((total_options+1)*11)+18;
    int8_t current_option = opt_init_val;
    int8_t current_last = opt_init_val;
    char * menuStr[total_options];
    for (uint8_t i = 0; i < total_options; i++) {
        menuStr[i] = (char *)va_arg(args, const char *);
    }
    bool CurChange = false;
    uint8_t AnimStep = 0;
    Animation AnimMenu = Animation();
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
        if (CurChange) {
            if (!AnimStep) {
                AnimMenu.initAnimation(frame.getCursorX()-2, (frame.getCursorY()-2)+((current_last-1)*11),
                    frame.getCursorX()-2, (frame.getCursorY()-2)+((current_option-1)*11), 16);
                printf("INIT! STARTX=%.1f STARTY=%.1f ENDX=%.1f ENDY=%.1f\n", AnimMenu.startX, AnimMenu.startY, AnimMenu.endX, AnimMenu.endY);
            }
            // printf("STARTX=%.1f STARTY=%.1f ENDX=%.1f ENDY=%.1f X=%d Y=%d\n", startX, startY, endX, endY, getAnimationX(), getAnimationY());
            frame.fillRect(AnimMenu.getAnimationX(), AnimMenu.getAnimationY(), Xlen - 10, 11, 0x7bef);
            AnimMenu.nextAnimation(10);
            AnimStep++;
            if (AnimStep > 14) {
                CurChange = false;
                AnimStep = 0;
            }
        } else {
            frame.fillRect(frame.getCursorX()-2, (frame.getCursorY()-2)+((current_option-1)*11), Xlen - 10, 11, 0x7bef);
        }
        for (uint8_t q = 0; q < total_options; q++) {
            frame.printf("%s\n", menuStr[q]);
            frame.setCursor(CXTmp, frame.getCursorY()+3);
        }
        frame.printf("Close");
        if (readOptionKeyEvent == pdTRUE) { if (optionKeyEvent.status == KEY_ATTACK) {
            switch (optionKeyEvent.num)
            {
            case KEY_UP:
                current_last = current_option;
                current_option--;
                if (current_option < 1) current_option = total_options+1;
                CurChange = true;
                AnimStep = 0;
                break;
            case KEY_DOWN:
                current_last = current_option;
                current_option++;
                if (current_option > total_options+1) current_option = 1;
                CurChange = true;
                AnimStep = 0;
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
    if (fromOtherPage) {fromOtherPage = false;xTaskCreatePinnedToCore(backlightCtrlFast, "BACKLIGHT++", 2048, NULL, 1, NULL, 0);}
    frame.drawFastHLine(0, 9, 160, 0xe71c);
    frame.drawFastHLine(0, 18, 160, 0xe71c);
    frame.fillRect(113, 19, 47, 7, 0xa514);
    frame.drawFastVLine(112, 19, 24, ST7735_WHITE);
    frame.fillRect(0, 10, 160, 8, 0x2104);
    uint8_t sideMenu = 0;
    uint8_t fileMenu = 0;
    uint8_t fileMenu_last = 0;
    uint16_t viewTmp[4];
    key_event_t optionKeyEvent;
    key_event_t touchPadEvent;
    BaseType_t PerStat = pdFALSE;
    bool fnA = false;
    bool fnB = false;
    bool fnC = false;
    // uint8_t test = 0;
    Animation AnimMenu = Animation();
    uint8_t animStep = 0;
    bool CurChange = false;
    for (;;) {
        if (RtyL) {RtyL = false;stroMix += 0.01;printf("MIX %f\n", stroMix);}
        if (RtyR) {RtyR = false;stroMix -= 0.01;printf("MIX %f\n", stroMix);}
        PerStat = readOptionKeyEvent;
        frame.setCursor(0, 10);
        frame.print(song_name);

        if (PerStat == pdTRUE) {
            if (optionKeyEvent.num == KEY_A) {
                if (optionKeyEvent.status == KEY_ATTACK) fnA = true;
                else fnA = false;
            } else
            if (optionKeyEvent.num == KEY_C) 
                if (optionKeyEvent.status == KEY_ATTACK) editMod = !editMod;
        }

        // printf("FnA STAT %d\n", fnA);

        if (PerStat == pdTRUE && optionKeyEvent.num == KEY_SPACE) {
            PerStat = pdFALSE;
            if (!playStat) row_point = 0;
            playStat = !playStat;
        }

        if (!ChlMenuPos && !sideMenu) {
            if (readTouchPadKeyEvent == pdTRUE) if (editMod && touchPadEvent.status == KEY_ATTACK) {
                uint16_t tmpData[4];
                read_part_data(tracker_data_pattern, part_table[part_point], ChlPos-1, row_point, tmpData);
                tmpData[0] = midi_period_table[inputOct][touchPadEvent.num];
                tmpData[1] = smp_num[ChlPos-1];
                write_part_data(tracker_data_pattern, part_table[part_point], ChlPos-1, row_point, tmpData);
            }
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

        frame.drawRect(0, 87, 160, 9, editMod ? 0xfc51 : 0x529f);

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
                frame.printf(row_point+i < 10 ? "0%d" : "%d", row_point+i);
                for (uint8_t chl = 0; chl < 4; chl++) {
                    read_part_data(tracker_data_pattern, part_table[part_point], row_point+i, chl, viewTmp);
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
        read_part_data(tracker_data_pattern, part_table[part_point], row_point, ChlPos-1, viewTmp);
        frame.setTextColor(ST7735_WHITE);

        frame.fillRect(0, 19, 112, 21, 0x3186);
        frame.setCursor(1, 20);
        frame.printf("BPM: %3d  FCT: %2d", BPM, inputOct+1);

        frame.setCursor(1, 28);
        frame.printf("SPD: %2d   SMP: %2d", SPD, ChlPos > 0 ? smp_num[ChlPos-1] : 0);

        if (sideMenu) {
            for (uint8_t i = 0; i < 3; i++) {
                if (sideMenu-1 == i) {
                    frame.fillRect(1+(i*37), 41, 36, 13, 0x630c);
                } else {
                    frame.fillRect(1+(i*37), 41, 36, 13, 0x2945);
                }
            }

            if (fileMenu) {
                windowsMenu("FILE", fileMenu, fileMenu_last, 5, 80, AnimMenu, CurChange, animStep, "New", "Open", recMod ? "Recording..." : "Record", "Setting", "Close");
                animStep++;
                if (animStep > 14) {
                    CurChange = false;
                    animStep = 0;
                }
                if (PerStat == pdTRUE) if (optionKeyEvent.status == KEY_ATTACK) {
                    PerStat = pdFALSE;
                    switch (optionKeyEvent.num)
                    {
                    case KEY_UP:
                        fileMenu_last = fileMenu;
                        fileMenu--;
                        if (fileMenu < 1) fileMenu = 5;
                        CurChange = true;
                        animStep = 0;
                        break;
                    
                    case KEY_DOWN:
                        fileMenu_last = fileMenu;
                        fileMenu++;
                        if (fileMenu > 5) fileMenu = 1;
                        CurChange = true;
                        animStep = 0;
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
                        if (fileMenu == 3 && !recMod) {
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
                        fileMenu_last = 0;
                        CurChange = false;
                        animStep = 0;
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
                        MenuPos = 5;
                        break;
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
            windowsMenu(menuShow, ChlMenuPos, ChlMenuPos_last, 3, 90, AnimMenu, CurChange, animStep, mute[ChlPos-1] ? "unMute" : "Mute", "CHL Editer", "Close");
            animStep++;
            if (animStep > 14) {
                CurChange = false;
                animStep = 0;
            }
            if (PerStat == pdTRUE) if (optionKeyEvent.status == KEY_ATTACK) {
                PerStat = pdFALSE;
                switch (optionKeyEvent.num)
                {
                case KEY_UP:
                    ChlMenuPos_last = ChlMenuPos;
                    ChlMenuPos--;
                    if (ChlMenuPos < 1) ChlMenuPos = 3;
                    CurChange = true;
                    animStep = 0;
                    break;
                
                case KEY_DOWN:
                    ChlMenuPos_last = ChlMenuPos;
                    ChlMenuPos++;
                    if (ChlMenuPos > 3) ChlMenuPos = 1;
                    CurChange = true;
                    animStep = 0;
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
                        ChlMenuPos_last = 0;
                        CurChange = false;
                        animStep = 0;
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
        frame.printf("SAMP INFO\nNAME:\n%s\nNUM: %d\nLEN: %d\nFTV: %d\nVOL: %d", samp_info[smp_num[ChlPos-1]].name, smp_num[ChlPos-1], samp_info[smp_num[ChlPos-1]].len<<1, samp_info[smp_num[ChlPos-1]].finetune, samp_info[smp_num[ChlPos-1]].vol);

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
                frame.printf(row_point+i < 10 ? "0%d" : "%d", row_point+i);
                read_part_data(tracker_data_pattern, part_table[part_point], row_point+i, ChlPos-1, viewTmp);
                
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
    char *wave_file_name = fileSelect("/sdcard");
    wave_file = fopen(wave_file_name, "rb");
    free(wave_file_name);
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
            buffer16BitStro[s].dataL *= i2s_vol;
            buffer16BitStro[s].dataR *= i2s_vol;
        }
        /*
        wave_MAX_L_OUT = wave_MAX_L / 2560;
        wave_MAX_L = 0;
        wave_MAX_R_OUT = wave_MAX_R / 2560;
        wave_MAX_R = 0;
        MAX_COMP_FINISH = true;
        */
        // i2s_write(I2S_NUM_0, buffer, 10240, &writeing, portMAX_DELAY);
        if (audio_master_write(buffer, sizeof(audio16BitStro), 2560) != 0) exit(-1);
        // printf("%d %d L=%5d R=%5d\n", writeing, read_p, wave_MAX_L_OUT, wave_MAX_R_OUT);
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
        if (RtyL) {RtyL = false;stroMix += 0.01;printf("MIX %f\n", stroMix);}
        if (RtyR) {RtyR = false;stroMix -= 0.01;printf("MIX %f\n", stroMix);}
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

void SampEdit() {
    const uint8_t OPTION_NUM = 6;
    const char *menuStr[OPTION_NUM] = {"New", "Load", "Info", "Next", "Prev", "Close"};
    frame.drawFastHLine(0, 9, 160, 0xe71c);
    frame.drawFastHLine(0, 18, 160, 0xe71c);
    frame.fillRect(0, 10, 160, 8, 0x42d0);
    frame.display();
    bool confMenu = false;
    uint8_t optPos = 0;
    uint8_t optPos_last = 0;
    bool CurChange = true;
    uint8_t AnimStep = 0;
    key_event_t optionKeyEvent;
    Animation AnimMenu = Animation();
    frame.setCursor(0, 10);
    frame.printf("Sample Editer");
    uint8_t show_num = 1;
    bool refresh_data = true;
    int8_t snap[117] = {0};
    uint8_t showLoopStart = 0;
    uint8_t showLoopEnd = 0;
    for (;;) {
        frame.fillRect(0, 19, 160, 20, ST7735_BLACK);
        frame.fillRect(0, 39, 42, 89, ST7735_BLACK);
        frame.drawFastVLine(42, 28, 141, 0xf79e);
        frame.drawFastHLine(0, 28, 160, 0x867f);
        frame.drawFastHLine(42, 63, 118, 0xf79e);
        frame.fillRect(0, 19, 160, 9, 0x2104);

        if (refresh_data) {
            refresh_data = false;

            if (tracker_data_sample[show_num] != NULL) {
                float showCount = samp_info[show_num].len / 58.5f;

                if (samp_info[show_num].len > 255) {
                    int16_t showFilter = 0;
                    for (uint8_t i = 0; i < 117; i++) {
                        showFilter += tracker_data_sample[show_num][(uint16_t)(i*showCount)-1]>>2;
                        showFilter += tracker_data_sample[show_num][(uint16_t)(i*showCount)]>>2;
                        showFilter += tracker_data_sample[show_num][(uint16_t)(i*showCount)+1]>>2;
                        snap[i] = showFilter / 3;
                        showFilter = 0;
                    }
                } else {
                    for (uint8_t i = 0; i < 117; i++) {
                        snap[i] = tracker_data_sample[show_num][(uint16_t)(i*showCount)]>>2;
                    }
                }
            } else {
                memset(snap, 0, 117);
            }
            if (samp_info[show_num].loopLen > 1) {
                showLoopStart = (uint8_t)((samp_info[show_num].loopStart<<1) * (58.5f / samp_info[show_num].len));
                showLoopEnd = (uint8_t)(((samp_info[show_num].loopStart<<1) + (samp_info[show_num].loopLen<<1)) * (58.5f / samp_info[show_num].len));
                showLoopEnd = showLoopEnd > 116 ? 116 : showLoopEnd;
            } else {
                showLoopStart = showLoopEnd = 0;
            }
        }


        frame.fillRect(43, 64, 117, 64, 0x4208);

        if (samp_info[show_num].len > 255) {
            for (uint8_t x = 0; x < 117; x++) {
                frame.drawFastVLine(43+x, 97, snap[x], 0xffdf);
            }
        } else {
            for (uint8_t x = 0; x < 117; x++) {
                frame.drawFastVLine(43+x, 97, snap[x], 0xffdf);
            }
        }

        if (showLoopEnd) {
            frame.drawFastVLine(43+showLoopStart, 64, 64, 0xfae8);
            frame.drawFastVLine(43+showLoopEnd, 64, 64, 0x429f);
        }

        if (tracker_data_sample[show_num] != NULL) {
            for (uint8_t chl = 0; chl < NUM_CHANNELS; chl++) {
                if (smp_num[chl] == show_num) frame.drawFastVLine(43+(data_index[chl] * (58.5f / samp_info[smp_num[chl]].len)), 64, 64, 0xc618);
            }
        } else {
            frame.setFont(NULL);
            frame.setCursor(80, 82);
            frame.setTextSize(2);
            frame.setTextColor(0);
            frame.print("NULL");
            frame.setTextSize(1);
            frame.setFont(&abcd);
            frame.setTextColor(ST7735_WHITE);
        }

        frame.setCursor(40, 30);
        frame.printf("VOL %d  FTV %d", samp_info[show_num].vol, samp_info[show_num].vol, samp_info[show_num].finetune);

        frame.setCursor(0, 20);
        frame.printf(show_num < 10 ? "0%d: %.16s" : "%d: %.16s", show_num, samp_info[show_num].name);

        frame.setCursor(0, 31);
        if (CurChange) {
            if (!AnimStep) {
                AnimMenu.initAnimation(0, (optPos_last*10)+29, 0, (optPos*10)+29, 16);
                printf("INIT! STARTX=%.1f STARTY=%.1f ENDX=%.1f ENDY=%.1f\n", AnimMenu.startX, AnimMenu.startY, AnimMenu.endX, AnimMenu.endY);
            }
            // printf("STARTX=%.1f STARTY=%.1f ENDX=%.1f ENDY=%.1f X=%d Y=%d\n", startX, startY, endX, endY, getAnimationX(), getAnimationY());
            frame.fillRect(AnimMenu.getAnimationX(), AnimMenu.getAnimationY(), 42, 11, 0x528a);
            AnimMenu.nextAnimation(9);
            AnimStep++;
            if (AnimStep > 14) {
                CurChange = false;
                AnimStep = 0;
            }
        } else {
            frame.fillRect(0, (optPos*10)+29, 42, 11, 0x528a);
        }
        for (uint8_t i = 0; i < OPTION_NUM; i++) {
            frame.printf("%s\n", menuStr[i]);
            frame.setCursor(0, frame.getCursorY()+2);
        }
        frame.display();
        vTaskDelay(2);
        if (readOptionKeyEvent == pdTRUE) if (optionKeyEvent.status == KEY_ATTACK) {
            if (optionKeyEvent.num == KEY_OK) {
                if (optPos == 3) {show_num++; if (show_num > 31) show_num = 1;}
                if (optPos == 4) {show_num--; if (show_num < 1) show_num = 31;}
                if (optPos == 5) {
                    MenuPos = 0;
                    break;
                }
                refresh_data = true;
            }
            switch (optionKeyEvent.num)
            {
            case KEY_UP:
                optPos_last = optPos;
                optPos--;
                CurChange = true;
                AnimStep = 0;
                break;
            
            case KEY_DOWN:
                optPos_last = optPos;
                optPos++;
                CurChange = true;
                AnimStep = 0;
                break;
            }
        }
    }
    fromOtherPage = true;
}

void Setting() {
    bool CurChange = true;
    uint8_t AnimStep = 0;

    const uint8_t SETTING_NUM = 4;
    key_event_t optionKeyEvent;

    const char *menuStr[SETTING_NUM] = {"Interpolation", "Filter Setting", "WAV Player", "Close"};
    uint8_t optPos = 0;
    uint8_t optPos_last = 0;
    frame.drawFastHLine(0, 9, 160, 0xe71c);
    frame.fillRect(0, 10, 160, 118, ST7735_BLACK);
    frame.setCursor(1, 11);
    frame.setTextSize(2);
    frame.fillRect(0, 10, 160, 16, 0x39c7);
    frame.printf("SETTINGS\n");
    frame.setTextSize(0);
    frame.drawFastHLine(0, 26, 160, 0xa6bf);
    Animation AnimMenu = Animation();
    for (;;) {
        frame.fillRect(0, 27, 160, 107, ST7735_BLACK);
        frame.setCursor(0, 29);
        if (CurChange) {
            if (!AnimStep) {
                AnimMenu.initAnimation(0, (optPos_last*10)+27, 0, (optPos*10)+27, 16);
                printf("INIT! STARTX=%.1f STARTY=%.1f ENDX=%.1f ENDY=%.1f\n", AnimMenu.startX, AnimMenu.startY, AnimMenu.endX, AnimMenu.endY);
            }
            // printf("STARTX=%.1f STARTY=%.1f ENDX=%.1f ENDY=%.1f X=%d Y=%d\n", startX, startY, endX, endY, getAnimationX(), getAnimationY());
            frame.fillRect(AnimMenu.getAnimationX(), AnimMenu.getAnimationY(), 160, 11, 0x528a);
            AnimMenu.nextAnimation(10);
            AnimStep++;
            if (AnimStep > 14) {
                CurChange = false;
                AnimStep = 0;
            }
        } else {
            frame.fillRect(0, (optPos*10)+27, 160, 11, 0x528a);
        }
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
                optPos_last = optPos;
                optPos--;
                CurChange = true;
                AnimStep = 0;
                break;
            
            case KEY_DOWN:
                optPos_last = optPos;
                optPos++;
                CurChange = true;
                AnimStep = 0;
                break;
            }
        }
    }
    fromOtherPage = true;
}

void display_lcd(void *arg) {
    xTaskCreatePinnedToCore(backlightCtrl, "BACKLIGHT++", 2048, NULL, 1, NULL, 0);
    MainReDraw();
    MenuPos = 0;
    read_pattern_table(tracker_data_header);
    read_samp_info(tracker_data_header);
    part_point = 0;
    xTaskCreate(&comp, "Play", 9000, NULL, 5, &COMP);
    vTaskDelay(16);
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
        else if (MenuPos == 5) {
            MainReDraw();
            SampEdit();
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
                    if (samp_info[smp_num[chl]].loopLen > 1) {
                        buffer_ch[chl][buffPtr] = make_data(frq[chl], vol[chl], chl, true, samp_info[smp_num[chl]].loopStart<<1, samp_info[smp_num[chl]].loopLen<<1, smp_num[chl], samp_info[smp_num[chl]].len<<1, enbLine, enbCos, enbCubic);
                    } else {
                        buffer_ch[chl][buffPtr] = make_data(frq[chl], vol[chl], chl, false, 0, 0, smp_num[chl], samp_info[smp_num[chl]].len<<1, enbLine, enbCos, enbCubic);
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
                        // i2s_write(I2S_NUM_0, buffer, BUFF_SIZE*sizeof(audio16BitStro), &wrin, portMAX_DELAY);
                        if (audio_master_write(buffer, sizeof(audio16BitStro), BUFF_SIZE) != 0) exit(-1);
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
                            read_part_data(tracker_data_pattern, part_table[part_point], row_point, chl, part_buffer);
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
                                vol[chl] = samp_info[smp_num[chl]].vol;
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
                                    vol[chl] = samp_info[smp_num[chl]].vol;
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

                                arpFreq[0][chl] = patch_table[samp_info[smp_num[chl]].finetune] / period[chl];
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
                            frq[chl] = patch_table[samp_info[smp_num[chl]].finetune] / (float)(period[chl] + VibratoItem[chl]);
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
            if (!view_mode) {
                for (uint16_t i = 0; i < BUFF_SIZE; i++) {
                    if (samp_info[smp_num[0]].loopLen > 1) {
                        buffer_ch[0][i] = buffer_ch[1][i] = buffer_ch[2][i] = buffer_ch[3][i] = make_data(frq[0], vol[0], 0, true, samp_info[smp_num[0]].loopStart<<1, samp_info[smp_num[0]].loopLen<<1, wav_ofst[smp_num[0]], samp_info[smp_num[0]].len<<1, false, false, false);
                    } else {
                        buffer_ch[0][i] = buffer_ch[1][i] = buffer_ch[2][i] = buffer_ch[3][i] = make_data(frq[0], vol[0], 0, false, 0, 0, wav_ofst[smp_num[0]], samp_info[smp_num[0]].len<<1, false, false, false);
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
                frq[0] = patch_table[samp_info[smp_num[0]].finetune] / period[0];
                // i2s_write(I2S_NUM_0, buffer, BUFF_SIZE*sizeof(audio16BitStro), &wrin, portMAX_DELAY);
                if (audio_master_write(buffer, sizeof(audio16BitStro), BUFF_SIZE) != 0) exit(-1);
                vTaskDelay(4);
            } else {
                vTaskDelay(32);
            }
        }
    }
}
// COMP TASK END ------------------------------------------------
FILE *f;

void read_pattern_table(uint8_t head_data[1084]) {
    for (uint8_t i = 0; i < 128; i++) {
       part_table[i] = 0;
    }
    for (uint8_t p = 0; p < 20; p++) {
        song_name[p] = head_data[p];
    }
    NUM_PATTERNS = head_data[950];
    for (uint8_t i = 0; i < NUM_PATTERNS; i++) {
        part_table[i] = head_data[952 + i];
        printf("%d ", part_table[i]);
    }
    printf("\n");
}

void read_samp_info(uint8_t head_data[1084]) {
    for (uint8_t i = 1; i < 33; i++) {
        uint8_t* sample_data = (uint8_t*)head_data + 20 + (i - 1) * 30;
        memcpy(samp_info[i].name, sample_data, 22);
        
        samp_info[i].len = (sample_data[22] << 8) | sample_data[23];  // 采样长度
        samp_info[i].finetune = sample_data[24];  // 微调值
        samp_info[i].vol = sample_data[25];  // 音量
        samp_info[i].loopStart = (sample_data[26] << 8) | sample_data[27];  // 重复点
        samp_info[i].loopLen = (sample_data[28] << 8) | sample_data[29];  // 重复长度
        // ESP_LOGI("WAVE INFO", "NUM=%d LEN=%d PAT=%d VOL=%d LOOPSTART=%d LOOPLEN=%d TRK_MAX=%d", i, wave_info[i][0], wave_info[i][1], wave_info[i][2], wave_info[i][3], wave_info[i][4], find_max(NUM_PATTERNS)+1);
    }
}

void write_samp_info(uint8_t head_data[1084]) {
    for (uint8_t i = 1; i < 33; i++) {
        uint8_t* sample_data = head_data + 20 + (i - 1) * 30;
        memcpy(sample_data, &samp_info[i], sizeof(samp_info_t));
    }
}

#define TAG "TAG"
const char* root_path = "/spiffs";

void rotate(ESPRotary& rotary) {
    printf("ROTARY %d\n", (int)rotary.getDirection());
    if ((uint8_t)rotary.getDirection() == 255) {
        RtyL = true;
    }
    if ((uint8_t)rotary.getDirection() == 1) {
        RtyR = true;
    }
}

void IRAM_ATTR setKeyOK() {
    RtyB = true;
}

void refesMpr121(void *arg) {
    uint16_t lasttouched = 0;
    uint16_t currtouched = 0;
    Wire1.begin(15, 16);
    touchPad.begin(0x5B, &Wire1);
    key_event_t touchPadEvent;
    printf("MPR121 READY!\n");
    for (;;) {
        currtouched = touchPad.touched();
        for (uint8_t i=0; i<12; i++) {
            // it if *is* touched and *wasnt* touched before, alert!
            if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
                touchPadEvent.num = i;
                touchPadEvent.status = KEY_ATTACK;
                if (xQueueSend(xTouchPadQueue, &touchPadEvent, portMAX_DELAY) != pdPASS) {
                    printf("WARNING: TOUCHPAD QUEUE LOSS A EVENT!\n");
                }// else {
                //    printf("INFO: TOUCHPAD SUCCESSFULLY SENT A EVENT. NUM=%d STATUS=ATTACK\n", touchPadEvent.num);
                //}
            }
            if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
                touchPadEvent.num = i;
                touchPadEvent.status = KEY_RELEASE;
                if (xQueueSend(xTouchPadQueue, &touchPadEvent, portMAX_DELAY) != pdPASS) {
                    printf("WARNING: TOUCHPAD QUEUE LOSS A EVENT!\n");
                }// else {
                //    printf("INFO: TOUCHPAD SUCCESSFULLY SENT A EVENT. NUM=%d STATUS=RELEASE\n", touchPadEvent.num);
                //}
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
    bool fnC = false;
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
            if (received == 99) {
                optionKeyEvent.num = KEY_C;
                optionKeyEvent.status = fnC ? KEY_RELEASE : KEY_ATTACK;
                fnC = !fnC;
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
                        printf("CHL0's SMP_NUM=%d NAME:%s\n", smp_num[0], samp_info[smp_num[0]].name);
                        break;
                    }
                    vTaskDelay(8);
                }
            }
            if (received == 50) {
                TestNote = true;
            }
            if (received == 102) {
                printf("Current CPU frequency: %u Hz\n", esp_clk_cpu_freq());
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
    pinMode(LCD_BK, OUTPUT);
    analogWriteFrequency(22050);
    analogWrite(LCD_BK, 0);
    xTaskCreatePinnedToCore(&display, "wave_view", 8192, NULL, 5, NULL, 0); 
    tft.initR(INITR_BLACKTAB);
    tft.setSPISpeed(79000000);
    tft.setRotation(3);
    tft.fillScreen(ST7735_BLACK);
    frame.setFont(&abcd);
    initLimiter(&limiter, 16384*global_vol, 16380*global_vol);
    xTouchPadQueue = xQueueCreate(4, sizeof(key_event_t));
    xOptionKeyQueue = xQueueCreate(4, sizeof(key_event_t));
    // initDelayBuffer();
    esp_err_t ret;
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
    key_event_t optionKeyEvent;
    xTaskCreatePinnedToCore(&input, "input", 4096, NULL, 2, NULL, 0);
    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);
    while (ret != ESP_OK) {
        analogWrite(LCD_BK, 255);
        MainReDraw();
        fillMidRect(151, 34, 0x4208);
        drawMidRect(151, 34, ST7735_WHITE);
        setMidCusr(151, 34, 4);
        frame.setTextColor(0x7bcf);
        uint8_t X=frame.getCursorX();
        frame.printf("SDCARD STATUS ERROR %x:\n", ret);
        frame.setCursor(X, frame.getCursorY()+2);
        frame.println((ret == 263) ? "SDCARD NOT FOUND" : ((ret == -1) ? "UNKNOW FILE SYSTEM" : "UNKNOW ERROR"));
        frame.setCursor(X, frame.getCursorY()+2);
        frame.printf("PRESS ANY KEY TO RETRY");
        setMidCusr(151, 34, 3);
        X=frame.getCursorX();
        frame.setTextColor(ST7735_WHITE);
        frame.printf("SDCARD STATUS ERROR %x:\n", ret);
        frame.setCursor(X, frame.getCursorY()+2);
        frame.println((ret == 263) ? "SDCARD NOT FOUND" : ((ret == -1) ? "UNKNOW FILE SYSTEM" : "UNKNOW ERROR"));
        frame.setCursor(X, frame.getCursorY()+2);
        frame.printf("PRESS ANY KEY TO RETRY");
        if (readOptionKeyEvent == pdTRUE) {
            MainReDraw();
            frame.display();
            ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);
            vTaskDelay(4);
        }
        vTaskDelay(16);
        frame.display();
    }
    xTaskCreatePinnedToCore(&display_lcd, "tracker_ui", 8192, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(&refesMpr121, "MPR121", 4096, NULL, 0, NULL, 0);
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
    vTaskDelay(256);
    printf("MAIN EXIT\n");
}

void loop() {
    printf("DELETE LOOP\n");
    vTaskDelete(NULL);
}