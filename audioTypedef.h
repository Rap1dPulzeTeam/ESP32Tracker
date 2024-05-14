#include <stdint.h>

typedef struct {
    int16_t dataL;
    int16_t dataR;
} audio16BitStro;

typedef struct {
    int16_t data;
} audio16BitMono;

typedef struct {
    int8_t dataL;
    int8_t dataR;
} audio8BitStro;

typedef struct {
    uint8_t dataL;
    uint8_t dataR;
} audioU8BitStro;

typedef struct {
    int8_t data;
} audio8BitMono;

typedef struct {
    uint8_t data;
} audioU8BitMono;

audio16BitStro *buffer16BitStro;
audio16BitMono *buffer16BitMono;
audio8BitMono *buffer8BitMono;
audio8BitStro *buffer8BitStro;
audioU8BitMono *bufferU8BitMono;
audioU8BitStro *bufferU8BitStro;