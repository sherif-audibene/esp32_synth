#ifndef WAVETABLE_MANAGER_H
#define WAVETABLE_MANAGER_H

#include <stdint.h>

#define WAVETABLE_SIZE 4096

typedef enum {
    WAVE_SINE = 0,
    WAVE_SQUARE,
    WAVE_TRIANGLE,
    WAVE_SAWTOOTH,
    WAVE_COUNT
} wavetable_type_t;

typedef struct {
    const int16_t* table;
    const char* name;
} wavetable_t;

// Function declarations
void init_wavetables(void);
const wavetable_t* get_current_wavetable(void);
void set_wavetable_type(wavetable_type_t type);
wavetable_type_t get_current_wavetable_type(void);

#endif 