#include "wavetable_manager.h"
#include "wavetable_sine.h"
#include "wavetable_square.h"
#include "wavetable_triangle.h"
#include "wavetable_sawtooth.h"

// Declare external references to the wavetables
extern const int16_t sine_table[WAVETABLE_SIZE];
extern const int16_t square_table[WAVETABLE_SIZE];
extern const int16_t triangle_table[WAVETABLE_SIZE];
extern const int16_t sawtooth_table[WAVETABLE_SIZE];

static wavetable_t wavetables[WAVE_COUNT] = {
    {sine_table, "Sine"},
    {square_table, "Square"},
    {triangle_table, "Triangle"},
    {sawtooth_table, "Sawtooth"}
};

static wavetable_type_t current_type = WAVE_SINE;

void init_wavetables(void) {
    current_type = WAVE_SINE;
}

const wavetable_t* get_current_wavetable(void) {
    return &wavetables[current_type];
}

void set_wavetable_type(wavetable_type_t type) {
    if (type < WAVE_COUNT) {
        current_type = type;
    }
}

wavetable_type_t get_current_wavetable_type(void) {
    return current_type;
} 