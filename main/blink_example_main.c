#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "esp_timer.h"
#include "driver/i2s.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include <math.h>
#include "wavetable_manager.h"

// First, define the ADSR structure type before any function prototypes
typedef struct {
    int attack_samples;     // Samples for attack phase
    int decay_samples;      // Samples for decay phase
    float sustain_level;    // Sustain level (0.0 to 1.0)
    int release_samples;    // Samples for release phase
    float current_level;    // Current envelope level
    float target_level;     // Target level for current phase
    float rate;            // Rate of change per sample
    enum {
        IDLE,
        ATTACK,
        DECAY,
        SUSTAIN,
        RELEASE
    } state;
    int samples_processed;  // Counter for samples in current phase
} adsr_envelope_t;

// Add after the ADSR structure
typedef struct {
    float *delay_buffer;
    int buffer_size;
    int write_pos;
    float decay;
    float mix;
} reverb_t;

// Then define the audio params structure
typedef struct {
    int16_t *buffer;
    int buffer_size;
    float freq1;
    float freq2;
    float freq3;
    bool *playing;
    adsr_envelope_t envelope;
    reverb_t reverb;
} audio_task_params_t;

// Now add function prototypes
void generate_mixed_sine_with_noise(int16_t *buffer, int buffer_size, float freq1, float freq2, float freq3, adsr_envelope_t *env);
void send_i2s_data(int16_t *mono_buffer, int buffer_size);
void generate_silence(int16_t *buffer, int buffer_size);
void i2s_init(void);

// Add this with your other function prototypes
static void handle_note_off(uint8_t note);

static const char *TAG = "example";

/** Helper defines **/

// Interface counter
enum interface_count {
#if CFG_TUD_MIDI
    ITF_NUM_MIDI = 0,
    ITF_NUM_MIDI_STREAMING,
#endif
    ITF_COUNT
};

// USB Endpoint numbers
enum usb_endpoints {
    // Available USB Endpoints: 5 IN/OUT EPs and 1 IN EP
    EP_EMPTY = 0,
#if CFG_TUD_MIDI
    EPNUM_MIDI,
#endif
};

/** TinyUSB descriptors **/

#define TUSB_DESCRIPTOR_TOTAL_LEN (TUD_CONFIG_DESC_LEN + CFG_TUD_MIDI * TUD_MIDI_DESC_LEN)

/**
 * @brief String descriptor
 */
static const char* s_str_desc[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "SherifHamad",             // 1: Manufacturer
    "SherifHamad",      // 2: Product
    "Sherif1",              // 3: Serials, should use chip ID
    "Sherif_Esp32_Midi", // 4: MIDI
};

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and a MIDI interface
 */
static const uint8_t s_midi_cfg_desc[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, TUSB_DESCRIPTOR_TOTAL_LEN, 0, 100),

    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 4, EPNUM_MIDI, (0x80 | EPNUM_MIDI), 64),
};

#if (TUD_OPT_HIGH_SPEED)
/**
 * @brief High Speed configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and a MIDI interface
 */
static const uint8_t s_midi_hs_cfg_desc[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, TUSB_DESCRIPTOR_TOTAL_LEN, 0, 100),

    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 4, EPNUM_MIDI, (0x80 | EPNUM_MIDI), 512),
};
#endif // TUD_OPT_HIGH_SPEED

// Then declare the global variables
static audio_task_params_t *params = NULL;
static bool note_playing = false;
static TaskHandle_t audio_task_handle = NULL;

// Add MIDI note definitions
#define NOTE_OFF        0x80
#define NOTE_ON         0x90
#define POLY_PRESSURE   0xA0
#define CONTROL_CHANGE  0xB0
#define PROGRAM_CHANGE  0xC0
#define CHANNEL_PRESSURE 0xD0
#define PITCH_BEND      0xE0

// Add MIDI note to frequency conversion defines
#define MIDI_NOTE_A4 69   // MIDI note number for A4
#define FREQ_A4 440.0f    // Frequency of A4 in Hz

// Add audio definitions
#define I2S_NUM (0)
#define I2S_BCK_PIN (15)
#define I2S_LRCK_PIN (16)
#define I2S_DATA_PIN (17)
#define SAMPLE_RATE (44100)
#define BUFFER_SIZE (1024)

// Add after other #define statements
#define REVERB_BUFFER_SIZE (SAMPLE_RATE)  // 1 second delay buffer
#define NUM_DELAYS 8  // Number of delay taps

// Add these definitions at the top with other defines
#define MAX_NOTES 16

// Add these global variables with other globals
static uint8_t pressed_notes[MAX_NOTES];  // Stack to store pressed notes
static int num_pressed_notes = 0;         // Number of currently pressed notes

// Helper functions to manage the note stack
static void push_note(uint8_t note) {
    if (num_pressed_notes < MAX_NOTES) {
        pressed_notes[num_pressed_notes++] = note;
    }
}

static void remove_note(uint8_t note) {
    // Find and remove the note
    for (int i = 0; i < num_pressed_notes; i++) {
        if (pressed_notes[i] == note) {
            // Shift remaining notes down
            for (int j = i; j < num_pressed_notes - 1; j++) {
                pressed_notes[j] = pressed_notes[j + 1];
            }
            num_pressed_notes--;
            break;
        }
    }
}

static uint8_t get_top_note(void) {
    return num_pressed_notes > 0 ? pressed_notes[num_pressed_notes - 1] : 0;
}

static int16_t buffer[BUFFER_SIZE];
static int32_t stereo_buffer[BUFFER_SIZE];

static i2s_chan_handle_t tx_handle;

// Add this function to convert MIDI note to frequency
float midi_note_to_freq(uint8_t note) {
    return FREQ_A4 * powf(2.0f, (note - MIDI_NOTE_A4) / 12.0f);
}

// Updated MIDI task with note memory
static void midi_task_read_example(void *arg)
{
    uint8_t packet[4];
    
    for (;;) {
        vTaskDelay(1);
        while (tud_midi_available()) {
            if (tud_midi_packet_read(packet)) {
                uint8_t status = packet[1] & 0xF0;
                uint8_t channel = packet[1] & 0x0F;
                uint8_t note = packet[2];
                uint8_t velocity = packet[3];

                switch (status) {
                    case NOTE_ON:
                        if (velocity > 0) {
                            //ESP_LOGI(TAG, "Note ON - Channel: %d, Note: %d, Velocity: %d", 
                            //        channel + 1, note, velocity);
                            
                            // Add note to stack
                            push_note(note);
                            
                            // Get the top note (most recently pressed)
                            uint8_t current_top = get_top_note();
                            
                            // Store current envelope level for smooth transition
                            float current_level = params->envelope.current_level;
                            
                            // Update frequencies for the new note
                            float base_freq = midi_note_to_freq(current_top);
                            params->freq1 = base_freq;
                            params->freq2 = midi_note_to_freq(current_top + 7);
                            params->freq3 = midi_note_to_freq(current_top + 12);
                            
                            // Start new attack from current level if already playing
                            if (note_playing) {
                                params->envelope.current_level = current_level;
                            } else {
                                params->envelope.state = ATTACK;
                            }
                            params->envelope.samples_processed = 0;
                            note_playing = true;
                            
                        } else {
                            // Note ON with velocity 0 is equivalent to Note OFF
                            remove_note(note);
                            handle_note_off(note);
                        }
                        break;
                        
                    case NOTE_OFF:
                        //ESP_LOGI(TAG, "Note OFF - Channel: %d, Note: %d, Velocity: %d", 
                        //        channel + 1, note, velocity);
                        remove_note(note);
                        handle_note_off(note);
                        break;

                    case PROGRAM_CHANGE:
                        ESP_LOGI(TAG, "Program Change - Channel: %d, Program: %d", 
                                 channel + 1, note);
                        if (note < WAVE_COUNT) {
                            set_wavetable_type((wavetable_type_t)note);
                            ESP_LOGI(TAG, "Switched to wavetable: %s", 
                                     get_current_wavetable()->name);
                        }
                        break;

                    default:
                        ESP_LOGI(TAG, "Other MIDI message: %02X %02X %02X %02X", 
                                packet[0], packet[1], packet[2], packet[3]);
                        break;
                }
            }
        }
    }
}

// Add this helper function to handle note off logic
static void handle_note_off(uint8_t note) {
    uint8_t top_note = get_top_note();
    
    if (num_pressed_notes > 0) {
        // There are still notes pressed, switch to the top one
        float base_freq = midi_note_to_freq(top_note);
        params->freq1 = base_freq;
        params->freq2 = midi_note_to_freq(top_note + 7);
        params->freq3 = midi_note_to_freq(top_note + 12);
        // Don't reset the envelope - continue with current level
    } else {
        // No more notes pressed, start release
        note_playing = false;
        params->envelope.state = RELEASE;
        params->envelope.samples_processed = 0;
    }
}

// Function to configure I2S peripheral
void i2s_init() {
    // Create I2S channel configuration
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, NULL));

    // Create I2S standard configuration
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK_PIN,
            .ws = I2S_LRCK_PIN,
            .dout = I2S_DATA_PIN,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
}

// Function to generate clean sine wave with noise
void generate_sine_with_noise(int16_t *buffer, int buffer_size, float frequency) {
    static float phase = 0.0f;
    float phase_increment = 2.0f * M_PI * frequency / SAMPLE_RATE;
    
    for (int i = 0; i < buffer_size; i++) {
        float sine_component = 3276.7f * sinf(phase);
        
        float noise = 0;
        for (int j = 0; j < 4; j++) {
            noise += (float)(rand() - RAND_MAX/2) / RAND_MAX;
        }
        noise = noise / 8.0f;
        
        buffer[i] = (int16_t)(sine_component + (noise * 500.35f))/10;
        
        phase += phase_increment;
        if (phase >= 2.0f * M_PI) {
            phase -= 2.0f * M_PI;
        }
    }
}

// Function to generate square wave
void generate_square_wave(int16_t *buffer, int buffer_size, float frequency) {
    int16_t value;
    int samples_per_period = SAMPLE_RATE / frequency;
    for (int i = 0; i < buffer_size; i++) {
        if ((i / samples_per_period) % 2 == 0)
            value = 32767;
        else
            value = -32768;
        buffer[i] = value;
    }
}

// Function to generate triangle wave
void generate_triangle_wave(int16_t *buffer, int buffer_size, float frequency) {
    int samples_per_period = SAMPLE_RATE / frequency;
    int16_t value;
    for (int i = 0; i < buffer_size; i++) {
        int pos_in_cycle = i % samples_per_period;
        if (pos_in_cycle < samples_per_period / 2)
            value = (int16_t)((2.0f * 32767.0f / (samples_per_period / 2)) * pos_in_cycle);
        else
            value = (int16_t)((2.0f * 32767.0f / (samples_per_period / 2)) * (samples_per_period - pos_in_cycle));
        buffer[i] = value;
    }
}

// Function to generate sawtooth wave
void generate_sawtooth_wave(int16_t *buffer, int buffer_size, float frequency) {
    int samples_per_period = SAMPLE_RATE / frequency;
    for (int i = 0; i < buffer_size; i++) {
        int value = (int16_t)((32767.0f / samples_per_period) * (i % samples_per_period));
        buffer[i] = value;
    }
}

// Function to generate silence
void generate_silence(int16_t *buffer, int buffer_size) {
    for (int i = 0; i < buffer_size; i++) {
        buffer[i] = 0;
    }
}

// Function to send data to I2S
void send_i2s_data(int16_t *mono_buffer, int buffer_size) {
    size_t bytes_written;
    
    int samples_to_process = buffer_size;
    if (samples_to_process > BUFFER_SIZE) {
        samples_to_process = BUFFER_SIZE;
    }
    
    for (int i = 0; i < samples_to_process; i++) {
        stereo_buffer[i] = (mono_buffer[i] << 16) | (mono_buffer[i] & 0xFFFF);
    }
    
    i2s_channel_write(tx_handle, stereo_buffer, 
                     samples_to_process * sizeof(int32_t), 
                     &bytes_written, portMAX_DELAY);
}

// Update ADSR initialization
void init_adsr(adsr_envelope_t *env) {
    // Convert time-based parameters to sample counts
    env->attack_samples = (int)(0.05f * SAMPLE_RATE);  // 100ms attack
    env->decay_samples = (int)(0.2f * SAMPLE_RATE);   // 200ms decay
    env->sustain_level = 0.3f;                        // 70% sustain
    env->release_samples = (int)(0.3f * SAMPLE_RATE); // 300ms release
    env->current_level = 0.0f;
    env->target_level = 0.0f;
    env->rate = 0.0f;
    env->state = IDLE;
    env->samples_processed = 0;
}

// Update ADSR processing function
float process_adsr(adsr_envelope_t *env, bool note_on) {
    switch (env->state) {
        case ATTACK:
            if (env->samples_processed == 0) {
                env->target_level = 1.0f;
                env->rate = (env->target_level - env->current_level) / env->attack_samples;
            }
            
            env->current_level += env->rate;
            env->samples_processed++;
            
            if (env->samples_processed >= env->attack_samples) {
                env->current_level = env->target_level;
                env->state = DECAY;
                env->samples_processed = 0;
            }
            break;

        case DECAY:
            if (env->samples_processed == 0) {
                env->target_level = env->sustain_level;
                env->rate = (env->target_level - env->current_level) / env->decay_samples;
            }
            
            env->current_level += env->rate;
            env->samples_processed++;
            
            if (env->samples_processed >= env->decay_samples) {
                env->current_level = env->sustain_level;
                env->state = SUSTAIN;
                env->samples_processed = 0;
            }
            break;

        case SUSTAIN:
            env->current_level = env->sustain_level;
            break;

        case RELEASE:
            if (env->samples_processed == 0) {
                env->target_level = 0.0f;
                env->rate = (env->target_level - env->current_level) / env->release_samples;
            }
            
            env->current_level += env->rate;
            env->samples_processed++;
            
            if (env->samples_processed >= env->release_samples || env->current_level <= 0.001f) {
                env->current_level = 0.0f;
                env->state = IDLE;
                env->samples_processed = 0;
            }
            break;

        case IDLE:
            env->current_level = 0.0f;
            break;
    }

    // Clamp the output between 0 and 1
    if (env->current_level > 1.0f) env->current_level = 1.0f;
    if (env->current_level < 0.0f) env->current_level = 0.0f;

    return env->current_level;
}

// Update the process_reverb function for stronger effect
void process_reverb(reverb_t *rev, float *input, int buffer_size) {
    for (int i = 0; i < buffer_size; i++) {
        float output = 0.0f;
        
        // Write input to delay buffer with feedback
        rev->delay_buffer[rev->write_pos] = input[i] + 
            (rev->delay_buffer[rev->write_pos] * rev->decay * 0.5f);
        
        // Read from multiple delay taps with increased amplitude
        for (int tap = 0; tap < NUM_DELAYS; tap++) {
            int delay_samples = (rev->buffer_size / (NUM_DELAYS + 1)) * (tap + 1);
            int read_pos = rev->write_pos - delay_samples;
            if (read_pos < 0) read_pos += rev->buffer_size;
            
            // Increase the tap amplitude
            float tap_amp = 1.0f - (tap / (float)NUM_DELAYS);
            output += rev->delay_buffer[read_pos] * rev->decay * tap_amp;
        }
        
        // Mix dry and wet signals with increased wet level
        input[i] = (input[i] * (1.0f - rev->mix)) + (output * rev->mix * 2.0f);
        
        // Update write position
        rev->write_pos = (rev->write_pos + 1) % rev->buffer_size;
    }
}

// Update the generate_mixed_sine_with_noise function
void generate_mixed_sine_with_noise(int16_t *buffer, int buffer_size, float freq1, float freq2, float freq3, adsr_envelope_t *env) {
    float float_buffer[buffer_size];
    const wavetable_t* current_wavetable = get_current_wavetable();
    
    if (!*params->playing && env->state == IDLE && env->current_level < 0.001f) {
        memset(float_buffer, 0, buffer_size * sizeof(float));
    } else {
        static float phase1 = 0.0f;
        static float phase2 = 0.0f;
        static float phase3 = 0.0f;
        
        float phase_increment1 = WAVETABLE_SIZE * freq1 / SAMPLE_RATE;
        float phase_increment2 = WAVETABLE_SIZE * freq2 / SAMPLE_RATE;
        float phase_increment3 = WAVETABLE_SIZE * freq3 / SAMPLE_RATE;
        
        for (int i = 0; i < buffer_size; i++) {
            float envelope_level = process_adsr(env, *params->playing);
            
            int index1 = (int)phase1;
            int index2 = (int)phase2;
            int index3 = (int)phase3;
            
            int next_index1 = (index1 + 1) % WAVETABLE_SIZE;
            int next_index2 = (index2 + 1) % WAVETABLE_SIZE;
            int next_index3 = (index3 + 1) % WAVETABLE_SIZE;
            
            float frac1 = phase1 - index1;
            float frac2 = phase2 - index2;
            float frac3 = phase3 - index3;
            
            float wave1 = (current_wavetable->table[index1] * (1.0f - frac1) + 
                          current_wavetable->table[next_index1] * frac1) / 32768.0f;
            float wave2 = (current_wavetable->table[index2] * (1.0f - frac2) + 
                          current_wavetable->table[next_index2] * frac2) / 32768.0f;
            float wave3 = (current_wavetable->table[index3] * (1.0f - frac3) + 
                          current_wavetable->table[next_index3] * frac3) / 32768.0f;
            
            // Rest of the function remains the same, just replace sine1, sine2, sine3 with wave1, wave2, wave3
            float detune_amount = 0.2f;
            float wave1_detuned = (wave1 + wave1 * (1.0f + detune_amount)) * 0.5f;
            float wave2_detuned = (wave2 + wave2 * (1.0f + detune_amount)) * 0.5f;
            float wave3_detuned = (wave3 + wave3 * (1.0f + detune_amount)) * 0.5f;
            
            float mixed_signal = (wave1_detuned + 0.5f * wave2_detuned + 0.3f * wave3_detuned) * envelope_level;
            float_buffer[i] = mixed_signal * 0.5f;
            
            // Update phases
            phase1 += phase_increment1;
            phase2 += phase_increment2;
            phase3 += phase_increment3;
            
            while (phase1 >= WAVETABLE_SIZE) phase1 -= WAVETABLE_SIZE;
            while (phase2 >= WAVETABLE_SIZE) phase2 -= WAVETABLE_SIZE;
            while (phase3 >= WAVETABLE_SIZE) phase3 -= WAVETABLE_SIZE;
        }
    }
    
    // Process reverb and convert to int16_t as before
    process_reverb(&params->reverb, float_buffer, buffer_size);
    
    for (int i = 0; i < buffer_size; i++) {
        float sample = float_buffer[i] * INT16_MAX;
        if (sample > INT16_MAX) sample = INT16_MAX;
        if (sample < INT16_MIN) sample = INT16_MIN;
        buffer[i] = (int16_t)sample;
    }
}

// Update the audio processing task
void audio_processing_task(void *arg) {
    audio_task_params_t *params = (audio_task_params_t *)arg;
    int64_t start_time;
    
    while (1) {
        // Only measure and log time if actually synthesizing
        if (*params->playing || params->envelope.state != IDLE) {
            start_time = esp_timer_get_time();
            generate_mixed_sine_with_noise(params->buffer, params->buffer_size, 
                                         params->freq1, params->freq2, params->freq3, 
                                         &params->envelope);
            //ESP_LOGI(TAG, "Wavetable synthesis took %lld microseconds", 
            //        esp_timer_get_time() - start_time);
        } else {
            // Just fill with silence when idle
            memset(params->buffer, 0, params->buffer_size * sizeof(int16_t));
        }
        
        send_i2s_data(params->buffer, params->buffer_size);
        vTaskDelay(1); // Small delay to prevent task starvation
    }
}

void app_main(void)
{
    // Initialize I2S
    i2s_init();
    
    // Create parameters structure
    params = malloc(sizeof(audio_task_params_t));
    params->buffer = buffer;
    params->buffer_size = 512;
    params->freq1 = 0;  // Initialize with silence
    params->freq2 = 0;
    params->freq3 = 0;
    params->playing = &note_playing;
    init_adsr(&params->envelope);
    
    // Initialize reverb with much lighter settings
    params->reverb.delay_buffer = (float*)calloc(REVERB_BUFFER_SIZE, sizeof(float));
    params->reverb.buffer_size = REVERB_BUFFER_SIZE;
    params->reverb.write_pos = 0;
    params->reverb.decay = 0.3f;    // Significantly reduced decay
    params->reverb.mix = 0.15f;     // Significantly reduced wet/dry mix
    
    // Create audio task
    xTaskCreatePinnedToCore(
        audio_processing_task,
        "audio_task",
        4096,
        params,
        5,
        &audio_task_handle,
        1
    );

    // Initialize USB MIDI
    ESP_LOGI(TAG, "USB initialization");

    tinyusb_config_t const tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = s_str_desc,
        .string_descriptor_count = sizeof(s_str_desc) / sizeof(s_str_desc[0]),
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = s_midi_cfg_desc,
        .hs_configuration_descriptor = s_midi_hs_cfg_desc,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = s_midi_cfg_desc,
#endif
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    ESP_LOGI(TAG, "USB initialization DONE");

    // Create MIDI read task
    xTaskCreate(midi_task_read_example, "midi_task_read_example", 4 * 1024, NULL, 5, NULL);
}