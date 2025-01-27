# ESP32 MIDI Synthesizer

## Overview
This project implements a USB MIDI synthesizer using an ESP32 microcontroller. It features polyphonic sound synthesis, ADSR envelope control, and reverb effects, all controlled via USB MIDI input.

## Features
- USB MIDI device support
- Polyphonic synthesis (up to 16 simultaneous notes)
- ADSR envelope control
- Multi-waveform synthesis (sine, square, triangle, sawtooth)
- Built-in reverb effect
- I2S audio output
- Note priority system with note memory

## Hardware Requirements
- ESP32-S2 or ESP32-S3 development board (with USB support)
- I2S DAC or I2S amplifier
- Speaker/audio output

## Pin Configuration
c
#define I2S_BCK_PIN (15) // I2S bit clock
#define I2S_LRCK_PIN (16) // I2S word select
#define I2S_DATA_PIN (17) // I2S data out

## Software Architecture

### 1. MIDI Processing
The system processes incoming MIDI messages through the TinyUSB stack. Key components:
- Note-On/Off handling
- Velocity sensitivity
- Note priority system with stack management

### 2. Sound Generation
Multiple synthesis methods are implemented:

#### Waveform Generators:
- Sine wave with noise
- Square wave
- Triangle wave
- Sawtooth wave

### 3. ADSR Envelope
Implements a professional-grade ADSR (Attack, Decay, Sustain, Release) envelope:
- Configurable attack, decay, sustain, and release times
- Smooth transitions between envelope stages
- State management

### 4. Audio Effects
#### Reverb
- Multi-tap delay implementation
- Configurable decay and mix parameters
- Real-time processing

### 5. Wavetable Synthesis
- Pre-computed sine table for efficient waveform generation
- Linear interpolation for smooth frequency transitions
- Mixed waveform generation with three simultaneous frequencies
- Amplitude scaling and normalization

## Setup and Configuration

### 1. TinyUSB Configuration
The device is configured as a USB MIDI device with customizable descriptors.

### 2. Audio Configuration
Default audio settings:

```c
#define SAMPLE_RATE (44100)
#define BUFFER_SIZE (1024)
#define REVERB_BUFFER_SIZE (SAMPLE_RATE)  // 1 second delay buffer
```

### 3. ADSR Default Settings
```c
env->attack_time = 0.1f;     // 100ms attack
env->decay_time = 0.2f;      // 200ms decay
env->sustain_level = 0.7f;   // 70% sustain
env->release_time = 0.3f;    // 300ms release
```

### 4. Task Configuration
```c
xTaskCreatePinnedToCore(
    audio_processing_task,
    "audio_task",
    4096,           // Stack size
    params,         // Task parameters
    5,              // Priority
    &audio_task_handle,
    1               // Core ID (runs on core 1)
);
```

### 5. USB MIDI Configuration
- Configurable USB descriptors for both Full Speed and High Speed modes
- Custom string descriptors for device identification
- 64-byte (FS) or 512-byte (HS) endpoint sizes
- Automatic USB driver installation with error checking

## Building and Flashing

1. Install ESP-IDF and set up the development environment
2. Clone the repository
3. Configure the project:
```bash
idf.py menuconfig
```

4. Build and flash:
```bash
idf.py -p [PORT] flash monitor
```

## Performance Considerations

1. **Memory Usage**
   - Uses static allocation for critical audio buffers
   - Implements efficient note stack management
   - Reverb buffer sized according to available memory

2. **CPU Usage**
   - Audio processing runs on a dedicated core
   - Efficient wavetable synthesis implementation
   - Optimized I2S output handling

## Contributing
Contributions are welcome! Please follow these steps:
1. Fork the repository
2. Create a feature branch
3. Submit a pull request

## License
This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments
- TinyUSB library for USB MIDI implementation
- ESP-IDF framework
- Contributors and testers

## Troubleshooting
Common issues and solutions:
1. No USB MIDI detection
   - Check USB cable connection
   - Verify USB descriptor configuration
2. Audio distortion
   - Adjust buffer sizes
   - Check I2S configuration
3. High latency
   - Optimize buffer sizes
   - Check task priorities

For more detailed information, please refer to the source code documentation and comments.
