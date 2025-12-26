# ESP32Feature28
[![Arduino Library](https://www.ardu-badge.com/badge/ESP32Feature28.svg)](https://www.ardu-badge.com/ESP32Feature28)

Arduino library for extracting **28 audio features** from I2S microphones (such as IMP441, INMP441) on ESP32.

## 📌 Features

This library extracts 28 features from audio signals:

| # | Feature | Description |
|---|-----------|-------|
| 0 | **Peak Frequency** | Main peak frequency in the spectrum (Hz) |
| 1 | **Spectral Centroid** | Spectral center of mass (Hz) |
| 2-14 | **MFCC 1-13 (Mean)** | Mean values of 13 MFCC coefficients |
| 15-27 | **MFCC 1-13 (Std)** | Standard deviation of 13 MFCC coefficients |

### Applications
- 🤖 Machine Learning / AI for audio recognition
- 🔊 Audio classification (sounds, voice, music)
- 🎤 Keyword spotting
- 📊 Audio feature analysis

## 🔧 Installation

### Method 1: From Arduino Library Manager (After publication)
1. Open Arduino IDE
2. Go to **Sketch > Include Library > Manage Libraries**
3. Search for "**ESP32Feature28**"
4. Click **Install**

### Method 2: Manual Installation (Current)
1. Download this library
2. Extract to `Arduino/libraries/` folder
3. Restart Arduino IDE

### Dependencies
Required library:
- **arduinoFFT** (by Enrique Condes) - Version 2.0.0 or higher

## 📚 Usage

### Basic Code

```cpp
#include <ESP32Feature28.h>

Feature28Extractor extractor(16000); // 16kHz sample rate

void setup() {
  Serial.begin(115200);
  
  // Initialize I2S (see example)
  setupI2S();
  
  // Reset extractor
  extractor.reset();
  
  // Feed audio (example: 2 seconds)
  for (uint32_t i = 0; i < totalSamples; i += chunkSize) {
    size_t bytesRead;
    i2s_read(I2S_NUM_0, audioBuffer, bufferSize, &bytesRead, portMAX_DELAY);
    extractor.feed(audioBuffer, bytesRead / 2);
  }
  
  // Compute 28 features
  float features[28];
  if (extractor.compute(features)) {
    Serial.printf("Peak Frequency: %.2f Hz\n", features[0]);
    Serial.printf("Spectral Centroid: %.2f Hz\n", features[1]);
    
    for (int i = 0; i < 13; i++) {
      Serial.printf("MFCC%d mean: %.6f\n", i+1, features[2+i]);
    }
    for (int i = 0; i < 13; i++) {
      Serial.printf("MFCC%d std: %.6f\n", i+1, features[15+i]);
    }
  }
}

void loop() {
  // Your code here
}
```

## 🔌 Hardware Connection

### ESP32-S3 + IMP441

| IMP441 | ESP32-S3 |
|--------|----------|
| SCK    | GPIO 12  |
| WS     | GPIO 11  |
| SD     | GPIO 10  |
| VDD    | 3.3V     |
| GND    | GND      |

*Note: GPIO pins can be changed in code*

## 📖 API Reference

### Class: `Feature28Extractor`

#### Constructor
```cpp
Feature28Extractor(uint32_t sampleRate = 16000)
```
Initialize extractor with sampling rate (Hz)

#### Methods

**`void reset()`**
- Reset all buffers and accumulators
- Call before processing a new audio segment

**`void feed(const int16_t* samples, size_t n)`**
- Feed PCM16 mono data into extractor
- `samples`: Array of 16-bit signed integer samples
- `n`: Number of samples

**`bool compute(float out28[28])`**
- Compute 28 features
- `out28`: Output array to store results
- Return: `true` if successful, `false` if insufficient data

**`uint32_t fftBlocksProcessed()`**
- Returns number of FFT blocks processed

**`uint32_t mfccFramesProcessed()`**
- Returns number of MFCC frames processed

## ⚙️ Advanced Configuration

You can customize parameters by defining them before including the library:

```cpp
#define F28_SAMPLE_RATE 16000    // Sample rate
#define F28_FMIN_HZ 100.0f       // Minimum frequency of interest
#define F28_FMAX_HZ 1000.0f      // Maximum frequency of interest
#define F28_MFCC_COEFFS 13       // Number of MFCC coefficients

#include <ESP32Feature28.h>
```

See [Feature28Extractor.h](src/Feature28Extractor.h) for all available parameters.

## 📝 Examples

### IMP441_BasicExample
Complete example of reading audio from IMP441 and extracting 28 features.

See `examples/IMP441_BasicExample/` folder

## 🤝 Contributing

All contributions are welcome! Please:
1. Fork the repository
2. Create a new branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📄 License

MIT License - See [LICENSE](LICENSE) file for details

## 👤 Author

**Your Name**
- Email: huuphuoc081102@gmail.com
- GitHub: [Huu-Phuoc Nguyen](https://github.com/HuuPhuoc2411)

## 🙏 Acknowledgments

- **arduinoFFT** library by Enrique Condes
- Arduino and ESP32 community

## ❓ FAQ

**Q: Why do I need to feed at least 2 seconds of audio?**  
A: To accurately calculate MFCC mean/std, multiple frames are needed. 2 seconds @ 16kHz = ~125 MFCC frames.

**Q: Does it support microphones other than IMP441?**  
A: Yes, any I2S microphone that outputs PCM16 mono is supported (INMP441, SPH0645, etc.)

**Q: How much RAM does it use?**  
A: Approximately ~30KB for buffers and mel filterbank. Suitable for ESP32.

**Q: Processing speed?**  
A: Real-time on ESP32 @ 240MHz. Processing time << recording time.

## 📊 Changelog

### Version 1.0.0 (2025-12-26)
- ✨ Initial release
- 🎯 28 features: Peak, Centroid, MFCC mean/std
- 📦 Support ESP32 + I2S microphones
- 📚 Full documentation and examples

