# ESP32Feature28

Thư viện Arduino để trích xuất **28 đặc trưng âm thanh** từ micro I2S (như IMP441, INMP441) trên ESP32.

## 📌 Tính năng

Thư viện này trích xuất 28 đặc trưng từ tín hiệu âm thanh:

| # | Đặc trưng | Mô tả |
|---|-----------|-------|
| 0 | **Peak Frequency** | Tần số đỉnh chính trong phổ (Hz) |
| 1 | **Spectral Centroid** | Trọng tâm phổ tần số (Hz) |
| 2-14 | **MFCC 1-13 (Mean)** | Giá trị trung bình của 13 hệ số MFCC |
| 15-27 | **MFCC 1-13 (Std)** | Độ lệch chuẩn của 13 hệ số MFCC |

### Ứng dụng
- 🤖 Machine Learning / AI cho nhận dạng âm thanh
- 🔊 Phân loại âm thanh (tiếng động, giọng nói, âm nhạc)
- 🎤 Nhận dạng từ khóa (keyword spotting)
- 📊 Phân tích đặc trưng âm thanh

## 🔧 Cài đặt

### Cách 1: Từ Arduino Library Manager (Sau khi publish)
1. Mở Arduino IDE
2. Vào **Sketch > Include Library > Manage Libraries**
3. Tìm "**ESP32Feature28**"
4. Click **Install**

### Cách 2: Thủ công (Hiện tại)
1. Tải thư viện này về
2. Giải nén vào thư mục `Arduino/libraries/`
3. Khởi động lại Arduino IDE

### Thư viện phụ thuộc
Cần cài đặt thư viện:
- **arduinoFFT** (by Enrique Condes) - Version 2.0.0 trở lên

## 📚 Cách sử dụng

### Code cơ bản

```cpp
#include <ESP32Feature28.h>

Feature28Extractor extractor(16000); // 16kHz sample rate

void setup() {
  Serial.begin(115200);
  
  // Khởi tạo I2S (xem example)
  setupI2S();
  
  // Reset extractor
  extractor.reset();
  
  // Feed âm thanh (ví dụ: 2 giây)
  for (uint32_t i = 0; i < totalSamples; i += chunkSize) {
    size_t bytesRead;
    i2s_read(I2S_NUM_0, audioBuffer, bufferSize, &bytesRead, portMAX_DELAY);
    extractor.feed(audioBuffer, bytesRead / 2);
  }
  
  // Tính toán 28 đặc trưng
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

## 🔌 Kết nối phần cứng

### ESP32-S3 + IMP441

| IMP441 | ESP32-S3 |
|--------|----------|
| SCK    | GPIO 12  |
| WS     | GPIO 11  |
| SD     | GPIO 10  |
| VDD    | 3.3V     |
| GND    | GND      |

*Lưu ý: Có thể thay đổi GPIO pins trong code*

## 📖 API Reference

### Class: `Feature28Extractor`

#### Constructor
```cpp
Feature28Extractor(uint32_t sampleRate = 16000)
```
Khởi tạo extractor với tần số lấy mẫu (Hz)

#### Methods

**`void reset()`**
- Reset tất cả bộ đệm và bộ tích lũy
- Gọi trước khi xử lý đoạn âm thanh mới

**`void feed(const int16_t* samples, size_t n)`**
- Feed dữ liệu PCM16 mono vào extractor
- `samples`: Mảng các mẫu 16-bit signed integer
- `n`: Số lượng samples

**`bool compute(float out28[28])`**
- Tính toán 28 đặc trưng
- `out28`: Mảng output để lưu kết quả
- Return: `true` nếu thành công, `false` nếu chưa đủ dữ liệu

**`uint32_t fftBlocksProcessed()`**
- Trả về số FFT blocks đã xử lý

**`uint32_t mfccFramesProcessed()`**
- Trả về số MFCC frames đã xử lý

## ⚙️ Cấu hình nâng cao

Có thể tùy chỉnh các tham số bằng cách define trước khi include thư viện:

```cpp
#define F28_SAMPLE_RATE 16000    // Tần số lấy mẫu
#define F28_FMIN_HZ 100.0f       // Tần số thấp nhất quan tâm
#define F28_FMAX_HZ 1000.0f      // Tần số cao nhất quan tâm
#define F28_MFCC_COEFFS 13       // Số hệ số MFCC

#include <ESP32Feature28.h>
```

Xem [Feature28Extractor.h](src/Feature28Extractor.h) để biết đầy đủ các tham số.

## 📝 Examples

### IMP441_BasicExample
Ví dụ đầy đủ về cách đọc âm thanh từ IMP441 và trích xuất 28 đặc trưng.

Xem trong thư mục `examples/IMP441_BasicExample/`

## 🤝 Đóng góp

Mọi đóng góp đều được chào đón! Vui lòng:
1. Fork repository
2. Tạo branch mới (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to branch (`git push origin feature/AmazingFeature`)
5. Mở Pull Request

## 📄 License

MIT License - Xem file [LICENSE](LICENSE) để biết thêm chi tiết

## 👤 Author

**Your Name**
- Email: huuphuoc081102@gmail.com
- GitHub: [Huu-Phuoc Nguyen](https://github.com/HuuPhuoc2411)

## 🙏 Acknowledgments

- Thư viện **arduinoFFT** by Enrique Condes
- Cộng đồng Arduino và ESP32

## ❓ FAQ

**Q: Tại sao cần feed ít nhất 2 giây âm thanh?**  
A: Để tính toán MFCC mean/std chính xác, cần nhiều frames. 2 giây @ 16kHz = ~125 frames MFCC.

**Q: Có hỗ trợ micro khác ngoài IMP441 không?**  
A: Có, bất kỳ micro I2S nào xuất PCM16 mono đều được hỗ trợ (INMP441, SPH0645, etc.)

**Q: RAM usage là bao nhiêu?**  
A: Khoảng ~30KB cho buffers và mel filterbank. Phù hợp với ESP32.

**Q: Tốc độ xử lý?**  
A: Real-time trên ESP32 @ 240MHz. Processing time << recording time.

## 📊 Changelog

### Version 1.0.0 (2025-12-26)
- ✨ Initial release
- 🎯 28 features: Peak, Centroid, MFCC mean/std
- 📦 Support ESP32 + I2S microphones
- 📚 Full documentation and examples
