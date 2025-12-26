/**
 * @file IMP441_BasicExample.ino
 * @brief Ví dụ cơ bản về cách sử dụng thư viện ESP32Feature28 với micro IMP441
 * 
 * @details
 * Example này minh họa:
 * 1. Cấu hình I2S để đọc dữ liệu từ micro IMP441/INMP441
 * 2. Feed dữ liệu âm thanh vào Feature28Extractor
 * 3. Tính toán và hiển thị 28 đặc trưng
 * 
 * Kết nối phần cứng (ESP32-S3 + IMP441):
 * - IMP441 SCK  -> GPIO 12 (BCLK)
 * - IMP441 WS   -> GPIO 11 (LRCK/Word Select)
 * - IMP441 SD   -> GPIO 10 (Data In)
 * - IMP441 VDD  -> 3.3V
 * - IMP441 GND  -> GND
 * 
 * @note Điều chỉnh các GPIO pins cho phù hợp với board của bạn
 * 
 * @author Huu-Phuoc Nguyen
 * @date 2025-12-26
 */

#include <Arduino.h>
#include <driver/i2s.h>

// Include thư viện ESP32Feature28
#include <ESP32Feature28.h>

// =====================================================
// CẤU HÌNH I2S CHO ESP32 + IMP441
// =====================================================

/**
 * @brief Port I2S sử dụng
 * ESP32 có I2S_NUM_0 và I2S_NUM_1
 */
static const i2s_port_t I2S_PORT = I2S_NUM_0;

/**
 * @brief Định nghĩa GPIO pins cho I2S
 * Thay đổi các giá trị này cho phù hợp với board của bạn
 */
static const int PIN_I2S_BCLK = 12;  // Bit Clock (SCK)
static const int PIN_I2S_LRCK = 11;  // Word Select / Left-Right Clock (WS)
static const int PIN_I2S_DOUT = 10;  // Serial Data (SD) - từ micro vào ESP32

/**
 * @brief Tần số lấy mẫu (Hz)
 * 16000 Hz là chuẩn cho speech processing
 */
static const uint32_t SAMPLE_RATE = 16000;

/**
 * @brief Thời gian ghi âm (giây)
 * Tối thiểu ~2 giây để có đủ frames cho MFCC statistics
 */
static const uint32_t RECORD_SECONDS = 2;

/**
 * @brief Tổng số samples cần ghi
 */
static const uint32_t TOTAL_SAMPLES = SAMPLE_RATE * RECORD_SECONDS;

/**
 * @brief Kích thước mỗi chunk đọc từ I2S
 * 512 samples = 1024 bytes (mỗi sample là 2 bytes - int16_t)
 */
static const size_t CHUNK_SAMPLES = 512;

/**
 * @brief Buffer tạm để đọc dữ liệu từ I2S
 */
static int16_t i2sChunk[CHUNK_SAMPLES];

/**
 * @brief Khởi tạo Feature Extractor với sample rate
 */
Feature28Extractor extractor(SAMPLE_RATE);

// =====================================================
// HÀM CẤU HÌNH I2S
// =====================================================

/**
 * @brief Cấu hình và khởi tạo I2S driver
 * 
 * @details
 * Cấu hình I2S với các thông số:
 * - Mode: Master RX (ESP32 là master, đọc từ micro)
 * - Sample rate: 16000 Hz
 * - Bits per sample: 16-bit
 * - Channel: Mono (LEFT channel only)
 * - Communication format: I2S standard
 * 
 * @note IMP441 xuất data trên LEFT channel khi WS = LOW
 */
void setupI2S() {
  // Cấu hình I2S
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),  // Master mode, Receive
    .sample_rate = SAMPLE_RATE,                           // 16000 Hz
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,        // 16-bit samples
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,         // Mono - LEFT channel
    .communication_format = I2S_COMM_FORMAT_I2S,         // I2S format chuẩn
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,            // Interrupt level 1
    .dma_buf_count = 8,                                   // 8 DMA buffers
    .dma_buf_len = 256,                                   // 256 samples per buffer
    .use_apll = false,                                    // Không dùng APLL
    .tx_desc_auto_clear = false,                         // TX không dùng
    .fixed_mclk = 0                                       // MCLK tự động
  };

  // Cấu hình GPIO pins
  i2s_pin_config_t pin_config = {
    .bck_io_num = PIN_I2S_BCLK,      // Bit Clock
    .ws_io_num = PIN_I2S_LRCK,       // Word Select
    .data_out_num = -1,              // Không dùng (TX)
    .data_in_num = PIN_I2S_DOUT      // Data Input (RX)
  };

  // Cài đặt I2S driver
  esp_err_t err;
  
  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Error installing I2S driver: %d\n", err);
    return;
  }
  
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("Error setting I2S pins: %d\n", err);
    return;
  }
  
  // Xóa DMA buffer
  i2s_zero_dma_buffer(I2S_PORT);
  
  Serial.println("✓ I2S initialized successfully");
}

// =====================================================
// SETUP VÀ MAIN LOOP
// =====================================================

void setup() {
  // Khởi tạo Serial
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n========================================");
  Serial.println("ESP32Feature28 - IMP441 Basic Example");
  Serial.println("========================================\n");
  
  Serial.printf("Sample Rate: %d Hz\n", SAMPLE_RATE);
  Serial.printf("Recording Duration: %d seconds\n", RECORD_SECONDS);
  Serial.printf("Total Samples: %d\n\n", TOTAL_SAMPLES);

  // Cấu hình I2S
  Serial.println("Configuring I2S...");
  setupI2S();
  
  // Reset extractor
  Serial.println("Resetting feature extractor...");
  extractor.reset();
  
  // Bắt đầu ghi âm
  Serial.println("\n🎤 Recording audio...");
  Serial.println("Please make some sound!");
  
  uint32_t samplesReceived = 0;
  uint32_t lastProgress = 0;
  
  // Đọc âm thanh từ I2S và feed vào extractor
  while (samplesReceived < TOTAL_SAMPLES) {
    size_t bytesRead = 0;
    
    // Đọc dữ liệu từ I2S
    esp_err_t result = i2s_read(
      I2S_PORT,                      // Port
      (void*)i2sChunk,               // Buffer
      sizeof(i2sChunk),              // Buffer size (bytes)
      &bytesRead,                    // Bytes actually read
      portMAX_DELAY                  // Timeout (chờ vô hạn)
    );
    
    if (result != ESP_OK) {
      Serial.printf("I2S read error: %d\n", result);
      continue;
    }
    
    // Tính số samples đọc được
    size_t samplesRead = bytesRead / sizeof(int16_t);
    
    if (samplesRead == 0) {
      continue; // Không có dữ liệu, tiếp tục
    }
    
    // Feed dữ liệu vào extractor
    extractor.feed(i2sChunk, samplesRead);
    
    samplesReceived += samplesRead;
    
    // Hiển thị tiến trình (mỗi 10%)
    uint32_t progress = (samplesReceived * 100) / TOTAL_SAMPLES;
    if (progress >= lastProgress + 10) {
      Serial.printf("  Progress: %d%%\n", progress);
      lastProgress = progress;
    }
  }
  
  Serial.println("✓ Recording completed!\n");
  
  // Hiển thị thống kê
  Serial.printf("FFT Blocks Processed: %d\n", extractor.fftBlocksProcessed());
  Serial.printf("MFCC Frames Processed: %d\n\n", extractor.mfccFramesProcessed());
  
  // Tính toán 28 đặc trưng
  Serial.println("🔬 Computing features...\n");
  
  float features[28];
  bool success = extractor.compute(features);
  
  if (success) {
    Serial.println("========================================");
    Serial.println("✓ EXTRACTED FEATURES (28)");
    Serial.println("========================================\n");
    
    // Feature 0: Peak Frequency
    Serial.println("--- Frequency Domain Features ---");
    Serial.printf("[0] Peak Frequency:      %.2f Hz\n", features[0]);
    Serial.printf("[1] Spectral Centroid:   %.2f Hz\n\n", features[1]);
    
    // Features 2-14: MFCC Mean
    Serial.println("--- MFCC Coefficients (Mean) ---");
    for (int i = 0; i < 13; i++) {
      Serial.printf("[%2d] MFCC%02d mean:  %+.6f\n", 2+i, i+1, features[2 + i]);
    }
    
    Serial.println();
    
    // Features 15-27: MFCC Std
    Serial.println("--- MFCC Coefficients (Std Dev) ---");
    for (int i = 0; i < 13; i++) {
      Serial.printf("[%2d] MFCC%02d std:   %+.6f\n", 15+i, i+1, features[15 + i]);
    }
    
    Serial.println("\n========================================");
    
    // Xuất dữ liệu dạng CSV để dễ copy vào Python/Excel
    Serial.println("\n📋 CSV Format (for ML/Data Analysis):");
    Serial.print("PeakFreq,Centroid");
    for (int i = 1; i <= 13; i++) Serial.printf(",MFCC%d_mean", i);
    for (int i = 1; i <= 13; i++) Serial.printf(",MFCC%d_std", i);
    Serial.println();
    
    for (int i = 0; i < 28; i++) {
      if (i > 0) Serial.print(",");
      Serial.printf("%.6f", features[i]);
    }
    Serial.println("\n");
    
  } else {
    Serial.println("❌ Feature computation failed!");
    Serial.println("   Possible reasons:");
    Serial.println("   - Not enough audio data");
    Serial.println("   - FFT blocks or MFCC frames = 0");
    Serial.println("   Please check I2S configuration and microphone connection.");
  }
  
  Serial.println("========================================");
  Serial.println("Example completed. Reset to run again.");
  Serial.println("========================================\n");
}

/**
 * @brief Main loop - không làm gì
 * Example này chạy một lần trong setup()
 */
void loop() {
  // Không làm gì, có thể thêm code ở đây nếu muốn lặp lại
  delay(2000);
}
