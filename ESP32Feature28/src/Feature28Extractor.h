/**
 * @file Feature28Extractor.h
 * @brief Header file định nghĩa class Feature28Extractor để trích xuất 28 đặc trưng âm thanh
 * 
 * Class này thực hiện:
 * 1. FFT để tính Peak Frequency và Spectral Centroid
 * 2. MFCC (Mel-Frequency Cepstral Coefficients) với 13 hệ số
 * 3. Tính toán mean và std của các MFCC coefficients
 */

#pragma once
#include <Arduino.h>

// Thư viện FFT - arduinoFFT by Enrique Condes (v2.x)
// Class name trong v2.x là ArduinoFFT
#include <arduinoFFT.h>

// ============================
// CẤU HÌNH CÁC THAM SỐ
// ============================

/**
 * @brief Tần số lấy mẫu (Hz)
 * @details Mặc định 16000 Hz (16 kHz) - chuẩn cho nhận dạng giọng nói
 */
#ifndef F28_SAMPLE_RATE
#define F28_SAMPLE_RATE 16000
#endif

// -------- Cấu hình cho Peak/Centroid FFT --------

/**
 * @brief Số điểm FFT cho việc tính Peak Frequency và Spectral Centroid
 * @details Phải là lũy thừa của 2. Giá trị lớn hơn cho độ phân giải tần số cao hơn
 */
#ifndef F28_FFT_N
#define F28_FFT_N 2048
#endif

/**
 * @brief Tần số thấp nhất của băng tần quan tâm (Hz)
 */
#ifndef F28_FMIN_HZ
#define F28_FMIN_HZ 100.0f
#endif

/**
 * @brief Tần số cao nhất của băng tần quan tâm (Hz)
 */
#ifndef F28_FMAX_HZ
#define F28_FMAX_HZ 1000.0f
#endif

// -------- Cấu hình cho thuật toán Peak Detection --------

/**
 * @brief Số lượng đỉnh cao nhất cần xem xét
 */
#ifndef F28_TOP_PEAKS
#define F28_TOP_PEAKS 8
#endif

/**
 * @brief Hệ số K cho IQR (Interquartile Range) filtering
 * @details Dùng để loại bỏ outliers: giá trị nằm ngoài [Q1 - k*IQR, Q3 + k*IQR]
 */
#ifndef F28_IQR_K
#define F28_IQR_K 1.5f
#endif

/**
 * @brief Số lần lặp IQR filtering
 */
#ifndef F28_IQR_ITERS
#define F28_IQR_ITERS 2
#endif

/**
 * @brief Độ rộng mỗi bin trong histogram để tìm mode (Hz)
 */
#ifndef F28_MODE_BIN_HZ
#define F28_MODE_BIN_HZ 5.0f
#endif

/**
 * @brief Cửa sổ xung quanh mode để ưu tiên các đỉnh gần mode (Hz)
 */
#ifndef F28_MODE_WINDOW_HZ
#define F28_MODE_WINDOW_HZ 25.0f
#endif

/**
 * @brief Hệ số beta để cân bằng giữa cường độ đỉnh và khoảng cách đến mode
 * @details Giá trị 0-1. Cao hơn = ưu tiên đỉnh gần mode hơn
 */
#ifndef F28_BETA_CLOSE
#define F28_BETA_CLOSE 0.6f
#endif

// -------- Cấu hình cho MFCC --------

/**
 * @brief Số điểm FFT cho MFCC
 * @details Phải là lũy thừa của 2
 */
#ifndef F28_MFCC_NFFT
#define F28_MFCC_NFFT 512
#endif

/**
 * @brief Độ dài frame cho MFCC (số samples)
 * @details 400 samples = 25ms @ 16kHz
 */
#ifndef F28_MFCC_FRAME_LEN
#define F28_MFCC_FRAME_LEN 400
#endif

/**
 * @brief Khoảng nhảy giữa các frame (hop size)
 * @details 160 samples = 10ms @ 16kHz (overlap 15ms)
 */
#ifndef F28_MFCC_HOP
#define F28_MFCC_HOP 160
#endif

/**
 * @brief Số lượng Mel filters
 */
#ifndef F28_MEL_FILTERS
#define F28_MEL_FILTERS 26
#endif

/**
 * @brief Số hệ số MFCC cần trích xuất (không tính C0)
 */
#ifndef F28_MFCC_COEFFS
#define F28_MFCC_COEFFS 13
#endif

/**
 * @brief Thứ tự các đặc trưng đầu ra (tổng 28):
 * 
 * [0]      Peak Frequency (Hz) - Tần số đỉnh
 * [1]      Spectral Centroid (Hz) - Tần số trọng tâm phổ
 * [2..14]  MFCC1..13 (mean) - Giá trị trung bình 13 hệ số MFCC
 * [15..27] MFCC1..13 (std) - Độ lệch chuẩn 13 hệ số MFCC
 */

/**
 * @class Feature28Extractor
 * @brief Class chính để trích xuất 28 đặc trưng âm thanh
 * 
 * Cách sử dụng:
 * 1. Khởi tạo: Feature28Extractor extractor(SAMPLE_RATE);
 * 2. Reset: extractor.reset();
 * 3. Feed dữ liệu: extractor.feed(samples, count);
 * 4. Tính toán: extractor.compute(features);
 */
class Feature28Extractor {
public:
  /**
   * @brief Constructor - Khởi tạo extractor
   * @param sampleRate Tần số lấy mẫu (Hz), mặc định F28_SAMPLE_RATE
   */
  Feature28Extractor(uint32_t sampleRate = F28_SAMPLE_RATE);

  /**
   * @brief Reset tất cả bộ đệm và bộ tích lũy
   * @details Gọi hàm này trước khi bắt đầu xử lý một đoạn âm thanh mới
   */
  void reset();

  /**
   * @brief Đưa dữ liệu PCM16 mono vào để xử lý
   * @param samples Mảng các mẫu âm thanh 16-bit signed integer
   * @param n Số lượng samples
   * @details Hàm này xử lý theo từng chunk từ I2S, tích lũy cho FFT và MFCC
   */
  void feed(const int16_t* samples, size_t n);

  /**
   * @brief Tính toán 28 đặc trưng sau khi đã feed đủ dữ liệu
   * @param out28 Mảng output chứa 28 đặc trưng
   * @return true nếu tính toán thành công, false nếu chưa đủ dữ liệu
   * @details Cần feed ít nhất vài giây âm thanh (ví dụ 2 giây) trước khi gọi
   */
  bool compute(float out28[28]);

  /**
   * @brief Lấy số lượng FFT blocks đã xử lý
   * @return Số FFT blocks
   */
  uint32_t fftBlocksProcessed() const { return _fftBlocks; }

  /**
   * @brief Lấy số lượng MFCC frames đã xử lý
   * @return Số MFCC frames
   */
  uint32_t mfccFramesProcessed() const { return _mfccFrames; }

private:
  uint32_t _sr;  ///< Tần số lấy mẫu

  // -------- Biến cho Peak/Centroid accumulation --------
  double _vReal[F28_FFT_N];    ///< Phần thực của FFT input/output
  double _vImag[F28_FFT_N];    ///< Phần ảo của FFT input/output
  ArduinoFFT<double> _fft;     ///< Object FFT từ thư viện arduinoFFT

  float _hannFFT[F28_FFT_N];                 ///< Hanning window cho FFT
  float _accPow[(F28_FFT_N/2) + 1];         ///< Tích lũy power spectrum
  uint32_t _fftFill;                         ///< Vị trí hiện tại trong buffer FFT
  int16_t  _fftBuf[F28_FFT_N];              ///< Buffer tạm cho FFT block
  uint32_t _fftBlocks;                       ///< Số FFT blocks đã xử lý

  // -------- Biến cho MFCC accumulation --------
  float _hannMFCC[F28_MFCC_FRAME_LEN];      ///< Hanning window cho MFCC frame
  int16_t _mfccFrame[F28_MFCC_FRAME_LEN];   ///< Buffer cho MFCC frame
  uint32_t _mfccFill;                        ///< Vị trí hiện tại trong MFCC frame buffer

  double _mr[F28_MFCC_NFFT];                 ///< Phần thực cho MFCC FFT
  double _mi[F28_MFCC_NFFT];                 ///< Phần ảo cho MFCC FFT
  ArduinoFFT<double> _mfft;                  ///< Object FFT cho MFCC

  bool _mfccInited;                          ///< Flag đã khởi tạo mel filterbank chưa
  
  /**
   * @brief Mel filterbank - ma trận trọng số
   * @details _melBank[m][b] = trọng số của bin b trong mel filter m
   */
  float _melBank[F28_MEL_FILTERS][(F28_MFCC_NFFT/2)+1];
  
  /**
   * @brief Bảng cosine cho DCT (Discrete Cosine Transform)
   * @details _dctCos[k][n] = cos(π*k*(n+0.5)/MEL_FILTERS)
   */
  float _dctCos[F28_MFCC_COEFFS][F28_MEL_FILTERS];

  float _mfccSum[F28_MFCC_COEFFS];     ///< Tổng các MFCC coefficients qua các frame
  float _mfccSumSq[F28_MFCC_COEFFS];   ///< Tổng bình phương MFCC coefficients
  uint32_t _mfccFrames;                 ///< Số MFCC frames đã xử lý

private:
  /**
   * @brief Khởi tạo Hanning windows
   */
  void _initWindows();

  /**
   * @brief Khởi tạo Mel filterbank và DCT cosine table
   */
  void _initMFCC();

  /**
   * @brief Xử lý một FFT block để tích lũy power spectrum
   * @param block Mảng 2048 samples
   */
  void _processFFTBlock(const int16_t* block);

  /**
   * @brief Xử lý một MFCC frame để tích lũy MFCC statistics
   * @param frame Mảng 400 samples
   */
  void _processMFCCFrame(const int16_t* frame);

  /**
   * @brief Hoàn tất tính toán Peak Frequency và Spectral Centroid
   * @param peakHz Output: Peak Frequency
   * @param centroidHz Output: Spectral Centroid
   * @return true nếu thành công
   */
  bool _finalizePeakCentroid(float& peakHz, float& centroidHz);

  /**
   * @brief Chọn lại peak tốt nhất từ các candidate peaks bằng IQR + mode scoring
   * @param candHz Mảng tần số của các candidate peaks
   * @param candVal Mảng giá trị (cường độ) của các candidate peaks
   * @param count Số lượng candidates
   * @return Tần số của peak được chọn
   */
  float _repickPeakIQRMode(const float* candHz, const float* candVal, uint32_t count) const;

  /**
   * @brief Sắp xếp mảng tăng dần (insertion sort)
   * @param a Mảng cần sắp xếp
   * @param n Số phần tử
   */
  static void _sortAsc(float* a, uint32_t n);

  /**
   * @brief Tính quantile từ mảng đã sắp xếp
   * @param sorted Mảng đã sắp xếp tăng dần
   * @param n Số phần tử
   * @param q Quantile cần tính (0.0 - 1.0)
   * @return Giá trị quantile
   */
  static float _quantile(const float* sorted, uint32_t n, float q);
};
