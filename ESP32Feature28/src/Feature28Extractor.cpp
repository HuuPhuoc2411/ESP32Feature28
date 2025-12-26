/**
 * @file Feature28Extractor.cpp
 * @brief Implementation của class Feature28Extractor
 */

#include "Feature28Extractor.h"
#include <math.h>
#include <string.h>

// ========================================
// CÁC HÀM CHUYỂN ĐỔI MEL SCALE
// ========================================

/**
 * @brief Chuyển đổi từ Hz sang Mel scale
 * @param hz Tần số tính bằng Hz
 * @return Tần số tính bằng Mel
 * @details Công thức: mel = 2595 * log10(1 + hz/700)
 */
static inline float hz_to_mel(float hz) {
  return 2595.0f * log10f(1.0f + hz / 700.0f);
}

/**
 * @brief Chuyển đổi từ Mel scale sang Hz
 * @param mel Tần số tính bằng Mel
 * @return Tần số tính bằng Hz
 * @details Công thức: hz = 700 * (10^(mel/2595) - 1)
 */
static inline float mel_to_hz(float mel) {
  return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

// ========================================
// CONSTRUCTOR VÀ INITIALIZATION
// ========================================

Feature28Extractor::Feature28Extractor(uint32_t sampleRate)
: _sr(sampleRate),
  _fft(_vReal, _vImag, F28_FFT_N, (double)sampleRate),
  _mfft(_mr, _mi, F28_MFCC_NFFT, (double)sampleRate),
  _mfccInited(false)
{
  reset();
  _initWindows();
  _initMFCC();
}

void Feature28Extractor::reset() {
  // Reset FFT accumulation
  _fftFill = 0;
  _fftBlocks = 0;
  for (uint32_t i = 0; i < (F28_FFT_N/2)+1; i++) {
    _accPow[i] = 0.0f;
  }

  // Reset MFCC accumulation
  _mfccFill = 0;
  _mfccFrames = 0;
  for (uint32_t i = 0; i < F28_MFCC_COEFFS; i++) {
    _mfccSum[i] = 0.0f;
    _mfccSumSq[i] = 0.0f;
  }
}

/**
 * @brief Khởi tạo các Hanning window
 * @details Hanning window giúp giảm spectral leakage trong FFT
 *          Công thức: w[n] = 0.5 - 0.5*cos(2π*n/(N-1))
 */
void Feature28Extractor::_initWindows() {
  // Hanning window cho FFT lớn (Peak/Centroid)
  for (uint32_t i = 0; i < F28_FFT_N; i++) {
    _hannFFT[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * (float)i / (float)(F28_FFT_N - 1));
  }
  
  // Hanning window cho MFCC frames
  for (uint32_t i = 0; i < F28_MFCC_FRAME_LEN; i++) {
    _hannMFCC[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * (float)i / (float)(F28_MFCC_FRAME_LEN - 1));
  }
}

/**
 * @brief Khởi tạo Mel filterbank và DCT cosine table cho MFCC
 * @details 
 * 1. Tạo mel filterbank: các bộ lọc tam giác trong miền Mel
 * 2. Tạo bảng cosine cho DCT-II (Discrete Cosine Transform)
 */
void Feature28Extractor::_initMFCC() {
  if (_mfccInited) return;

  const float fMin = F28_FMIN_HZ;
  const float fMax = F28_FMAX_HZ;

  const uint32_t nFft = F28_MFCC_NFFT;
  const uint32_t nBins = (nFft / 2) + 1;

  // Khởi tạo mel bank với 0
  for (uint32_t m = 0; m < F28_MEL_FILTERS; m++) {
    for (uint32_t b = 0; b < nBins; b++) {
      _melBank[m][b] = 0.0f;
    }
  }

  // Tính các điểm Mel đều nhau
  float melMin = hz_to_mel(fMin);
  float melMax = hz_to_mel(fMax);

  float melPoints[F28_MEL_FILTERS + 2];
  for (uint32_t i = 0; i < F28_MEL_FILTERS + 2; i++) {
    melPoints[i] = melMin + (melMax - melMin) * (float)i / (float)(F28_MEL_FILTERS + 1);
  }

  // Chuyển từ Mel về Hz
  float hzPoints[F28_MEL_FILTERS + 2];
  for (uint32_t i = 0; i < F28_MEL_FILTERS + 2; i++) {
    hzPoints[i] = mel_to_hz(melPoints[i]);
  }

  // Chuyển từ Hz sang FFT bin index
  uint32_t binPoints[F28_MEL_FILTERS + 2];
  for (uint32_t i = 0; i < F28_MEL_FILTERS + 2; i++) {
    float freq = hzPoints[i];
    uint32_t bin = (uint32_t)floorf((nFft + 1) * freq / (float)_sr);
    if (bin >= nBins) bin = nBins - 1;
    binPoints[i] = bin;
  }

  // Tạo các bộ lọc tam giác
  for (uint32_t m = 0; m < F28_MEL_FILTERS; m++) {
    uint32_t left = binPoints[m];      // Điểm bắt đầu
    uint32_t center = binPoints[m + 1]; // Điểm đỉnh
    uint32_t right = binPoints[m + 2];  // Điểm kết thúc

    // Đảm bảo các điểm hợp lệ
    if (center <= left) center = left + 1;
    if (right <= center) right = center + 1;
    if (right >= nBins) right = nBins - 1;

    // Tạo cạnh lên (left -> center)
    for (uint32_t b = left; b < center && b < nBins; b++) {
      _melBank[m][b] = (float)(b - left) / (float)(center - left);
    }
    
    // Tạo cạnh xuống (center -> right)
    for (uint32_t b = center; b < right && b < nBins; b++) {
      _melBank[m][b] = (float)(right - b) / (float)(right - center);
    }
  }

  // Tạo bảng cosine cho DCT-II
  // DCT[k] = sum(x[n] * cos(π*k*(n+0.5)/N))
  for (uint32_t k = 0; k < F28_MFCC_COEFFS; k++) {
    for (uint32_t n = 0; n < F28_MEL_FILTERS; n++) {
      _dctCos[k][n] = cosf((float)M_PI * (float)k * ((float)n + 0.5f) / (float)F28_MEL_FILTERS);
    }
  }

  _mfccInited = true;
}

// ========================================
// FEED VÀ XỬ LÝ DỮ LIỆU
// ========================================

/**
 * @brief Feed PCM16 samples vào extractor
 * @details Mỗi sample được đưa vào:
 *          1. FFT buffer để tính Peak/Centroid
 *          2. MFCC frame buffer để tính MFCC
 */
void Feature28Extractor::feed(const int16_t* samples, size_t n) {
  for (size_t i = 0; i < n; i++) {
    int16_t s = samples[i];

    // Đưa vào FFT block buffer
    _fftBuf[_fftFill++] = s;
    if (_fftFill >= F28_FFT_N) {
      _processFFTBlock(_fftBuf);
      _fftFill = 0;
    }

    // Đưa vào MFCC frame buffer (sliding window với hop)
    _mfccFrame[_mfccFill++] = s;
    if (_mfccFill >= F28_MFCC_FRAME_LEN) {
      _processMFCCFrame(_mfccFrame);
      
      // Shift buffer theo hop size (overlap)
      const uint32_t keep = F28_MFCC_FRAME_LEN - F28_MFCC_HOP;
      if (keep > 0) {
        memmove(_mfccFrame, _mfccFrame + F28_MFCC_HOP, keep * sizeof(int16_t));
      }
      _mfccFill = keep;
    }
  }
}

/**
 * @brief Xử lý một FFT block để tích lũy power spectrum
 * @details 
 * 1. Nhân với Hanning window
 * 2. Thực hiện FFT
 * 3. Tính magnitude spectrum
 * 4. Tích lũy power (magnitude^2)
 */
void Feature28Extractor::_processFFTBlock(const int16_t* block) {
  // Chuẩn bị input cho FFT (windowing)
  for (uint32_t i = 0; i < F28_FFT_N; i++) {
    _vReal[i] = (double)((float)block[i] * _hannFFT[i]);
    _vImag[i] = 0.0;
  }

  // Thực hiện FFT (arduinoFFT v2.x API)
  _fft.compute(FFTDirection::Forward);
  _fft.complexToMagnitude();

  // Tích lũy power spectrum
  const uint32_t nBins = (F28_FFT_N/2) + 1;
  for (uint32_t b = 0; b < nBins; b++) {
    float mag = (float)_vReal[b];
    _accPow[b] += mag * mag; // power = magnitude^2
  }
  
  _fftBlocks++;
}

/**
 * @brief Xử lý một MFCC frame để tính MFCC và tích lũy statistics
 * @details 
 * 1. Windowing với Hanning
 * 2. FFT (zero-padding nếu frame ngắn hơn NFFT)
 * 3. Tính power spectrum
 * 4. Áp dụng mel filterbank
 * 5. Log của mel energies
 * 6. DCT để ra MFCC coefficients
 * 7. Tích lũy sum và sum of squares cho tính mean/std
 */
void Feature28Extractor::_processMFCCFrame(const int16_t* frame) {
  if (!_mfccInited) _initMFCC();

  // Chuẩn bị MFCC FFT input (windowing + zero padding)
  for (uint32_t i = 0; i < F28_MFCC_NFFT; i++) {
    float x = 0.0f;
    if (i < F28_MFCC_FRAME_LEN) {
      x = (float)frame[i] * _hannMFCC[i];
    }
    _mr[i] = (double)x;
    _mi[i] = 0.0;
  }

  // Thực hiện FFT
  _mfft.compute(FFTDirection::Forward);
  _mfft.complexToMagnitude();

  const uint32_t nBins = (F28_MFCC_NFFT/2) + 1;

  // Tính power spectrum
  float pwr[(F28_MFCC_NFFT/2)+1];
  for (uint32_t b = 0; b < nBins; b++) {
    float mag = (float)_mr[b];
    pwr[b] = mag * mag + 1e-12f; // Thêm epsilon để tránh log(0)
  }

  // Áp dụng mel filterbank và tính log mel energies
  float melE[F28_MEL_FILTERS];
  for (uint32_t m = 0; m < F28_MEL_FILTERS; m++) {
    float sum = 0.0f;
    for (uint32_t b = 0; b < nBins; b++) {
      float w = _melBank[m][b];
      if (w > 0.0f) {
        sum += w * pwr[b];
      }
    }
    melE[m] = logf(sum + 1e-12f); // Log mel energy
  }

  // DCT-II để ra MFCC coefficients
  float c[F28_MFCC_COEFFS];
  for (uint32_t k = 0; k < F28_MFCC_COEFFS; k++) {
    float s = 0.0f;
    for (uint32_t n = 0; n < F28_MEL_FILTERS; n++) {
      s += melE[n] * _dctCos[k][n];
    }
    c[k] = s;
  }

  // Tích lũy statistics (sum và sum of squares)
  for (uint32_t k = 0; k < F28_MFCC_COEFFS; k++) {
    _mfccSum[k] += c[k];
    _mfccSumSq[k] += c[k] * c[k];
  }
  
  _mfccFrames++;
}

// ========================================
// COMPUTE FEATURES
// ========================================

/**
 * @brief Tính toán 28 đặc trưng cuối cùng
 * @details 
 * Output features:
 * [0] Peak Frequency
 * [1] Spectral Centroid
 * [2..14] MFCC mean (13 coefficients)
 * [15..27] MFCC std (13 coefficients)
 */
bool Feature28Extractor::compute(float out28[28]) {
  // Kiểm tra đã có đủ dữ liệu chưa
  if (_fftBlocks == 0 || _mfccFrames == 0) {
    return false;
  }

  // Tính Peak Frequency và Spectral Centroid
  float peakHz = 0.0f, centroidHz = 0.0f;
  if (!_finalizePeakCentroid(peakHz, centroidHz)) {
    return false;
  }

  const float invN = 1.0f / (float)_mfccFrames;

  // Feature 0-1: Peak và Centroid
  out28[0] = peakHz;
  out28[1] = centroidHz;

  // Features 2-14: MFCC mean
  for (uint32_t k = 0; k < F28_MFCC_COEFFS; k++) {
    float mean = _mfccSum[k] * invN;
    out28[2 + k] = mean;
  }

  // Features 15-27: MFCC std
  for (uint32_t k = 0; k < F28_MFCC_COEFFS; k++) {
    float mean = _mfccSum[k] * invN;
    float ex2  = _mfccSumSq[k] * invN;
    float var  = ex2 - mean * mean; // Variance = E[X^2] - E[X]^2
    if (var < 0.0f) var = 0.0f; // Xử lý lỗi số học
    out28[2 + F28_MFCC_COEFFS + k] = sqrtf(var); // Std = sqrt(Variance)
  }

  return true;
}

// ========================================
// PEAK & CENTROID FINALIZATION
// ========================================

/**
 * @brief Hoàn tất tính toán Peak Frequency và Spectral Centroid
 * @details 
 * 1. Tính average power spectrum
 * 2. Tính Spectral Centroid = weighted average frequency
 * 3. Tìm local maxima (peaks) trong band [FMIN, FMAX]
 * 4. Chọn peak tốt nhất bằng thuật toán IQR + mode scoring
 */
bool Feature28Extractor::_finalizePeakCentroid(float& peakHz, float& centroidHz) {
  const uint32_t nBins = (F28_FFT_N/2) + 1;
  const float binHz = (float)_sr / (float)F28_FFT_N;

  // Tính average power spectrum
  float avgPow[nBins];
  const float invBlocks = 1.0f / (float)_fftBlocks;
  for (uint32_t b = 0; b < nBins; b++) {
    avgPow[b] = _accPow[b] * invBlocks;
  }

  // Xác định bins trong band quan tâm [FMIN, FMAX]
  uint32_t bMin = (uint32_t)ceilf(F28_FMIN_HZ / binHz);
  uint32_t bMax = (uint32_t)floorf(F28_FMAX_HZ / binHz);
  if (bMax >= nBins) bMax = nBins - 1;
  if (bMin < 1) bMin = 1;
  if (bMax <= bMin) return false;

  // ---- Tính Spectral Centroid ----
  // Centroid = sum(f * E) / sum(E)
  double num = 0.0, den = 0.0;
  for (uint32_t b = bMin; b <= bMax; b++) {
    float f = (float)b * binHz;
    float e = avgPow[b];
    num += (double)f * (double)e;
    den += (double)e;
  }
  centroidHz = (den > 0.0) ? (float)(num / den) : 0.0f;

  // ---- Tìm local maxima peaks ----
  float candHz[F28_TOP_PEAKS]  = {0};  // Tần số của các peaks
  float candVal[F28_TOP_PEAKS] = {0};  // Giá trị (cường độ) của các peaks
  uint32_t count = 0;

  // Tìm tất cả local maxima trong band
  for (uint32_t b = bMin + 1; b + 1 <= bMax; b++) {
    float v0 = avgPow[b - 1];
    float v1 = avgPow[b];
    float v2 = avgPow[b + 1];
    
    // Local maximum: v1 >= v0 và v1 >= v2
    if (v1 >= v0 && v1 >= v2) {
      float hz = (float)b * binHz;
      
      // Chèn vào danh sách top peaks (sắp xếp theo giá trị giảm dần)
      for (uint32_t k = 0; k < F28_TOP_PEAKS; k++) {
        if (v1 > candVal[k]) {
          // Shift các phần tử sau xuống
          for (uint32_t s = F28_TOP_PEAKS - 1; s > k; s--) {
            candVal[s] = candVal[s - 1];
            candHz[s]  = candHz[s - 1];
          }
          candVal[k] = v1;
          candHz[k]  = hz;
          break;
        }
      }
    }
  }

  // Đếm số peaks hợp lệ
  for (uint32_t k = 0; k < F28_TOP_PEAKS; k++) {
    if (candVal[k] > 0.0f) count++;
  }
  if (count == 0) return false;

  // Chọn peak tốt nhất bằng thuật toán IQR + mode
  peakHz = _repickPeakIQRMode(candHz, candVal, count);
  
  return true;
}

/**
 * @brief Chọn lại peak tốt nhất từ candidates bằng IQR filtering + mode-based scoring
 * @details 
 * Thuật toán:
 * 1. IQR filtering: Loại bỏ outliers dựa trên Interquartile Range
 * 2. Mode detection: Tìm mode (giá trị xuất hiện nhiều nhất) bằng histogram
 * 3. Scoring: Chọn peak có điểm cao nhất, ưu tiên:
 *    - Cường độ cao (giá trị magnitude lớn)
 *    - Gần với mode (tần số phổ biến)
 */
float Feature28Extractor::_repickPeakIQRMode(const float* candHz, const float* candVal, uint32_t count) const {
  // Copy Hz values để xử lý
  float hzTmp[F28_TOP_PEAKS];
  uint32_t n = count;
  for (uint32_t i = 0; i < n; i++) {
    hzTmp[i] = candHz[i];
  }

  // Mảng đánh dấu peaks còn được giữ lại sau filtering
  bool keep[F28_TOP_PEAKS];
  for (uint32_t i = 0; i < n; i++) {
    keep[i] = true;
  }

  // ---- IQR filtering (lặp lại nhiều lần) ----
  for (uint32_t it = 0; it < F28_IQR_ITERS; it++) {
    // Lấy ra các Hz values còn được keep
    float kept[F28_TOP_PEAKS];
    uint32_t kn = 0;
    for (uint32_t i = 0; i < n; i++) {
      if (keep[i]) kept[kn++] = hzTmp[i];
    }
    
    if (kn < 3) break; // Không đủ để tính IQR

    // Sắp xếp để tính quantiles
    _sortAsc(kept, kn);
    
    // Tính Q1 (25th percentile) và Q3 (75th percentile)
    float q1 = _quantile(kept, kn, 0.25f);
    float q3 = _quantile(kept, kn, 0.75f);
    float iqr = q3 - q1;
    
    // Xác định bounds: [Q1 - k*IQR, Q3 + k*IQR]
    float lo = q1 - F28_IQR_K * iqr;
    float hi = q3 + F28_IQR_K * iqr;

    // Loại bỏ outliers
    for (uint32_t i = 0; i < n; i++) {
      if (!keep[i]) continue;
      float v = hzTmp[i];
      if (v < lo || v > hi) {
        keep[i] = false;
      }
    }
  }

  // ---- Tìm mode bằng histogram ----
  float minHz = 1e9f, maxHz = -1e9f;
  uint32_t keptCount = 0;
  
  for (uint32_t i = 0; i < n; i++) {
    if (!keep[i]) continue;
    keptCount++;
    if (hzTmp[i] < minHz) minHz = hzTmp[i];
    if (hzTmp[i] > maxHz) maxHz = hzTmp[i];
  }
  
  if (keptCount == 0) {
    return candHz[0]; // Fallback: trả về peak mạnh nhất
  }

  // Tạo histogram với bin width = MODE_BIN_HZ
  float binW = F28_MODE_BIN_HZ;
  uint32_t bins = (uint32_t)floorf((maxHz - minHz) / binW) + 1;
  if (bins < 1) bins = 1;
  if (bins > 128) bins = 128;

  uint16_t hist[128] = {0};
  for (uint32_t i = 0; i < n; i++) {
    if (!keep[i]) continue;
    uint32_t bi = (uint32_t)floorf((hzTmp[i] - minHz) / binW);
    if (bi >= bins) bi = bins - 1;
    hist[bi]++;
  }

  // Tìm bin có count cao nhất (mode)
  uint32_t bestBi = 0;
  uint16_t bestCnt = 0;
  for (uint32_t i = 0; i < bins; i++) {
    if (hist[i] > bestCnt) {
      bestCnt = hist[i];
      bestBi = i;
    }
  }
  
  // Tần số ở giữa bin là mode
  float modeHz = minHz + (bestBi + 0.5f) * binW;

  // ---- Scoring: chọn peak tốt nhất ----
  // Score = normalized_magnitude * (1 - beta * normalized_distance_to_mode)
  
  float maxVal = candVal[0];
  if (maxVal <= 0.0f) maxVal = 1.0f;

  float bestScore = -1e9f;
  float bestHz = candHz[0];

  for (uint32_t i = 0; i < n; i++) {
    // Nếu đã có IQR filtering, chỉ xét peaks còn lại; nếu không thì xét tất cả
    bool ok = (keptCount > 0) ? keep[i] : true;

    float hz = candHz[i];
    float val = candVal[i];
    
    // Normalize magnitude [0,1]
    float normVal = val / maxVal;

    // Normalize distance to mode [0,1]
    float dist = fabsf(hz - modeHz);
    float nd = dist / F28_MODE_WINDOW_HZ;
    if (nd > 1.0f) nd = 1.0f;

    // Scoring formula
    float score = normVal * (1.0f - F28_BETA_CLOSE * nd);
    
    // Penalty cho outliers (nếu có IQR filtering)
    if (!ok) score *= 0.3f;

    if (score > bestScore) {
      bestScore = score;
      bestHz = hz;
    }
  }
  
  return bestHz;
}

// ========================================
// UTILITY FUNCTIONS
// ========================================

/**
 * @brief Sắp xếp mảng tăng dần bằng Insertion Sort
 * @details Đơn giản và hiệu quả với mảng nhỏ (< 20 phần tử)
 */
void Feature28Extractor::_sortAsc(float* a, uint32_t n) {
  for (uint32_t i = 1; i < n; i++) {
    float key = a[i];
    int j = (int)i - 1;
    while (j >= 0 && a[j] > key) {
      a[j + 1] = a[j];
      j--;
    }
    a[j + 1] = key;
  }
}

/**
 * @brief Tính quantile từ mảng đã sắp xếp
 * @param sorted Mảng đã sắp xếp tăng dần
 * @param n Số phần tử
 * @param q Quantile cần tính (0.0 = min, 0.5 = median, 1.0 = max)
 * @return Giá trị quantile (linear interpolation)
 */
float Feature28Extractor::_quantile(const float* sorted, uint32_t n, float q) {
  if (n == 0) return 0.0f;
  if (q <= 0.0f) return sorted[0];
  if (q >= 1.0f) return sorted[n - 1];

  // Linear interpolation
  float pos = q * (float)(n - 1);
  uint32_t i = (uint32_t)floorf(pos);
  uint32_t j = i + 1;
  if (j >= n) j = n - 1;
  
  float t = pos - (float)i; // Phần thập phân
  return sorted[i] * (1.0f - t) + sorted[j] * t;
}
