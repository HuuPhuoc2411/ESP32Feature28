/**
 * @file Feature28Extractor.cpp
 * @brief Implementation of Feature28Extractor class
 */

#include "Feature28Extractor.h"
#include <math.h>
#include <string.h>

// ========================================
// MEL SCALE CONVERSION FUNCTIONS
// ========================================

/**
 * @brief Convert from Hz to Mel scale
 * @param hz Frequency in Hz
 * @return Frequency in Mel
 * @details Formula: mel = 2595 * log10(1 + hz/700)
 */
static inline float hz_to_mel(float hz) {
  return 2595.0f * log10f(1.0f + hz / 700.0f);
}

/**
 * @brief Convert from Mel scale to Hz
 * @param mel Frequency in Mel
 * @return Frequency in Hz
 * @details Formula: hz = 700 * (10^(mel/2595) - 1)
 */
static inline float mel_to_hz(float mel) {
  return 700.0f * (powf(10.0f, mel / 2595.0f) - 1.0f);
}

// ========================================
// CONSTRUCTOR AND INITIALIZATION
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
 * @brief Initialize Hanning windows
 * @details Hanning window helps reduce spectral leakage in FFT
 *          Formula: w[n] = 0.5 - 0.5*cos(2π*n/(N-1))
 */
void Feature28Extractor::_initWindows() {
  // Hanning window for large FFT (Peak/Centroid)
  for (uint32_t i = 0; i < F28_FFT_N; i++) {
    _hannFFT[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * (float)i / (float)(F28_FFT_N - 1));
  }
  
  // Hanning window for MFCC frames
  for (uint32_t i = 0; i < F28_MFCC_FRAME_LEN; i++) {
    _hannMFCC[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * (float)i / (float)(F28_MFCC_FRAME_LEN - 1));
  }
}

/**
 * @brief Initialize Mel filterbank and DCT cosine table for MFCC
 * @details 
 * 1. Create mel filterbank: triangular filters in Mel domain
 * 2. Create cosine table for DCT-II (Discrete Cosine Transform)
 */
void Feature28Extractor::_initMFCC() {
  if (_mfccInited) return;

  const float fMin = F28_FMIN_HZ;
  const float fMax = F28_FMAX_HZ;

  const uint32_t nFft = F28_MFCC_NFFT;
  const uint32_t nBins = (nFft / 2) + 1;

  // Initialize mel bank with 0
  for (uint32_t m = 0; m < F28_MEL_FILTERS; m++) {
    for (uint32_t b = 0; b < nBins; b++) {
      _melBank[m][b] = 0.0f;
    }
  }

  // Calculate evenly spaced Mel points
  float melMin = hz_to_mel(fMin);
  float melMax = hz_to_mel(fMax);

  float melPoints[F28_MEL_FILTERS + 2];
  for (uint32_t i = 0; i < F28_MEL_FILTERS + 2; i++) {
    melPoints[i] = melMin + (melMax - melMin) * (float)i / (float)(F28_MEL_FILTERS + 1);
  }

  // Convert from Mel back to Hz
  float hzPoints[F28_MEL_FILTERS + 2];
  for (uint32_t i = 0; i < F28_MEL_FILTERS + 2; i++) {
    hzPoints[i] = mel_to_hz(melPoints[i]);
  }

  // Convert from Hz to FFT bin index
  uint32_t binPoints[F28_MEL_FILTERS + 2];
  for (uint32_t i = 0; i < F28_MEL_FILTERS + 2; i++) {
    float freq = hzPoints[i];
    uint32_t bin = (uint32_t)floorf((nFft + 1) * freq / (float)_sr);
    if (bin >= nBins) bin = nBins - 1;
    binPoints[i] = bin;
  }

  // Create triangular filters
  for (uint32_t m = 0; m < F28_MEL_FILTERS; m++) {
    uint32_t left = binPoints[m];      // Start point
    uint32_t center = binPoints[m + 1]; // Peak point
    uint32_t right = binPoints[m + 2];  // End point

    // Ensure valid points
    if (center <= left) center = left + 1;
    if (right <= center) right = center + 1;
    if (right >= nBins) right = nBins - 1;

    // Create rising edge (left -> center)
    for (uint32_t b = left; b < center && b < nBins; b++) {
      _melBank[m][b] = (float)(b - left) / (float)(center - left);
    }
    
    // Create falling edge (center -> right)
    for (uint32_t b = center; b < right && b < nBins; b++) {
      _melBank[m][b] = (float)(right - b) / (float)(right - center);
    }
  }

  // Create cosine table for DCT-II
  // DCT[k] = sum(x[n] * cos(π*k*(n+0.5)/N))
  for (uint32_t k = 0; k < F28_MFCC_COEFFS; k++) {
    for (uint32_t n = 0; n < F28_MEL_FILTERS; n++) {
      _dctCos[k][n] = cosf((float)M_PI * (float)k * ((float)n + 0.5f) / (float)F28_MEL_FILTERS);
    }
  }

  _mfccInited = true;
}

// ========================================
// FEED AND DATA PROCESSING
// ========================================

/**
 * @brief Feed PCM16 samples into extractor
 * @details Each sample is fed into:
 *          1. FFT buffer to calculate Peak/Centroid
 *          2. MFCC frame buffer to calculate MFCC
 */
void Feature28Extractor::feed(const int16_t* samples, size_t n) {
  for (size_t i = 0; i < n; i++) {
    int16_t s = samples[i];

    // Feed into FFT block buffer
    _fftBuf[_fftFill++] = s;
    if (_fftFill >= F28_FFT_N) {
      _processFFTBlock(_fftBuf);
      _fftFill = 0;
    }

    // Feed into MFCC frame buffer (sliding window with hop)
    _mfccFrame[_mfccFill++] = s;
    if (_mfccFill >= F28_MFCC_FRAME_LEN) {
      _processMFCCFrame(_mfccFrame);
      
      // Shift buffer by hop size (overlap)
      const uint32_t keep = F28_MFCC_FRAME_LEN - F28_MFCC_HOP;
      if (keep > 0) {
        memmove(_mfccFrame, _mfccFrame + F28_MFCC_HOP, keep * sizeof(int16_t));
      }
      _mfccFill = keep;
    }
  }
}

/**
 * @brief Process one FFT block to accumulate power spectrum
 * @details 
 * 1. Multiply by Hanning window
 * 2. Perform FFT
 * 3. Calculate magnitude spectrum
 * 4. Accumulate power (magnitude^2)
 */
void Feature28Extractor::_processFFTBlock(const int16_t* block) {
  // Prepare FFT input (windowing)
  for (uint32_t i = 0; i < F28_FFT_N; i++) {
    _vReal[i] = (double)((float)block[i] * _hannFFT[i]);
    _vImag[i] = 0.0;
  }

  // Perform FFT (arduinoFFT v2.x API)
  _fft.compute(FFTDirection::Forward);
  _fft.complexToMagnitude();

  // Accumulate power spectrum
  const uint32_t nBins = (F28_FFT_N/2) + 1;
  for (uint32_t b = 0; b < nBins; b++) {
    float mag = (float)_vReal[b];
    _accPow[b] += mag * mag; // power = magnitude^2
  }
  
  _fftBlocks++;
}

/**
 * @brief Process one MFCC frame to calculate MFCC and accumulate statistics
 * @details 
 * 1. Windowing with Hanning
 * 2. FFT (zero-padding if frame shorter than NFFT)
 * 3. Calculate power spectrum
 * 4. Apply mel filterbank
 * 5. Log of mel energies
 * 6. DCT to get MFCC coefficients
 * 7. Accumulate sum and sum of squares for calculating mean/std
 */
void Feature28Extractor::_processMFCCFrame(const int16_t* frame) {
  if (!_mfccInited) _initMFCC();

  // Prepare MFCC FFT input (windowing + zero padding)
  for (uint32_t i = 0; i < F28_MFCC_NFFT; i++) {
    float x = 0.0f;
    if (i < F28_MFCC_FRAME_LEN) {
      x = (float)frame[i] * _hannMFCC[i];
    }
    _mr[i] = (double)x;
    _mi[i] = 0.0;
  }

  // Perform FFT
  _mfft.compute(FFTDirection::Forward);
  _mfft.complexToMagnitude();

  const uint32_t nBins = (F28_MFCC_NFFT/2) + 1;

  // Calculate power spectrum
  float pwr[(F28_MFCC_NFFT/2)+1];
  for (uint32_t b = 0; b < nBins; b++) {
    float mag = (float)_mr[b];
    pwr[b] = mag * mag + 1e-12f; // Add epsilon to avoid log(0)
  }

  // Apply mel filterbank and calculate log mel energies
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

  // DCT-II to get MFCC coefficients
  float c[F28_MFCC_COEFFS];
  for (uint32_t k = 0; k < F28_MFCC_COEFFS; k++) {
    float s = 0.0f;
    for (uint32_t n = 0; n < F28_MEL_FILTERS; n++) {
      s += melE[n] * _dctCos[k][n];
    }
    c[k] = s;
  }

  // Accumulate statistics (sum and sum of squares)
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
 * @brief Calculate final 28 features
 * @details 
 * Output features:
 * [0] Peak Frequency
 * [1] Spectral Centroid
 * [2..14] MFCC mean (13 coefficients)
 * [15..27] MFCC std (13 coefficients)
 */
bool Feature28Extractor::compute(float out28[28]) {
  // Check if there is enough data
  if (_fftBlocks == 0 || _mfccFrames == 0) {
    return false;
  }

  // Calculate Peak Frequency and Spectral Centroid
  float peakHz = 0.0f, centroidHz = 0.0f;
  if (!_finalizePeakCentroid(peakHz, centroidHz)) {
    return false;
  }

  const float invN = 1.0f / (float)_mfccFrames;

  // Feature 0-1: Peak and Centroid
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
    if (var < 0.0f) var = 0.0f; // Handle numerical error
    out28[2 + F28_MFCC_COEFFS + k] = sqrtf(var); // Std = sqrt(Variance)
  }

  return true;
}

// ========================================
// PEAK & CENTROID FINALIZATION
// ========================================

/**
 * @brief Finalize calculation of Peak Frequency and Spectral Centroid
 * @details 
 * 1. Calculate average power spectrum
 * 2. Calculate Spectral Centroid = weighted average frequency
 * 3. Find local maxima (peaks) in band [FMIN, FMAX]
 * 4. Select best peak using IQR + mode scoring algorithm
 */
bool Feature28Extractor::_finalizePeakCentroid(float& peakHz, float& centroidHz) {
  const uint32_t nBins = (F28_FFT_N/2) + 1;
  const float binHz = (float)_sr / (float)F28_FFT_N;

  // Calculate average power spectrum
  float avgPow[nBins];
  const float invBlocks = 1.0f / (float)_fftBlocks;
  for (uint32_t b = 0; b < nBins; b++) {
    avgPow[b] = _accPow[b] * invBlocks;
  }

  // Determine bins in band of interest [FMIN, FMAX]
  uint32_t bMin = (uint32_t)ceilf(F28_FMIN_HZ / binHz);
  uint32_t bMax = (uint32_t)floorf(F28_FMAX_HZ / binHz);
  if (bMax >= nBins) bMax = nBins - 1;
  if (bMin < 1) bMin = 1;
  if (bMax <= bMin) return false;

  // ---- Calculate Spectral Centroid ----
  // Centroid = sum(f * E) / sum(E)
  double num = 0.0, den = 0.0;
  for (uint32_t b = bMin; b <= bMax; b++) {
    float f = (float)b * binHz;
    float e = avgPow[b];
    num += (double)f * (double)e;
    den += (double)e;
  }
  centroidHz = (den > 0.0) ? (float)(num / den) : 0.0f;

  // ---- Find local maxima peaks ----
  float candHz[F28_TOP_PEAKS]  = {0};  // Peak frequencies
  float candVal[F28_TOP_PEAKS] = {0};  // Peak values (intensities)
  uint32_t count = 0;

  // Find all local maxima in band
  for (uint32_t b = bMin + 1; b + 1 <= bMax; b++) {
    float v0 = avgPow[b - 1];
    float v1 = avgPow[b];
    float v2 = avgPow[b + 1];
    
    // Local maximum: v1 >= v0 and v1 >= v2
    if (v1 >= v0 && v1 >= v2) {
      float hz = (float)b * binHz;
      
      // Insert into top peaks list (sorted by value descending)
      for (uint32_t k = 0; k < F28_TOP_PEAKS; k++) {
        if (v1 > candVal[k]) {
          // Shift elements down
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

  // Count valid peaks
  for (uint32_t k = 0; k < F28_TOP_PEAKS; k++) {
    if (candVal[k] > 0.0f) count++;
  }
  if (count == 0) return false;

  // Select best peak using IQR + mode algorithm
  peakHz = _repickPeakIQRMode(candHz, candVal, count);
  
  return true;
}

/**
 * @brief Re-select best peak from candidates using IQR filtering + mode-based scoring
 * @details 
 * Algorithm:
 * 1. IQR filtering: Remove outliers based on Interquartile Range
 * 2. Mode detection: Find mode (most frequent value) using histogram
 * 3. Scoring: Select peak with highest score, prioritizing:
 *    - High intensity (large magnitude value)
 *    - Close to mode (common frequency)
 */
float Feature28Extractor::_repickPeakIQRMode(const float* candHz, const float* candVal, uint32_t count) const {
  // Copy Hz values for processing
  float hzTmp[F28_TOP_PEAKS];
  uint32_t n = count;
  for (uint32_t i = 0; i < n; i++) {
    hzTmp[i] = candHz[i];
  }

  // Array to mark peaks kept after filtering
  bool keep[F28_TOP_PEAKS];
  for (uint32_t i = 0; i < n; i++) {
    keep[i] = true;
  }

  // ---- IQR filtering (iterate multiple times) ----
  for (uint32_t it = 0; it < F28_IQR_ITERS; it++) {
    // Get remaining Hz values that are kept
    float kept[F28_TOP_PEAKS];
    uint32_t kn = 0;
    for (uint32_t i = 0; i < n; i++) {
      if (keep[i]) kept[kn++] = hzTmp[i];
    }
    
    if (kn < 3) break; // Not enough to calculate IQR

    // Sort to calculate quantiles
    _sortAsc(kept, kn);
    
    // Calculate Q1 (25th percentile) and Q3 (75th percentile)
    float q1 = _quantile(kept, kn, 0.25f);
    float q3 = _quantile(kept, kn, 0.75f);
    float iqr = q3 - q1;
    
    // Determine bounds: [Q1 - k*IQR, Q3 + k*IQR]
    float lo = q1 - F28_IQR_K * iqr;
    float hi = q3 + F28_IQR_K * iqr;

    // Remove outliers
    for (uint32_t i = 0; i < n; i++) {
      if (!keep[i]) continue;
      float v = hzTmp[i];
      if (v < lo || v > hi) {
        keep[i] = false;
      }
    }
  }

  // ---- Find mode using histogram ----
  float minHz = 1e9f, maxHz = -1e9f;
  uint32_t keptCount = 0;
  
  for (uint32_t i = 0; i < n; i++) {
    if (!keep[i]) continue;
    keptCount++;
    if (hzTmp[i] < minHz) minHz = hzTmp[i];
    if (hzTmp[i] > maxHz) maxHz = hzTmp[i];
  }
  
  if (keptCount == 0) {
    return candHz[0]; // Fallback: return strongest peak
  }

  // Create histogram with bin width = MODE_BIN_HZ
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

  // Find bin with highest count (mode)
  uint32_t bestBi = 0;
  uint16_t bestCnt = 0;
  for (uint32_t i = 0; i < bins; i++) {
    if (hist[i] > bestCnt) {
      bestCnt = hist[i];
      bestBi = i;
    }
  }
  
  // Frequency at bin center is the mode
  float modeHz = minHz + (bestBi + 0.5f) * binW;

  // ---- Scoring: select best peak ----
  // Score = normalized_magnitude * (1 - beta * normalized_distance_to_mode)
  
  float maxVal = candVal[0];
  if (maxVal <= 0.0f) maxVal = 1.0f;

  float bestScore = -1e9f;
  float bestHz = candHz[0];

  for (uint32_t i = 0; i < n; i++) {
    // If IQR filtering applied, only consider remaining peaks; otherwise consider all
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
    
    // Penalty for outliers (if IQR filtering applied)
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
 * @brief Sort array in ascending order using Insertion Sort
 * @details Simple and efficient for small arrays (< 20 elements)
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
 * @brief Calculate quantile from sorted array
 * @param sorted Array sorted in ascending order
 * @param n Number of elements
 * @param q Quantile to calculate (0.0 = min, 0.5 = median, 1.0 = max)
 * @return Quantile value (linear interpolation)
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
  
  float t = pos - (float)i; // Fractional part
  return sorted[i] * (1.0f - t) + sorted[j] * t;
}
