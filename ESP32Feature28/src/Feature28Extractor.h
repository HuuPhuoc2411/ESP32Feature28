/**
 * @file Feature28Extractor.h
 * @brief Header file defining Feature28Extractor class for extracting 28 audio features
 * 
 * This class performs:
 * 1. FFT to calculate Peak Frequency and Spectral Centroid
 * 2. MFCC (Mel-Frequency Cepstral Coefficients) with 13 coefficients
 * 3. Calculate mean and std of MFCC coefficients
 */

#pragma once
#include <Arduino.h>

// FFT library - arduinoFFT by Enrique Condes (v2.x)
// Class name in v2.x is ArduinoFFT
#include <arduinoFFT.h>

// ============================
// PARAMETER CONFIGURATION
// ============================

/**
 * @brief Sample rate (Hz)
 * @details Default 16000 Hz (16 kHz) - standard for speech recognition
 */
#ifndef F28_SAMPLE_RATE
#define F28_SAMPLE_RATE 16000
#endif

// -------- Configuration for Peak/Centroid FFT --------

/**
 * @brief FFT size for calculating Peak Frequency and Spectral Centroid
 * @details Must be power of 2. Larger value gives higher frequency resolution
 */
#ifndef F28_FFT_N
#define F28_FFT_N 2048
#endif

/**
 * @brief Lowest frequency of interest band (Hz)
 */
#ifndef F28_FMIN_HZ
#define F28_FMIN_HZ 100.0f
#endif

/**
 * @brief Highest frequency of interest band (Hz)
 */
#ifndef F28_FMAX_HZ
#define F28_FMAX_HZ 1000.0f
#endif

// -------- Configuration for Peak Detection Algorithm --------

/**
 * @brief Number of top peaks to consider
 */
#ifndef F28_TOP_PEAKS
#define F28_TOP_PEAKS 8
#endif

/**
 * @brief K coefficient for IQR (Interquartile Range) filtering
 * @details Used to remove outliers: values outside [Q1 - k*IQR, Q3 + k*IQR]
 */
#ifndef F28_IQR_K
#define F28_IQR_K 1.5f
#endif

/**
 * @brief Number of IQR filtering iterations
 */
#ifndef F28_IQR_ITERS
#define F28_IQR_ITERS 2
#endif

/**
 * @brief Bin width in histogram for finding mode (Hz)
 */
#ifndef F28_MODE_BIN_HZ
#define F28_MODE_BIN_HZ 5.0f
#endif

/**
 * @brief Window around mode to prioritize nearby peaks (Hz)
 */
#ifndef F28_MODE_WINDOW_HZ
#define F28_MODE_WINDOW_HZ 25.0f
#endif

/**
 * @brief Beta coefficient to balance peak intensity and distance to mode
 * @details Value 0-1. Higher = prioritize peaks closer to mode
 */
#ifndef F28_BETA_CLOSE
#define F28_BETA_CLOSE 0.6f
#endif

// -------- Configuration for MFCC --------

/**
 * @brief FFT size for MFCC
 * @details Must be power of 2
 */
#ifndef F28_MFCC_NFFT
#define F28_MFCC_NFFT 512
#endif

/**
 * @brief Frame length for MFCC (number of samples)
 * @details 400 samples = 25ms @ 16kHz
 */
#ifndef F28_MFCC_FRAME_LEN
#define F28_MFCC_FRAME_LEN 400
#endif

/**
 * @brief Hop size between frames
 * @details 160 samples = 10ms @ 16kHz (15ms overlap)
 */
#ifndef F28_MFCC_HOP
#define F28_MFCC_HOP 160
#endif

/**
 * @brief Number of Mel filters
 */
#ifndef F28_MEL_FILTERS
#define F28_MEL_FILTERS 26
#endif

/**
 * @brief Number of MFCC coefficients to extract (excluding C0)
 */
#ifndef F28_MFCC_COEFFS
#define F28_MFCC_COEFFS 13
#endif

/**
 * @brief Output feature order (total 28):
 * 
 * [0]      Peak Frequency (Hz)
 * [1]      Spectral Centroid (Hz)
 * [2..14]  MFCC1..13 (mean)
 * [15..27] MFCC1..13 (std - standard deviation)
 */

/**
 * @class Feature28Extractor
 * @brief Main class for extracting 28 audio features
 * 
 * Usage:
 * 1. Initialize: Feature28Extractor extractor(SAMPLE_RATE);
 * 2. Reset: extractor.reset();
 * 3. Feed data: extractor.feed(samples, count);
 * 4. Compute: extractor.compute(features);
 */
class Feature28Extractor {
public:
  /**
   * @brief Constructor - Initialize extractor
   * @param sampleRate Sample rate (Hz), default F28_SAMPLE_RATE
   */
  Feature28Extractor(uint32_t sampleRate = F28_SAMPLE_RATE);

  /**
   * @brief Reset all buffers and accumulators
   * @details Call this function before processing a new audio segment
   */
  void reset();

  /**
   * @brief Feed PCM16 mono data for processing
   * @param samples Array of 16-bit signed integer audio samples
   * @param n Number of samples
   * @details This function processes chunks from I2S, accumulating for FFT and MFCC
   */
  void feed(const int16_t* samples, size_t n);

  /**
   * @brief Compute 28 features after feeding enough data
   * @param out28 Output array containing 28 features
   * @return true if computation successful, false if insufficient data
   * @details Need to feed at least several seconds of audio (e.g., 2 seconds) before calling
   */
  bool compute(float out28[28]);

  /**
   * @brief Get number of FFT blocks processed
   * @return Number of FFT blocks
   */
  uint32_t fftBlocksProcessed() const { return _fftBlocks; }

  /**
   * @brief Get number of MFCC frames processed
   * @return Number of MFCC frames
   */
  uint32_t mfccFramesProcessed() const { return _mfccFrames; }

private:
  uint32_t _sr;  ///< Sample rate

  // -------- Variables for Peak/Centroid accumulation --------
  double _vReal[F28_FFT_N];    ///< Real part of FFT input/output
  double _vImag[F28_FFT_N];    ///< Imaginary part of FFT input/output
  ArduinoFFT<double> _fft;     ///< FFT object from arduinoFFT library

  float _hannFFT[F28_FFT_N];                 ///< Hanning window for FFT
  float _accPow[(F28_FFT_N/2) + 1];         ///< Accumulated power spectrum
  uint32_t _fftFill;                         ///< Current position in FFT buffer
  int16_t  _fftBuf[F28_FFT_N];              ///< Temporary buffer for FFT block
  uint32_t _fftBlocks;                       ///< Number of FFT blocks processed

  // -------- Variables for MFCC accumulation --------
  float _hannMFCC[F28_MFCC_FRAME_LEN];      ///< Hanning window for MFCC frame
  int16_t _mfccFrame[F28_MFCC_FRAME_LEN];   ///< Buffer for MFCC frame
  uint32_t _mfccFill;                        ///< Current position in MFCC frame buffer

  double _mr[F28_MFCC_NFFT];                 ///< Real part for MFCC FFT
  double _mi[F28_MFCC_NFFT];                 ///< Imaginary part for MFCC FFT
  ArduinoFFT<double> _mfft;                  ///< FFT object for MFCC

  bool _mfccInited;                          ///< Flag for mel filterbank initialization
  
  /**
   * @brief Mel filterbank - weight matrix
   * @details _melBank[m][b] = weight of bin b in mel filter m
   */
  float _melBank[F28_MEL_FILTERS][(F28_MFCC_NFFT/2)+1];
  
  /**
   * @brief Cosine table for DCT (Discrete Cosine Transform)
   * @details _dctCos[k][n] = cos(π*k*(n+0.5)/MEL_FILTERS)
   */
  float _dctCos[F28_MFCC_COEFFS][F28_MEL_FILTERS];

  float _mfccSum[F28_MFCC_COEFFS];     ///< Sum of MFCC coefficients across frames
  float _mfccSumSq[F28_MFCC_COEFFS];   ///< Sum of squared MFCC coefficients
  uint32_t _mfccFrames;                 ///< Number of MFCC frames processed

private:
  /**
   * @brief Initialize Hanning windows
   */
  void _initWindows();

  /**
   * @brief Initialize Mel filterbank and DCT cosine table
   */
  void _initMFCC();

  /**
   * @brief Process one FFT block to accumulate power spectrum
   * @param block Array of 2048 samples
   */
  void _processFFTBlock(const int16_t* block);

  /**
   * @brief Process one MFCC frame to accumulate MFCC statistics
   * @param frame Array of 400 samples
   */
  void _processMFCCFrame(const int16_t* frame);

  /**
   * @brief Finalize computation of Peak Frequency and Spectral Centroid
   * @param peakHz Output: Peak Frequency
   * @param centroidHz Output: Spectral Centroid
   * @return true if successful
   */
  bool _finalizePeakCentroid(float& peakHz, float& centroidHz);

  /**
   * @brief Re-pick best peak from candidate peaks using IQR + mode scoring
   * @param candHz Array of frequencies of candidate peaks
   * @param candVal Array of values (intensities) of candidate peaks
   * @param count Number of candidates
   * @return Frequency of selected peak
   */
  float _repickPeakIQRMode(const float* candHz, const float* candVal, uint32_t count) const;

  /**
   * @brief Sort array in ascending order (insertion sort)
   * @param a Array to sort
   * @param n Number of elements
   */
  static void _sortAsc(float* a, uint32_t n);

  /**
   * @brief Calculate quantile from sorted array
   * @param sorted Array sorted in ascending order
   * @param n Number of elements
   * @param q Quantile to calculate (0.0 - 1.0)
   * @return Quantile value
   */
  static float _quantile(const float* sorted, uint32_t n, float q);
};
