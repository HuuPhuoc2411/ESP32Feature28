/**
 * @file ESP32Feature28.h
 * @brief Library for extracting 28 audio features from ESP32 with I2S microphone
 * 
 * This library provides the ability to extract 28 features from audio signals:
 * - Peak Frequency
 * - Spectral Centroid
 * - 13 MFCC coefficients (mean)
 * - 13 MFCC coefficients (std - standard deviation)
 * 
 * @author Huu-Phuoc Nguyen
 * @version 1.0.0
 * @date 2025-12-26
 * 
 * @section dependencies Dependencies
 * - arduinoFFT (by Enrique Condes) version 2.x
 * 
 * @section license License
 * MIT License
 */

#pragma once

// Include main library header file
#include "Feature28Extractor.h"
