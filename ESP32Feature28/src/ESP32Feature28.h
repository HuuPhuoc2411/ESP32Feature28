/**
 * @file ESP32Feature28.h
 * @brief Thư viện trích xuất 28 đặc trưng âm thanh từ ESP32 với micro I2S
 * 
 * Thư viện này cung cấp khả năng trích xuất 28 đặc trưng từ tín hiệu âm thanh:
 * - Peak Frequency (tần số đỉnh)
 * - Spectral Centroid (tâm phổ)
 * - 13 MFCC coefficients (mean - giá trị trung bình)
 * - 13 MFCC coefficients (std - độ lệch chuẩn)
 * 
 * @author Huu-Phuoc Nguyen
 * @version 1.0.0
 * @date 2025-12-26
 * 
 * @section dependencies Các thư viện phụ thuộc
 * - arduinoFFT (by Enrique Condes) version 2.x
 * 
 * @section license Giấy phép
 * MIT License
 */

#pragma once

// Include file header chính của thư viện
#include "Feature28Extractor.h"
