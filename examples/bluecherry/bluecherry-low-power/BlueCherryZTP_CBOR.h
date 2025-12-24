/**
 * @file BlueCherryZTP_CBOR.h
 * @author Daan Pape <daan@dptechnics.com>
 * @author Thibo Verheyde <thibo@dptechnics.com>
 * @date 14 Jan 2025
 * @copyright DPTechnics bv
 * @brief Simplified CBOR library for BlueCherry ZTP.
 *
 * @section LICENSE
 *
 * Copyright (C) 2025, DPTechnics bv
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *   3. Neither the name of DPTechnics bv nor the names of its contributors may
 *      be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 *   4. This software, with or without modification, must only be used with a
 *      Walter board from DPTechnics bv.
 *
 *   5. Any software provided in binary form under this license must not be
 *      reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY DPTECHNICS BV “AS IS” AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DPTECHNICS BV OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 *
 * This file contains a minimalist implementation of the CBOR standard for the
 * BlueCherry ZTP service.
 *
 */

#ifndef BLUECHERRYZTP_CBOR_H
#define BLUECHERRYZTP_CBOR_H

#include <stddef.h>
#include <stdint.h>

/**
 * @brief The CBOR context structure.
 */
typedef struct {
  /**
   * @brief Output buffer pointer.
   */
  uint8_t* buffer;

  /**
   * @brief Maximum size of the buffer.
   */
  size_t capacity;

  /**
   * @brief Current write position in the buffer.
   */
  size_t position;
} ZTP_CBOR;

/**
 * @brief Initializes the CBOR context.
 *
 * @param cbor CBOR context to initialize.
 * @param buffer Output buffer to use.
 * @param capacity Maximum size of the buffer.
 *
 * @return 0 on success, non-zero on failure.
 */
int ztp_cbor_init(ZTP_CBOR* cbor, uint8_t* buffer, size_t capacity);

/**
 * @brief Returns the size of encoded data.
 *
 * @param cbor CBOR context.
 *
 * @return Size of encoded data.
 */
size_t ztp_cbor_size(const ZTP_CBOR* cbor);

/**
 * @brief Encodes a byte string into CBOR format.
 *
 * @param cbor CBOR context.
 * @param data Data to encode.
 * @param length Length of data to encode.
 *
 * @return 0 on success, non-zero on failure.
 */
int ztp_cbor_encode_bytes(ZTP_CBOR* cbor, const uint8_t* data, size_t length);

/**
 * @brief Encodes a string into CBOR format.
 *
 * @param cbor CBOR context.
 * @param str String to encode.
 *
 * @return 0 on success, non-zero on failure.
 */
int ztp_cbor_encode_string(ZTP_CBOR* cbor, const char* str);

/**
 * @brief Encodes a 64-bit unsigned integer into CBOR format.
 *
 * @param cbor CBOR context.
 * @param value Value to encode.
 *
 * @return 0 on success, non-zero on failure.
 */
int ztp_cbor_encode_uint64(ZTP_CBOR* cbor, uint64_t value);

/**
 * @brief Encodes a signed integer into CBOR format.
 *
 * @param cbor @brief CBOR context.
 * @param value @brief Value to encode.
 *
 * @return @brief 0 on success, non-zero on failure.
 */
int ztp_cbor_encode_int(ZTP_CBOR* cbor, int value);

/**
 * @brief Starts encoding an array into CBOR format.
 *
 * @param cbor @brief CBOR context.
 * @param size @brief Expected size of the array.
 *
 * @return @brief 0 on success, non-zero on failure.
 */
int ztp_cbor_start_array(ZTP_CBOR* cbor, size_t size);

/**
 * @brief Starts encoding a map into CBOR format.
 *
 * @param cbor @brief CBOR context.
 * @param size @brief Expected size of the map.
 *
 * @return @brief 0 on success, non-zero on failure.
 */
int ztp_cbor_start_map(ZTP_CBOR* cbor, size_t size);

/**
 * @brief Decodes a device ID from CBOR data.
 *
 * @param cbor_data @brief CBOR data to decode.
 * @param cbor_size @brief Size of CBOR data.
 * @param decoded_str @brief Buffer to store decoded device ID.
 * @param decoded_size @brief Size of decoded device ID buffer.
 *
 * @return @brief 0 on success, non-zero on failure.
 */
int ztp_cbor_decode_device_id(const uint8_t* cbor_data, size_t cbor_size, char* decoded_str,
                              size_t decoded_size);

/**
 * @brief Decodes a signed certificate from CBOR data.
 *
 * @param cbor_data @brief CBOR data to decode.
 * @param cbor_size @brief Size of CBOR data.
 * @param decoded_data @brief Buffer to store decoded certificate.
 * @param decoded_len @brief Pointer to store size of decoded certificate.
 *
 * @return @brief 0 on success, non-zero on failure.
 */
int ztp_cbor_decode_certificate(const uint8_t* cbor_data, size_t cbor_size,
                                unsigned char* decoded_data, size_t* decoded_len);

#endif // BLUECHERRYZTP_CBOR_H
