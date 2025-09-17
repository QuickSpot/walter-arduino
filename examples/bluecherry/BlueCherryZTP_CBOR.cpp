/**
 * @file BlueCherryZTP_CBOR.cpp
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "BlueCherryZTP_CBOR.h"

int ztp_cbor_init(ZTP_CBOR* cbor, uint8_t* buffer, size_t capacity)
{
  if(buffer == NULL || capacity == 0) {
    return -1;
  }

  cbor->buffer = buffer;
  cbor->capacity = capacity;
  cbor->position = 0;

  return 0;
}

size_t ztp_cbor_size(const ZTP_CBOR* cbor)
{
  return cbor->position;
}

static int ztp_cbor_write_byte(ZTP_CBOR* cbor, uint8_t byte)
{
  if(cbor->position < cbor->capacity) {
    cbor->buffer[cbor->position++] = byte;
    return 0; // Success
  }
  return -1; // Buffer overflow
}

static int ztp_cbor_write_bytes(ZTP_CBOR* cbor, const uint8_t* data, size_t length)
{
  if(cbor->position + length <= cbor->capacity) {
    memcpy(&cbor->buffer[cbor->position], data, length);
    cbor->position += length;
    return 0; // Success
  }
  return -1; // Buffer overflow
}

static int ztp_cbor_encode_type_and_value(ZTP_CBOR* cbor, uint8_t majorType, size_t value)
{
  if(value < 24) {
    return ztp_cbor_write_byte(cbor, (majorType << 5) | value);
  } else if(value < 256) {
    if(ztp_cbor_write_byte(cbor, (majorType << 5) | 0x18) < 0)
      return -1;
    return ztp_cbor_write_byte(cbor, (uint8_t) value);
  } else if(value < 65536) {
    if(ztp_cbor_write_byte(cbor, (majorType << 5) | 0x19) < 0)
      return -1;
    uint8_t bytes[] = { (uint8_t) (value >> 8), (uint8_t) value };
    return ztp_cbor_write_bytes(cbor, bytes, 2);
  }
  return -1; // Larger values not supported
}

int ztp_cbor_encode_bytes(ZTP_CBOR* cbor, const uint8_t* data, size_t length)
{
  if(ztp_cbor_encode_type_and_value(cbor, 2, length) < 0)
    return -1;                                     // Major type 2 (byte string)
  return ztp_cbor_write_bytes(cbor, data, length); // Write byte array to buffer
}

int ztp_cbor_encode_string(ZTP_CBOR* cbor, const char* str)
{
  size_t len = strlen(str);
  if(ztp_cbor_encode_type_and_value(cbor, 3, len) < 0)
    return -1; // Major type 3 (text string)
  return ztp_cbor_write_bytes(cbor, (const uint8_t*) str, len);
}

int ztp_cbor_encode_uint64(ZTP_CBOR* cbor, uint64_t value)
{
  if(ztp_cbor_encode_type_and_value(cbor, 2, 8) < 0)
    return -1;

  uint8_t bytes[] = { (uint8_t) (value >> 56), (uint8_t) (value >> 48), (uint8_t) (value >> 40),
                      (uint8_t) (value >> 32), (uint8_t) (value >> 24), (uint8_t) (value >> 16),
                      (uint8_t) (value >> 8),  (uint8_t) value };

  return ztp_cbor_write_bytes(cbor, bytes, 8);
}

int ztp_cbor_encode_int(ZTP_CBOR* cbor, int value)
{
  if(value >= 0) {
    return ztp_cbor_encode_type_and_value(cbor, 0,
                                          (size_t) value); // Major type 0
  } else {
    return ztp_cbor_encode_type_and_value(cbor, 1,
                                          (size_t) (-value - 1)); // Major type 1
  }
}

int ztp_cbor_start_array(ZTP_CBOR* cbor, size_t size)
{
  return ztp_cbor_encode_type_and_value(cbor, 4, size); // Major type 4 (array)
}

int ztp_cbor_start_map(ZTP_CBOR* cbor, size_t size)
{
  return ztp_cbor_encode_type_and_value(cbor, 5, size); // Major type 5 (map)
}

int ztp_cbor_decode_device_id(const uint8_t* cbor_data, size_t cbor_size, char* decoded_str,
                              size_t decoded_size)
{
  if(cbor_size < 1 || !cbor_data) {
    return -1; // CBOR data is invalid
  }

  // Ensure initial byte is a text string (major type 3)
  uint8_t initial_byte = cbor_data[0];
  if((initial_byte >> 5) != 3) {
    return -2; // CBOR data is not a text string
  }

  // Extract the length of the string
  size_t length = 0;
  uint8_t additional_info = initial_byte & 0x1F;

  if(additional_info > 23) {
    return -3; // String length unsupported
  }

  length = additional_info;
  cbor_data++;
  cbor_size--;

  // Validate the length against the remaining CBOR data
  if(length > cbor_size) {
    return -4; // Incomplete CBOR data for string length
  }

  // Validate the length against the output buffer size
  if(length >= decoded_size) {
    return -5; // Decoded string buffer too small
  }

  // Copy the string into the output buffer and null-terminate it
  memcpy(decoded_str, cbor_data, length);
  decoded_str[length] = '\0';

  return 0;
}

int ztp_cbor_decode_certificate(const uint8_t* cbor_data, size_t cbor_size,
                                unsigned char* decoded_data, size_t* decoded_len)
{
  if(cbor_size < 1 || !cbor_data) {
    return -1; // CBOR data is invalid
  }

  // Ensure initial byte is a byte string (major type 2)
  uint8_t initial_byte = cbor_data[0];
  if((initial_byte >> 5) != 2) {
    return -2; // CBOR data is not a byte string
  }

  // Extract the length of the string
  size_t length = 0;
  size_t offset = 1;
  uint8_t additional_info = initial_byte & 0x1F;

  if(additional_info < 24) {
    length = additional_info;
  } else if(additional_info == 24) {
    length = cbor_data[offset++];
  } else if(additional_info == 25) {
    length = (cbor_data[offset] << 8) | cbor_data[offset + 1];
    offset += 2;
  } else if(additional_info == 26) {
    length = (cbor_data[offset] << 24) | (cbor_data[offset + 1] << 16) |
             (cbor_data[offset + 2] << 8) | cbor_data[offset + 3];
    offset += 4;
  } else {
    return -3; // Length not supported
  }

  if(offset + length > cbor_size) {
    return -4; // Length exceeds buffer size
  }

  memcpy(decoded_data, cbor_data + offset, length);
  *decoded_len = length;

  return 0;
}
