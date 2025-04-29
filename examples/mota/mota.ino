/**
 * @file mota.ino
 * @author Jonas Maes <jonas@dptechnics.com>
 * @date 28 Apr 2025
 * @copyright DPTechnics bv
 * @brief Walter Modem offline OTA update
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
 * This file contains a sketch that updates the modem firmware.
 * The current included fat.img contains release version LR8.2.1.0-61488.
 * 
 * How to use?
 * 1) compile and upload sketch - use partition scheme 16M Flash (2MB APP/12.5MB FATFS)
 *    and also set option erase all flash before sketch upload to Enabled
 *    in tools menu
 * 2) run this command from the sketch folder (terminal or command prompt;
 *    on windows it will be COMx instead of /dev/ttyACMx):
      esptool.py --chip esp32s3 --port /dev/ttyACM0 --baud 921600 write_flash 0x410000 fat.img
 *
 * To update the mota.dup file to a newer dup file:
 * run a MOTA update through BlueCherry Lite,
 * and after the update is ready, use esptool to obtain the new filesystem image
 * which you can update in the repo and use on offline Walters
 * using esptool write_flash above.
   esptool.py --chip esp32s3 --port /dev/ttyACM0 --baud 921600 read_flash 0x410000 0xbe0000 fat.img
 */

#include <esp_mac.h>
#include <WalterModem.h>
#include <HardwareSerial.h>

WalterModem modem;

uint8_t otaBuffer[SPI_FLASH_BLOCK_SIZE];

void setup()
{
  Serial.begin(115200);
  delay(5000);

  Serial.print("Walter Modem firmware updater v0.0.1\r\n");

  WalterModem::begin(&Serial2);

  modem.offlineMotaUpgrade(otaBuffer);
}

void loop()
{
  delay(15000);
}
