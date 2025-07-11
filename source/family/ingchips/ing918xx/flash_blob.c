/* Flash algorithm for ING918xxx eFlash
 *
 * Copyright (c) 2009-2022 INGCHIPS, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Generated from 'ING91800.FLM' (ING918xxx eFlash)
// digest = bc4393901525a194633d5eec1095a605cb0d9334b7b175233251ce71c6eaab2d, file size = 17604
// algo version = 0x101, algo size = 512 (0x200)
static const uint32_t ING91800_flash_prog_blob[] = {
    0xe7fdbe00,
    0x4449497e, 0x29016809, 0xf641d002, 0xe00762fa, 0x6274f642, 0x2100e004, 0x1c49e000, 0xd3fc4291,
    0xd2f81e40, 0xb5004770, 0x2144f44f, 0x60484874, 0xf7ff2001, 0x4873ffe5, 0x08496ac1, 0xd1fb07c9,
    0x4971bd00, 0x4408b500, 0x2144f44f, 0x20016048, 0xffd6f7ff, 0x6ac1486b, 0x07c90849, 0xbd00d1fb,
    0x2144f44f, 0x30c8f44f, 0x47706088, 0x2044f44f, 0xf0416801, 0x60010101, 0x07c96801, 0x4770d1fc,
    0x20024962, 0x47706008, 0x20004960, 0x47706008, 0x2001495e, 0x485b6008, 0x090968c1, 0xd1fb07c9,
    0xf44f4770, 0x20022144, 0x477060c8, 0x2144f44f, 0x60c82000, 0x49514770, 0x68094449, 0xd0012901,
    0xe006224f, 0xe0042277, 0xe0002100, 0x42911c49, 0x1e40d3fc, 0x4770d2f8, 0x6001b500, 0xf7ff2001,
    0x4848ffea, 0x07c96ac1, 0xbd00d1fc, 0xf5b0b500, 0xd30d2f04, 0x2f06f5b0, 0x2001d301, 0x2000e000,
    0x21044b43, 0xf7ff6019, 0x2000ff9c, 0xbd006018, 0x4080f5a0, 0x3045f3c0, 0xeb04f85d, 0xb570e791,
    0x2544f44f, 0xf44f2002, 0xf44f4480, 0x60e82608, 0xf7ff4620, 0xf504ffdb, 0x42b42400, 0x2000d3f8,
    0xbd7060e8, 0x41f0e92d, 0x46164605, 0x0780460c, 0xb164d119, 0xf5b01928, 0xd8142f08, 0x2844f44f,
    0xf8c82102, 0x2c04100c, 0x2704d904, 0x2000e003, 0x81f0e8bd, 0x04e84627, 0x4628d102, 0xffb6f7ff,
    0x1c406828, 0x2001d001, 0xce02e7f2, 0xf7ff4628, 0x443dffa3, 0xd1e61be4, 0xf8c82100, 0xe7e6100c,
    0x4916481c, 0x44497900, 0x00c0f3c0, 0x49176008, 0x60082002, 0x2144f44f, 0x30c8f44f, 0x47706088,
    0xf7ffb500, 0x2000ffed, 0x4910bd00, 0x2000b500, 0xf7ff6008, 0x2000ff5d, 0x2000bd00, 0xb5704770,
    0x23004604, 0x5ce5e004, 0x42b55cd6, 0x1c5bd102, 0xd3f8428b, 0xbd7018e0, 0x0000e7a4, 0x00000004,
    0x44650000, 0x40070000, 0x84650000, 0x40041000, 0x000c40a0, 0x40050000, 0x00000000, 0x00000000
};

// Start address of flash
#define flash_start 0x00004000
// Size of flash
#define flash_size 0x00084000

/**
* List of start and size for each size of flash sector - even indexes are start, odd are size
* The size will apply to all sectors between the listed address and the next address
* in the list.
* The last pair in the list will have sectors starting at that address and ending
* at address flash_start + flash_size.
*/
static const sector_info_t sectors_info[] = {
    {0x00004000, 0x00002000},
};

static const program_target_t flash = {
    0x200001c5, // Init
    0x200001cf, // UnInit
    0x20000123, // EraseChip
    0x200001df, // EraseSector
    0x200001fd, // ProgramPage
    0x200001e3, // Verify

    // BKPT : start of blob + 1
    // RSB  : blob start + header + rw data offset
    // RSP  : stack pointer
    {
        0x20000001,
        0x2000021c,
        0x2000a000
    },

    // mem buffer location
    0x2000a000,
    // location to write prog_blob in target RAM
    0x20000000,
    // prog_blob size
    sizeof(ING91800_flash_prog_blob),
    // address of prog_blob
    ING91800_flash_prog_blob,
    // ram_to_flash_bytes_to_be_written
    0x00002000,
    // algo_flags
    kAlgoVerifyReturnsAddress + kAlgoSkipChipErase,
};
