/* Copyright 2022 Winterbloom LLC & Alethea Katherine Flowers

Use of this source code is governed by an MIT-style
license that can be found in the LICENSE.md file or at
https://opensource.org/licenses/MIT. */

#pragma once

/* Helpers for packing and unpacking data.

This doesn't do any byte-swapping or anything, see:
    https://commandcenter.blogspot.com/2012/04/byte-order-fallacy.html
*/

#define WNTR_UNPACK_16(data, idx) data[idx] << 8 | data[idx + 1]
#define WNTR_UNPACK_32(data, idx) data[idx] << 24 | data[idx + 1] << 16 | data[idx + 2] << 8 | data[idx + 3]
#define WNTR_UNPACK_FLOAT(data, idx)                                                                                   \
    ({                                                                                                                 \
        uint32_t pun = (uint32_t)(data[idx] << 24u | data[idx + 1] << 16u | data[idx + 2] << 8u | data[idx + 3]);      \
        (*(float*)&pun);                                                                                               \
    })

#define WNTR_PACK_16(val, data, idx)                                                                                   \
    data[idx] = val >> 8;                                                                                              \
    data[idx + 1] = val & 0xFF;
#define WNTR_PACK_32(val, data, idx)                                                                                   \
    data[idx] = val >> 24;                                                                                             \
    data[idx + 1] = val >> 16;                                                                                         \
    data[idx + 2] = val >> 8;                                                                                          \
    data[idx + 3] = val & 0xFF;
#define WNTR_PACK_FLOAT(val, data, idx)                                                                                \
    ({                                                                                                                 \
        uint32_t pun = (*(uint32_t*)&val);                                                                             \
        WNTR_PACK_32(pun, data, idx);                                                                                  \
    })
