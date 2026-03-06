/*
  ┌─────────────────────────────────────────────────────────────┐
  │  BLUEPAWS V4 — AES-128-CTR PACKET ENCRYPTION               │
  │  Compact software AES for both ESP32 and nRF52840           │
  │  Encrypts packet payload (bytes 1-N), preserving version    │
  │  byte [0] so the receiver can identify the protocol.        │
  │  CTR mode: encrypt == decrypt, no padding needed.           │
  └─────────────────────────────────────────────────────────────┘

  Usage:
    uint8_t key[16] = LORA_AES_KEY;

    // After pkt_finalize(), before lora.transmit():
    bp_aes_ctr_apply(buf, pktLen, key);

    // After lora.readData(), before pkt_validate_crc():
    bp_aes_ctr_apply(buf, rxLen, key);

  The nonce is derived from bytes already in the packet
  (device_id + msg_seq), so no extra bytes are needed.
*/

#ifndef BP_CRYPTO_H
#define BP_CRYPTO_H

#include <stdint.h>
#include <string.h>

// ═══════════════════════════════════════════════
// AES-128 Core (ECB encrypt only — CTR needs encrypt only)
// Compact implementation, ~1KB flash.
// ═══════════════════════════════════════════════

static const uint8_t _aes_sbox[256] = {
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16
};

static const uint8_t _aes_rcon[11] = {
    0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36
};

// Expand 16-byte key into 11 round keys (176 bytes)
static inline void _aes_key_expand(const uint8_t key[16], uint8_t rk[176]) {
    memcpy(rk, key, 16);
    for (uint8_t i = 4; i < 44; i++) {
        uint8_t tmp[4];
        memcpy(tmp, &rk[(i - 1) * 4], 4);
        if (i % 4 == 0) {
            uint8_t t = tmp[0];
            tmp[0] = _aes_sbox[tmp[1]] ^ _aes_rcon[i / 4];
            tmp[1] = _aes_sbox[tmp[2]];
            tmp[2] = _aes_sbox[tmp[3]];
            tmp[3] = _aes_sbox[t];
        }
        for (uint8_t j = 0; j < 4; j++)
            rk[i * 4 + j] = rk[(i - 4) * 4 + j] ^ tmp[j];
    }
}

// GF(2^8) multiply by 2
static inline uint8_t _gf_mul2(uint8_t x) {
    return (x << 1) ^ ((x >> 7) * 0x1b);
}

// AES-128 encrypt one 16-byte block in-place
static inline void _aes_encrypt_block(uint8_t blk[16], const uint8_t rk[176]) {
    // AddRoundKey (round 0)
    for (uint8_t i = 0; i < 16; i++) blk[i] ^= rk[i];

    for (uint8_t round = 1; round <= 10; round++) {
        // SubBytes
        for (uint8_t i = 0; i < 16; i++) blk[i] = _aes_sbox[blk[i]];

        // ShiftRows
        uint8_t t;
        // Row 1: shift left 1
        t = blk[1]; blk[1] = blk[5]; blk[5] = blk[9]; blk[9] = blk[13]; blk[13] = t;
        // Row 2: shift left 2
        t = blk[2]; blk[2] = blk[10]; blk[10] = t;
        t = blk[6]; blk[6] = blk[14]; blk[14] = t;
        // Row 3: shift left 3
        t = blk[15]; blk[15] = blk[11]; blk[11] = blk[7]; blk[7] = blk[3]; blk[3] = t;

        // MixColumns (skip on last round)
        if (round < 10) {
            for (uint8_t c = 0; c < 4; c++) {
                uint8_t *col = &blk[c * 4];
                uint8_t a0 = col[0], a1 = col[1], a2 = col[2], a3 = col[3];
                uint8_t m0 = _gf_mul2(a0), m1 = _gf_mul2(a1);
                uint8_t m2 = _gf_mul2(a2), m3 = _gf_mul2(a3);
                col[0] = m0 ^ m1 ^ a1 ^ a2 ^ a3;
                col[1] = a0 ^ m1 ^ m2 ^ a2 ^ a3;
                col[2] = a0 ^ a1 ^ m2 ^ m3 ^ a3;
                col[3] = m0 ^ a0 ^ a1 ^ a2 ^ m3;
            }
        }

        // AddRoundKey
        const uint8_t *rkr = &rk[round * 16];
        for (uint8_t i = 0; i < 16; i++) blk[i] ^= rkr[i];
    }
}

// ═══════════════════════════════════════════════
// AES-128-CTR — Apply to packet buffer
//
// Nonce layout (16 bytes):
//   [0-1]   device_id  (from packet bytes 1-2)
//   [2-5]   msg_seq    (from packet bytes 3-6)
//   [6-11]  zero padding
//   [12-15] block counter (big-endian)
//
// Encrypts bytes 1..len-1 in-place.
// Byte 0 (protocol version) is left cleartext so
// the receiver can identify the packet format.
// ═══════════════════════════════════════════════

static inline void bp_aes_ctr_apply(uint8_t *buf, uint8_t len, const uint8_t key[16]) {
    if (len < 8) return;  // too short to have a valid nonce

    // Expand key
    uint8_t rk[176];
    _aes_key_expand(key, rk);

    // Build nonce from packet header fields (already in buf before encryption)
    uint8_t nonce[16];
    memset(nonce, 0, 16);
    memcpy(&nonce[0], &buf[1], 2);  // device_id
    memcpy(&nonce[2], &buf[3], 4);  // msg_seq

    // Encrypt bytes 1..len-1 using CTR mode
    uint8_t ctr_block[16];
    uint32_t block_ctr = 0;
    uint8_t keystream[16];
    uint8_t ks_pos = 16;  // force generation of first keystream block

    for (uint8_t i = 1; i < len; i++) {
        if (ks_pos >= 16) {
            // Generate next keystream block
            memcpy(ctr_block, nonce, 12);
            ctr_block[12] = (block_ctr >> 24) & 0xFF;
            ctr_block[13] = (block_ctr >> 16) & 0xFF;
            ctr_block[14] = (block_ctr >> 8)  & 0xFF;
            ctr_block[15] = (block_ctr)       & 0xFF;
            memcpy(keystream, ctr_block, 16);
            _aes_encrypt_block(keystream, rk);
            block_ctr++;
            ks_pos = 0;
        }
        buf[i] ^= keystream[ks_pos++];
    }
}

// Check if the AES key is all zeros (unconfigured)
static inline bool bp_aes_key_is_zero(const uint8_t key[16]) {
    for (uint8_t i = 0; i < 16; i++) {
        if (key[i] != 0) return false;
    }
    return true;
}

#endif // BP_CRYPTO_H
