//============================================================================
// vSMC/example/rng/include/rng_aes.cpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c); 2013-2016, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION); HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE);
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#include <vsmc/rng/aes_ni.hpp>

int main()
{
    unsigned char test_plain_text[64] = {0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40,
        0x9f, 0x96, 0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a, 0xae, 0x2d,
        0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c, 0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf,
        0x8e, 0x51, 0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11, 0xe5, 0xfb,
        0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef, 0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f,
        0x9b, 0x17, 0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10};

    unsigned char test_key_128[16] = {0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2,
        0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};

    unsigned char test_key_192[24] = {0x8e, 0x73, 0xb0, 0xf7, 0xda, 0x0e, 0x64,
        0x52, 0xc8, 0x10, 0xf3, 0x2b, 0x80, 0x90, 0x79, 0xe5, 0x62, 0xf8, 0xea,
        0xd2, 0x52, 0x2c, 0x6b, 0x7b};

    unsigned char test_key_256[32] = {0x60, 0x3d, 0xeb, 0x10, 0x15, 0xca, 0x71,
        0xbe, 0x2b, 0x73, 0xae, 0xf0, 0x85, 0x7d, 0x77, 0x81, 0x1f, 0x35, 0x2c,
        0x07, 0x3b, 0x61, 0x08, 0xd7, 0x2d, 0x98, 0x10, 0xa3, 0x09, 0x14, 0xdf,
        0xf4};

    unsigned char test_init_vector[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
        0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

    unsigned char test_cipher_128_cbc[64] = {0x76, 0x49, 0xab, 0xac, 0x81,
        0x19, 0xb2, 0x46, 0xce, 0xe9, 0x8e, 0x9b, 0x12, 0xe9, 0x19, 0x7d, 0x50,
        0x86, 0xcb, 0x9b, 0x50, 0x72, 0x19, 0xee, 0x95, 0xdb, 0x11, 0x3a, 0x91,
        0x76, 0x78, 0xb2, 0x73, 0xbe, 0xd6, 0xb8, 0xe3, 0xc1, 0x74, 0x3b, 0x71,
        0x16, 0xe6, 0x9e, 0x22, 0x22, 0x95, 0x16, 0x3f, 0xf1, 0xca, 0xa1, 0x68,
        0x1f, 0xac, 0x09, 0x12, 0x0e, 0xca, 0x30, 0x75, 0x86, 0xe1, 0xa7};

    unsigned char test_cipher_192_cbc[64] = {0x4f, 0x02, 0x1d, 0xb2, 0x43,
        0xbc, 0x63, 0x3d, 0x71, 0x78, 0x18, 0x3a, 0x9f, 0xa0, 0x71, 0xe8, 0xb4,
        0xd9, 0xad, 0xa9, 0xad, 0x7d, 0xed, 0xf4, 0xe5, 0xe7, 0x38, 0x76, 0x3f,
        0x69, 0x14, 0x5a, 0x57, 0x1b, 0x24, 0x20, 0x12, 0xfb, 0x7a, 0xe0, 0x7f,
        0xa9, 0xba, 0xac, 0x3d, 0xf1, 0x02, 0xe0, 0x08, 0xb0, 0xe2, 0x79, 0x88,
        0x59, 0x88, 0x81, 0xd9, 0x20, 0xa9, 0xe6, 0x4f, 0x56, 0x15, 0xcd};

    unsigned char test_cipher_256_cbc[64] = {0xf5, 0x8c, 0x4c, 0x04, 0xd6,
        0xe5, 0xf1, 0xba, 0x77, 0x9e, 0xab, 0xfb, 0x5f, 0x7b, 0xfb, 0xd6, 0x9c,
        0xfc, 0x4e, 0x96, 0x7e, 0xdb, 0x80, 0x8d, 0x67, 0x9f, 0x77, 0x7b, 0xc6,
        0x70, 0x2c, 0x7d, 0x39, 0xf2, 0x33, 0x69, 0xa9, 0xd9, 0xba, 0xcf, 0xa5,
        0x30, 0xe2, 0x63, 0x04, 0x23, 0x14, 0x61, 0xb2, 0xeb, 0x05, 0xe2, 0xc3,
        0x9b, 0xe9, 0xfc, 0xda, 0x6c, 0x19, 0x07, 0x8c, 0x6a, 0x9d, 0x1b};

    std::array<std::array<std::uint64_t, 2>, 4> plain_text;
    std::array<std::uint64_t, 2> key_128;
    std::array<std::uint64_t, 3> key_192;
    std::array<std::uint64_t, 4> key_256;
    std::array<std::uint64_t, 2> init_vector;
    std::array<std::array<std::uint64_t, 2>, 4> cipher_128_cbc;
    std::array<std::array<std::uint64_t, 2>, 4> cipher_192_cbc;
    std::array<std::array<std::uint64_t, 2>, 4> cipher_256_cbc;

    std::memcpy(plain_text.data(), test_plain_text, 64);
    std::memcpy(key_128.data(), test_key_128, 16);
    std::memcpy(key_192.data(), test_key_192, 24);
    std::memcpy(key_256.data(), test_key_256, 32);
    std::memcpy(init_vector.data(), test_init_vector, 16);
    std::memcpy(cipher_128_cbc.data(), test_cipher_128_cbc, 64);
    std::memcpy(cipher_192_cbc.data(), test_cipher_192_cbc, 64);
    std::memcpy(cipher_256_cbc.data(), test_cipher_256_cbc, 64);

    vsmc::AES128x1_64::generator_type aes128;
    vsmc::AES192x1_64::generator_type aes192;
    vsmc::AES256x1_64::generator_type aes256;

    aes128.reset(key_128);
    aes192.reset(key_192);
    aes256.reset(key_256);

    auto ctr128 = init_vector;
    auto ctr192 = init_vector;
    auto ctr256 = init_vector;
    for (std::size_t j = 0; j != 2; ++j) {
        ctr128[j] ^= plain_text[0][j];
        ctr192[j] ^= plain_text[0][j];
        ctr256[j] ^= plain_text[0][j];
    }

    std::array<std::array<std::uint64_t, 2>, 4> buf128 = plain_text;
    std::array<std::array<std::uint64_t, 2>, 4> buf192 = plain_text;
    std::array<std::array<std::uint64_t, 2>, 4> buf256 = plain_text;

    std::cout << std::uppercase << std::hex;
    for (std::size_t i = 0; i != 4; ++i) {
        aes128.enc(ctr128, buf128[i]);
        aes192.enc(ctr192, buf192[i]);
        aes256.enc(ctr256, buf256[i]);
        if (i < 3) {
            for (std::size_t j = 0; j != 2; ++j) {
                ctr128[j] = buf128[i][j] ^ plain_text[i + 1][j];
                ctr192[j] = buf192[i][j] ^ plain_text[i + 1][j];
                ctr256[j] = buf256[i][j] ^ plain_text[i + 1][j];
            }
        }
    }

    bool pass128 = cipher_128_cbc == buf128;
    bool pass192 = cipher_192_cbc == buf192;
    bool pass256 = cipher_256_cbc == buf256;

    std::cout << "AES128: " << (pass128 ? "Passed" : "Failed") << std::endl;
    std::cout << "AES192: " << (pass192 ? "Passed" : "Failed") << std::endl;
    std::cout << "AES256: " << (pass256 ? "Passed" : "Failed") << std::endl;

    return 0;
}
