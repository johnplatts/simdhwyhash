// Copyright 2024 John Platts. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simdhwyhash.h"

#include <gtest/gtest.h>

namespace simdhwyhash {
namespace test {
namespace {

TEST(SimdHwyHashTest, TestKnownValuesWithKey1234) {
  static constexpr uint64_t kKey[4] = {1, 2, 3, 4};
  static constexpr uint8_t kB0[33] = {
      128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138,
      139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149,
      150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160};
  static constexpr uint8_t kB1[1] = {255};

  const uint64_t actual_hash0 = SimdHwyHash_Hash64(kB0, 33, kKey);
  EXPECT_EQ(actual_hash0, uint64_t{0x53c516cce478cad7U});

  const uint64_t actual_hash1 = SimdHwyHash_Hash64(kB1, 1, kKey);
  EXPECT_EQ(actual_hash1, uint64_t{0x7858f24d2d79b2b2U});
}

TEST(SimdHwyHashTest, TestArrays) {
  static constexpr uint64_t kExpected64[65] = {
      0x907A56DE22C26E53U, 0x7EAB43AAC7CDDD78U, 0xB8D0569AB0B53D62U,
      0x5C6BEFAB8A463D80U, 0xF205A46893007EDAU, 0x2B8A1668E4A94541U,
      0xBD4CCC325BEFCA6FU, 0x4D02AE1738F59482U, 0xE1205108E55F3171U,
      0x32D2644EC77A1584U, 0xF6E10ACDB103A90BU, 0xC3BBF4615B415C15U,
      0x243CC2040063FA9CU, 0xA89A58CE65E641FFU, 0x24B031A348455A23U,
      0x40793F86A449F33BU, 0xCFAB3489F97EB832U, 0x19FE67D2C8C5C0E2U,
      0x04DD90A69C565CC2U, 0x75D9518E2371C504U, 0x38AD9B1141D3DD16U,
      0x0264432CCD8A70E0U, 0xA9DB5A6288683390U, 0xD7B05492003F028CU,
      0x205F615AEA59E51EU, 0xEEE0C89621052884U, 0x1BFC1A93A7284F4FU,
      0x512175B5B70DA91DU, 0xF71F8976A0A2C639U, 0xAE093FEF1F84E3E7U,
      0x22CA92B01161860FU, 0x9FC7007CCF035A68U, 0xA0C964D9ECD580FCU,
      0x2C90F73CA03181FCU, 0x185CF84E5691EB9EU, 0x4FC1F5EF2752AA9BU,
      0xF5B7391A5E0A33EBU, 0xB9B84B83B4E96C9CU, 0x5E42FE712A5CD9B4U,
      0xA150F2F90C3F97DCU, 0x7FA522D75E2D637DU, 0x181AD0CC0DFFD32BU,
      0x3889ED981E854028U, 0xFB4297E8C586EE2DU, 0x6D064A45BB28059CU,
      0x90563609B3EC860CU, 0x7AA4FCE94097C666U, 0x1326BAC06B911E08U,
      0xB926168D2B154F34U, 0x9919848945B1948DU, 0xA2A98FC534825EBEU,
      0xE9809095213EF0B6U, 0x582E5483707BC0E9U, 0x086E9414A88A6AF5U,
      0xEE86B98D20F6743DU, 0xF89B7FF609B1C0A7U, 0x4C7D9CC19E22C3E8U,
      0x9A97005024562A6FU, 0x5DD41CF423E6EBEFU, 0xDF13609C0468E227U,
      0x6E0DA4F64188155AU, 0xB755BA4B50D7D4A1U, 0x887A3484647479BDU,
      0xAB8EEBE9BF2139A0U, 0x75542C5D4CD2A6FFU};
  static constexpr uint64_t kKey[4] = {0x0706050403020100U, 0x0F0E0D0C0B0A0908U,
                                       0x1716151413121110U,
                                       0x1F1E1D1C1B1A1918U};
  static constexpr uint8_t kData[65] = {
      0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16,
      17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33,
      34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
      51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64};

  for (size_t i = 0; i <= 64; i++) {
    const uint64_t actual_64 = SimdHwyHash_Hash64(kData, i, kKey);
    EXPECT_EQ(actual_64, kExpected64[i]);
  }
}

static inline uint64_t HwyHashZipperMerge0(uint64_t v1, uint64_t v0) {
  return (((v0 & 0xff000000U) | (v1 & 0xff00000000U)) >> 24) |
         (((v0 & 0xff0000000000U) | (v1 & 0xff000000000000U)) >> 16) |
         (v0 & 0xff0000U) | ((v0 & 0xff00U) << 32) |
         ((v1 & 0xff00000000000000U) >> 8) | (v0 << 56);
}

static inline uint64_t HwyHashZipperMerge1(uint64_t v1, uint64_t v0) {
  return (((v1 & 0xff000000U) | (v0 & 0xff00000000U)) >> 24) |
         (v1 & 0xff0000U) | ((v1 & 0xff0000000000U) >> 16) |
         ((v1 & 0xff00U) << 24) | ((v0 & 0xff000000000000U) >> 8) |
         ((v1 & 0xffU) << 48) | (v0 & 0xff00000000000000U);
}

static void UpdateHwyHashState(SimdHwyHashState* SIMDHWYHASH_RESTRICT state,
                               uint64_t a0, uint64_t a1, uint64_t a2,
                               uint64_t a3) {
  state->v1[0] += state->mul0[0] + a0;
  state->v1[1] += state->mul0[1] + a1;
  state->v1[2] += state->mul0[2] + a2;
  state->v1[3] += state->mul0[3] + a3;
  for (int i = 0; i < 4; ++i) {
    state->mul0[i] ^=
        (state->v1[i] & uint64_t{0xffffffffU}) * (state->v0[i] >> 32);
    state->v0[i] += state->mul1[i];
    state->mul1[i] ^=
        (state->v0[i] & uint64_t{0xffffffffU}) * (state->v1[i] >> 32);
  }
  state->v0[0] += HwyHashZipperMerge0(state->v1[1], state->v1[0]);
  state->v0[1] += HwyHashZipperMerge1(state->v1[1], state->v1[0]);
  state->v0[2] += HwyHashZipperMerge0(state->v1[3], state->v1[2]);
  state->v0[3] += HwyHashZipperMerge1(state->v1[3], state->v1[2]);
  state->v1[0] += HwyHashZipperMerge0(state->v0[1], state->v0[0]);
  state->v1[1] += HwyHashZipperMerge1(state->v0[1], state->v0[0]);
  state->v1[2] += HwyHashZipperMerge0(state->v0[3], state->v0[2]);
  state->v1[3] += HwyHashZipperMerge1(state->v0[3], state->v0[2]);
}

static inline void PermuteAndUpdateHwyHashState(
    SimdHwyHashState* SIMDHWYHASH_RESTRICT state) {
  UpdateHwyHashState(state, (state->v0[2] >> 32) | (state->v0[2] << 32),
                     (state->v0[3] >> 32) | (state->v0[3] << 32),
                     (state->v0[0] >> 32) | (state->v0[0] << 32),
                     (state->v0[1] >> 32) | (state->v0[1] << 32));
}

template <size_t kPos>
static inline void HwyHashModularReduction(uint64_t a3_unmasked, uint64_t a2,
                                           uint64_t a1, uint64_t a0,
                                           uint64_t (&hash)[4]) {
  static_assert(kPos <= 2, "kPos <= 2 must be true");
  const uint64_t a3 = a3_unmasked & 0x3FFFFFFFFFFFFFFFU;
  hash[kPos + 1] = a1 ^ ((a3 << 1) | (a2 >> 63)) ^ ((a3 << 2) | (a2 >> 62));
  hash[kPos + 0] = a0 ^ (a2 << 1) ^ (a2 << 2);
}

static inline void HwyHashFinalize128(
    SimdHwyHashState* SIMDHWYHASH_RESTRICT state, uint64_t (&hash)[2]) {
  PermuteAndUpdateHwyHashState(state);
  PermuteAndUpdateHwyHashState(state);
  PermuteAndUpdateHwyHashState(state);
  PermuteAndUpdateHwyHashState(state);
  PermuteAndUpdateHwyHashState(state);
  PermuteAndUpdateHwyHashState(state);

  hash[0] = state->v0[0] + state->mul0[0] + state->v1[2] + state->mul1[2];
  hash[1] = state->v0[1] + state->mul0[1] + state->v1[3] + state->mul1[3];
}

static inline void HwyHashFinalize256(
    SimdHwyHashState* SIMDHWYHASH_RESTRICT state, uint64_t (&hash)[4]) {
  PermuteAndUpdateHwyHashState(state);
  PermuteAndUpdateHwyHashState(state);
  PermuteAndUpdateHwyHashState(state);
  PermuteAndUpdateHwyHashState(state);
  PermuteAndUpdateHwyHashState(state);
  PermuteAndUpdateHwyHashState(state);
  PermuteAndUpdateHwyHashState(state);
  PermuteAndUpdateHwyHashState(state);
  PermuteAndUpdateHwyHashState(state);
  PermuteAndUpdateHwyHashState(state);

  HwyHashModularReduction<0>(
      state->v1[1] + state->mul1[1], state->v1[0] + state->mul1[0],
      state->v0[1] + state->mul0[1], state->v0[0] + state->mul0[0], hash);
  HwyHashModularReduction<2>(
      state->v1[3] + state->mul1[3], state->v1[2] + state->mul1[2],
      state->v0[3] + state->mul0[3], state->v0[2] + state->mul0[2], hash);
}

static inline void ComputeExpectedHwyHashExpectedHash128(
    const void* SIMDHWYHASH_RESTRICT ptr, size_t byte_len,
    const uint64_t* SIMDHWYHASH_RESTRICT key, uint64_t (&hash)[2]) {
  SimdHwyHashState state;
  SimdHwyHash_Reset(&state, key);
  SimdHwyHash_Update(&state, ptr, byte_len);
  HwyHashFinalize128(&state, hash);
}

static inline void ComputeExpectedHwyHashExpectedHash256(
    const void* SIMDHWYHASH_RESTRICT ptr, size_t byte_len,
    const uint64_t* SIMDHWYHASH_RESTRICT key, uint64_t (&hash)[4]) {
  SimdHwyHashState state;
  SimdHwyHash_Reset(&state, key);
  SimdHwyHash_Update(&state, ptr, byte_len);
  HwyHashFinalize256(&state, hash);
}

TEST(SimdHwyHashTest, TestHash128) {
  static constexpr uint64_t kKey[4] = {0x0706050403020100U, 0x0F0E0D0C0B0A0908U,
                                       0x1716151413121110U,
                                       0x1F1E1D1C1B1A1918U};
  static constexpr uint8_t kData[65] = {
      0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16,
      17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33,
      34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
      51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64};

  uint64_t expected_hash[2];
  uint64_t actual_hash[2];

  for (size_t i = 0; i <= 64; i++) {
    ComputeExpectedHwyHashExpectedHash128(kData, i, kKey, expected_hash);
    SimdHwyHash_Hash128(kData, i, kKey, actual_hash);
    EXPECT_EQ(actual_hash[0], expected_hash[0]);
    EXPECT_EQ(actual_hash[1], expected_hash[1]);
  }
}

TEST(SimdHwyHashTest, TestHash256) {
  static constexpr uint64_t kKey[4] = {0x0706050403020100U, 0x0F0E0D0C0B0A0908U,
                                       0x1716151413121110U,
                                       0x1F1E1D1C1B1A1918U};
  static constexpr uint8_t kData[65] = {
      0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16,
      17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33,
      34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
      51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64};

  uint64_t expected_hash[4];
  uint64_t actual_hash[4];

  for (size_t i = 0; i <= 64; i++) {
    ComputeExpectedHwyHashExpectedHash256(kData, i, kKey, expected_hash);
    SimdHwyHash_Hash256(kData, i, kKey, actual_hash);
    EXPECT_EQ(actual_hash[0], expected_hash[0]);
    EXPECT_EQ(actual_hash[1], expected_hash[1]);
    EXPECT_EQ(actual_hash[2], expected_hash[2]);
    EXPECT_EQ(actual_hash[3], expected_hash[3]);
  }
}

}  // namespace
}  // namespace test
}  // namespace simdhwyhash

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
