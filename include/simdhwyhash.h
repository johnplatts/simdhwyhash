/* Copyright 2024 John Platts. All Rights Reserved.                         */
/*                                                                          */
/* Licensed under the Apache License, Version 2.0 (the "License");          */
/* you may not use this file except in compliance with the License.         */
/* You may obtain a copy of the License at                                  */
/*                                                                          */
/*     http://www.apache.org/licenses/LICENSE-2.0                           */
/*                                                                          */
/* Unless required by applicable law or agreed to in writing, software      */
/* distributed under the License is distributed on an "AS IS" BASIS,        */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. */
/* See the License for the specific language governing permissions and      */
/* limitations under the License.                                           */

#ifndef SIMDHWYHASH_H_
#define SIMDHWYHASH_H_

#include <stddef.h>
#include <stdint.h>

#if defined(__GNUC__) || defined(__clang__)
#define SIMDHWYHASH_RESTRICT __restrict__
#elif defined(_MSC_VER)
#define SIMDHWYHASH_RESTRICT __restrict
#else /* !defined(__GNUC__) && !defined(__clang__) && !defined(_MSC_VER) */
#define SIMDHWYHASH_RESTRICT
#endif /* defined(__GNUC__) || defined(__clang__) */

#if !defined(SIMDHWYHASH_SHARED_DEFINE)
#define SIMDHWYHASH_DLLEXPORT
#define SIMDHWYHASH_CONTRIB_DLLEXPORT
#define SIMDHWYHASH_TEST_DLLEXPORT
#else  // !SIMDHWYHASH_SHARED_DEFINE

#ifndef SIMDHWYHASH_DLLEXPORT
#if defined(simdhwyhash_EXPORTS)
/* We are building this library */
#ifdef _WIN32
#define SIMDHWYHASH_DLLEXPORT __declspec(dllexport)
#else
#define SIMDHWYHASH_DLLEXPORT __attribute__((visibility("default")))
#endif
#else /* defined(simdhwyhash_EXPORTS) */
/* We are using this library */
#ifdef _WIN32
#define SIMDHWYHASH_DLLEXPORT __declspec(dllimport)
#else
#define SIMDHWYHASH_DLLEXPORT __attribute__((visibility("default")))
#endif
#endif /* defined(simdhwyhash_EXPORTS) */
#endif /* SIMDHWYHASH_DLLEXPORT */

#endif  // !defined(SIMDHWYHASH_SHARED_DEFINE)

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

typedef struct {
  uint64_t v0[4];
  uint64_t v1[4];
  uint64_t mul0[4];
  uint64_t mul1[4];
} SimdHwyHashState;

SIMDHWYHASH_DLLEXPORT void SimdHwyHash_Reset(
    SimdHwyHashState* SIMDHWYHASH_RESTRICT state,
    const uint64_t* SIMDHWYHASH_RESTRICT key);
SIMDHWYHASH_DLLEXPORT void SimdHwyHash_Update(
    SimdHwyHashState* SIMDHWYHASH_RESTRICT state,
    const void* SIMDHWYHASH_RESTRICT ptr, size_t byte_len);

SIMDHWYHASH_DLLEXPORT uint64_t
SimdHwyHash_Finalize64(SimdHwyHashState* SIMDHWYHASH_RESTRICT state);
SIMDHWYHASH_DLLEXPORT void SimdHwyHash_Finalize128(
    SimdHwyHashState* SIMDHWYHASH_RESTRICT state,
    uint64_t* SIMDHWYHASH_RESTRICT hash);
SIMDHWYHASH_DLLEXPORT void SimdHwyHash_Finalize256(
    SimdHwyHashState* SIMDHWYHASH_RESTRICT state,
    uint64_t* SIMDHWYHASH_RESTRICT hash);

SIMDHWYHASH_DLLEXPORT uint64_t
SimdHwyHash_Hash64(const void* SIMDHWYHASH_RESTRICT ptr, size_t byte_len,
                   const uint64_t* SIMDHWYHASH_RESTRICT key);
SIMDHWYHASH_DLLEXPORT void SimdHwyHash_Hash128(
    const void* SIMDHWYHASH_RESTRICT ptr, size_t byte_len,
    const uint64_t* SIMDHWYHASH_RESTRICT key,
    uint64_t* SIMDHWYHASH_RESTRICT hash);
SIMDHWYHASH_DLLEXPORT void SimdHwyHash_Hash256(
    const void* SIMDHWYHASH_RESTRICT ptr, size_t byte_len,
    const uint64_t* SIMDHWYHASH_RESTRICT key,
    uint64_t* SIMDHWYHASH_RESTRICT hash);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* SIMDHWYHASH_H_ */
