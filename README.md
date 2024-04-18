# simdhwyhash

simdhwyhash is an implementation of 
[HighwayHash](https://github.com/google/highwayhash) that uses the
[Google Highway](https://github.com/google/highway) SIMD library.

## Building simdhwyhash library

simdhwyhash requires a C++17 compiler capable of compiling Google Highway (such
as GCC 7 or later or Clang 6 or later) and cmake to build.

simdhwyhash also requires that git is installed to download the Google Highway
dependency if SIMDHWYHASH_SYSTEM_HIGHWAY is set to OFF.

To build simdhwyhash as a shared or static library (depending on 
BUILD_SHARED_LIBS), the standard CMake workflow can be used:
```
mkdir -p build && cd build
cmake ..
make -j && make test
```

## simdhwyhash API

simdhwyhash exposes a C API to allow simdhwyhash to be used from languages
other than C++.

- `void SimdHwyHash_Reset(SimdHwyHashState* state, const uint64_t* key)` -
initializes `state` using `key` (which is an array of 4 uint64_t values)

- `void SimdHwyHash_Update(SimdHwyHashState* state, const void* ptr, size_t
byte_len)` - updates `state` with `byte_len` bytes from `ptr`

  `state` must be initialized using `SimdHwyHash_Reset` prior to the first
  call to `SimdHwyHash_Update`.

  If `state` is going to be updated with additional data, `byte_len` should be
  a multiple of 32. Otherwise, if this is the final `SimdHwyHash_Update` step,
  `byte_len` should be equal to the length of the remaining data.

- `uint64_t SimdHwyHash_Finalize64(SimdHwyHashState* state)` - returns the
64-bit hash of the data

  `state` must be initialized using `SimdHwyHash_Reset`, followed by zero or
  more calls to `SimdHwyHash_Update`, prior to calling `SimdHwyHash_Finalize64`.

- `void SimdHwyHash_Finalize128(SimdHwyHashState* state, uint64_t* hash)` - 
returns the 128-bit hash of the data in `hash[0]` and `hash[1]`

  `state` must be initialized using `SimdHwyHash_Reset`, followed by zero or
  more calls to `SimdHwyHash_Update`, prior to calling
  `SimdHwyHash_Finalize128`.

- `void SimdHwyHash_Finalize256(SimdHwyHashState* state, uint64_t* hash)` - 
returns the 256-bit hash of the data in `hash[0]`, `hash[1]`, `hash[2]`, and
`hash[3]`

  `state` must be initialized using `SimdHwyHash_Reset`, followed by zero or
  more calls to `SimdHwyHash_Update`, prior to calling
  `SimdHwyHash_Finalize256`.

- `uint64_t SimdHwyHash_Hash64(const void* ptr, size_t byte_len, const uint64_t*
key)` - returns the 64-bit hash of `byte_len` bytes of data pointed to by `ptr`, 
hashed using `key` (which is an array of 4 uint64_t values)

  `SimdHwyHash_Hash64(ptr, byte_len, key)` is equivalent to the following:
  ```
  SimdHwyHashState state;
  SimdHwyHash_Reset(&state, key);
  SimdHwyHash_Update(&state, ptr, byte_len);
  const uint64_t hash64 = SimdHwyHash_Finalize64(&state);
  ```

- `void SimdHwyHash_Hash128(const void* ptr, size_t byte_len, const uint64_t*
key, uint64_t* hash)` - returns the 128-bit hash of `byte_len` bytes of data
pointed to by `ptr` in `hash[0]` and `hash[1]`, hashed using `key` (which is an
array of 4 uint64_t values)

  `SimdHwyHash_Hash128(ptr, byte_len, key, hash)` is equivalent to the
  following:
  ```
  SimdHwyHashState state;
  SimdHwyHash_Reset(&state, key);
  SimdHwyHash_Update(&state, ptr, byte_len);
  SimdHwyHash_Finalize128(&state, hash);
  ```

- `void SimdHwyHash_Hash256(const void* ptr, size_t byte_len, const uint64_t*
key, uint64_t* hash)` - returns the 256-bit hash of `byte_len` bytes of data
pointed to by `ptr` in `hash[0]`, `hash[1]`, `hash[2]`, and `hash[3]`, hashed
using `key` (which is an array of 4 uint64_t values)

  `SimdHwyHash_Hash256(ptr, byte_len, key, hash)` is equivalent to the
  following:
  ```
  SimdHwyHashState state;
  SimdHwyHash_Reset(&state, key);
  SimdHwyHash_Update(&state, ptr, byte_len);
  SimdHwyHash_Finalize256(&state, hash);
  ```

## simdhwyhash CMake configuration options

- BUILD_SHARED_LIBS (defaults to ON) - set to OFF to build simdhwyhash as
a static library. simdhwyhash is configured to be built as a shared library if 
BUILD_SHARED_LIBS is ON and SIMDHWYHASH_FORCE_STATIC_LIBS is OFF.

- HWY_CMAKE_ARM7 (defaults to OFF) - set to enable Armv7 Highway compilation
flags

- HWY_CMAKE_SSE2 (defaults to OFF) - set to enable SSE2 as the baseline on
32-bit x86

- HWY_CMAKE_RVV (defaults to ON) - set to enable the RISC-V "V" extension if
compiling on RISC-V

- SIMDHWYHASH_ENABLE_INSTALL (defaults to ON) - set to OFF to disable the 
installation of the simdhwyhash library

- SIMDHWYHASH_ENABLE_TESTS (defaults to ON) - set to OFF to disable the
simdhwyhash unit tests

- SIMDHWYHASH_FORCE_STATIC_LIBS (defaults to OFF) - set to ON to build
simdhwyhash as a static library, regardless of whether BUILD_SHARED_LIBS is set
to ON or OFF.

- SIMDHWYHASH_HWY_CXX_FLAGS - Additional C++ compiler flags used to compile the 
simdhwyhash library and Google Highway (if the system included Google Highway 
library is not used)

- SIMDHWYHASH_SYSTEM_HIGHWAY (defaults to OFF) - set to ON to use the system
included Google Highway library

- SIMDHWYHASH_SYSTEM_GTEST (defaults to OFF) - set to ON to use the system
included Google Test library

- SIMDHWYHASH_WARNINGS_ARE_ERRORS (defaults to OFF) - set to ON to treat 
compiler warnings as errors
