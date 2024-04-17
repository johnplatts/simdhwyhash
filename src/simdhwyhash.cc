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

#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "simdhwyhash.cc"
#include "hwy/foreach_target.h"
#include "hwy/highway.h"

// simdhwyhash is a implementation of HighwayHash that uses the Google Highway
// SIMD library

namespace simdhwyhash {

HWY_BEFORE_NAMESPACE();
namespace HWY_NAMESPACE {
namespace {

using hwy::CopyBytes;
using hwy::ZeroBytes;
using hwy::HWY_NAMESPACE::Add;
using hwy::HWY_NAMESPACE::And;
using hwy::HWY_NAMESPACE::BroadcastBlock;
using hwy::HWY_NAMESPACE::DFromV;
using hwy::HWY_NAMESPACE::FixedTag;
using hwy::HWY_NAMESPACE::GetLane;
using hwy::HWY_NAMESPACE::IfThenElse;
using hwy::HWY_NAMESPACE::IfThenElseZero;
using hwy::HWY_NAMESPACE::InsertLane;
using hwy::HWY_NAMESPACE::Load;
using hwy::HWY_NAMESPACE::LoadU;
using hwy::HWY_NAMESPACE::Min;
using hwy::HWY_NAMESPACE::Mul;
#if HWY_TARGET != HWY_SCALAR
using hwy::HWY_NAMESPACE::MulEven;
using hwy::HWY_NAMESPACE::MulOdd;
#endif
using hwy::HWY_NAMESPACE::Or;
using hwy::HWY_NAMESPACE::Per4LaneBlockShuffle;
using hwy::HWY_NAMESPACE::Repartition;
using hwy::HWY_NAMESPACE::Rol;
using hwy::HWY_NAMESPACE::RotateRight;
using hwy::HWY_NAMESPACE::ShiftLeft;
using hwy::HWY_NAMESPACE::ShiftRight;
#if HWY_TARGET != HWY_SCALAR
using hwy::HWY_NAMESPACE::ShiftLeftLanes;
#endif
using hwy::HWY_NAMESPACE::Store;
using hwy::HWY_NAMESPACE::StoreU;
using hwy::HWY_NAMESPACE::TableLookupBytes;
using hwy::HWY_NAMESPACE::TableLookupLanes;
using hwy::HWY_NAMESPACE::TFromD;
using hwy::HWY_NAMESPACE::Twice;
using hwy::HWY_NAMESPACE::Vec;
using hwy::HWY_NAMESPACE::Xor;
using hwy::HWY_NAMESPACE::Xor3;

#if HWY_TARGET == HWY_SCALAR

#if !HWY_HAVE_TUPLE
#error "HWY_HAVE_TUPLE must be 1 on the HWY_SCALAR target"
#endif

#endif  // HWY_TARGET == HWY_SCALAR

#if HWY_HAVE_TUPLE
using hwy::HWY_NAMESPACE::Create2;
using hwy::HWY_NAMESPACE::Create4;
using hwy::HWY_NAMESPACE::Get2;
using hwy::HWY_NAMESPACE::Get4;
using hwy::HWY_NAMESPACE::Vec2;
using hwy::HWY_NAMESPACE::Vec4;
#endif

template <class D>
static constexpr size_t MinVecLanes(D d) {
#if HWY_HAVE_SCALABLE
  constexpr int kPow2 = d.Pow2();

#if HWY_TARGET == HWY_RVV
  constexpr int kMaxPow2 = 3;
#else
  constexpr int kMaxPow2 = 0;
#endif

  constexpr int kClampedPow2 = HWY_MIN(HWY_MAX(kPow2, -5), kMaxPow2);
  constexpr size_t kLanesPerBlock = 16 / sizeof(TFromD<D>);

  constexpr size_t kMinLanesPerFullVec =
      (kClampedPow2 >= 0) ? (kLanesPerBlock << kClampedPow2)
                          : (kLanesPerBlock >> (-kClampedPow2));

  return HWY_MIN(HWY_MAX_LANES_D(D), kMinLanesPerFullVec);
#else
  (void)d;
  return HWY_MAX_LANES_D(D);
#endif
}

using Exactly1LaneDU64 = FixedTag<uint64_t, 1>;
#if HWY_TARGET == HWY_RVV
using AtMost2LaneDU64 = FixedTag<uint64_t, 2>;
using HighwayHashDU64 = Twice<AtMost2LaneDU64>;
#else
using AtMost2LaneDU64 = FixedTag<uint64_t, HWY_MIN(HWY_LANES(uint64_t), 2)>;
using HighwayHashDU64 = FixedTag<uint64_t, HWY_MIN(HWY_LANES(uint64_t), 4)>;
#endif

static_assert(HWY_MAX_LANES_D(HighwayHashDU64) <= 4,
              "HWY_MAX_LANES_D(HighwayHashDU64) <= 4 must be true");

#if HWY_TARGET == HWY_SCALAR
static_assert(MinVecLanes(HighwayHashDU64()) == 1,
              "MinVecLanes(HighwayHashDU64()) == 1 must be true on the "
              "HWY_SCALAR target");
static_assert(MinVecLanes(AtMost2LaneDU64()) == 1,
              "MinVecLanes(AtMost2LaneDU64()) == 1 must be true on the "
              "HWY_SCALAR target");
static_assert(HWY_MAX_LANES_D(AtMost2LaneDU64) == 1,
              "HWY_MAX_LANES_D(AtMost2LaneDU64) == 1 must be true on the "
              "HWY_SCALAR target");
#else
static_assert(MinVecLanes(HighwayHashDU64()) >= 2,
              "MinVecLanes(HighwayHashDU64()) >= 2 must be true on targets "
              "other than HWY_SCALAR");
static_assert(MinVecLanes(AtMost2LaneDU64()) == 2,
              "MinVecLanes(AtMost2LaneDU64()) == 2 must be true on targets "
              "other than HWY_SCALAR");
static_assert(HWY_MAX_LANES_D(AtMost2LaneDU64) == 2,
              "HWY_MAX_LANES_D(AtMost2LaneDU64) == 2 must be true on targets "
              "other than HWY_SCALAR");
#endif

#if !HWY_HAVE_TUPLE
static_assert(MinVecLanes(HighwayHashDU64()) >= 4,
              "MinVecLanes(HighwayHashDU64()) >= 4 must be true if "
              "HWY_HAVE_TUPLE is false");
#endif

template <class D>
static constexpr bool LoadOrStoreStateVecUsingAlignedAccess(D d) {
  constexpr size_t kMaxBytes = d.MaxBytes();
  constexpr size_t kHashStateArrAlign = HWY_MIN(alignof(SimdHwyHashState), 32);
  return (kMaxBytes <= kHashStateArrAlign);
}

template <class D>
static HWY_INLINE Vec<D> LoadStateVec(D d, const TFromD<D>* HWY_RESTRICT ptr) {
  if constexpr (LoadOrStoreStateVecUsingAlignedAccess(d)) {
    return Load(d, ptr);
  } else {
    return LoadU(d, ptr);
  }
}

template <class D>
static HWY_INLINE void StoreStateVec(Vec<D> v, D d,
                                     TFromD<D>* HWY_RESTRICT ptr) {
  if constexpr (LoadOrStoreStateVecUsingAlignedAccess(d)) {
    Store(v, d, ptr);
  } else {
    StoreU(v, d, ptr);
  }
}

static void ResetHwyHashState(SimdHwyHashState* HWY_RESTRICT state,
                              const uint64_t* HWY_RESTRICT key) {
  const HighwayHashDU64 du64;
#if HWY_TARGET != HWY_SCALAR
  const Repartition<uint32_t, decltype(du64)> du32;
#endif

  alignas(32) static constexpr uint64_t kMul0[4] = {
      0xdbe6d5d5fe4cce2fU, 0xa4093822299f31d0U, 0x13198a2e03707344U,
      0x243f6a8885a308d3U};
  alignas(32) static constexpr uint64_t kMul1[4] = {
      0x3bd39e10cb0ef593U, 0xc0acf169b5f18a8cU, 0xbe5466cf34e90c6cU,
      0x452821e638d01377U};

  const size_t lanes_per_u64_vec = Lanes(du64);

  for (size_t i = 0; i < 4; i += lanes_per_u64_vec) {
    const auto v_key = LoadU(du64, key + i);
    const auto v_mul0 = Load(du64, kMul0 + i);
    const auto v_mul1 = Load(du64, kMul1 + i);

    const auto v0 = Xor(v_key, v_mul0);

#if HWY_TARGET == HWY_SCALAR
    const auto v_key_rot32 = RotateRight<32>(v_key);
#else
    const auto v_key_rot32 =
        BitCast(du64, Reverse2(du32, BitCast(du32, v_key)));
#endif

    const auto v1 = Xor(v_key_rot32, v_mul1);

    StoreStateVec(v_mul0, du64, state->mul0 + i);
    StoreStateVec(v_mul1, du64, state->mul1 + i);
    StoreStateVec(v0, du64, state->v0 + i);
    StoreStateVec(v1, du64, state->v1 + i);
  }
}

using Exactly1LaneU64Vec = Vec<Exactly1LaneDU64>;

#if HWY_TARGET == HWY_SCALAR
using AtLeast2LaneU64Vec = Vec2<HighwayHashDU64>;
using AtLeast4LaneU64Vec = Vec4<HighwayHashDU64>;
#elif HWY_TARGET == HWY_RVV
using AtLeast2LaneU64Vec = Vec<HighwayHashDU64>;
using AtLeast4LaneU64Vec = Vec<HighwayHashDU64>;
#else
using AtLeast2LaneU64Vec = Vec<HighwayHashDU64>;
using AtLeast4LaneU64Vec = hwy::If<(MinVecLanes(HighwayHashDU64()) < 4),
                                   Vec2<HighwayHashDU64>, AtLeast2LaneU64Vec>;
#endif

#if HWY_TARGET == HWY_SCALAR
using Exactly2LaneU64Vec = AtLeast2LaneU64Vec;
#else
using Exactly2LaneU64Vec = Vec<AtMost2LaneDU64>;
#endif

#if HWY_TARGET == HWY_SCALAR
static HWY_INLINE AtLeast4LaneU64Vec
CombineToAtLeast4LaneVec(AtLeast2LaneU64Vec lo, AtLeast2LaneU64Vec hi) {
  return Create4(HighwayHashDU64(), Get2<0>(lo), Get2<1>(lo), Get2<0>(hi),
                 Get2<1>(hi));
}

static HWY_INLINE AtLeast2LaneU64Vec
GetLowerAtLeast2LaneVec(AtLeast4LaneU64Vec v) {
  return Create2(HighwayHashDU64(), Get4<0>(v), Get4<1>(v));
}

static HWY_INLINE AtLeast2LaneU64Vec
GetUpperAtLeast2LaneVec(AtLeast4LaneU64Vec v) {
  return Create2(HighwayHashDU64(), Get4<2>(v), Get4<3>(v));
}
#else  // HWY_TARGET != HWY_SCALAR
template <class VFrom,
          hwy::EnableIf<
              (hwy::IsSame<hwy::RemoveCvRef<VFrom>, AtLeast2LaneU64Vec>() &&
               hwy::IsSame<hwy::RemoveCvRef<VFrom>, AtLeast4LaneU64Vec>())>* =
              nullptr>
static HWY_INLINE AtLeast4LaneU64Vec CombineToAtLeast4LaneVec(VFrom lo,
                                                              VFrom /*hi*/) {
  return lo;
}

template <class VFrom,
          hwy::EnableIf<
              (hwy::IsSame<hwy::RemoveCvRef<VFrom>, AtLeast2LaneU64Vec>() &&
               !hwy::IsSame<hwy::RemoveCvRef<VFrom>, AtLeast4LaneU64Vec>())>* =
              nullptr>
static HWY_INLINE AtLeast4LaneU64Vec CombineToAtLeast4LaneVec(VFrom lo,
                                                              VFrom hi) {
  return Create2(HighwayHashDU64(), lo, hi);
}

template <class VFrom,
          hwy::EnableIf<
              (hwy::IsSame<hwy::RemoveCvRef<VFrom>, AtLeast2LaneU64Vec>() &&
               hwy::IsSame<hwy::RemoveCvRef<VFrom>, AtLeast4LaneU64Vec>())>* =
              nullptr>
static HWY_INLINE AtLeast2LaneU64Vec GetLowerAtLeast2LaneVec(VFrom v) {
  return v;
}

template <class VFrom,
          hwy::EnableIf<
              (hwy::IsSame<hwy::RemoveCvRef<VFrom>, AtLeast2LaneU64Vec>() &&
               hwy::IsSame<hwy::RemoveCvRef<VFrom>, AtLeast4LaneU64Vec>())>* =
              nullptr>
static HWY_INLINE AtLeast2LaneU64Vec GetUpperAtLeast2LaneVec(VFrom v) {
  return v;
}

#if HWY_HAVE_TUPLE
template <class VFrom,
          hwy::EnableIf<
              (!hwy::IsSame<hwy::RemoveCvRef<VFrom>, AtLeast2LaneU64Vec>() &&
               hwy::IsSame<hwy::RemoveCvRef<VFrom>, AtLeast4LaneU64Vec>())>* =
              nullptr>
static HWY_INLINE AtLeast2LaneU64Vec GetLowerAtLeast2LaneVec(VFrom v) {
  return Get2<0>(v);
}

template <class VFrom,
          hwy::EnableIf<
              (!hwy::IsSame<hwy::RemoveCvRef<VFrom>, AtLeast2LaneU64Vec>() &&
               hwy::IsSame<hwy::RemoveCvRef<VFrom>, AtLeast4LaneU64Vec>())>* =
              nullptr>
static HWY_INLINE AtLeast2LaneU64Vec GetUpperAtLeast2LaneVec(VFrom v) {
  return Get2<1>(v);
}
#endif  // HWY_HAVE_TUPLE
#endif  // HWY_TARGET == HWY_SCALAR

static HWY_INLINE AtLeast2LaneU64Vec
LoadAtLeast2LanePacketVec(const uint8_t* packet) {
  const HighwayHashDU64 du64;
  using VU64 = Vec<decltype(du64)>;

#if HWY_TARGET == HWY_SCALAR
  uint64_t lo;
  uint64_t hi;

  CopyBytes(packet, &lo, sizeof(uint64_t));
  CopyBytes(packet + sizeof(uint64_t), &hi, sizeof(uint64_t));

  VU64 v_lo = Set(du64, lo);
  VU64 v_hi = Set(du64, hi);

#if HWY_IS_BIG_ENDIAN
  v_lo = ReverseLaneBytes(v_lo);
  v_hi = ReverseLaneBytes(v_hi);
#endif

  return Create2(du64, v_lo, v_hi);
#else
  const Repartition<uint8_t, decltype(du64)> du8;
  VU64 vec = BitCast(du64, LoadU(du8, packet));

#if HWY_IS_BIG_ENDIAN
  vec = ReverseLaneBytes(vec);
#endif

  return vec;
#endif
}

static HWY_INLINE AtLeast4LaneU64Vec LoadAtLeast4LanePacketVec(
    const size_t lanes_per_u64_vec, const uint8_t* packet) {
  const auto v_lo = LoadAtLeast2LanePacketVec(packet);
  auto v_hi =
      (lanes_per_u64_vec >= 4) ? v_lo : LoadAtLeast2LanePacketVec(packet + 16);

  return CombineToAtLeast4LaneVec(v_lo, v_hi);
}

static HWY_INLINE AtLeast2LaneU64Vec
LoadAtLeast2LaneStateVec(const uint64_t* HWY_RESTRICT ptr) {
  const HighwayHashDU64 du64;

#if HWY_TARGET == HWY_SCALAR
  const auto v0 = LoadStateVec(du64, ptr);
  const auto v1 = LoadStateVec(du64, ptr + 1);
  return Create2(du64, v0, v1);
#else
  return LoadStateVec(du64, ptr);
#endif
}

static HWY_INLINE AtLeast4LaneU64Vec LoadAtLeast4LaneStateVec(
    const size_t lanes_per_u64_vec, const uint64_t* HWY_RESTRICT ptr) {
  const auto v_lo = LoadAtLeast2LaneStateVec(ptr);
  auto v_hi =
      (lanes_per_u64_vec >= 4) ? v_lo : LoadAtLeast2LaneStateVec(ptr + 2);

  return CombineToAtLeast4LaneVec(v_lo, v_hi);
}

static HWY_INLINE void StoreAtLeast2LaneStateVec(AtLeast2LaneU64Vec v,
                                                 uint64_t* HWY_RESTRICT ptr) {
  const HighwayHashDU64 du64;

#if HWY_TARGET == HWY_SCALAR
  StoreStateVec(Get2<0>(v), du64, ptr);
  StoreStateVec(Get2<1>(v), du64, ptr + 1);
#else
  StoreStateVec(v, du64, ptr);
#endif
}

static HWY_INLINE void StoreAtLeast4LaneStateVec(const size_t lanes_per_u64_vec,
                                                 AtLeast4LaneU64Vec v,
                                                 uint64_t* HWY_RESTRICT ptr) {
  StoreAtLeast2LaneStateVec(GetLowerAtLeast2LaneVec(v), ptr);
  if (lanes_per_u64_vec < 4) {
    StoreAtLeast2LaneStateVec(GetUpperAtLeast2LaneVec(v), ptr + 2);
  }
}

template <class DU64, class V,
          hwy::EnableIf<hwy::IsSame<hwy::RemoveRef<V>, Vec<DU64>>()>* = nullptr>
static HWY_INLINE void HwyHashUpdateStep1(DU64 du64, V& v0, V& v1, V& mul0,
                                          V& mul1, const V a) {
  v1 = Add(v1, Add(mul0, a));

#if HWY_TARGET == HWY_SCALAR || HWY_TARGET == HWY_EMU128
  const auto lo32_mask = Set(du64, uint64_t{0xffffffffU});
  const auto p_v1_v0 = Mul(And(v1, lo32_mask), ShiftRight<32>(v0));
#else
  const Repartition<uint32_t, decltype(du64)> du32;
#if HWY_IS_BIG_ENDIAN
  const auto p_v1_v0 =
      MulOdd(BitCast(du32, v1), Reverse2(du32, BitCast(du32, v0)));
#else
  const auto p_v1_v0 =
      MulEven(BitCast(du32, v1), Reverse2(du32, BitCast(du32, v0)));
#endif  // HWY_IS_BIG_ENDIAN
#endif  // HWY_TARGET == HWY_SCALAR || HWY_TARGET == HWY_EMU128

  mul0 = Xor(mul0, p_v1_v0);
  v0 = Add(v0, mul1);

#if HWY_TARGET == HWY_SCALAR || HWY_TARGET == HWY_EMU128
  const auto p_v0_v1 = Mul(And(v0, lo32_mask), ShiftRight<32>(v1));
#else
#if HWY_IS_BIG_ENDIAN
  const auto p_v0_v1 =
      MulOdd(BitCast(du32, v0), Reverse2(du32, BitCast(du32, v1)));
#else
  const auto p_v0_v1 =
      MulEven(BitCast(du32, v0), Reverse2(du32, BitCast(du32, v1)));
#endif  // HWY_IS_BIG_ENDIAN
#endif  // HWY_TARGET == HWY_SCALAR || HWY_TARGET == HWY_EMU128

  mul1 = Xor(mul1, p_v0_v1);
}

#if HWY_TARGET == HWY_SCALAR
template <
    class DU64, class V,
    hwy::EnableIf<hwy::IsSame<hwy::RemoveRef<V>, Vec2<DU64>>()>* = nullptr>
static HWY_INLINE void HwyHashUpdateStep1(DU64 du64, V& v0, V& v1, V& mul0,
                                          V& mul1, const V a) {
  using VU64 = Vec<DU64>;

  VU64 v0_lo = Get2<0>(v0);
  VU64 v1_lo = Get2<0>(v1);
  VU64 mul0_lo = Get2<0>(mul0);
  VU64 mul1_lo = Get2<0>(mul1);
  const VU64 a_lo = Get2<0>(a);

  VU64 v0_hi = Get2<1>(v0);
  VU64 v1_hi = Get2<1>(v1);
  VU64 mul0_hi = Get2<1>(mul0);
  VU64 mul1_hi = Get2<1>(mul1);
  const VU64 a_hi = Get2<1>(a);

  HwyHashUpdateStep1(du64, v0_lo, v1_lo, mul0_lo, mul1_lo, a_lo);
  HwyHashUpdateStep1(du64, v0_hi, v1_hi, mul0_hi, mul1_hi, a_hi);

  v0 = Create2(du64, v0_lo, v0_hi);
  v1 = Create2(du64, v1_lo, v1_hi);
  mul0 = Create2(du64, mul0_lo, mul0_hi);
  mul1 = Create2(du64, mul1_lo, mul1_hi);
}
#endif  // HWY_TARGET == HWY_SCALAR

static HWY_INLINE AtLeast2LaneU64Vec
AtLeast2LaneU64VecAdd(AtLeast2LaneU64Vec a, AtLeast2LaneU64Vec b) {
#if HWY_TARGET == HWY_SCALAR
  return Create2(HighwayHashDU64(), Add(Get2<0>(a), Get2<0>(b)),
                 Add(Get2<1>(a), Get2<1>(b)));
#else
  return Add(a, b);
#endif
}

static HWY_INLINE AtLeast2LaneU64Vec
AtLeast2LaneU64VecRolU32(AtLeast2LaneU64Vec a, AtLeast2LaneU64Vec b) {
  const HighwayHashDU64 du64;

#if HWY_TARGET == HWY_SCALAR
  const int shl_amt =
      static_cast<int>(GetLane(Get2<0>(b)) & hwy::LimitsMax<uint32_t>());
  const int shr_amt = (-shl_amt) & 31;

  const uint64_t a0 = GetLane(Get2<0>(a));
  const uint64_t a1 = GetLane(Get2<1>(a));

  const uint32_t a0L = static_cast<uint32_t>(a0);
  const uint32_t a0H = static_cast<uint32_t>(a0 >> 32);
  const uint32_t a1L = static_cast<uint32_t>(a1);
  const uint32_t a1H = static_cast<uint32_t>(a1 >> 32);

  const uint32_t rot_a0L =
      static_cast<uint32_t>((a0L << shl_amt) | (a0L >> shr_amt));
  const uint32_t rot_a0H =
      static_cast<uint32_t>((a0H << shl_amt) | (a0H >> shr_amt));
  const uint32_t rot_a1L =
      static_cast<uint32_t>((a1L << shl_amt) | (a1L >> shr_amt));
  const uint32_t rot_a1H =
      static_cast<uint32_t>((a1H << shl_amt) | (a1H >> shr_amt));

  const uint64_t rot_a0 = static_cast<uint64_t>(
      (static_cast<uint64_t>(rot_a0H) << 32) | static_cast<uint64_t>(rot_a0L));
  const uint64_t rot_a1 = static_cast<uint64_t>(
      (static_cast<uint64_t>(rot_a1H) << 32) | static_cast<uint64_t>(rot_a1L));

  return Create2(du64, Set(du64, rot_a0), Set(du64, rot_a1));
#else  // HWY_TARGET != HWY_SCALAR
  const Repartition<uint32_t, decltype(du64)> du32;

#if (HWY_TARGET > HWY_AVX2 && HWY_TARGET <= HWY_SSE2) || \
    HWY_TARGET == HWY_EMU128
  const int shl_amt = static_cast<int>(GetLane(b) & hwy::LimitsMax<uint32_t>());
  return BitCast(du64, RotateLeftSame(BitCast(du32, a), shl_amt));
#else
  return BitCast(du64, Rol(BitCast(du32, a), BitCast(du32, b)));
#endif  // (HWY_TARGET > HWY_AVX2 && HWY_TARGET <= HWY_SSE2) ||
        // HWY_TARGET == HWY_EMU128
#endif  // HWY_TARGET == HWY_SCALAR
}

template <
    class V,
    hwy::EnableIf<(hwy::IsSame<hwy::RemoveCvRef<V>, AtLeast4LaneU64Vec>() &&
                   hwy::IsSame<hwy::RemoveCvRef<V>, AtLeast2LaneU64Vec>())>* =
        nullptr>
static HWY_INLINE V AtLeast4LaneU64VecAdd(V a, V b) {
  return AtLeast2LaneU64VecAdd(a, b);
}

template <
    class V,
    hwy::EnableIf<(hwy::IsSame<hwy::RemoveCvRef<V>, AtLeast4LaneU64Vec>() &&
                   !hwy::IsSame<hwy::RemoveCvRef<V>, AtLeast2LaneU64Vec>())>* =
        nullptr>
static HWY_INLINE V AtLeast4LaneU64VecAdd(V a, V b) {
  const auto sum_lo = AtLeast2LaneU64VecAdd(GetLowerAtLeast2LaneVec(a),
                                            GetLowerAtLeast2LaneVec(b));
  const auto sum_hi = AtLeast2LaneU64VecAdd(GetUpperAtLeast2LaneVec(a),
                                            GetUpperAtLeast2LaneVec(b));
  return CombineToAtLeast4LaneVec(sum_lo, sum_hi);
}

template <
    class V,
    hwy::EnableIf<(hwy::IsSame<hwy::RemoveCvRef<V>, AtLeast4LaneU64Vec>() &&
                   hwy::IsSame<hwy::RemoveCvRef<V>, AtLeast2LaneU64Vec>())>* =
        nullptr>
static HWY_INLINE V AtLeast4LaneU64VecRol32(V a, V b) {
  return AtLeast2LaneU64VecRolU32(a, b);
}

template <
    class V,
    hwy::EnableIf<(hwy::IsSame<hwy::RemoveCvRef<V>, AtLeast4LaneU64Vec>() &&
                   !hwy::IsSame<hwy::RemoveCvRef<V>, AtLeast2LaneU64Vec>())>* =
        nullptr>
static HWY_INLINE V AtLeast4LaneU64VecRol32(V a, V b) {
  const auto result_lo = AtLeast2LaneU64VecRolU32(GetLowerAtLeast2LaneVec(a),
                                                  GetLowerAtLeast2LaneVec(b));
  const auto result_hi = AtLeast2LaneU64VecRolU32(GetUpperAtLeast2LaneVec(a),
                                                  GetUpperAtLeast2LaneVec(b));
  return CombineToAtLeast4LaneVec(result_lo, result_hi);
}

static HWY_INLINE AtLeast2LaneU64Vec ZipperMerge(AtLeast2LaneU64Vec v) {
  const HighwayHashDU64 du64;

#if HWY_TARGET == HWY_SCALAR
  const auto v0 = Get2<0>(v);
  const auto v1 = Get2<1>(v);

  const auto r0 =
      Or(Or3(ShiftRight<24>(OrAnd(And(v0, Set(du64, 0xff000000U)), v1,
                                  Set(du64, 0xff00000000U))),
             ShiftRight<16>(OrAnd(And(v0, Set(du64, 0xff0000000000U)), v1,
                                  Set(du64, 0xff000000000000U))),
             And(v0, Set(du64, 0xff0000U))),
         Or3(ShiftLeft<32>(And(v0, Set(du64, 0xff00U))),
             ShiftRight<8>(And(v1, Set(du64, 0xff00000000000000U))),
             ShiftLeft<56>(v0)));
  const auto r1 = Or3(Or3(ShiftRight<24>(OrAnd(And(v1, Set(du64, 0xff000000U)),
                                               v0, Set(du64, 0xff00000000U))),
                          And(v1, Set(du64, 0xff0000U)),
                          ShiftRight<16>(And(v1, Set(du64, 0xff0000000000U)))),
                      Or3(ShiftLeft<24>(And(v1, Set(du64, 0xff00U))),
                          ShiftRight<8>(And(v0, Set(du64, 0xff000000000000U))),
                          ShiftLeft<48>(And(v1, Set(du64, 0xffU)))),
                      And(v0, Set(du64, 0xff00000000000000U)));

  return Create2(du64, r0, r1);
#else  // HWY_TARGET != HWY_SCALAR

#if HWY_IS_BIG_ENDIAN
  const auto shuf_idx =
      Dup128VecFromValues(du64, 0x0708060902050b04U, 0x000f010e0a0d030cU);
#else
  const auto shuf_idx =
      Dup128VecFromValues(du64, 0x000f010e05020c03U, 0x070806090d0a040bU);
#endif

  return TableLookupBytes(v, shuf_idx);
#endif  // HWY_TARGET == HWY_SCALAR
}

static HWY_INLINE void DoHwyHashUpdate(AtLeast2LaneU64Vec& v0,
                                       AtLeast2LaneU64Vec& v1,
                                       AtLeast2LaneU64Vec& mul0,
                                       AtLeast2LaneU64Vec& mul1,
                                       const AtLeast2LaneU64Vec a) {
  HwyHashUpdateStep1(HighwayHashDU64(), v0, v1, mul0, mul1, a);
  v0 = AtLeast2LaneU64VecAdd(v0, ZipperMerge(v1));
  v1 = AtLeast2LaneU64VecAdd(v1, ZipperMerge(v0));
}

template <class V,
          hwy::EnableIf<(hwy::IsSame<V, AtLeast4LaneU64Vec>() &&
                         !hwy::IsSame<V, AtLeast2LaneU64Vec>())>* = nullptr>
static HWY_INLINE void DoHwyHashUpdate(V& v0, V& v1, V& mul0, V& mul1,
                                       const V a) {
  AtLeast2LaneU64Vec v0_lo = GetLowerAtLeast2LaneVec(v0);
  AtLeast2LaneU64Vec v1_lo = GetLowerAtLeast2LaneVec(v1);
  AtLeast2LaneU64Vec mul0_lo = GetLowerAtLeast2LaneVec(mul0);
  AtLeast2LaneU64Vec mul1_lo = GetLowerAtLeast2LaneVec(mul1);

  AtLeast2LaneU64Vec v0_hi = GetUpperAtLeast2LaneVec(v0);
  AtLeast2LaneU64Vec v1_hi = GetUpperAtLeast2LaneVec(v1);
  AtLeast2LaneU64Vec mul0_hi = GetUpperAtLeast2LaneVec(mul0);
  AtLeast2LaneU64Vec mul1_hi = GetUpperAtLeast2LaneVec(mul1);

  DoHwyHashUpdate(v0_lo, v1_lo, mul0_lo, mul1_lo, GetLowerAtLeast2LaneVec(a));
  DoHwyHashUpdate(v0_hi, v1_hi, mul0_hi, mul1_hi, GetUpperAtLeast2LaneVec(a));

  v0 = CombineToAtLeast4LaneVec(v0_lo, v0_hi);
  v1 = CombineToAtLeast4LaneVec(v1_lo, v1_hi);
  mul0 = CombineToAtLeast4LaneVec(mul0_lo, mul0_hi);
  mul1 = CombineToAtLeast4LaneVec(mul1_lo, mul1_hi);
}

static HWY_INLINE AtLeast4LaneU64Vec LoadRemainderPacket(
    const size_t lanes_per_u64_vec, const uint8_t* HWY_RESTRICT ptr,
    const unsigned remainder_len) {
  const unsigned u32_load_byte_len = remainder_len & (~3u);

#if HWY_TARGET == HWY_SCALAR
  uint8_t packet_bytes[32];
  ZeroBytes(packet_bytes, 32);
  CopyBytes(ptr, packet_bytes, u32_load_byte_len);

  if (remainder_len >= 16) {
    uint32_t trailing_bytes;
    CopyBytes(ptr + remainder_len - sizeof(uint32_t), &trailing_bytes,
              sizeof(uint32_t));
    CopyBytes(&trailing_bytes, packet_bytes + 32 - sizeof(uint32_t),
              sizeof(uint32_t));
  } else {
    unsigned trailing3_len = remainder_len & 3u;
    if (trailing3_len != 0) {
      packet_bytes[16] = ptr[u32_load_byte_len];
      packet_bytes[17] = ptr[u32_load_byte_len + (trailing3_len >> 1)];
      packet_bytes[18] = ptr[u32_load_byte_len + trailing3_len - 1];
    }
  }

  return LoadAtLeast4LanePacketVec(lanes_per_u64_vec, packet_bytes);
#else  // HWY_TARGET == HWY_SCALAR
  const HighwayHashDU64 du64;
  using VU64 = Vec<decltype(du64)>;
  const Repartition<uint8_t, decltype(du64)> du8;
  const Repartition<uint32_t, decltype(du64)> du32;

  const size_t lanes_per_u8_vec = lanes_per_u64_vec * sizeof(uint64_t);

#if HWY_MAX_BYTES >= 32 && (!HWY_HAVE_SCALABLE || HWY_TARGET == HWY_RVV)
  (void)lanes_per_u8_vec;
  auto v_lo = BitCast(du32, LoadN(du8, ptr, u32_load_byte_len));
#else
  const size_t lo_u32_load_byte_len =
      HWY_MIN(u32_load_byte_len, lanes_per_u8_vec);
  const size_t hi_u32_load_byte_len = u32_load_byte_len - lo_u32_load_byte_len;

  auto v_lo = BitCast(du32, LoadN(du8, ptr, lo_u32_load_byte_len));
  auto v_hi = BitCast(
      du32, LoadN(du8, ptr + lo_u32_load_byte_len, hi_u32_load_byte_len));
#endif

  if (remainder_len >= 16) {
    uint32_t trailing_bytes;
    CopyBytes(ptr + remainder_len - sizeof(uint32_t), &trailing_bytes,
              sizeof(uint32_t));

#if HWY_MAX_BYTES >= 32
#if HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV
    if (lanes_per_u8_vec >= 32) {
#endif  // HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV
      v_lo = InsertLane(v_lo, 7, trailing_bytes);
#if HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV
    } else
#endif  // HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV
#endif  // HWY_MAX_BYTES >= 32
#if HWY_MAX_BYTES < 32 || (HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV)
    {
      v_hi = InsertLane(v_hi, 3, trailing_bytes);
    }
#endif  // HWY_MAX_BYTES < 32 || (HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV)
  } else {
    unsigned trailing3_len = remainder_len & 3u;
    if (trailing3_len != 0) {
      uint8_t packet_b16 = ptr[u32_load_byte_len];
      uint8_t packet_b17 = ptr[u32_load_byte_len + (trailing3_len >> 1)];
      uint8_t packet_b18 = ptr[u32_load_byte_len + trailing3_len - 1];

#if HWY_IS_BIG_ENDIAN
      const uint32_t trailing_bytes =
          (static_cast<uint32_t>(packet_b16) << 24) |
          (static_cast<uint32_t>(packet_b17) << 16) |
          (static_cast<uint32_t>(packet_b18) << 8);
#else
      const uint32_t trailing_bytes = static_cast<uint32_t>(packet_b16) |
                                      (static_cast<uint32_t>(packet_b17) << 8) |
                                      (static_cast<uint32_t>(packet_b18) << 16);
#endif

#if HWY_MAX_BYTES >= 32
#if HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV
      if (lanes_per_u8_vec >= 32) {
#endif  // HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV
        v_lo = InsertLane(v_lo, 4, trailing_bytes);
#if HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV
      } else
#endif  // HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV
#endif  // HWY_MAX_BYTES >= 32
#if HWY_MAX_BYTES < 32 || (HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV)
      {
        v_hi = InsertLane(v_hi, 0, trailing_bytes);
      }
#endif  // HWY_MAX_BYTES < 32 || (HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV)
    }
  }

  VU64 v64_lo = BitCast(du64, v_lo);
#if HWY_MAX_BYTES < 32 || (HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV)
  VU64 v64_hi = BitCast(du64, v_hi);
#endif  // HWY_MAX_BYTES < 32 || (HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV)

#if HWY_IS_BIG_ENDIAN
  v64_lo = ReverseLaneBytes(v64_lo);
#if HWY_MAX_BYTES < 32 || (HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV)
  v64_hi = ReverseLaneBytes(v64_hi);
#endif  // HWY_MAX_BYTES < 32 || (HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV)
#endif  // HWY_IS_BIG_ENDIAN

#if HWY_MAX_BYTES >= 32 && (!HWY_HAVE_SCALABLE || HWY_TARGET == HWY_RVV)
  return CombineToAtLeast4LaneVec(v64_lo, v64_lo);
#else
  return CombineToAtLeast4LaneVec(v64_lo, v64_hi);
#endif  // HWY_MAX_BYTES >= 32 && (!HWY_HAVE_SCALABLE || HWY_TARGET == HWY_RVV)
#endif  // HWY_TARGET == HWY_SCALAR
}

static HWY_INLINE void UpdateHwyHashState(SimdHwyHashState* HWY_RESTRICT state,
                                          const uint8_t* HWY_RESTRICT ptr,
                                          size_t byte_len) {
  const HighwayHashDU64 du64;
#if HWY_TARGET != HWY_SCALAR
  const Repartition<uint32_t, decltype(du64)> du32;
#endif

  const size_t lanes_per_u64_vec = Lanes(du64);
  AtLeast4LaneU64Vec v0 =
      LoadAtLeast4LaneStateVec(lanes_per_u64_vec, state->v0);
  AtLeast4LaneU64Vec v1 =
      LoadAtLeast4LaneStateVec(lanes_per_u64_vec, state->v1);
  AtLeast4LaneU64Vec mul0 =
      LoadAtLeast4LaneStateVec(lanes_per_u64_vec, state->mul0);
  AtLeast4LaneU64Vec mul1 =
      LoadAtLeast4LaneStateVec(lanes_per_u64_vec, state->mul1);

  const uint8_t* full32_end_ptr = ptr + (byte_len & static_cast<size_t>(-32));
  for (; ptr != full32_end_ptr; ptr += 32) {
    const auto a = LoadAtLeast4LanePacketVec(lanes_per_u64_vec, ptr);
    DoHwyHashUpdate(v0, v1, mul0, mul1, a);
  }

  const unsigned remainder_len = static_cast<unsigned>(byte_len & 31u);
  if (remainder_len != 0) {
#if HWY_TARGET != HWY_SCALAR
    const AtLeast2LaneU64Vec vu64_len_x2 =
        BitCast(du64, Set(du32, static_cast<uint32_t>(remainder_len)));
#else
    const Vec<HighwayHashDU64> vu64_len_x1 =
        Set(du64,
            static_cast<uint64_t>((static_cast<uint64_t>(remainder_len) << 32) |
                                  remainder_len));
    const AtLeast2LaneU64Vec vu64_len_x2 =
        Create2(du64, vu64_len_x1, vu64_len_x1);
#endif
    const AtLeast4LaneU64Vec vu64_len_x4 =
        CombineToAtLeast4LaneVec(vu64_len_x2, vu64_len_x2);

    v0 = AtLeast4LaneU64VecAdd(v0, vu64_len_x4);
    v1 = AtLeast4LaneU64VecRol32(v1, vu64_len_x4);

    const auto a = LoadRemainderPacket(lanes_per_u64_vec, ptr, remainder_len);
    DoHwyHashUpdate(v0, v1, mul0, mul1, a);
  }

  StoreAtLeast4LaneStateVec(lanes_per_u64_vec, v0, state->v0);
  StoreAtLeast4LaneStateVec(lanes_per_u64_vec, v1, state->v1);
  StoreAtLeast4LaneStateVec(lanes_per_u64_vec, mul0, state->mul0);
  StoreAtLeast4LaneStateVec(lanes_per_u64_vec, mul1, state->mul1);
}

#if HWY_TARGET == HWY_SCALAR
static HWY_INLINE AtLeast4LaneU64Vec PermuteV0(AtLeast4LaneU64Vec& v0) {
  return Create4(HighwayHashDU64(), RotateRight<32>(Get4<2>(v0)),
                 RotateRight<32>(Get4<3>(v0)), RotateRight<32>(Get4<0>(v0)),
                 RotateRight<32>(Get4<1>(v0)));
}
#else
template <class V,
          hwy::EnableIf<hwy::IsSame<hwy::RemoveRef<V>, AtLeast4LaneU64Vec>()>* =
              nullptr>
static HWY_INLINE hwy::RemoveRef<V> PermuteV0(V& v0) {
  const HighwayHashDU64 du64;
  const Repartition<uint32_t, decltype(du64)> du32;

#if HWY_TARGET != HWY_RVV
  if constexpr (hwy::IsSame<hwy::RemoveRef<V>, Vec2<HighwayHashDU64>>()) {
    auto v0_01 = BitCast(du64, Reverse2(du32, BitCast(du32, Get2<0>(v0))));
    auto v0_23 = BitCast(du64, Reverse2(du32, BitCast(du32, Get2<1>(v0))));

#if HWY_HAVE_SCALABLE
    if (Lanes(du64) >= 4) {
      v0_23 = Per4LaneBlockShuffle<1, 0, 3, 2>(v0_01);
    }
#endif

    return Create2(du64, v0_23, v0_01);
  } else
#endif
  {
    return Per4LaneBlockShuffle<1, 0, 3, 2>(
        BitCast(du64, Reverse2(du32, BitCast(du32, v0))));
  }
}
#endif

static HWY_INLINE void PermuteAndUpdate(AtLeast4LaneU64Vec& v0,
                                        AtLeast4LaneU64Vec& v1,
                                        AtLeast4LaneU64Vec& mul0,
                                        AtLeast4LaneU64Vec& mul1) {
  const auto permuted_v0 = PermuteV0(v0);
  DoHwyHashUpdate(v0, v1, mul0, mul1, permuted_v0);
}

template <class V, hwy::EnableIf<hwy::IsSame<hwy::RemoveCvRef<V>,
                                             AtLeast4LaneU64Vec>()>* = nullptr>
static HWY_INLINE Exactly2LaneU64Vec GetLowerExactly2LaneU64Vec(V v) {
#if HWY_MAX_BYTES < 32
  return GetLowerAtLeast2LaneVec(v);
#else  // HWY_MAX_BYTES >= 32
  const AtMost2LaneDU64 du64;

#if HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV
  if constexpr (hwy::IsSame<hwy::RemoveCvRef<V>, Vec2<HighwayHashDU64>>()) {
    return ResizeBitCast(du64, Get2<0>(v));
  } else
#endif  // HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV
  {
    return ResizeBitCast(du64, v);
  }
#endif  // HWY_MAX_BYTES < 32
}

template <class V, hwy::EnableIf<hwy::IsSame<hwy::RemoveCvRef<V>,
                                             AtLeast4LaneU64Vec>()>* = nullptr>
static HWY_INLINE Exactly2LaneU64Vec GetUpperExactly2LaneU64Vec(V v) {
#if HWY_MAX_BYTES < 32
  return GetUpperAtLeast2LaneVec(v);
#else  // HWY_MAX_BYTES >= 32
#if HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV
  if constexpr (hwy::IsSame<hwy::RemoveCvRef<V>, Vec2<HighwayHashDU64>>()) {
    auto v_lo = Get2<0>(v);
    auto v_hi = Get2<1>(v);
    if (Lanes(HighwayHashDU64()) >= 4) {
      v_hi = SlideDownLanes(DFromV<decltype(v_lo)>(), v_lo, 2);
    }

    return v_hi;
  } else
#endif  // HWY_HAVE_SCALABLE && HWY_TARGET != HWY_RVV
  {
    return UpperHalf(AtMost2LaneDU64(), v);
  }
#endif  // HWY_MAX_BYTES < 32
}

template <class V, hwy::EnableIf<hwy::IsSame<hwy::RemoveCvRef<V>,
                                             Exactly2LaneU64Vec>()>* = nullptr>
static HWY_INLINE Exactly1LaneU64Vec GetLowerExactly1LaneU64Vec(V v) {
#if HWY_TARGET == HWY_SCALAR
  return Get2<0>(v);
#else
  return ResizeBitCast(Exactly1LaneDU64(), v);
#endif
}

template <
    class V,
    hwy::EnableIf<(!hwy::IsSame<hwy::RemoveCvRef<V>, Exactly2LaneU64Vec>() &&
                   hwy::IsSame<hwy::RemoveCvRef<V>, AtLeast4LaneU64Vec>())>* =
        nullptr>
static HWY_INLINE Exactly1LaneU64Vec GetLowerExactly1LaneU64Vec(V v) {
  return GetLowerExactly1LaneU64Vec(GetLowerExactly2LaneU64Vec(v));
}

static uint64_t Finalize64(SimdHwyHashState* HWY_RESTRICT state) {
  const size_t lanes_per_u64_vec = Lanes(HighwayHashDU64());
  AtLeast4LaneU64Vec v0 =
      LoadAtLeast4LaneStateVec(lanes_per_u64_vec, state->v0);
  AtLeast4LaneU64Vec v1 =
      LoadAtLeast4LaneStateVec(lanes_per_u64_vec, state->v1);
  AtLeast4LaneU64Vec mul0 =
      LoadAtLeast4LaneStateVec(lanes_per_u64_vec, state->mul0);
  AtLeast4LaneU64Vec mul1 =
      LoadAtLeast4LaneStateVec(lanes_per_u64_vec, state->mul1);

  PermuteAndUpdate(v0, v1, mul0, mul1);
  PermuteAndUpdate(v0, v1, mul0, mul1);
  PermuteAndUpdate(v0, v1, mul0, mul1);
  PermuteAndUpdate(v0, v1, mul0, mul1);

  return GetLane(Add(
      Add(GetLowerExactly1LaneU64Vec(v0), GetLowerExactly1LaneU64Vec(v1)),
      Add(GetLowerExactly1LaneU64Vec(mul0), GetLowerExactly1LaneU64Vec(mul1))));
}

static HWY_INLINE Exactly2LaneU64Vec
Exactly2LaneU64VecAdd(Exactly2LaneU64Vec a, Exactly2LaneU64Vec b) {
#if HWY_TARGET == HWY_SCALAR
  return AtLeast2LaneU64VecAdd(a, b);
#else
  return Add(a, b);
#endif
}

static void Finalize128(SimdHwyHashState* HWY_RESTRICT state,
                        uint64_t* HWY_RESTRICT hash) {
  const size_t lanes_per_u64_vec = Lanes(HighwayHashDU64());
  AtLeast4LaneU64Vec v0 =
      LoadAtLeast4LaneStateVec(lanes_per_u64_vec, state->v0);
  AtLeast4LaneU64Vec v1 =
      LoadAtLeast4LaneStateVec(lanes_per_u64_vec, state->v1);
  AtLeast4LaneU64Vec mul0 =
      LoadAtLeast4LaneStateVec(lanes_per_u64_vec, state->mul0);
  AtLeast4LaneU64Vec mul1 =
      LoadAtLeast4LaneStateVec(lanes_per_u64_vec, state->mul1);

  PermuteAndUpdate(v0, v1, mul0, mul1);
  PermuteAndUpdate(v0, v1, mul0, mul1);
  PermuteAndUpdate(v0, v1, mul0, mul1);
  PermuteAndUpdate(v0, v1, mul0, mul1);
  PermuteAndUpdate(v0, v1, mul0, mul1);
  PermuteAndUpdate(v0, v1, mul0, mul1);

  const Exactly2LaneU64Vec v0_lo = GetLowerExactly2LaneU64Vec(v0);
  const Exactly2LaneU64Vec v1_hi = GetUpperExactly2LaneU64Vec(v1);
  const Exactly2LaneU64Vec mul0_lo = GetLowerExactly2LaneU64Vec(mul0);
  const Exactly2LaneU64Vec mul1_hi = GetUpperExactly2LaneU64Vec(mul1);

  const Exactly2LaneU64Vec result =
      Exactly2LaneU64VecAdd(Exactly2LaneU64VecAdd(v0_lo, mul0_lo),
                            Exactly2LaneU64VecAdd(v1_hi, mul1_hi));

  const AtMost2LaneDU64 du64;
#if HWY_TARGET == HWY_SCALAR
  Store(Get2<0>(result), du64, hash);
  Store(Get2<1>(result), du64, hash + 1);
#else
  StoreU(result, du64, hash);
#endif
}

#if HWY_TARGET == HWY_SCALAR
static HWY_INLINE AtLeast4LaneU64Vec AtLeast4LaneU64VecDup128(uint64_t val0,
                                                              uint64_t val1) {
  const HighwayHashDU64 du64;
  const auto v_lo = Set(du64, val0);
  const auto v_hi = Set(du64, val1);
  return Create4(du64, v_lo, v_hi, v_lo, v_hi);
}

static HWY_INLINE AtLeast4LaneU64Vec
ShiftA23LeftBy1Lane(AtLeast4LaneU64Vec a23) {
  const HighwayHashDU64 du64;
  const auto zero = Zero(du64);
  return Create4(du64, zero, Get4<0>(a23), zero, Get4<2>(a23));
}

template <int kShiftAmt>
static HWY_INLINE AtLeast4LaneU64Vec
AtLeast4LaneU64VecShiftLeft(AtLeast4LaneU64Vec v) {
  return Create4(HighwayHashDU64(), ShiftLeft<kShiftAmt>(Get4<0>(v)),
                 ShiftLeft<kShiftAmt>(Get4<1>(v)),
                 ShiftLeft<kShiftAmt>(Get4<2>(v)),
                 ShiftLeft<kShiftAmt>(Get4<3>(v)));
}

template <int kShiftAmt>
static HWY_INLINE AtLeast4LaneU64Vec
AtLeast4LaneU64VecShiftRight(AtLeast4LaneU64Vec v) {
  return Create4(HighwayHashDU64(), ShiftRight<kShiftAmt>(Get4<0>(v)),
                 ShiftRight<kShiftAmt>(Get4<1>(v)),
                 ShiftRight<kShiftAmt>(Get4<2>(v)),
                 ShiftRight<kShiftAmt>(Get4<3>(v)));
}

static HWY_INLINE AtLeast4LaneU64Vec
AtLeast4LaneU64VecAnd(AtLeast4LaneU64Vec a, AtLeast4LaneU64Vec b) {
  return Create4(HighwayHashDU64(), And(Get4<0>(a), Get4<0>(b)),
                 And(Get4<1>(a), Get4<1>(b)), And(Get4<2>(a), Get4<2>(b)),
                 And(Get4<3>(a), Get4<3>(b)));
}

static HWY_INLINE AtLeast4LaneU64Vec
AtLeast4LaneU64VecOr(AtLeast4LaneU64Vec a, AtLeast4LaneU64Vec b) {
  return Create4(HighwayHashDU64(), Or(Get4<0>(a), Get4<0>(b)),
                 Or(Get4<1>(a), Get4<1>(b)), Or(Get4<2>(a), Get4<2>(b)),
                 Or(Get4<3>(a), Get4<3>(b)));
}

static HWY_INLINE AtLeast4LaneU64Vec AtLeast4LaneU64VecXor3(
    AtLeast4LaneU64Vec a, AtLeast4LaneU64Vec b, AtLeast4LaneU64Vec c) {
  return Create4(HighwayHashDU64(), Xor3(Get4<0>(a), Get4<0>(b), Get4<0>(c)),
                 Xor3(Get4<1>(a), Get4<1>(b), Get4<1>(c)),
                 Xor3(Get4<2>(a), Get4<2>(b), Get4<2>(c)),
                 Xor3(Get4<3>(a), Get4<3>(b), Get4<3>(c)));
}
#else
static HWY_INLINE AtLeast4LaneU64Vec AtLeast4LaneU64VecDup128(uint64_t val0,
                                                              uint64_t val1) {
  const HighwayHashDU64 du64;
  const AtLeast2LaneU64Vec vec = Dup128VecFromValues(du64, val0, val1);
  return CombineToAtLeast4LaneVec(vec, vec);
}

template <class V, hwy::EnableIf<hwy::IsSame<hwy::RemoveCvRef<V>,
                                             AtLeast2LaneU64Vec>()>* = nullptr>
static HWY_INLINE hwy::RemoveCvRef<V> ShiftA23LeftBy1Lane(V a23) {
  return ShiftLeftLanes<1>(DFromV<decltype(a23)>(), a23);
}
template <
    class V,
    hwy::EnableIf<(!hwy::IsSame<hwy::RemoveCvRef<V>, AtLeast2LaneU64Vec>() &&
                   hwy::IsSame<hwy::RemoveCvRef<V>, AtLeast4LaneU64Vec>())>* =
        nullptr>
static HWY_INLINE hwy::RemoveCvRef<V> ShiftA23LeftBy1Lane(V a23) {
  return CombineToAtLeast4LaneVec(
      ShiftA23LeftBy1Lane(GetLowerAtLeast2LaneVec(a23)),
      ShiftA23LeftBy1Lane(GetUpperAtLeast2LaneVec(a23)));
}

template <int kShiftAmt, class V,
          hwy::EnableIf<hwy::IsSame<hwy::RemoveCvRef<V>,
                                    AtLeast4LaneU64Vec>()>* = nullptr>
static HWY_INLINE hwy::RemoveCvRef<V> AtLeast4LaneU64VecShiftLeft(V v) {
  if constexpr (hwy::IsSame<hwy::RemoveCvRef<V>, AtLeast2LaneU64Vec>()) {
    return ShiftLeft<kShiftAmt>(v);
  } else {
    return CombineToAtLeast4LaneVec(
        ShiftLeft<kShiftAmt>(GetLowerAtLeast2LaneVec(v)),
        ShiftLeft<kShiftAmt>(GetUpperAtLeast2LaneVec(v)));
  }
}

template <int kShiftAmt, class V,
          hwy::EnableIf<hwy::IsSame<hwy::RemoveCvRef<V>,
                                    AtLeast4LaneU64Vec>()>* = nullptr>
static HWY_INLINE hwy::RemoveCvRef<V> AtLeast4LaneU64VecShiftRight(V v) {
  if constexpr (hwy::IsSame<hwy::RemoveCvRef<V>, AtLeast2LaneU64Vec>()) {
    return ShiftRight<kShiftAmt>(v);
  } else {
    return CombineToAtLeast4LaneVec(
        ShiftRight<kShiftAmt>(GetLowerAtLeast2LaneVec(v)),
        ShiftRight<kShiftAmt>(GetUpperAtLeast2LaneVec(v)));
  }
}

template <class V, hwy::EnableIf<hwy::IsSame<hwy::RemoveCvRef<V>,
                                             AtLeast4LaneU64Vec>()>* = nullptr>
static HWY_INLINE hwy::RemoveCvRef<V> AtLeast4LaneU64VecAnd(V a, V b) {
  if constexpr (hwy::IsSame<hwy::RemoveCvRef<V>, AtLeast2LaneU64Vec>()) {
    return And(a, b);
  } else {
    return CombineToAtLeast4LaneVec(
        And(GetLowerAtLeast2LaneVec(a), GetLowerAtLeast2LaneVec(b)),
        And(GetUpperAtLeast2LaneVec(a), GetUpperAtLeast2LaneVec(b)));
  }
}

template <class V, hwy::EnableIf<hwy::IsSame<hwy::RemoveCvRef<V>,
                                             AtLeast4LaneU64Vec>()>* = nullptr>
static HWY_INLINE hwy::RemoveCvRef<V> AtLeast4LaneU64VecOr(V a, V b) {
  if constexpr (hwy::IsSame<hwy::RemoveCvRef<V>, AtLeast2LaneU64Vec>()) {
    return Or(a, b);
  } else {
    return CombineToAtLeast4LaneVec(
        Or(GetLowerAtLeast2LaneVec(a), GetLowerAtLeast2LaneVec(b)),
        Or(GetUpperAtLeast2LaneVec(a), GetUpperAtLeast2LaneVec(b)));
  }
}

template <class V, hwy::EnableIf<hwy::IsSame<hwy::RemoveCvRef<V>,
                                             AtLeast4LaneU64Vec>()>* = nullptr>
static HWY_INLINE hwy::RemoveCvRef<V> AtLeast4LaneU64VecXor3(V a, V b, V c) {
  if constexpr (hwy::IsSame<hwy::RemoveCvRef<V>, AtLeast2LaneU64Vec>()) {
    return Xor3(a, b, c);
  } else {
    return CombineToAtLeast4LaneVec(
        Xor3(GetLowerAtLeast2LaneVec(a), GetLowerAtLeast2LaneVec(b),
             GetLowerAtLeast2LaneVec(c)),
        Xor3(GetUpperAtLeast2LaneVec(a), GetUpperAtLeast2LaneVec(b),
             GetUpperAtLeast2LaneVec(c)));
  }
}
#endif

static HWY_INLINE AtLeast4LaneU64Vec ModularReduction(AtLeast4LaneU64Vec a01,
                                                      AtLeast4LaneU64Vec a23) {
  // a0 is in the even lanes of a01
  // a1 is in the odd lanes of a01
  // a2 is in the even lanes of a23
  // a3 is in the odd lanes of a23

  const auto a2_in_odd = ShiftA23LeftBy1Lane(a23);
  a23 = AtLeast4LaneU64VecAnd(
      a23, AtLeast4LaneU64VecDup128(hwy::LimitsMax<uint64_t>(),
                                    0x3FFFFFFFFFFFFFFFu));
  return AtLeast4LaneU64VecXor3(
      a01,
      AtLeast4LaneU64VecOr(AtLeast4LaneU64VecShiftLeft<1>(a23),
                           AtLeast4LaneU64VecShiftRight<63>(a2_in_odd)),
      AtLeast4LaneU64VecOr(AtLeast4LaneU64VecShiftLeft<2>(a23),
                           AtLeast4LaneU64VecShiftRight<62>(a2_in_odd)));
}

#if HWY_TARGET == HWY_SCALAR
static HWY_INLINE void StoreHash256(const size_t /*lanes_per_u64_vec*/,
                                    AtLeast4LaneU64Vec v_hash,
                                    uint64_t* HWY_RESTRICT hash) {
  const HighwayHashDU64 du64;
  Store(Get4<0>(v_hash), du64, hash);
  Store(Get4<1>(v_hash), du64, hash + 1);
  Store(Get4<2>(v_hash), du64, hash + 2);
  Store(Get4<3>(v_hash), du64, hash + 3);
}
#else  // HWY_TARGET != HWY_SCALAR
template <class V, hwy::EnableIf<hwy::IsSame<hwy::RemoveCvRef<V>,
                                             AtLeast4LaneU64Vec>()>* = nullptr>
static HWY_INLINE void StoreHash256(const size_t lanes_per_u64_vec, V v_hash,
                                    uint64_t* HWY_RESTRICT hash) {
  const HighwayHashDU64 du64;
#if HWY_TARGET != HWY_RVV
  if constexpr (hwy::IsSame<V, Vec2<HighwayHashDU64>>()) {
    auto v_lo = GetLowerAtLeast2LaneVec(v_hash);
    auto v_hi = GetUpperAtLeast2LaneVec(v_hash);

    StoreU(v_lo, du64, hash);
    if (lanes_per_u64_vec < 4) {
      StoreU(v_hi, du64, hash + 2);
    }
  } else
#endif  // HWY_TARGET != HWY_RVV
  {
    (void)lanes_per_u64_vec;
    StoreU(v_hash, du64, hash);
  }
}
#endif  // HWY_TARGET == HWY_SCALAR

static void Finalize256(SimdHwyHashState* HWY_RESTRICT state,
                        uint64_t* HWY_RESTRICT hash) {
  const size_t lanes_per_u64_vec = Lanes(HighwayHashDU64());
  AtLeast4LaneU64Vec v0 =
      LoadAtLeast4LaneStateVec(lanes_per_u64_vec, state->v0);
  AtLeast4LaneU64Vec v1 =
      LoadAtLeast4LaneStateVec(lanes_per_u64_vec, state->v1);
  AtLeast4LaneU64Vec mul0 =
      LoadAtLeast4LaneStateVec(lanes_per_u64_vec, state->mul0);
  AtLeast4LaneU64Vec mul1 =
      LoadAtLeast4LaneStateVec(lanes_per_u64_vec, state->mul1);

  PermuteAndUpdate(v0, v1, mul0, mul1);
  PermuteAndUpdate(v0, v1, mul0, mul1);
  PermuteAndUpdate(v0, v1, mul0, mul1);
  PermuteAndUpdate(v0, v1, mul0, mul1);
  PermuteAndUpdate(v0, v1, mul0, mul1);
  PermuteAndUpdate(v0, v1, mul0, mul1);
  PermuteAndUpdate(v0, v1, mul0, mul1);
  PermuteAndUpdate(v0, v1, mul0, mul1);
  PermuteAndUpdate(v0, v1, mul0, mul1);
  PermuteAndUpdate(v0, v1, mul0, mul1);

  const AtLeast4LaneU64Vec v_hash = ModularReduction(
      AtLeast4LaneU64VecAdd(v0, mul0), AtLeast4LaneU64VecAdd(v1, mul1));
  StoreHash256(lanes_per_u64_vec, v_hash, hash);
}

}  // namespace
}  // namespace HWY_NAMESPACE
HWY_AFTER_NAMESPACE();

#if HWY_ONCE
namespace {
HWY_EXPORT(ResetHwyHashState);
HWY_EXPORT(UpdateHwyHashState);
HWY_EXPORT(Finalize64);
HWY_EXPORT(Finalize128);
HWY_EXPORT(Finalize256);
}  // namespace
#endif  // HWY_ONCE

}  // namespace simdhwyhash

#if HWY_ONCE
extern "C" {

void SimdHwyHash_Reset(SimdHwyHashState* SIMDHWYHASH_RESTRICT state,
                       const uint64_t* SIMDHWYHASH_RESTRICT key) {
  using namespace simdhwyhash;
  HWY_DYNAMIC_DISPATCH(ResetHwyHashState)(state, key);
}

void SimdHwyHash_Update(SimdHwyHashState* SIMDHWYHASH_RESTRICT state,
                        const void* SIMDHWYHASH_RESTRICT ptr, size_t byte_len) {
  using namespace simdhwyhash;
  HWY_DYNAMIC_DISPATCH(UpdateHwyHashState)
  (state, reinterpret_cast<const uint8_t*>(ptr), byte_len);
}

uint64_t SimdHwyHash_Finalize64(SimdHwyHashState* SIMDHWYHASH_RESTRICT state) {
  using namespace simdhwyhash;
  return HWY_DYNAMIC_DISPATCH(Finalize64)(state);
}

void SimdHwyHash_Finalize128(SimdHwyHashState* SIMDHWYHASH_RESTRICT state,
                             uint64_t* SIMDHWYHASH_RESTRICT hash) {
  using namespace simdhwyhash;
  return HWY_DYNAMIC_DISPATCH(Finalize128)(state, hash);
}

void SimdHwyHash_Finalize256(SimdHwyHashState* SIMDHWYHASH_RESTRICT state,
                             uint64_t* SIMDHWYHASH_RESTRICT hash) {
  using namespace simdhwyhash;
  return HWY_DYNAMIC_DISPATCH(Finalize256)(state, hash);
}

uint64_t SimdHwyHash_Hash64(const void* SIMDHWYHASH_RESTRICT ptr,
                            size_t byte_len,
                            const uint64_t* SIMDHWYHASH_RESTRICT key) {
  SimdHwyHashState state;
  SimdHwyHash_Reset(&state, key);
  SimdHwyHash_Update(&state, ptr, byte_len);
  return SimdHwyHash_Finalize64(&state);
}

void SimdHwyHash_Hash128(const void* SIMDHWYHASH_RESTRICT ptr, size_t byte_len,
                         const uint64_t* SIMDHWYHASH_RESTRICT key,
                         uint64_t* SIMDHWYHASH_RESTRICT hash) {
  SimdHwyHashState state;
  SimdHwyHash_Reset(&state, key);
  SimdHwyHash_Update(&state, ptr, byte_len);
  SimdHwyHash_Finalize128(&state, hash);
}

void SimdHwyHash_Hash256(const void* SIMDHWYHASH_RESTRICT ptr, size_t byte_len,
                         const uint64_t* SIMDHWYHASH_RESTRICT key,
                         uint64_t* SIMDHWYHASH_RESTRICT hash) {
  SimdHwyHashState state;
  SimdHwyHash_Reset(&state, key);
  SimdHwyHash_Update(&state, ptr, byte_len);
  SimdHwyHash_Finalize256(&state, hash);
}

}  // extern "C"
#endif  // HWY_ONCE
