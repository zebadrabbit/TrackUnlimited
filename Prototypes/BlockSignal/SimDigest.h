// TrackUnlimited: a fingerprint of the simulation's state, for proving two runs
// are the same run.
// Plain C++17, no dependencies at all — doubles in, one integer out.
//
// WHY THIS EXISTS. If what this project offers downstream is a foundation rather
// than a claim, determinism is not a feature of it — it is the property that
// makes everything else provable. A fault-injection scenario that reproduces
// differently every run cannot demonstrate anything to anybody, and a bug report
// that cannot be replayed is a story.
//
// The fixed scan period (ATUCoasterRide::SimStep) made determinism POSSIBLE.
// This is what makes it CHECKABLE: run the same scenario twice and the digest
// sequences must be identical, scan for scan.
//
// ===================== WHAT IT IS NOT =====================
//
// Not a checksum for integrity and not a hash for security — FNV-1a is neither.
// It is a cheap fingerprint whose only job is to differ when the state differs.
// Collisions are possible in principle and irrelevant in practice here: two runs
// of the same code on the same inputs produce identical bytes, not similar ones,
// so the check is equality rather than approximation.
//
// ===================== BIT PATTERNS, NOT VALUES =====================
//
// Doubles are hashed by their BITS. That is deliberate and it is the strict
// choice: -0.0 and +0.0 compare equal as numbers and are different states of the
// arithmetic, and two NaNs that compare unequal to everything including
// themselves are the same state. A digest that used == would call a run
// reproducible while its floating point drifted underneath.

#pragma once

#include <cstdint>
#include <cstring>

class FSimDigest
{
public:
    void Reset() { H = Offset; }

    void Add(double V)
    {
        std::uint64_t Bits = 0;
        static_assert(sizeof(Bits) == sizeof(V), "double is not 64 bits here");
        std::memcpy(&Bits, &V, sizeof(Bits));
        AddU64(Bits);
    }

    void Add(int V)  { AddU64(static_cast<std::uint64_t>(static_cast<std::int64_t>(V))); }
    void Add(bool V) { AddU64(V ? 1u : 0u); }
    void Add(std::size_t V) { AddU64(static_cast<std::uint64_t>(V)); }

    std::uint64_t Value() const { return H; }

    // ORDER MATTERS, and that is a feature. Two runs that visit the same trains
    // in a different order are not the same run — the scan order IS part of the
    // control system's behaviour, and a digest that ignored it would call a
    // reordered scan reproducible.
    void AddU64(std::uint64_t X)
    {
        for (int i = 0; i < 8; ++i)
        {
            H ^= static_cast<std::uint64_t>((X >> (i * 8)) & 0xFF);
            H *= Prime;
        }
    }

private:
    // FNV-1a, 64-bit.
    static constexpr std::uint64_t Offset = 1469598103934665603ull;
    static constexpr std::uint64_t Prime = 1099511628211ull;
    std::uint64_t H = Offset;
};
