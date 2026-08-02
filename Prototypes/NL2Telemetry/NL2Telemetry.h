// TrackUnlimited Phase 0 prototype: NoLimits 2 telemetry wire protocol.
// Plain C++17, no engine dependency and NO SOCKETS — this header only turns
// bytes into messages and back, so it is testable offline. The socket lives in
// record.cpp, which is the part you cannot unit test.
// Protocol reference: https://nolimitscoaster.com/nolimits2/help/pages/telemetry.html
//
// WHY THIS EXISTS. Two entries in Docs/PHASE0_FINDINGS.md "Still unknown" need
// numbers out of a *running* NL2, not out of a track file:
//
//   * the lateral-G sign versus NoLimits 2, which rests on a documentation
//     reading nobody has checked against a running copy;
//   * calibration of RollingResistance and DragK, which have never been
//     compared to a measured speed trace.
//
// Telemetry is available in the STANDARD edition (Main Menu -> Setup -> Others,
// or --telemetry, default port 15151), unlike the CSV spline export, which is
// Professional-only. So this path works regardless of licence tier.
//
// Units: NL2 reports metres and m/s. Position is in NL2 world axes (right
// handed, +Y up) and is mapped to prototype axes on the way in, exactly as in
// NL2Csv.h — the two must agree or nothing correlates.
//
// The G-force frame is NOT mapped, and that is deliberate: see FNL2Telemetry.

#pragma once

#include "../NL2Csv/NL2Csv.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

// ------------------------------------------------------------------- framing

// "All multi-byte values are in network-byte-order (big-endian)!!" — the spec is
// emphatic, and it is the opposite of this machine, so every field goes through
// these rather than a struct overlay.
namespace NL2Wire
{

inline std::uint16_t ReadU16(const unsigned char* P)
{
    return static_cast<std::uint16_t>((static_cast<std::uint32_t>(P[0]) << 8) | P[1]);
}

inline std::uint32_t ReadU32(const unsigned char* P)
{
    return (static_cast<std::uint32_t>(P[0]) << 24) | (static_cast<std::uint32_t>(P[1]) << 16)
         | (static_cast<std::uint32_t>(P[2]) << 8) | static_cast<std::uint32_t>(P[3]);
}

inline std::int32_t ReadI32(const unsigned char* P)
{
    return static_cast<std::int32_t>(ReadU32(P));
}

// The 4 bytes are the big-endian IEEE-754 representation. Assemble the bit
// pattern first, then memcpy it — type-punning through a pointer cast would be
// undefined, and memcpy compiles to nothing.
inline float ReadF32(const unsigned char* P)
{
    const std::uint32_t Bits = ReadU32(P);
    float Value = 0.0f;
    std::memcpy(&Value, &Bits, sizeof(Value));
    return Value;
}

inline void WriteU16(std::vector<unsigned char>& Out, std::uint16_t V)
{
    Out.push_back(static_cast<unsigned char>(V >> 8));
    Out.push_back(static_cast<unsigned char>(V & 0xFF));
}

inline void WriteU32(std::vector<unsigned char>& Out, std::uint32_t V)
{
    Out.push_back(static_cast<unsigned char>(V >> 24));
    Out.push_back(static_cast<unsigned char>((V >> 16) & 0xFF));
    Out.push_back(static_cast<unsigned char>((V >> 8) & 0xFF));
    Out.push_back(static_cast<unsigned char>(V & 0xFF));
}

} // namespace NL2Wire

// Message type IDs. Only the handful this prototype needs are named; the rest
// (station control, park loading, e-stop) are deliberately absent because
// nothing here should be able to drive someone's ride by accident.
enum ENL2MessageType : std::uint16_t
{
    NL2_Idle = 0,
    NL2_OK = 1,
    NL2_Error = 2,
    NL2_GetVersion = 3,
    NL2_Version = 4,
    NL2_GetTelemetry = 5,
    NL2_Telemetry = 6,
};

struct FNL2Message
{
    std::uint16_t Type = 0;
    std::uint32_t RequestId = 0;
    std::vector<unsigned char> Data;
};

// 'N' | type(2) | requestId(4) | dataSize(2) | data | 'L'
constexpr std::size_t NL2HeaderBytes = 9;
constexpr std::size_t NL2MinMessageBytes = 10;

inline std::vector<unsigned char> EncodeRequest(std::uint16_t Type, std::uint32_t RequestId)
{
    std::vector<unsigned char> Out;
    Out.push_back('N');
    NL2Wire::WriteU16(Out, Type);
    NL2Wire::WriteU32(Out, RequestId);
    NL2Wire::WriteU16(Out, 0); // every request this prototype sends is payload-free
    Out.push_back('L');
    return Out;
}

enum ENL2Decode
{
    NL2Decode_Ok = 0,
    NL2Decode_Incomplete = 1, // need more bytes, keep reading
    NL2Decode_Malformed = 2,  // stream is desynchronised, reconnect
};

// Try to peel one message off the front of a stream buffer. On Ok, OutConsumed
// says how many bytes to erase. Incomplete is the normal case on a partial read
// and must NOT be treated as an error — a 76-byte telemetry payload routinely
// arrives split across TCP segments.
inline ENL2Decode DecodeMessage(const unsigned char* Buffer, std::size_t Available,
                                FNL2Message& Out, std::size_t& OutConsumed)
{
    OutConsumed = 0;
    if (Available < NL2MinMessageBytes)
    {
        return NL2Decode_Incomplete;
    }
    if (Buffer[0] != 'N')
    {
        return NL2Decode_Malformed;
    }

    const std::size_t DataSize = NL2Wire::ReadU16(Buffer + 7);
    const std::size_t Total = NL2HeaderBytes + DataSize + 1;
    if (Available < Total)
    {
        return NL2Decode_Incomplete;
    }
    if (Buffer[Total - 1] != 'L')
    {
        return NL2Decode_Malformed;
    }

    Out.Type = NL2Wire::ReadU16(Buffer + 1);
    Out.RequestId = NL2Wire::ReadU32(Buffer + 3);
    Out.Data.assign(Buffer + NL2HeaderBytes, Buffer + NL2HeaderBytes + DataSize);
    OutConsumed = Total;
    return NL2Decode_Ok;
}

// ----------------------------------------------------------------- telemetry

constexpr std::size_t NL2TelemetryBytes = 76;

struct FNL2Telemetry
{
    std::int32_t StateFlags = 0;
    std::int32_t Frame = 0;
    std::int32_t ViewMode = 0;
    std::int32_t Coaster = 0;
    std::int32_t StyleId = 0;
    std::int32_t Train = 0;
    std::int32_t Car = 0;
    std::int32_t Seat = 0;

    float Speed = 0.0f; // m/s

    // Mapped into prototype axes (+Z up) with FromNL2, same as the CSV reader.
    // Correlating telemetry against an imported track depends on both using one
    // convention, so this must not drift from NL2Csv.h.
    FVec3 Position;

    float QuatX = 0.0f, QuatY = 0.0f, QuatZ = 0.0f, QuatW = 0.0f;

    // RAW, deliberately NOT mapped. The telemetry spec states units and offsets
    // but never says what frame these are in — world, or the car's local
    // (front/left/up) basis. Mapping them as if world axes would bake in the
    // very assumption this prototype exists to test. Identify the frame from
    // the data first (see IdentifyGravityAxis), then convert once, knowingly.
    float GX = 0.0f, GY = 0.0f, GZ = 0.0f;

    bool InPlayMode() const { return (StateFlags & 1) != 0; }
    bool Braking() const { return (StateFlags & 2) != 0; }
    bool Paused() const { return (StateFlags & 4) != 0; }
    bool Onride() const { return ViewMode == 1 || ViewMode == 2; }
};

// Field offsets are from the published table; a 76-byte payload is required
// rather than assumed, so a protocol change shows up as a clean failure instead
// of silently shifted fields.
inline bool ParseTelemetry(const std::vector<unsigned char>& Data, FNL2Telemetry& Out)
{
    if (Data.size() != NL2TelemetryBytes)
    {
        return false;
    }
    const unsigned char* P = Data.data();
    Out.StateFlags = NL2Wire::ReadI32(P + 0);
    Out.Frame = NL2Wire::ReadI32(P + 4);
    Out.ViewMode = NL2Wire::ReadI32(P + 8);
    Out.Coaster = NL2Wire::ReadI32(P + 12);
    Out.StyleId = NL2Wire::ReadI32(P + 16);
    Out.Train = NL2Wire::ReadI32(P + 20);
    Out.Car = NL2Wire::ReadI32(P + 24);
    Out.Seat = NL2Wire::ReadI32(P + 28);
    Out.Speed = NL2Wire::ReadF32(P + 32);
    Out.Position = FromNL2({NL2Wire::ReadF32(P + 36), NL2Wire::ReadF32(P + 40),
                            NL2Wire::ReadF32(P + 44)});
    Out.QuatX = NL2Wire::ReadF32(P + 48);
    Out.QuatY = NL2Wire::ReadF32(P + 52);
    Out.QuatZ = NL2Wire::ReadF32(P + 56);
    Out.QuatW = NL2Wire::ReadF32(P + 60);
    Out.GX = NL2Wire::ReadF32(P + 64);
    Out.GY = NL2Wire::ReadF32(P + 68);
    Out.GZ = NL2Wire::ReadF32(P + 72);
    return true;
}

// ------------------------------------------------------- frame identification

// Which G axis is carrying gravity, decided by measurement rather than by
// reading the docs — because the docs do not say.
//
// The trick is that a train sitting still (or rolling gently on level track)
// feels exactly one g of apparent gravity and nothing else. Whichever component
// parks near +/-1 across those samples IS the vertical axis, and its sign tells
// you whether NL2 reports the force on the rider or the acceleration of the
// car. Everything else follows from that one fact.
struct FGravityAxisReport
{
    int Axis = -1;      // 0 = x, 1 = y, 2 = z; -1 if undecided
    double Value = 0.0; // its mean reading over the low-speed samples used
    int Samples = 0;
    double OtherMax = 0.0; // largest |other component|, i.e. how clean the call is
};

// Is that G vector in the CAR's frame or the WORLD's? The spec does not say,
// and at rest on level track the two are indistinguishable — which is exactly
// where IdentifyGravityAxis looks, so it cannot tell them apart on its own.
//
// A lift hill separates them cleanly. Riding a steady 28-degree slope, apparent
// gravity is still 1 g straight down. Resolved into the CAR's basis that reads
// cos(28) = 0.88 on the vertical axis with the missing 0.47 showing up fore/aft;
// left in WORLD axes it stays a flat 1.00 with nothing anywhere else. So compare
// the gravity-axis reading against cos(pitch) and against 1, and take whichever
// it is closer to.
//
// This matters more than it looks: if G is world-referenced then "the lateral
// axis" is not a fixed component at all — it rotates with the train's heading —
// and any single-axis sign verdict would be meaningless.
struct FGFrameReport
{
    enum EFrame
    {
        Undecided,
        CarLocal,
        World
    };

    EFrame Frame = Undecided;
    double MeanReading = 0.0; // mean |gravity-axis component| over the steep samples
    double MeanCosPitch = 0.0;
    int Samples = 0;
};

// Each entry is (track pitch in radians at that point, gravity-axis reading).
inline FGFrameReport ClassifyGFrame(const std::vector<std::pair<double, double>>& PitchAndReading,
                                    double MinPitchRad = 15.0 * 3.14159265358979323846 / 180.0)
{
    FGFrameReport R;
    double SumRead = 0.0, SumCos = 0.0;
    for (const std::pair<double, double>& E : PitchAndReading)
    {
        if (std::fabs(E.first) < MinPitchRad)
        {
            continue;
        }
        SumRead += E.second < 0.0 ? -E.second : E.second;
        SumCos += std::cos(E.first);
        ++R.Samples;
    }
    if (R.Samples < 10)
    {
        return R; // not enough steep track to say anything
    }

    R.MeanReading = SumRead / R.Samples;
    R.MeanCosPitch = SumCos / R.Samples;

    // If the track was never steep enough, cos(pitch) is ~1 and the two
    // hypotheses predict the same number. Refuse rather than coin-flip.
    if (R.MeanCosPitch > 0.95)
    {
        return R;
    }

    const double ToCar = std::fabs(R.MeanReading - R.MeanCosPitch);
    const double ToWorld = std::fabs(R.MeanReading - 1.0);
    R.Frame = ToCar < ToWorld ? FGFrameReport::CarLocal : FGFrameReport::World;
    return R;
}

inline FGravityAxisReport IdentifyGravityAxis(const std::vector<FNL2Telemetry>& Log,
                                              double MaxSpeedMs = 1.0)
{
    FGravityAxisReport R;
    double Sum[3] = {0.0, 0.0, 0.0};
    for (const FNL2Telemetry& T : Log)
    {
        if (T.Speed > MaxSpeedMs)
        {
            continue;
        }
        Sum[0] += T.GX;
        Sum[1] += T.GY;
        Sum[2] += T.GZ;
        ++R.Samples;
    }
    if (R.Samples == 0)
    {
        return R;
    }
    for (int i = 0; i < 3; ++i)
    {
        Sum[i] /= R.Samples;
    }
    for (int i = 0; i < 3; ++i)
    {
        const double Abs = Sum[i] < 0.0 ? -Sum[i] : Sum[i];
        const double Best = R.Value < 0.0 ? -R.Value : R.Value;
        if (Abs > Best)
        {
            R.Axis = i;
            R.Value = Sum[i];
        }
    }
    for (int i = 0; i < 3; ++i)
    {
        if (i == R.Axis)
        {
            continue;
        }
        const double Abs = Sum[i] < 0.0 ? -Sum[i] : Sum[i];
        R.OtherMax = Abs > R.OtherMax ? Abs : R.OtherMax;
    }
    return R;
}
