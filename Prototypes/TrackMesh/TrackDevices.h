// TrackUnlimited: device hardware on the track — what makes a zone LOOK like one.
//
// ===================== THE GAP =====================
//
// Until this, the only thing that showed a span was a device was its catwalk,
// and a catwalk is not always wanted. A lift hill, a brake run, a launch and a
// station were rails, spine and ties exactly like open track — the physics and
// the signalling knew, and a screenshot did not.
//
// ===================== THE SHAPE, FROM SEVEN REFERENCE PHOTOS =====================
//
// Every device except the station platform is "a block of geometry between the
// rails, repeated per span, on a zone" — the same sweep the ties use, driven by
// the same frame walk. So this is one pass with five kinds of hardware:
//
//   CHAIN    a trough on the centreline with the chain lying in it (lift)
//   CATCH    the toothed anti-rollback dog rail beside the trough (bAntiRollback)
//   FINS     a long fin block between the rails with pads inset (friction brake)
//   TYRES    paired drive tyres standing between the rails (block brake, station)
//   STATORS  flat stator blocks in pairs (LSM launch)
//
// A zone kind maps to a BITMASK of these, because a block brake is fins AND
// tyres — the "a block brake is TWO machines" fact, drawn.
//
// ===================== TWO BUFFERS, BECAUSE TWO MATERIALS =====================
//
// Hardware is painted steel like the spine. Rubber is a tyre tread, a brake pad,
// a chain, a stator's magnet face: the dark parts, which are what the eye reads
// as "a machine" rather than more rail. Keeping them apart costs nothing.
//
// Everything is generic: no manufacturer's fin profile, no real motor housing.
// Dimensions are ordinary practice and are KNOBS.
//
// Held to the same bar as the ties and the catwalk: every box is closed and
// outward-wound, asserted by signed volume and watertightness in
// test_trackdevices.cpp. ponytail: boxes and short tubes only — a gearbox
// casting, a chain link, a caliper are detail for a model nobody has.

#pragma once

#include "TrackMesh.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

enum EDeviceHardware : unsigned
{
    DeviceNone = 0,
    DeviceChain = 1u << 0,
    DeviceCatch = 1u << 1,
    DeviceFins = 1u << 2,
    DeviceTyres = 1u << 3,
    DeviceStators = 1u << 4,
};

struct FTrackDeviceSpan
{
    double StartS = 0.0;
    double EndS = 0.0;
    unsigned Hardware = DeviceNone;   // EDeviceHardware bits
};

struct FTrackDeviceSettings
{
    // Chain trough: width and height, top face at the rail centreline.
    double TroughWidthM = 0.18;
    double TroughHeightM = 0.12;
    double ChainWidthM = 0.06;
    double ChainHeightM = 0.03;

    // Anti-rollback dogs: one tooth every PitchM, offset to the rider's LEFT
    // of the trough. A tooth is a small block; the rack under it is a strip.
    double CatchOffsetM = 0.22;
    double CatchPitchM = 0.30;
    double CatchToothM = 0.08;
    double CatchHeightM = 0.07;
    double CatchRailWidthM = 0.05;

    // Brake fins: one block every FinPitchM, FinLengthM long, standing
    // FinAboveRailM proud of the rail centreline so it meets the train's fin.
    double FinPitchM = 2.0;
    double FinLengthM = 1.7;
    double FinWidthM = 0.10;
    double FinHeightM = 0.30;
    double FinAboveRailM = 0.12;
    double PadInsetM = 0.02;   // the rubber strip inset into each flank

    // Drive tyres: a pair every TyrePitchM, one each side of the centreline,
    // axle lateral, tread top TyreAboveRailM above the rail centreline.
    double TyrePitchM = 3.0;
    double TyreRadiusM = 0.22;
    double TyreWidthM = 0.12;
    double TyreLateralM = 0.26;
    double TyreAboveRailM = 0.10;
    int TyreSides = 12;

    // LSM stators: a pair of flat blocks every StatorPitchM, magnet faces inward.
    double StatorPitchM = 2.0;
    double StatorLengthM = 1.8;
    double StatorWidthM = 0.22;
    double StatorHeightM = 0.20;
    double StatorLateralM = 0.24;
    double StatorAboveRailM = 0.08;
};

struct FTrackDeviceMesh
{
    FMeshBuffer Hardware;   // painted steel
    FMeshBuffer Rubber;     // tread, pads, chain, magnet faces

    std::size_t NumTriangles() const { return Hardware.NumTriangles() + Rubber.NumTriangles(); }
};

// The zone enumerator names are the actor's; this header is engine-free, so the
// mapping takes the kind as the NAME the actor already prints.
inline unsigned HardwareForZoneName(const char* ZoneName)
{
    auto Eq = [&](const char* K) { return std::strcmp(ZoneName, K) == 0; };
    if (Eq("LIFT"))        { return DeviceChain; }
    if (Eq("LAUNCH"))      { return DeviceStators; }
    if (Eq("TRIM"))        { return DeviceFins; }
    if (Eq("BLOCK BRAKE")) { return DeviceFins | DeviceTyres; }
    if (Eq("STATION") || Eq("UNLOAD") || Eq("LOAD")) { return DeviceFins | DeviceTyres; }
    return DeviceNone;
}

namespace DeviceDetail
{

// A closed box: centre, three unit axes, half extents. Six faces, flat normals,
// outward-wound with the same handedness the rest of the mesher uses
// (Tangent x Lateral = Up), so signed volume is +8abc.
inline void AddBox(FMeshBuffer& Out, const FVec3& C, const FVec3& A, const FVec3& B,
                   const FVec3& U, double Ha, double Hb, double Hu)
{
    const FVec3 Ax[3] = {A, B, U};
    const double H[3] = {Ha, Hb, Hu};
    for (int axis = 0; axis < 3; ++axis)
    {
        for (int sign = -1; sign <= 1; sign += 2)
        {
            const FVec3 N = Ax[axis] * static_cast<double>(sign);
            const FVec3 P = Ax[(axis + 1) % 3];
            const FVec3 Q = Ax[(axis + 2) % 3];
            const double Hp = H[(axis + 1) % 3];
            const double Hq = H[(axis + 2) % 3];
            const FVec3 Face = C + N * H[axis];
            const std::uint32_t Base = static_cast<std::uint32_t>(Out.Position.size());
            // Corners in an order that is counter-clockwise seen from outside
            // when sign is +1 for a right-handed (P, Q, N) triple.
            const FVec3 Corner[4] = {
                Face - P * Hp - Q * Hq, Face + P * Hp - Q * Hq,
                Face + P * Hp + Q * Hq, Face - P * Hp + Q * Hq};
            const FVec2 Tex[4] = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
            for (int k = 0; k < 4; ++k)
            {
                Out.Position.push_back(Corner[k]);
                Out.Normal.push_back(N);
                Out.UV.push_back(Tex[k]);
            }
            // (P, Q, N) is right-handed when N = P x Q; for axis order
            // (A,B,U),(B,U,A),(U,A,B) that holds with sign +1, so the winding
            // 0-1-2, 0-2-3 faces +N and is reversed for the -N face.
            if (sign > 0)
            {
                Out.Index.insert(Out.Index.end(), {Base, Base + 1, Base + 2, Base, Base + 2, Base + 3});
            }
            else
            {
                Out.Index.insert(Out.Index.end(), {Base, Base + 2, Base + 1, Base, Base + 3, Base + 2});
            }
        }
    }
}

// A short cylinder with its axis along `Axis` — a tyre, or a chain sprocket.
inline void AddDisc(FMeshBuffer& Out, const FVec3& C, const FVec3& Axis, const FVec3& Hint,
                    double Radius, double Width, int Sides)
{
    SweepStrut(Out, C - Axis * (Width * 0.5), C + Axis * (Width * 0.5), Hint, Radius, Sides);
}

// A rectangular tube along a run of frames, capped: four walls and two ends,
// ONE closed body rather than a box per interval. A box per interval puts two
// coincident faces at every frame, which is z-fighting and is not watertight.
struct FRectRing { FVec3 C, Lat, Up; };
inline void SweepRect(FMeshBuffer& Out, const std::vector<FRectRing>& R, double Hw, double Hh)
{
    if (R.size() < 2) { return; }
    auto Corner = [&](std::size_t i, int k)
    {
        // 0: +lat -up, 1: +lat +up, 2: -lat +up, 3: -lat -up — around the tube.
        const double Sl = (k == 0 || k == 1) ? 1.0 : -1.0;
        const double Su = (k == 1 || k == 2) ? 1.0 : -1.0;
        return R[i].C + R[i].Lat * (Sl * Hw) + R[i].Up * (Su * Hh);
    };
    // Walls: each of the four edges k..k+1 is a strip along the run.
    for (int k = 0; k < 4; ++k)
    {
        const int k1 = (k + 1) % 4;
        for (std::size_t i = 0; i + 1 < R.size(); ++i)
        {
            const std::uint32_t Base = static_cast<std::uint32_t>(Out.Position.size());
            const FVec3 P[4] = {Corner(i, k), Corner(i, k1), Corner(i + 1, k1), Corner(i + 1, k)};
            FVec3 N = Normalised(Cross(P[1] - P[0], P[3] - P[0]));
            for (int c = 0; c < 4; ++c)
            {
                Out.Position.push_back(P[c]);
                Out.Normal.push_back(N);
                Out.UV.push_back({static_cast<double>(c / 2), static_cast<double>((c == 1 || c == 2) ? 1 : 0)});
            }
            Out.Index.insert(Out.Index.end(), {Base, Base + 1, Base + 2, Base, Base + 2, Base + 3});
        }
    }
    // Caps. The start faces backwards (reverse order), the end forwards.
    for (int end = 0; end < 2; ++end)
    {
        const std::size_t i = end == 0 ? 0 : R.size() - 1;
        const std::uint32_t Base = static_cast<std::uint32_t>(Out.Position.size());
        const FVec3 Along = Normalised(Cross(R[i].Lat, R[i].Up));   // lat x up = -tangent? see below
        const FVec3 N = end == 0 ? Along : Along * -1.0;
        for (int k = 0; k < 4; ++k)
        {
            Out.Position.push_back(Corner(i, k));
            Out.Normal.push_back(N);
            Out.UV.push_back({static_cast<double>(k & 1), static_cast<double>(k >> 1)});
        }
        if (end == 0) { Out.Index.insert(Out.Index.end(), {Base, Base + 1, Base + 2, Base, Base + 2, Base + 3}); }
        else          { Out.Index.insert(Out.Index.end(), {Base, Base + 2, Base + 1, Base, Base + 3, Base + 2}); }
    }
}

// The frame at an arc length, POSITIONED THERE rather than snapped to the
// nearest sample: the first version returned the nearest frame and every tooth
// on a 0.3 m pitch landed on a 0.5 m sample, several to a frame. The basis is
// the sample's (a 0.25 m extrapolation along a tangent is well under anything
// visible); the position is exact.
inline FTrackFrame FrameAt(const std::vector<FTrackFrame>& Frames,
                           const std::vector<double>& S, double At)
{
    std::size_t Lo = 0, Hi = S.size() - 1;
    while (Lo + 1 < Hi)
    {
        const std::size_t Mid = (Lo + Hi) / 2;
        if (S[Mid] <= At) { Lo = Mid; } else { Hi = Mid; }
    }
    FTrackFrame F = Frames[Lo];
    F.Position = F.Position + F.Tangent * (At - S[Lo]);
    return F;
}

} // namespace DeviceDetail

inline FTrackDeviceMesh BuildDeviceHardware(const std::vector<FTrackFrame>& Frames,
                                       const std::vector<FTrackDeviceSpan>& Spans,
                                       double HeartlineHeight,
                                       const FTrackProfile& /*Profile*/,
                                       const FTrackDeviceSettings& D = FTrackDeviceSettings())
{
    using namespace DeviceDetail;
    FTrackDeviceMesh Out;
    if (Frames.size() < 2) { return Out; }

    std::vector<double> S(Frames.size(), 0.0);
    for (std::size_t k = 1; k < Frames.size(); ++k)
    {
        S[k] = S[k - 1] + Length(Frames[k].Position - Frames[k - 1].Position);
    }

    // The rail-centre point and the frame's basis at an arc length: everything
    // is placed relative to it. Up is the frame's own, so a brake on a banked
    // section banks with the track as the real one is bolted to it.
    auto Centre = [&](const FTrackFrame& F) { return F.Position - F.Up * HeartlineHeight; };

    for (const FTrackDeviceSpan& Span : Spans)
    {
        if (Span.Hardware == DeviceNone || Span.EndS <= Span.StartS) { continue; }
        const double Len = Span.EndS - Span.StartS;

        // ---- CONTINUOUS: the chain trough, the chain and the catch rail are
        // each ONE rectangular sweep over the frames inside the span.
        if (Span.Hardware & (DeviceChain | DeviceCatch))
        {
            std::vector<FRectRing> Trough, Chain, Rack;
            for (std::size_t i = 0; i < Frames.size(); ++i)
            {
                if (S[i] < Span.StartS - 1e-6 || S[i] > Span.EndS + 1e-6) { continue; }
                const FTrackFrame& F = Frames[i];
                const FVec3 C = Centre(F);
                Trough.push_back({C - F.Up * (D.TroughHeightM * 0.5), F.Lateral, F.Up});
                Chain.push_back({C + F.Up * (D.ChainHeightM * 0.5), F.Lateral, F.Up});
                Rack.push_back({C + F.Lateral * D.CatchOffsetM - F.Up * (D.CatchHeightM * 0.25), F.Lateral, F.Up});
            }
            if (Span.Hardware & DeviceChain)
            {
                SweepRect(Out.Hardware, Trough, D.TroughWidthM * 0.5, D.TroughHeightM * 0.5);
                SweepRect(Out.Rubber, Chain, D.ChainWidthM * 0.5, D.ChainHeightM * 0.5);
            }
            if (Span.Hardware & DeviceCatch)
            {
                SweepRect(Out.Hardware, Rack, D.CatchRailWidthM * 0.5, D.CatchHeightM * 0.25);
            }
        }
        if (Span.Hardware & DeviceCatch)
        {
            // The teeth, at a pitch rather than per frame: a dog rail's teeth are
            // what say "anti-rollback" from a distance, and per-frame teeth
            // would be a pitch that changed with the mesher's sampling.
            for (double At = Span.StartS + D.CatchPitchM * 0.5; At < Span.EndS; At += D.CatchPitchM)
            {
                const FTrackFrame F = FrameAt(Frames, S, At);
                // Narrower than the rack and sunk 1 cm into it, so no face of a
                // tooth is coplanar with a face of the rack: coincident faces are
                // z-fighting on screen and a shared edge in the watertight check.
                AddBox(Out.Hardware, Centre(F) + F.Lateral * D.CatchOffsetM + F.Up * (D.CatchHeightM * 0.5 - 0.01),
                       F.Tangent, F.Lateral, F.Up,
                       D.CatchToothM * 0.5, D.CatchRailWidthM * 0.4, D.CatchHeightM * 0.5);
            }
        }

        // ---- REPEATED: fins, tyres, stators at their own pitch, centred so a
        // span gets floor(Len / Pitch) of them and the leftover splits evenly.
        auto Repeat = [&](double Pitch, auto&& Place)
        {
            const int N = static_cast<int>(std::floor(Len / Pitch));
            if (N <= 0) { return; }
            const double Start = Span.StartS + (Len - N * Pitch) * 0.5 + Pitch * 0.5;
            for (int k = 0; k < N; ++k) { Place(FrameAt(Frames, S, Start + k * Pitch)); }
        };

        if (Span.Hardware & DeviceFins)
        {
            Repeat(D.FinPitchM, [&](const FTrackFrame& F)
            {
                const FVec3 C = Centre(F) + F.Up * (D.FinAboveRailM - D.FinHeightM * 0.5);
                AddBox(Out.Hardware, C, F.Tangent, F.Lateral, F.Up,
                       D.FinLengthM * 0.5, D.FinWidthM * 0.5, D.FinHeightM * 0.5);
                // The pad strips, one each flank, inset into the fin's upper half.
                for (int side = -1; side <= 1; side += 2)
                {
                    AddBox(Out.Rubber, C + F.Lateral * (side * (D.FinWidthM * 0.5 - D.PadInsetM * 0.5))
                               + F.Up * (D.FinHeightM * 0.25),
                           F.Tangent, F.Lateral, F.Up,
                           D.FinLengthM * 0.45, D.PadInsetM, D.FinHeightM * 0.2);
                }
            });
        }
        if (Span.Hardware & DeviceTyres)
        {
            Repeat(D.TyrePitchM, [&](const FTrackFrame& F)
            {
                const FVec3 C = Centre(F) + F.Up * (D.TyreAboveRailM - D.TyreRadiusM);
                for (int side = -1; side <= 1; side += 2)
                {
                    const FVec3 Hub = C + F.Lateral * (side * D.TyreLateralM);
                    AddDisc(Out.Rubber, Hub, F.Lateral, F.Tangent, D.TyreRadiusM, D.TyreWidthM, D.TyreSides);
                    // The motor behind it: a block dropped below the axle.
                    AddBox(Out.Hardware, Hub - F.Up * (D.TyreRadiusM * 0.9) - F.Lateral * (side * D.TyreWidthM),
                           F.Tangent, F.Lateral, F.Up, D.TyreRadiusM * 0.5, D.TyreWidthM * 0.6, D.TyreRadiusM * 0.4);
                }
            });
        }
        if (Span.Hardware & DeviceStators)
        {
            Repeat(D.StatorPitchM, [&](const FTrackFrame& F)
            {
                for (int side = -1; side <= 1; side += 2)
                {
                    const FVec3 C = Centre(F) + F.Lateral * (side * D.StatorLateralM)
                        + F.Up * (D.StatorAboveRailM - D.StatorHeightM * 0.5);
                    AddBox(Out.Hardware, C, F.Tangent, F.Lateral, F.Up,
                           D.StatorLengthM * 0.5, D.StatorWidthM * 0.5, D.StatorHeightM * 0.5);
                    // The magnet face, on the INNER flank, where the train's
                    // reaction plate runs through the air gap.
                    AddBox(Out.Rubber, C - F.Lateral * (side * D.StatorWidthM * 0.5),
                           F.Tangent, F.Lateral, F.Up,
                           D.StatorLengthM * 0.48, 0.01, D.StatorHeightM * 0.4);
                }
            });
        }
    }
    return Out;
}
