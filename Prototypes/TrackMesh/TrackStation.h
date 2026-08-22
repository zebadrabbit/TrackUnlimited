// TrackUnlimited: the station as an OBJECT — platform, hazard stripe, airgates,
// operator cabinet.
//
// The device hardware (TrackDevices.h) is what sits BETWEEN the rails on a
// zone; this is what sits BESIDE them on a station zone, and it is a separate
// pass because it is a different shape: level, outboard, built for people.
// The same convention as the catwalk applies and for the same reason — a
// platform is a floor, so it is level across however the track is banked, and
// its outward direction is horizontal.
//
// From the reference photograph: a concrete slab with a block-wall face, a
// yellow stripe along its edge, one airgate per car, and a cabinet at the
// dispatch end. Stairs are not here; a platform at grade has none, and one in
// the air wants a support pass that knows about the ground, which this does
// not. ponytail: a single stripe colour rather than yellow/black hatching,
// which is a texture and there is no texture pipeline yet.
//
// Three buffers, three materials: concrete, painted steel, stripe.

#pragma once

#include "TrackDevices.h"
#include "TrackCatwalk.h"

#include <cmath>
#include <cstddef>
#include <vector>

struct FStationSpan
{
    double StartS = 0.0;
    double EndS = 0.0;
    bool bLeft = true;   // which side the platform is on, in the rider's terms
};

struct FStationSettings
{
    // The slab. Inboard edge is measured from the track centreline and must
    // clear the widest train; the top sits PlatformBelowHeartM under the
    // heartline, which puts it about at a car's floor.
    double InboardM = 0.95;
    double WidthM = 2.50;
    double PlatformBelowHeartM = 0.75;
    double SlabDepthM = 1.00;

    // The stripe along the inboard edge of the top.
    double StripeWidthM = 0.15;
    double StripeThickM = 0.012;

    // Airgates: one per GatePitchM (a car), GateHeightM tall, set back from the
    // edge so a gate can be stood behind.
    double GatePitchM = 3.0;
    double GateHeightM = 1.10;
    double GateSetbackM = 0.35;
    double PostDiameterM = 0.05;
    double RailDiameterM = 0.04;
    int Sides = 6;

    // The cabinet, at the dispatch end, against the outboard edge.
    double CabinetLengthM = 1.20;
    double CabinetWidthM = 0.60;
    double CabinetHeightM = 1.30;
};

struct FStationMesh
{
    FMeshBuffer Concrete;
    FMeshBuffer Steel;
    FMeshBuffer Stripe;

    std::size_t NumTriangles() const
    {
        return Concrete.NumTriangles() + Steel.NumTriangles() + Stripe.NumTriangles();
    }
};

inline FStationMesh BuildStations(const std::vector<FTrackFrame>& Frames,
                                  const std::vector<FStationSpan>& Spans,
                                  const FStationSettings& St = FStationSettings())
{
    using namespace DeviceDetail;
    using CatwalkDetail::DeckUp;
    using CatwalkDetail::FlatOutward;
    FStationMesh Out;
    if (Frames.size() < 2) { return Out; }

    std::vector<double> S(Frames.size(), 0.0);
    for (std::size_t k = 1; k < Frames.size(); ++k)
    {
        S[k] = S[k - 1] + Length(Frames[k].Position - Frames[k - 1].Position);
    }

    for (const FStationSpan& Span : Spans)
    {
        const double Len = Span.EndS - Span.StartS;
        if (Len <= 0.0) { continue; }
        const double Side = Span.bLeft ? 1.0 : -1.0;

        // ---- THE SLAB AND THE STRIPE, as rectangular sweeps. The ring's lateral
        // axis is always the LEFT horizontal, whichever side the platform is on,
        // so the sweep's winding is the same on both sides; the side goes into
        // the centre's offset only.
        std::vector<FRectRing> Slab, Stripe;
        for (std::size_t i = 0; i < Frames.size(); ++i)
        {
            if (S[i] < Span.StartS - 1e-6 || S[i] > Span.EndS + 1e-6) { continue; }
            const FTrackFrame& F = Frames[i];
            const FVec3 Left = FlatOutward(F, true);
            const FVec3 Top = F.Position - DeckUp() * St.PlatformBelowHeartM;
            const FVec3 Mid = Top + Left * (Side * (St.InboardM + St.WidthM * 0.5));
            Slab.push_back({Mid - DeckUp() * (St.SlabDepthM * 0.5), Left, DeckUp()});
            Stripe.push_back({Top + Left * (Side * (St.InboardM + St.StripeWidthM * 0.5))
                                  + DeckUp() * (St.StripeThickM * 0.5), Left, DeckUp()});
        }
        SweepRect(Out.Concrete, Slab, St.WidthM * 0.5, St.SlabDepthM * 0.5);
        SweepRect(Out.Stripe, Stripe, St.StripeWidthM * 0.5, St.StripeThickM * 0.5);

        // ---- THE AIRGATES: one per car, a panel of two posts and two rails
        // along the edge, with a gap between panels where the gate swings.
        const int N = static_cast<int>(std::floor(Len / St.GatePitchM));
        const double Start = Span.StartS + (Len - N * St.GatePitchM) * 0.5;
        for (int g = 0; g < N; ++g)
        {
            const double A = Start + g * St.GatePitchM + 0.15;
            const double B = Start + (g + 1) * St.GatePitchM - 0.15;
            const FTrackFrame Fa = FrameAt(Frames, S, A);
            const FTrackFrame Fb = FrameAt(Frames, S, B);
            auto Foot = [&](const FTrackFrame& F)
            {
                return F.Position - DeckUp() * St.PlatformBelowHeartM
                    + FlatOutward(F, true) * (Side * (St.InboardM + St.GateSetbackM));
            };
            const FVec3 Pa = Foot(Fa), Pb = Foot(Fb);
            const FVec3 Rise = DeckUp() * St.GateHeightM;
            SweepStrut(Out.Steel, Pa, Pa + Rise, Fa.Tangent, St.PostDiameterM * 0.5, St.Sides);
            SweepStrut(Out.Steel, Pb, Pb + Rise, Fb.Tangent, St.PostDiameterM * 0.5, St.Sides);
            SweepStrut(Out.Steel, Pa + Rise, Pb + Rise, DeckUp(), St.RailDiameterM * 0.5, St.Sides);
            SweepStrut(Out.Steel, Pa + Rise * 0.5, Pb + Rise * 0.5, DeckUp(), St.RailDiameterM * 0.5, St.Sides);
        }

        // ---- THE CABINET, at the dispatch end: the operator stands where the
        // train leaves. Against the outboard edge, so it is not in the gates.
        if (Len > St.CabinetLengthM + 1.0)
        {
            const FTrackFrame F = FrameAt(Frames, S, Span.EndS - St.CabinetLengthM * 0.5 - 0.5);
            const FVec3 Left = FlatOutward(F, true);
            const FVec3 Along = Normalised(Cross(Left, DeckUp()));   // horizontal travel direction
            const FVec3 C = F.Position - DeckUp() * St.PlatformBelowHeartM
                + Left * (Side * (St.InboardM + St.WidthM - St.CabinetWidthM * 0.5 - 0.1))
                + DeckUp() * (St.CabinetHeightM * 0.5);
            AddBox(Out.Steel, C, Along, Left, DeckUp(),
                   St.CabinetLengthM * 0.5, St.CabinetWidthM * 0.5, St.CabinetHeightM * 0.5);
        }
    }
    return Out;
}
