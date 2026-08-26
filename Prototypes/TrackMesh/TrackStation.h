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
    // ONE CABINET PER PLATFORM, NOT PER POSITION. A four-position platform is
    // four of these spans and one console; the caller marks the span the
    // console belongs to, which is the dispatch end of the contiguous run.
    bool bCabinet = true;
    // WHERE EACH AIRGATE IS, 0 = open, 1 = closed, one per gate along the span
    // in travel order — FCommandedBank::GroupPosition, read by the caller. Empty
    // means closed, which is the fixed panel this drew before gates moved. A
    // gate not listed is closed too. The mesh SUBSCRIBES; nothing here commands.
    std::vector<double> GatePositions;
    // Where the parked train's NOSE is — the zone's stop mark — so the gates
    // are laid back from it one per row and face the seats. Negative: unknown,
    // and the rows are centred in the span instead.
    double NoseS = -1.0;
};

struct FStationSettings
{
    // The slab. Inboard edge is measured from the track centreline and must
    // clear the widest train; the top sits PlatformBelowHeartM under the
    // heartline, which puts it about at a car's floor.
    double InboardM = 0.95;
    // Wide enough for an OPERATOR'S STRIP between the stripe and the fence
    // line (GateSetbackM) and queue bays behind the fence. 2.5 m put the fence
    // a third of a metre off the stripe, with nowhere for anyone to stand.
    double WidthM = 4.00;
    double PlatformBelowHeartM = 0.75;
    double SlabDepthM = 1.00;

    // The stripe along the inboard edge of the top.
    double StripeWidthM = 0.15;
    double StripeThickM = 0.012;

    // AIRGATES AND THE FENCE BETWEEN THEM. From the reference shots
    // (W:\screenshots, 2026-08-25 2047xx, not committed): the platform edge is
    // a FIXED fence of posts, two rails and vertical bars along its whole
    // length, and an airgate is a NARROW gate in it — one per seat row, about
    // 0.9 m, hinged on a post and swinging into the queue bay behind it. The
    // first version drew one whole fence section per car and swung THAT, 2.7 m
    // of it, off the far edge of a 2.5 m platform.
    //
    // Rows are laid back from the PARKED train's nose (FStationSpan::NoseS,
    // the stop mark) at GatePitchM / RowsPerCar, so a gate faces the seat it
    // serves; without a nose the rows are centred in the span. One row per
    // car is the seat model's own default (Seat.h).
    double GatePitchM = 3.0;        // a car
    int RowsPerCar = 1;
    double GateWidthM = 0.90;
    double GateHeightM = 1.10;
    // THE FENCE LINE, back from the stripe: the operator's strip. From the
    // reference shots, about a metre and a half of platform between the stripe
    // and the gates, which is where the crew walks the train and where an open
    // gate swings INTO.
    double GateSetbackM = 1.60;
    double PostDiameterM = 0.05;
    double RailDiameterM = 0.04;
    double BarDiameterM = 0.02;
    double BarSpacingM = 0.15;
    int Sides = 6;
    // A gate is hinged on its upstream post and swings TOWARD THE TRAIN, away
    // from the guests: an automated gate does not swing into a queue of
    // people, and the strip between gate and train is CONTROLLED space — the
    // harnesses stay locked until the gates have opened, so nobody is in it.
    // (The first version swung onto the platform, into the queue.) Fully open
    // is this many degrees off the fence line.
    double GateSwingDeg = 90.0;

    // The cabinet, at the dispatch end, against the outboard edge.
    double CabinetLengthM = 1.20;
    double CabinetWidthM = 0.60;
    double CabinetHeightM = 1.30;
};

// THE GATE LAYOUT, one answer for the mesher and for whoever maps gates onto a
// bank's sensed sections: gate centres along the span, in travel order. A row
// is GatePitchM / RowsPerCar; from a known nose the rows are laid back from it
// and only those wholly inside the span are gates; otherwise as many rows as
// fit are centred in the span.
inline std::vector<double> StationGateCentres(const FStationSpan& Span, const FStationSettings& St)
{
    std::vector<double> Out;
    const int Rows = St.RowsPerCar < 1 ? 1 : St.RowsPerCar;
    const double Pitch = St.GatePitchM / Rows;
    const double Len = Span.EndS - Span.StartS;
    if (!(Pitch > 0.0) || Len < St.GateWidthM) { return Out; }
    if (Span.NoseS >= 0.0)
    {
        for (int k = 0; k < 1000; ++k)
        {
            const double C = Span.NoseS - (k + 0.5) * Pitch;
            if (C - St.GateWidthM * 0.5 < Span.StartS) { break; }
            if (C + St.GateWidthM * 0.5 <= Span.EndS) { Out.insert(Out.begin(), C); }
        }
        return Out;
    }
    const int N = static_cast<int>(std::floor(Len / Pitch));
    const double Start = Span.StartS + (Len - N * Pitch) * 0.5;
    for (int g = 0; g < N; ++g) { Out.push_back(Start + (g + 0.5) * Pitch); }
    return Out;
}

inline int StationGateCount(const FStationSpan& Span, const FStationSettings& St)
{
    return static_cast<int>(StationGateCentres(Span, St).size());
}

// A GATE OPENS ONLY ONTO A CAR. A gate whose row has nothing parked at it is
// an open door onto the track, whatever the bank was commanded -- the
// developer's platform rule (2026-08-26), found on the small-batch preset:
// a 6 m train on a 10 m position has three gates and two cars, and the third
// gate swung open onto nothing. True whether a train never reaches the row or
// parks short of it. The row is served when its centre lies within a train's
// span, measured the short way round on a circuit. Stands in for the
// car-in-position sensor a real gate section has; the caller supplies the
// spans, this decides nothing about the bank.
inline bool StationGateServed(double CentreS, double RearS, double FrontS,
                              bool bCircuit, double TotalLength)
{
    if (!bCircuit || !(TotalLength > 0.0))
    {
        return CentreS >= RearS - 1e-9 && CentreS <= FrontS + 1e-9;
    }
    auto Wrap = [TotalLength](double S)
    {
        S = std::fmod(S, TotalLength);
        return S < 0.0 ? S + TotalLength : S;
    };
    const double Len = Wrap(FrontS - RearS);
    return Wrap(CentreS - RearS) <= Len + 1e-9;
}

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

        // ---- THE FENCE AND THE AIRGATES IN IT. Along the whole edge line, a
        // fixed fence: a post at each end and at each gate opening, a top and a
        // mid rail between posts, vertical bars every BarSpacingM. In each
        // opening, one NARROW gate per seat row — the same posts, rails and bars
        // as a fence panel, GateWidthM wide — hinged on the opening's upstream
        // post and swinging TOWARD THE TRAIN, into the operator's strip, by
        // (1 - position) of the full swing. Closed, it closes the fence line;
        // open, it lies across the strip and stays clear of the train; jammed,
        // it stays wherever its bank says.
        //
        // Same struts in the same order whatever the angles, which is what
        // lets the caller UPDATE the steel buffer's vertices in place each
        // frame rather than recreate the section.
        auto EdgeFoot = [&](double At)
        {
            const FTrackFrame F = FrameAt(Frames, S, At);
            return F.Position - DeckUp() * St.PlatformBelowHeartM
                + FlatOutward(F, true) * (Side * (St.InboardM + St.GateSetbackM));
        };
        const FVec3 Rise = DeckUp() * St.GateHeightM;
        const FTrackFrame F0 = FrameAt(Frames, S, Span.StartS);
        const FVec3 TowardTrain = FlatOutward(F0, true) * (-Side);
        // A panel between two feet, in a given direction for its free end: post
        // at the far end only (the near post belongs to whoever came before),
        // two rails, and bars from the deck to the top rail.
        auto Panel = [&](const FVec3& From, const FVec3& To, bool bFarPost, double PostR)
        {
            if (bFarPost)
            {
                SweepStrut(Out.Steel, To, To + Rise, F0.Tangent, PostR, St.Sides);
            }
            const FVec3 Run = To - From;
            const double RunLen = Length(Run);
            if (RunLen <= 2.0 * PostR) { return; }
            // Rails end on the posts' FACES, not their axes: two rails meeting
            // on a post with identical end rings weld into one open tube, and
            // the signed-volume check says so. It is also where rails end.
            const FVec3 Dir = Run * (1.0 / RunLen);
            const FVec3 A = From + Dir * PostR;
            const FVec3 B = To - Dir * PostR;
            SweepStrut(Out.Steel, A + Rise, B + Rise, DeckUp(), St.RailDiameterM * 0.5, St.Sides);
            SweepStrut(Out.Steel, A + Rise * 0.5, B + Rise * 0.5, DeckUp(), St.RailDiameterM * 0.5, St.Sides);
            for (int b = 1; b * St.BarSpacingM < RunLen - PostR; ++b)
            {
                const FVec3 P = From + Dir * (b * St.BarSpacingM);
                SweepStrut(Out.Steel, P, P + Rise, F0.Tangent, St.BarDiameterM * 0.5, St.Sides);
            }
        };

        const std::vector<double> Centres = StationGateCentres(Span, St);
        const double Hw = St.GateWidthM * 0.5;
        double At = Span.StartS;
        SweepStrut(Out.Steel, EdgeFoot(At), EdgeFoot(At) + Rise, F0.Tangent, St.PostDiameterM * 0.5, St.Sides);
        for (std::size_t g = 0; g < Centres.size(); ++g)
        {
            const double HingeS = Centres[g] - Hw;
            const double LatchS = Centres[g] + Hw;
            // The fence up to this opening, ending on the hinge post.
            Panel(EdgeFoot(At), EdgeFoot(HingeS), true, St.PostDiameterM * 0.5);

            // THE GATE, hinged on that post. Its closed line runs to the latch
            // post less a gap; its swing goes onto the platform.
            const double Position = g < Span.GatePositions.size() ? Span.GatePositions[g] : 1.0;
            const double Open = Position < 0.0 ? 1.0 : Position > 1.0 ? 0.0 : 1.0 - Position;
            const double Theta = Open * St.GateSwingDeg * 3.14159265358979323846 / 180.0;
            const FVec3 Hinge = EdgeFoot(HingeS);
            FVec3 Along = EdgeFoot(LatchS - 0.06) - Hinge;
            Along = Along - DeckUp() * Dot(Along, DeckUp());
            const FVec3 Free = Hinge + Along * std::cos(Theta) + TowardTrain * (std::sin(Theta) * Length(Along));
            Panel(Hinge, Free, true, St.PostDiameterM * 0.5);

            // And the latch post, where the fence resumes.
            SweepStrut(Out.Steel, EdgeFoot(LatchS), EdgeFoot(LatchS) + Rise, F0.Tangent, St.PostDiameterM * 0.5, St.Sides);
            At = LatchS;
        }
        // The fence from the last opening to the end of the span.
        Panel(EdgeFoot(At), EdgeFoot(Span.EndS), true, St.PostDiameterM * 0.5);

        // ---- THE CABINET, at the dispatch end: the operator stands where the
        // train leaves. Against the outboard edge, so it is not in the gates.
        if (Span.bCabinet && Len > St.CabinetLengthM + 1.0)
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
