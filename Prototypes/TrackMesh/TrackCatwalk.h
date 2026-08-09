// TrackUnlimited Phase 4: the catwalk, and why it is not just a strip of deck.
// Plain C++17, no engine dependency, same conventions as TrackMesh.h.
//
// ===================== WHY THIS EXISTS AT ALL =====================
//
// `ETUWalkway` has been authored per segment since the evacuation model was
// built, derived into `FWalkwaySpan` on every rebuild, and consumed by
// `Evacuation.h` — which is where "5 holding blocks catwalked, 258 m of route,
// every train reachable" comes from. None of it has ever been drawn.
//
// So a safety system that decides whether a stopped train can be reached has
// been reasoning about a walkway nobody can see. Drawing it turns "every train is
// reachable" from a log line into something obvious at a glance, and a GAP in the
// catwalk into something you notice rather than something you are told.
//
// ===================== A CATWALK IMPLIES ITS HANDRAIL =====================
//
// One toggle, not two, and that is a safety shape rather than a convenience:
// a walkway without a rail is not a cheaper walkway, it is a fall hazard 40 m up.
// `ETUWalkway` was already this shape, and NoLimits 2 independently ships one
// checkbox reading "Left Railing and Catwalk" — the same conclusion from the
// other direction.
//
// TWO BUFFERS, because they are two materials. Deck plate and tube handrail do
// not look alike and NL2 gives them separate colour pickers; a mesh section is
// the unit of material in the engine, which is the same reason the track is three
// buffers rather than one.
//
// ===================== WHAT IS DELIBERATELY NOT HERE =====================
//
// ponytail: no brackets under the deck, no toe board, no railing lights, no
// transparent mode. The deck and the guardrail are the silhouette — the things
// that make a lift hill read as a lift hill — and each of the others is detail
// that costs geometry on every metre of every catwalked run. Railing lights in
// particular are a SHOW fixture that happens to be mounted here, and belong with
// the rest of the show layer rather than in a mesher.

#pragma once

#include "TrackMesh.h"
#include "../BlockSignal/Evacuation.h"

#include <cmath>
#include <cstdio>
#include <cstddef>
#include <string>
#include <vector>

struct FCatwalkSettings
{
    // Outboard of the rail, with a gap so the deck does not foul the wheels or
    // the train's swept width. Industrial access decks run 600-900 mm; 0.7 m is
    // mid-range and is a knob rather than a constant.
    double DeckWidthM = 0.70;
    double DeckThicknessM = 0.04;

    // From the outer rail's centreline to the inboard edge of the deck. The
    // train has to clear it, and somebody has to be able to stand on it while a
    // train goes past — which is the whole reason a catwalk is outboard.
    double InboardGapM = 0.25;

    // How far below the rail centreline the deck surface sits. Roughly at the
    // spine's top, so somebody walking is level with the track rather than
    // wading through it.
    double DeckDropM = 0.18;

    // GUARDRAIL, at the heights industrial practice uses: a top rail at about
    // 1.1 m and a mid rail halfway. Two rails rather than three because the third
    // is a toe board, which is a plate rather than a tube.
    double RailHeightM = 1.05;
    double PostSpacingM = 2.50;
    double PostDiameterM = 0.05;
    double RailDiameterM = 0.04;
    int RailSides = 6;

    // ===================== THE ONE THAT IS A JUDGEMENT =====================
    //
    // A catwalk follows the track's own frame, so on banked track it banks with
    // it — which is correct, and is exactly why real rides put catwalks on lift
    // hills and brake runs and not through banked turns.
    //
    // Past some angle it stops being walkable, and that is REPORTED rather than
    // refused: the author asked for a catwalk there and may have a reason, and a
    // mesher that silently dropped it would leave a gap in an evacuation route
    // with nothing saying why. 25 degrees is a steep ramp somebody can still get
    // along; beyond it they are climbing.
    double MaxWalkableBankDeg = 25.0;
};

struct FCatwalkMesh
{
    FMeshBuffer Deck;
    FMeshBuffer Rail;

    // REPORTED, NEVER REPAIRED, like every other finding in this project.
    std::vector<FMeshFinding> Finding;

    std::size_t NumVertices() const { return Deck.Position.size() + Rail.Position.size(); }
    std::size_t NumTriangles() const { return (Deck.Index.size() + Rail.Index.size()) / 3; }
};

namespace CatwalkDetail
{

inline bool SideWanted(EWalkway Side, bool bLeft)
{
    if (Side == EWalkway::Both) { return true; }
    return bLeft ? Side == EWalkway::Left : Side == EWalkway::Right;
}

// Is this arc length inside any span that wants this side?
inline bool Wanted(const std::vector<FWalkwaySpan>& Spans, double S, bool bLeft)
{
    for (const FWalkwaySpan& W : Spans)
    {
        if (S >= W.StartS && S <= W.EndS && SideWanted(W.Side, bLeft)) { return true; }
    }
    return false;
}

// The deck's inboard and outboard edge points for one frame and one side.
//
// +LATERAL IS THE RIDER'S LEFT, which is the convention the whole project uses
// and the one thing here that silently mirrors an entire ride if it is wrong.
inline void DeckEdges(const FTrackFrame& F, const FTrackProfile& Profile,
                      const FCatwalkSettings& S, bool bLeft, FVec3& Inner, FVec3& Outer)
{
    const double Sign = bLeft ? 1.0 : -1.0;
    const FVec3 Out = F.Lateral * Sign;
    const FVec3 RailCentre = F.Position;
    const double Start = Profile.Gauge * 0.5 + S.InboardGapM;
    const FVec3 Base = RailCentre - F.Up * S.DeckDropM;
    Inner = Base + Out * Start;
    Outer = Base + Out * (Start + S.DeckWidthM);
}

} // namespace CatwalkDetail

// ===================== BUILD =====================
//
// Takes FRAMES, exactly as `BuildTrackMesh` does, and never an FTrack — so the
// O(n^2) trap is unreachable here for the same reason it is unreachable there.
inline FCatwalkMesh BuildCatwalks(const std::vector<FTrackFrame>& Frames,
                                  const std::vector<FWalkwaySpan>& Spans,
                                  const FTrackProfile& Profile,
                                  const FCatwalkSettings& Settings = FCatwalkSettings())
{
    using namespace CatwalkDetail;
    FCatwalkMesh Out;
    if (Frames.size() < 2) { return Out; }

    // ARC LENGTH IS THE CALLER'S TO ACCUMULATE — a frame carries a position and a
    // basis and not where it is along the track, so this walks it the same way
    // PlanSupports does. Summing chord lengths rather than integrating: the walk
    // is already dense, and the two agree to well under the spacing.
    std::vector<double> S(Frames.size(), 0.0);
    for (std::size_t k = 1; k < Frames.size(); ++k)
    {
        S[k] = S[k - 1] + Length(Frames[k].Position - Frames[k - 1].Position);
    }

    const double CosLimit = std::cos(Settings.MaxWalkableBankDeg * 3.14159265358979323846 / 180.0);

    for (int Side = 0; Side < 2; ++Side)
    {
        const bool bLeft = Side == 0;

        // Runs of consecutive wanted frames, so a span boundary ends a strip
        // rather than stitching a quad across the gap between two catwalks.
        std::size_t i = 0;
        while (i + 1 < Frames.size())
        {
            if (!Wanted(Spans, S[i], bLeft)) { ++i; continue; }
            const std::size_t Begin = i;
            while (i + 1 < Frames.size() && Wanted(Spans, S[i + 1], bLeft)) { ++i; }
            const std::size_t End = i;
            ++i;
            if (End <= Begin) { continue; }

            // ---- THE DECK, as a quad strip between the two edges.
            const std::uint32_t Base = static_cast<std::uint32_t>(Out.Deck.Position.size());
            double WorstCos = 1.0;
            double WorstS = S[Begin];
            bool bInverted = false;
            double InvertedS = 0.0;

            for (std::size_t k = Begin; k <= End; ++k)
            {
                FVec3 Inner, Outer;
                DeckEdges(Frames[k], Profile, Settings, bLeft, Inner, Outer);

                // A CLOSED SLAB, NOT A ONE-SIDED STRIP. A catwalk is looked at
                // from UNDERNEATH more often than from above — that is what a lift
                // hill is from the queue — and a single quad strip is invisible
                // from below under backface culling. Four corners per ring, walls
                // and end caps below, which also makes the signed-volume check
                // available: it is what found the track mesh inside out on the
                // first try.
                //
                // THE DECK'S OWN UP is the frame's, so it banks with the track —
                // right, and why the walkability check below exists.
                const FVec3 N = Frames[k].Up;
                const FVec3 Down = N * Settings.DeckThicknessM;
                const double V = S[k] / 4.0;
                Out.Deck.Position.push_back(Inner);
                Out.Deck.Normal.push_back(N);
                Out.Deck.UV.push_back({0.0, V});
                Out.Deck.Position.push_back(Outer);
                Out.Deck.Normal.push_back(N);
                Out.Deck.UV.push_back({1.0, V});
                Out.Deck.Position.push_back(Outer - Down);
                Out.Deck.Normal.push_back(N * -1.0);
                Out.Deck.UV.push_back({1.0, V});
                Out.Deck.Position.push_back(Inner - Down);
                Out.Deck.Normal.push_back(N * -1.0);
                Out.Deck.UV.push_back({0.0, V});

                // Up-ness of the deck, which is the frame's Up against world up.
                if (Frames[k].Up.Z < WorstCos)
                {
                    WorstCos = Frames[k].Up.Z;
                    WorstS = S[k];
                }
                if (Frames[k].Up.Z <= 0.0 && !bInverted)
                {
                    bInverted = true;
                    InvertedS = S[k];
                }
            }

            const std::size_t Rings = End - Begin + 1;
            auto Quad = [&](std::uint32_t A, std::uint32_t B, std::uint32_t C, std::uint32_t D)
            {
                Out.Deck.Index.push_back(A); Out.Deck.Index.push_back(B); Out.Deck.Index.push_back(C);
                Out.Deck.Index.push_back(A); Out.Deck.Index.push_back(C); Out.Deck.Index.push_back(D);
            };
            for (std::size_t k = 0; k + 1 < Rings; ++k)
            {
                const std::uint32_t R0 = Base + static_cast<std::uint32_t>(k * 4);
                const std::uint32_t R1 = R0 + 4;
                // Ring order is inner-top, outer-top, outer-bottom, inner-bottom,
                // so the four walls are consecutive pairs and every one is wound
                // outward — asserted by signed volume rather than by reading this.
                Quad(R0 + 0, R1 + 0, R1 + 1, R0 + 1);   // top
                Quad(R0 + 1, R1 + 1, R1 + 2, R0 + 2);   // outer edge
                Quad(R0 + 2, R1 + 2, R1 + 3, R0 + 3);   // underside
                Quad(R0 + 3, R1 + 3, R1 + 0, R0 + 0);   // inboard edge
            }
            // CAPPED AT BOTH ENDS, because a catwalk run has two free ends by
            // definition — the same reason a strut is capped, and the same defect
            // if it is not: a length of open channel with both ends in shot.
            //
            // BOTH OF THESE WERE REVERSED, and watertightness did not notice —
            // edge sharing is blind to orientation, so a mesh with two inside-out
            // caps counts every edge exactly twice and passes. The signed volume
            // caught it: a 24 m run reported 8 m of slab. That is the whole reason
            // this project checks volume rather than winding.
            const std::uint32_t First = Base;
            const std::uint32_t Last = Base + static_cast<std::uint32_t>((Rings - 1) * 4);
            Quad(First + 0, First + 1, First + 2, First + 3);
            Quad(Last + 3, Last + 2, Last + 1, Last + 0);

            // ---- THE GUARDRAIL: posts at a spacing in METRES, and two rails.
            double NextPost = S[Begin];
            std::vector<FVec3> TopPts, MidPts;
            for (std::size_t k = Begin; k <= End; ++k)
            {
                FVec3 Inner, Outer;
                DeckEdges(Frames[k], Profile, Settings, bLeft, Inner, Outer);
                const FVec3 Up = Frames[k].Up;
                TopPts.push_back(Outer + Up * Settings.RailHeightM);
                MidPts.push_back(Outer + Up * (Settings.RailHeightM * 0.5));

                if (S[k] + 1e-9 >= NextPost)
                {
                    SweepStrut(Out.Rail, Outer, Outer + Up * Settings.RailHeightM,
                               Frames[k].Tangent, Settings.PostDiameterM * 0.5,
                               Settings.RailSides);
                    NextPost = S[k] + Settings.PostSpacingM;
                }
            }
            // ---- THE RAILS AS ONE CONTINUOUS TUBE EACH, not a strut per frame.
            //
            // Strut-per-frame was the first version and it is the O(n) mistake in
            // a new coat: a capped tube between every pair of samples put 5088
            // triangles on twenty-five metres of handrail, almost all of them caps
            // buried inside the next segment. A handrail is one tube; SweepTube is
            // what the track's own rails already use, and it caps only the ends.
            auto SweepRail = [&](const std::vector<FVec3>& Pts)
            {
                if (Pts.size() < 2) { return; }
                std::vector<FTubeRing> Rings;
                Rings.reserve(Pts.size());
                for (std::size_t k = 0; k < Pts.size(); ++k)
                {
                    const FTrackFrame& F = Frames[Begin + k];
                    Rings.push_back({Pts[k], F.Lateral, F.Up, S[Begin + k]});
                }
                SweepTube(Out.Rail, Rings, Settings.RailDiameterM * 0.5,
                          Settings.RailSides, 1.0, true, true);
            };
            SweepRail(TopPts);
            SweepRail(MidPts);

            // ---- AND WHAT IS WRONG WITH IT, said rather than fixed.
            if (bInverted)
            {
                // A CATWALK ON THE UNDERSIDE OF INVERTED TRACK IS NONSENSE, and it
                // is the same refusal shape supports needed at the same place —
                // except that here the author explicitly asked for it, so it is
                // drawn and reported rather than dropped.
                FMeshFinding F;
                F.S = InvertedS;
                F.What = "catwalk runs through inverted track: it is on the underside there";
                Out.Finding.push_back(F);
            }
            else if (WorstCos < CosLimit)
            {
                const double Deg = std::acos(WorstCos < -1.0 ? -1.0 : (WorstCos > 1.0 ? 1.0 : WorstCos))
                                   * 180.0 / 3.14159265358979323846;
                FMeshFinding F;
                F.S = WorstS;
                char Buf[128];
                std::snprintf(Buf, sizeof(Buf),
                    "catwalk is banked %.0f degrees here: too steep to walk on, "
                    "and it is an evacuation route", Deg);
                F.What = Buf;
                Out.Finding.push_back(F);
            }
        }
    }
    return Out;
}
