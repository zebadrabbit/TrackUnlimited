// TrackUnlimited Phase 4: getting the track off the ground.
// Plain C++17, no engine dependency, same conventions as TrackMesh.h.
//
// The card calls this "a genuinely hard sub-problem, treated as its own milestone
// rather than an assumed detail", and that is the right framing. A coaster's
// supports are most of what you actually SEE — the track is a thin ribbon and the
// structure under it is the silhouette — and placing them is a constraint problem
// with no single right answer.
//
// So this is NOT an attempt to place them perfectly. It is the layer that:
//
//   - places them where the obvious rules say they go,
//   - REFUSES to place one where it would be wrong, and says why,
//   - leaves a person the final say, which is the same conclusion the evacuation
//     walkways reached ("we can guess where one SHOULD go; it may be
//     architecturally inaccessible or dangerous").
//
// ===================== THE FOUR RULES THAT MATTER =====================
//
// 1. SPACING IS A SPAN, NOT A SAMPLE COUNT. Steel track spans a distance, and how
//    far depends on the section and the load. Placing one support every N samples
//    puts them 0.5 m apart on a tight helix and 40 m apart on a straight.
//
// 2. A COLUMN MUST NOT PASS THROUGH THE TRACK. The classic failure and the one
//    that looks worst: a footer under a high point, with the column running
//    straight up through a lower piece of track it happens to sit under. Every
//    layout with a helix over a straight has this.
//
// 3. IT MUST REACH THE GROUND. Track below ground level is in a trench or a
//    tunnel and gets no column at all, rather than one of negative length.
//
// 4. IT ATTACHES TO THE SPINE, AND THE SPINE MOVES. `SpineDrop` puts it below the
//    rails, so through an INVERSION the spine is ABOVE the track and a column
//    coming up from below would have to pass through the rails to reach it. That
//    is where a naive placer produces its most obviously wrong geometry, and it is
//    detected rather than guessed at.
//
// Units: metres. World Z is up, as everywhere in these prototypes.

#pragma once

#include "TrackMesh.h"

#include <cmath>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

// Where the ground is. A function rather than a number, because Phase 5 adds
// terrain and a flat default should not become an assumption baked into every
// support ever placed.
//
// ponytail: flat by default at z = 0. Swap in a heightfield lookup when there is
// one; nothing here needs to change.
using FGroundHeight = std::function<double(double X, double Y)>;

inline FGroundHeight FlatGround(double Z = 0.0)
{
    return [Z](double, double) { return Z; };
}

// One column: a footer on the ground and a top where it meets the track.
struct FSupportLeg
{
    FVec3 Foot;
    FVec3 Top;
    double Diameter = 0.25;
    double S = 0.0;          // arc length of the track point it carries

    double Height() const { return Top.Z - Foot.Z; }
};

// Why a support was NOT placed. Reported, never repaired — the fix is a track
// edit or a person putting one somewhere this cannot see, and a placer that
// quietly skipped would leave a ride floating with no explanation.
// WHY a support could not go here, as a KIND rather than only as a sentence.
//
// The kind is what merging is keyed on. Keying on the message instead looked
// right and was not: every fouling finding names the metre it fouls at, so no two
// messages ever matched and a 48 m stretch that can take nothing came out as
// seven separate problems. Same flooding the acceleration envelope had, in a
// subtler form.
enum class ESupportProblem
{
    BelowGround,
    Inverted,
    FoulsTrack,
};

struct FSupportFinding
{
    ESupportProblem Kind = ESupportProblem::FoulsTrack;
    double S = 0.0;        // where the run of this problem starts
    double EndS = 0.0;     // and where it ends; equal to S for a single point
    std::string What;      // the first instance's detail, which names a place

    double LengthM() const { return EndS - S; }
};

struct FSupportPlan
{
    std::vector<FSupportLeg> Leg;

    // Findings, MERGED INTO RUNS. A long inverted stretch is one problem with a
    // length, not forty copies of the same sentence — the same lesson the
    // acceleration envelope learned when every ride reported dozens of wobbles
    // over the band it rests at. A report that floods is worse than no report,
    // because somebody stops reading it.
    std::vector<FSupportFinding> Finding;

    double LongestGapM = 0.0;   // the biggest unsupported run, which is the number
                                // an engineer would actually ask for
};

struct FSupportSettings
{
    // THE SPAN, in metres of arc length. Real steel coaster track carries roughly
    // 6-12 m between bents depending on section and load; 9 m sits mid-range and
    // is a knob rather than a constant for the same reason every figure in
    // TrackProfile is.
    double SpanM = 9.0;

    // A column shorter than this is not a column — the track is essentially on the
    // ground and wants a footer plate, not a tower.
    double MinHeightM = 0.6;

    // How close a column may pass to any other part of the track before it is
    // refused. The track's own swept width plus a working clearance: somebody has
    // to be able to walk past it, and a train has to clear it.
    double ClearanceM = 1.5;

    double LegDiameterM = 0.25;
};

// Shortest distance from a point to a line segment. The whole of the
// self-intersection test, and the reason it is a segment rather than a line is
// that a column has a top and a bottom — track far above the attachment point is
// not fouled by a column that stops below it.
inline double DistancePointToSegment(const FVec3& P, const FVec3& A, const FVec3& B)
{
    const FVec3 AB = B - A;
    const double LenSq = Dot(AB, AB);
    if (LenSq <= 0.0) { return Length(P - A); }
    double T = Dot(P - A, AB) / LenSq;
    T = T < 0.0 ? 0.0 : (T > 1.0 ? 1.0 : T);
    return Length(P - (A + AB * T));
}

inline std::string FormatMetres(double V)
{
    const long long Tenths = static_cast<long long>(V * 10.0 + 0.5);
    return std::to_string(Tenths / 10) + "." + std::to_string(Tenths % 10) + " m";
}

// Add a finding, EXTENDING the previous one if it is the same problem continuing.
// Two consecutive refusals a span apart are one stretch of track that cannot take
// a support, and reporting them separately buries the one that is different.
inline void Report(FSupportPlan& Plan, ESupportProblem Kind, double S, const std::string& What)
{
    if (!Plan.Finding.empty() && Plan.Finding.back().Kind == Kind)
    {
        // The DETAIL is the first instance's, deliberately. A run's message should
        // send somebody to where the trouble starts, not to wherever it happened
        // to end.
        Plan.Finding.back().EndS = S;
        return;
    }
    Plan.Finding.push_back({Kind, S, S, What});
}

// ===================== THE PLACER =====================
//
// Takes the frames the mesher already walked, so it costs nothing extra and
// cannot go quadratic — the same split TrackMesh.h is built on.
inline FSupportPlan PlanSupports(const std::vector<FTrackFrame>& Path,
                                 double HeartlineHeight,
                                 const FTrackProfile& Profile,
                                 const FSupportSettings& Settings,
                                 const FGroundHeight& Ground)
{
    FSupportPlan Plan;
    if (Path.size() < 2 || !(Settings.SpanM > 0.0))
    {
        return Plan;
    }

    // Arc length along the walked path, accumulated once.
    std::vector<double> S(Path.size(), 0.0);
    for (std::size_t i = 1; i < Path.size(); ++i)
    {
        S[i] = S[i - 1] + Length(Path[i].Position - Path[i - 1].Position);
    }
    const double Total = S.back();

    // The spine centreline, which is what a column actually attaches to. Computed
    // once because the self-intersection test below needs all of it.
    std::vector<FVec3> Spine(Path.size());
    for (std::size_t i = 0; i < Path.size(); ++i)
    {
        Spine[i] = CrossSectionAt(Path[i], HeartlineHeight, Profile).SpineCentre;
    }

    double LastPlaced = -1e9;
    double LastSupportedS = 0.0;

    for (std::size_t i = 0; i < Path.size(); ++i)
    {
        // RULE 1: SPACING IS A SPAN. Metres of track between bents, not samples —
        // one support every N samples is 0.5 m apart on a tight helix and 40 m
        // apart on a straight.
        if (S[i] - LastPlaced < Settings.SpanM) { continue; }

        const FVec3& Attach = Spine[i];
        const double GroundZ = Ground(Attach.X, Attach.Y);

        // RULE 3: IT MUST REACH THE GROUND. Track at or below ground level is in a
        // trench or a tunnel and gets no column, rather than one of negative
        // length.
        if (Attach.Z - GroundZ < Settings.MinHeightM)
        {
            if (Attach.Z < GroundZ)
            {
                Report(Plan, ESupportProblem::BelowGround, S[i],
                       "track is below ground: trench or tunnel, no support");
            }
            // At grade and needing only a footer plate is not a finding — it is
            // the normal case for a station, and reporting it would bury the real
            // ones.
            LastPlaced = S[i];
            LastSupportedS = S[i];
            continue;
        }

        // RULE 4: THE SPINE MOVES. SpineDrop puts it below the rails, so through an
        // INVERSION the spine is ABOVE the track and a column coming up from
        // underneath would have to pass through the rails to reach it. That is
        // where a naive placer makes its most obviously wrong geometry.
        //
        // Detected from the frame rather than from the geometry: the spine is
        // above the rail centre exactly when the track's own up vector points
        // down.
        if (Path[i].Up.Z < 0.0)
        {
            Report(Plan, ESupportProblem::Inverted, S[i],
                   "track is inverted: a column would pass through the rails");
            LastPlaced = S[i];
            continue;
        }

        // RULE 2: A COLUMN MUST NOT PASS THROUGH THE TRACK. The classic failure
        // and the one that looks worst — a footer under a high point with the
        // column running straight up through a lower piece of track it happens to
        // sit under. Every layout with a helix over a straight has this.
        //
        // Tested against the spine centreline of every OTHER part of the track,
        // excluding the neighbourhood of the attachment itself, since a column
        // obviously passes close to the track it is holding up.
        const FVec3 Foot{Attach.X, Attach.Y, GroundZ};
        bool bFouls = false;
        double FoulAt = 0.0;
        for (std::size_t j = 0; j < Path.size(); ++j)
        {
            if (std::fabs(S[j] - S[i]) < Settings.SpanM) { continue; }
            const double D = DistancePointToSegment(Spine[j], Foot, Attach);
            if (D < Settings.ClearanceM)
            {
                bFouls = true;
                FoulAt = S[j];
                break;
            }
        }
        if (bFouls)
        {
            Report(Plan, ESupportProblem::FoulsTrack, S[i],
                   "a column would foul the track at " + FormatMetres(FoulAt)
                   + " — needs a cantilever, an A-frame, or a person");
            LastPlaced = S[i];
            continue;
        }

        FSupportLeg L;
        L.Foot = Foot;
        L.Top = Attach;
        L.Diameter = Settings.LegDiameterM;
        L.S = S[i];
        Plan.Leg.push_back(L);

        // THE NUMBER AN ENGINEER WOULD ACTUALLY ASK FOR: the longest run with
        // nothing under it. A placer that reported only what it placed would say
        // nothing about the 60 m of unsupported track its refusals left behind.
        const double Gap = S[i] - LastSupportedS;
        if (Gap > Plan.LongestGapM) { Plan.LongestGapM = Gap; }
        LastSupportedS = S[i];
        LastPlaced = S[i];
    }

    const double TailGap = Total - LastSupportedS;
    if (TailGap > Plan.LongestGapM) { Plan.LongestGapM = TailGap; }
    return Plan;
}

// ponytail: single vertical columns only. No bents (the A-frame pair that carries
// a banked turn), no diagonal bracing, no cantilevers, no shared footers where two
// legs land within a metre of each other. Each is a real thing a real coaster has
// and each is a placement problem of its own — and every one of them is easier to
// design once something can already say WHERE a plain column goes and where it
// cannot. The refusals above are exactly the list of cases those would solve.
