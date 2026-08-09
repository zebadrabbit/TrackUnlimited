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

// ===================== EVERY LEG LANDS ON SOMETHING =====================
//
// A column does not stop in the dirt. It lands on a concrete spread footing, and
// the track that is too low for a column still lands on one — the MinHeightM rule
// above already says so in words ("the track is essentially on the ground and
// wants a footer plate, not a tower") and then placed nothing at all.
//
// That was visible the moment the legs were drawn: the lowest run of a layout
// floated with no column and no pad, which is the one part of a real coaster
// nobody has ever seen unsupported.
//
// A SEPARATE LIST RATHER THAN A FIELD ON THE LEG, because the two do not
// correspond one to one: at-grade track has a footing and NO leg, which is the
// case that prompted this. Keeping them apart also leaves `Plan.Leg` meaning
// exactly what it meant before — the assertion that at-grade track produces no
// column and no finding is still true and still bites.
struct FSupportFooting
{
    // The centre of the TOP face. For a column that is the ground under its foot;
    // for at-grade track it is the track's own attachment point, because the pad
    // IS the support there.
    FVec3 Top;
    double Width = 1.0;       // across, and a footing is much wider than its column
    double Thickness = 0.45;  // down from Top

    FVec3 Bottom() const { return FVec3{Top.X, Top.Y, Top.Z - Thickness}; }
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

    // ONE PER LEG, PLUS THE ONES WITH NO LEG. A column lands on a footing, and
    // track too low for a column lands on one directly -- which is what the
    // MinHeightM rule has always said in words and never produced.
    std::vector<FSupportFooting> Footing;

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
    // THE SPAN, in metres of arc length.
    //
    // CITED RATHER THAN RECALLED. This said "roughly 6-12 m" from general
    // knowledge until Self (2024) gave it a source: that review reports a MAXIMUM
    // SPACING OF FORTY FEET (~12.2 m) as reference practice, citing Hunt (2018).
    // 9 m therefore sits inside a figure somebody can check rather than inside a
    // recollection, and it is still a knob for the same reason every figure in
    // TrackProfile is.
    //
    //   Self, Ian. Parametric Design and Optimization of Roller Coaster Support
    //   Structures Considering Sustainability and Maintenance. MS thesis,
    //   Architectural Engineering, Penn State, defended 27 February 2024.
    //   https://etda.libraries.psu.edu/catalog/27672izs5144
    //   Docs/REFERENCES.md records what was and was not taken from it.
    //
    // ponytail: UNIFORM SPACING, and the same source says that is not what a real
    // design does -- its topology optimisation clustered supports where ride
    // G-FORCES ARE HIGHEST, which it found rather than imposed. This project
    // already computes G at every arc length, so modulating the span by it is a
    // refinement with a citation behind it rather than a guess. Uniform until
    // somebody wants the difference.
    double SpanM = 9.0;

    // A column shorter than this is not a column — the track is essentially on the
    // ground and wants a footer plate, not a tower.
    double MinHeightM = 0.6;

    // How close a column may pass to any other part of the track before it is
    // refused. The track's own swept width plus a working clearance: somebody has
    // to be able to walk past it, and a train has to clear it.
    double ClearanceM = 1.5;

    double LegDiameterM = 0.25;

    // A SPREAD FOOTING IS MUCH WIDER THAN ITS COLUMN -- it exists to spread the
    // load into the ground, so the ratio is the whole point of it. Four times the
    // leg reads correctly at any distance somebody looks at a support from.
    //
    // ORDINARY PRACTICE, NOT A CITED FIGURE. Self (2024) explicitly puts
    // foundations outside its scope, so nothing here rests on that source; these
    // are plausible dimensions for something that has to be visible, not a
    // structural design.
    double FootingWidthRatio = 4.0;
    double FootingThicknessM = 0.45;

    // How far the top of a footing stands proud of the ground. Real ones are
    // mostly buried with a little showing, and something entirely below grade
    // would be invisible -- which defeats the point of drawing it.
    // HOW MUCH OF THE PIER SHOWS. 0.12 was a guess and it was too shy: the pad
    // is 0.57 m thick, so at 0.12 proud 79% of it was underground and what was
    // left read as a seam in the dirt rather than as a thing the column stands
    // on. From a low angle that lip plus the column's own shadowed side looks
    // convincingly like seeing through a surface into a hollow -- which is what
    // it was reported as, and the geometry was measured outward-wound both ways
    // before this was changed rather than after.
    //
    // 0.30 is ordinary practice: a coaster column lands on a concrete pier that
    // projects visibly above grade, with the baseplate and anchor bolts on top of
    // it. Cosmetic, and MORE faithful rather than less.
    double FootingProudM = 0.30;
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
            //
            // AND IT NOW GETS THE FOOTER PLATE IT WAS PROMISED. This branch said
            // "wants a footer plate, not a tower" and then placed nothing, which
            // was invisible for as long as nothing was drawn and obvious the
            // moment legs were: the lowest run of a layout floated with no column
            // and no pad, which is the one part of a real coaster nobody has ever
            // seen unsupported.
            //
            // NO FINDING, STILL. The assertion that at-grade track is silent is
            // about REPORTING, and it stays true — a station getting a pad is not
            // a placement failure, it is a station.
            if (Attach.Z >= GroundZ && S[i] - LastPlaced >= Settings.SpanM - 1e-9)
            {
                FSupportFooting Pad;
                Pad.Top = Attach;
                Pad.Width = Settings.LegDiameterM * Settings.FootingWidthRatio;
                // Down from the track to below the ground it sits on, so a pad
                // under track a few centimetres up is still a pad and not a film.
                Pad.Thickness = (Attach.Z - GroundZ) + Settings.FootingThicknessM;
                Plan.Footing.push_back(Pad);
            }
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

        // AND THE THING IT STANDS ON. A column does not stop in the dirt: it
        // lands on a spread footing, and one appears here rather than being
        // derived later so that a caller cannot draw legs without them.
        FSupportFooting Pad;
        Pad.Top = FVec3{Foot.X, Foot.Y, Foot.Z + Settings.FootingProudM};
        Pad.Width = Settings.LegDiameterM * Settings.FootingWidthRatio;
        Pad.Thickness = Settings.FootingThicknessM + Settings.FootingProudM;
        Plan.Footing.push_back(Pad);

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

// ===================== AND THE LEGS AS GEOMETRY =====================
//
// `PlanSupports` has produced legs — foot, top, diameter — since it was written,
// and the actor read only `Plan.Finding`, so every rebuild computed them and threw
// them away. A meshed track with nothing under it reads as a toy; the structure is
// most of a coaster's silhouette, which is the argument at the top of this file
// arriving at its own conclusion.
//
// IT IS A SEPARATE BUFFER, like the rails and the ties, because it is a separate
// MATERIAL. Steel columns are not painted rail, and a mesh section is the unit of
// material in the engine — the same reason the track is three buffers rather than
// one.
//
// AND IT IS THE TIE STRUT AGAIN. A column is a capped tube between two points,
// which is exactly what `SweepStrut` already builds and already asserts: two open
// ends were the visible defect that made ties get caps, and a column standing on
// the ground with an open bottom would be the same bug with a better view of it.
inline FMeshBuffer BuildSupportMesh(const FSupportPlan& Plan, int Sides = 8)
{
    FMeshBuffer Out;
    // FEWER SIDES THAN THE RAILS, deliberately. A column is background: it is seen
    // at distance and in quantity, where a rail is the thing under the camera in a
    // ride view. Eight reads as round at the range anything looks at a support
    // from, and a 1288 m circuit is a lot of columns to pay twelve sides for.
    const int N = Sides < 3 ? 3 : Sides;
    for (const FSupportLeg& L : Plan.Leg)
    {
        // A LEG OF NO HEIGHT IS NOT A LEG. `PlanSupports` already refuses these by
        // MinHeightM, so this is a guard against a caller that built a plan some
        // other way rather than a case that happens — but a zero-length sweep is a
        // degenerate ring pair and it is cheaper to refuse than to reason about.
        if (!(L.Height() > 1e-6)) { continue; }
        // THE HINT IS WORLD X, and it is genuinely arbitrary here: a column is not
        // textured along its length and nothing about its cross-section has a
        // preferred orientation, unlike a rail whose seam follows the track. The
        // degenerate case cannot arise -- a leg is vertical by construction and X
        // is never parallel to it -- but SweepStrut handles it anyway.
        SweepStrut(Out, L.Foot, L.Top, FVec3{1.0, 0.0, 0.0}, L.Diameter * 0.5, N);
    }

    // THE FOOTINGS, in the SAME buffer as the legs. Concrete and steel are two
    // materials and this is one section, which is a compromise made knowingly:
    // splitting them is one more buffer and one more component for a difference
    // nothing can currently show, since neither has a material yet. The day the
    // structure gets one, this is where the split goes.
    for (const FSupportFooting& F : Plan.Footing)
    {
        if (!(F.Thickness > 1e-6)) { continue; }
        SweepStrut(Out, F.Bottom(), F.Top, FVec3{1.0, 0.0, 0.0}, F.Width * 0.5, N);
    }
    return Out;
}

// ponytail: single vertical columns only. No bents (the A-frame pair that carries
// a banked turn), no diagonal bracing, no cantilevers, no shared footers where two
// legs land within a metre of each other. Each is a real thing a real coaster has
// and each is a placement problem of its own — and every one of them is easier to
// design once something can already say WHERE a plain column goes and where it
// cannot. The refusals above are exactly the list of cases those would solve.
