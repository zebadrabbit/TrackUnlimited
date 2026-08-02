// TrackUnlimited Phase 0 prototype: track cross-section.
// Plain C++17, no engine dependency, same conventions as TrackSpline.h.
//
// The spline says where the track GOES. This says what it IS at any point
// along it — where the two running rails sit, where the spine hangs, where the
// ties cross. That is everything a mesh generator, a support-placement pass, or
// a hand-modelled track style needs to line up with.
//
// It is deliberately just geometry: positions derived from a frame, no meshing,
// no engine types, no rendering. The frame is already exactly orthonormal (see
// PHASE0_FINDINGS.md), so a cross-section placed on it inherits that for free —
// rails stay exactly `Gauge` apart through inversions and banked turns without
// any renormalisation.
//
// ON THE NUMBERS. These are a GENERIC profile, not a copy of anyone's track.
// Real steel coaster track spans roughly 0.76–1.22 m gauge, 100–127 mm running
// rail outside diameter with ~10 mm wall, and tubular spines up to ~0.36 m.
// The defaults below sit mid-range. Per CLAUDE.md constraint 5 this project
// does not reproduce any manufacturer's trademarked dimensions or naming, so
// the presets are named for what they are, not for who builds them.
//
// Units: metres, radians. Convert at the port boundary, never in here.

#pragma once

#include "TrackSpline.h"

// One track style's cross-section. Every field is a tuning knob — a modelled
// track style should carry its own instance rather than hardcoding numbers into
// a mesh.
struct FTrackProfile
{
    // Centre of one running rail to the centre of the other, measured across
    // the track. This is the dimension everything else hangs off.
    double Gauge = 1.10;

    // Running rail tube. Wall thickness matters for a hollow-section model and
    // for anyone computing mass later; it is ignored by the positions below.
    double RailDiameter = 0.115;
    double RailWall = 0.010;

    // Spine centre below the rail plane, and its diameter. A box spine can use
    // the same drop with its own section — only the centreline is defined here.
    double SpineDrop = 0.45;
    double SpineDiameter = 0.35;

    // Cross-ties along the track, joining the rails to the spine.
    double TieSpacing = 1.00;
    double TieDiameter = 0.05;
};

// Where the track's parts are at one point along it, in world space.
//
// RailCentre is the midpoint between the running rails — the same reference
// line FTrack::RailCentreAt returns, and NOT the heartline. Banking swings this
// whole cross-section around the heartline, which is the entire reason the
// heartline model exists.
struct FTrackCrossSection
{
    FVec3 RailCentre;
    FVec3 LeftRail;  // rider's left
    FVec3 RightRail;
    FVec3 SpineCentre;
};

// Place a profile on an already-evaluated frame.
//
// Takes the frame rather than an arc length on purpose: EvaluateAt is O(track
// length), so a meshing pass that walks the track with AdvanceFrom and calls
// this per step stays linear instead of going quadratic.
inline FTrackCrossSection CrossSectionAt(const FTrackFrame& Frame, double HeartlineHeight,
                                         const FTrackProfile& Profile)
{
    FTrackCrossSection Out;
    Out.RailCentre = Frame.Position - Frame.Up * HeartlineHeight;

    // +Lateral is the rider's left, so the left rail is the +ve offset. Getting
    // this backwards mirrors the track without making it look wrong — the same
    // failure mode the UE port checklist warns about.
    const FVec3 HalfGauge = Frame.Lateral * (Profile.Gauge * 0.5);
    Out.LeftRail = Out.RailCentre + HalfGauge;
    Out.RightRail = Out.RailCentre - HalfGauge;

    Out.SpineCentre = Out.RailCentre - Frame.Up * Profile.SpineDrop;
    return Out;
}

// Convenience for one-off queries. O(track length) — do not call this in a loop
// over a track; walk with AdvanceFrom and use CrossSectionAt instead.
inline FTrackCrossSection CrossSectionAt(const FTrack& Track, double S, const FTrackProfile& Profile)
{
    return CrossSectionAt(Track.EvaluateAt(S), Track.GetHeartlineHeight(), Profile);
}

// Total width the track sweeps, rail edge to rail edge. What a clearance
// envelope or a support-placement pass needs before it knows anything else.
inline double TrackWidth(const FTrackProfile& Profile)
{
    return Profile.Gauge + Profile.RailDiameter;
}

// ponytail: ties are a SPACING, not a placement. Where they actually land is a
// meshing decision — evenly along arc length is the obvious rule, but a real
// track puts them where the supports and the rail joints want them. Emitting
// positions here would bake one policy into the geometry layer before there is
// a meshing layer to have an opinion. TieSpacing is enough to lay them out.
