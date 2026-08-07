// TrackUnlimited Phase 4: the track, as geometry you can see.
// Plain C++17, no engine dependency, same conventions as TrackSpline.h.
//
// `TrackProfile.h` says where the rails, spine and ties ARE at a point.
// This sweeps actual surfaces along them. It is the first thing in this project
// that exists to be looked at rather than to be correct about a ride, and the
// reason Phase 4 was picked over the runtime editor: the control system is now
// deeper than anything on the market and completely invisible in a screenshot.
//
// ===================== TWO HALVES, AND THE SPLIT IS LOAD-BEARING =====================
//
// `WalkTrack` is the only thing here that touches an FTrack, and it walks with
// AdvanceFrom. `BuildTrackMesh` takes the frames it produced and has no FTrack
// at all.
//
// That is not tidiness. `EvaluateAt` is O(track length) per call, so a mesher
// that called it per ring would be O(n^2) — the exact bug the vertical slice's
// debug draw had, roughly 118 million integrator steps at BeginPlay. Splitting
// it this way means the mesher CANNOT make that mistake, because it has nothing
// to make it with. A structural answer rather than a comment asking nicely.
//
// ===================== WHAT A SWEEP GETS WRONG =====================
//
// Three failures produce geometry that looks plausible and is not, and each has
// an assertion in test_trackmesh.cpp:
//
//   INSIDE OUT. Wind the quads the other way and the mesh renders black, or
//   invisible with backface culling, or fine until it meets a light. Nothing
//   about the vertex positions is wrong.
//
//   THE UV SEAM. A tube's ring closes, but its texture coordinate does not —
//   the last vertex is both V=1 and V=0. Share it and the whole texture
//   reverses across one quad, which reads as a smear rather than as a bug.
//
//   TURNING INSIDE ITSELF. A tube swept round a curve tighter than its own
//   radius folds through its axis. On track it is the INNER RAIL that goes
//   first, at |curvature| * gauge/2 >= 1, and the fold is small and dark and
//   easy to miss. REPORTED, never repaired — the same rule as everywhere else
//   here, because the fix is authoring a wider curve and a mesher that silently
//   straightened it would hide a layout that cannot be built.
//
// Units: metres. Convert and flip handedness at the port boundary, never here.

#pragma once

#include "../TrackSpline/TrackProfile.h"
#include "../TrackSpline/TrackSpline.h"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

struct FVec2
{
    double U = 0.0, V = 0.0;
};

// Named for what it is used for rather than declared bare, because a header
// compiled INTO the engine only needs its names to be unique against all of UE
// — the rule that turned FFrame into FTrackFrame and FField into FTrackIOField,
// and which only ever fails at the port.
constexpr double TrackMeshTwoPi = 6.28318530717958647692;

// Triangles, and everything a renderer needs to draw them. Deliberately not an
// engine type: UE wants TArray<FVector> and centimetres, and that conversion
// belongs at the port boundary with the handedness flip, not in here.
struct FMeshBuffer
{
    std::vector<FVec3> Position;
    std::vector<FVec3> Normal;
    std::vector<FVec2> UV;
    std::vector<std::uint32_t> Index;

    std::size_t NumVertices() const { return Position.size(); }
    std::size_t NumTriangles() const { return Index.size() / 3; }
};

// Rails, spine and ties as three buffers rather than one.
//
// They are three materials — running rail is polished where the train touches
// it, spine is painted structure, ties are usually neither — and keeping them
// apart costs nothing now and saves a submesh split later. It also means a
// track style can omit one: a wooden-style spine is a different mesh entirely,
// and a mesher that had welded them together would have to be taken apart.
struct FTrackMesh
{
    FMeshBuffer Rails;
    FMeshBuffer Spine;
    FMeshBuffer Ties;

    std::size_t NumVertices() const
    {
        return Rails.NumVertices() + Spine.NumVertices() + Ties.NumVertices();
    }
    std::size_t NumTriangles() const
    {
        return Rails.NumTriangles() + Spine.NumTriangles() + Ties.NumTriangles();
    }
};

struct FMeshSettings
{
    // Metres between rings along the track. THE ONE KNOB THAT MATTERS: it is
    // the whole quality/cost trade, and it is a distance rather than a count so
    // a long track does not get coarser than a short one.
    double SampleSpacing = 0.5;

    // Segments around a tube. Eight is a visible octagon up close and honest at
    // a distance; twelve is the sensible shipping figure. Left low by default so
    // a first run is fast rather than pretty.
    int Sides = 8;

    // Metres of track per texture repeat along a tube. A knob because a rail and
    // a spine want different ones, and because getting it wrong is invisible in
    // wireframe and obvious the moment there is a material.
    double TextureMetres = 1.0;

    bool bTies = true;
};

// A problem found while sweeping. REPORTED, NEVER REPAIRED — the rule this
// project applies to validation, to the envelope judge and to a device shorter
// than its train, and for the same reason: the repair for a curve too tight for
// its own gauge is a wider curve, and geometry quietly straightened to fit reads
// as a layout that works.
struct FMeshFinding
{
    double S = 0.0;               // where along the track
    double Curvature = 0.0;       // the yaw curvature that did it
    double MinRadius = 0.0;       // what the inner rail's radius came to
    std::string What;
};

// One ring of a tube: a centre and an orthonormal pair spanning its plane.
//
// The pair is passed in rather than derived because the caller always knows a
// better basis than a sweep could guess. On a rail it is the frame's own Lateral
// and Up, which are exactly orthonormal by construction and carry the banking —
// so a rail through an inverted loop needs no special case and gets no
// renormalisation drift.
struct FTubeRing
{
    FVec3 Centre;
    FVec3 AxisA;   // unit, in the ring plane
    FVec3 AxisB;   // unit, in the ring plane, perpendicular to AxisA
    double Along = 0.0;   // distance travelled, for the U coordinate
};

inline FVec3 Normalised(const FVec3& V)
{
    const double L = Length(V);
    return L > 0.0 ? V * (1.0 / L) : FVec3{0.0, 0.0, 0.0};
}

// Sweep a circular section through a run of rings.
//
// Sides+1 vertices per ring, and the extra one is the UV SEAM: it sits exactly
// on top of the first vertex with V = 1 instead of V = 0. Sharing them instead
// would run the texture backwards across one quad of every tube on the ride —
// which looks like a smear rather than like a bug, and is the reason this is
// spelled out rather than assumed.
inline void SweepTube(FMeshBuffer& Out, const std::vector<FTubeRing>& Rings,
                      double Radius, int Sides, double TextureMetres)
{
    if (Rings.size() < 2 || Sides < 3)
    {
        return;
    }
    const std::uint32_t Base = static_cast<std::uint32_t>(Out.Position.size());
    const int Cols = Sides + 1;
    const double UScale = TextureMetres > 0.0 ? 1.0 / TextureMetres : 1.0;

    for (const FTubeRing& R : Rings)
    {
        for (int j = 0; j < Cols; ++j)
        {
            const double Theta = TrackMeshTwoPi * static_cast<double>(j) / static_cast<double>(Sides);
            // The radial direction IS the normal, exactly, for a circular
            // section — no averaging of face normals and no smoothing groups.
            // That is a property of sweeping a circle rather than a shortcut.
            const FVec3 N = R.AxisA * std::cos(Theta) + R.AxisB * std::sin(Theta);
            Out.Position.push_back(R.Centre + N * Radius);
            Out.Normal.push_back(N);
            Out.UV.push_back({R.Along * UScale,
                              static_cast<double>(j) / static_cast<double>(Sides)});
        }
    }

    // Two triangles per quad, wound so the geometric normal agrees with the
    // radial one above. Asserted rather than eyeballed: an inside-out tube is
    // invisible under backface culling and looks like a missing mesh.
    for (std::size_t i = 0; i + 1 < Rings.size(); ++i)
    {
        for (int j = 0; j < Sides; ++j)
        {
            const std::uint32_t A = Base + static_cast<std::uint32_t>(i * Cols + j);
            const std::uint32_t B = A + 1;
            const std::uint32_t C = A + static_cast<std::uint32_t>(Cols);
            const std::uint32_t D = C + 1;
            // AROUND THEN ALONG, not along then around, and the first version of
            // this file had it the other way. In a right-handed frame where
            // Tangent x Lateral = Up, Cross(along, around) comes out as MINUS the
            // radial normal — every tube on the ride inside out, every vertex
            // position correct. Caught by the winding assertion and by nothing
            // else that could have been looked at.
            Out.Index.push_back(A); Out.Index.push_back(B); Out.Index.push_back(D);
            Out.Index.push_back(A); Out.Index.push_back(D); Out.Index.push_back(C);
        }
    }
}

// A straight strut between two points, as a two-ring tube. Ties are built from
// these: a real cross-tie is not one bar across the track but a pair of struts
// dropping from each rail to the spine, which is why this takes endpoints rather
// than a direction and a length.
//
// `Hint` only has to be non-parallel to the strut — the perpendicular basis is
// derived from it, and the frame's tangent is always a good one because a tie
// crosses the track rather than following it.
inline void SweepStrut(FMeshBuffer& Out, const FVec3& From, const FVec3& To,
                       const FVec3& Hint, double Radius, int Sides)
{
    const FVec3 D = To - From;
    const double L = Length(D);
    if (L <= 0.0)
    {
        return;
    }
    const FVec3 Dir = D * (1.0 / L);
    FVec3 A = Cross(Dir, Hint);
    if (Length(A) < 1e-9)
    {
        // Degenerate hint. Any perpendicular will do for a strut nobody is
        // texturing along its length.
        A = Cross(Dir, FVec3{0.0, 0.0, 1.0});
        if (Length(A) < 1e-9) { A = Cross(Dir, FVec3{1.0, 0.0, 0.0}); }
    }
    A = Normalised(A);
    const FVec3 B = Normalised(Cross(Dir, A));

    std::vector<FTubeRing> Rings(2);
    Rings[0] = {From, A, B, 0.0};
    Rings[1] = {To, A, B, L};
    SweepTube(Out, Rings, Radius, Sides, 1.0);
}

// THE ONLY THING HERE THAT TOUCHES AN FTrack, and it walks with AdvanceFrom.
//
// Everything downstream takes the result, which is what makes the O(n^2) trap
// unreachable rather than merely discouraged.
//
// A CIRCUIT'S LAST FRAME IS ITS FIRST, and it is emitted anyway rather than
// wrapped: the sweep needs a closing ring at S = TotalLength to put a quad
// between the last sample and the seam, and leaving it out opens a gap at the
// station exactly one sample wide. Whether the geometry closes is then a
// property of the track, which `bCircuit` already measures, and not something
// the mesher decides.
inline std::vector<FTrackFrame> WalkTrack(const FTrack& Track, double Spacing)
{
    std::vector<FTrackFrame> Out;
    const double Total = Track.TotalLength();
    if (Total <= 0.0 || !(Spacing > 0.0))
    {
        return Out;
    }
    const int Steps = static_cast<int>(std::ceil(Total / Spacing));
    Out.reserve(static_cast<std::size_t>(Steps) + 1);

    double S = 0.0;
    FTrackFrame F = Track.EvaluateAt(0.0);    // once, not per ring
    Out.push_back(F);
    for (int i = 1; i <= Steps; ++i)
    {
        const double Next = std::min(Total, Spacing * static_cast<double>(i));
        F = Track.AdvanceFrom(F, S, Next);
        S = Next;
        Out.push_back(F);
    }
    return Out;
}

// Sweep the whole cross-section along a walked path.
//
// Takes frames and a heartline height rather than a track, so it is pure
// geometry with nothing to be slow about. `OutFindings` is optional and is
// appended to, never cleared, so a caller can accumulate over several tracks.
inline FTrackMesh BuildTrackMesh(const std::vector<FTrackFrame>& Path, double HeartlineHeight,
                                 const FTrackProfile& Profile, const FMeshSettings& Settings,
                                 std::vector<FMeshFinding>* OutFindings = nullptr)
{
    FTrackMesh Mesh;
    if (Path.size() < 2)
    {
        return Mesh;
    }

    const double HalfGauge = Profile.Gauge * 0.5;
    const double RailRadius = Profile.RailDiameter * 0.5;

    std::vector<FTubeRing> Left, Right, Spine;
    Left.reserve(Path.size());
    Right.reserve(Path.size());
    Spine.reserve(Path.size());

    double Along = 0.0;
    for (std::size_t i = 0; i < Path.size(); ++i)
    {
        const FTrackFrame& F = Path[i];
        if (i > 0)
        {
            Along += Length(F.Position - Path[i - 1].Position);
        }
        const FTrackCrossSection X = CrossSectionAt(F, HeartlineHeight, Profile);

        // The ring plane is the frame's OWN Lateral and Up, which are exactly
        // orthonormal by construction and already carry the banking. So a rail
        // through an inverted loop needs no special case, and nothing here
        // renormalises anything — the property TrackProfile.h relies on, one
        // level up.
        Left.push_back({X.LeftRail, F.Lateral, F.Up, Along});
        Right.push_back({X.RightRail, F.Lateral, F.Up, Along});
        Spine.push_back({X.SpineCentre, F.Lateral, F.Up, Along});

        // TURNING INSIDE ITSELF. The inner rail's own radius is the heartline's
        // less half the gauge, so a curve tighter than that folds the rail
        // through its own axis. Reported at the point it happens, with the
        // number, because "your curve is too tight" without a radius is not
        // somewhere to go and look.
        const double K = std::fabs(F.YawCurvature);
        if (OutFindings != nullptr && K > 0.0)
        {
            const double R = 1.0 / K;
            const double Inner = R - HalfGauge;
            if (Inner <= RailRadius)
            {
                FMeshFinding Bad;
                Bad.S = Along;
                Bad.Curvature = F.YawCurvature;
                Bad.MinRadius = Inner;
                Bad.What = Inner <= 0.0
                    ? "curve is tighter than half the track gauge: the inner rail folds through itself"
                    : "inner rail radius is under its own tube radius: the sweep self-intersects";
                OutFindings->push_back(Bad);
            }
        }
    }

    SweepTube(Mesh.Rails, Left, RailRadius, Settings.Sides, Settings.TextureMetres);
    SweepTube(Mesh.Rails, Right, RailRadius, Settings.Sides, Settings.TextureMetres);
    SweepTube(Mesh.Spine, Spine, Profile.SpineDiameter * 0.5, Settings.Sides, Settings.TextureMetres);

    // TIES ARE A SPACING, NOT A PLACEMENT — TrackProfile.h's own note, and this
    // is the meshing layer finally having the opinion it said belonged here.
    // Evenly along arc length is the obvious rule and it is the one taken; a real
    // track puts them where the supports and the rail joints want them, and that
    // is a decision for whatever places supports.
    //
    // Each tie is TWO struts, rail down to spine, because a cross-tie on real
    // steel track is a truss and not a bar. It also means a tie never crosses
    // the space the train occupies, which one bar straight across would.
    if (Settings.bTies && Profile.TieSpacing > 0.0)
    {
        double NextTie = 0.0;
        for (std::size_t i = 0; i < Path.size(); ++i)
        {
            if (Left[i].Along + 1e-9 < NextTie)
            {
                continue;
            }
            NextTie += Profile.TieSpacing;
            const FVec3& Hint = Path[i].Tangent;
            SweepStrut(Mesh.Ties, Left[i].Centre, Spine[i].Centre, Hint,
                       Profile.TieDiameter * 0.5, Settings.Sides);
            SweepStrut(Mesh.Ties, Right[i].Centre, Spine[i].Centre, Hint,
                       Profile.TieDiameter * 0.5, Settings.Sides);
        }
    }

    return Mesh;
}

// ponytail: no LODs, no instancing, no material assignment, no vertex colours,
// no tangents for normal mapping. Every one of those is a rendering decision and
// none can be made before something renders this. Tangents are the first to want
// and are one cross product from the normal and the along-track direction.
//
// Also no support towers: that is its own card and a genuinely hard sub-problem
// (where the ground is, what a footer may stand on, how a tower reaches track
// that banks over it), not a detail of sweeping a tube.
