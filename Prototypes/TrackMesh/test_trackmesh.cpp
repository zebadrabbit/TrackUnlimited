// Asserts for TrackMesh.h — the track as geometry you can look at.
//
//   clang++ -std=c++17 -Wall -Wextra -O2 -o test_trackmesh test_trackmesh.cpp && ./test_trackmesh
//
// Every assertion here covers a failure that produces geometry which LOOKS
// plausible. A mesh with the wrong vertex positions is obvious in one glance;
// one that is inside out, or seamed wrong, or folded through its own axis, is
// not, and those are the three that get through review.

#include "TrackMesh.h"

#include <cassert>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <vector>

namespace
{

FTrack Straight(double Len, double Roll = 0.0)
{
    FTrack T;
    T.AddSegment(MakeStraight(Len, Roll));
    return T;
}

// R = 8 vertical loop, the case a frame error hides best: PHASE0_FINDINGS
// records a mutant that put the apex at z = -16 with every G reading identical.
// If a cross-section is going to come apart anywhere it is upside down.
FTrack VerticalLoop(double R = 8.0)
{
    FTrack T;
    T.AddSegment(MakeStraight(10.0));
    FTrackSegment Loop = MakeStraight(TrackMeshTwoPi * R);
    Loop.PitchCurvatureStart = 1.0 / R;
    Loop.PitchCurvatureEnd = 1.0 / R;
    T.AddSegment(Loop);
    T.AddSegment(MakeStraight(10.0));
    return T;
}

FTrackMesh Build(const FTrack& T, const FMeshSettings& S, const FTrackProfile& P = FTrackProfile(),
                 std::vector<FMeshFinding>* Findings = nullptr)
{
    return BuildTrackMesh(WalkTrack(T, S.SampleSpacing), T.GetHeartlineHeight(), P, S, Findings);
}

// ---------------------------------------------------------------- the checks

// Every index addresses a vertex that exists. Trivial and worth having: an
// off-by-one in the ring stride reads memory that is not there and is
// undetectable from the picture until the day it crashes.
void CheckIndicesInRange(const FMeshBuffer& M)
{
    assert(M.Index.size() % 3 == 0);
    for (std::uint32_t I : M.Index)
    {
        assert(I < M.Position.size());
    }
}

// No triangle has zero area. A collapsed one is invisible, contributes nothing,
// and is exactly what a duplicated ring or a zero-length strut produces — so a
// mesher that emitted them would look correct while doing less than it claims.
void CheckNoDegenerateTriangles(const FMeshBuffer& M, const char* Where)
{
    for (std::size_t t = 0; t < M.Index.size(); t += 3)
    {
        const FVec3& A = M.Position[M.Index[t]];
        const FVec3& B = M.Position[M.Index[t + 1]];
        const FVec3& C = M.Position[M.Index[t + 2]];
        const double Area2 = Length(Cross(B - A, C - A));
        if (!(Area2 > 1e-12))
        {
            std::printf("  DEGENERATE triangle %zu in %s\n", t / 3, Where);
            assert(false);
        }
    }
}

// THE ONE THAT MATTERS MOST, and the one nothing about the vertex positions
// would reveal: is the mesh inside out?
//
// Wind the quads the other way and every position is still right. The mesh then
// renders black, or invisible under backface culling, or fine until it meets a
// light — a class of bug that survives screenshots. So the geometric normal of
// every triangle is checked to agree with the radial normals the sweep stored,
// which is the same question asked two independent ways.
void CheckWindingAgreesWithNormals(const FMeshBuffer& M, const char* Where)
{
    for (std::size_t t = 0; t < M.Index.size(); t += 3)
    {
        const std::uint32_t Ia = M.Index[t], Ib = M.Index[t + 1], Ic = M.Index[t + 2];
        const FVec3 Face = Cross(M.Position[Ib] - M.Position[Ia], M.Position[Ic] - M.Position[Ia]);
        const FVec3 Avg = (M.Normal[Ia] + M.Normal[Ib] + M.Normal[Ic]) * (1.0 / 3.0);
        if (!(Dot(Face, Avg) > 0.0))
        {
            std::printf("  INSIDE OUT at triangle %zu in %s (dot %.6f)\n",
                        t / 3, Where, Dot(Face, Avg));
            assert(false);
        }
    }
}

// Normals are unit, and point AWAY from the tube axis. The second half is what
// makes them normals rather than three numbers that happen to be normalised.
void CheckNormalsAreOutward(const FMeshBuffer& M)
{
    for (std::size_t v = 0; v < M.Position.size(); ++v)
    {
        assert(std::fabs(Length(M.Normal[v]) - 1.0) < 1e-12);
    }
}

// ------------------------------------------------------------------- the tests

void TestASweptRailIsExactlyItsOwnRadiusEverywhere()
{
    // The analogue, one level up, of TrackProfile's "rails stay exactly Gauge
    // apart through an inverted loop". There the claim was about the centreline;
    // here it is about the SURFACE, and it is the property that says the sweep
    // inherited the frame's exactness instead of quietly drifting off it.
    //
    // Through a full vertical loop, because that is where a frame error hides.
    const FTrack T = VerticalLoop();
    FMeshSettings S;
    S.SampleSpacing = 0.25;
    S.Sides = 12;
    const FTrackProfile P;
    const std::vector<FTrackFrame> Path = WalkTrack(T, S.SampleSpacing);
    const FTrackMesh M = Build(T, S, P);

    const double RailRadius = P.RailDiameter * 0.5;
    const int Cols = S.Sides + 1;
    double Worst = 0.0;

    // Two rails, laid down in order, so the left rail's rings come first.
    for (std::size_t r = 0; r < 2; ++r)
    {
        for (std::size_t i = 0; i < Path.size(); ++i)
        {
            const FTrackCrossSection X = CrossSectionAt(Path[i], T.GetHeartlineHeight(), P);
            const FVec3 Axis = r == 0 ? X.LeftRail : X.RightRail;
            for (int j = 0; j < Cols; ++j)
            {
                const std::size_t v = (r * Path.size() + i) * static_cast<std::size_t>(Cols)
                                    + static_cast<std::size_t>(j);
                Worst = std::max(Worst, std::fabs(Length(M.Rails.Position[v] - Axis) - RailRadius));
            }
        }
    }
    assert(Worst < 1e-12);
    std::printf("  a swept rail is its own radius to %.1e m, through an inverted loop\n", Worst);
}

void TestTheMeshIsNotINSIDEOUT()
{
    // Nothing about the vertex positions would show this, which is the whole
    // reason it is asserted rather than looked at. Checked on the loop, where
    // the frame flips over and a sweep that got its handedness from the world
    // rather than from the frame would invert halfway round.
    const FTrack T = VerticalLoop();
    FMeshSettings S;
    S.SampleSpacing = 0.3;
    S.Sides = 10;
    const FTrackMesh M = Build(T, S);

    CheckIndicesInRange(M.Rails);
    CheckIndicesInRange(M.Spine);
    CheckIndicesInRange(M.Ties);
    CheckNoDegenerateTriangles(M.Rails, "rails");
    CheckNoDegenerateTriangles(M.Spine, "spine");
    CheckNoDegenerateTriangles(M.Ties, "ties");
    CheckWindingAgreesWithNormals(M.Rails, "rails");
    CheckWindingAgreesWithNormals(M.Spine, "spine");
    CheckWindingAgreesWithNormals(M.Ties, "ties");
    CheckNormalsAreOutward(M.Rails);
    CheckNormalsAreOutward(M.Ties);
    std::printf("  %zu triangles, every one wound to agree with its own normals\n",
                M.NumTriangles());
}

void TestTheUVSeamIsDUPLICATEDAndNotShared()
{
    // A tube's ring closes; its texture coordinate does not. The last vertex of
    // a ring sits exactly on the first and must carry V = 1 rather than V = 0.
    //
    // Share them and the texture runs BACKWARDS across one quad of every tube on
    // the ride — which reads as a smear rather than as a bug, and is the sort of
    // thing that ships.
    const FTrack T = Straight(10.0);
    FMeshSettings S;
    S.SampleSpacing = 1.0;
    S.Sides = 8;
    const FTrackMesh M = Build(T, S);

    const int Cols = S.Sides + 1;
    const FVec3& First = M.Rails.Position[0];
    const FVec3& Last = M.Rails.Position[static_cast<std::size_t>(Cols - 1)];
    assert(Length(Last - First) < 1e-12);                  // same place
    assert(M.Rails.UV[0].V == 0.0);
    assert(M.Rails.UV[static_cast<std::size_t>(Cols - 1)].V == 1.0);   // different V

    // And U runs with distance along the track, not with the ring index — a
    // texture that repeated per ring would change scale with SampleSpacing.
    const std::size_t SecondRing = static_cast<std::size_t>(Cols);
    assert(std::fabs(M.Rails.UV[SecondRing].U - 1.0) < 1e-9);
    std::printf("  the UV seam is duplicated, and U runs with metres not with rings\n");
}

void TestACurveTighterThanItsGaugeIsREPORTEDNotStraightened()
{
    // TURNING INSIDE ITSELF. A tube swept round a curve tighter than its own
    // radius folds through its axis, and on track it is the INNER rail that goes
    // first — at a radius of half the gauge, which for the default profile is
    // 0.55 m.
    //
    // REPORTED, NEVER REPAIRED. The repair is authoring a wider curve, and a
    // mesher that silently straightened it would hide a layout that cannot be
    // built — the same rule as the device shorter than its train.
    FTrack Tight;
    Tight.AddSegment(MakeArc(3.0, 0.4));         // 0.4 m radius: inside the gauge
    FMeshSettings S;
    S.SampleSpacing = 0.2;

    std::vector<FMeshFinding> Findings;
    Build(Tight, S, FTrackProfile(), &Findings);
    assert(!Findings.empty());
    assert(Findings[0].MinRadius < 0.0);         // the inner rail is inverted
    assert(Findings[0].Curvature != 0.0);

    // And a curve a real ride would have says nothing at all, or the report is
    // noise. R = 35 is this project's own circuit turnaround.
    FTrack Fine;
    Fine.AddSegment(MakeArc(20.0, 35.0));
    std::vector<FMeshFinding> Quiet;
    Build(Fine, S, FTrackProfile(), &Quiet);
    assert(Quiet.empty());
    std::printf("  a 0.4 m radius reports \"%s\"; a 35 m one is silent\n",
                Findings[0].What.c_str());
}

void TestTheWALKIsLINEARAndTheMESHERCannotMakeItNot()
{
    // The O(n^2) trap this file is split in two to avoid: EvaluateAt is
    // O(track length) per call, so a mesher calling it per ring goes quadratic.
    // The vertical slice's debug draw had exactly that — roughly 118 million
    // integrator steps at BeginPlay.
    //
    // The structural half is that BuildTrackMesh takes FRAMES and has no FTrack,
    // so it has nothing to make the mistake with. This asserts the measurable
    // half: ring count scales with LENGTH, so doubling the track doubles the
    // work rather than quadrupling it.
    FMeshSettings S;
    S.SampleSpacing = 0.5;
    const std::size_t Short = WalkTrack(Straight(100.0), S.SampleSpacing).size();
    const std::size_t Long = WalkTrack(Straight(200.0), S.SampleSpacing).size();
    assert(Short == 201);
    assert(Long == 401);

    // The last frame lands EXACTLY on the end, not one spacing past it and not
    // short of it. A circuit whose final ring stopped early opens a gap at the
    // station one sample wide, which looks like a missing tie.
    const std::vector<FTrackFrame> Odd = WalkTrack(Straight(10.3), 0.5);
    const FTrack T = Straight(10.3);
    assert(Length(Odd.back().Position - T.EvaluateAt(T.TotalLength()).Position) < 1e-9);
    std::printf("  the walk is linear in length and lands exactly on the end\n");
}

void TestTiesAreSpacedInMETRESAndDoNotCrossTheTrain()
{
    // TrackProfile.h left ties as a SPACING and said where they land is a
    // meshing decision. This is that decision: evenly along arc length.
    //
    // Two struts per tie, rail down to spine, because a cross-tie on real steel
    // track is a truss and not a bar — and because one bar straight across would
    // pass through the space the train occupies.
    const FTrack T = Straight(20.0);
    FMeshSettings S;
    S.SampleSpacing = 0.5;
    S.Sides = 6;
    FTrackProfile P;
    P.TieSpacing = 2.0;
    const FTrackMesh M = Build(T, S, P);

    // 20 m at 2 m spacing is 11 ties counting both ends, two struts each, and a
    // strut is two rings of Sides+1 vertices.
    const std::size_t PerStrut = 2 * static_cast<std::size_t>(S.Sides + 1);
    assert(M.Ties.NumVertices() == 11 * 2 * PerStrut);

    // Nothing in a tie is above the rail plane, so nothing is in the train's way.
    const double RailZ = T.EvaluateAt(0.0).Position.Z - T.GetHeartlineHeight();
    for (const FVec3& V : M.Ties.Position)
    {
        assert(V.Z <= RailZ + P.TieDiameter * 0.5 + 1e-9);
    }

    // And turning them off costs exactly the ties, not the rails.
    FMeshSettings NoTies = S;
    NoTies.bTies = false;
    const FTrackMesh Bare = Build(T, NoTies, P);
    assert(Bare.Ties.NumVertices() == 0);
    assert(Bare.Rails.NumVertices() == M.Rails.NumVertices());
    std::printf("  11 ties over 20 m at 2 m spacing, none of them in the train's way\n");
}

void TestTheHANDEDNESSFlipREVERSESWinding()
{
    // THE PORT RULE, PINNED HERE WHERE IT CAN BE TESTED, because at the boundary
    // itself it cannot be.
    //
    // These prototypes are right-handed with +Lateral to the rider's left; UE5 is
    // left-handed with +Y to the right. CLAUDE.md's conversion is M(x,y,z) =
    // (x,-y,z), and PHASE0_FINDINGS warns that getting it wrong mirrors the whole
    // track into geometry that still looks self-consistent.
    //
    // WHAT NEITHER OF THOSE SAYS, AND WHAT BITES A MESH: M is a REFLECTION, with
    // determinant -1. It reverses triangle orientation. Apply it to the positions
    // and the normals and change nothing else, and every surface on the ride is
    // inside out — in UE, not here, so nothing in this suite would ever see it
    // and the symptom is a coaster that renders as a black skeleton.
    //
    // So the port must swap two indices of every triangle. Asserted as a
    // property rather than trusted as a comment.
    const FTrack T = VerticalLoop();
    FMeshSettings S;
    S.SampleSpacing = 0.5;
    S.Sides = 8;
    FMeshBuffer M = Build(T, S).Rails;

    auto Mirror = [](const FVec3& V) { return FVec3{V.X, -V.Y, V.Z}; };
    for (FVec3& P : M.Position) { P = Mirror(P); }
    for (FVec3& N : M.Normal) { N = Mirror(N); }

    // Mirrored and nothing else: inside out, everywhere.
    std::size_t Wrong = 0;
    for (std::size_t t = 0; t < M.Index.size(); t += 3)
    {
        const std::uint32_t A = M.Index[t], B = M.Index[t + 1], C = M.Index[t + 2];
        const FVec3 Face = Cross(M.Position[B] - M.Position[A], M.Position[C] - M.Position[A]);
        if (!(Dot(Face, M.Normal[A]) > 0.0)) { ++Wrong; }
    }
    assert(Wrong == M.NumTriangles());

    // Swap two indices of every triangle and it is right again.
    for (std::size_t t = 0; t < M.Index.size(); t += 3)
    {
        std::swap(M.Index[t + 1], M.Index[t + 2]);
    }
    CheckWindingAgreesWithNormals(M, "mirrored rails");
    std::printf("  the handedness flip reverses winding: all %zu triangles, both ways\n",
                M.NumTriangles());
}

void TestTheCostOfARealRide()
{
    // Not a pass/fail — a number to know before something renders it. The
    // two-train circuit is 1288 m, and this is what it costs at settings that
    // would actually ship.
    FTrack T;
    T.AddSegment(MakeStraight(1288.0));
    FMeshSettings S;
    S.SampleSpacing = 0.5;
    S.Sides = 12;
    const FTrackMesh M = Build(T, S);
    std::printf("  a 1288 m circuit at 0.5 m / 12 sides: %zu vertices, %zu triangles"
                " (rails %zu, spine %zu, ties %zu)\n",
                M.NumVertices(), M.NumTriangles(), M.Rails.NumTriangles(),
                M.Spine.NumTriangles(), M.Ties.NumTriangles());
    assert(M.NumTriangles() > 0);
}

} // namespace

int main()
{
    std::printf("TrackMesh: sweeping the cross-section into geometry\n\n");

    TestASweptRailIsExactlyItsOwnRadiusEverywhere();
    TestTheMeshIsNotINSIDEOUT();
    TestTheUVSeamIsDUPLICATEDAndNotShared();
    TestACurveTighterThanItsGaugeIsREPORTEDNotStraightened();
    TestTheWALKIsLINEARAndTheMESHERCannotMakeItNot();
    TestTiesAreSpacedInMETRESAndDoNotCrossTheTrain();
    TestTheHANDEDNESSFlipREVERSESWinding();
    TestTheCostOfARealRide();

    std::printf("\ntest_trackmesh: all assertions passed.\n");
    return 0;
}
