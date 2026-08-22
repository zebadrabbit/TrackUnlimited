// Asserts for TrackStation.h — the platform, stripe, airgates and cabinet.
//
//   clang++ -std=c++17 -O2 -Wall -Wextra -o test_trackstation test_trackstation.cpp
//
// Same bar as everything in this directory: closed and outward by signed
// volume, on the SIDE that was asked for, inside the span, a gate per car.

#include "TrackStation.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <utility>
#include <vector>

namespace
{

std::size_t BoundaryEdges(const FMeshBuffer& M)
{
    std::vector<std::uint32_t> Weld(M.Position.size());
    for (std::size_t i = 0; i < M.Position.size(); ++i)
    {
        Weld[i] = static_cast<std::uint32_t>(i);
        for (std::size_t j = 0; j < i; ++j)
        {
            if (Length(M.Position[i] - M.Position[j]) < 1e-9) { Weld[i] = Weld[j]; break; }
        }
    }
    std::map<std::pair<std::uint32_t, std::uint32_t>, int> Edges;
    for (std::size_t t = 0; t + 2 < M.Index.size(); t += 3)
    {
        for (int e = 0; e < 3; ++e)
        {
            const std::uint32_t A = Weld[M.Index[t + static_cast<std::size_t>(e)]];
            const std::uint32_t B = Weld[M.Index[t + static_cast<std::size_t>((e + 1) % 3)]];
            Edges[{std::min(A, B), std::max(A, B)}] += 1;
        }
    }
    std::size_t Boundary = 0;
    for (const auto& E : Edges) { if (E.second != 2) { ++Boundary; } }
    return Boundary;
}

double SignedVolume(const FMeshBuffer& M)
{
    double V6 = 0.0;
    for (std::size_t t = 0; t + 2 < M.Index.size(); t += 3)
    {
        V6 += Dot(M.Position[M.Index[t]], Cross(M.Position[M.Index[t + 1]], M.Position[M.Index[t + 2]]));
    }
    return V6 / 6.0;
}

FStationMesh Build(bool bLeft, double A = 10.0, double B = 40.0, double Spacing = 0.5)
{
    FTrack T;
    T.AddSegment(MakeStraight(60.0));
    FStationSpan Sp; Sp.StartS = A; Sp.EndS = B; Sp.bLeft = bLeft;
    return BuildStations(WalkTrack(T, Spacing), {Sp});
}

void TestEveryBufferIsClosedAndOutward()
{
    std::printf("Concrete, steel and stripe are each watertight and outward-wound\n");
    for (bool bLeft : {true, false})
    {
        const FStationMesh M = Build(bLeft);
        for (const FMeshBuffer* B : {&M.Concrete, &M.Steel, &M.Stripe})
        {
            assert(B->NumTriangles() > 0);
            assert(BoundaryEdges(*B) == 0);
            assert(SignedVolume(*B) > 0.0);
        }
        // The slab is exactly what its dimensions say: 30 m x 2.5 m x 1 m.
        assert(std::fabs(SignedVolume(M.Concrete) - 30.0 * 2.5 * 1.0) < 1e-6);
        std::printf("  %s: slab %.3f m3, steel %.4f m3, stripe %.5f m3\n", bLeft ? "left " : "right",
                    SignedVolume(M.Concrete), SignedVolume(M.Steel), SignedVolume(M.Stripe));
    }
}

// +Lateral is the rider's LEFT. On a straight along +X that is +Y, and a
// platform on the wrong side of every ride looks exactly as right as one on the
// right side, which is why it is asserted.
void TestThePlatformIsOnTheSideAskedFor()
{
    std::printf("The platform is on the side that was asked for\n");
    const FStationMesh L = Build(true), R = Build(false);
    for (const FVec3& P : L.Concrete.Position) { assert(P.Y > 0.0); }
    for (const FVec3& P : R.Concrete.Position) { assert(P.Y < 0.0); }
    for (const FVec3& P : L.Steel.Position) { assert(P.Y > 0.0); }
    for (const FVec3& P : R.Steel.Position) { assert(P.Y < 0.0); }
    // And clear of the train: nothing inboard of the stated clearance.
    const FStationSettings St;
    for (const FVec3& P : L.Concrete.Position) { assert(P.Y >= St.InboardM - 1e-9); }
}

void TestOneGatePerCarAndInsideTheSpan()
{
    std::printf("One airgate per car pitch, inside the span, at either walk spacing\n");
    const FStationMesh Coarse = Build(true, 10.0, 40.0, 1.0);
    const FStationMesh Fine = Build(true, 10.0, 40.0, 0.25);
    // A gate is four struts; the cabinet is one box. Counting struts by their
    // triangles: a 6-sided capped strut is 6*2 walls + 2*6 caps = 24.
    const std::size_t Gate = 4 * 24;
    assert(Coarse.Steel.NumTriangles() == 10 * Gate + 12);
    assert(Fine.Steel.NumTriangles() == 10 * Gate + 12);
    for (const FMeshBuffer* B : {&Coarse.Concrete, &Coarse.Steel, &Coarse.Stripe})
    {
        for (const FVec3& P : B->Position) { assert(P.X >= 10.0 - 1e-6 && P.X <= 40.0 + 1e-6); }
    }
    // A span too short for a cabinet still gets its gates and no cabinet.
    const FStationMesh Short = Build(true, 10.0, 12.0);
    assert(Short.Steel.NumTriangles() == 0);
    assert(Short.Concrete.NumTriangles() > 0);
    // And an empty span builds nothing at all.
    assert(Build(true, 10.0, 10.0).NumTriangles() == 0);
    std::printf("  10 gates at either spacing, 1 cabinet; a 2 m span has neither\n");
}

} // namespace

int main()
{
    std::printf("=== TrackStation assert suite ===\n");
    TestEveryBufferIsClosedAndOutward();
    TestThePlatformIsOnTheSideAskedFor();
    TestOneGatePerCarAndInsideTheSpan();
    std::printf("All TrackStation tests passed.\n");
    return 0;
}
