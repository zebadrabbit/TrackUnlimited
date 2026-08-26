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
        // The slab is exactly what its dimensions say: 30 m x 4 m x 1 m.
        assert(std::fabs(SignedVolume(M.Concrete) - 30.0 * 4.0 * 1.0) < 1e-6);
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

void TestOneGatePerRowAndInsideTheSpan()
{
    std::printf("One airgate per seat row, facing the parked train, inside the span, at either walk spacing\n");
    const FStationSettings St;
    const FStationMesh Coarse = Build(true, 10.0, 40.0, 1.0);
    const FStationMesh Fine = Build(true, 10.0, 40.0, 0.25);
    // The steel does not depend on the walk spacing, and the layout says ten
    // rows fit a 30 m span at a 3 m car with one row per car.
    assert(Coarse.Steel.NumTriangles() == Fine.Steel.NumTriangles());
    {
        FStationSpan Sp; Sp.StartS = 10.0; Sp.EndS = 40.0;
        assert(StationGateCount(Sp, St) == 10);
        // Centred in the span without a nose: the first row's centre at 11.5.
        assert(std::fabs(StationGateCentres(Sp, St).front() - 11.5) < 1e-9);
        // With the parked nose known, rows lay back from it: nose at 39 (the
        // stop mark, a metre short of the end) puts the front row at 37.5 and
        // the rearmost gate that still fits wholly inside the span at 10.5 --
        // ten gates, the front five facing the five cars of a 15 m train.
        Sp.NoseS = 39.0;
        const std::vector<double> C = StationGateCentres(Sp, St);
        assert(C.size() == 10);
        assert(std::fabs(C.back() - 37.5) < 1e-9 && std::fabs(C.front() - 10.5) < 1e-9);
        // Two rows a car halves the pitch and doubles the gates.
        FStationSettings TwoRows; TwoRows.RowsPerCar = 2;
        assert(StationGateCount(Sp, TwoRows) == 19);
    }
    // Inside the span, to the end posts' own radius: the fence ends ON the
    // platform's end, so its last post straddles the line by half its width.
    const double PostR = St.PostDiameterM * 0.5 + 1e-6;
    for (const FMeshBuffer* B : {&Coarse.Concrete, &Coarse.Steel, &Coarse.Stripe})
    {
        for (const FVec3& P : B->Position) { assert(P.X >= 10.0 - PostR && P.X <= 40.0 + PostR); }
    }
    // A span too short for a row still gets its fence, and no cabinet: the
    // steel is the fence alone, which is less than one gate opening's worth.
    const FStationMesh Short = Build(true, 10.0, 12.0);
    {
        FStationSpan Sp; Sp.StartS = 10.0; Sp.EndS = 12.0;
        assert(StationGateCount(Sp, St) == 0);
    }
    assert(Short.Steel.NumTriangles() > 0);
    assert(Short.Concrete.NumTriangles() > 0);
    // A four-position platform is four spans and ONE cabinet: three marked
    // bCabinet = false and the last one carrying it. Composable: four single
    // spans' steel plus one cabinet box (12 triangles).
    {
        FTrack T; T.AddSegment(MakeStraight(60.0));
        const std::vector<FTrackFrame> Frames = WalkTrack(T, 0.5);
        std::vector<FStationSpan> Four;
        std::size_t SteelAlone = 0;
        for (int p = 0; p < 4; ++p)
        {
            FStationSpan Sp; Sp.StartS = 10.0 + p * 10.0; Sp.EndS = 20.0 + p * 10.0; Sp.bCabinet = p == 3;
            Four.push_back(Sp);
            FStationSpan Bare = Sp; Bare.bCabinet = false;
            SteelAlone += BuildStations(Frames, {Bare}).Steel.NumTriangles();
        }
        const FStationMesh M = BuildStations(Frames, Four);
        assert(M.Steel.NumTriangles() == SteelAlone + 12);
    }
    // And an empty span builds nothing at all.
    assert(Build(true, 10.0, 10.0).NumTriangles() == 0);
    std::printf("  10 rows centred, 10 laid back from a nose at 39 m, 19 at two rows a car; "
                "steel %zu triangles at either spacing\n", Coarse.Steel.NumTriangles());
}

// The gates swing onto the platform, hinged on the upstream post, and the
// buffer keeps its topology at every angle — the property the per-frame
// in-place vertex update depends on.
void TestTheGatesSwingOntoThePlatform()
{
    std::printf("Airgates swing toward the train about the upstream post, same topology\n");
    FTrack T;
    T.AddSegment(MakeStraight(60.0));
    const std::vector<FTrackFrame> Frames = WalkTrack(T, 0.5);
    auto With = [&](std::vector<double> Positions, bool bLeft = true)
    {
        FStationSpan Sp; Sp.StartS = 10.0; Sp.EndS = 40.0; Sp.bLeft = bLeft;
        Sp.GatePositions = std::move(Positions);
        return BuildStations(Frames, {Sp});
    };
    const FStationMesh Closed = With({});
    const FStationMesh Explicit = With(std::vector<double>(10, 1.0));
    const FStationMesh Open = With(std::vector<double>(10, 0.0));
    const FStationMesh Half = With({0.5});
    const FStationMesh Jammed = With({1.0, 0.0, 1.0}); // the middle one stayed open

    // Empty and all-closed are the same panel, bit for bit; every angle has the
    // same vertex and triangle count; every angle is still closed and outward.
    assert(Closed.Steel.Position.size() == Explicit.Steel.Position.size());
    for (std::size_t i = 0; i < Closed.Steel.Position.size(); ++i)
    {
        assert(Length(Closed.Steel.Position[i] - Explicit.Steel.Position[i]) < 1e-12);
    }
    for (const FStationMesh* M : {&Closed, &Open, &Half, &Jammed})
    {
        assert(M->Steel.NumVertices() == Closed.Steel.NumVertices());
        assert(M->Steel.NumTriangles() == Closed.Steel.NumTriangles());
        assert(BoundaryEdges(M->Steel) == 0);
        assert(SignedVolume(M->Steel) > 0.0);
    }

    // Open: the gate lies across the operator's strip, TOWARD the train. Its
    // free end is a panel length inboard (smaller Y, on the left) of its hinge;
    // closed, it is a panel length further along the track on the fence line.
    // Measured short of the cabinet (X > 38). Closed stays on the fence line;
    // open reaches the panel length (0.84 m) toward the train -- AND STAYS
    // CLEAR OF IT: the free end is still outboard of the platform's own edge,
    // which is outboard of the train. (The first version swung a 2.7 m fence
    // section onto the queue side and off a 2.5 m platform.)
    const FStationSettings St;
    const double EdgeY = St.InboardM + St.GateSetbackM;
    const double PanelLen = St.GateWidthM - 0.06;
    auto MinYShortOfCabinet = [](const FStationMesh& M)
    {
        double Y = 1e9;
        for (const FVec3& P : M.Steel.Position) { if (P.X < 30.0) { Y = std::min(Y, P.Y); } }
        return Y;
    };
    const double ClosedMinY = MinYShortOfCabinet(Closed);
    const double OpenMinY = MinYShortOfCabinet(Open);
    const double HalfMinY = MinYShortOfCabinet(Half);
    assert(std::fabs(ClosedMinY - (EdgeY - St.PostDiameterM * 0.5)) < 1e-9);
    assert(std::fabs(OpenMinY - (EdgeY - PanelLen - St.PostDiameterM * 0.5)) < 1e-6);
    assert(OpenMinY > St.InboardM);                                    // clear of the train
    assert(std::fabs(HalfMinY - (EdgeY - PanelLen * std::sin(0.25 * 3.14159265358979323846)
                                 - St.PostDiameterM * 0.5)) < 0.03);

    // And the jammed one: gate 1 open while 0 and 2 are closed -- its panel is
    // the only steel inboard of the fence line before the cabinet's X, and it
    // sits at its own hinge: row 1's centre is 14.5, so the hinge post is at 14.05.
    int PastEdge = 0;
    for (const FVec3& P : Jammed.Steel.Position)
    {
        if (P.Y < EdgeY - 0.3 && P.X < 30.0)
        {
            ++PastEdge;
            assert(std::fabs(P.X - 14.05) < St.PostDiameterM);
        }
    }
    assert(PastEdge > 0);
    // The right-hand platform swings the other way, by symmetry.
    const FStationMesh OpenRight = With(std::vector<double>(10, 0.0), false);
    double OpenRightMaxY = -1e9;
    for (const FVec3& P : OpenRight.Steel.Position) { if (P.X < 30.0) { OpenRightMaxY = std::max(OpenRightMaxY, P.Y); } }
    assert(std::fabs(OpenRightMaxY + OpenMinY) < 1e-9);
    std::printf("  fence line y=%.2f: closed reaches y=%.2f, open y=%.2f (platform edge %.2f), half y=%.2f; "
                "jammed middle gate alone off the line\n",
                EdgeY, ClosedMinY, OpenMinY, St.InboardM, HalfMinY);
}

// A gate opens only onto a car: the small-batch case, a 6 m train on a 10 m
// position with three gates, and the one nobody is sitting at.
void TestAGateOpensOnlyOntoACar()
{
    std::printf("A gate whose row has no car parked at it is served by nobody\n");
    FStationSpan Sp; Sp.StartS = 10.0; Sp.EndS = 20.0; Sp.NoseS = 19.0;
    const FStationSettings St;
    const std::vector<double> C = StationGateCentres(Sp, St);   // 11.5, 14.5, 17.5
    assert(C.size() == 3);
    // A 6 m train parked at the mark: rear 13, front 19. Rows at 14.5 and 17.5
    // have a car; the row at 11.5 has nothing and must stay shut.
    assert(!StationGateServed(C[0], 13.0, 19.0, false, 0.0));
    assert(StationGateServed(C[1], 13.0, 19.0, false, 0.0));
    assert(StationGateServed(C[2], 13.0, 19.0, false, 0.0));
    // Parked short by two metres and the front row is uncovered too.
    assert(!StationGateServed(C[2], 11.0, 17.0, false, 0.0));
    // On a circuit the span is read the short way round, across the seam.
    assert(StationGateServed(2.0, 1280.0, 8.0, true, 1288.0));
    assert(!StationGateServed(20.0, 1280.0, 8.0, true, 1288.0));
    assert(StationGateServed(1285.0, 1280.0, 8.0, true, 1288.0));
    std::printf("  rows at 14.5 and 17.5 served by a 13..19 m train, 11.5 not; seam-straddling span read the short way\n");
}

} // namespace

int main()
{
    std::printf("=== TrackStation assert suite ===\n");
    TestAGateOpensOnlyOntoACar();
    TestEveryBufferIsClosedAndOutward();
    TestThePlatformIsOnTheSideAskedFor();
    TestOneGatePerRowAndInsideTheSpan();
    TestTheGatesSwingOntoThePlatform();
    std::printf("All TrackStation tests passed.\n");
    return 0;
}
