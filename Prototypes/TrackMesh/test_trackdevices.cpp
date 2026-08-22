// Asserts for TrackDevices.h — the hardware that makes a zone look like one.
//
//   clang++ -std=c++17 -O2 -Wall -Wextra -o test_trackdevices test_trackdevices.cpp
//
// Same bar as the ties, the supports and the catwalk: closed, outward-wound
// geometry (signed volume and watertightness), placed INSIDE the span it was
// asked for and nowhere else, at a pitch that is a distance rather than a
// sample count, and nothing at all for a span with no hardware.

#include "TrackDevices.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <utility>
#include <vector>

namespace
{

FTrack Straight(double Len = 60.0)
{
    FTrack T;
    T.AddSegment(MakeStraight(Len));
    return T;
}

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
        V6 += Dot(M.Position[M.Index[t]],
                  Cross(M.Position[M.Index[t + 1]], M.Position[M.Index[t + 2]]));
    }
    return V6 / 6.0;
}

// Along-track extent of everything in a buffer, on a straight track along +X.
void Extent(const FMeshBuffer& M, double& MinX, double& MaxX)
{
    MinX = 1e9; MaxX = -1e9;
    for (const FVec3& P : M.Position) { MinX = std::min(MinX, P.X); MaxX = std::max(MaxX, P.X); }
}

FTrackDeviceMesh Build(unsigned Hardware, double A = 10.0, double B = 40.0, double Spacing = 0.5)
{
    const FTrack T = Straight();
    FTrackDeviceSpan Span; Span.StartS = A; Span.EndS = B; Span.Hardware = Hardware;
    return BuildDeviceHardware(WalkTrack(T, Spacing), {Span}, T.GetHeartlineHeight(), FTrackProfile());
}

// ===================== A BOX IS A BOX =====================
//
// Everything here is built from one primitive, so it gets its own check: closed,
// outward, and exactly the volume its half extents say. One reversed face drops
// the volume by a third, which is the failure that hid in the ties for months.
void TestTheBoxEnclosesExactlyItsVolume()
{
    std::printf("A box encloses exactly its own volume, outward-wound and watertight\n");
    FMeshBuffer M;
    DeviceDetail::AddBox(M, FVec3{3.0, -2.0, 1.0}, FVec3{1, 0, 0}, FVec3{0, 1, 0}, FVec3{0, 0, 1},
                         0.5, 0.25, 0.125);
    assert(M.NumTriangles() == 12);
    assert(BoundaryEdges(M) == 0);
    assert(std::fabs(SignedVolume(M) - 8.0 * 0.5 * 0.25 * 0.125) < 1e-12);

    // And on a basis that is rotated, because a fin on a banked brake is.
    FMeshBuffer R;
    const double c = std::cos(0.7), s = std::sin(0.7);
    DeviceDetail::AddBox(R, FVec3{0, 0, 0}, FVec3{1, 0, 0}, FVec3{0, c, s}, FVec3{0, -s, c},
                         1.0, 1.0, 1.0);
    assert(std::fabs(SignedVolume(R) - 8.0) < 1e-12);
    std::printf("  12 triangles, volume %.6f = %.6f\n", SignedVolume(M), 8.0 * 0.5 * 0.25 * 0.125);
}

// ===================== EVERY KIND IS CLOSED AND POSITIVE =====================
void TestEveryHardwareKindIsWatertightAndOutward()
{
    std::printf("Every hardware kind is watertight and outward-wound, in both buffers\n");
    const unsigned Kinds[] = {DeviceChain, DeviceCatch, DeviceFins, DeviceTyres, DeviceStators};
    const char* Names[] = {"chain", "catch", "fins", "tyres", "stators"};
    for (int k = 0; k < 5; ++k)
    {
        const FTrackDeviceMesh M = Build(Kinds[k]);
        assert(M.NumTriangles() > 0);
        assert(BoundaryEdges(M.Hardware) == 0);
        assert(BoundaryEdges(M.Rubber) == 0);
        assert(SignedVolume(M.Hardware) > 0.0);
        // Every kind puts something in BOTH buffers except the catch, which is
        // all steel: a dog rail has no rubber on it.
        if (Kinds[k] != DeviceCatch) { assert(SignedVolume(M.Rubber) > 0.0); }
        else { assert(M.Rubber.NumTriangles() == 0); }
        // EXACT for the sweep, because "positive" let inward caps through: a
        // 30 m trough of 0.18 x 0.12 is 0.648 m3, and the first version's caps
        // both faced inward and it came to a third of that.
        if (Kinds[k] == DeviceChain) { assert(std::fabs(SignedVolume(M.Hardware) - 30.0 * 0.18 * 0.12) < 1e-9); }
        std::printf("  %-8s %6zu triangles, steel %.4f m3, rubber %.4f m3\n", Names[k],
                    M.NumTriangles(), SignedVolume(M.Hardware), SignedVolume(M.Rubber));
    }
}

// ===================== INSIDE THE SPAN, AND NOWHERE ELSE =====================
//
// A brake fin that leaks a metre past its zone is a fin on open track, and a
// train's underside would pass through it. The continuous kinds stop at the
// frame interval containing the boundary, which at 0.5 m spacing is half a
// metre of slack and is asserted as such.
void TestHardwareStaysInsideItsSpan()
{
    std::printf("Hardware lies inside the span it was asked for\n");
    const unsigned Kinds[] = {DeviceChain, DeviceCatch, DeviceFins, DeviceTyres, DeviceStators};
    for (unsigned K : Kinds)
    {
        const FTrackDeviceMesh M = Build(K, 10.0, 40.0);
        double Lo, Hi;
        Extent(M.Hardware, Lo, Hi);
        assert(Lo >= 10.0 - 0.5 && Hi <= 40.0 + 0.5);
        if (M.Rubber.NumTriangles() > 0)
        {
            Extent(M.Rubber, Lo, Hi);
            assert(Lo >= 10.0 - 0.5 && Hi <= 40.0 + 0.5);
        }
    }
    // And a span with no hardware builds nothing, so a refusal stays a refusal.
    assert(Build(DeviceNone).NumTriangles() == 0);
    std::printf("  five kinds within [10, 40] m; none builds nothing\n");
}

// ===================== A PITCH IS A DISTANCE =====================
//
// Halve the walk spacing and a 30 m brake still has the same number of fins —
// the same assertion the supports make about bents. And the count is what the
// length says: 30 m at 2 m pitch is 15 fins, 10 tyre pairs at 3 m.
void TestPitchIsADistanceNotASampleCount()
{
    std::printf("A pitch is a distance, not a sample count\n");
    const FTrackDeviceMesh Coarse = Build(DeviceFins, 10.0, 40.0, 1.0);
    const FTrackDeviceMesh Fine = Build(DeviceFins, 10.0, 40.0, 0.25);
    // Each fin is one box of 12 triangles in steel.
    assert(Coarse.Hardware.NumTriangles() == 15 * 12);
    assert(Fine.Hardware.NumTriangles() == 15 * 12);

    const FTrackDeviceMesh Tyres = Build(DeviceTyres, 10.0, 40.0);
    // Ten pairs: two motors a pair in steel (12 triangles each).
    assert(Tyres.Hardware.NumTriangles() == 10 * 2 * 12);

    const FTrackDeviceMesh Teeth = Build(DeviceCatch, 10.0, 40.0, 0.5);
    const FTrackDeviceMesh TeethFine = Build(DeviceCatch, 10.0, 40.0, 0.1);
    // The rack is a rect sweep, 8 triangles an interval plus two caps, and it
    // changes with spacing; the teeth do not. 30 m at 0.3 m pitch is 100 teeth
    // of 12 triangles each.
    assert(Teeth.Hardware.NumTriangles() == 100 * 12 + 60 * 8 + 4);
    assert(TeethFine.Hardware.NumTriangles() == 100 * 12 + 300 * 8 + 4);
    std::printf("  15 fins at either spacing, 10 tyre pairs, 100 teeth at either spacing\n");
}

// ===================== A BLOCK BRAKE IS TWO MACHINES, DRAWN =====================
void TestZoneNamesMapToHardware()
{
    std::printf("A zone name maps to its hardware, and a block brake is fins AND tyres\n");
    assert(HardwareForZoneName("LIFT") == DeviceChain);
    assert(HardwareForZoneName("LAUNCH") == DeviceStators);
    assert(HardwareForZoneName("TRIM") == DeviceFins);
    assert(HardwareForZoneName("BLOCK BRAKE") == (DeviceFins | DeviceTyres));
    assert(HardwareForZoneName("STATION") == (DeviceFins | DeviceTyres));
    assert(HardwareForZoneName("LOAD") == (DeviceFins | DeviceTyres));
    assert(HardwareForZoneName("UNLOAD") == (DeviceFins | DeviceTyres));
    assert(HardwareForZoneName("nonsense") == DeviceNone);

    // And the combined span is both sets of geometry, not one or the other.
    const FTrackDeviceMesh Both = Build(DeviceFins | DeviceTyres);
    const FTrackDeviceMesh F = Build(DeviceFins);
    const FTrackDeviceMesh T = Build(DeviceTyres);
    assert(Both.NumTriangles() == F.NumTriangles() + T.NumTriangles());
}

// ===================== THE HARDWARE SITS BETWEEN THE RAILS =====================
//
// Laterally inside the gauge, and never above the rail top by more than the
// clearance a train's underside fin allows — which is the whole reason a brake
// fin is 0.12 m proud and not 0.5.
void TestHardwareSitsBetweenTheRails()
{
    std::printf("The hardware sits between the rails and barely above them\n");
    const FTrackProfile P;
    const unsigned Kinds[] = {DeviceChain, DeviceCatch, DeviceFins, DeviceTyres, DeviceStators};
    for (unsigned K : Kinds)
    {
        const FTrackDeviceMesh M = Build(K);
        for (const FMeshBuffer* B : {&M.Hardware, &M.Rubber})
        {
            for (const FVec3& V : B->Position)
            {
                assert(std::fabs(V.Y) < P.Gauge * 0.5 - P.RailDiameter * 0.5);
                // Z = 0 is the heartline on this flat track; the rail centre is
                // HeartlineHeight below it.
                assert(V.Z < -Straight().GetHeartlineHeight() + 0.13);
            }
        }
    }
}

} // namespace

int main()
{
    std::printf("=== TrackDevices assert suite ===\n");
    TestTheBoxEnclosesExactlyItsVolume();
    TestEveryHardwareKindIsWatertightAndOutward();
    TestHardwareStaysInsideItsSpan();
    TestPitchIsADistanceNotASampleCount();
    TestZoneNamesMapToHardware();
    TestHardwareSitsBetweenTheRails();
    std::printf("All TrackDevices tests passed.\n");
    return 0;
}
