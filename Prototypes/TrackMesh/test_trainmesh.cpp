// Asserts for TrainMesh.h — the train, which was nine engine cubes until now.
//
//   clang++ -std=c++17 -O2 -Wall -Wextra -o test_trainmesh test_trainmesh.cpp
//
// Three classes of failure are worth the file, and each of them produces
// geometry that LOOKS plausible:
//
//   THE WRONG LIST. Drawing off the physics sample points instead of the
//   authored cars gives a train of the right total length made of the wrong
//   number of the wrong-sized things — which is exactly what shipped, and which
//   looked fine at 15 m and absurd at 6 m.
//
//   INSIDE OUT. The port rule was got backwards for months and every surface on
//   the ride was reversed; it hid because a thin tube inside out has the same
//   silhouette. A car body is SOLID, so this file is the first thing since that
//   would show it — and signed volume is how it is caught rather than by looking.
//
//   THE WHEELS NOT ON THE RAIL. Running, side and upstop wheels at offsets that
//   are nearly right read as a coaster from any distance at all, and are the one
//   part of this that a screenshot cannot check.

#include "TrainMesh.h"
#include "../TrainPhysics/Seat.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

namespace
{

constexpr double Pi = 3.14159265358979323846;

FTrack Straight(double Len = 60.0)
{
    FTrack T;
    T.AddSegment(MakeStraight(Len));
    return T;
}

// A tight-ish curve, because articulation is invisible on anything gentle: a
// 20 m radius is about as hard as a real layout turns and is where a rigid body
// chording across the arc actually shows.
FTrack Curve(double Radius = 20.0, double Len = 60.0)
{
    FTrack T;
    T.AddSegment(MakeArc(Len, Radius));
    return T;
}

// Every undirected edge shared by exactly two triangles.
//
// WELDED BY POSITION, NOT BY INDEX — the caps duplicate a whole rim and the
// section sweep deliberately duplicates every vertex per edge for flat shading,
// so two different indices at the same point are the same edge to this question.
// Comparing indices would report a hole at every corner of the body.
std::size_t BoundaryEdges(const FMeshBuffer& M)
{
    using FKey = std::tuple<long long, long long, long long>;
    auto Key = [](const FVec3& P) {
        return FKey{std::llround(P.X * 1e6), std::llround(P.Y * 1e6), std::llround(P.Z * 1e6)};
    };
    std::map<FKey, std::uint32_t> Weld;
    auto Welded = [&](std::uint32_t Index) {
        const FKey K = Key(M.Position[Index]);
        auto It = Weld.find(K);
        if (It != Weld.end()) { return It->second; }
        const std::uint32_t Id = static_cast<std::uint32_t>(Weld.size());
        Weld[K] = Id;
        return Id;
    };

    std::map<std::pair<std::uint32_t, std::uint32_t>, int> Edges;
    for (std::size_t t = 0; t + 2 < M.Index.size(); t += 3)
    {
        for (int e = 0; e < 3; ++e)
        {
            const std::uint32_t A = Welded(M.Index[t + static_cast<std::size_t>(e)]);
            const std::uint32_t B = Welded(M.Index[t + static_cast<std::size_t>((e + 1) % 3)]);
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

void CheckWindingAgreesWithNormals(const FMeshBuffer& M, const char* Where)
{
    for (std::size_t t = 0; t + 2 < M.Index.size(); t += 3)
    {
        const std::uint32_t Ia = M.Index[t], Ib = M.Index[t + 1], Ic = M.Index[t + 2];
        const FVec3 Face = Cross(M.Position[Ib] - M.Position[Ia],
                                 M.Position[Ic] - M.Position[Ia]);
        const FVec3 Avg = (M.Normal[Ia] + M.Normal[Ib] + M.Normal[Ic]) * (1.0 / 3.0);
        if (!(Dot(Face, Avg) > 0.0))
        {
            std::printf("  INSIDE OUT at triangle %zu in %s (dot %.9f)\n",
                        t / 3, Where, Dot(Face, Avg));
            assert(false);
        }
    }
}

void CheckIndicesInRange(const FMeshBuffer& M)
{
    for (std::uint32_t I : M.Index) { assert(I < M.Position.size()); }
    assert(M.Index.size() % 3 == 0);
    assert(M.Normal.size() == M.Position.size());
    assert(M.UV.size() == M.Position.size());
}

// ------------------------------------------------------------------- the tests

// ===================== THE WHOLE REASON THE CARD EXISTS =====================
void TestTheMeshIsBuiltFromCARSNotFromSAMPLEPOINTS()
{
    std::printf("A sample point is not a car\n");

    const FTrackProfile P;
    FTrainSettings S;                 // 5 cars x 3 m = 15 m, the reference train
    const FTrainMesh Five = BuildCarMesh(S, 1.1, P);

    // The 6 m small-batch vehicle: TWO cars, not nine playing cards on edge.
    // Same car geometry, so one car's mesh is IDENTICAL — which is the point:
    // the car is the unit, and the train is however many of them.
    FTrainSettings Small = S;
    Small.CarCount = 2;
    const FTrainMesh Two = BuildCarMesh(Small, 1.1, P);
    assert(Two.NumTriangles() == Five.NumTriangles());
    assert(Two.NumVertices() == Five.NumVertices());
    std::printf("  car geometry does not depend on how many cars there are\n");

    // And the TRAIN scales with the authored count rather than with nine.
    const FTrack T = Straight(200.0);
    const double Spacing = 0.5;
    const std::vector<FTrackFrame> Path = WalkTrack(T, Spacing);
    const double Total = T.TotalLength();

    const std::vector<FCarPlacement> A = PlaceCars(Path, Spacing, Total, 100.0, S, false);
    const std::vector<FCarPlacement> B = PlaceCars(Path, Spacing, Total, 100.0, Small, false);
    assert(A.size() == 5 && B.size() == 2);

    // Nine would be the old answer for BOTH of these, and it was.
    assert(A.size() != 9 && B.size() != 9);
    std::printf("  a 15 m train is 5 cars and a 6 m one is 2, where both used to be 9\n");

    // The authored length is the product, exactly — no rounding, no drift.
    assert(std::fabs(S.TrainLengthM() - 15.0) < 1e-12);
    assert(std::fabs(Small.TrainLengthM() - 6.0) < 1e-12);

    // Cars are half a pitch behind the nose and a pitch apart, in travel order.
    assert(std::fabs(A[0].S - (100.0 - 1.5)) < 1e-9);
    for (std::size_t i = 0; i + 1 < A.size(); ++i)
    {
        assert(std::fabs((A[i].S - A[i + 1].S) - S.CarLengthM) < 1e-9);
    }
    std::printf("  the nose is the reference, and the cars sit one pitch apart behind it\n");
}

// ===================== SOLID, AND THE RIGHT WAY OUT =====================
void TestEveryPartIsCLOSEDAndENCLOSESVolume()
{
    std::printf("Every part is watertight and wound outward\n");

    const FTrackProfile P;
    const FTrainSettings S;
    const FTrainMesh C = BuildCarMesh(S, 1.1, P);

    struct FPart { const FMeshBuffer* M; const char* Name; };
    const FPart Parts[] = {{&C.Body, "body"}, {&C.Chassis, "chassis"}, {&C.Wheels, "wheels"},
                           {&C.Seats, "seats"}};

    for (const FPart& Part : Parts)
    {
        CheckIndicesInRange(*Part.M);
        CheckWindingAgreesWithNormals(*Part.M, Part.Name);

        const std::size_t Open = BoundaryEdges(*Part.M);
        const double V = SignedVolume(*Part.M);
        std::printf("  %-8s %6zu tris, open edges %zu, volume %+.6f m^3\n",
                    Part.Name, Part.M->NumTriangles(), Open, V);
        assert(Open == 0 && "a closed part has no open rim");
        assert(V > 0.0 && "an outward-wound closed mesh has positive signed volume");
    }

    // ---- The chassis beam's volume is a rectangular prism, exactly. A number
    // that can be worked out by hand is worth more than one that merely looks
    // positive, because it catches a section that closed but came out the wrong
    // size — which a sign check never would.
    const double Expected = S.ChassisWidthM * S.ChassisDepthM * (ChassisHalfLength(S) * 2.0);
    const double Got = SignedVolume(C.Chassis);
    std::printf("  chassis %.8f m^3 against a hand-computed %.8f m^3\n", Got, Expected);
    assert(std::fabs(Got - Expected) < 1e-9);

    // ---- And the check BITES. Reverse one triangle and the volume must move;
    // a test that cannot fail is decoration. This is the assertion that would
    // have caught the port rule being backwards, and the reason it is here is
    // that a car body is the first SOLID thing this project draws.
    FMeshBuffer Flipped = C.Body;
    std::swap(Flipped.Index[1], Flipped.Index[2]);
    assert(std::fabs(SignedVolume(Flipped) - SignedVolume(C.Body)) > 1e-12);
    std::printf("  reversing one triangle moves the volume, so the check bites\n");
}

// ===================== THE WHEELS ARE ON THE RAIL =====================
//
// The part a screenshot cannot check, and the part that makes it read as a
// coaster rather than a bus. Three sets gripping one rail from three directions,
// at offsets taken from the profile rather than typed.
void TestTheThreeWheelSetsGRIPTheRail()
{
    std::printf("Three wheel sets, gripping one rail from three directions\n");

    const FTrackProfile P;
    FTrainSettings S;
    const double H = 1.1;
    const FTrainMesh C = BuildCarMesh(S, H, P);

    const double RailZ = -H;
    const double RailY = P.Gauge * 0.5;          // the rider's LEFT rail
    const double RailR = P.RailDiameter * 0.5;
    const double BogieX = BogieOffset(S);
    const FVec3 Across{0.0, 1.0, 0.0};
    const FVec3 Vertical{0.0, 0.0, 1.0};

    // The three wheel centres this predicts, on the left rail at the front bogie.
    const FVec3 Running{BogieX, RailY, RailZ + RailR + S.RunningWheelDiameterM * 0.5};
    const FVec3 Upstop {BogieX, RailY, RailZ - RailR - S.UpstopWheelDiameterM * 0.5};
    const FVec3 Side   {BogieX, RailY - (RailR + S.SideWheelDiameterM * 0.5), RailZ};

    // A wheel is a capped cylinder, so its centre is the middle of its own
    // bounding box - exact for an even-sided prism, where the MEAN of the
    // vertices is not: the UV seam deliberately duplicates one column and both
    // cap fans carry a centre vertex, so an average is pulled off-axis by the
    // tessellation rather than by the geometry.
    //
    // SELECTED BY CYLINDER MEMBERSHIP RATHER THAN BY A RADIUS. A plain distance
    // probe was the first version and it quietly swallowed part of the SIDE
    // wheel, which sits 0.168 m from the running wheel centre where the running
    // wheel own vertices reach 0.158 - a 10 mm window, which is not a test, it
    // is a coincidence. Axial extent and radial extent are what actually say
    // which wheel a vertex belongs to, and they cannot overlap.
    auto WheelCentre = [&](const FVec3& Want, const FVec3& Axis, double Dia,
                           double Width, int& OutCount)
    {
        const double R = Dia * 0.5 + 1e-9;
        const double H = Width * 0.5 + 1e-9;
        FVec3 Lo{0.0, 0.0, 0.0}, Hi{0.0, 0.0, 0.0};
        int N = 0;
        for (const FVec3& V : C.Wheels.Position)
        {
            const FVec3 D = V - Want;
            const double Ax = Dot(D, Axis);
            if (std::fabs(Ax) > H) { continue; }
            if (Length(D - Axis * Ax) > R) { continue; }
            if (N == 0) { Lo = V; Hi = V; }
            Lo.X = std::fmin(Lo.X, V.X); Hi.X = std::fmax(Hi.X, V.X);
            Lo.Y = std::fmin(Lo.Y, V.Y); Hi.Y = std::fmax(Hi.Y, V.Y);
            Lo.Z = std::fmin(Lo.Z, V.Z); Hi.Z = std::fmax(Hi.Z, V.Z);
            ++N;
        }
        OutCount = N;
        return N > 0 ? (Lo + Hi) * 0.5 : FVec3{0.0, 0.0, 0.0};
    };

    struct FCheck { FVec3 Want; FVec3 Axis; const char* Name; double Dia; };
    const FCheck Checks[] = {
        {Running, Across,   "running", S.RunningWheelDiameterM},
        {Upstop,  Across,   "upstop",  S.UpstopWheelDiameterM},
        {Side,    Vertical, "side",    S.SideWheelDiameterM},
    };

    for (const FCheck& Ck : Checks)
    {
        int N = 0;
        const FVec3 Got = WheelCentre(Ck.Want, Ck.Axis, Ck.Dia, S.WheelWidthM, N);
        assert(N > 0 && "no wheel found where one was predicted");
        const double Err = Length(Got - Ck.Want);
        std::printf("  %-8s wheel at (%.4f, %.4f, %.4f), %d vertices, error %.9f m\n",
                    Ck.Name, Got.X, Got.Y, Got.Z, N, Err);
        assert(Err < 1e-9);
    }

    // ---- RUNNING WHEELS SIT ON TOP AND UPSTOPS UNDERNEATH, which is the
    // relationship rather than the coordinates: get the sign wrong and the train
    // is held down by the wheels that carry it and vice versa. The upstops are
    // why a train survives -0.94 g on the showcase instead of leaving the track.
    assert(Running.Z > RailZ && Upstop.Z < RailZ);
    assert(std::fabs(Running.Z - RailZ) > RailR);
    assert(std::fabs(Upstop.Z - RailZ) > RailR);

    // ---- SIDE FRICTION WHEELS BEAR ON THE INNER FACE, so they sit between the
    // rails rather than outside them. Outside is the single most plausible-
    // looking way to get this wrong and it is a train gripping thin air.
    assert(std::fabs(Side.Y) < RailY);
    std::printf("  running above, upstops below, side wheels INBOARD of the rail\n");

    // ---- Twelve wheels: two assemblies, six each. Counted from the geometry
    // rather than trusted, by how many disjoint clusters there are.
    int Total = 0;
    for (int End = 0; End < 2; ++End)
    {
        for (int Sd = 0; Sd < 2; ++Sd)
        {
            const double X = End == 0 ? -BogieX : BogieX;
            const double Sign = Sd == 0 ? 1.0 : -1.0;
            const double Y = P.Gauge * 0.5 * Sign;
            int N = 0;
            WheelCentre(FVec3{X, Y, RailZ + RailR + S.RunningWheelDiameterM * 0.5},
                        Across, S.RunningWheelDiameterM, S.WheelWidthM, N);
            Total += N > 0 ? 1 : 0;
            WheelCentre(FVec3{X, Y, RailZ - RailR - S.UpstopWheelDiameterM * 0.5},
                        Across, S.UpstopWheelDiameterM, S.WheelWidthM, N);
            Total += N > 0 ? 1 : 0;
            WheelCentre(FVec3{X, Y - Sign * (RailR + S.SideWheelDiameterM * 0.5), RailZ},
                        Vertical, S.SideWheelDiameterM, S.WheelWidthM, N);
            Total += N > 0 ? 1 : 0;
        }
    }
    std::printf("  %d wheels on the car: two assemblies of six\n", Total);
    assert(Total == 12);
}

// ===================== THE BODY MUST NOT SWALLOW THE RIDER =====================
void TestTheBodyLeavesTheHEARTLINEInTheOpen()
{
    std::printf("The body stops below the heartline, and says so when it does not\n");

    const FTrackProfile P;
    const FTrainSettings S;
    const double H = 1.1;

    // The floor sits ON the rail plane, so the roof is the floor plus the height.
    const FTrainMesh C = BuildCarMesh(S, H, P);
    double Top = C.Body.Position[0].Z, Bottom = C.Body.Position[0].Z;
    for (const FVec3& V : C.Body.Position)
    {
        Top = V.Z > Top ? V.Z : Top;
        Bottom = V.Z < Bottom ? V.Z : Bottom;
    }
    std::printf("  body spans %.3f to %.3f m about the heartline (rails at %.3f)\n",
                Bottom, Top, -H);
    assert(std::fabs(Bottom - (-H)) < 1e-9 && "the floor sits on the rail plane");
    assert(Top < 0.0 && "the rim must stay below the heartline the camera sits at");

    // ---- AND IT IS A TUB, NOT A BOX. The first lap bars closed into a roofed
    // shell and vanished. A point just under the rim on the centreline is OPEN
    // AIR; a point in the wall is shell. Point-in-polygon on the section.
    const std::vector<FVec2> Sec = CarBodySection(S, ShellKeepOut(S, P));
    auto Inside = [&](FVec2 Q)
    {
        bool bIn = false;
        for (std::size_t i = 0, j = Sec.size() - 1; i < Sec.size(); j = i++)
        {
            const FVec2& A = Sec[i];
            const FVec2& B = Sec[j];
            if ((A.V > Q.V) != (B.V > Q.V)
                && Q.U < (B.U - A.U) * (Q.V - A.V) / (B.V - A.V) + A.U)
            {
                bIn = !bIn;
            }
        }
        return bIn;
    };
    const double Hh = S.BodyHeightM * 0.5;
    assert(!Inside({0.0, Hh - 0.01}) && "open on top: the cabin is air, not shell");
    assert(Inside({S.BodyWidthM * 0.5 - 0.02, Hh - S.BodyCornerRadiusM - 0.05}) && "the wall is shell");
    const double CabinZ = -H + CabinFloorHeight(S, ShellKeepOut(S, P));
    assert(!Inside({0.0, CabinZ - (-H) - Hh + 0.01}) && "air just above the cabin floor");
    assert(Inside({0.0, CabinZ - (-H) - Hh - 0.01}) && "shell just below it");
    std::printf("  open on top: cabin floor %.2f m above the rails, rim at %.2f\n",
                CabinZ - (-H), S.BodyHeightM);

    // ---- AND CLOSED AT BOTH ENDS. The tub's caps close the shell's thickness
    // and nothing else, so without a bulkhead the cabin is open front and
    // back. Looked for as geometry: a body triangle within the end of the
    // shell whose centroid is INSIDE the cabin — above the floor, between the
    // walls — which the tub alone never has, because its cap triangles all lie
    // in the shell material.
    {
        const double ShellHalf = (S.CarLengthM - S.BodyGapM) * 0.5;
        const double Wi = S.BodyWidthM * 0.5 - S.BodyCornerRadiusM;
        for (int End = 0; End < 2; ++End)
        {
            bool bClosed = false;
            for (std::size_t t = 0; t + 2 < C.Body.Index.size(); t += 3)
            {
                FVec3 Cen{0.0, 0.0, 0.0};
                bool bAtEnd = true;
                for (int k = 0; k < 3; ++k)
                {
                    const FVec3& V = C.Body.Position[C.Body.Index[t + k]];
                    Cen = Cen + V * (1.0 / 3.0);
                    bAtEnd = bAtEnd && (End == 0 ? V.X : -V.X) > ShellHalf - S.EndPanelThickM - 0.02;
                }
                if (bAtEnd && Cen.Z > CabinZ + 0.05 && std::fabs(Cen.Y) < Wi - 0.05) { bClosed = true; }
            }
            assert(bClosed && "a bulkhead closes the cabin at each end");
        }
        std::printf("  and closed at both ends: a %.2f m bulkhead from the cabin floor to the rim\n",
                    S.EndPanelThickM);
    }

    // ---- SILENT ON AN ORDINARY TRAIN. The checks that matter are the ones that
    // say nothing about the normal case; one that complained about every vehicle
    // would be switched off within a day.
    assert(AuditTrain(S, H, P, 1288.0).empty());
    std::printf("  and an ordinary train reports nothing at all\n");

    // ---- RAISE IT AND IT COMPLAINS. 0.9 m is not a guess and must not be
    // raised; a shell taller than the heartline closes over the point the
    // rider's eye and the rider's felt G are both measured at.
    FTrainSettings Tall = S;
    Tall.BodyHeightM = 1.30;
    const std::vector<FMeshFinding> F = AuditTrain(Tall, H, P, 1288.0);
    assert(F.size() == 1);
    std::printf("  %s\n", F[0].What.c_str());
    assert(BodySwallowsHeartline(Tall, H));
}

// ===================== THE SHELL CLEARS EVERY WHEEL IT PASSES =====================
//
// THE TEST BELOW THIS ONE USED TO BE THE WHOLE STORY, AND IT WAS HALF OF IT. It
// measured the shell at the floor line against the RAILS, while its own comment
// claimed the wheels were the reason -- so the shell was drawn straight through
// the running wheels by 0.168 m and through the side-friction wheels by 0.167 m
// and the suite said nothing. The first was reported from a screenshot. The
// second could not be: it is buried inside the floor, where nothing can see it.
//
// A WHEEL IS A BAND OF HEIGHTS, NOT A POINT, which is the whole reason one Z
// sample missed it. What matters is the widest the shell gets ANYWHERE in the
// band a wheel occupies, and that maximum falls between section vertices rather
// than on one -- so this walks the section POLYLINE instead of its points.
static double ShellHalfWidthAt(const std::vector<FVec2>& Sec, double BodyCentreZ, double Z)
{
    const double B = Z - BodyCentreZ;
    double Best = -1.0;
    for (std::size_t i = 0; i < Sec.size(); ++i)
    {
        const FVec2& P = Sec[i];
        const FVec2& Q = Sec[(i + 1) % Sec.size()];
        const double Lo = std::min(P.V, Q.V), Hi = std::max(P.V, Q.V);
        if (B < Lo - 1e-9 || B > Hi + 1e-9) { continue; }
        const double T = std::fabs(Q.V - P.V) < 1e-12 ? 0.0 : (B - P.V) / (Q.V - P.V);
        const double A = P.U + T * (Q.U - P.U);
        Best = std::max(Best, std::fabs(A));
    }
    return Best;
}

// The worst (smallest) clearance between the shell and any wheel, in metres.
// Negative means the shell is inside a wheel.
static double WorstWheelClearance(const FTrainSettings& S, const FTrackProfile& P,
                                  double HeartlineHeight, const char** Which)
{
    const double RailZ = -HeartlineHeight;
    const double RailR = P.RailDiameter * 0.5;
    const double HalfGauge = P.Gauge * 0.5;
    const double BodyCentreZ = RailZ + S.BodyHeightM * 0.5;
    const std::vector<FVec2> Sec = CarBodySection(S, ShellKeepOut(S, P));

    struct W { const char* Name; double Zlo, Zhi, InnerY; };
    const double RunC = RailZ + RailR + S.RunningWheelDiameterM * 0.5;
    const double UpC = RailZ - RailR - S.UpstopWheelDiameterM * 0.5;
    const double SideY = HalfGauge - (RailR + S.SideWheelDiameterM * 0.5);
    const W Wheels[] = {
        {"running", RunC - S.RunningWheelDiameterM * 0.5, RunC + S.RunningWheelDiameterM * 0.5,
         HalfGauge - S.WheelWidthM * 0.5},
        {"upstop",  UpC - S.UpstopWheelDiameterM * 0.5,  UpC + S.UpstopWheelDiameterM * 0.5,
         HalfGauge - S.WheelWidthM * 0.5},
        {"side",    RailZ - S.WheelWidthM * 0.5,          RailZ + S.WheelWidthM * 0.5,
         SideY - S.SideWheelDiameterM * 0.5},
    };

    double Worst = 1e9;
    for (const W& w : Wheels)
    {
        for (int i = 0; i <= 400; ++i)
        {
            const double Z = w.Zlo + (w.Zhi - w.Zlo) * (i / 400.0);
            const double HW = ShellHalfWidthAt(Sec, BodyCentreZ, Z);
            if (HW < 0.0) { continue; }   // the shell is not at this height at all
            const double Clear = w.InnerY - HW;
            if (Clear < Worst) { Worst = Clear; if (Which) { *Which = w.Name; } }
        }
    }
    return Worst;
}

void TestTheShellCLEARSEveryWheelItPasses()
{
    std::printf("The shell clears every wheel it passes\n");

    const FTrackProfile P;
    const FTrainSettings S;
    const char* Which = "";
    const double Clear = WorstWheelClearance(S, P, 1.1, &Which);
    std::printf("  worst clearance %.4f m, at the %s wheel\n", Clear, Which);
    assert(Clear > 0.0 && "the shell is drawn through its own wheels");
    assert(Clear >= S.WheelClearanceM - 1e-6 && "the shell is closer than the fitted gap");

    // ---- IT IS DERIVED, WHICH IS THE CLAIM WORTH ASSERTING. Three numbers in
    // settings would clear these wheels too, and would go on clearing THESE
    // wheels after somebody changed a diameter. Move the hardware and the shell
    // has to move out of its way on its own, or this is taste wearing a formula.
    const double Gauges[] = {0.90, 1.10, 1.35};
    const double RunDias[] = {0.24, 0.30, 0.42};
    for (double G : Gauges)
    {
        for (double D : RunDias)
        {
            FTrackProfile P2 = P;
            P2.Gauge = G;
            FTrainSettings S2 = S;
            S2.RunningWheelDiameterM = D;
            const char* W2 = "";
            const double C2 = WorstWheelClearance(S2, P2, 1.1, &W2);
            assert(C2 > 0.0 && "a changed wheel put the shell back inside one");
        }
    }
    std::printf("  and it stays clear across 9 gauge x wheel-size combinations\n");

    // ---- AND IT BITES. Zero the fitted gap and ask for a floor as wide as the
    // car: the clamp still keeps geometry out of the wheels, so what proves the
    // check works is that the clearance collapses to exactly nothing.
    FTrainSettings Tight = S;
    Tight.WheelClearanceM = 0.0;
    Tight.BodyFloorWidthM = 1.40;
    const double Touching = WorstWheelClearance(Tight, P, 1.1, nullptr);
    std::printf("  with no fitted gap the shell touches at %.6f m\n", Touching);
    assert(std::fabs(Touching) < 1e-6 && "the clamp is not what is holding it out");

    // ---- And asking for that floor is REPORTED, because the author asked for
    // something they did not get.
    const std::vector<FMeshFinding> F = AuditTrain(Tight, 1.1, P, 1288.0);
    assert(F.size() == 1);
    std::printf("  %s\n", F[0].What.c_str());
}

// ===================== THE FLOOR CLEARS THE RAILS =====================
//
// The taper is not styling. The floor line runs through the rail plane, so a
// body that went full width all the way down is drawn through both running
// rails — and the wheels, the entire reason this reads as a coaster, end up
// buried inside it.
void TestTheTaperedFloorClearsBOTHRunningRails()
{
    std::printf("The tapered floor clears the running rails\n");

    const FTrackProfile P;
    const FTrainSettings S;
    const FTrainMesh C = BuildCarMesh(S, 1.1, P);

    // Widest lateral extent found anywhere at the floor line.
    const double FloorZ = -1.1;
    double WidestAtFloor = 0.0;
    for (const FVec3& V : C.Body.Position)
    {
        if (std::fabs(V.Z - FloorZ) < 1e-6)
        {
            WidestAtFloor = std::fabs(V.Y) > WidestAtFloor ? std::fabs(V.Y) : WidestAtFloor;
        }
    }
    const double RailInner = P.Gauge * 0.5 - P.RailDiameter * 0.5;
    std::printf("  floor half-width %.4f m against %.4f m of clear rail\n",
                WidestAtFloor, RailInner);
    assert(WidestAtFloor > 0.0);
    assert(WidestAtFloor < RailInner && "the floor is drawn through the rails");

    // And it really is a TAPER — the roof is wider than the floor, or this test
    // would pass just as well on a body that was narrow all the way up.
    double Widest = 0.0;
    for (const FVec3& V : C.Body.Position)
    {
        Widest = std::fabs(V.Y) > Widest ? std::fabs(V.Y) : Widest;
    }
    assert(Widest > WidestAtFloor + 0.1);
    std::printf("  and the shell is %.2f m across higher up, so it really tapers\n",
                Widest * 2.0);

    // ---- Widen the floor past the rails and it is REPORTED, never repaired.
    FTrainSettings Fat = S;
    Fat.BodyFloorWidthM = 1.30;
    const std::vector<FMeshFinding> F = AuditTrain(Fat, 1.1, P, 1288.0);
    assert(F.size() == 1);
    std::printf("  %s\n", F[0].What.c_str());
}

// ===================== ARTICULATION, WHICH COMES FREE =====================
//
// Each car takes its frame from its OWN arc length, so through a curve the rigid
// bodies chord across the arc while the wheels stay on it. It is the motion
// nobody manages to fake, and it is not simulated — it falls out of not
// interpolating one transform for the whole train.
void TestCarsCHORDAcrossACurveRatherThanBendingWithIt()
{
    std::printf("Cars chord across a curve, which is what articulation is\n");

    const FTrainSettings S;
    const double Radius = 20.0;
    const double Spacing = 0.5;
    const FTrack T = Curve(Radius, 60.0);
    const std::vector<FTrackFrame> Path = WalkTrack(T, Spacing);
    const double Total = T.TotalLength();

    const std::vector<FCarPlacement> Cars = PlaceCars(Path, Spacing, Total, 40.0, S, false);
    assert(Cars.size() == 5);

    // ---- EVERY CAR HAS ITS OWN HEADING. One transform for the whole train is
    // precisely the thing this does not do, and on a 20 m radius the difference
    // between the front car and the back one is large and obvious.
    const double Cos = Dot(Cars.front().Frame.Tangent, Cars.back().Frame.Tangent);
    const double Deg = std::acos(std::max(-1.0, std::min(1.0, Cos))) * 180.0 / Pi;
    std::printf("  front car and back car differ by %.1f degrees of heading\n", Deg);
    // 12 m of a 20 m radius is 12/20 rad = 34.4 degrees.
    assert(std::fabs(Deg - (4.0 * S.CarLengthM / Radius) * 180.0 / Pi) < 0.5);

    // ---- AND THE BODY CHORDS. A rigid car placed on an arc has its ENDS off
    // the arc, inboard of it, by the sagitta — which is the visible effect and
    // the thing a bent body would not have.
    //
    // Measured against the track's own centre, which for an arc starting along
    // +X and turning is Radius away along the frame's own lateral.
    const FTrackFrame& F = Cars[2].Frame;
    const FVec3 Centre = F.Position + F.Lateral * Radius;
    const FVec3 Nose = CarToWorld(F, FVec3{S.CarLengthM * 0.5, 0.0, 0.0});
    const double RNose = Length(Nose - Centre);

    // A rigid car is TANGENT to the arc at its own centre, so each end sits
    // sqrt(R^2 + h^2) - R OUTSIDE the arc, where h is half the pitch. That is
    // not the chord sagitta, which is the inward deflection of the arc from a
    // chord and is a different number - 0.0563 against 0.0562 here, close
    // enough to look right and wrong enough to mean the body was being placed
    // some other way.
    const double H = S.CarLengthM * 0.5;
    const double Overhang = std::sqrt(Radius * Radius + H * H) - Radius;
    std::printf("  a car end sits %.6f m outside the arc; a rigid chord says %.6f m",
                RNose - Radius, Overhang);
    std::printf(" (%.1f mm)", Overhang * 1000.0);
    std::printf("%c", 10);
    assert(RNose > Radius + 1e-6 && "a rigid car end must sit OUTSIDE the arc");
    assert(std::fabs((RNose - Radius) - Overhang) < 1e-9);

    // ---- ON A STRAIGHT THERE IS NO ARTICULATION AT ALL, which is the other
    // half: a check that fires on straight track is measuring something else.
    const FTrack St = Straight(60.0);
    const std::vector<FTrackFrame> SPath = WalkTrack(St, Spacing);
    const std::vector<FCarPlacement> SCars =
        PlaceCars(SPath, Spacing, St.TotalLength(), 40.0, S, false);
    for (const FCarPlacement& C : SCars)
    {
        assert(std::fabs(Dot(C.Frame.Tangent, SCars[0].Frame.Tangent) - 1.0) < 1e-12);
    }
    std::printf("  and on a straight every car shares one heading, exactly\n");
}

// ===================== THE COUPLERS REACH =====================
void TestCouplersJOINAdjacentCarsOnAStraightAndACurve()
{
    std::printf("Couplers join adjacent cars, straight and curved\n");

    const FTrackProfile P;
    const FTrainSettings S;
    const double Spacing = 0.5;

    for (int bCurved = 0; bCurved < 2; ++bCurved)
    {
        const FTrack T = bCurved ? Curve(20.0, 60.0) : Straight(60.0);
        const std::vector<FTrackFrame> Path = WalkTrack(T, Spacing);
        const std::vector<FCarPlacement> Cars =
            PlaceCars(Path, Spacing, T.TotalLength(), 40.0, S, false);

        const FTrainMesh Car = BuildCarMesh(S, 1.1, P);
        const FTrainMesh Train = BuildTrainMesh(Cars, Car, S, 1.1);

        // Four couplers for five cars, and each one closed — a strut is capped at
        // both ends by definition, so an open rim here is a length of pipe.
        CheckIndicesInRange(Train.Couplers);
        CheckWindingAgreesWithNormals(Train.Couplers, "couplers");
        assert(BoundaryEdges(Train.Couplers) == 0);
        assert(SignedVolume(Train.Couplers) > 0.0);

        // The whole train is five cars' worth of every other part.
        assert(Train.Body.NumTriangles() == Car.Body.NumTriangles() * 5);
        assert(Train.Wheels.NumTriangles() == Car.Wheels.NumTriangles() * 5);

        std::printf("  %-8s train: %zu triangles, %zu of them couplers\n",
                    bCurved ? "curved" : "straight",
                    Train.NumTriangles(), Train.Couplers.NumTriangles());
    }

    // ---- A ONE-CAR TRAIN HAS NO COUPLERS, and that is a real case rather than a
    // degenerate one: a single-car vehicle is what a wild mouse is.
    FTrainSettings One = S;
    One.CarCount = 1;
    const FTrack T = Straight(60.0);
    const std::vector<FTrackFrame> Path = WalkTrack(T, Spacing);
    const std::vector<FCarPlacement> Cars =
        PlaceCars(Path, Spacing, T.TotalLength(), 40.0, One, false);
    const FTrainMesh Train = BuildTrainMesh(Cars, BuildCarMesh(One, 1.1, P), One, 1.1);
    assert(Cars.size() == 1);
    assert(Train.Couplers.NumTriangles() == 0);
    std::printf("  a one-car train has no couplers rather than a stub\n");
}

// ===================== THE FRAME BETWEEN TWO SAMPLES =====================
void TestFrameAtDistanceLANDSOnItsOwnSamplesAndStaysOrthonormal()
{
    std::printf("An interpolated frame is exact at the samples and square between\n");

    const double Spacing = 0.5;
    const FTrack T = Curve(25.0, 40.0);
    const std::vector<FTrackFrame> Path = WalkTrack(T, Spacing);
    const double Total = T.TotalLength();

    // ---- Exact ON a sample. Anything else means the index arithmetic is off by
    // a fraction everywhere, which reads as a train that floats.
    for (std::size_t i = 0; i + 1 < Path.size(); ++i)
    {
        const FTrackFrame F = FrameAtDistance(Path, Spacing, Total,
                                              Spacing * static_cast<double>(i));
        assert(Length(F.Position - Path[i].Position) < 1e-12);
    }
    std::printf("  every sample returns itself, exactly\n");

    // ---- ORTHONORMAL BETWEEN THEM. A lerp of two unit vectors is not a unit
    // vector; without the re-orthonormalisation a body is visibly not square to
    // its own wheels, and the error grows with how hard the track is turning.
    double WorstUnit = 0.0, WorstPerp = 0.0;
    for (double S = 0.0; S <= Total; S += 0.07)
    {
        const FTrackFrame F = FrameAtDistance(Path, Spacing, Total, S);
        WorstUnit = std::max(WorstUnit, std::fabs(Length(F.Tangent) - 1.0));
        WorstUnit = std::max(WorstUnit, std::fabs(Length(F.Lateral) - 1.0));
        WorstUnit = std::max(WorstUnit, std::fabs(Length(F.Up) - 1.0));
        WorstPerp = std::max(WorstPerp, std::fabs(Dot(F.Tangent, F.Lateral)));
        WorstPerp = std::max(WorstPerp, std::fabs(Dot(F.Tangent, F.Up)));
        WorstPerp = std::max(WorstPerp, std::fabs(Dot(F.Lateral, F.Up)));
        // Right-handed, which is the convention the whole project rests on:
        // Tangent x Lateral = Up. Flip it and the entire train is mirrored.
        assert(Length(Cross(F.Tangent, F.Lateral) - F.Up) < 1e-9);
    }
    std::printf("  worst unit-length error %.2e, worst non-perpendicularity %.2e\n",
                WorstUnit, WorstPerp);
    assert(WorstUnit < 1e-12 && WorstPerp < 1e-12);

    // ---- THE LAST INTERVAL IS SHORT, because WalkTrack clamps its final sample
    // to the track length rather than overshooting. Assuming every interval is
    // Spacing puts the last car of a train standing at the station in the wrong
    // place, which is the one spot somebody is looking closely at.
    const FTrack Odd = Straight(10.3);        // 20 x 0.5 then a 0.3 remainder
    const std::vector<FTrackFrame> OddPath = WalkTrack(Odd, Spacing);
    const FTrackFrame End = FrameAtDistance(OddPath, Spacing, 10.3, 10.3);
    assert(Length(End.Position - OddPath.back().Position) < 1e-12);
    const FTrackFrame Mid = FrameAtDistance(OddPath, Spacing, 10.3, 10.15);
    assert(std::fabs(Mid.Position.X - 10.15) < 1e-9);
    std::printf("  and a track whose length is not a whole number of samples still lands\n");
}

// ===================== A CIRCUIT'S TRAIN COMES ROUND =====================
void TestATrainStraddlingTheSEAMWrapsRatherThanPilingUp()
{
    std::printf("On a circuit a train straddles the seam instead of piling up\n");

    const FTrainSettings S;
    const double Spacing = 0.5;
    const FTrack T = Straight(100.0);         // stands in for a closed layout
    const std::vector<FTrackFrame> Path = WalkTrack(T, Spacing);
    const double Total = T.TotalLength();

    // A nose just past the seam: two cars ahead of it, three still behind.
    const std::vector<FCarPlacement> Wrapped =
        PlaceCars(Path, Spacing, Total, 4.0, S, true);
    assert(Wrapped.size() == 5);
    for (const FCarPlacement& C : Wrapped)
    {
        assert(C.S >= 0.0 && C.S <= Total);
    }
    // The ones behind the seam came round to the far end rather than clamping to
    // zero — clamping is what piles every trailing car on top of the first.
    assert(Wrapped[0].S < 10.0 && Wrapped[4].S > Total - 20.0);
    std::printf("  cars at %.1f, %.1f, %.1f, %.1f, %.1f m on a %.0f m circuit\n",
                Wrapped[0].S, Wrapped[1].S, Wrapped[2].S, Wrapped[3].S, Wrapped[4].S, Total);

    // ---- WITHOUT WRAP THEY CLAMP, which is right for an open layout: a train
    // shunted off the end of a line should not reappear at the station.
    const std::vector<FCarPlacement> Open =
        PlaceCars(Path, Spacing, Total, 4.0, S, false);
    assert(Open[4].S == 0.0);
    // ...and the placement agrees with its own frame, rather than reporting a
    // car at -9.5 m while carrying the frame from 0 m.
    assert(Length(Open[4].Frame.Position - Path.front().Position) < 1e-12);
    std::printf("  and on an open layout they stop at the end instead\n");
}

// ===================== NOTHING ASKED FOR IS NOTHING BUILT =====================
void TestADegenerateTrainBuildsNOTHINGRatherThanSomethingBroken()
{
    std::printf("A train that cannot exist builds nothing at all\n");

    const FTrackProfile Profile;
    const double Spacing = 0.5;
    const FTrack T = Straight(60.0);
    const std::vector<FTrackFrame> Path = WalkTrack(T, Spacing);

    FTrainSettings Zero;
    Zero.CarCount = 0;
    assert(PlaceCars(Path, Spacing, T.TotalLength(), 30.0, Zero, false).empty());

    FTrainSettings NoLength;
    NoLength.CarLengthM = 0.0;
    assert(BuildCarMesh(NoLength, 1.1, Profile).NumTriangles() == 0);
    assert(PlaceCars(Path, Spacing, T.TotalLength(), 30.0, NoLength, false).empty());

    // An empty path is what a blank track gives, and a blank track is the very
    // first thing a new author has — it took the editor down twice before.
    assert(PlaceCars({}, Spacing, 0.0, 0.0, FTrainSettings(), false).empty());
    std::printf("  zero cars, zero length and an empty track are all silent\n");

    // ---- A TRAIN LONGER THAN ITS TRACK IS REPORTED. Not repaired: the fix is
    // fewer cars or more track, and one silently truncated reads as a vehicle
    // that fits.
    FTrainSettings Long;
    Long.CarCount = 40;                        // 120 m
    const std::vector<FMeshFinding> F = AuditTrain(Long, 1.1, Profile, 60.0);
    assert(F.size() == 1);
    std::printf("  %s\n", F[0].What.c_str());
}

// ===================== THE LAP BARS SWING WITH THE BANK =====================
//
// Three things a screenshot would accept and a bank would not. The bars must
// be CLOSED GEOMETRY at every angle (a strut is capped, so an open rim is a
// pipe); the topology must not change with the angle, or the actor's in-place
// vertex update silently recreates a section every frame the bars move; and a
// row's bar must move ONLY when its own group does, because a stuck group is
// exactly the fault a walk-round exists to find, and it is only findable if
// the other bars come down around it.
// ===================== THE EYE IS IN THE SEAT THE MESH DRAWS =====================
//
// Seat.h places a rider along the train and TrainMesh.h draws a squab: two
// formulas for where a row is, and the camera sits on the first while the
// picture shows the second. Asserted equal, row by row, and the eye it
// produces is over the squab, behind the row centre and above the rim.
void TestTheRidersEyeSitsInTheSeatTheMeshDraws()
{
    std::printf("The rider's eye sits in the seat the mesh draws\n");
    const FTrackProfile P;
    const FTrainSettings S;
    const double H = 1.1;
    const double Half = S.TrainLengthM() * 0.5;
    for (int Car = 0; Car < S.CarCount; ++Car)
    {
        for (int Row = 0; Row < S.RowsPerCar; ++Row)
        {
            FSeat Seat; Seat.Car = Car; Seat.Row = Row;
            const double FromSeat = SeatOffsetAlongM(Seat, S.CarCount, S.CarLengthM, S.RowsPerCar, S.BodyGapM);
            const double FromMesh = Half - S.CarLengthM * (Car + 0.5) + RowCentreX(S, Row);
            assert(std::fabs(FromSeat - FromMesh) < 1e-9 && "Seat.h and the mesh disagree about where a row is");
        }
    }
    // The eye: over the rear of the squab (which runs from 0.65 of a seat depth
    // behind the row centre to 0.35 ahead of it), and high enough to see out.
    assert(S.RiderEyeBehindRowM > 0.0 && S.RiderEyeBehindRowM < S.SeatDepthM * 0.65);
    const double EyeAboveRails = RiderEyeAboveHeartline(S, P, H) + H;
    const double SquabTop = CabinFloorHeight(S, ShellKeepOut(S, P)) + S.SeatHeightM;
    assert(EyeAboveRails > SquabTop + 0.5 && "a seated adult's eye, not a child's");
    assert(EyeAboveRails > S.BodyHeightM && "the eye sees over the rim");
    std::printf("  %d rows agree with the mesh; the eye is %.2f m over the squab and %.2f m over the rim\n",
                S.CarCount * S.RowsPerCar, EyeAboveRails - SquabTop, EyeAboveRails - S.BodyHeightM);
}

void TestTheLapBarsSwingWithTheBankAndStayClosed()
{
    std::printf("The lap bars swing with the bank, and are closed geometry at every angle\n");

    const FTrackProfile P;
    FTrainSettings S;
    S.RowsPerCar = 2;
    const double Spacing = 0.5;
    const double Heartline = 1.1;
    const FTrack T = Straight(60.0);
    const std::vector<FTrackFrame> Path = WalkTrack(T, Spacing);
    const std::vector<FCarPlacement> Cars =
        PlaceCars(Path, Spacing, T.TotalLength(), 40.0, S, false);
    const FTrainMesh Car = BuildCarMesh(S, Heartline, P);
    const int Rows = S.CarCount * S.RowsPerCar;

    auto MaxZ = [](const FMeshBuffer& M)
    {
        double Z = -1e9;
        for (const FVec3& V : M.Position) { Z = std::max(Z, V.Z); }
        return Z;
    };

    const FTrainMesh Closed =
        BuildTrainMesh(Cars, Car, S, Heartline, std::vector<double>(Rows, 1.0));
    const FTrainMesh Open =
        BuildTrainMesh(Cars, Car, S, Heartline, std::vector<double>(Rows, 0.0));
    const FTrainMesh Default = BuildTrainMesh(Cars, Car, S, Heartline);

    for (const FTrainMesh* M : {&Closed, &Open})
    {
        CheckIndicesInRange(M->Restraints);
        CheckWindingAgreesWithNormals(M->Restraints, "restraints");
        assert(BoundaryEdges(M->Restraints) == 0);
        assert(SignedVolume(M->Restraints) > 0.0);
    }
    assert(Closed.Restraints.NumTriangles() > 0);

    // ---- SAME TOPOLOGY AT EVERY ANGLE: the angle moves vertices and nothing
    // else, which is what the actor's UpdateMeshSection path depends on.
    assert(Closed.Restraints.NumVertices() == Open.Restraints.NumVertices());
    assert(Closed.Restraints.Index == Open.Restraints.Index);

    // ---- NO POSITIONS MEANS CLOSED. A train out on the course is carrying
    // riders; a bar defaulting open there is a fault the ride does not have.
    assert(Default.Restraints.NumVertices() == Closed.Restraints.NumVertices());
    for (std::size_t v = 0; v < Default.Restraints.Position.size(); ++v)
    {
        assert(Length(Default.Restraints.Position[v] - Closed.Restraints.Position[v]) < 1e-12);
    }

    // ---- CLOSED STANDS AT THE RIM, OPEN FALLS AWAY FROM IT. A floor-hinged bar
    // is nearly upright when it is holding somebody and leans FORWARD when it
    // lets go, so what reads from the platform is the angle rather than the
    // height — and open is the LOWER of the two, which the swing-arm version had
    // the other way round.
    const double RimZ = Cars[0].Frame.Position.Z - Heartline + S.BodyHeightM;
    const double SeatTopZ = Cars[0].Frame.Position.Z - Heartline
        + CabinFloorHeight(S, ShellKeepOut(S, P)) + S.SeatHeightM;
    assert(MaxZ(Closed.Restraints) < RimZ && "closed, the bar stays inside the shell");
    assert(MaxZ(Closed.Restraints) > SeatTopZ && "closed, the bar is over the lap, not the floor");
    assert(MaxZ(Open.Restraints) < MaxZ(Closed.Restraints) && "open, it falls forward and down");
    std::printf("  %d bars: closed tops out %.2f m under the rim, open drops %.2f m below that\n",
                Rows, RimZ - MaxZ(Closed.Restraints),
                MaxZ(Closed.Restraints) - MaxZ(Open.Restraints));

    // ---- AND IT GOES ALL THE WAY DOWN TO THE FLOOR, where the lock is. That is
    // the difference between this and a swing arm, and it is the only thing that
    // makes the tube a locking member rather than decoration.
    double LowZ = 1e9;
    for (const FVec3& V : Closed.Restraints.Position) { LowZ = std::min(LowZ, V.Z); }
    const double FloorZ = Cars[0].Frame.Position.Z - Heartline + CabinFloorHeight(S, ShellKeepOut(S, P));
    assert(std::fabs(LowZ - FloorZ) < S.BarDiameterM && "the bar reaches the cabin floor");
    std::printf("  and the tube reaches the floor, %.3f m off it, where the lock is\n",
                std::fabs(LowZ - FloorZ));

    // ---- HINGED AT THE FRONT, COMING DOWN BACK OVER THE LAP. One bar in car
    // space: closed, it runs from ahead of the row centre back past it; open,
    // it stands entirely ahead of the row centre, so the seat is clear from
    // above. The first version had both the other way round.
    {
        const double RowX = RowCentreX(S, 0);
        auto Span = [&](double Pos, double& MinX, double& MaxX)
        {
            FMeshBuffer B;
            AddRestraintBar(B, S, P, -Heartline, RowX, Pos);
            MinX = 1e9; MaxX = -1e9;
            for (const FVec3& V : B.Position) { MinX = std::min(MinX, V.X); MaxX = std::max(MaxX, V.X); }
        };
        double CMin, CMax, OMin, OMax;
        Span(1.0, CMin, CMax);
        Span(0.0, OMin, OMax);
        // THE PIVOT IS AT THE FRONT EDGE OF THE SQUAB, on the floor: the tube
        // goes down beside the seat rather than reaching in from the bay ahead.
        const double SquabBack = RowX - S.SeatDepthM * 0.65;
        const double SquabFront = RowX + S.SeatDepthM * 0.35;
        assert(std::fabs(CMax - (SquabFront + S.BarDiameterM * 0.5)) < 0.02
            && "the pivot is at the squab's front edge");
        assert(OMin > SquabBack && "open, the bar has left the lap");
        assert(OMax > CMax && "open, it leans forward past where it stood");

        // CLOSED, IT LEANS BACK OVER THE SQUAB — which is the lap. The row CENTRE
        // is not: the squab sits back from it, so a bar landing on the centre
        // line would be over the knees.
        assert(CMin > SquabBack && CMin < SquabFront && "closed, the bar is over the squab");

        // AND THERE IS ROOM TO SIT. The gap between the bar and the face of the
        // backrest is where a person goes: an arm authored a hand's width too
        // long puts the bar in their chest, and nothing else says so.
        const double Room = CMin - SquabBack;
        assert(Room > 0.30 && "a rider has to fit between the backrest and the bar");
        std::printf("  pivot on the floor at the squab front, leaning %.0f deg back closed and %.0f deg forward open;"
                    " %.2f m of seat behind it\n",
                    S.BarClosedLeanDeg, S.BarOpenLeanDeg, Room);
    }

    // ---- ONE ROW RAISED MOVES ONE BAR. Bars are appended in row order, car by
    // car, so each bar is a contiguous run of the same vertex count.
    std::vector<double> One(Rows, 1.0);
    One[3] = 0.0;
    const FTrainMesh Mixed = BuildTrainMesh(Cars, Car, S, Heartline, One);
    const std::size_t PerBar = Mixed.Restraints.NumVertices() / static_cast<std::size_t>(Rows);
    assert(PerBar * static_cast<std::size_t>(Rows) == Mixed.Restraints.NumVertices());
    for (int r = 0; r < Rows; ++r)
    {
        double Moved = 0.0;
        for (std::size_t v = r * PerBar; v < (r + 1) * PerBar; ++v)
        {
            Moved = std::max(Moved,
                Length(Mixed.Restraints.Position[v] - Closed.Restraints.Position[v]));
        }
        if (r == 3) { assert(Moved > 0.3); }
        else        { assert(Moved < 1e-12); }
    }
    std::printf("  raising row 3 alone moves row 3 alone\n");

    // ---- HALF WAY IS BETWEEN, so a bank in transit reads as one. Measured on
    // the LEAN — how far forward the top has swung — and NOT on a height: a bar
    // that tips 15 deg back and 45 deg forward passes through vertical, so its
    // height is the same at 15 forward as at 15 back and says nothing about
    // which way it is going. The first version of this check asserted a height
    // and was true only by accident of the old swing-arm arrangement.
    const FTrainMesh Half =
        BuildTrainMesh(Cars, Car, S, Heartline, std::vector<double>(Rows, 0.5));
    auto MaxX = [](const FMeshBuffer& M)
    {
        double X = -1e9;
        for (const FVec3& V : M.Position) { X = std::max(X, V.X); }
        return X;
    };
    assert(MaxX(Half.Restraints) > MaxX(Closed.Restraints));
    assert(MaxX(Half.Restraints) < MaxX(Open.Restraints));

    // ---- ROWS RUN FRONT TO BACK INSIDE THE SHELL, the order the station lays
    // its gates in, so bar g and gate g face each other.
    const double ShellHalf = (S.CarLengthM - S.BodyGapM) * 0.5;
    assert(RowCentreX(S, 0) > RowCentreX(S, 1));
    assert(std::fabs(RowCentreX(S, 0)) < ShellHalf);
    assert(std::fabs(RowCentreX(S, 1)) < ShellHalf);
}

} // namespace

int main()
{
    // UNBUFFERED, so a failing assertion still shows the numbers printed
    // just before it. abort() does not flush, and the one run that matters
    // is the one that FAILS - a suite that goes silent exactly when it has
    // something to say costs more time than this line costs to write.
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    std::printf("\n=== TrainMesh: the train, which was nine engine cubes ===\n\n");

    TestTheMeshIsBuiltFromCARSNotFromSAMPLEPOINTS();
    TestEveryPartIsCLOSEDAndENCLOSESVolume();
    TestTheThreeWheelSetsGRIPTheRail();
    TestTheBodyLeavesTheHEARTLINEInTheOpen();
    TestTheShellCLEARSEveryWheelItPasses();
    TestTheTaperedFloorClearsBOTHRunningRails();
    TestCarsCHORDAcrossACurveRatherThanBendingWithIt();
    TestCouplersJOINAdjacentCarsOnAStraightAndACurve();
    TestFrameAtDistanceLANDSOnItsOwnSamplesAndStaysOrthonormal();
    TestATrainStraddlingTheSEAMWrapsRatherThanPilingUp();
    TestADegenerateTrainBuildsNOTHINGRatherThanSomethingBroken();
    TestTheLapBarsSwingWithTheBankAndStayClosed();
    TestTheRidersEyeSitsInTheSeatTheMeshDraws();

    std::printf("\ntest_trainmesh: all assertions passed.\n");
    return 0;
}
