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
    const FPart Parts[] = {{&C.Body, "body"}, {&C.Chassis, "chassis"}, {&C.Wheels, "wheels"}};

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
    assert(Top < 0.0 && "the roof must stay below the heartline the camera sits at");

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
    TestTheTaperedFloorClearsBOTHRunningRails();
    TestCarsCHORDAcrossACurveRatherThanBendingWithIt();
    TestCouplersJOINAdjacentCarsOnAStraightAndACurve();
    TestFrameAtDistanceLANDSOnItsOwnSamplesAndStaysOrthonormal();
    TestATrainStraddlingTheSEAMWrapsRatherThanPilingUp();
    TestADegenerateTrainBuildsNOTHINGRatherThanSomethingBroken();

    std::printf("\ntest_trainmesh: all assertions passed.\n");
    return 0;
}
