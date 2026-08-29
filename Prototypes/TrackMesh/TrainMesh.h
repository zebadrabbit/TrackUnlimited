// TrackUnlimited Phase 4: the train, as geometry you can see.
// Plain C++17, no engine dependency, same conventions as TrackMesh.h.
//
// SPEC: `Docs/TRAIN_DESIGN.md`. Read it before changing a dimension here; every
// number below already existed somewhere else, and this file's job is to say
// which list drives the mesh rather than to invent measurements.
//
// ===================== A SAMPLE POINT IS NOT A CAR =====================
//
// What shipped before this was one engine cube per PHYSICS SAMPLE POINT — nine
// of them, spaced TrainLength/8. Nine is a physics resolution: it is how finely
// the gravity integration reads the train's mean height, and it is right for
// that. It is not the number of cars, and cars are AUTHORED. `CarCount x
// CarLengthM` is what somebody typed; nine is what the integrator needed.
//
// The evidence was already sitting in the old comment: dividing the length into
// nine made the 15 m train look accidentally plausible and the 6 m small-batch
// vehicle look like nine playing cards standing on edge. The boxes were then
// made to touch, which hid it, and the wrong source stayed.
//
// So the mesh is built from CarCount and CarLengthM, and the sample points keep
// doing physics and stop doing rendering. The two lists are allowed to differ
// and always will — a nine-point train of five cars is the normal case, and
// neither number is derivable from the other.
//
// ===================== TWO HALVES, AND THE SPLIT IS THE SAME ONE =====================
//
// `BuildCarMesh` builds ONE car in the car's own local space and knows nothing
// about a track. `PlaceCars` decides where each car sits and produces no
// geometry at all.
//
// That is the WalkTrack/BuildTrackMesh split again, and for a sharper reason
// here: a train MOVES. Rebuilding a body shell every frame to slide it three
// metres is the same class of mistake as calling EvaluateAt per ring. Build the
// car once; move a transform per frame. Neither half can make that mistake,
// because neither holds the data it would need to make it with.
//
// ===================== ARTICULATION COMES FREE =====================
//
// Each car takes its frame from ITS OWN ARC LENGTH. Through a curve the rigid
// bodies then visibly chord across the arc while the wheels stay on it, which is
// the motion nobody manages to fake and the reason a coaster train looks alive
// from outside. It is not simulated and it costs nothing: it falls out of NOT
// interpolating one transform for the whole train.
//
// ===================== LOCAL SPACE =====================
//
// +X forward, +Y the rider's LEFT, +Z the rider's up, origin at the car's
// HEARTLINE. That is the track frame's own basis (Tangent, Lateral, Up), which
// is right-handed with Tangent x Lateral = Up, so placing a car is three
// multiply-adds and no conversion.
//
// The origin is the heartline rather than the floor because the heartline is the
// one point on a train this project already treats as authoritative — it is
// where the rider is, where the ride camera sits and where felt G is computed,
// and hanging the mesh off anything else would create a second answer to a
// question that already has one.
//
// Units: metres. Convert and flip handedness at the port boundary, never here.
// THE PORT RULE: M(x,y,z) = (x,-y,z) is a reflection and you do NOT swap indices
// — corrected 2026-08-09 after months of every surface being inside out. A thin
// tube inside out has the same silhouette; A CAR BODY IS SOLID and does not,
// which is precisely how that bug was finally caught. This file is the first
// thing since to draw something solid, so it is the first thing that would show
// it again.

#pragma once

#include "TrackMesh.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

// ===================== THE TRAIN, AS FOUR MATERIALS =====================
//
// Four buffers because they are four materials, which is the rule the track mesh
// already follows for rails, spine and ties: fibreglass shell, painted steel
// chassis, polyurethane wheels, and the couplers between cars. Keeping them
// apart costs nothing now and saves a submesh split later — and it is what lets
// the later art pass replace the shell without touching anything that carries
// load.
struct FTrainMesh
{
    FMeshBuffer Body;
    FMeshBuffer Chassis;
    FMeshBuffer Wheels;
    FMeshBuffer Couplers;
    // A FIFTH, since 2026-08-27: the lap bars. Padded steel, and the one part of
    // the train that moves relative to the car — so it is built per PLACEMENT
    // from the bank's position rather than once per car. See AddRestraintBar.
    FMeshBuffer Restraints;
    // And a sixth: the seats. Built with the car and stamped like the shell, in
    // their own buffer because upholstery is not gelcoat — and because a bar
    // over an empty red tub reads as nothing, where a bar over a seat reads as
    // a lap bar.
    FMeshBuffer Seats;

    std::size_t NumVertices() const
    {
        return Body.NumVertices() + Chassis.NumVertices()
             + Wheels.NumVertices() + Couplers.NumVertices() + Restraints.NumVertices()
             + Seats.NumVertices();
    }
    std::size_t NumTriangles() const
    {
        return Body.NumTriangles() + Chassis.NumTriangles()
             + Wheels.NumTriangles() + Couplers.NumTriangles() + Restraints.NumTriangles()
             + Seats.NumTriangles();
    }
};

struct FTrainSettings
{
    // ---- What somebody authored. NOBODY SPECIFIES A TRAIN IN METRES.
    int CarCount = 5;

    // COUPLED PITCH, not body length: centre of one car to centre of the next,
    // which is what the physics divides up and what a platform has to be long
    // enough for. The shell is shorter than this by BodyGapM.
    double CarLengthM = 3.0;

    // ---- The shell. THE ONLY PART WITH TASTE IN IT; see CarBodySection.
    double BodyWidthM = 1.40;         // at its widest, which is the top
    double BodyHeightM = 0.90;        // LOAD-BEARING — see BodySwallowsHeartline
    double BodyFloorWidthM = 0.52;    // at the floor, and it must clear the WHEELS
    double BodyTaperFraction = 0.45;  // how far up the flank reaches full width
    double BodyCornerRadiusM = 0.15;  // the rolled rim, and the WALL THICKNESS below it
    double BodyGapM = 0.30;           // pitch minus shell, so cars do not weld

    // ---- The cabin. THE SHELL IS A TUB, NOT A BOX (2026-08-27). It had a roof,
    // and the first lap bars closed into a closed box and vanished, then poked
    // through the roof when raised — reported from a screenshot within the
    // hour. A car body is open on top because riders sit in it; the cabin floor
    // sits just above the shoulder, where the shell first reaches full width,
    // so the cavity cannot poke out through the tapered flank.
    double CabinFloorClearM = 0.06;   // cabin floor above the shoulder knee
    double SeatWidthM = 0.45;
    double SeatDepthM = 0.45;
    double SeatHeightM = 0.30;        // squab, above the cabin floor
    double SeatBackHeightM = 0.60;    // backrest, above the squab
    double SeatBackDepthM = 0.08;
    double SeatPitchM = 0.60;         // between the two seats of a row

    // THE CABIN IS CLOSED AT BOTH ENDS (2026-08-28). The tub's end caps close
    // the shell's own thickness and nothing else, so the first tub was open
    // front and back and a rider in the front row had nothing between their
    // knees and the car ahead — reported from a screenshot. A bulkhead at each
    // end, cabin floor to rim, wall to wall; set 1 cm in from the end so its
    // face is not coplanar with the tub's cap, and let INTO the walls and floor
    // by 2 cm rather than butted against them, because two closed bodies
    // sharing a face weld into one with a doubled edge (the seats' lesson).
    double EndPanelThickM = 0.05;

    // ---- The rider's eye, FROM THE SEAT (2026-08-28). The camera used to sit
    // an authored 0.25 m over the heartline at the car's centre, which with two
    // rows is between them and with seats is a child's eye over a squab that
    // is now drawn. The seat is the one answer for where a rider is, so the eye
    // is derived from it: over the rear of the squab, a seated adult's eye
    // above it. Cosmetic — it moves the view and never a G figure.
    double RiderEyeAboveSeatM = 0.75;
    double RiderEyeBehindRowM = 0.20;  // behind the row centre: over the hips, not the knees

    // ---- The chassis: what the body and the bogies both attach to, so
    // articulation has somewhere to happen.
    double ChassisWidthM = 0.50;
    double ChassisDepthM = 0.18;
    double ChassisDropM = 0.05;       // below the rail plane, so it reads as under

    // ---- The wheels. THREE SETS, and this is what makes it read as a coaster.
    double RunningWheelDiameterM = 0.30;
    double SideWheelDiameterM = 0.20;
    double UpstopWheelDiameterM = 0.20;
    double WheelWidthM = 0.10;

    // HARDWARE NEEDS A GAP, and this is the one number here that is a fitter's
    // judgement rather than a derivation. The shell is held this far clear of
    // every wheel; at zero they would touch exactly, which is a rendering
    // coin-toss rather than a design.
    double WheelClearanceM = 0.03;

    // How far in from each end of the car the wheel assembly sits.
    double BogieInsetM = 0.50;

    // How far the chassis reaches PAST the bogies. It has to leave the beam
    // shorter than half the pitch, or adjacent chassis ends touch, the coupler
    // between them has zero length, and the piece that makes articulation
    // legible silently does not exist. Clamped below for exactly that reason.
    double ChassisOverhangM = 0.20;

    double CouplerDiameterM = 0.07;

    // ---- The restraint: ONE LAP BAR PER SEAT ROW, hinged at the hip either side
    // of the seat and swung by the bank's position (FCommandedBank::GroupPosition,
    // 0 open, 1 closed). Closed it lies forward across the lap, INSIDE the shell
    // — which is where a real one is, and why a closed bar is invisible from the
    // platform. Raised it stands above the rim, which is the state that means
    // something: a bar you can see is a train that is not ready.
    //
    // RowsPerCar is the train's number and the station reads it — the gate per
    // seat row on the platform faces the bar it serves, so the two cannot be
    // allowed to disagree about how many rows a car has. TWO, since 2026-08-28:
    // a 2.7 m shell has room for two rows and one left the car half empty on
    // screen. Rows are spaced evenly along the shell (RowCentreX), so the
    // pitch is the shell's and not a second authored number.
    int RowsPerCar = 2;
    double BarDiameterM = 0.06;
    double BarHingeUpM = 0.05;        // above the squab: the hip, seated. DERIVED from the seat
    double BarHingeBackM = 0.15;      // behind the row centre, so it swings over the lap
    double BarArmLengthM = 0.45;
    double BarInsetM = 0.12;          // arms this far in from the shell's full width
    double BarRaisedDeg = 80.0;       // fully open: swung up out of the seat

    // ---- Tessellation. Coarser than the rails DELIBERATELY, the same argument
    // the support columns get eight sides on: a wheel is 300 mm across and there
    // are twelve per car, where a rail is one continuous run under the camera in
    // a ride view.
    int WheelSides = 10;
    int StrutSides = 6;

    double TextureMetres = 1.0;

    // Derived, never authored — the same relationship the actor already keeps.
    double TrainLengthM() const { return CarLengthM * static_cast<double>(CarCount); }
};

// ===================== TWO DERIVED LENGTHS, ONE ANSWER EACH =====================
//
// BuildCarMesh needs these to place the bogies and the beam; BuildTrainMesh
// needs the same numbers to hang the couplers off the beam ends. They were
// worked out separately in both, which is two sources of truth for one dimension
// and the mistake `GraphRect` and `ConsolePlatformPtr()` each exist to avoid. A
// coupler attached to where the chassis is NOT would be invisible until somebody
// looked under the train.
inline double BogieOffset(const FTrainSettings& S)
{
    return std::max(0.10, S.CarLengthM * 0.5 - S.BogieInsetM);
}

// Where row j of a car sits along the shell, front row first — the same order
// the station lays its gates in, so gate g and bar g face each other.
inline double RowCentreX(const FTrainSettings& S, int Row)
{
    const int Rows = std::max(1, S.RowsPerCar);
    const double ShellHalf = std::max(0.05, (S.CarLengthM - S.BodyGapM) * 0.5);
    const double Pitch = 2.0 * ShellHalf / Rows;
    return ShellHalf - (static_cast<double>(Row) + 0.5) * Pitch;
}

// SHORTER THAN HALF THE PITCH, ALWAYS. Two chassis that meet leave nowhere for
// the coupler between them, and SweepStrut builds NOTHING from a zero-length
// strut — so the failure is not an ugly coupler, it is no coupler and no
// complaint, which is the kind that ships.
inline double ChassisHalfLength(const FTrainSettings& S)
{
    const double Want = BogieOffset(S) + S.ChassisOverhangM;
    const double Cap = S.CarLengthM * 0.5 - S.CouplerDiameterM;
    return std::max(0.05, std::min(Want, Cap));
}

// ===================== 0.9 m IS NOT A GUESS, AND MUST NOT BE RAISED =====================
//
// The heartline sits HeartlineHeight (1.1 m) above the rail centreline, and the
// body's floor sits ON the rail plane. A shell taller than the heartline
// therefore closes over the point the ride camera sits at, and the rider spends
// the lap inside a box. A real car body comes up to about the chest of a seated
// rider for exactly the same reason — you have to be able to see out of it.
//
// A PREDICATE RATHER THAN A CLAMP, because REPORTED NEVER REPAIRED is this
// project's rule everywhere: the repair for a body that swallows its own camera
// is a shorter body, and one silently clamped would read as a dimension that
// worked.
inline bool BodySwallowsHeartline(const FTrainSettings& S, double HeartlineHeight)
{
    return S.BodyHeightM > HeartlineHeight;
}

// ===================== A CAP THAT SURVIVES A REFLEX CORNER =====================
//
// A fan from the centroid caps a convex section and nothing else: on a tub the
// centroid lies in the cavity, and every fan triangle across the inner wall
// comes out reversed — watertight, and inside out where it shows most. Ear
// clipping triangulates any simple polygon, and the section is one by
// construction. Returned in the section's own winding.
inline std::vector<std::size_t> TriangulateSection(const std::vector<FVec2>& P)
{
    std::vector<std::size_t> Tri;
    const std::size_t N = P.size();
    if (N < 3) { return Tri; }
    std::vector<std::size_t> Idx(N);
    for (std::size_t i = 0; i < N; ++i) { Idx[i] = i; }

    double Area2 = 0.0;
    for (std::size_t i = 0; i < N; ++i)
    {
        const FVec2& A = P[i];
        const FVec2& B = P[(i + 1) % N];
        Area2 += A.U * B.V - B.U * A.V;
    }
    const bool bReversed = Area2 < 0.0;
    if (bReversed) { std::reverse(Idx.begin(), Idx.end()); }

    auto Cross2 = [&](std::size_t a, std::size_t b, std::size_t c)
    {
        return (P[b].U - P[a].U) * (P[c].V - P[a].V) - (P[b].V - P[a].V) * (P[c].U - P[a].U);
    };
    const double Eps = 1e-12;
    auto Emit = [&](std::size_t a, std::size_t b, std::size_t c)
    {
        // Back in the caller's winding, so the flip test downstream still applies.
        if (bReversed) { Tri.push_back(a); Tri.push_back(c); Tri.push_back(b); }
        else           { Tri.push_back(a); Tri.push_back(b); Tri.push_back(c); }
    };

    while (Idx.size() > 3)
    {
        bool bFound = false;
        const std::size_t M = Idx.size();
        for (std::size_t i = 0; i < M && !bFound; ++i)
        {
            const std::size_t a = Idx[(i + M - 1) % M], b = Idx[i], c = Idx[(i + 1) % M];
            if (Cross2(a, b, c) <= Eps) { continue; }   // reflex or flat: not an ear
            bool bEar = true;
            for (std::size_t k = 0; k < M && bEar; ++k)
            {
                const std::size_t q = Idx[k];
                if (q == a || q == b || q == c) { continue; }
                if (Cross2(a, b, q) >= -Eps && Cross2(b, c, q) >= -Eps && Cross2(c, a, q) >= -Eps)
                {
                    bEar = false;
                }
            }
            if (!bEar) { continue; }
            Emit(a, b, c);
            Idx.erase(Idx.begin() + static_cast<std::ptrdiff_t>(i));
            bFound = true;
        }
        if (!bFound) { break; }   // degenerate remainder: fan what is left, below
    }
    for (std::size_t i = 1; i + 1 < Idx.size(); ++i) { Emit(Idx[0], Idx[i], Idx[i + 1]); }
    return Tri;
}

// ===================== A SECTION SWEEP, WHICH SweepTube IS NOT =====================
//
// SweepTube sweeps a CIRCLE, which is right for a rail, a handrail and a wheel
// and cannot express a car body at all. This takes an arbitrary closed polygon
// in the ring's own basis.
//
// FLAT-SHADED PER EDGE, and that is the difference that matters rather than an
// optimisation detail. Sharing a vertex between two section edges averages their
// normals, which rounds every corner — correct for a tube, and it would turn a
// car body into a lozenge. So each edge gets its own four vertices and its own
// normal, and the corners stay corners.
//
// WOUND THE SAME WAY SweepTube WINDS. Section points run counter-clockwise in
// the (AxisA -> AxisB) sense, exactly as SweepTube's angle does; the face normal
// then comes out as Cross(edge, sweep), which is the outward one. Getting this
// backwards is the bug that made every tube on the ride inside out once already,
// and it is invisible on a thin tube.
inline void SweepSection(FMeshBuffer& Out, const std::vector<FTubeRing>& Rings,
                         const std::vector<FVec2>& Section, double TextureMetres,
                         bool bCapStart = true, bool bCapEnd = true)
{
    const std::size_t N = Section.size();
    if (Rings.size() < 2 || N < 3)
    {
        return;
    }

    // Perimeter distance for V rather than the point index: a polygon's points
    // are not evenly spaced, and indexing them would compress the texture into
    // whichever corner happened to carry the most vertices.
    std::vector<double> Cum(N + 1, 0.0);
    for (std::size_t j = 0; j < N; ++j)
    {
        const FVec2& A = Section[j];
        const FVec2& B = Section[(j + 1) % N];
        Cum[j + 1] = Cum[j]
            + std::sqrt((B.U - A.U) * (B.U - A.U) + (B.V - A.V) * (B.V - A.V));
    }
    const double Perimeter = Cum[N] > 0.0 ? Cum[N] : 1.0;
    const double UScale = TextureMetres > 0.0 ? 1.0 / TextureMetres : 1.0;

    auto At = [](const FTubeRing& R, const FVec2& P)
    {
        return R.Centre + R.AxisA * P.U + R.AxisB * P.V;
    };

    for (std::size_t i = 0; i + 1 < Rings.size(); ++i)
    {
        const FTubeRing& R0 = Rings[i];
        const FTubeRing& R1 = Rings[i + 1];
        const FVec3 Sweep = R1.Centre - R0.Centre;
        if (Length(Sweep) < 1e-12) { continue; }

        for (std::size_t j = 0; j < N; ++j)
        {
            const FVec2& S0 = Section[j];
            const FVec2& S1 = Section[(j + 1) % N];

            const FVec3 P00 = At(R0, S0);
            const FVec3 P01 = At(R0, S1);
            const FVec3 P10 = At(R1, S0);
            const FVec3 P11 = At(R1, S1);

            const FVec3 Nrm = Normalised(Cross(P01 - P00, Sweep));
            if (Length(Nrm) < 1e-12) { continue; }   // a zero-length section edge

            const std::uint32_t Base = static_cast<std::uint32_t>(Out.Position.size());
            const double V0 = Cum[j] / Perimeter;
            const double V1 = Cum[j + 1] / Perimeter;

            Out.Position.push_back(P00); Out.Normal.push_back(Nrm);
            Out.UV.push_back({R0.Along * UScale, V0});
            Out.Position.push_back(P01); Out.Normal.push_back(Nrm);
            Out.UV.push_back({R0.Along * UScale, V1});
            Out.Position.push_back(P10); Out.Normal.push_back(Nrm);
            Out.UV.push_back({R1.Along * UScale, V0});
            Out.Position.push_back(P11); Out.Normal.push_back(Nrm);
            Out.UV.push_back({R1.Along * UScale, V1});

            // Around then along, matching SweepTube exactly.
            Out.Index.push_back(Base);     Out.Index.push_back(Base + 1);
            Out.Index.push_back(Base + 3);
            Out.Index.push_back(Base);     Out.Index.push_back(Base + 3);
            Out.Index.push_back(Base + 2);
        }
    }

    // Caps by ear clipping, so a tub's bulkheads close the cavity as well as
    // the walls. A car body has two free ends by definition, so an uncapped one
    // is a length of open box — the same thing that made every tie a piece of
    // open pipe before the ties were capped.
    const std::vector<std::size_t> CapTris = TriangulateSection(Section);
    auto Cap = [&](const FTubeRing& R, const FVec3& Outward)
    {
        if (Length(Outward) < 1e-12) { return; }

        const std::uint32_t Base = static_cast<std::uint32_t>(Out.Position.size());
        for (const FVec2& P : Section)
        {
            Out.Position.push_back(At(R, P));
            Out.Normal.push_back(Outward);
            Out.UV.push_back({0.5 + P.U, 0.5 + P.V});
        }

        // The same test CapRing uses rather than an assumption about the basis:
        // a caller's AxisA/AxisB need not be right-handed with the sweep.
        const bool bFlip = Dot(Cross(R.AxisA, R.AxisB), Outward) < 0.0;
        for (std::size_t t = 0; t + 2 < CapTris.size(); t += 3)
        {
            const std::uint32_t A = Base + static_cast<std::uint32_t>(CapTris[t]);
            const std::uint32_t B = Base + static_cast<std::uint32_t>(CapTris[t + 1]);
            const std::uint32_t C = Base + static_cast<std::uint32_t>(CapTris[t + 2]);
            Out.Index.push_back(A);
            if (bFlip) { Out.Index.push_back(C); Out.Index.push_back(B); }
            else       { Out.Index.push_back(B); Out.Index.push_back(C); }
        }
    };

    // THE END AXIS COMES FROM THE ADJACENT RING, exactly as SweepTube's does and
    // for the same reason: on anything but a straight run the chord between the
    // two ends points nowhere useful.
    if (bCapStart)
    {
        Cap(Rings.front(), Normalised(Rings.front().Centre - Rings[1].Centre));
    }
    if (bCapEnd)
    {
        const std::size_t M = Rings.size();
        Cap(Rings[M - 1], Normalised(Rings[M - 1].Centre - Rings[M - 2].Centre));
    }
}

// ===================== THE ONLY PART WITH TASTE IN IT =====================
//
// One function returning a cross-section, so the later art pass replaces exactly
// this and nothing else. The chassis, the wheels and the articulation are
// engineering; only the shell is taste, and the split is what keeps pass two
// from touching pass one.
//
// If that pass happens in Blender, the useful output is a handful of points
// replacing this list — a generated section that stays parametric and stays
// diffable, rather than a .blend nobody can review.
//
// THE TAPER IS NOT STYLING. The floor line runs through the rail plane, and the
// running rails sit at +/- Gauge/2 on it. A body that went full width all the
// way down would be drawn straight through both rails, and the wheels — the
// entire reason this reads as a coaster rather than a bus — would be buried
// inside it. Real car bodies taper toward the floor for the same reason.
//
// ===================== WHERE THE WHEELS WILL NOT LET THE SHELL GO =====================
//
// DERIVED FROM THE WHEELS, NEVER AUTHORED. Two of the three sets sit in the same
// vertical band as the bottom of the shell, so each one says how wide the shell
// may be while it is passing them. Leaving these as taste numbers is what
// produced the defect this replaces: the shell was drawn through the running
// wheels by 0.168 m and through the side wheels by 0.167 m, and only the first
// was ever visible, because the second is buried inside the floor.
//
// Heights are measured ABOVE THE FLOOR rather than in car space, which is what
// keeps CarBodySection a pure shape function that knows nothing about a track.
struct FShellKeepOut
{
    double FloorHalfWidth = 0.0;  // between the side-friction wheels
    double SideTopM       = 0.0;  // ... which the shell is past at this height
    double MidHalfWidth   = 0.0;  // inboard of the running wheels
    double RunTopM        = 0.0;  // ... which the shell is past at this height
};

inline FShellKeepOut ShellKeepOut(const FTrainSettings& S, const FTrackProfile& Profile)
{
    const double HalfGauge = Profile.Gauge * 0.5;
    const double RailR = Profile.RailDiameter * 0.5;
    const double Gap = std::max(0.0, S.WheelClearanceM);

    FShellKeepOut K;

    // SIDE FRICTION RUNS AGAINST THE RAIL'S INNER FACE, so it is a puck lying flat
    // at rail height with its axis vertical — the same height as the floor, which
    // is exactly why it constrains the floor and why nothing else does.
    K.SideTopM = S.WheelWidthM * 0.5;
    K.FloorHalfWidth = HalfGauge - RailR - S.SideWheelDiameterM - Gap;

    // THE RUNNING WHEEL STANDS ON TOP OF THE RAIL, so it reaches a whole diameter
    // above the rail plane and the shell must stay inboard of its inner face for
    // all of that. This is the one the eye catches first.
    K.RunTopM = RailR + S.RunningWheelDiameterM;
    K.MidHalfWidth = HalfGauge - S.WheelWidthM * 0.5 - Gap;

    // The upstop hangs BELOW the rail, and the floor is above it, so it never
    // meets the shell and gets no entry here. Asserted rather than assumed.
    return K;
}

// Where the flank first reaches full width, above the floor. FULL WIDTH CANNOT
// ARRIVE BEFORE THE WHEELS ARE PAST: the authored taper is taste and is honoured
// wherever it is legal; below the running wheels it is not, so the taper gets
// pushed UP rather than the shell being pushed OUT.
inline double ShellShoulderHeight(const FTrainSettings& S, const FShellKeepOut& K)
{
    const double W = S.BodyWidthM * 0.5;
    const double H = S.BodyHeightM * 0.5;
    const double R = std::min(S.BodyCornerRadiusM, std::min(W, H) * 0.9);
    const double Shoulder = S.BodyHeightM - R;
    const double hAuthored = std::max(0.0, std::min(1.0, S.BodyTaperFraction)) * S.BodyHeightM;
    return std::max(std::min(hAuthored, Shoulder), std::min(K.RunTopM + 0.02, Shoulder));
}

// The cabin floor, above the shell's floor line: just over the shoulder, so the
// cavity sits inside the full-width part of the shell and never pokes out
// through the tapered flank over the wheels. The rider sits on the wheel wells,
// which is where a real one sits too.
inline double CabinFloorHeight(const FTrainSettings& S, const FShellKeepOut& K)
{
    return std::min(ShellShoulderHeight(S, K) + S.CabinFloorClearM, S.BodyHeightM * 0.9);
}

// The rider's eye above the HEARTLINE — what FSeat::VerticalM wants — from the
// seat the mesh draws: cabin floor, squab, then a seated adult's eye.
inline double RiderEyeAboveHeartline(const FTrainSettings& S, const FTrackProfile& Profile,
                                     double HeartlineHeight)
{
    return CabinFloorHeight(S, ShellKeepOut(S, Profile)) + S.SeatHeightM + S.RiderEyeAboveSeatM
         - HeartlineHeight;
}

// Returned counter-clockwise in (AxisA = rider's left, AxisB = up), relative to
// the BODY CENTRE, which is the winding SweepSection wants.
//
// A TUB: up the outer flank, over the rolled rim, DOWN the inner wall, across
// the cabin floor, and back the same way on the other side. Non-convex, which
// is why the caps are ear-clipped.
inline std::vector<FVec2> CarBodySection(const FTrainSettings& S,
                                        const FShellKeepOut& K)
{
    std::vector<FVec2> Out;
    const double W = S.BodyWidthM * 0.5;
    const double H = S.BodyHeightM * 0.5;
    const double R = std::min(S.BodyCornerRadiusM, std::min(W, H) * 0.9);

    // CLAMPED, NOT REFUSED, and the audit reports what was asked for. A shell
    // merely narrower than requested is still a car; one drawn through its own
    // wheels is a defect, so the clamp fails in the direction that stays a car.
    const double Wf = std::min(std::min(S.BodyFloorWidthM * 0.5, K.FloorHalfWidth), W);
    const double Wm = std::min(std::max(Wf, K.MidHalfWidth), W);

    const double hFull = ShellShoulderHeight(S, K);
    const double Wi = W - R;                            // the inner wall
    const double Vf = -H + CabinFloorHeight(S, K);      // the cabin floor

    const double Quarter = TrackMeshTwoPi * 0.25;
    const int ArcSteps = 3;

    // Up the LEFT flank, and it has TWO KNEES rather than one straight flare: a
    // narrow pan between the side wheels, out over the running wheels, then full
    // width at the shoulder. That is the shape the wheel envelope forces, and it
    // is also the shape a real car has, which is not a coincidence.
    Out.push_back({Wf, -H});
    Out.push_back({Wf, -H + K.SideTopM});
    Out.push_back({Wm, -H + K.RunTopM});
    Out.push_back({W, -H + hFull});
    Out.push_back({W, H - R});

    // Top-left corner. Three points reads as rounded without spending vertices
    // on a shape a later pass is going to replace anyway.
    for (int k = 1; k <= ArcSteps; ++k)
    {
        const double A = Quarter * (static_cast<double>(k) / (ArcSteps + 1.0));
        Out.push_back({W - R + R * std::cos(A), H - R + R * std::sin(A)});
    }

    // NO ROOF. Down the inner wall, across the cabin floor, up the other inner
    // wall, then the mirror of the flank back down the right-hand side.
    Out.push_back({Wi, H});
    Out.push_back({Wi, Vf});
    Out.push_back({-Wi, Vf});
    Out.push_back({-Wi, H});
    for (int k = ArcSteps; k >= 1; --k)
    {
        const double A = Quarter * (static_cast<double>(k) / (ArcSteps + 1.0));
        Out.push_back({-(W - R) - R * std::cos(A), H - R + R * std::sin(A)});
    }
    Out.push_back({-W, H - R});
    Out.push_back({-W, -H + hFull});
    Out.push_back({-Wm, -H + K.RunTopM});
    Out.push_back({-Wf, -H + K.SideTopM});
    Out.push_back({-Wf, -H});
    return Out;
}

// One wheel: a short fat cylinder about `Axis`, which SweepStrut already draws
// and already caps at both ends.
inline void AddWheel(FMeshBuffer& Out, const FVec3& Centre, const FVec3& Axis,
                     double Diameter, double Width, int Sides)
{
    const FVec3 A = Normalised(Axis);
    if (Length(A) < 1e-12 || !(Diameter > 0.0) || !(Width > 0.0)) { return; }
    // A hint that cannot be parallel to the axis, whichever axis it is.
    const FVec3 Hint = std::fabs(A.Z) > 0.9 ? FVec3{1.0, 0.0, 0.0} : FVec3{0.0, 0.0, 1.0};
    SweepStrut(Out, Centre - A * (Width * 0.5), Centre + A * (Width * 0.5),
               Hint, Diameter * 0.5, Sides);
}

// ===================== ONE CAR, IN ITS OWN SPACE =====================
//
// Knows nothing about a track, which is what makes it buildable once and movable
// for free. `HeartlineHeight` and the profile are the only things it borrows,
// and both are already the authority for where a rail is.
//
// THE WHEELS ARE THE POINT, and they are why a coaster train is not a railway
// train. Three sets gripping one rail from three directions: running wheels on
// TOP carrying the weight, side friction wheels against the INNER face taking
// lateral load, and upstop wheels UNDERNEATH — and the upstops are why a train
// survives -0.94 g on the showcase instead of leaving the track. From the chase
// camera that assembly is more of the silhouette than the fibreglass is, and it
// is exactly what a spline in space most obviously lacks.
inline FTrainMesh BuildCarMesh(const FTrainSettings& S, double HeartlineHeight,
                               const FTrackProfile& Profile)
{
    FTrainMesh M;
    if (!(S.CarLengthM > 0.0)) { return M; }

    // Where the rails are, in the car's own space. The rail plane sits a
    // heartline below the origin; the running rails a half-gauge either side.
    const double RailZ = -HeartlineHeight;
    const double HalfGauge = Profile.Gauge * 0.5;
    const double RailR = Profile.RailDiameter * 0.5;

    const FVec3 Across{0.0, 1.0, 0.0};     // the rider's left
    const FVec3 Vertical{0.0, 0.0, 1.0};

    // ---- The shell, shorter than the pitch so cars do not weld into one tube.
    const double ShellHalf = std::max(0.05, (S.CarLengthM - S.BodyGapM) * 0.5);

    // THE FLOOR SITS ON THE RAIL PLANE and the body rises from it. That is the
    // placement the cube draw already had right — `Position - Up * (Heartline -
    // BodyHeight/2)` — and the one thing about it that must survive the rewrite.
    const double BodyCentreZ = RailZ + S.BodyHeightM * 0.5;
    {
        std::vector<FTubeRing> Rings(2);
        Rings[0] = {FVec3{-ShellHalf, 0.0, BodyCentreZ}, Across, Vertical, 0.0};
        Rings[1] = {FVec3{ ShellHalf, 0.0, BodyCentreZ}, Across, Vertical, S.CarLengthM};
        SweepSection(M.Body, Rings, CarBodySection(S, ShellKeepOut(S, Profile)),
                     S.TextureMetres, true, true);
    }

    // A box swept along the car exactly as the shell is: the seats and the
    // end panels are all this.
    auto Box = [&](FMeshBuffer& Into, double X0, double X1, double Y, double Z0, double Z1, double HalfW)
    {
        if (!(X1 > X0) || !(Z1 > Z0) || !(HalfW > 0.0)) { return; }
        const double Zc = (Z0 + Z1) * 0.5, Hh = (Z1 - Z0) * 0.5;
        const std::vector<FVec2> Rect = {{HalfW, -Hh}, {HalfW, Hh}, {-HalfW, Hh}, {-HalfW, -Hh}};
        std::vector<FTubeRing> Rings(2);
        Rings[0] = {FVec3{X0, Y, Zc}, Across, Vertical, 0.0};
        Rings[1] = {FVec3{X1, Y, Zc}, Across, Vertical, X1 - X0};
        SweepSection(Into, Rings, Rect, S.TextureMetres, true, true);
    };

    // ---- The bulkheads: the cabin closed at both ends, floor to rim, let into
    // the walls and the floor. See EndPanelThickM.
    {
        const FShellKeepOut K = ShellKeepOut(S, Profile);
        const double FloorZ = RailZ + CabinFloorHeight(S, K);
        // A centimetre under the rim: the let-in strip runs under the rolled
        // corner, which is already dropping away there.
        const double RimZ = RailZ + S.BodyHeightM - 0.01;
        const double W = S.BodyWidthM * 0.5;
        const double R = std::min(S.BodyCornerRadiusM, std::min(W, S.BodyHeightM * 0.5) * 0.9);
        const double Let = std::min(0.02, R * 0.5);
        const double T = std::min(S.EndPanelThickM, ShellHalf * 0.5);
        for (int End = 0; End < 2; ++End)
        {
            const double X1 = ShellHalf - 0.01;
            const double X0 = X1 - T;
            Box(M.Body, End == 0 ? X0 : -X1, End == 0 ? X1 : -X0, 0.0,
                FloorZ - Let, RimZ, W - R + Let);
        }
    }

    // ---- The seats: two a row on the cabin floor, a squab and a backrest each,
    // so the lap bar has a lap to come down over.
    {
        const FShellKeepOut K = ShellKeepOut(S, Profile);
        const double FloorZ = RailZ + CabinFloorHeight(S, K);
        const int Rows = std::max(1, S.RowsPerCar);
        for (int r = 0; r < Rows; ++r)
        {
            const double RowX = RowCentreX(S, r);
            for (int Side = 0; Side < 2; ++Side)
            {
                const double Y = (Side == 0 ? 1.0 : -1.0) * S.SeatPitchM * 0.5;
                // The squab sits with its front third ahead of the row centre, and
                // the backrest rises behind it, above the rim if it must — a
                // headrest over the shell is what a real train has.
                Box(M.Seats, RowX - S.SeatDepthM * 0.65, RowX + S.SeatDepthM * 0.35, Y,
                    FloorZ, FloorZ + S.SeatHeightM, S.SeatWidthM * 0.5);
                // 5 mm behind the squab rather than touching it: two boxes sharing a
                // face are one mesh with a doubled edge, and the watertight check says so.
                Box(M.Seats, RowX - S.SeatDepthM * 0.65 - S.SeatBackDepthM - 0.005, RowX - S.SeatDepthM * 0.65 - 0.005, Y,
                    FloorZ, FloorZ + S.SeatHeightM + S.SeatBackHeightM, S.SeatWidthM * 0.5);
            }
        }
    }

    // ---- The chassis: a beam between the bogies, hung just under the rail plane
    // so it reads as structure rather than as part of the shell. Mostly hidden
    // under the floor, and clearly visible in the gap between cars — which is
    // where a real one is visible too.
    const double BogieX = BogieOffset(S);
    const double ChassisHalf = ChassisHalfLength(S);
    const double ChassisZ = RailZ - S.ChassisDropM;
    {
        const double Cw = S.ChassisWidthM * 0.5;
        const double Cd = S.ChassisDepthM * 0.5;
        // A rectangle, counter-clockwise in (left, up) exactly as the shell is.
        const std::vector<FVec2> Beam = {{Cw, -Cd}, {Cw, Cd}, {-Cw, Cd}, {-Cw, -Cd}};
        std::vector<FTubeRing> Rings(2);
        Rings[0] = {FVec3{-ChassisHalf, 0.0, ChassisZ}, Across, Vertical, 0.0};
        Rings[1] = {FVec3{ ChassisHalf, 0.0, ChassisZ}, Across, Vertical, S.CarLengthM};
        SweepSection(M.Chassis, Rings, Beam, S.TextureMetres, true, true);
    }

    // ---- The wheels: two assemblies per car, six wheels in each.
    for (int End = 0; End < 2; ++End)
    {
        const double X = End == 0 ? -BogieX : BogieX;
        for (int Side = 0; Side < 2; ++Side)
        {
            // +1 is the rider's LEFT, the convention the whole project uses and
            // the one thing here that silently mirrors a train if it is wrong.
            const double Sign = Side == 0 ? 1.0 : -1.0;
            const double RailY = HalfGauge * Sign;

            // Running: on top of the rail, carrying the weight. Polyurethane,
            // which is not cosmetic trivia — RollingResistance = 0.026 is
            // measured for polyurethane on steel, so the wheel that gets drawn
            // is the wheel the physics was fitted to.
            AddWheel(M.Wheels,
                     FVec3{X, RailY, RailZ + RailR + S.RunningWheelDiameterM * 0.5},
                     Across, S.RunningWheelDiameterM, S.WheelWidthM, S.WheelSides);

            // Upstop: underneath, and these are what make the ride possible.
            AddWheel(M.Wheels,
                     FVec3{X, RailY, RailZ - RailR - S.UpstopWheelDiameterM * 0.5},
                     Across, S.UpstopWheelDiameterM, S.WheelWidthM, S.WheelSides);

            // Side friction: against the INNER face, so its offset is toward the
            // track centre and its axis is vertical rather than across.
            AddWheel(M.Wheels,
                     FVec3{X, RailY - Sign * (RailR + S.SideWheelDiameterM * 0.5), RailZ},
                     Vertical, S.SideWheelDiameterM, S.WheelWidthM, S.WheelSides);
        }
    }

    return M;
}

// Where one car sits: its own arc length, and the heartline frame there.
struct FCarPlacement
{
    double S = 0.0;      // arc length of the car's CENTRE
    FTrackFrame Frame;   // the heartline frame at that arc length
};

// ===================== A FRAME BETWEEN TWO SAMPLES =====================
//
// The path is walked at a fixed spacing and a car centre lands wherever it
// lands, so the frame has to be interpolated. NEAREST-SAMPLE WOULD QUANTISE THE
// TRAIN: at 0.5 m spacing the cars would visibly snap forward half a metre at a
// time while the ride ran smoothly underneath them.
//
// WalkTrack's LAST interval is short — it clamps the final sample to the track
// length rather than overshooting — so the index arithmetic takes the total
// rather than assuming every interval is Spacing. Assuming it puts the last car
// of a train standing at the station in the wrong place, which is the one place
// somebody is looking closely.
//
// The basis is RE-ORTHONORMALISED after the lerp, because a lerp of two unit
// vectors is not a unit vector and a lerp of two perpendicular pairs is not
// perpendicular. At half a metre the correction is tiny; without it a body is
// visibly not square to its own wheels.
inline FTrackFrame FrameAtDistance(const std::vector<FTrackFrame>& Path,
                                   double Spacing, double TotalLength, double S)
{
    if (Path.empty()) { return FTrackFrame(); }
    if (Path.size() == 1 || !(Spacing > 0.0) || !(TotalLength > 0.0))
    {
        return Path.front();
    }

    const double Clamped = std::max(0.0, std::min(TotalLength, S));
    std::size_t i = static_cast<std::size_t>(Clamped / Spacing);
    if (i + 1 >= Path.size()) { i = Path.size() - 2; }

    const double S0 = Spacing * static_cast<double>(i);
    const double S1 = std::min(TotalLength, Spacing * static_cast<double>(i + 1));
    const double Span = S1 - S0;
    const double T = Span > 1e-12
        ? std::max(0.0, std::min(1.0, (Clamped - S0) / Span))
        : 0.0;

    const FTrackFrame& A = Path[i];
    const FTrackFrame& B = Path[i + 1];

    FTrackFrame F = A;
    F.Position = A.Position + (B.Position - A.Position) * T;
    F.Tangent = Normalised(A.Tangent + (B.Tangent - A.Tangent) * T);
    if (Length(F.Tangent) < 1e-12) { return A; }

    // Gram-Schmidt the lateral against the tangent, then take Up from the cross,
    // so the triple comes out exactly right-handed rather than approximately so.
    FVec3 Lat = A.Lateral + (B.Lateral - A.Lateral) * T;
    Lat = Lat - F.Tangent * Dot(Lat, F.Tangent);
    if (Length(Lat) < 1e-12) { return A; }
    F.Lateral = Normalised(Lat);
    F.Up = Normalised(Cross(F.Tangent, F.Lateral));

    F.Roll = A.Roll + (B.Roll - A.Roll) * T;
    F.YawCurvature = A.YawCurvature + (B.YawCurvature - A.YawCurvature) * T;
    F.PitchCurvature = A.PitchCurvature + (B.PitchCurvature - A.PitchCurvature) * T;
    return F;
}

// ===================== WHERE THE CARS ARE =====================
//
// `NoseS` is the arc length of the FRONT of the leading car, which is the same
// reference the physics and the stop marks already use — a train is dispatched
// and held by where its nose is, so the mesh asking for the same number means
// there is nothing to convert and nothing to disagree about.
//
// Car 0 is the leading car. Its centre is half a pitch behind the nose, and each
// one after that a further pitch back.
//
// bWrap IS FOR A CIRCUIT, and it is the caller's to know rather than this
// function's to guess — exactly as `bCapEnds` is for the track mesh, and settled
// by the same measured `bCircuit`. On an open layout a train that runs past the
// end should run past it; on a circuit it should come round.
inline std::vector<FCarPlacement> PlaceCars(const std::vector<FTrackFrame>& Path,
                                            double Spacing, double TotalLength,
                                            double NoseS, const FTrainSettings& S,
                                            bool bWrap)
{
    std::vector<FCarPlacement> Out;
    if (Path.empty() || S.CarCount <= 0 || !(S.CarLengthM > 0.0) || !(TotalLength > 0.0))
    {
        return Out;
    }
    Out.reserve(static_cast<std::size_t>(S.CarCount));
    for (int i = 0; i < S.CarCount; ++i)
    {
        double Centre = NoseS - S.CarLengthM * (static_cast<double>(i) + 0.5);
        if (bWrap)
        {
            Centre = std::fmod(Centre, TotalLength);
            if (Centre < 0.0) { Centre += TotalLength; }
        }
        else
        {
            // CLAMPED HERE RATHER THAN ONLY INSIDE FrameAtDistance, so that a
            // placement S and its own frame describe the same place. Leaving the
            // raw value would report a car at -9.5 m carrying the frame from 0 m,
            // which is two answers to one question and exactly the kind that gets
            // read off in a debug overlay and believed.
            Centre = std::max(0.0, std::min(TotalLength, Centre));
        }
        FCarPlacement P;
        P.S = Centre;
        P.Frame = FrameAtDistance(Path, Spacing, TotalLength, Centre);
        Out.push_back(P);
    }
    return Out;
}

// A car's local point, put into the world by its OWN frame. The whole of
// "articulation comes free" is that this is called with a different frame per
// car rather than one frame for the train.
inline FVec3 CarToWorld(const FTrackFrame& F, const FVec3& Local)
{
    return F.Position + F.Tangent * Local.X + F.Lateral * Local.Y + F.Up * Local.Z;
}

// Concatenate one buffer onto another, shifting the indices. Several trains run
// on one circuit and they are one material each, so they are one section each —
// six trains as six draw calls of the same fibreglass would be six times the
// cost for nothing.
inline void AppendBuffer(FMeshBuffer& Out, const FMeshBuffer& In)
{
    const std::uint32_t Base = static_cast<std::uint32_t>(Out.Position.size());
    Out.Position.insert(Out.Position.end(), In.Position.begin(), In.Position.end());
    Out.Normal.insert(Out.Normal.end(), In.Normal.begin(), In.Normal.end());
    Out.UV.insert(Out.UV.end(), In.UV.begin(), In.UV.end());
    for (std::uint32_t I : In.Index) { Out.Index.push_back(Base + I); }
}

// Copy a car's buffer into world space at one placement.
//
// NORMALS ROTATE WITHOUT THE TRANSLATION. Adding the position to a normal is the
// classic version of this bug, and it does not crash or disappear — it lights
// the whole train from nowhere, which reads as a material problem.
inline void AppendCarBuffer(FMeshBuffer& Out, const FMeshBuffer& Car, const FTrackFrame& F)
{
    const std::uint32_t Base = static_cast<std::uint32_t>(Out.Position.size());
    for (std::size_t v = 0; v < Car.Position.size(); ++v)
    {
        Out.Position.push_back(CarToWorld(F, Car.Position[v]));
        const FVec3& N = Car.Normal[v];
        Out.Normal.push_back(F.Tangent * N.X + F.Lateral * N.Y + F.Up * N.Z);
        Out.UV.push_back(Car.UV[v]);
    }
    for (std::uint32_t I : Car.Index) { Out.Index.push_back(Base + I); }
}

// ===================== ONE LAP BAR, IN THE CAR'S SPACE =====================
//
// Two arms from the hinges and a crossbar between their tips: three struts,
// which SweepStrut already caps, so the bar is closed geometry at every
// position. `Position` is the bank's — 0 fully open, 1 fully closed — and it
// moves only the angle, never the topology, which is what lets the actor update
// vertices in place as the bars come down instead of recreating a section.
//
// The hinge sits behind the row centre and the arm points forward when closed,
// so the crossbar comes down over the lap rather than onto the hip.
inline void AddRestraintBar(FMeshBuffer& Out, const FTrainSettings& S,
                            const FTrackProfile& Profile, double RailZ,
                            double RowX, double Position)
{
    const double P = std::max(0.0, std::min(1.0, Position));
    // THE HINGE IS ON THE SEAT, not at an authored height: the hip is where the
    // squab puts it, and a bar hinged anywhere else swings over nobody.
    const double HingeZ = RailZ + CabinFloorHeight(S, ShellKeepOut(S, Profile))
                        + S.SeatHeightM + S.BarHingeUpM;
    const double Swing = (1.0 - P) * S.BarRaisedDeg * (TrackMeshTwoPi / 360.0);
    const FVec3 Dir{std::cos(Swing), 0.0, std::sin(Swing)};
    const double HalfY = std::max(0.05, S.BodyWidthM * 0.5 - S.BarInsetM);
    const double R = S.BarDiameterM * 0.5;
    if (!(R > 0.0) || !(S.BarArmLengthM > 0.0)) { return; }

    const FVec3 Across{0.0, 1.0, 0.0};
    const FVec3 Vertical{0.0, 0.0, 1.0};
    FVec3 Tip[2];
    for (int Side = 0; Side < 2; ++Side)
    {
        const double Y = Side == 0 ? HalfY : -HalfY;
        const FVec3 Hinge{RowX - S.BarHingeBackM, Y, HingeZ};
        Tip[Side] = Hinge + Dir * S.BarArmLengthM;
        SweepStrut(Out, Hinge, Tip[Side], Across, R, S.StrutSides);
    }
    SweepStrut(Out, Tip[0], Tip[1], Vertical, R, S.StrutSides);
}

// ===================== THE WHOLE TRAIN, IN WORLD SPACE =====================
//
// One car built once and stamped at each placement, plus the couplers — the only
// geometry that spans two cars and therefore the only geometry that cannot be
// built in a car's own space.
//
// A COUPLER IS A SHORT STRUT between adjacent chassis ends: SweepStrut, which
// the ties and the support legs already share. It is also the piece that makes
// the articulation legible, because on a curve it visibly angles between two
// bodies that are each dead straight.
//
// THIS IS THE MEASURING FORM. In the engine the per-car buffers stay separate
// and only the transforms move each frame, which is the whole reason
// BuildCarMesh does not take a track. This exists so the geometry can be
// asserted as one closed thing, and so a caller with no per-car component
// support still has something to draw.
//
// `RowPositions` is one bank position per seat row over the whole train, car 0
// row 0 first. A row it does not cover is drawn CLOSED: a train out on the course
// with no bank in reach of it is carrying riders, and a bar that defaulted open
// there would read as a fault the ride does not have.
inline FTrainMesh BuildTrainMesh(const std::vector<FCarPlacement>& Cars,
                                 const FTrainMesh& Car, const FTrainSettings& S,
                                 double HeartlineHeight,
                                 const std::vector<double>& RowPositions = {},
                                 const FTrackProfile& Profile = FTrackProfile())
{
    FTrainMesh Out;
    const int Rows = std::max(1, S.RowsPerCar);
    const double RailZ = -HeartlineHeight;
    for (std::size_t c = 0; c < Cars.size(); ++c)
    {
        const FCarPlacement& P = Cars[c];
        AppendCarBuffer(Out.Body, Car.Body, P.Frame);
        AppendCarBuffer(Out.Chassis, Car.Chassis, P.Frame);
        AppendCarBuffer(Out.Wheels, Car.Wheels, P.Frame);
        AppendCarBuffer(Out.Seats, Car.Seats, P.Frame);

        // The bars are the one part built here rather than stamped: their
        // angle is per row, so a car's bars are not the same geometry as the
        // next car's.
        FMeshBuffer Bars;
        for (int r = 0; r < Rows; ++r)
        {
            const std::size_t i = c * static_cast<std::size_t>(Rows) + static_cast<std::size_t>(r);
            const double Pos = i < RowPositions.size() ? RowPositions[i] : 1.0;
            AddRestraintBar(Bars, S, Profile, RailZ, RowCentreX(S, r), Pos);
        }
        AppendCarBuffer(Out.Restraints, Bars, P.Frame);
    }

    const double ChassisZ = -HeartlineHeight - S.ChassisDropM;
    const double ChassisHalf = ChassisHalfLength(S);

    for (std::size_t i = 0; i + 1 < Cars.size(); ++i)
    {
        // Car i is AHEAD of car i+1, so the coupler runs from the rear end of the
        // one in front to the front end of the one behind.
        const FVec3 From = CarToWorld(Cars[i].Frame, FVec3{-ChassisHalf, 0.0, ChassisZ});
        const FVec3 To = CarToWorld(Cars[i + 1].Frame, FVec3{ChassisHalf, 0.0, ChassisZ});
        if (Length(To - From) < 1e-9) { continue; }
        SweepStrut(Out.Couplers, From, To, Cars[i].Frame.Up,
                   S.CouplerDiameterM * 0.5, S.StrutSides);
    }
    return Out;
}

// ===================== WHAT THE TRAIN CANNOT BE BUILT AS =====================
//
// REPORTED, NEVER REPAIRED. The same rule as the mesher's too-tight curve, the
// validator, the envelope judge and the device shorter than its train: the fix
// for a train longer than its track is a shorter train, and one silently
// truncated reads as a vehicle that fits.
//
// The SILENCE is what keeps these honest — a train on default settings reports
// nothing at all, and a checker that complained about every ordinary vehicle
// would be switched off within a day.
inline std::vector<FMeshFinding> AuditTrain(const FTrainSettings& S,
                                            double HeartlineHeight,
                                            const FTrackProfile& Profile,
                                            double TotalLength)
{
    std::vector<FMeshFinding> Out;
    char Buf[240];

    if (BodySwallowsHeartline(S, HeartlineHeight))
    {
        std::snprintf(Buf, sizeof(Buf),
            "the body is %.2f m tall on a %.2f m heartline, so its roof closes over the "
            "point the rider's eye and the rider's felt G are both measured at. Lower it, "
            "or the lap is spent inside a box.", S.BodyHeightM, HeartlineHeight);
        Out.push_back({0.0, 0.0, 0.0, Buf});
    }

    // THE SIDE WHEELS BIND BEFORE THE RAILS DO, which is why this reports against
    // them. The floor line runs through the rail plane and the side-friction
    // pucks lie flat in that same plane, INBOARD of the rails — so a floor that
    // clears the rails can still be drawn straight through both pucks, and that
    // one is invisible from outside because the wheel is inside the shell.
    const double FloorHalf = S.BodyFloorWidthM * 0.5;
    const FShellKeepOut K = ShellKeepOut(S, Profile);
    if (FloorHalf > K.FloorHalfWidth)
    {
        std::snprintf(Buf, sizeof(Buf),
            "the body floor is %.2f m across where the side-friction wheels leave only "
            "%.2f m clear between them, so the shell would be drawn through both. It is "
            "built narrow instead. Narrow the floor, or widen the gauge.",
            S.BodyFloorWidthM, K.FloorHalfWidth * 2.0);
        Out.push_back({0.0, 0.0, 0.0, Buf});
    }

    if (TotalLength > 0.0 && S.TrainLengthM() > TotalLength)
    {
        std::snprintf(Buf, sizeof(Buf),
            "the train is %.1f m long on %.1f m of track, so it meets its own tail. "
            "Fewer cars, or more track.", S.TrainLengthM(), TotalLength);
        Out.push_back({0.0, 0.0, 0.0, Buf});
    }

    return Out;
}

// ponytail: no nose on the leading car, no wheel spin, no suspension travel, and
// one train style rather than several. Each is named as out of scope in
// TRAIN_DESIGN.md, and none of them changes an interface here — a nose is
// another section function, and wheel spin is a per-wheel angle this already has
// the axis for. One credible vehicle is the gate. (The restraint came off this
// list on 2026-08-27: a lap bar per row, swung by the bank, in AddRestraintBar.)
//
// ponytail: no FSeat here either, deliberately. The design sheet specifies it and
// three cards want it, but its consumers are the ride camera, the wing-coaster
// lateral offset and the restraint — and none of those is this card. Building it
// now would be a data model with nothing reading it, which is precisely how a
// second source of truth for where the rider is gets established by accident.
