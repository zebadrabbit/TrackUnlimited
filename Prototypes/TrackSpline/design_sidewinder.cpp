// The SIDEWINDER, designed: the sixth template, and the first ride on this
// project built to a brief rather than to prove a feature.
//
//   clang++ -std=c++17 -O2 -Wall -Wextra -o design_sidewinder design_sidewinder.cpp
//
// THE BRIEF (developer, 2026-08-23): about 45 s, a train of 6-8 cars, a chain
// lift and a drop, banking left AND right, a helix, several small hills back
// to the station, block brakes enough for two trains moving and one staging,
// the station a little above the ground.
//
// THIS PROGRAM IS THE DESIGN. It authors the ride in the same vocabulary the
// editor and the presets use, SOLVES the two closure lengths rather than
// eyeballing them (the drop straight is the height lever, the fill straight
// the plan lever -- the same pair the two-train circuit uses), runs the ride
// through the physics, and prints every figure the preset and the docs quote.
// Run it before changing a number in SidewinderLayout().
//
// UX NOTES WERE THE OTHER DELIVERABLE. Everything here that the RUNTIME editor
// could not have done is marked UX: -- the list is on the Trello card.

#include "TrackIO.h"
#include "TrackClose.h"
#include "../TrainPhysics/RideProfile.h"

#include <cmath>
#include <cstdio>
#include <vector>

namespace
{
constexpr double Pi = 3.14159265358979323846;
constexpr double G = 9.81;
inline double Deg(double D) { return D * Pi / 180.0; }
inline double BankDeg(double SpeedMs, double R) { return std::atan2(SpeedMs * SpeedMs, G * R) * 180.0 / Pi; }

struct FDesign
{
    FTrackDocument Doc;
    std::size_t DropIndex = 0;   // the height lever
    std::size_t FillIndex = 0;   // the plan lever (along)

    void Add(const FAuthoredSegment& A) { Doc.Segments.push_back(A); }

    std::size_t Straight(double L, EAuthoredZone Z = EAuthoredZone::None, double Speed = 4.0,
                         double Accel = 6.0, double Decel = 6.0, double Pad = 0.0)
    {
        FAuthoredSegment A; A.Kind = ESegmentKind::Straight; A.Length = L;
        A.Zone = Z; A.ZoneSpeed = Speed; A.ZoneAccel = Accel; A.ZoneDecel = Decel; A.ZoneBrakeDecel = Pad;
        Add(A); return Doc.Segments.size() - 1;
    }
    // UX: a vertical curve is two RAW segments with pitch curvature, which the
    // runtime editor cannot author at all -- Raw is not a cycle destination
    // and has no fields. Every hill in every preset comes from this helper.
    void EasedPitch(double DeltaRad, double K, EAuthoredZone Z = EAuthoredZone::None, double Speed = 4.0,
                    double Accel = 6.0)
    {
        const double Kk = DeltaRad >= 0 ? K : -K;
        const double L = std::fabs(DeltaRad) / K;
        FAuthoredSegment In; In.Kind = ESegmentKind::Raw; In.RawSegment.Length = L;
        In.RawSegment.PitchCurvatureEnd = Kk; In.Zone = Z; In.ZoneSpeed = Speed; In.ZoneAccel = Accel; In.ZoneDecel = Accel; Add(In);
        FAuthoredSegment Out; Out.Kind = ESegmentKind::Raw; Out.RawSegment.Length = L;
        Out.RawSegment.PitchCurvatureStart = Kk; Out.Zone = Z; Out.ZoneSpeed = Speed; Out.ZoneAccel = Accel; Out.ZoneDecel = Accel; Add(Out);
    }
    // Clothoid in, arc, clothoid out, roll ramped across the easements.
    // UX: three segments and seven numbers for one banked turn; the radius and
    // the bank are typed three times each.
    void BankedTurn(double R, double ArcDeg, double Ease, double Bank)
    {
        const double Arc = Deg(ArcDeg) * std::fabs(R) - Ease;
        FAuthoredSegment A; A.Kind = ESegmentKind::Clothoid; A.Length = Ease; A.CurvatureStart = 0; A.CurvatureEnd = 1.0 / R;
        A.RollStartDegrees = 0; A.RollEndDegrees = Bank; A.RollMode = ERollMode::WorldBank; Add(A);
        FAuthoredSegment B; B.Kind = ESegmentKind::Arc; B.Length = Arc; B.Radius = R;
        B.RollStartDegrees = Bank; B.RollEndDegrees = Bank; B.RollMode = ERollMode::WorldBank; Add(B);
        FAuthoredSegment C; C.Kind = ESegmentKind::Clothoid; C.Length = Ease; C.CurvatureStart = 1.0 / R; C.CurvatureEnd = 0;
        C.RollStartDegrees = Bank; C.RollEndDegrees = 0; C.RollMode = ERollMode::WorldBank; Add(C);
    }
    // Up, over, down, back to level: a symmetric airtime hill of a given
    // approach angle. Net height zero by construction.
    void Hill(double AngleDeg, double K)
    {
        EasedPitch(Deg(AngleDeg), K);
        EasedPitch(-2.0 * Deg(AngleDeg), K);
        EasedPitch(Deg(AngleDeg), K);
    }
};

FDesign Sidewinder()
{
    FDesign D;
    // Seven 3 m cars: 21 m. Every holding device is 30 m, which is the train
    // plus the 1 m nose clearance plus margin.
    const double Station = 26.0;   // 21 m train + 1 m nose clearance + margin

    // ---- STATION and the LIFT. Chain at 6 m/s, 35 degrees, crest about 27 m
    // up. The station sits at z = 0 and the helix dips below it, so the station
    // ends up 3 m above the ground -- the brief's "slightly above".
    D.Straight(Station, EAuthoredZone::Station, 3.0, 1.5, 1.5);            // 0
    // 8 m/s^2 of grip: a 35 degree chain needs g*sin(35) = 5.6 just to HOLD,
    // and the first number typed here was 2, which stalled the train 51 m in.
    // UX: nothing says a device's grip is under its own gradient; the audit
    // checks whether a brake can stop a train and not whether a lift can lift.
    D.Straight(6.0, EAuthoredZone::Lift, 8.0, 8.0, 8.0);                    // 1  chain engages
    D.EasedPitch(Deg(35.0), 0.05, EAuthoredZone::Lift, 8.0, 8.0);                // 2-3
    D.Straight(56.0, EAuthoredZone::Lift, 8.0, 8.0, 8.0);                   // 4  the climb (56: at 46 the train died 1 m short of the last brake)
    D.EasedPitch(-Deg(35.0), 0.05, EAuthoredZone::Lift, 8.0, 8.0);               // 5-6 over the crest
    D.Straight(4.0, EAuthoredZone::Lift, 8.0, 8.0, 8.0);                    // 7  chain lets go, level

    // ---- THE DROP. 40 degrees; the straight's length is SOLVED for closure.
    // 50 degrees was the first number and its easements ALONE descended 60 m
    // from a 26 m crest -- the height lever had nowhere to go. UX: the editor
    // shows no height at a segment end, so that is found by riding it.
    D.EasedPitch(-Deg(40.0), 0.025);                                        // 8-9
    D.DropIndex = D.Straight(15.0);                                         // 10 height lever
    D.EasedPitch(Deg(40.0), 0.025);                                         // 11-12 pull-out, level (0.025 keeps the bottom near 3.2 g)

    // ---- THE FILL, on the OUTBOUND leg at the bottom of the drop: the plan
    // lever, where the train is fastest and a straight costs least. A hill
    // stood here for most of the design; every metre of it was a metre of
    // dead fill on the return leg, which is where the energy runs out.
    // UX: the plan balance of an oval -- outbound extent equals return
    // extent -- is nowhere in the editor; it is found by the seam not closing.
    D.FillIndex = D.Straight(40.0);

    // ---- TURN 1: 180 left at about 22 m/s.
    D.BankedTurn(35.0, 180.0, 40.0, BankDeg(27.0, 35.0));   // 27 m/s is what a 46 m crest delivers here; banked for 22 it read 0.8 g lateral

    // ---- MID-COURSE BLOCK BRAKE, level. Bites to 13 m/s, can hold and convey.
    // 17 m/s out, not 13: at 13 the train stalled on hill 2 (8 m of hill is
    // 8.6 m of v^2/2g, and a brake run is where that margin goes).
    // UX: the profile graph shows the energy running out, but only after
    // riding it; nothing in the editor says "7 m of energy left, 3 m of hill".
    // 25 m/s out, which on this ride is a BLOCK more than a brake: the return
    // leg loses about 0.4 m/s^2 to rolling and aerodynamic drag over 700 m and
    // then climbs 3 m, and that is 600 m2/s2 of v2 -- all of it.
    D.Straight(30.0, EAuthoredZone::BlockBrake, 25.0, 6.0, 6.0, 4.0);       // 22

    // ---- THE SNAKE: right, left, left, right at 40 m. Banking both ways, and
    // plan-neutral, so the closure never sees it.
    // PLAN-NEUTRAL BY SYMMETRY: R,L then L,R with equal angles and easements
    // puts the line back where it was. (A straight between the halves of one S
    // was tried as a third, across lever, and was the only thing NOT closing:
    // the helix's easements are symmetric too, so nothing needed moving.)
    const double Sb = BankDeg(24.0, 45.0);   // banked for the speed it actually gets; at R 35 for 20 m/s the lateral read 0.84 g
    // 45 degree bends with 25 m easements. They were 25 degree bends, which at
    // R 35 is 15 m of arc -- LESS than the two 25 m easements turn (20.5 deg
    // each) -- so the arc came out 9.7 m NEGATIVE and AddSegment silently
    // dropped all four. The snake still closed (symmetric), but the zone walk
    // counted the missing metres and put every later device 39 m early,
    // which is where "the final brake stops driving 20 m in" came from.
    // UX: a negative arc should be a red row, not a vanished segment.
    D.BankedTurn(-45.0, 45.0, 25.0, -Sb);
    D.BankedTurn(45.0, 45.0, 25.0, Sb);
    D.BankedTurn(45.0, 45.0, 25.0, Sb);
    D.BankedTurn(-45.0, 45.0, 25.0, -Sb);

    // ---- HILLS 2 and 3, smaller. (Three was the brief; the third cost the
    // helix its entry speed and went.)
    D.Hill(16.0, 0.035);
    D.Hill(12.0, 0.04);

    // ---- THE HELIX: one full turn left, LEVEL, R 20. It was descending 4
    // degrees first, and the clothoid OUT of a pitched, banked helix turned 13
    // degrees of pitch on its own: un-banking while pitched puts the easement's
    // yaw curvature partly into pitch. A level helix has no such problem, and
    // the descent it was carrying moved into the drop, which the solver sizes.
    // UX: nothing in the editor says a helix should be entered level, or that
    // MakeHelix wants the track already at its climb angle (TrackSpline.h).
    const double Hb = BankDeg(15.0, 18.0);
    { FAuthoredSegment A; A.Kind = ESegmentKind::Clothoid; A.Length = 20.0; A.CurvatureStart = 0; A.CurvatureEnd = 1.0 / 18.0;
      A.RollEndDegrees = Hb; A.RollMode = ERollMode::WorldBank; D.Add(A); }                 // 49
    // ONE FULL TURN INCLUDING THE EASEMENTS: each 20 m clothoid to 1/20 turns
    // L*k/2 = 0.5 rad, so the helix itself is a turn minus a radian. The arc
    // helper subtracts its easements for you; a helix's Turns does not.
    // UX: typing 1.0 here gives 417 degrees and a circuit that does not close.
    const double HelixTurns = 1.0 - (2.0 * (20.0 / 18.0) * 0.5) / (2.0 * Pi);   // L*k/2 each, k = 1/18
    { FAuthoredSegment H; H.Kind = ESegmentKind::Helix; H.Radius = 18.0; H.ClimbAngleDegrees = 0.0; H.Turns = HelixTurns;
      H.RollStartDegrees = Hb; H.RollEndDegrees = Hb; H.RollMode = ERollMode::WorldBank; D.Add(H); }  // 50
    { FAuthoredSegment C; C.Kind = ESegmentKind::Clothoid; C.Length = 20.0; C.CurvatureStart = 1.0 / 18.0; C.CurvatureEnd = 0;
      C.RollStartDegrees = Hb; C.RollMode = ERollMode::WorldBank; D.Add(C); }               // 51

    // ---- TURN 2: 180 left, same radius as turn 1 -- or it does not close.
    // IDENTICAL TO TURN 1 -- radius AND easement -- or the return line lands
    // 2 x (R1 - R2) off the station line. R 30 was tried for speed: 11 m short.
    // Taken at the low level, BEFORE the rise: with the rise first the train
    // entered this 150 m turn at 8 m/s, spent 18 s in it and came out at 3.6.
    D.BankedTurn(35.0, 180.0, 40.0, BankDeg(12.0, 35.0));

    // ---- THE RISE back to station height: about 3 m at 12 degrees, nearly
    // all of it in the easements. 4.5 m stalled the train 1 m short.
    D.EasedPitch(Deg(12.0), 0.03);
    D.Straight(1.0);
    D.EasedPitch(-Deg(12.0), 0.03);

    // ---- THE FINAL BLOCK BRAKE, and home.
    D.Straight(Station, EAuthoredZone::BlockBrake, 3.0, 3.0, 3.0, 3.0);     // 63
    return D;
}

// Zones from contiguous runs of equal kind and speed, as the actor derives them.
void AddZones(FTrain& Train, const FTrackDocument& Doc)
{
    double S = 0.0; std::size_t i = 0;
    while (i < Doc.Segments.size())
    {
        const FAuthoredSegment& A = Doc.Segments[i];
        const double Start = S; const EAuthoredZone Z = A.Zone; const double Sp = A.ZoneSpeed;
        const double Accel = A.ZoneAccel, Decel = A.ZoneDecel, Pad = A.ZoneBrakeDecel;
        while (i < Doc.Segments.size() && Doc.Segments[i].Zone == Z && Doc.Segments[i].ZoneSpeed == Sp)
        { S += BuildSegment(Doc.Segments[i]).Length; ++i; }
        if (Z == EAuthoredZone::None) { continue; }
        std::printf("  zone %-10s %7.1f - %7.1f m  %.1f m/s\n",
            Z == EAuthoredZone::Station ? "STATION" : Z == EAuthoredZone::Lift ? "LIFT" : Z == EAuthoredZone::BlockBrake ? "BLOCK BRAKE" : "BRAKE",
            Start, S, Sp);
        if (Z == EAuthoredZone::Brake) { Train.AddZone(MakeBrake(Start, S, Sp, Decel)); continue; }
        FTrackZone T{Start, S, Sp, Accel, Decel};
        if (Pad > 0.0) { T.BrakeLimit = Sp; T.BrakeDecel = Pad; }
        Train.AddZone(T);
    }
}
} // namespace

int main()
{
    FDesign D = Sidewinder();
    FTrack T0 = BuildTrack(D.Doc);
    // EVERY AUTHORED SEGMENT MUST BUILD. A negative arc (easements longer than
    // the turn) is refused by AddSegment without a word, and the zone walk
    // below would then disagree with the track about where everything is.
    if (T0.NumSegments() != D.Doc.Segments.size())
    {
        for (std::size_t i = 0; i < D.Doc.Segments.size(); ++i)
        {
            const double L = BuildSegment(D.Doc.Segments[i]).Length;
            if (!(L > 0.0)) { std::printf("segment %zu has length %g and was REFUSED\n", i, L); }
        }
        return 3;
    }

    // ---- CLOSE IT. Position in all three axes and heading; roll closes by
    // construction because every bank returns to zero.
    FClosureTarget Target = CircuitTarget(T0);
    Target.bMatchHeading = true;
    FClosureOptions Opt; Opt.MaxIterations = 80;
    const FClosureResult R = SolveClosure(D.Doc, Target,
        {FreeLength(D.DropIndex, 1.0, 200.0), FreeLength(D.FillIndex, 1.0, 400.0)}, Opt);
    std::printf("closure: %s after %d iterations -- gap %.6f m -> %.6f m, heading %.6f deg\n",
        R.bConverged ? "CONVERGED" : "FAILED", R.Iterations, R.Before.ActiveError, R.After.ActiveError,
        R.After.HeadingError * 180.0 / Pi);
    std::printf("  drop straight = %.7f m   fill straight = %.7f m\n",
        D.Doc.Segments[D.DropIndex].Length, D.Doc.Segments[D.FillIndex].Length);
    if (!R.bConverged) { std::printf("%s\n", R.Message.c_str()); return 1; }

    // ---- RIDE IT. Seven cars.
    const FTrack T = BuildTrack(D.Doc);
    FTrainConfig C; C.TrainLength = 21.0;
    FTrain Train(T, C);
    AddZones(Train, D.Doc);
    const FRideProfile P = RunRideProfile(Train, T, 0.5);
    std::printf("track: %zu segments, %.1f m\n", D.Doc.Segments.size(), T.TotalLength());
    std::printf("ride:  %s, %.1f s, top %.1f m/s (%.1f km/h) at %.0f m\n",
        P.bCompleted ? "completed" : "STALLED", P.Duration, P.TopSpeed, P.TopSpeed * 3.6, P.TopSpeedS);
    std::printf("       vertical %.2f..%.2f g, lateral %.2f g, roll rate %.1f deg/s\n",
        P.MinVerticalG, P.MaxVerticalG, P.MaxAbsLateralG, P.MaxAbsRollRate);
    std::printf("       (min vertical at %.0f m, max at %.0f m, lateral at %.0f m, roll rate at %.0f m)\n",
        P.MinVerticalGS, P.MaxVerticalGS, P.MaxAbsLateralGS, P.MaxAbsRollRateS);
    std::printf("       height %.1f..%.1f m (station at 0: %.1f m above the lowest point)\n",
        P.LowestHeight, P.HighestHeight, -P.LowestHeight);
    if (!P.bCompleted) { std::printf("       stalled at %.1f m, %.1f m up\n", P.StalledAtS, P.StalledHeight); }

    // ---- The holding places: station, lift, two block brakes = 4, so 3 trains.
    int Holding = 0;
    for (const FAuthoredSegment& A : D.Doc.Segments)
    {
        if (A.Zone == EAuthoredZone::Station || A.Zone == EAuthoredZone::Lift || A.Zone == EAuthoredZone::BlockBrake) { ++Holding; }
    }
    std::printf("holding segments: %d (runs merge; expect 4 places: station, lift, mid brake, final brake)\n", Holding);

    // ---- The numbers to transcribe into SidewinderLayout().
    std::printf("\nTRANSCRIBE:\n  const double DropLen = %.7f;\n  const double FillLen = %.7f;\n",
        D.Doc.Segments[D.DropIndex].Length, D.Doc.Segments[D.FillIndex].Length);
    return P.bCompleted ? 0 : 2;
}
