// Build & run:  clang++ -std=c++17 -Wall -Wextra -O2 -o test_twotrains test_twotrains.cpp && ./test_twotrains
//
// Two trains, one circuit, real geometry. The only file that crosses all three
// layers — FTrack geometry, FTrain physics, FRideSignals interlocking — and the
// reason it exists is that nothing below it can tell whether they agree.
//
// The layout is a TRANSCRIPTION of ATUCoasterRide::TwoTrainCircuitLayout(), which
// lives in the Unreal actor and cannot be included from here. Same precedent as
// TrackSpline/reference_figures.cpp. If the preset is edited, edit this too — the
// block lengths asserted below are the thing that would notice.
//
// THE LAYOUT IS A CLOSED CIRCUIT, to 0.000000 m of position, 0.000084 degrees of
// heading and 0.000000 degrees of roll. It did not used to be: it closed in
// HEIGHT only, ending 373.794 m from the station pointing 86.421 degrees wrong,
// because its two turns summed to 446.42 degrees where a circuit needs 360.
//
// What closed it is a SHAPE, not a solver. An oval closes analytically when both
// turns are the same way, exactly 180 degrees, the same radius and the same
// easement — the lateral displacements cancel, the along-track ones cancel,
// heading sums to 360, and one scalar condition is left over.
//
// WHAT THIS STILL DOES NOT DO: FTrain has no lap. It clamps at the end of the
// track, so the actor returns a train to the station by teleporting it. With the
// seam closed to zero that teleport is now INVISIBLE — the two points are the
// same place — so it is a tidiness problem rather than a visible one, and the
// real fix is S -= TotalLength once FTrack can say it is a circuit.

#include "../BlockSignal/RideSignals.h"
#include "../BlockSignal/TrackDrives.h"
#include "../BlockSignal/TrackSensors.h"
#include "../TrackSpline/TrackIO.h"
#include "../TrackSpline/TrackProfile.h"
#include "../TrackSpline/TrackValidate.h"
#include "RideProfile.h"
#include "TrainPhysics.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

namespace
{

const double Pi = 3.14159265358979323846;
const double Grip = 6.0;          // as ATUCoasterRide sets it for every zone
const double TrainLen = 15.0;

// Where a held train parks: its NOSE this far short of the far end of the block.
// About a metre is typical, and the margin is the reason the number exists - it
// is what stops a train protruding into the next zone through a defect. The
// brake puts it down well short of here; the tyres truck it the rest of the way.
const double NoseClearance = 1.0;

double Deg(double D) { return D * Pi / 180.0; }

// Mirrors ETUSegmentZone. BlockBrake is the kind the actor gained for this: a
// brake run WITH drive tyres, which is what every real block brake is and the
// only thing that can both stop a train and let it go again. It is a separate
// enumerator rather than a reused Lift because block boundaries fall where the
// KIND changes, and two holding devices in a row have to stay two blocks.
enum class EZone { None, Lift, Launch, Brake, BlockBrake, Station };

struct FItem
{
    FAuthoredSegment A;
    EZone Zone = EZone::None;
    double Speed = 0.0;
};

void AddStraight(std::vector<FItem>& O, double L, EZone Z = EZone::None, double Sp = 0.0)
{
    FItem I;
    I.A.Kind = ESegmentKind::Straight;
    I.A.Length = L;
    I.Zone = Z;
    I.Speed = Sp;
    O.push_back(I);
}

// A hill as two raw segments: curvature up to a peak, then back to zero. Same
// shape the actor uses, and the reason hills are Raw rather than a kind of their
// own — a pitch change IS a curvature profile.
void AddEasedPitch(std::vector<FItem>& O, double Delta, double Peak)
{
    const double K = Delta >= 0.0 ? Peak : -Peak;
    const double L = std::fabs(Delta) / Peak;
    FItem A;
    A.A.Kind = ESegmentKind::Raw;
    A.A.Length = L;
    A.A.RawSegment.Length = L;
    A.A.RawSegment.PitchCurvatureEnd = K;
    O.push_back(A);
    FItem B;
    B.A.Kind = ESegmentKind::Raw;
    B.A.Length = L;
    B.A.RawSegment.Length = L;
    B.A.RawSegment.PitchCurvatureStart = K;
    O.push_back(B);
}

void AddBankedTurn(std::vector<FItem>& O, double R, double Arc, double Ease, double BankDeg)
{
    FItem A;
    A.A.Kind = ESegmentKind::Clothoid;
    A.A.Length = Ease;
    A.A.CurvatureStart = 0.0;
    A.A.CurvatureEnd = 1.0 / R;
    A.A.RollEndDegrees = BankDeg;
    O.push_back(A);
    FItem B;
    B.A.Kind = ESegmentKind::Arc;
    B.A.Length = Arc;
    B.A.Radius = R;
    B.A.RollStartDegrees = B.A.RollEndDegrees = BankDeg;
    O.push_back(B);
    FItem C;
    C.A.Kind = ESegmentKind::Clothoid;
    C.A.Length = Ease;
    C.A.CurvatureStart = 1.0 / R;
    C.A.CurvatureEnd = 0.0;
    C.A.RollStartDegrees = BankDeg;
    O.push_back(C);
}

double BankDegreesFor(double V, double R)
{
    return std::atan((V * V) / (GravityMs2 * R)) * 180.0 / Pi;
}

std::vector<FItem> Layout()
{
    const double Up = Deg(26.0);       // pull-up out of the launch
    const double Dn = Deg(32.0);       // the drop out of the turnaround
    const double R = 35.0;
    const double Ease = 50.0;
    const double Arc = Pi * R - Ease;  // exactly 180 degrees, easements included
    const double DropLen = 15.6847323;    // solved: height returns to station level
    const double FillLen = 75.5024975;    // solved: horizontal extents balance

    std::vector<FItem> Out;
    // LEG A, outbound.
    AddStraight(Out, 26.0, EZone::Station, 1.5);           // 0 STATION, drive tyres
    AddStraight(Out, 150.0, EZone::Launch, 38.0);          // 1 LAUNCH
    AddEasedPitch(Out, Up, 0.0130);
    AddStraight(Out, 40.0);
    AddEasedPitch(Out, -Up, 0.0130);
    // TURN 1, level, at the top of the hill and taken slowly.
    AddBankedTurn(Out, R, Arc, Ease, BankDegreesFor(14.2, R));
    // LEG B, the return.
    AddEasedPitch(Out, -Dn, 0.0150);
    AddStraight(Out, DropLen);
    AddEasedPitch(Out, Dn, 0.0150);
    AddEasedPitch(Out, Deg(20.0), 0.024);                  // airtime hill
    AddEasedPitch(Out, Deg(-40.0), 0.024);
    AddEasedPitch(Out, Deg(20.0), 0.024);
    AddStraight(Out, FillLen);
    AddStraight(Out, 130.0, EZone::BlockBrake, 20.0);      // 3 MID-COURSE, holds
    // TURN 2, level, at station height.
    AddBankedTurn(Out, R, Arc, Ease, BankDegreesFor(18.1, R));
    // LEG C, the approach, collinear with leg A and closing onto the station.
    AddStraight(Out, 24.0);
    AddStraight(Out, 37.5, EZone::BlockBrake, 6.0);        // 5 OUTER, holds
    AddStraight(Out, 27.0, EZone::Lift, 4.0);              // 6 TRANSFER tyres, holds
    AddStraight(Out, 37.5, EZone::BlockBrake, 2.0);        // 7 INNER, holds
    return Out;
}

// Everything the walk over the segment list produces, in one place, because zones
// and block boundaries are the SAME fact derived once: a boundary is only
// meaningful where there is a device that can hold a train.
struct FCircuit
{
    FTrackDocument Doc;
    std::vector<double> Boundaries;
    std::vector<double> Authored;    // per zone, the speed it releases at
    std::vector<double> HoldMidS;    // where a train may stand: mid-device
    std::vector<double> StopMarkS;   // per zone, the switch that says "far enough"
};

// Tr is optional so the shape can be asked for without a train to hang zones on
// — FTrain wants a built track in its constructor, and the track comes out of
// this same walk.
FCircuit BuildCircuit(FTrain* Tr)
{
    const std::vector<FItem> Items = Layout();
    FCircuit C;
    C.Doc.HeartlineHeight = 1.1;
    for (const FItem& I : Items)
    {
        C.Doc.Segments.push_back(I.A);
    }
    C.Boundaries.push_back(0.0);

    EZone Open = EZone::None;
    double AccS = 0.0;
    double OpenS = 0.0;
    double OpenSpeed = 0.0;

    auto Close = [&](double EndS)
    {
        if (Open == EZone::None || !(EndS > OpenS))
        {
            return;
        }
        switch (Open)
        {
        case EZone::Lift:
            if (Tr) { Tr->AddZone(MakeLift(OpenS, EndS, OpenSpeed, Grip)); }
            break;
        case EZone::Launch:
            if (Tr) { Tr->AddZone(MakeLaunch(OpenS, EndS, OpenSpeed, Grip)); }
            break;
        case EZone::Brake:
            if (Tr) { Tr->AddZone(MakeBrake(OpenS, EndS, OpenSpeed, Grip)); }
            break;
        case EZone::BlockBrake:
        case EZone::Station:
            // Brakes AND drive tyres, so identical in shape to a lift. The
            // enumerators are separate for the block boundary, not for the
            // physics — and for the station that boundary is the whole point.
            // Authored as a Lift it MERGES into any lift behind it, which on the
            // reference layout is the lift hill, and a station sharing a block
            // with a lift means no train can board while another is climbing.
            if (Tr) { Tr->AddZone(MakeLift(OpenS, EndS, OpenSpeed, Grip)); }
            break;
        default:
            return;
        }
        C.Authored.push_back(OpenSpeed);

        // THE STOP MARK, and it is a PHYSICAL SWITCH rather than a sum. One per
        // zone so its index is the zone's own; a zone with no drive tyres can
        // never be commanded to creep, so its mark is simply never read.
        //
        // Train length appears here and NOT in the dispatcher, which is the whole
        // point of moving it: where you bolt a switch to the track is something an
        // installer knows at design time, and a PLC does not know at run time.
        // Clamped so a device barely longer than the train still puts the mark
        // where the whole train fits behind it.
        C.StopMarkS.push_back(std::max(OpenS + TrainLen, EndS - NoseClearance));
        if (Open == EZone::Lift || Open == EZone::BlockBrake || Open == EZone::Station)
        {
            C.HoldMidS.push_back(0.5 * (OpenS + EndS));
        }
    };

    for (std::size_t i = 0; i < Items.size(); ++i)
    {
        const double L = BuildSegment(Items[i].A).Length;
        if (!(L > 0.0))
        {
            continue;   // AddSegment refused it, so it occupies no arc length
        }
        if (Items[i].Zone != Open)
        {
            Close(AccS);
            if (AccS > C.Boundaries.back())
            {
                C.Boundaries.push_back(AccS);
            }
            Open = Items[i].Zone;
            OpenS = AccS;
            OpenSpeed = Items[i].Speed;
        }
        AccS += L;
    }
    Close(AccS);
    return C;
}

// THE DISPATCHER, in four lines, and the same four the actor runs.
//
// A holding device is CLOSED unless a permissive is live — brakes-on is the
// resting state of real ride control, and the alternative (open by default,
// closed on demand) fails open for exactly one frame every time, which is one
// frame of a train being pushed through a red.
//
// It asks about the train's CENTRE because that is what FTrain::Step tests a zone
// against, and because zones and blocks come from the same walk, so a zone never
// straddles a block boundary and "the block my centre is in" is unambiguous.
// It writes to a DRIVE, not to the track. A command is a request; how fast the
// drive gets there, and whether it manages to, is the drive's business and the
// panel's story. This is the whole of the PLC's authority over the ride.
void ServeHolds(FTrain& Tr, const FRideSignals& Sig, std::size_t Id,
                const std::vector<double>& Authored, const FTrackSensors& Marks,
                FTrackDrives& Drives)
{
    const int Z = Tr.FindHoldZoneAt(Tr.GetDistance());
    if (Z < 0)
    {
        return;   // not standing at a holding device; nothing to command
    }
    const std::size_t Zi = static_cast<std::size_t>(Z);
    if (Sig.CanRelease(Id, Tr.GetDistance()))
    {
        Drives.Command(Zi, Authored[Zi]);
        return;
    }

    // HELD, and the hardware does this in two stages rather than one glide.
    //
    // A real block brake is TWO devices sharing a stretch of track. A sensor sits
    // just before the pad; the brake trips as the train ENTERS and clamps a fin
    // under the car, stopping it as hard as it is allowed to — the limit being
    // rider comfort, not distance, because the alternative is whiplash. The train
    // therefore stops WHEREVER THAT LANDS. Only then do rubber tyres engage and
    // convey it forward into an acceptable holding position.
    //
    // Commanding a crawl speed expresses both stages in one number, because a zone
    // closes the gap to its target using its full authority: from 26 m/s the brake
    // bites at everything it has, and from rest the tyres push. So the sequence
    // falls out — hard stop, then creep to the mark, then held.
    //
    // The earlier version glided in on sqrt(2*a*d), which arrives at the mark
    // exactly and is wrong in character: it spreads the deceleration over the whole
    // approach instead of braking on entry, and it needs the pad to modulate itself
    // against a distance it has no way to know.
    //
    // The conveying stage is ALSO what keeps the train inside its own block. Brake
    // alone and it stops ~0.3 m past the zone start; in the station, whose start is
    // the circuit's seam, that leaves its back half in the LAST block, a dwelling
    // train holds two, and three trains deadlock — each denied by the tail of the
    // one in front. Real rides reposition for the same reason.
    // WHERE it parks: the train's NOSE about a metre short of the far end of the
    // block. Measured from the END and applied to the NOSE, because the thing
    // being prevented is the train protruding into the next zone — a lift, a
    // launch, open course — through a defect or a mistake. The margin is the
    // whole point of the number, so it is expressed as the margin.
    //
    // That is also why it is not the middle. Mid-device was the minimum fix for
    // the seam straddle and is arbitrary everywhere else: on a 130 m block brake
    // it parks a train 65 m in with 65 m of empty brake ahead of it, where a real
    // one holds near the exit.
    //
    // AND IT IS A SWITCH THAT SAYS SO, not a sum. The rule is "truck forward until
    // the stop mark trips", which is the whole of it — no train length, no arc
    // length, no zone extent. A PLC has none of those three and does not need
    // them: the margin was surveyed into the track when the switch was bolted down,
    // and thereafter the ride enforces it by geometry rather than by arithmetic.
    //
    // The mark trips on the NOSE, because a span covers a point the moment its
    // front reaches it. That is the same asymmetry the block counter runs on, and
    // it is the reason a stop mark measures the thing it is named after.
    //
    // A mark can be tripped by ANY train, since a switch has no idea which. It is
    // only unambiguous because the interlocking guarantees the train standing at
    // this device is the only one that can be in this block — the same mutual
    // support the sensor layer is built on. Read it for a zone whose block is not
    // yours and you are reading somebody else's train.
    //
    // The train now stops a little PAST the mark rather than on it — about 0.19 m,
    // being 1.5 m/s against 6 m/s^2 of grip. Real placement absorbs that in the
    // margin, which is what a margin is for, and the clearance stays under a metre.
    // ponytail: 1.5 m/s of crawl, a maintenance-pace guess.
    const double Convey = std::min(Authored[Zi], 1.5);
    Drives.Command(Zi, Marks.IsBlocked(Zi) ? 0.0 : Convey);
}

// The drives as the ride opens: every one already running at its authored speed,
// except the holding devices, which rest CLOSED. Preset rather than commanded,
// because a drive that ramps up from zero on the first frame of the session is a
// lift chain standing still when the first train reaches it.
FTrackDrives OpenDrives(const FTrain& Tr, const FCircuit& C)
{
    FTrackDrives D(C.Authored.size());
    for (std::size_t z = 0; z < C.Authored.size(); ++z)
    {
        const FTrackZone Zone = Tr.GetZone(z);
        const bool bHolds = Tr.FindHoldZoneAt(0.5 * (Zone.StartS + Zone.EndS)) >= 0;
        D.Preset(z, bHolds ? 0.0 : C.Authored[z]);
    }
    return D;
}

// THE OUTPUT HALF OF THE SCAN: one drive, and every train's copy of that zone
// gets its output. A real brake is ONE device acting on whatever is in it, and
// this is what makes the per-train zone lists agree about that — before drives
// existed, each train carried its own independent idea of what every brake on the
// ride was doing.
void DriveTheTrack(const FTrackDrives& D, const std::vector<FTrain*>& Trains)
{
    for (FTrain* Tr : Trains)
    {
        for (std::size_t z = 0; z < D.Num(); ++z)
        {
            Tr->SetZoneTargetSpeed(z, D.Output(z));
        }
    }
}

// And the feedback the other way: what the motor is turning at, and how much of
// its authority that is taking. A drive with no train on it is left unreported,
// which is how EndFeedback learns it is free-running rather than slipping against
// a stale reading from the last train through.
void ReadTheDrives(FTrackDrives& D, const std::vector<FTrain*>& Trains)
{
    D.BeginFeedback();
    for (const FTrain* Tr : Trains)
    {
        for (std::size_t z = 0; z < D.Num(); ++z)
        {
            if (Tr->IsInZone(z, Tr->GetDistance()))
            {
                D.ReportFeedback(z, Tr->GetSpeed(), Tr->GetZoneLoad(z));
            }
        }
    }
    D.EndFeedback();
}

// The input half of a PLC scan: every switch on the track read ONCE, before any
// logic runs. IEC 61131-3 works exactly this way — read inputs, execute program,
// write outputs — and the reason is that a program re-reading a sensor mid-scan
// acts on two different worlds in one pass.
//
// A parked train that is not being stepped still covers what it covers. A switch
// under it reads blocked, which is the physical truth and the whole point.
void ScanStopMarks(FTrackSensors& Marks, const std::vector<FTrain*>& Trains,
                   bool bCircuit, double Total)
{
    Marks.BeginScan();
    for (const FTrain* Tr : Trains)
    {
        Marks.Cover(Tr->GetRearS(), Tr->GetFrontS(), bCircuit, Total);
    }
    Marks.EndScan();
}

// Brakes on, before anyone asks. A holding device that starts at its authored
// speed is open for the frame it takes a dispatcher to notice a train arriving,
// and that frame is a train being pushed through a red signal.
void CloseAllHolds(FTrain& Tr, const std::vector<double>& Boundaries, double Total)
{
    for (std::size_t b = 0; b < Boundaries.size(); ++b)
    {
        const double End = (b + 1 < Boundaries.size()) ? Boundaries[b + 1] : Total;
        const int Z = Tr.FindHoldZoneAt(0.5 * (Boundaries[b] + End));
        if (Z >= 0)
        {
            Tr.SetZoneTargetSpeed(static_cast<std::size_t>(Z), 0.0);
        }
    }
}

double SpeedAt(const FRideProfile& P, double S)
{
    for (const FRideSample& Sm : P.Samples)
    {
        if (Sm.S >= S)
        {
            return Sm.Speed;
        }
    }
    return 0.0;
}

// ---------------------------------------------------------------- the tests

void TestRetargetingIsValidated()
{
    // A zone target reaches the energy accounting directly, so it is guarded the
    // same way AddZone guards it. A NaN here would spread to speed, then to
    // distance, then to every frame after it, and nothing downstream would say
    // where it came from.
    FTrack T;
    FTrackSegment S;
    S.Length = 100.0;
    T.AddSegment(S);
    FTrain Tr(T);
    assert(Tr.AddZone(MakeLift(0.0, 50.0, 4.0, Grip)));

    assert(Tr.SetZoneTargetSpeed(0, 0.0));
    assert(Tr.GetZoneTargetSpeed(0) == 0.0);
    assert(Tr.SetZoneTargetSpeed(0, 4.0));
    assert(Tr.GetZoneTargetSpeed(0) == 4.0);

    assert(!Tr.SetZoneTargetSpeed(0, -1.0));
    assert(!Tr.SetZoneTargetSpeed(0, std::nan("")));
    assert(!Tr.SetZoneTargetSpeed(1, 4.0));          // no such zone
    assert(Tr.GetZoneTargetSpeed(0) == 4.0);         // and none of them landed
    assert(Tr.GetZoneTargetSpeed(1) < 0.0);          // reports absence, does not read
}

void TestOnlyDriveTyresCountAsHolds()
{
    // The exclusions ARE the function. A launch cannot hold and a friction brake
    // cannot release, so neither is somewhere a dispatcher may park a train.
    FTrack T;
    FTrackSegment S;
    S.Length = 400.0;
    T.AddSegment(S);
    FTrain Tr(T);
    assert(Tr.AddZone(MakeLift(0.0, 100.0, 4.0, Grip)));      // 0: drive tyres
    assert(Tr.AddZone(MakeLaunch(100.0, 200.0, 38.0, Grip))); // 1: push only
    assert(Tr.AddZone(MakeBrake(200.0, 300.0, 6.0, Grip)));   // 2: stop only

    assert(Tr.FindHoldZoneAt(50.0) == 0);
    assert(Tr.FindHoldZoneAt(150.0) < 0);   // a gated launch is an aborted launch
    assert(Tr.FindHoldZoneAt(250.0) < 0);   // parking here is a permanent stop
    assert(Tr.FindHoldZoneAt(350.0) < 0);   // no zone at all
}

void TestTheCircuitIsTheOneTheActorBuilds()
{
    // Pins the transcription. If TwoTrainCircuitLayout is edited and this is not,
    // these are the numbers that notice — and they are the same ones quoted on the
    // preset itself.
    const FCircuit C = BuildCircuit(nullptr);
    const FTrack T = BuildTrack(C.Doc);

    assert(T.NumSegments() == 30);
    assert(std::fabs(T.TotalLength() - 1288.00) < 0.1);
    assert(T.IsCurvatureContinuous(1e-9));

    assert(C.Boundaries.size() == 8);
    assert(C.Authored.size() == 6);   // station, launch, mid-course, outer, transfer, inner

    // Eight blocks is what makes this the only preset that can hold two trains.
    // Every other one has three, because a boundary falls only where a powered run
    // starts or ends and they each have just two powered runs.
    const double Want[8] = {0.0, 26.0, 176.0, 872.06, 1002.06, 1186.02, 1223.52, 1250.52};
    for (std::size_t i = 0; i < 8; ++i)
    {
        assert(std::fabs(C.Boundaries[i] - Want[i]) < 0.05);
    }
}

void TestTheCircuitActuallyCloses()
{
    // THE THING THAT MAKES IT A CIRCUIT RATHER THAN A STRIP. The layout used to
    // close in HEIGHT only: it ended 373.794 m from the station pointing 86.421
    // degrees wrong, because the two turns summed to 446.42 degrees where a
    // circuit needs 360. The train ran off the end and was teleported back.
    //
    // The fix is a shape, not a solver. An oval closes ANALYTICALLY when both
    // turns are the same way, exactly 180 degrees, same radius, same easement:
    // the lateral displacements (+2R, -2R) cancel, the along-track ones cancel,
    // heading sums to 360, and one scalar condition remains — the return leg's
    // horizontal extent equals the two collinear outbound legs'.
    //
    // These tolerances are deliberately brutal. A seam that closes to a
    // millimetre is a seam a rider cannot see, and anything looser would let the
    // shape drift back into "nearly closed", which is what this replaced.
    const FCircuit C = BuildCircuit(nullptr);
    const FTrack T = BuildTrack(C.Doc);
    const double Total = T.TotalLength();

    const FTrackFrame Start = T.EvaluateAt(0.0);
    const FTrackFrame End = T.EvaluateAt(Total);

    const FVec3 Gap = End.Position - Start.Position;
    assert(Length(Gap) < 1e-3);                       // position, all three axes
    assert(std::fabs(End.Position.Z) < 1e-6);         // and height exactly

    const double HeadingDot = Dot(End.Tangent, Start.Tangent);
    assert(HeadingDot > 0.0);                         // same way round, not reversed
    assert(std::acos(std::min(1.0, HeadingDot)) < 1e-4);
    assert(std::fabs(End.Roll - Start.Roll) < 1e-9);  // and not rolled at the seam

    // Total turning is one lap, not two and not none. Measured by unwrapping the
    // heading, because the angle between two tangents cannot tell 360+86 from
    // 360-86 and the whole defect lived in that ambiguity.
    double Turn = 0.0;
    double Prev = std::atan2(Start.Tangent.Y, Start.Tangent.X);
    FTrackFrame W = Start;
    for (double S = 0.0; S < Total; )
    {
        const double Next = std::min(S + 0.5, Total);
        W = T.AdvanceFrom(W, S, Next);
        const double Now = std::atan2(W.Tangent.Y, W.Tangent.X);
        double D = Now - Prev;
        while (D > Pi) { D -= 2.0 * Pi; }
        while (D < -Pi) { D += 2.0 * Pi; }
        Turn += D;
        Prev = Now;
        S = Next;
    }
    assert(std::fabs(std::fabs(Turn) - 2.0 * Pi) < 1e-3);
}

void TestClearanceMustBeMeasuredTheShortWayRound()
{
    // A closed circuit's first and last samples ARE the same piece of track, so a
    // clearance check that measures separation linearly calls them TotalLength
    // apart, never skips them, and reports the closure as a 0.00 m
    // self-intersection. That is not a near miss to fix; it is the one place a
    // circuit is guaranteed to touch.
    const FCircuit C = BuildCircuit(nullptr);
    const FTrack T = BuildTrack(C.Doc);
    const FTrackProfile Cross;

    const FClearanceReport Linear = AnalyseSelfClearance(T, Cross, 0.5, 12.0, false);
    assert(Linear.ClosestApproach < 0.01);            // the seam, reported as a fault
    assert(Linear.AtS < 0.6);
    assert(Linear.AndS > T.TotalLength() - 0.6);

    const FClearanceReport Circular = AnalyseSelfClearance(T, Cross, 0.5, 12.0, true);
    assert(Circular.ClosestApproach > 10.0);          // and the truth: 11.68 m
    assert(!Circular.bStructureOverlaps);
}

void TestEveryHoldingBlockCanActuallyStopWhatArrives()
{
    // A block brake is only a block brake if it can stop the train it RECEIVES,
    // in the block it has: v^2/2a against the length. That is a layout question
    // no zone kind can answer, and on the open version of this layout the
    // mid-course brake failed it — 28.19 m/s arriving, 66.2 m needed, 45 m
    // available — so it was authored as a plain trim and recorded as one.
    //
    // Closing the circuit is what fixed it. The return leg had to grow to balance
    // the outbound legs, and that spare length went into the mid-course brake
    // rather than into dead straight: 130 m against the 58.1 m it needs. The
    // circuit gained a third queueing position on the far side from the station.
    const FCircuit Shape = BuildCircuit(nullptr);
    const FTrack T = BuildTrack(Shape.Doc);
    FTrainConfig Cfg;
    Cfg.TrainLength = TrainLen;
    FTrain Tr(T, Cfg);
    BuildCircuit(&Tr);
    const FRideProfile P = RunRideProfile(Tr, T, 1.0);
    assert(P.bCompleted);

    // Every boundary that opens a hold-capable zone, checked against the block it
    // opens. Driven off the derived boundaries rather than typed positions, so
    // moving a device cannot quietly skip the check for it.
    const double Total = T.TotalLength();
    int Holds = 0;
    for (std::size_t b = 0; b < Shape.Boundaries.size(); ++b)
    {
        const double Start = Shape.Boundaries[b];
        const double End = (b + 1 < Shape.Boundaries.size()) ? Shape.Boundaries[b + 1] : Total;
        if (Tr.FindHoldZoneAt(0.5 * (Start + End)) < 0)
        {
            continue;   // free course or the launch: nothing to hold with
        }
        ++Holds;
        const double V = SpeedAt(P, Start);
        assert(V * V / (2.0 * Grip) <= End - Start);
    }
    assert(Holds == 5);   // station, mid-course, outer, transfer, inner

    // The mid-course one specifically, because it is the one that changed.
    assert(Tr.FindHoldZoneAt(900.0) >= 0);
    const double Mid = SpeedAt(P, 872.06);
    assert(Mid > 26.0 && Mid < 27.0);
    assert(Mid * Mid / (2.0 * Grip) < 130.0);

    // And the train is not crawling over the top. A turnaround entered at walking
    // pace is a stall waiting to happen and a very dull thirty seconds; the first
    // closed layout that worked did exactly that at 2.70 m/s, which is why the
    // crest is 48.5 m and not 57.3.
    double Slowest = 1e9;
    for (const FRideSample& Sm : P.Samples)
    {
        if (Sm.S > 200.0 && Sm.S < 560.0) { Slowest = std::min(Slowest, Sm.Speed); }
    }
    assert(Slowest > 10.0);
}

void TestTwoTrainsQueueBeforeTheStation()
{
    // The whole point of the file. Train A stands at the transfer tyres, train B
    // arrives behind it at speed, and the outer block brake has to stop B, hold
    // it while A is in the way, and let it go afterwards — without either train
    // ever being granted a block the other is in.
    const FCircuit Shape = BuildCircuit(nullptr);
    const FTrack T = BuildTrack(Shape.Doc);

    FTrainConfig Cfg;
    Cfg.TrainLength = TrainLen;
    FTrain A(T, Cfg);
    FTrain B(T, Cfg);
    const FCircuit C = BuildCircuit(&A);
    BuildCircuit(&B);

    // Speeds come from the ride profile rather than being typed, so B arrives at
    // whatever the layout actually delivers.
    const FRideProfile P = RunRideProfile(A, T, 1.0);
    const double StartB = 1150.0;   // block 4, free course, closing on the outer brake

    // Lookahead ONE here, not the two the launch wants. Two is the
    // braking-distance rule for high-speed sections; in a slow pre-station queue
    // it means a train may not enter the block behind another until the one
    // beyond that is clear, and with no lap the leading train parks at the end of
    // the track and never vacates it — so the queue would be correct, permanent
    // and untestable. One is the classic block rule and the one this stretch runs.
    FRideSignals Sig(C.Boundaries, 0.0, 1, 2);
    assert(Sig.NumBlocks() == 8);
    assert(Sig.NumTrains() == 2);
    assert(Sig.Lookahead() == 1);

    CloseAllHolds(A, C.Boundaries, T.TotalLength());
    CloseAllHolds(B, C.Boundaries, T.TotalLength());

    A.Place(1237.0, 0.0);                       // standing at the transfer tyres
    B.Place(StartB, SpeedAt(P, StartB));        // still on the course, closing
    assert(Sig.Update(0, A.GetRearS(), A.GetFrontS()));
    assert(Sig.Update(1, B.GetRearS(), B.GetFrontS()));
    assert(Sig.OccupiedBy(0, 6));
    assert(Sig.OccupiedBy(1, 4));

    const double Dt = 1.0 / 240.0;
    int HeldFrames = 0;
    double SlowestB = 1e9;
    bool bBStoppedInOuter = false;
    bool bEverShared = false;

    // A is PARKED for the first half — not served, not stepped — so it is simply
    // an obstacle standing in block 6. Deterministic on purpose: with both trains
    // running, whether B finishes being trucked to its mark before A vacates is a
    // race, and the thing under test is the brake, not the timing.
    const int Release = 240 * 45;

    FTrackSensors Marks(C.StopMarkS);
    FTrackDrives Drives = OpenDrives(A, C);
    const std::vector<FTrain*> Both = {&A, &B};

    for (int Frame = 0; Frame < 240 * 120; ++Frame)
    {
        ScanStopMarks(Marks, Both, false, T.TotalLength());
        if (Frame >= Release)
        {
            ServeHolds(A, Sig, 0, C.Authored, Marks, Drives);
        }
        ServeHolds(B, Sig, 1, C.Authored, Marks, Drives);
        Drives.Tick(Dt);
        DriveTheTrack(Drives, Both);

        if (Frame >= Release)
        {
            A.Step(Dt);
        }
        B.Step(Dt);

        // Both trains, then ONE tick. Overlaps live on blocks, not on trains.
        assert(Sig.Update(0, A.GetRearS(), A.GetFrontS()));
        assert(Sig.Update(1, B.GetRearS(), B.GetFrontS()));
        Sig.Tick(Dt);
        ReadTheDrives(Drives, Both);

        for (std::size_t b = 0; b < Sig.NumBlocks(); ++b)
        {
            if (Sig.OccupiedBy(0, b) && Sig.OccupiedBy(1, b))
            {
                bEverShared = true;
            }
        }

        const std::size_t BlockB = Sig.BlockAt(B.GetDistance());
        if (BlockB == 5)
        {
            SlowestB = std::min(SlowestB, B.GetSpeed());
            if (B.GetSpeed() < 1e-6)
            {
                bBStoppedInOuter = true;
                // Stopped AND red. Counting "stopped" alone would also count a
                // train that simply ran out of energy there, which is a stall and
                // not an interlock.
                if (!Sig.CanRelease(1, B.GetDistance()))
                {
                    ++HeldFrames;
                }
            }
        }
    }

    // 1. The block brake really stopped a moving train, in the block, on its own.
    assert(bBStoppedInOuter);
    assert(SlowestB < 1e-6);

    // 2. It held it there, for a long time, and only while A was in the way. Not
    //    a frame or two of rounding — seconds.
    assert(HeldFrames > 240);

    // 3. B did not stay stopped once A moved on: it advanced into the block A
    //    vacated. Without SetZoneTargetSpeed this is the assertion that could not
    //    pass — a friction brake at zero never lets go.
    assert(Sig.BlockAt(B.GetDistance()) > 5);
    assert(B.GetDistance() > 1223.52);

    // 4. A ran on ahead and B never caught it.
    assert(A.GetDistance() > B.GetDistance());
    assert(Sig.BlockAt(A.GetDistance()) == 7);

    // 5. And nothing anywhere was granted into an occupied block. This is the
    //    assertion the old single-train FRideSignals could not have failed,
    //    because it reported zero violations while two trains sat in one block.
    assert(!bEverShared);
    assert(Sig.Violations() == 0);
}

// Which blocks contain a device that can stop a train AND let it go again. This
// is what turns the fixed lookahead into a real braking-distance rule: a train
// let into a block with nothing in it is committed until the next one that can
// hold it.
std::vector<bool> HoldingBlocks(const FTrain& Tr, const FCircuit& C, double Total)
{
    std::vector<bool> Out(C.Boundaries.size(), false);
    for (std::size_t b = 0; b < C.Boundaries.size(); ++b)
    {
        const double End = (b + 1 < C.Boundaries.size()) ? C.Boundaries[b + 1] : Total;
        Out[b] = Tr.FindHoldZoneAt(0.5 * (C.Boundaries[b] + End)) >= 0;
    }
    return Out;
}

struct FRunResult
{
    std::vector<int> Laps;
    std::size_t Violations = 0;
    bool bShared = false;
    int SeamFrames = 0;
    double ClosestToStationStart = 1e9;   // how far a dwelling train parks past the seam

    // Per zone, the furthest a STOPPED train's nose reached in it — where the stop
    // mark actually put it, as opposed to where the mark is. The gap between the
    // two is the crawl overshoot, and what is left of the clearance after it is
    // the safety margin the ride really achieves. SIGNALLING.md quotes this.
    std::vector<double> ParkedNoseS;

    // Per drive, the most torque it was ever asked for, and whether any drive
    // tripped. A ride that never faults a drive over a full run is the claim worth
    // asserting: the fault condition has to be quiet when nothing is wrong, or it
    // is noise rather than a diagnostic.
    std::vector<double> PeakLoad;
    bool bDriveFaulted = false;
    int FirstFaultedDrive = -1;   // which one, so a fault is a place to go and look
};

// The actor's tick, N trains, for Seconds of ride. One place, because the only
// way to trust a capacity number is for the capacity test and the two-train test
// to be running the SAME policy.
FRunResult RunTrains(std::size_t N, std::size_t Lookahead, double Seconds)
{
    const FCircuit Shape = BuildCircuit(nullptr);
    const FTrack T = BuildTrack(Shape.Doc);
    FTrainConfig Cfg;
    Cfg.TrainLength = TrainLen;

    std::vector<std::unique_ptr<FTrain>> Owned;
    FCircuit C;
    for (std::size_t t = 0; t < N; ++t)
    {
        Owned.push_back(std::unique_ptr<FTrain>(new FTrain(T, Cfg)));
        C = BuildCircuit(Owned.back().get());
    }

    FRideSignals Sig(C.Boundaries, 5.0, Lookahead, N, true);
    Sig.SetHoldingBlocks(HoldingBlocks(*Owned[0], C, T.TotalLength()));

    for (std::size_t t = 0; t < N; ++t)
    {
        CloseAllHolds(*Owned[t], C.Boundaries, T.TotalLength());
        Owned[t]->SetCircuit(true);
        Owned[t]->Place(C.HoldMidS[t], 0.0);
        Sig.Update(t, Owned[t]->GetRearS(), Owned[t]->GetFrontS());
    }

    FRunResult R;
    R.Laps.assign(N, 0);
    R.ParkedNoseS.assign(C.Authored.size(), 0.0);
    R.PeakLoad.assign(C.Authored.size(), 0.0);
    std::vector<double> Prev(N);
    for (std::size_t t = 0; t < N; ++t) { Prev[t] = Owned[t]->GetDistance(); }

    FTrackSensors Marks(C.StopMarkS);
    FTrackDrives Drives = OpenDrives(*Owned[0], C);
    std::vector<FTrain*> All;
    for (std::size_t t = 0; t < N; ++t) { All.push_back(Owned[t].get()); }

    const double Dt = 1.0 / 240.0;
    for (int F = 0; F < static_cast<int>(240.0 * Seconds); ++F)
    {
        ScanStopMarks(Marks, All, true, T.TotalLength());
        for (std::size_t t = 0; t < N; ++t)
        {
            ServeHolds(*Owned[t], Sig, t, C.Authored, Marks, Drives);
        }
        Drives.Tick(Dt);
        DriveTheTrack(Drives, All);

        for (std::size_t t = 0; t < N; ++t)
        {
            Owned[t]->Step(Dt);
            Sig.Update(t, Owned[t]->GetRearS(), Owned[t]->GetFrontS());

            const double Now = Owned[t]->GetDistance();
            if (Now + 100.0 < Prev[t]) { ++R.Laps[t]; }
            Prev[t] = Now;
            if (Owned[t]->GetFrontS() < Owned[t]->GetRearS()) { ++R.SeamFrames; }

            // Where a train actually comes to rest in the station: the number that
            // decides whether a dwelling train costs one block or two.
            if (Owned[t]->GetSpeed() <= 0.0 && Sig.BlockAt(Now) == 0)
            {
                R.ClosestToStationStart = std::min(R.ClosestToStationStart, Now);
            }
            if (Owned[t]->GetSpeed() <= 0.0)
            {
                const int Zp = Owned[t]->FindHoldZoneAt(Now);
                if (Zp >= 0)
                {
                    double& Rec = R.ParkedNoseS[static_cast<std::size_t>(Zp)];
                    Rec = std::max(Rec, Owned[t]->GetFrontS());
                }
            }
        }
        Sig.Tick(Dt);
        ReadTheDrives(Drives, All);
        for (std::size_t a = 0; a < N; ++a)
        {
            for (std::size_t b = a + 1; b < N; ++b)
            {
                for (std::size_t k = 0; k < Sig.NumBlocks(); ++k)
                {
                    if (Sig.OccupiedBy(a, k) && Sig.OccupiedBy(b, k)) { R.bShared = true; }
                }
            }
        }
        for (std::size_t z = 0; z < Drives.Num(); ++z)
        {
            R.PeakLoad[z] = std::max(R.PeakLoad[z], Drives.Read(z).Load);
            if (Drives.IsFaulted(z))
            {
                if (!R.bDriveFaulted) { R.FirstFaultedDrive = static_cast<int>(z); }
                R.bDriveFaulted = true;
            }
        }
    }
    R.Violations = Sig.Violations();
    return R;
}

void TestTheCatchHoldsAFailedLaunchOnTheRealLayout()
{
    // The preset fits anti-rollback over the launch and the whole climb, which is
    // where a real launched coaster puts it. It does NOTHING while the ride works
    // — the train crests at 12.14 m/s with margin — so this has to detune the
    // launch to make the device matter at all. That property is the point: a
    // safety device that changes the ride when the ride is fine is not a safety
    // device, it is a bug.
    const FCircuit Shape = BuildCircuit(nullptr);
    const FTrack T = BuildTrack(Shape.Doc);

    FTrainConfig Cfg;
    Cfg.TrainLength = TrainLen;
    Cfg.bAllowRollback = true;      // the honest physics, so there is something to catch

    // Launch and climb, matching the preset: station 26, launch 150, and the pull
    // up and climb that follow.
    const double CatchEnd = 400.0;

    FTrain Caught(T, Cfg);
    BuildCircuit(&Caught);
    assert(Caught.AddAntiRollback(26.0, CatchEnd));
    // Half the launch speed: nowhere near enough to reach the turnaround.
    Caught.Place(30.0, 19.0);

    FTrain Loose(T, Cfg);
    BuildCircuit(&Loose);
    Loose.Place(30.0, 19.0);

    // Both zones are live, so the launch will push. Shut them, because what is
    // under test is what happens when the train runs out on the climb.
    CloseAllHolds(Caught, Shape.Boundaries, T.TotalLength());
    CloseAllHolds(Loose, Shape.Boundaries, T.TotalLength());
    for (std::size_t z = 0; z < Shape.Authored.size(); ++z)
    {
        Caught.SetZoneTargetSpeed(z, 0.0);
        Loose.SetZoneTargetSpeed(z, 0.0);
    }

    for (int i = 0; i < 240 * 120; ++i)
    {
        Caught.Step(1.0 / 240.0);
        Loose.Step(1.0 / 240.0);
    }

    // The caught train is HELD on the climb, above where it started, stopped.
    assert(Caught.GetRollbacksCaught() >= 1);
    assert(Caught.IsHeldByCatch());
    assert(Caught.GetSpeed() < 1e-6);
    assert(Caught.GetDistance() > 30.0);
    assert(Caught.GetDistance() < CatchEnd);

    // The loose one came back down past where it began — into the station, at
    // speed, backwards, which is the outcome the device exists to prevent.
    assert(Loose.GetRollbacksCaught() == 0);
    assert(Loose.GetDistance() < Caught.GetDistance());
}

void TestAHeldTrainParksInsideItsBlock()
{
    // Commanded to plain zero, a holding device stops the train within ~0.3 m of
    // where the ZONE starts, because a zone says "reach this speed" and zero is
    // reachable at once. In the station, whose start IS the seam, that leaves the
    // back half of the train in the LAST block — so a dwelling train holds two
    // blocks. Measured consequence: three trains DEADLOCK, each denied by the tail
    // of the one in front, with no violation and nothing moving.
    //
    // The dispatcher therefore trucks the train forward at a crawl until A SWITCH
    // SAYS FAR ENOUGH. No new authored concept: SetZoneTargetSpeed was already
    // being commanded every frame, and the mark is a position on the track rather
    // than a number in the program.
    const FRunResult R = RunTrains(2, 1, 240.0);
    assert(R.ClosestToStationStart < 1e8);          // somebody did dwell there

    // THE MARGIN, MEASURED, because it is the safety number this whole mechanism
    // exists to produce. The station block is 0..26 and its mark is at 25, so a
    // parked nose must be past the switch and still short of the block end. The
    // gap between those two is the clearance, and the overshoot from crawl speed
    // is what eats into it — assert it rather than trust it.
    const double ParkedNose = R.ClosestToStationStart + TrainLen * 0.5;
    assert(ParkedNose >= 25.0);                     // it went all the way to the mark
    assert(ParkedNose < 26.0);                      // and never nosed over into the launch

    // The mark is the NOSE a metre short of the block end: on the 26 m station
    // that centres a 15 m train at 17.5, spanning 10.0..25.0 — wholly inside its
    // own block, with nothing hanging back over the seam and nothing nosing over
    // into the launch.
    assert(R.ClosestToStationStart > TrainLen * 0.5);
    assert(R.ClosestToStationStart < 26.0 - TrainLen * 0.5);
    assert(R.ClosestToStationStart > 26.0 - TrainLen * 0.5 - 2.0);   // near the far end
}

void TestTheDrivesTellTheStoryOfTheRide()
{
    // THE OUTPUT SIDE, on the real circuit. A drive's three numbers — commanded,
    // output, actual — plus its torque are what a control panel is a picture of,
    // and this asserts they behave over a full session rather than in isolation.
    const FRunResult R = RunTrains(4, 1, 420.0);

    // 1. NOT ONE DRIVE FAULTED. This is the claim worth having: a diagnostic that
    //    fires while the ride is working correctly is noise, not a diagnostic. It
    //    is also the assertion that caught the launch — slip, torque and time alone
    //    reported a healthy 0-to-38 launch as a failure, because a launch IS
    //    sustained slip at full torque. The rule needs "and not gaining".
    // Named on the way out, on stderr so it survives the abort. A fault that says
    // only "something tripped" is a fault nobody can act on, and that is as true
    // of this assertion as it is of a control panel.
    if (R.bDriveFaulted)
    {
        std::fprintf(stderr, "drive %d faulted\n", R.FirstFaultedDrive);
    }
    assert(!R.bDriveFaulted);

    // 2. Every drive on the ride was worked to its limit at some point. A block
    //    brake stopping a train from speed, a launch, a chain starting a train —
    //    all of them saturate, and a drive that never does is oversized for its
    //    job, which is a thing a panel would show an engineer.
    for (std::size_t z = 0; z < R.PeakLoad.size(); ++z)
    {
        assert(R.PeakLoad[z] > 0.99);
    }
}

void TestTheCircuitCarriesFourTrains()
{
    // CAPACITY, measured rather than assumed. The old fixed-lookahead permissive
    // let four trains collide — 14 violations at lookahead 1, 18 at lookahead 2 —
    // because a train was granted a block with no device in it and found the one
    // beyond it occupied on arrival. A count of blocks cannot express "far enough
    // to stop" when one free run is 696 m and the next is 184.
    //
    // With the holding list supplied, the permissive clears all the way to the
    // next block that can hold the train, and all four run.
    for (std::size_t N = 1; N <= 4; ++N)
    {
        const FRunResult R = RunTrains(N, 1, 420.0);
        assert(R.Violations == 0);
        assert(!R.bShared);
        for (std::size_t t = 0; t < N; ++t)
        {
            assert(R.Laps[t] >= 1);   // nobody starved, nobody deadlocked
        }
    }

    // AND FOUR IS THE CEILING, for a reason that is about the layout rather than
    // the signalling: the circuit has FIVE places a train can stand, and one has
    // to stay free or every train is parked where the train behind it needs to go.
    // Five trains never move at all — no violation, no deadlock in the code, just
    // a ride that is full. The actor caps at holding places minus one because of
    // this, so it is a log line rather than a puzzle.
    const FRunResult Full = RunTrains(5, 1, 420.0);
    int Moving = 0;
    for (std::size_t t = 0; t < 5; ++t)
    {
        if (Full.Laps[t] > 0) { ++Moving; }
    }
    assert(Moving == 0);
}

void TestTheActorsOwnLoopRunsTwoTrains()
{
    // A LINE-FOR-LINE STAND-IN FOR ATUCoasterRide::Tick, because that function
    // cannot be compiled without Unreal and this is the only place its policy can
    // be checked at all. Same order, same defaults:
    //
    //   scan the stop marks once
    //   for each train: ServeHolds, Step, Signals->Update
    //   Signals->Tick once
    //
    // and NO teleport, because the layout closes. The trains drive round, through
    // the seam, into the station, under their own power. The actor makes exactly
    // that choice the same way: it measures the seam and only wraps if it meets.
    //
    // What it is really looking for is DEADLOCK. Every individual rule here is
    // "fail closed", and a circuit of fail-closed rules is exactly the shape that
    // stops for ever with nothing reporting anything wrong.
    const FCircuit Shape = BuildCircuit(nullptr);
    const FTrack T = BuildTrack(Shape.Doc);

    FTrainConfig Cfg;
    Cfg.TrainLength = TrainLen;
    FTrain A(T, Cfg);
    FTrain B(T, Cfg);
    const FCircuit C = BuildCircuit(&A);
    BuildCircuit(&B);
    FTrain* Trains[2] = {&A, &B};

    // Five places a train may stand: station, mid-course, outer, transfer, inner.
    // The mid-course one is new, and it is there because closing the circuit made
    // the return leg long enough for a brake that can stop what arrives.
    assert(C.HoldMidS.size() == 5);
    // The station's MIDDLE, not its start. Placed at the start, a 15 m train hangs
    // back over the seam into the last block and collides with anything standing
    // there — which is exactly how the fifth train used to arrive already in
    // violation, before it had moved a metre.
    assert(std::fabs(C.HoldMidS[0] - 13.0) < 1e-9);

    // The actor's defaults: 5 s overlap, lookahead 2. Circuit mode on both layers,
    // which the actor sets from the same measurement TestTheCircuitActuallyCloses
    // makes.
    FRideSignals Sig(C.Boundaries, 5.0, 2, 2, true);
    assert(Sig.Lookahead() == 2);

    CloseAllHolds(A, C.Boundaries, T.TotalLength());
    CloseAllHolds(B, C.Boundaries, T.TotalLength());
    A.SetCircuit(true);
    B.SetCircuit(true);
    assert(!A.IsAtEnd());   // a circuit has no end to be at, ever

    // Train 0 in the station; the rest at the holding places in track order.
    A.Place(C.HoldMidS[0], 0.0);
    B.Place(C.HoldMidS[1], 0.0);
    Sig.Update(0, A.GetRearS(), A.GetFrontS());
    Sig.Update(1, B.GetRearS(), B.GetFrontS());

    const double Dt = 1.0 / 240.0;
    int Laps[2] = {0, 0};
    int SeamStraddles[2] = {0, 0};
    double Prev[2] = {A.GetDistance(), B.GetDistance()};
    bool bShared = false;

    FTrackSensors Marks(C.StopMarkS);
    FTrackDrives Drives = OpenDrives(A, C);
    const std::vector<FTrain*> All = {&A, &B};

    for (int Frame = 0; Frame < 240 * 600; ++Frame)
    {
        // THE SCAN CYCLE, and it is the whole shape of the tick: read the inputs,
        // run the program for every train against that one snapshot, let the drives
        // ramp, write the outputs, then step the world. Interleaving the program
        // with the physics — serving train 1 after train 0 has already moved — is
        // what a game does and what a PLC cannot.
        ScanStopMarks(Marks, All, true, T.TotalLength());
        for (std::size_t t = 0; t < 2; ++t)
        {
            ServeHolds(*Trains[t], Sig, t, C.Authored, Marks, Drives);
        }
        Drives.Tick(Dt);
        DriveTheTrack(Drives, All);

        for (std::size_t t = 0; t < 2; ++t)
        {
            Trains[t]->Step(Dt);
            Sig.Update(t, Trains[t]->GetRearS(), Trains[t]->GetFrontS());

            // A lap is the distance going backwards, which on a wrapping train is
            // the only thing that can make it do so.
            const double Now = Trains[t]->GetDistance();
            if (Now + 100.0 < Prev[t]) { ++Laps[t]; }
            Prev[t] = Now;

            // And the train really does span the seam on the way through, rather
            // than stepping over it in one frame and never being seen there.
            if (Trains[t]->GetFrontS() < Trains[t]->GetRearS()) { ++SeamStraddles[t]; }
        }
        Sig.Tick(Dt);
        ReadTheDrives(Drives, All);

        for (std::size_t b = 0; b < Sig.NumBlocks(); ++b)
        {
            if (Sig.OccupiedBy(0, b) && Sig.OccupiedBy(1, b))
            {
                bShared = true;
            }
        }
    }

    // Ten minutes of ride. BOTH trains keep going round — the deadlock this is
    // looking for would show up as one stuck at zero while the other cycles,
    // which is what a fail-closed rule set does when two trains each hold what
    // the other is waiting for.
    assert(Laps[0] >= 3);
    assert(Laps[1] >= 3);

    // Each lap crosses the seam with the train ACROSS it for a while — 15 m at
    // station speed is a good few frames — and that is the state that used to be
    // unrepresentable. If this is zero, the trains are jumping the seam in one
    // step and the wrapped-range handling is never being exercised.
    assert(SeamStraddles[0] > 10);
    assert(SeamStraddles[1] > 10);

    // And the interlocking held throughout: never two trains in one block, never
    // a permissive granted into an occupied one.
    assert(!bShared);
    assert(Sig.Violations() == 0);
}

} // namespace

int main()
{
    TestRetargetingIsValidated();
    TestOnlyDriveTyresCountAsHolds();
    TestTheCircuitIsTheOneTheActorBuilds();
    TestTheCircuitActuallyCloses();
    TestClearanceMustBeMeasuredTheShortWayRound();
    TestEveryHoldingBlockCanActuallyStopWhatArrives();
    TestTwoTrainsQueueBeforeTheStation();
    TestTheCatchHoldsAFailedLaunchOnTheRealLayout();
    TestAHeldTrainParksInsideItsBlock();
    TestTheDrivesTellTheStoryOfTheRide();
    TestTheCircuitCarriesFourTrains();
    TestTheActorsOwnLoopRunsTwoTrains();

    // The canonical figures, printed rather than only asserted, because the docs
    // quote them and CLAUDE.md's rule is that a number should come from running
    // something. Same job reference_figures.cpp does for the reference layout.
    {
        const FCircuit C = BuildCircuit(nullptr);
        const FTrack T = BuildTrack(C.Doc);
        const double Total = T.TotalLength();
        FTrainConfig Cfg;
        Cfg.TrainLength = TrainLen;
        FTrain Tr(T, Cfg);
        BuildCircuit(&Tr);
        const FRideProfile P = RunRideProfile(Tr, T, 1.0);
        const FTrackProfile Cross;
        const FClearanceReport Cl = AnalyseSelfClearance(T, Cross, 0.5, 12.0, true);
        const FTrackFrame A = T.EvaluateAt(0.0);
        const FTrackFrame B = T.EvaluateAt(Total);

        double Crest = 0.0;
        for (const FRideSample& S : P.Samples) { Crest = std::max(Crest, S.Height); }

        std::printf("\nTWO-TRAIN CIRCUIT, measured from the layout above\n");
        std::printf("  %zu segments, %.2f m, C2 %s\n", T.NumSegments(), Total,
                    T.IsCurvatureContinuous(1e-9) ? "yes" : "NO");
        std::printf("  seam %.6f m, %.6f deg heading, %.6f deg roll\n",
                    Length(B.Position - A.Position),
                    std::acos(std::min(1.0, Dot(B.Tangent, A.Tangent))) * 180.0 / Pi,
                    std::fabs(B.Roll - A.Roll) * 180.0 / Pi);
        std::printf("  top %.1f km/h   vertical %+.2f .. %+.2f   lateral %.2f\n",
                    P.TopSpeed * 3.6, P.MinVerticalG, P.MaxVerticalG, P.MaxAbsLateralG);
        std::printf("  crest %.1f m   clearance %.2f m   peak roll rate %.1f deg/s   %.0f s\n",
                    Crest, Cl.ClosestApproach, P.MaxAbsRollRate, P.Duration);
        std::printf("  %zu blocks, %zu of them able to hold a train, so %zu trains\n",
                    C.Boundaries.size(), C.HoldMidS.size(), C.HoldMidS.size() - 1);

        // The safety margin, as the ride actually achieves it rather than as the
        // constant asks for it: the mark is surveyed at 1.0 m, and what is left
        // after the train overshoots at crawl speed is the number that matters.
        FRunResult R = RunTrains(4, 1, 1200.0);
        {
            // Merged with a heavier-headway run, because the shipping configuration
            // never actually HOLDS a train at the mid-course brake — four trains at
            // lookahead 1 flow straight through it. A device the traffic never uses
            // is still a device, and its mark is still worth measuring.
            const FRunResult H = RunTrains(4, 2, 1200.0);
            for (std::size_t z = 0; z < R.ParkedNoseS.size(); ++z)
            {
                R.ParkedNoseS[z] = std::max(R.ParkedNoseS[z], H.ParkedNoseS[z]);
            }
        }
        std::printf("  stop marks: zone  mark    parked nose   clear of block end\n");
        for (std::size_t z = 0; z < R.ParkedNoseS.size(); ++z)
        {
            if (R.ParkedNoseS[z] <= 0.0)
            {
                continue;   // no device, or nothing ever stood at it
            }
            // The mark is 1.0 m short of the device end, so the end is mark + 1.
            const double BlockEnd = C.StopMarkS[z] + NoseClearance;
            std::printf("              %2zu  %7.2f  %11.2f  %10.2f\n",
                        z, C.StopMarkS[z], R.ParkedNoseS[z], BlockEnd - R.ParkedNoseS[z]);
        }
    }

    std::printf("\ntest_twotrains: all assertions passed\n");
    return 0;
}
