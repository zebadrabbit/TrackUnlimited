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
#include "../BlockSignal/Evacuation.h"
#include "../BlockSignal/Scenario.h"
#include "../BlockSignal/ShowBus.h"
#include "../BlockSignal/SimDigest.h"
#include "../BlockSignal/StationProcess.h"
#include "../BlockSignal/TrackDrives.h"
#include "../BlockSignal/TrackSensors.h"
#include "../TrackSpline/TrackIO.h"
#include "../TrackSpline/TrackProfile.h"
#include "../TrackSpline/TrackValidate.h"
#include "RideProfile.h"
#include "TrainPhysics.h"

#include <cassert>
#include <cmath>
#include <cstdint>
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
enum class EZone { None, Lift, Launch, Brake, BlockBrake, Station, StationUnload, StationLoad };

struct FItem
{
    FAuthoredSegment A;
    EZone Zone = EZone::None;
    double Speed = 0.0;

    // "A NEW DEVICE STARTS HERE", even though the kind and the speed are the same
    // as what came before. The last thing needed to author several IDENTICAL
    // devices in a row — three load positions on one platform, a queue of brake
    // sections — where kind and speed cannot tell them apart because there is
    // genuinely nothing different about them except that they are separate
    // machines with separate motors.
    bool bNewDevice = false;

    // THE DEVICE'S OWN RATES, negative meaning "the default", because this
    // transcription silently ran every device at the one Grip constant and that
    // cost a real diagnosis: the actor's showcase violated with six trains while
    // this harness ran six and seven clean, and the difference was exactly the
    // rates this struct could not carry. A harness whose devices are all the
    // same machine measures a different ride.
    double Accel = -1.0;      // tyre/chain/launch drive rate, m/s^2
    double Decel = -1.0;      // tyre braking rate, m/s^2
    double PadDecel = -1.0;   // friction pad bite; < 0 is "no pad beyond the kind's own"
};

void AddStraight(std::vector<FItem>& O, double L, EZone Z = EZone::None, double Sp = 0.0,
                 bool bNewDevice = false,
                 double Accel = -1.0, double Decel = -1.0, double PadDecel = -1.0)
{
    FItem I;
    I.A.Kind = ESegmentKind::Straight;
    I.A.Length = L;
    I.Zone = Z;
    I.Speed = Sp;
    I.bNewDevice = bNewDevice;
    I.Accel = Accel;
    I.Decel = Decel;
    I.PadDecel = PadDecel;
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

// THE SMALL-BATCH CIRCUIT, and it is the two-train oval with a different front
// end. Deliberately: that shape closes to 0.000000 m because of its LEG LENGTHS,
// and leg A is 176 m of flat whether that is one 26 m station plus a 150 m launch
// or four short platforms plus a 136 m one. Keep the total and the closure comes
// along unchanged, along with every G figure that was measured against it.
//
// What changes is the operation. Small vehicles — 6 m rather than 15 — so the same
// 40 m of platform holds an unload and THREE loading positions instead of one
// station, and the ride runs a queue of trains through them.
const double BatchTrainLen = 6.0;

std::vector<FItem> SmallBatchCircuitLayout()
{
    const double Up = Deg(26.0);
    const double Dn = Deg(32.0);
    const double R = 35.0;
    const double Ease = 50.0;
    const double Arc = Pi * R - Ease;
    const double DropLen = 15.6847323;
    const double FillLen = 75.5024975;

    std::vector<FItem> Out;
    // LEG A. 40 m of platform and a 136 m launch: 176 m, exactly as before.
    AddStraight(Out, 10.0, EZone::StationUnload, 1.5);          // riders off
    AddStraight(Out, 10.0, EZone::StationLoad, 1.5);            // position 3, rear
    AddStraight(Out, 10.0, EZone::StationLoad, 1.5, true);      // position 2
    AddStraight(Out, 10.0, EZone::StationLoad, 1.5, true);      // position 1, front
    AddStraight(Out, 136.0, EZone::Launch, 38.0);
    AddEasedPitch(Out, Up, 0.0130);
    AddStraight(Out, 40.0);
    AddEasedPitch(Out, -Up, 0.0130);
    AddBankedTurn(Out, R, Arc, Ease, BankDegreesFor(14.2, R));
    // LEG B, unchanged.
    AddEasedPitch(Out, -Dn, 0.0150);
    AddStraight(Out, DropLen);
    AddEasedPitch(Out, Dn, 0.0150);
    AddEasedPitch(Out, Deg(20.0), 0.024);
    AddEasedPitch(Out, Deg(-40.0), 0.024);
    AddEasedPitch(Out, Deg(20.0), 0.024);
    AddStraight(Out, FillLen);
    AddStraight(Out, 130.0, EZone::BlockBrake, 20.0);
    AddBankedTurn(Out, R, Arc, Ease, BankDegreesFor(18.1, R));
    // LEG C, unchanged.
    AddStraight(Out, 24.0);
    AddStraight(Out, 37.5, EZone::BlockBrake, 6.0);
    AddStraight(Out, 27.0, EZone::Lift, 4.0);
    AddStraight(Out, 37.5, EZone::BlockBrake, 2.0);
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
    std::vector<EZone> Kinds;        // per zone, what it was authored as
    std::vector<double> ZoneStartS;  // per zone, where it begins — a block boundary
};

// One platform position, with everything it needs to run its own sequence. A
// three-position load platform would be three of these; this circuit has one.
struct FPlatform
{
    std::size_t Zone = 0;
    FStationProcess Process{EStationRole::Combined};
    FAutoStationCrew Crew;
    FStationInputs Inputs;
};

// Tr is optional so the shape can be asked for without a train to hang zones on
// — FTrain wants a built track in its constructor, and the track comes out of
// this same walk.
// TrainLenM is what the stop marks are surveyed against, so it has to be the
// length of the train that will actually stand there. Defaulted for the two-train
// circuit and passed explicitly by anything running shorter vehicles — getting it
// wrong puts the mark BEYOND the device, where no train ever trips it, and the
// train crawls straight out of its block into the next one.
FCircuit BuildCircuitFrom(FTrain* Tr, const std::vector<FItem>& Items,
                          double TrainLenM = TrainLen)
{
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
    // The open device's own rates, carried from its FItem. Negative is "the
    // default", which keeps every existing layout in this file bit-identical —
    // the same rule the actor's segment struct applies to the same three fields.
    double OpenAccel = -1.0;
    double OpenDecel = -1.0;
    double OpenPad = -1.0;

    auto Close = [&](double EndS)
    {
        if (Open == EZone::None || !(EndS > OpenS))
        {
            return;
        }
        const double UpRate = OpenAccel > 0.0 ? OpenAccel : Grip;
        const double DownRate = OpenDecel > 0.0 ? OpenDecel : Grip;
        switch (Open)
        {
        case EZone::Lift:
            if (Tr)
            {
                FTrackZone Z = MakeLift(OpenS, EndS, OpenSpeed, Grip);
                Z.MaxAccel = UpRate;
                Z.MaxDecel = DownRate;
                Tr->AddZone(Z);
            }
            break;
        case EZone::Launch:
            if (Tr) { Tr->AddZone(MakeLaunch(OpenS, EndS, OpenSpeed, UpRate)); }
            break;
        case EZone::Brake:
            if (Tr) { Tr->AddZone(MakeBrake(OpenS, EndS, OpenSpeed,
                                            OpenPad > 0.0 ? OpenPad : Grip)); }
            break;
        case EZone::BlockBrake:
        case EZone::Station:
        case EZone::StationUnload:
        case EZone::StationLoad:
            // Brakes AND drive tyres, so identical in shape to a lift. The
            // enumerators are separate for the block boundary, not for the
            // physics — and for the station that boundary is the whole point.
            // Authored as a Lift it MERGES into any lift behind it, which on the
            // reference layout is the lift hill, and a station sharing a block
            // with a lift means no train can board while another is climbing.
            //
            // A PAD rides on top when one is authored: the two-machine block
            // brake, with the pad tracking the same commanded speed the tyres
            // drive toward — the actor's rule, transcribed rather than invented.
            if (Tr)
            {
                FTrackZone Z = OpenPad > 0.0
                    ? MakeBlockBrake(OpenS, EndS, OpenSpeed, Grip, OpenSpeed, OpenPad)
                    : MakeLift(OpenS, EndS, OpenSpeed, Grip);
                Z.MaxAccel = UpRate;
                Z.MaxDecel = DownRate;
                Tr->AddZone(Z);
            }
            break;
        default:
            return;
        }
        C.Authored.push_back(OpenSpeed);
        C.Kinds.push_back(Open);
        C.ZoneStartS.push_back(OpenS);

        // THE STOP MARK, and it is a PHYSICAL SWITCH rather than a sum. One per
        // zone so its index is the zone's own; a zone with no drive tyres can
        // never be commanded to creep, so its mark is simply never read.
        //
        // Train length appears here and NOT in the dispatcher, which is the whole
        // point of moving it: where you bolt a switch to the track is something an
        // installer knows at design time, and a PLC does not know at run time.
        // Clamped so a device barely longer than the train still puts the mark
        // where the whole train fits behind it.
        C.StopMarkS.push_back(std::max(OpenS + TrainLenM, EndS - NoseClearance));
        if (Open == EZone::Lift || Open == EZone::BlockBrake || Open == EZone::Station
            || Open == EZone::StationUnload || Open == EZone::StationLoad)
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
        // A RUN ENDS WHERE THE DEVICE CHANGES, and the device is its kind AND its
        // speed. Kind alone was the old rule, and it meant a brake at 6 m/s
        // followed immediately by one at 2 m/s became ONE zone targeting 6, with
        // the 2 discarded — a typed number thrown away, which this project treats
        // as a defect everywhere else. Two speeds is two devices, so two zones and
        // two blocks. It is also what lets several holding devices be authored in
        // a row at all.
        const bool bKindChanged = Items[i].Zone != Open;
        const bool bSpeedChanged = Open != EZone::None
            && std::fabs(Items[i].Speed - OpenSpeed) > 1e-9;
        // And the author saying so outright, for devices that are identical in
        // every respect the walk can see and are still separate machines.
        const bool bDeclared = Items[i].bNewDevice && Items[i].Zone != EZone::None;
        if (bKindChanged || bSpeedChanged || bDeclared)
        {
            Close(AccS);
            if (AccS > C.Boundaries.back())
            {
                C.Boundaries.push_back(AccS);
            }
            Open = Items[i].Zone;
            OpenS = AccS;
            OpenSpeed = Items[i].Speed;
            OpenAccel = Items[i].Accel;
            OpenDecel = Items[i].Decel;
            OpenPad = Items[i].PadDecel;
        }
        AccS += L;
    }
    Close(AccS);
    return C;
}

FCircuit BuildCircuit(FTrain* Tr) { return BuildCircuitFrom(Tr, Layout()); }

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
bool StationSaysGo(const std::vector<FPlatform>& Platforms, std::size_t Zone);

// PRE-LAUNCH. Is the device this train is about to be handed to ready to take it?
//
// The step between "everything is secured" and "you may go", and it belongs to the
// DEVICE rather than the platform: a launch armed and charged, a chain turning,
// tyres up to speed. The interlocking already asks whether the next blocks are
// CLEAR; this asks whether the one taking the train is READY, which is a different
// question and the one a real console puts a lamp on.
//
// Zones and blocks fall out of the same walk, so a device that exists in the next
// block starts exactly at its boundary. NO DEVICE THERE IS TRIVIALLY READY - plain
// track takes a train perfectly well, and a term that denied a dispatch onto open
// course would stop the ride rather than protect it.
bool DeviceAheadIsReady(const FRideSignals& Sig, const FCircuit& C,
                        const FTrackDrives& Drives, double AtS)
{
    const std::size_t N = Sig.NumBlocks();
    if (N == 0)
    {
        return true;
    }
    const std::size_t Next = (Sig.BlockAt(AtS) + 1) % N;
    const double NextS = Sig.Boundaries()[Next];

    // A zone's start IS a block boundary by construction, so the device in the next
    // block is the zone that begins there.
    for (std::size_t z = 0; z < C.ZoneStartS.size(); ++z)
    {
        if (std::fabs(C.ZoneStartS[z] - NextS) < 0.01)
        {
            return Drives.IsReady(z);
        }
    }
    return true;   // plain track, and plain track is always ready
}

// It writes to a DRIVE, not to the track. A command is a request; how fast the
// drive gets there, and whether it manages to, is the drive's business and the
// panel's story. This is the whole of the PLC's authority over the ride.
void ServeHolds(FTrain& Tr, const FRideSignals& Sig, std::size_t Id,
                const FCircuit& C, const FTrackSensors& Marks,
                FTrackDrives& Drives, const std::vector<FPlatform>& Platforms)
{
    const int Z = Tr.FindHoldZoneAt(Tr.GetDistance());
    if (Z < 0)
    {
        return;   // not standing at a holding device; nothing to command
    }
    const std::size_t Zi = static_cast<std::size_t>(Z);

    // THE PERMISSIVE IS AN AND, and the interlocking is only one term of it. A
    // real dispatch needs the blocks clear AND the riders aboard AND the
    // restraints locked AND the platform confirmed AND the device about to take
    // the train ready — and on a working ride the block is usually the term that
    // went green first while an operator was still walking the train. Before this,
    // a train left the station the instant the track ahead was free, which is a
    // ride with nobody in it.
    //
    // The third term is PRE-LAUNCH: clear is not the same as ready. A block with a
    // launch in it can be empty and still refuse a train, because the launch has
    // not armed.
    if (Sig.CanRelease(Id, Tr.GetDistance()) && StationSaysGo(Platforms, Zi)
        && DeviceAheadIsReady(Sig, C, Drives, Tr.GetDistance()))
    {
        Drives.Command(Zi, C.Authored[Zi]);
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
    const double Convey = std::min(C.Authored[Zi], 1.5);
    Drives.Command(Zi, Marks.IsBlocked(Zi) ? 0.0 : Convey);
}

// One platform per station zone. Combined here — riders off and on in one place,
// which is what every preset in this project has.
std::vector<FPlatform> BuildPlatforms(const FCircuit& C)
{
    std::vector<FPlatform> Out;
    for (std::size_t z = 0; z < C.Kinds.size(); ++z)
    {
        EStationRole PlatformRole;
        switch (C.Kinds[z])
        {
        case EZone::Station:       PlatformRole = EStationRole::Combined; break;
        case EZone::StationUnload: PlatformRole = EStationRole::Unload;   break;
        case EZone::StationLoad:   PlatformRole = EStationRole::Load;     break;
        default: continue;
        }
        FPlatform P;
        P.Zone = z;
        P.Process = FStationProcess(PlatformRole);
        Out.push_back(P);
    }
    return Out;
}

// The station's inputs, from instruments, once per scan. Only two of the six are
// real here and both are readings a control system genuinely has: the train is in
// the zone, and it is stopped ON ITS MARK. The other four are the crew's, which
// is the part that goes away when there are riders.
//
// "In position" is the stop mark AND a motor reading nothing, because a train
// running through the platform covers the same switch. Two instruments, and the
// pair means something neither does alone.
// bLoaded is per TRAIN and is the caller's to keep, because "has this vehicle got
// its riders yet" is exactly the sort of thing a real PLC tracks per vehicle and
// exactly the sort of thing a platform cannot know — a switch has no idea which
// train is over it. On a multi-position platform every position after the first
// sees an already-loaded train and must not board it again.
void ServeStations(std::vector<FPlatform>& Platforms, const std::vector<FTrain*>& Trains,
                   const FTrackSensors& Marks, const FTrackDrives& Drives, double Dt,
                   std::vector<bool>* bLoaded = nullptr)
{
    for (FPlatform& P : Platforms)
    {
        bool bPresent = false;
        std::size_t Who = 0;
        for (std::size_t t = 0; t < Trains.size(); ++t)
        {
            if (Trains[t]->IsInZone(P.Zone, Trains[t]->GetDistance()))
            {
                bPresent = true;
                Who = t;
            }
        }
        P.Inputs.bTrainPresent = bPresent;
        P.Inputs.bTrainInPosition = bPresent && Marks.IsBlocked(P.Zone)
            && std::fabs(Drives.Read(P.Zone).Actual) < 1e-6;

        const bool bAlready = bLoaded != nullptr && bPresent
            && P.Process.GetRole() == EStationRole::Load && (*bLoaded)[Who];

        P.Process.Update(P.Inputs);
        P.Crew.Serve(P.Process, P.Inputs, Dt, bAlready);

        // Boarded here, and it stays boarded. Set on readiness rather than on the
        // load contact so it survives the train being re-checked at the next
        // position — which is the whole reason the flag exists.
        if (bLoaded != nullptr && bPresent && P.Process.IsReadyToDispatch()
            && P.Process.NeedsLoad())
        {
            (*bLoaded)[Who] = true;
        }
    }
}

// Whether the station at this zone, if there is one, will let its train go. No
// station means nothing to ask, which is every device that is not a platform.
bool StationSaysGo(const std::vector<FPlatform>& Platforms, std::size_t Zone)
{
    for (const FPlatform& P : Platforms)
    {
        if (P.Zone == Zone) { return P.Process.IsReadyToDispatch(); }
    }
    return true;
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
            // THE PAD TRACKS THE SAME COMMAND — the line the actor has at its own
            // serve loop and this transcription lacked, found the expensive way: a
            // gated mid-course whose pad still held its authored ceiling relied on
            // 1.5 m/s^2 tyres to stop a 20 m/s train, which needs 133 m of a 130 m
            // block. The train escaped, the counter saw two trains in one block,
            // and the harness reproduced the actor's violation for the WRONG
            // reason. Guarded on the pad's own rate, exactly as the actor guards
            // on ZoneBrakeDecel: a zone with no pad keeps its negative limit.
            if (Tr->GetZone(z).BrakeDecel > 0.0)
            {
                Tr->SetZoneBrakeLimit(z, D.Output(z));
            }
            // And how much of that the hardware is actually producing. One device,
            // so every train's copy hears the same thing about it — the same reason
            // Output is pushed here rather than held per train.
            Tr->SetZoneHealth(z, D.DeliveredFraction(z));
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

// The other switches: one at every block boundary, feeding the train counter.
// Always circuit-wrapped, because a counter over a ring is the only shape
// FBlockCounter has — block N-1 is bounded by sensor N-1 and sensor 0.
void ScanBlockSensors(FTrackSensors& S, const std::vector<FTrain*>& Trains, double Total)
{
    S.BeginScan();
    for (const FTrain* Tr : Trains)
    {
        S.Cover(Tr->GetRearS(), Tr->GetFrontS(), true, Total);
    }
    S.EndScan();
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
    // 30.4 m/s, up from 26.5 when DragK was the derived 0.00045. Measuring drag
    // against a 142 km/h coast-down put it at 0.000100 — 4.5x lower — and this
    // number is the single most sensitive thing in the suite to that, because it
    // is a speed after 872 m of free running with nothing but resistance acting.
    // It is the assertion that would notice if the coefficient ever drifted back.
    assert(Mid > 30.0 && Mid < 31.0);
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
    std::vector<FPlatform> Platforms = BuildPlatforms(C);

    for (int Frame = 0; Frame < 240 * 120; ++Frame)
    {
        ScanStopMarks(Marks, Both, false, T.TotalLength());
        ServeStations(Platforms, Both, Marks, Drives, Dt);
        if (Frame >= Release)
        {
            ServeHolds(A, Sig, 0, C, Marks, Drives, Platforms);
        }
        ServeHolds(B, Sig, 1, C, Marks, Drives, Platforms);
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

    // A FINGERPRINT OF EVERY SCAN, so two runs can be compared as runs rather
    // than as outcomes. Outcomes agreeing proves very little — two runs that
    // diverged in the middle and converged on the same parked positions would
    // pass every other assertion in this file.
    //
    // The fixed scan period made determinism possible; this is what makes it
    // checkable, and it is the thing a downstream builder needs before a recorded
    // scenario can mean anything.
    std::uint64_t Digest = 0;
    int FirstFaultedDrive = -1;   // which one, so a fault is a place to go and look

    // Where the interlocking first said no, because "22 violations" is a count and
    // "train 1 at 34.7 m, nine seconds in" is somewhere to go and look.
    double FirstViolation = -1.0;
    int FirstViolationTrain = -1;
    double FirstViolationS = 0.0;

    // The two independent means of knowing where the trains are, disagreeing.
    // Either one of them is wrong, and neither can say which.
    double FirstDivergence = -1.0;
    int FirstDivergenceBlock = -1;
    bool bCounterOverOccupied = false;
    bool bCounterInconsistent = false;

    std::vector<double> FinalSpeed;     // per train, at the end of the run
    std::vector<double> FinalS;
    std::vector<int> FinalHoldZone;     // -1 if it did not stop at a holding device

    // Where each train's whole body ended up, for the evacuation question.
    std::vector<double> FinalRearS;
    std::vector<double> FinalFrontS;

    // Tier 3, if it was attached at all. Counted rather than kept, because the
    // point of these is that the ride does not care: they must be able to be any
    // number without the digest above moving by a bit.
    // The speed trap, which is a SECOND question rather than a better answer to
    // the interlocking's. Blocks ask "is the space ahead free"; this asks "can
    // what is ahead stop what is coming".
    bool bOverspeed = false;
    double FirstOverspeed = -1.0;    // seconds
    double OverspeedSpeed = 0.0;     // what the switches measured
    double OverspeedNeeded = 0.0;    // metres to stop
    double OverspeedHave = 0.0;      // metres of device
    int OverspeedZone = -1;

    // WHAT THE TRAP ACTUALLY BUYS on a layout that can absorb the failure: the
    // smallest gap between what a train needed to stop and what the device it was
    // entering had. It collapses long before it goes negative, which is a
    // measurement of degradation rather than a trip.
    double WorstTrapMarginM = 1e9;
    int WorstTrapZone = -1;
    double WorstTrapSpeed = 0.0;

    std::size_t TrapTrips = 0;
    std::size_t ShowEvents = 0;
    std::size_t ShowFirings = 0;
    std::size_t ShowInhibited = 0;
};

// The actor's tick, N trains, for Seconds of ride. One place, because the only
// way to trust a capacity number is for the capacity test and the two-train test
// to be running the SAME policy.
bool GTraceRun = false;

FRunResult RunTrains(std::size_t N, std::size_t Lookahead, double Seconds,
                     double EStopAtSeconds = -1.0,
                     const std::vector<FItem>* InItems = nullptr,
                     double InTrainLen = TrainLen,
                     int DegradeZone = -1, double DegradeTo = 1.0,
                     bool bShow = false,
                     FScenario* Record = nullptr, FScenario* Play = nullptr)
{
    const std::vector<FItem> Items = InItems != nullptr ? *InItems : Layout();
    const FCircuit Shape = BuildCircuitFrom(nullptr, Items);
    const FTrack T = BuildTrack(Shape.Doc);
    FTrainConfig Cfg;
    Cfg.TrainLength = InTrainLen;

    std::vector<std::unique_ptr<FTrain>> Owned;
    FCircuit C;
    for (std::size_t t = 0; t < N; ++t)
    {
        Owned.push_back(std::unique_ptr<FTrain>(new FTrain(T, Cfg)));
        C = BuildCircuitFrom(Owned.back().get(), Items, InTrainLen);
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
    FSimDigest Digest;
    R.Laps.assign(N, 0);
    R.ParkedNoseS.assign(C.Authored.size(), 0.0);
    R.PeakLoad.assign(C.Authored.size(), 0.0);
    std::vector<double> Prev(N);
    for (std::size_t t = 0; t < N; ++t) { Prev[t] = Owned[t]->GetDistance(); }

    FTrackSensors Marks(C.StopMarkS);
    FTrackDrives Drives = OpenDrives(*Owned[0], C);
    // TIER 3, and it is bolted on at the END of the scan on purpose — it reads
    // outputs, so it belongs after they are written, which is the same read /
    // execute / write order everything else here runs on.
    FShowPublisher Show;
    FShowBus Bus;
    if (bShow)
    {
        // The card's own worked examples, one of each: a camera and a pyro on the
        // same sensor, audio on a station phase, scenery on a block going
        // occupied. One mechanism, different fixtures — which is the evidence the
        // boundary is in the right place.
        Bus.AddTrigger({1, ERideEventKind::SensorTrip,   1, 1, false});   // camera
        Bus.AddTrigger({2, ERideEventKind::SensorTrip,   1, 1, true});    // pyro
        Bus.AddTrigger({3, ERideEventKind::StationPhase, 0, static_cast<int>(EStationPhase::Loading), false});
        Bus.AddTrigger({4, ERideEventKind::BlockState,   2, static_cast<int>(EBlockState::Occupied), false});
        Bus.SetHazardPermissive(true);
    }

    // A piece of hardware that does not deliver. Set on the DRIVE, before the run,
    // and never touched again — nothing here commands it differently and nothing
    // is told it is broken.
    if (DegradeZone >= 0)
    {
        Drives.SetDeliveredFraction(static_cast<std::size_t>(DegradeZone), DegradeTo);
    }
    std::vector<FTrain*> All;
    for (std::size_t t = 0; t < N; ++t) { All.push_back(Owned[t].get()); }
    std::vector<FPlatform> Platforms = BuildPlatforms(C);
    // Trains start the session already loaded: an operator's sweep confirms the
    // ride is empty, and everything at a platform is boarded before it opens.
    std::vector<bool> Loaded(N, true);

    // A SECOND, INDEPENDENT MEANS OF KNOWING WHERE THE TRAINS ARE. One sensor per
    // block boundary, and a counter deriving occupancy from their trips alone —
    // no position, no train identity, nothing the interlocking is reading.
    //
    // The point is the DISAGREEMENT. Two ways of knowing the same fact, arrived at
    // from different information, and if they ever differ then one of them is
    // wrong and neither can say which. That is what a real installation buys with
    // its second detection method, and it is worth more than either method alone:
    // the counter proved equal to perfect knowledge over three laps in its own
    // suite, and this is what keeps it equal on every layout after.
    // SPEED TRAPS, one ahead of every holding device. Two switches a surveyed
    // 10 m apart, ending 2 m before the device starts — far enough back that a
    // train has been measured before it commits, and wide enough that the scan
    // quantisation is 1.25% at 30 m/s rather than the 12.5% a 1 m gap gives.
    //
    // This is the detector the failed-brake measurement asked for. Nothing here
    // decides anything: the trap reports a speed, and the comparison against what
    // the device can do is the PLC program's, the same rule the drives run on.
    std::vector<double> TrapAt;
    std::vector<std::size_t> TrapZone;
    std::vector<double> TrapDecel;
    std::vector<double> TrapDeviceLength;
    for (std::size_t z = 0; z < C.Authored.size(); ++z)
    {
        const FTrackZone Z = Owned[0]->GetZone(z);
        if (!(Z.MaxAccel > 0.0 && Z.MaxDecel > 0.0))
        {
            continue;                     // not a holding device: nothing to protect
        }
        const double Total2 = T.TotalLength();
        auto Wrap = [Total2](double X) { return X < 0.0 ? X + Total2 : X; };
        TrapZone.push_back(z);
        // THE HARDER OF THE TWO MACHINES, the same rule the device audit already
        // applies, and the trap predates pads so it had never learned it. Judged
        // on the tyres alone, a mid-course with 1.5 m/s^2 tyres and an 8 m/s^2
        // pad reads as needing 232 m of a 130 m device -- so the trap tripped the
        // E-stop on a healthy arrival THE PAD WAS ABOUT TO STOP IN 44 m, froze
        // every drive at output zero, and parked the whole ride with zero
        // violations on the board. A commissioned trap threshold is surveyed
        // from the device's actual braking capability, which includes its pad.
        TrapDecel.push_back(std::max(Z.MaxDecel, Z.BrakeDecel));
        TrapDeviceLength.push_back(Z.EndS - Z.StartS);
        TrapAt.push_back(Wrap(Z.StartS - 12.0));
        TrapAt.push_back(Wrap(Z.StartS - 2.0));
    }
    FTrackSensors TrapSensors(TrapAt);
    FSpeedTraps Traps(TrapSensors);
    for (std::size_t k = 0; k < TrapZone.size(); ++k)
    {
        Traps.Add({2 * k, 2 * k + 1, 10.0});
    }

    FTrackSensors BlockSensors(C.Boundaries);
    FBlockCounter Counter(BlockSensors);
    {
        // SEEDED from where the trains actually are, which is the operator's sweep
        // before the ride opens. Without it the counter is right in the middle of a
        // run and wrong at the start of one.
        ScanBlockSensors(BlockSensors, All, T.TotalLength());
        Counter.Scan();
        for (std::size_t t = 0; t < N; ++t)
        {
            Counter.Seed(Sig.BlockAt(Owned[t]->GetDistance()));
        }
    }

    // What a recording IS: operator actions and fault injections, at the scan
    // they happened. Not initial state — the preset, the train count and a device
    // that was already broken when the session opened are not events, and a
    // recording that tried to carry them would be a save file wearing a
    // scenario's clothes.
    FSignalWatch Recorder;

    const double Dt = 1.0 / 240.0;
    for (int F = 0; F < static_cast<int>(240.0 * Seconds); ++F)
    {
        // PLAYBACK AT THE TOP OF THE SCAN, with the inputs, because that is where
        // a real operator's button is read. Pumped every scan — Due() fires on the
        // exact scan and counts anything it was not asked about, so a caller that
        // skipped one is told rather than quietly running a different session.
        if (Play != nullptr)
        {
            for (const FScenarioStep& St : Play->Due(static_cast<std::uint64_t>(F)))
            {
                switch (St.Action)
                {
                case EScenarioAction::PressEmergencyStop:
                    Drives.TripEmergencyStop("scenario");
                    break;
                case EScenarioAction::DegradeDrive:
                    Drives.SetDeliveredFraction(static_cast<std::size_t>(St.A), St.B / 100.0);
                    break;
                default:
                    break;
                }
            }
        }
        if (EStopAtSeconds >= 0.0 && F * Dt >= EStopAtSeconds)
        {
            Drives.TripEmergencyStop("test");
        }

        // THE RECORDER, and it needs no class of its own: an edge detector and
        // FScenario::Add are the whole of it. Same seeding rule as everything else
        // here, so the state the session STARTED in is a baseline rather than an
        // event at scan zero.
        if (Record != nullptr && Recorder.Changed(0, Drives.IsEmergencyStopped() ? 1 : 0))
        {
            Record->Add(static_cast<std::uint64_t>(F),
                        Drives.IsEmergencyStopped() ? EScenarioAction::PressEmergencyStop
                                                    : EScenarioAction::ReleaseEmergencyStop);
        }
        // INPUTS ONCE AT THE TOP OF THE FRAME, with everything else that is read.
        TrapSensors.BeginScan();
        for (FTrain* Tr : All)
        {
            TrapSensors.Cover(Tr->GetRearS(), Tr->GetFrontS(), true, T.TotalLength());
        }
        TrapSensors.EndScan();
        Traps.Scan(Dt);

        // THE PLC'S DECISION, not the trap's. A fresh measurement, over enough
        // scans to mean something, against v^2/2a for the device it is about to
        // enter — the same arithmetic build time already runs, with a MEASURED
        // speed instead of a predicted one.
        for (std::size_t k = 0; k < Traps.Num(); ++k)
        {
            const FSpeedTrapReading& Tr = Traps.Read(k);
            if (!Tr.bValid || Tr.AtScan != Traps.ScansTaken() || Tr.OverScans < 8)
            {
                continue;
            }
            ++R.TrapTrips;
            const double Needed = StoppingDistanceM(Tr.SpeedMs, TrapDecel[k]);
            const double Margin = TrapDeviceLength[k] - Needed;
            if (Margin < R.WorstTrapMarginM)
            {
                R.WorstTrapMarginM = Margin;
                R.WorstTrapZone = static_cast<int>(TrapZone[k]);
                R.WorstTrapSpeed = Tr.SpeedMs;
            }
            if (Needed > TrapDeviceLength[k] && !R.bOverspeed)
            {
                R.bOverspeed = true;
                R.FirstOverspeed = F * Dt;
                R.OverspeedSpeed = Tr.SpeedMs;
                R.OverspeedNeeded = Needed;
                R.OverspeedHave = TrapDeviceLength[k];
                R.OverspeedZone = static_cast<int>(TrapZone[k]);
                Drives.TripEmergencyStop("overspeed: too fast for the next device");
            }
        }

        ScanStopMarks(Marks, All, true, T.TotalLength());
        ServeStations(Platforms, All, Marks, Drives, Dt, &Loaded);
        for (std::size_t t = 0; t < N; ++t)
        {
            ServeHolds(*Owned[t], Sig, t, C, Marks, Drives, Platforms);
        }
        Drives.Tick(Dt);
        DriveTheTrack(Drives, All);

        // A DIAGNOSTIC TAP, off unless a test turns it on. One line a second for
        // the first train: where it is, what it feels, and what the drive under
        // it believes — the same three numbers the maintenance panel shows,
        // because "the train does not move" has too many candidate causes to
        // argue about and exactly one of these columns goes wrong first.
        if (GTraceRun && F % 240 == 0)
        {
            const FTrain& Tr0 = *Owned[0];
            const int Hz = Tr0.FindHoldZoneAt(Tr0.GetDistance());
            const std::size_t Zi = Hz >= 0 ? static_cast<std::size_t>(Hz) : 0;
            std::printf("      t=%3.0f s S=%7.1f v=%5.2f zone=%2d cmd=%5.2f out=%5.2f "
                        "tgt=%5.2f pad=%5.2f mark=%d\n",
                        F * Dt, Tr0.GetDistance(), Tr0.GetSpeed(), Hz,
                        Hz >= 0 ? Drives.Read(Zi).Commanded : -1.0,
                        Hz >= 0 ? Drives.Output(Zi) : -1.0,
                        Hz >= 0 ? Tr0.GetZoneTargetSpeed(Zi) : -1.0,
                        Hz >= 0 ? Tr0.GetZoneBrakeLimit(Zi) : -1.0,
                        Hz >= 0 ? (Marks.IsBlocked(Zi) ? 1 : 0) : -1);
        }

        for (std::size_t t = 0; t < N; ++t)
        {
            Owned[t]->Step(Dt);
            if (!Sig.Update(t, Owned[t]->GetRearS(), Owned[t]->GetFrontS())
                && R.FirstViolation < 0.0)
            {
                R.FirstViolation = F * Dt;
                R.FirstViolationTrain = static_cast<int>(t);
                R.FirstViolationS = Owned[t]->GetDistance();
            }

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

        // THE CROSS-CHECK, every frame, on every block. The interlocking is handed
        // each train's span; the counter has only edges. They must agree.
        ScanBlockSensors(BlockSensors, All, T.TotalLength());
        Counter.Scan();
        if (Counter.IsInconsistent()) { R.bCounterInconsistent = true; }
        for (std::size_t b = 0; b < Sig.NumBlocks(); ++b)
        {
            if (Counter.IsOverOccupied(b)) { R.bCounterOverOccupied = true; }
            if (Counter.IsOccupied(b) != Sig.Occupies(b) && R.FirstDivergence < 0.0)
            {
                R.FirstDivergence = F * Dt;
                R.FirstDivergenceBlock = static_cast<int>(b);
            }
        }

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

        // EVERY SCAN, not just the last. A digest taken once at the end cannot
        // tell a run that never diverged from one that diverged and came back,
        // and the second is the interesting failure.
        //
        // Physics AND control state, because either can drift alone: a train in
        // the right place with the wrong block state is still a different run.
        for (std::size_t t = 0; t < N; ++t)
        {
            Digest.Add(Owned[t]->GetDistance());
            Digest.Add(Owned[t]->GetSpeed());
        }
        for (std::size_t k = 0; k < Sig.NumBlocks(); ++k)
        {
            Digest.Add(static_cast<int>(Sig.GetState(k)));
        }
        for (std::size_t z = 0; z < Drives.Num(); ++z)
        {
            Digest.Add(Drives.Output(z));
        }

        // AFTER THE DIGEST, deliberately. Everything above is the ride; this is a
        // subscriber to it, and the whole claim being asserted is that it cannot
        // reach back. If it could, it would have to be hashed too.
        if (bShow)
        {
            Show.BeginScan(static_cast<std::uint64_t>(F));
            for (std::size_t k = 0; k < Sig.NumBlocks(); ++k)
            {
                Show.Observe(ERideEventKind::BlockState, static_cast<int>(k),
                             static_cast<int>(Sig.GetState(k)));
            }
            for (std::size_t i = 0; i < Marks.Num(); ++i)
            {
                Show.Observe(ERideEventKind::SensorTrip, static_cast<int>(i),
                             Marks.IsBlocked(i) ? 1 : 0);
            }
            for (std::size_t pi = 0; pi < Platforms.size(); ++pi)
            {
                Show.Observe(ERideEventKind::StationPhase, static_cast<int>(pi),
                             static_cast<int>(Platforms[pi].Process.GetPhase()));
            }
            R.ShowEvents += Show.Scanned().size();
            for (const FShowFiring& Fi : Bus.Deliver(Show.Scanned()))
            {
                ++R.ShowFirings;
                if (Fi.bInhibited) { ++R.ShowInhibited; }
            }
        }
    }
    R.Digest = Digest.Value();
    R.Violations = Sig.Violations();
    for (std::size_t t = 0; t < N; ++t)
    {
        R.FinalSpeed.push_back(Owned[t]->GetSpeed());
        R.FinalS.push_back(Owned[t]->GetDistance());
        R.FinalHoldZone.push_back(Owned[t]->FindHoldZoneAt(Owned[t]->GetDistance()));
        // NOSE AND TAIL, not the centre, because evacuation is a question about
        // the whole train: the back car is the one that gets forgotten.
        R.FinalRearS.push_back(Owned[t]->GetRearS());
        R.FinalFrontS.push_back(Owned[t]->GetFrontS());
    }
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

void TestAFailedBrakeAndWhatDoesAndDoesNOTCatchIt()
{
    // FAULTS.md listed the failed brake twice: as the one fault this project could
    // not EXPRESS, and as the one with the least protection. Expressing it is
    // FTrackDrives::SetDeliveredFraction — the command is still correct, the drive
    // still writes it, and less comes out of the hardware. A glazed pad, low line
    // pressure, a worn tyre.
    //
    // What it is protected BY turned out to be three different answers depending
    // on where the ride was when the device failed, and one of them is "nothing".
    // Zones: 0 station, 1 launch, 2 mid-course brake, 3 outer brake,
    // 4 transfer tyres, 5 inner brake.
    const FCircuit Shape = BuildCircuit(nullptr);
    assert(Shape.Kinds[3] == EZone::BlockBrake && Shape.Authored[3] == 6.0);

    // FIRST, THE DEFAULT COSTS NOTHING. Every figure this file prints was measured
    // before health existed, so healthy hardware has to leave the ride not merely
    // similar but IDENTICAL — the same digest, which is every drive output and
    // every train position on all 57,600 scans of a four-minute run.
    const FRunResult Well = RunTrains(2, 1, 240.0);
    const FRunResult Same = RunTrains(2, 1, 240.0, -1.0, nullptr, TrainLen, 3, 1.0);
    assert(Same.Digest == Well.Digest);
    assert(Well.Violations == 0);

    // ---- 1. A DEAD DEVICE UNDER A TRAIN STOPS THE RIDE, and nobody arranged it.
    //
    // The dispatcher's rule is "truck forward until a switch says far enough". A
    // train standing on a device that delivers nothing never reaches its mark, so
    // it is never in position, so the permissive never grants. The ride does not
    // dispatch a train it could not have held.
    //
    // That is the fail-safe falling out of the stop mark rather than out of any
    // check for it, and it is the strongest argument yet for having made holding a
    // question a SWITCH answers instead of a number in the program.
    const FRunResult DeadStation = RunTrains(2, 1, 240.0, -1.0, nullptr, TrainLen, 0, 0.0);
    assert(DeadStation.Laps[0] == 0 && DeadStation.Laps[1] == 0);
    assert(DeadStation.Violations == 0);
    assert(DeadStation.FinalSpeed[0] == 0.0);

    // ---- 2. LOADED, THE INTERLOCKING CATCHES IT, AND SO DOES THE COUNTER.
    //
    // Three trains, and the transfer tyres fail while a train is approaching them
    // rather than standing on them. It overruns into a block it was not given, the
    // interlocking says so, and the second independent means of detection — the
    // block counter, which knows only that switches tripped — independently counts
    // a block occupied twice. Two mechanisms with nothing in common but the answer.
    const FRunResult Loaded = RunTrains(3, 1, 240.0, -1.0, nullptr, TrainLen, 4, 0.0);
    assert(Loaded.Violations > 0);
    assert(Loaded.FirstViolation > 0.0 && Loaded.FirstViolation < 60.0);
    assert(Loaded.bCounterOverOccupied);

    // ---- 3. AND ON A LIGHTLY LOADED RIDE IT IS ABSORBED, SILENTLY.
    //
    // THIS ENTRY WAS WRONG WHEN IT WAS FIRST WRITTEN, and the speed trap is what
    // corrected it. The claim was that the arriving train "circulates at 30.5 m/s
    // past a station it should be parked in, undetected" — a runaway nothing
    // caught. The 30.5 was the train's speed at the END of a four-minute run,
    // which says only that it was somewhere fast at t = 240, and was read as
    // evidence of something it does not show.
    //
    // What actually happens, measured at every trap on the way round: the train
    // is not held at the dead outer brake, arrives at the transfer tyres at
    // 16.1 m/s instead of 6.1, and IS STOPPED THERE. The layout has the length.
    // No violation is raised because no train was ever endangered, and that is
    // correct rather than a gap.
    //
    // SO A FAILED BRAKE ON A WELL-LAID-OUT RIDE IS NOT A SAFETY EVENT. It is a
    // capacity and schedule event: a block that should have held a train did not,
    // the headway is wrong, and every downstream device is working harder than it
    // was specified to. Nothing in this model says any of that, and the detector
    // that is actually missing is "a train did not stop where it was told to" —
    // not overspeed.
    const FRunResult Sparse = RunTrains(2, 1, 240.0, -1.0, nullptr, TrainLen, 3, 0.0);
    assert(Sparse.Violations == 0);            // and rightly so
    assert(Sparse.Laps[1] >= 1);               // the ride kept running

    // ---- 4. WHAT THE TRAP BUYS IS THE MARGIN, NOT THE TRIP.
    //
    // Two switches a surveyed 10 m apart, ending 2 m before each holding device,
    // giving speed from the time between two rising edges. No position and no
    // train identity — what a real installation measures and what this layer is
    // entitled to know. Against v^2/2a for the device ahead, which is the same
    // arithmetic build time already runs with a MEASURED speed instead of a
    // predicted one.
    //
    // IT DOES NOT TRIP ON EITHER RUN, and that is the result rather than a
    // disappointment: a protective detector that fires when nothing is unsafe is
    // worse than no detector at all. What it shows instead is the STOPPING MARGIN
    // collapsing — the healthy ride never comes within 20 m of running out of
    // brake, and with one brake dead the worst case is down to about 5 m.
    //
    // That is the number an operator would actually be shown, and it degrades
    // continuously where a trip is a cliff.
    assert(!Well.bOverspeed);
    assert(!Sparse.bOverspeed);
    assert(!DeadStation.bOverspeed);
    // Healthy, the tightest moment on the whole circuit is the outer brake: the
    // trap 12 m upstream reads 16.90 m/s, which needs 23.8 m of the 37.5 m it has.
    // Slightly conservative by construction, because a trap sits BEFORE the device
    // and reads a train that has not started braking — which is the right side to
    // be wrong on.
    assert(Well.WorstTrapMarginM > 13.0);
    assert(Well.WorstTrapZone == 3);
    // With that brake dead the worst case moves to the transfer tyres and falls to
    // 5.1 m. Still positive, so still absorbed — and down by a factor of nearly
    // three, which is the thing worth showing somebody.
    assert(Sparse.WorstTrapMarginM < 6.0);
    assert(Sparse.WorstTrapMarginM > 0.0);
    assert(Sparse.WorstTrapZone == 4);

    std::printf("  a failed brake: parked on it the ride never dispatches;"
                " loaded it violates at %.1f s and the counter agrees;"
                " sparse it is ABSORBED — stopping margin falls from %.1f m to"
                " %.1f m (zone %d, arriving %.1f m/s) with no violation\n",
                Loaded.FirstViolation, Well.WorstTrapMarginM, Sparse.WorstTrapMarginM,
                Sparse.WorstTrapZone, Sparse.WorstTrapSpeed);
}

void TestTheRideIsIDENTICALWithTheShowLayerAbsent()
{
    // CONSTRAINT 7'S TIER 3 BOUNDARY, PROVEN RATHER THAN PROMISED.
    //
    // The claim on the card is that show is READ-ONLY to the ride — a train
    // passes a sensor, the DMX side is told, and nothing comes back. DMX512
    // agrees at the wire level: it is unidirectional by design with no return
    // path. But "there is no method to call" is an argument from reading the
    // code, and this project has been wrong that way before.
    //
    // FSimDigest is the instrument that turns it into a measurement. Two runs of
    // the same circuit, one with the whole show layer attached and one without,
    // hashing every train position, every block state and every drive output on
    // all 57,600 scans. Identical, to the bit.
    const FRunResult Dark = RunTrains(2, 1, 240.0);
    const FRunResult Lit  = RunTrains(2, 1, 240.0, -1.0, nullptr, TrainLen, -1, 1.0, true);
    assert(Lit.Digest == Dark.Digest);
    assert(Lit.Violations == Dark.Violations);

    // AND IT IS NOT A VACUOUS PASS. A show layer that fired nothing would satisfy
    // the assertion above perfectly and prove nothing at all — the exact trap the
    // envelope suite fell into. So the events and the firings are counted, and
    // both have to be real.
    assert(Lit.ShowEvents > 100);
    assert(Lit.ShowFirings > 0);
    assert(Dark.ShowEvents == 0);

    // The pyro and the camera hang off the SAME sensor, so with the hazard
    // permissive open every trip fires both — one mechanism, two fixtures, which
    // is the evidence the boundary is drawn in the right place rather than around
    // a special case.
    assert(Lit.ShowInhibited == 0);          // the permissive was opened
    assert(Lit.ShowFirings >= 2);

    std::printf("  the show layer sees %zu events and fires %zu cues, and the ride"
                " is bit-identical without it\n", Lit.ShowEvents, Lit.ShowFirings);
}

void TestASessionCanBeRECORDEDAndREPLAYEDBitForBit()
{
    // THE DETERMINISM CARD'S REMAINING HALF. What existed proved that two
    // IDENTICAL runs match. What it could not do is take a session somebody
    // actually had — an operator hitting the stop, a device failing under them —
    // and run it again.
    //
    // A recorder needs no class of its own. FSignalWatch detects the edge and
    // FScenario::Add writes the step, and both already exist for other reasons;
    // building a third thing to hold them would be scaffolding.
    //
    // The session: three trains, the outer brake dead from the start, and an
    // operator who stops the ride at 30 seconds.
    FScenario Recorded;
    const FRunResult Live = RunTrains(3, 1, 120.0, 30.0, nullptr, TrainLen, 3, 0.0,
                                      false, &Recorded);
    assert(Recorded.Num() == 1);                       // one edge, not one a frame
    assert(Recorded.LastScan() == 240 * 30);           // and at the scan it happened

    // WHAT A RECORDING IS AND IS NOT. It carries EVENTS. It does not carry the
    // preset, the train count, or a brake that was already dead when the session
    // opened — those are initial state, and a recording that tried to hold them
    // would be a save file wearing a scenario's clothes. So the replay is handed
    // the same starting conditions and nothing else.
    const FRunResult Replay = RunTrains(3, 1, 120.0, -1.0, nullptr, TrainLen, 3, 0.0,
                                        false, nullptr, &Recorded);
    assert(Replay.Digest == Live.Digest);
    assert(Recorded.MissedSteps() == 0);               // every step fired on its scan
    assert(Recorded.IsFinished());

    // AND IT IS NOT VACUOUS. A recording of a session where nothing happened
    // would replay perfectly and prove nothing. This one stopped a moving ride:
    // the digest has to differ from the same layout left alone.
    const FRunResult Untouched = RunTrains(3, 1, 120.0, -1.0, nullptr, TrainLen, 3, 0.0);
    assert(Untouched.Digest != Live.Digest);

    // REWIND, and the second replay is the first. A scenario that could only be
    // played once is a recording you get one look at.
    Recorded.Rewind();
    const FRunResult Again = RunTrains(3, 1, 120.0, -1.0, nullptr, TrainLen, 3, 0.0,
                                       false, nullptr, &Recorded);
    assert(Again.Digest == Live.Digest);

    std::printf("  a session recorded (%zu step) and replayed to digest %016llx,"
                " which an untouched run does not reach\n",
                Recorded.Num(), static_cast<unsigned long long>(Live.Digest));
}

void TestSeveralHoldingDevicesInARowStaySeveral()
{
    // A QUEUE OF BLOCK BRAKES, which is all a backstage buffer keeping trains fed
    // to a platform ever was — not a new kind of track, and not something the
    // interlocking cannot express. Ten trains stacked is ten trains in ten
    // consecutive blocks, which is ordinary.
    //
    // What was missing was the ability to SAY it. A run used to be a contiguous
    // stretch of the same KIND, so four brake sections in a row became one zone
    // and one block, holding one train instead of four.
    std::vector<FItem> Items;
    AddStraight(Items, 26.0, EZone::Station, 1.5);
    AddStraight(Items, 30.0, EZone::BlockBrake, 5.0);
    AddStraight(Items, 30.0, EZone::BlockBrake, 4.0);
    AddStraight(Items, 30.0, EZone::BlockBrake, 3.0);
    AddStraight(Items, 30.0, EZone::BlockBrake, 2.0);
    AddStraight(Items, 60.0);

    const FCircuit C = BuildCircuitFrom(nullptr, Items);

    // Five devices, five blocks, five places a train can stand — plus the plain
    // stretch at the end, which is a block with nothing in it.
    assert(C.Authored.size() == 5);
    assert(C.HoldMidS.size() == 5);
    assert(C.Boundaries.size() == 6);

    // And every one kept the speed it was TYPED at. Under the old rule the four
    // brakes were a single zone at 5.0 and the 4, 3 and 2 never existed.
    assert(std::fabs(C.Authored[1] - 5.0) < 1e-9);
    assert(std::fabs(C.Authored[2] - 4.0) < 1e-9);
    assert(std::fabs(C.Authored[3] - 3.0) < 1e-9);
    assert(std::fabs(C.Authored[4] - 2.0) < 1e-9);

    // Boundaries fall where the devices meet, so a train in one is not in another.
    const double Want[6] = {0.0, 26.0, 56.0, 86.0, 116.0, 146.0};
    for (std::size_t b = 0; b < 6; ++b)
    {
        assert(std::fabs(C.Boundaries[b] - Want[b]) < 1e-9);
    }

    // Same kind AND same speed still merges, which is the rule doing what it says
    // rather than an oversight: that really is one device spanning two segments,
    // and it is how every lift hill in this project is authored.
    std::vector<FItem> Same;
    AddStraight(Same, 20.0, EZone::Lift, 4.0);
    AddStraight(Same, 20.0, EZone::Lift, 4.0);
    const FCircuit M = BuildCircuitFrom(nullptr, Same);
    assert(M.Authored.size() == 1);
}

void TestADispatchWaitsForTheLaunchToBeArmed()
{
    // PRE-LAUNCH, on the real layout. The step a real console has between
    // "everything is secured" and "you may go": harness locked and gates closed ->
    // COMPLETE -> PRE-LAUNCH -> advance/dispatch.
    //
    // It is the DEVICE declaring itself ready rather than the platform, so the
    // permissive gained a third term. CLEAR IS NOT READY — a block with a launch in
    // it can be perfectly empty and still refuse a train, because the launch has
    // not armed.
    const FCircuit C = BuildCircuit(nullptr);
    const FTrack T = BuildTrack(C.Doc);
    FRideSignals Sig(C.Boundaries, 5.0, 1, 1, true);

    FTrainConfig Cfg;
    Cfg.TrainLength = TrainLen;
    FTrain Tr(T, Cfg);
    const FCircuit Live = BuildCircuit(&Tr);
    FTrackDrives Drives = OpenDrives(Tr, Live);

    // A train standing in the station. Block 1 ahead of it is the launch.
    const double AtStation = Live.HoldMidS[0];
    assert(Sig.BlockAt(AtStation) == 0);

    // As the ride opens, every drive is PRESET — commanded and already there — so
    // the launch is armed and the term is satisfied. That is why nothing measured
    // before this moved: a drive with no ramp reaches its command in one frame.
    assert(DeviceAheadIsReady(Sig, Live, Drives, AtStation));

    // Now give the launch a real ramp and start it from rest, which is a launch
    // that has been TOLD to arm and has not finished doing it.
    FDriveSpec Slow;
    Slow.AccelRampMs2 = 4.0;
    assert(Drives.Configure(1, Slow));
    Drives.Preset(1, 0.0);
    Drives.Command(1, Live.Authored[1]);       // 38 m/s

    assert(!DeviceAheadIsReady(Sig, Live, Drives, AtStation));
    Drives.Tick(1.0 / 240.0);
    assert(!DeviceAheadIsReady(Sig, Live, Drives, AtStation));   // still ramping

    // 38 m/s at 4 m/s^2 is 9.5 seconds, and only then may a train be handed to it.
    for (int i = 0; i < 240 * 10; ++i) { Drives.Tick(1.0 / 240.0); }
    assert(DeviceAheadIsReady(Sig, Live, Drives, AtStation));

    // AND PLAIN TRACK IS ALWAYS READY. A term that denied a dispatch onto open
    // course would stop the ride rather than protect it — the mid-course brake is
    // preceded by unpowered track, and a train has to be allowed onto it.
    const double BeforePlainTrack = 200.0;      // out on the course, no device ahead
    assert(DeviceAheadIsReady(Sig, Live, Drives, BeforePlainTrack));

    // An E-STOPPED ride is ready for nothing, so no dispatch survives one.
    Drives.TripEmergencyStop("test");
    assert(!DeviceAheadIsReady(Sig, Live, Drives, AtStation));
}

void TestTwoIndependentMeansOfKnowingAgreeOnEveryBlock()
{
    // THE SENSOR LAYER DOING SAFETY WORK RATHER THAN SITTING DECORATIVE. Until
    // this, FBlockCounter was proven equal to perfect knowledge in its own suite —
    // one train, a synthetic ring, three laps — and then nothing read it. The
    // interlocking went on being handed each train's exact span.
    //
    // Now both run, on the real circuit, and their DISAGREEMENT is the product.
    // One knows where every train is because it is told; the other has nothing but
    // rising and falling edges at ten switches. If they ever differ then one of
    // them is wrong and neither can say which, which is exactly what a second
    // detection method buys a real installation.
    //
    // It is also the only thing that keeps the counter honest on layouts nobody
    // wrote a bespoke test for: every run through this harness now checks it.
    for (std::size_t N = 1; N <= 4; ++N)
    {
        const FRunResult R = RunTrains(N, 1, 420.0);
        assert(R.FirstDivergence < 0.0);
        assert(!R.bCounterOverOccupied);
        assert(!R.bCounterInconsistent);
        assert(R.Violations == 0);
    }

    // And on the small-batch circuit, whose platform is four short blocks in a row
    // — the shape most likely to catch a counter out, because a 6 m train crosses
    // three boundaries in the time a 15 m one crosses two.
    const std::vector<FItem> Items = SmallBatchCircuitLayout();
    const FRunResult B = RunTrains(5, 1, 420.0, -1.0, &Items, BatchTrainLen);
    assert(B.FirstDivergence < 0.0);
    assert(!B.bCounterOverOccupied);
    assert(!B.bCounterInconsistent);
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
        if (!(R.PeakLoad[z] > 0.99))
        {
            std::fprintf(stderr, "drive %zu peaked at only %.3f\n", z, R.PeakLoad[z]);
        }
        assert(R.PeakLoad[z] > 0.99);
    }
}

// A SMALL-BATCH PLATFORM: three load positions in a row, each its own device with
// its own motor, then the course. Common on rides with small vehicles, where one
// train's worth of riders is a handful of people and the platform is long enough
// to work three at once.
//
// Positions are identical — same kind, same speed, same length — so nothing the
// walk can see distinguishes them and bNewDevice is what says they are three
// machines rather than one long one.
std::vector<FItem> SmallBatchLayout()
{
    std::vector<FItem> Out;
    AddStraight(Out, 20.0, EZone::StationLoad, 2.0);         // position 3, rearmost
    AddStraight(Out, 20.0, EZone::StationLoad, 2.0, true);   // position 2
    AddStraight(Out, 20.0, EZone::StationLoad, 2.0, true);   // position 1, front
    AddStraight(Out, 60.0, EZone::Launch, 18.0);             // onto the course
    AddStraight(Out, 150.0);
    // And a run of brake sections to receive them, which is the same authoring
    // trick as the platform and the other thing bNewDevice exists for. Without
    // somewhere for three trains to park, the first one to leave stands in the
    // only holding block on the course and the permissive correctly refuses to
    // release anybody else — a dead end rather than a ride.
    AddStraight(Out, 60.0, EZone::BlockBrake, 3.0);
    AddStraight(Out, 60.0, EZone::BlockBrake, 3.0, true);
    AddStraight(Out, 60.0, EZone::BlockBrake, 3.0, true);
    return Out;
}

struct FBatchResult
{
    std::vector<double> LeftPlatformAt;   // per train, seconds; -1 if it never did
    std::vector<double> FinalS;
    bool bShared = false;
    std::size_t Violations = 0;

    // A SAFETY INVARIANT CHECKED EVERY FRAME, not an outcome measured at the end.
    //
    // A train permitted to leave, or actually leaving, must be secured AT THAT
    // MOMENT — bars locked where the role carries riders, gates shut always. The
    // station suite tested the PROCESS and never what the CREW commanded, so the
    // ride shipped for a week dispatching trains with their restraints travelling
    // open and every assertion in the project passed.
    //
    // Outcome assertions cannot catch that class. "It got round" and "no blocks
    // were shared" are both true of a ride running with the bars up.
    std::size_t UnsecuredFrames = 0;
    std::size_t SecuredFrames = 0;   // so a vacuous pass is visible
};

// Three trains, one at each position, and the platform run for real: geometry,
// physics, interlocking, drives and three independent station sequences.
//
// SlowPosition gets a load dwell long enough to matter — the rider who needs more
// time — and the whole question is what that does to the two trains in front.
//
// PLATFORM INDICES RUN IN TRACK ORDER, so 0 is the REARMOST position and 2 is the
// front one that dispatches. Train indices run the other way, 0 being the train
// at the front. Getting those two backwards is what the first run of this test
// did, and it looks exactly like a bug in the sequencing rather than in the test.
FBatchResult RunSmallBatchAt(const std::vector<double>& LoadAt)
{
    const std::vector<FItem> Items = SmallBatchLayout();
    const FCircuit Shape = BuildCircuitFrom(nullptr, Items);
    const FTrack T = BuildTrack(Shape.Doc);

    const double BatchLen = 8.0;   // small vehicle, and it has to fit a 20 m position
    FTrainConfig Cfg;
    Cfg.TrainLength = BatchLen;

    std::vector<std::unique_ptr<FTrain>> Owned;
    FCircuit C;
    for (std::size_t t = 0; t < 3; ++t)
    {
        Owned.push_back(std::unique_ptr<FTrain>(new FTrain(T, Cfg)));
        C = BuildCircuitFrom(Owned.back().get(), Items, BatchLen);
    }
    std::vector<FTrain*> All;
    for (std::size_t t = 0; t < 3; ++t) { All.push_back(Owned[t].get()); }

    // Four devices, so four blocks plus the plain stretch. THREE of them are the
    // platform, which is the whole point: identical devices, separate blocks.
    FRideSignals Sig(C.Boundaries, 5.0, 1, 3, false);
    Sig.SetHoldingBlocks(HoldingBlocks(*Owned[0], C, T.TotalLength()));

    FTrackSensors Marks(C.StopMarkS);
    FTrackDrives Drives = OpenDrives(*Owned[0], C);
    std::vector<FPlatform> Platforms = BuildPlatforms(C);
    // ONE DWELL PER POSITION, and in this rig that is one per TRAIN: three
    // positions, three trains, each position serves exactly one of them. So a
    // vector here is the whole of "every train loads for a different length of
    // time" without the crew having to be re-armed mid-run.
    for (std::size_t p = 0; p < Platforms.size(); ++p)
    {
        Platforms[p].Crew.LoadSeconds = p < LoadAt.size() ? LoadAt[p] : 8.0;
    }

    // Train 0 at the FRONT position, 2 at the rear, so index order is travel order
    // — the same convention the block indices use.
    for (std::size_t t = 0; t < 3; ++t)
    {
        Owned[t]->Place(C.HoldMidS[2 - t], 0.0);
        Sig.Update(t, Owned[t]->GetRearS(), Owned[t]->GetFrontS());
    }

    FBatchResult R;
    R.LeftPlatformAt.assign(3, -1.0);
    const double PlatformEnd = C.Boundaries[3];   // where the launch begins

    std::vector<bool> Loaded(3, false);

    const double Dt = 1.0 / 240.0;
    for (int F = 0; F < 240 * 200; ++F)
    {
        ScanStopMarks(Marks, All, false, T.TotalLength());
        ServeStations(Platforms, All, Marks, Drives, Dt, &Loaded);
        for (std::size_t t = 0; t < 3; ++t)
        {
            ServeHolds(*Owned[t], Sig, t, C, Marks, Drives, Platforms);
        }
        Drives.Tick(Dt);
        DriveTheTrack(Drives, All);

        for (std::size_t t = 0; t < 3; ++t)
        {
            Owned[t]->Step(Dt);
            Sig.Update(t, Owned[t]->GetRearS(), Owned[t]->GetFrontS());
            if (R.LeftPlatformAt[t] < 0.0 && Owned[t]->GetRearS() > PlatformEnd)
            {
                R.LeftPlatformAt[t] = F * Dt;
            }
        }
        Sig.Tick(Dt);
        ReadTheDrives(Drives, All);

        // THE INVARIANT, EVERY FRAME. Permission to leave, or actually leaving,
        // with anything unsecured is the failure the transition log found in the
        // shipped build — and no outcome assertion here would ever have noticed,
        // because a ride dispatching trains with the bars up still gets round.
        //
        // Gates are unconditional and the bars follow the ROLE: an unload platform
        // sends its train on empty, so its bars are open on purpose.
        for (const FPlatform& P : Platforms)
        {
            const EStationPhase Ph = P.Process.GetPhase();
            if (Ph != EStationPhase::Ready && Ph != EStationPhase::Departing)
            {
                continue;
            }
            const bool bBars = !P.Process.NeedsRestraints()
                || P.Crew.Restraints.IsClosedAndLocked();
            if (bBars && P.Crew.Gates.IsClosedAndLocked()) { ++R.SecuredFrames; }
            else                                           { ++R.UnsecuredFrames; }
        }

        for (std::size_t a = 0; a < 3; ++a)
        {
            for (std::size_t b = a + 1; b < 3; ++b)
            {
                for (std::size_t k = 0; k < Sig.NumBlocks(); ++k)
                {
                    if (Sig.OccupiedBy(a, k) && Sig.OccupiedBy(b, k)) { R.bShared = true; }
                }
            }
        }
    }
    R.Violations = Sig.Violations();
    for (std::size_t t = 0; t < 3; ++t) { R.FinalS.push_back(Owned[t]->GetDistance()); }
    return R;
}

void TestTheSameRunTwiceIsTheSameRun()
{
    // DETERMINISM, WHICH IS WHAT MAKES EVERYTHING ELSE PROVABLE. A recorded
    // scenario, a replayed bug report and a fault-injection test are all the same
    // claim underneath: run it again and get the same run. Without that they are
    // anecdotes.
    //
    // The digest covers EVERY SCAN rather than the ending state, because two runs
    // that diverged in the middle and converged on the same parked positions
    // would satisfy every other assertion in this file.
    const FRunResult A = RunTrains(3, 1, 120.0);
    const FRunResult B = RunTrains(3, 1, 120.0);

    assert(A.Digest == B.Digest);
    assert(A.Violations == B.Violations);
    assert(A.Laps == B.Laps);

    std::printf("\nDETERMINISM\n");
    std::printf("  three trains, 120 s, run twice: digest %016llx both times\n",
                static_cast<unsigned long long>(A.Digest));

    // AND THE DIGEST HAS TO BITE. A fingerprint that matched everything would
    // pass this test on a simulation that was wildly non-deterministic, which is
    // the vacuous pass this project has already been caught by once tonight.
    // A different train count is a different run and must say so.
    const FRunResult C = RunTrains(2, 1, 120.0);
    assert(C.Digest != A.Digest);

    // So is the same ride with one more block of headway, which changes when
    // trains are released without changing where any of them ends up.
    const FRunResult D = RunTrains(3, 2, 120.0);
    assert(D.Digest != A.Digest);
    std::printf("  and it differs on train count and on lookahead, so it bites\n");

    // A run stopped mid-lap is a different run from one that was not, even though
    // both end with every train stationary at a device.
    const FRunResult E = RunTrains(3, 1, 120.0, 60.0);
    assert(E.Digest != A.Digest);
}

void TestTheSmallBatchCircuitIsTheSameOvalAndStillCloses()
{
    // THE CLOSURE IS A PROPERTY OF THE LEG LENGTHS, so keeping leg A at 176 m
    // keeps it exactly. Asserted rather than assumed, because "I did not change
    // the geometry" is the sort of claim that is wrong 5% of the time and silent
    // when it is — a seam that misses by a metre looks fine from the cockpit and
    // teleports the train once a lap.
    const FCircuit Two = BuildCircuitFrom(nullptr, Layout());
    const FCircuit Batch = BuildCircuitFrom(nullptr, SmallBatchCircuitLayout(), BatchTrainLen);
    const FTrack T2 = BuildTrack(Two.Doc);
    const FTrack TB = BuildTrack(Batch.Doc);

    assert(std::fabs(TB.TotalLength() - T2.TotalLength()) < 1e-9);
    assert(TB.IsCurvatureContinuous(1e-9));

    const FTrackFrame A = TB.EvaluateAt(0.0);
    const FTrackFrame B = TB.EvaluateAt(TB.TotalLength());
    assert(Length(B.Position - A.Position) < 1e-3);
    assert(std::acos(std::min(1.0, Dot(B.Tangent, A.Tangent))) < 1e-4);
    assert(std::fabs(B.Roll - A.Roll) < 1e-4);

    // EIGHT places a train may stand where the two-train circuit has five: an
    // unload, three loading positions, and the four it already had.
    assert(Batch.Authored.size() == 9);
    assert(Batch.HoldMidS.size() == 8);
    assert(Batch.Boundaries.size() == 11);   // the two-train circuit's 8, plus 3

    // Every position is longer than a train plus its clearance, or the stop mark
    // lands outside the device it belongs to and no train ever trips it. This is
    // the constraint that decides how short the vehicles have to be.
    for (std::size_t z = 0; z < 4; ++z)
    {
        const double Len = Batch.StopMarkS[z] + NoseClearance
            - (z == 0 ? 0.0 : Batch.Boundaries[z]);
        assert(Len >= BatchTrainLen + NoseClearance);
    }

    // And it still gets round, which a shorter train is not guaranteed to: less
    // of it straddles each crest, so it pays more of the height.
    FTrainConfig Cfg;
    Cfg.TrainLength = BatchTrainLen;
    FTrain Tr(TB, Cfg);
    BuildCircuitFrom(&Tr, SmallBatchCircuitLayout(), BatchTrainLen);
    const FRideProfile P = RunRideProfile(Tr, TB, 1.0);
    assert(P.bCompleted);

    // EIGHT holding places, so seven trains — and the whole point of the layout is
    // that it can actually run a queue of them. Five, for ten minutes, is a busy
    // operation rather than a demonstration.
    const std::vector<FItem> Items = SmallBatchCircuitLayout();
    const FRunResult R = RunTrains(5, 1, 600.0, -1.0, &Items, BatchTrainLen);
    assert(R.Violations == 0);
    assert(!R.bShared);
    assert(!R.bDriveFaulted);
    for (std::size_t t = 0; t < 5; ++t)
    {
        assert(R.Laps[t] >= 1);   // nobody starved and nothing deadlocked
    }
}

// The original shape, kept because every published figure was taken through it
// and a wrapper is cheaper than re-verifying them all.
FBatchResult RunSmallBatch(double SlowLoadSeconds, std::size_t SlowPosition)
{
    std::vector<double> Load(3, 8.0);
    if (SlowPosition < Load.size()) { Load[SlowPosition] = SlowLoadSeconds; }
    return RunSmallBatchAt(Load);
}

void TestASmallBatchPlatformWorksThreeTrainsAtOnce()
{
    // THE SHAPE THIS WAS BUILT FOR, and most of it turns out to be emergent rather
    // than written: the interlocking already says "advance when the space ahead
    // frees", and each position already gates its own dispatch. Making the process
    // one per POSITION rather than one per platform is what left nothing to add.
    const std::vector<FItem> Items = SmallBatchLayout();
    const FCircuit C = BuildCircuitFrom(nullptr, Items);

    // Three identical devices stayed three. Under every rule but bNewDevice they
    // are one 60 m zone holding one train.
    assert(C.Authored.size() == 7);          // 3 positions, a launch, 3 brakes
    assert(C.HoldMidS.size() == 6);          // everything but the launch holds
    assert(std::fabs(C.Boundaries[1] - 20.0) < 1e-9);
    assert(std::fabs(C.Boundaries[2] - 40.0) < 1e-9);
    assert(std::fabs(C.Boundaries[3] - 60.0) < 1e-9);

    // 1. All three get away, in order, front first. Nothing overtakes anything.
    const FBatchResult Even = RunSmallBatch(8.0, 99);
    assert(Even.LeftPlatformAt[0] > 0.0);
    assert(Even.LeftPlatformAt[1] > Even.LeftPlatformAt[0]);
    assert(Even.LeftPlatformAt[2] > Even.LeftPlatformAt[1]);
    assert(!Even.bShared);
    assert(Even.Violations == 0);

    // AND NOT ONE FRAME OF IT UNSECURED. Checked continuously rather than at the
    // end, because every assertion above is equally true of a ride dispatching
    // trains with the bars travelling open — which is exactly what this build did
    // for a week until a transition log was pointed at it.
    //
    // SecuredFrames is asserted too, and it is not decoration: without it the
    // whole check passes vacuously the day a refactor stops platforms reaching
    // Ready at all, which is the failure mode this session already hit once on
    // the envelope suite.
    assert(Even.SecuredFrames > 1000);
    assert(Even.UnsecuredFrames == 0);
    std::printf("  small batch: %zu platform-frames permitted to move, "
                "0 of them unsecured\n", Even.SecuredFrames);

    // 2. THE RIDER WHO NEEDS LONGER, at the REAR position (platform 0). It must
    //    not delay the two trains in front by a single frame — they are ahead of
    //    it and nothing about their own sequences has changed. Asserted EXACTLY,
    //    because "about the same" would pass on a mechanism that coupled them
    //    weakly, and the claim is that they are not coupled at all.
    const FBatchResult SlowRear = RunSmallBatch(60.0, 0);
    assert(std::fabs(SlowRear.LeftPlatformAt[0] - Even.LeftPlatformAt[0]) < 1e-9);
    assert(std::fabs(SlowRear.LeftPlatformAt[1] - Even.LeftPlatformAt[1]) < 1e-9);
    assert(SlowRear.Violations == 0);

    //    AND THE WHOLE RIDE BARELY NOTICES, which is the throughput argument for
    //    building a platform like this and is worth more than the assertion above.
    //    Loading happens in PARALLEL with the queue clearing, so 52 extra seconds
    //    at the back costs the last departure about five. Positions are not a
    //    queue for one loading bay; they are three loading bays.
    const double RearCost = SlowRear.LeftPlatformAt[2] - Even.LeftPlatformAt[2];
    assert(RearCost > 0.0 && RearCost < 10.0);

    // 3. The same rider at the FRONT (platform 2) is a different story, and it is
    //    the other half of the same fact rather than a different one: a train
    //    cannot pass the train in front of it, so everybody behind waits. This is
    //    why a real operation moves a slow load to the BACK of the platform when it
    //    can see one coming.
    const FBatchResult SlowFront = RunSmallBatch(60.0, 2);
    const double FrontCost = SlowFront.LeftPlatformAt[2] - Even.LeftPlatformAt[2];
    assert(FrontCost > RearCost * 5.0);
    assert(SlowFront.LeftPlatformAt[0] > Even.LeftPlatformAt[0] + 40.0);
    assert(SlowFront.LeftPlatformAt[1] > Even.LeftPlatformAt[1] + 40.0);
    assert(SlowFront.Violations == 0);
    assert(!SlowFront.bShared);
}

// ===================== WHAT A SLOW LOAD COSTS, AND WHERE =====================
//
// The test above samples the asymmetry at two points — 52 extra seconds at the
// rear and the same at the front — and concludes rear-is-cheap, front-is-dear.
// That is true and it is not the question an operations team asks. They ask HOW
// MUCH LONGER a load may run before it costs anything, because that is what
// decides whether it is worth walking a slow party down the platform.
//
// Two points cannot tell a knee from a constant. The sweep below shows there is
// one, and that the shape is the same at every position: A BUDGET OF FREE TIME,
// AND 1:1 BEYOND IT. The budget is the only thing that differs, and it is the
// time the queue in front takes to clear — which is why the front's is zero.
double FreeLoadBudget(std::size_t Position, double Base)
{
    // BISECTION, not a fine sweep: each sample is a 200-second three-train run,
    // and thirty of them per position to find one number is a suite nobody waits
    // for. Half a second of cost is the threshold — below that is scheduling
    // jitter in when a mark trips, not a train being held.
    double Free = 0.0, Costly = 120.0;
    for (int Step = 0; Step < 8; ++Step)
    {
        const double Mid = 0.5 * (Free + Costly);
        const FBatchResult R = RunSmallBatch(8.0 + Mid, Position);
        assert(R.Violations == 0);
        assert(R.UnsecuredFrames == 0);
        ((R.LeftPlatformAt[2] - Base) > 0.5 ? Costly : Free) = Mid;
    }
    return Free;
}

void TestWhereASlowLoadStartsCosting()
{
    std::printf("What a slow load costs, and where\n");

    const FBatchResult Even = RunSmallBatch(8.0, 99);
    const double Base = Even.LeftPlatformAt[2];   // the last train away

    // THE LAST DEPARTURE IS THE MEASURE, not the average. A platform is finished
    // when the last train has gone, and a mean over three would hide the one that
    // is holding everybody up — which is the entire subject.
    std::printf("    extra load       rear (P0)   middle (P1)    front (P2)\n");
    double Cost50[3] = {0.0, 0.0, 0.0};
    for (int Extra = 10; Extra <= 50; Extra += 20)
    {
        double Cost[3] = {0.0, 0.0, 0.0};
        for (std::size_t Pos = 0; Pos < 3; ++Pos)
        {
            const FBatchResult R = RunSmallBatch(8.0 + Extra, Pos);
            assert(R.Violations == 0);
            assert(R.UnsecuredFrames == 0);
            Cost[Pos] = R.LeftPlatformAt[2] - Base;
        }
        std::printf("    +%2d s          %8.1f s    %8.1f s    %8.1f s\n",
                    Extra, Cost[0], Cost[1], Cost[2]);
        // A TRAIN CANNOT PASS THE TRAIN IN FRONT OF IT, and this is that fact
        // stated as a number at every delay rather than at one: a delay further
        // forward is never cheaper than the same delay further back.
        assert(Cost[0] <= Cost[1] + 1e-6);
        assert(Cost[1] <= Cost[2] + 1e-6);
        if (Extra == 50) { Cost50[0] = Cost[0]; Cost50[1] = Cost[1]; Cost50[2] = Cost[2]; }
    }

    // THE FRONT PAYS ONE FOR ONE. Nothing can pass it, so every second it spends
    // loading is a second the two behind it stand still — which makes its free
    // budget exactly zero, and that is a property rather than a measurement.
    assert(std::fabs(Cost50[2] - 50.0) < 1.0);

    const double Budget[3] = {FreeLoadBudget(0, Base), FreeLoadBudget(1, Base),
                              FreeLoadBudget(2, Base)};
    std::printf("    free budget:   %8.1f s    %8.1f s    %8.1f s\n",
                Budget[0], Budget[1], Budget[2]);

    // AND THE BUDGET IS THE ANSWER SOMEBODY CAN USE. "Move a slow party to the
    // back" is advice; "the back absorbs about forty seconds and the middle about
    // twenty before anybody waits" is a decision.
    assert(Budget[2] < 2.0);              // the front has none, by construction
    assert(Budget[1] > Budget[2] + 5.0);  // and each position back has more
    assert(Budget[0] > Budget[1] + 5.0);
}


// ===================== A DWELL IS A DISTRIBUTION, NOT A FIGURE =====================
//
// Everything above holds the load time CONSTANT and varies it deliberately. A real
// platform does not work that way: most loads are quick, some run long because a
// party is slow or a bar will not latch, and what an operations team lives with is
// the SPREAD rather than the mean.
//
// THE SHAPE HERE IS INVENTED AND IS LABELLED AS SUCH. This project has no dwell
// data from a real park and will not pretend to. What is measured is the
// platform's SENSITIVITY to variation of a given shape — a property of the layout
// and the interlocking rather than of anybody's ridership — so swapping the curve
// for measured data later answers the same question better without changing it.
//
// Deterministic on purpose: a fixed LCG, so a regression is a real change rather
// than a different draw.
struct FDwellDraw
{
    std::uint64_t State = 0x9E3779B97F4A7C15ull;

    double Next01()
    {
        State = State * 6364136223846793005ull + 1442695040888963407ull;
        return static_cast<double>((State >> 11) & ((1ull << 53) - 1))
               / static_cast<double>(1ull << 53);
    }

    // MOST LOADS ARE QUICK AND A FEW ARE NOT, which is the only property of a real
    // dwell curve this needs. Cubed uniform: half the draws land within 6 s of the
    // floor, and the tail reaches the long ones without a cliff.
    double Next(double Floor, double Tail)
    {
        const double U = Next01();
        return Floor + Tail * U * U * U;
    }
};

struct FVariationResult
{
    double MeanCost = 0.0;
    double WorstCost = 0.0;
    int Costly = 0;
};

// Vary ONE position and hold the others at the ordinary dwell, which is what
// isolates the effect. Varying all three at once mixes the answers together and
// reports the front's, because the front has no budget to absorb anything with.
FVariationResult VaryOnePosition(std::size_t Position, int Runs, double Base)
{
    FDwellDraw Rng;
    FVariationResult V;
    double Total = 0.0;
    for (int i = 0; i < Runs; ++i)
    {
        std::vector<double> Load(3, 8.0);
        Load[Position] = Rng.Next(8.0, 45.0);

        const FBatchResult R = RunSmallBatchAt(Load);
        // EVERY DRAW IS STILL A SAFE RIDE, and that is half the reason to run
        // these at all: each one is a differently-timed sequence through the same
        // interlocking, and none of them may violate.
        assert(R.Violations == 0);
        assert(R.UnsecuredFrames == 0);
        assert(!R.bShared);

        const double Cost = R.LeftPlatformAt[2] - Base;
        Total += Cost;
        V.WorstCost = std::fmax(V.WorstCost, Cost);
        if (Cost > 0.5) { ++V.Costly; }
    }
    V.MeanCost = Total / Runs;
    return V;
}

void TestOrdinaryVariationIsAbsorbedAtTheBackAndNotTheFront()
{
    std::printf("What ordinary variation costs, by position\n");

    const FBatchResult Even = RunSmallBatchAt(std::vector<double>(3, 8.0));
    const double Base = Even.LeftPlatformAt[2];

    const int Runs = 40;
    FVariationResult V[3];
    for (std::size_t Pos = 0; Pos < 3; ++Pos)
    {
        V[Pos] = VaryOnePosition(Pos, Runs, Base);
    }

    static const char* Name[3] = {"rear  ", "middle", "front "};
    for (std::size_t Pos = 0; Pos < 3; ++Pos)
    {
        std::printf("    %s  %2d of %d loads cost anything   mean %5.1f s   worst %5.1f s\n",
                    Name[Pos], V[Pos].Costly, Runs, V[Pos].MeanCost, V[Pos].WorstCost);
    }
    // FLUSHED, because the assertions below abort and a block-buffered stdout
    // loses the numbers that say why — the one moment they matter.
    std::fflush(stdout);

    // ===================== THE RESULT, AND IT IS NOT WHAT I EXPECTED =====================
    //
    // The first version of this varied all three positions at once and asserted
    // that ordinary variation is "mostly absorbed". It is not: 54 of 64 runs cost
    // something, because a run costs whatever its FRONT draw costs and the front
    // has no budget at all. That assertion was a guess and the measurement said
    // otherwise, which is the whole reason to run it.
    //
    // Stated properly: THE PLATFORM ABSORBS VARIATION AT THE BACK AND NONE AT THE
    // FRONT. It is the budget result again, in the language an operator thinks in
    // — and the operational reading is sharper than "put slow parties at the
    // back". It is that the FRONT POSITION IS WHERE PREDICTABILITY IS WORTH MOST,
    // because every second of variance there is a second off the whole platform.
    assert(V[2].Costly > (Runs * 3) / 4);   // the front pays for nearly every draw
    assert(V[0].Costly < Runs / 4);         // the back pays for hardly any

    // Asserted as an ORDERING as well, for the same reason the budget test is:
    // three magic counts would need re-deriving on any layout change, where the
    // shape is the thing being claimed.
    assert(V[0].Costly <= V[1].Costly);
    assert(V[1].Costly <= V[2].Costly);
    assert(V[0].MeanCost <= V[1].MeanCost);
    assert(V[1].MeanCost <= V[2].MeanCost);
}


void TestAnEmergencyStopStopsTheRideNotTheTrains()
{
    // THE PROPERTY THAT FALLS OUT OF THE MODEL RATHER THAN BEING ARRANGED, and the
    // reason a ride is built out of block brakes in the first place.
    //
    // An E-stop cuts power to every drive. It does NOT reach out and grab a train:
    // one on open course simply has nothing touching it, so it coasts, runs to the
    // next brake and is held there. One already in a brake run stops at once,
    // because a brake commanded to zero bites. Nothing anywhere had to special-case
    // "where is everybody" — cutting the outputs is the whole of it.
    //
    // Tripped at 200 s, deliberately mid-lap with trains spread round the circuit,
    // then 300 s more so everything has time to come to rest.
    const FRunResult R = RunTrains(3, 1, 500.0, 200.0);

    // 1. EVERYTHING STOPPED. Not slowed, not still creeping — stopped.
    for (std::size_t t = 0; t < R.FinalSpeed.size(); ++t)
    {
        assert(R.FinalSpeed[t] < 1e-6);
    }

    // 2. And every one of them stopped AT A DEVICE, which is the claim worth
    //    having. A train left standing on open course is a train that has to be
    //    walked out to and evacuated; one held at a brake is a ride that can be
    //    restarted. This layout has a device within reach from anywhere on it, and
    //    that is a property of the LAYOUT rather than of the E-stop.
    for (std::size_t t = 0; t < R.FinalHoldZone.size(); ++t)
    {
        assert(R.FinalHoldZone[t] >= 0);
    }

    // 3. Nobody ran into anybody on the way down. An E-stop that produced a
    //    collision would be worse than no E-stop.
    assert(R.Violations == 0);
    assert(!R.bShared);

    // ---- 4. AND CAN ANYBODY GET TO THEM? --------------------------------
    //
    // The half this test could not ask until catwalks were modelled. Points 1-3
    // are statements about SIGNALLING: everything stopped, at a device, without
    // colliding. None of them says whether the riders can then be walked off,
    // and "stopped safely" and "reachable on foot" are different properties of a
    // layout that happen to be about the same event.
    const FCircuit C = BuildCircuit(nullptr);
    const FTrack ET = BuildTrack(C.Doc);
    FTrainConfig ECfg;
    ECfg.TrainLength = TrainLen;
    FTrain ETrain(ET, ECfg);
    BuildCircuit(&ETrain);          // gives it the zones, so hold-capability is askable
    const double ETotal = ET.TotalLength();

    std::vector<FStoppedTrain> Stopped;
    for (std::size_t t = 0; t < R.FinalRearS.size(); ++t)
    {
        FStoppedTrain S;
        S.RearS = R.FinalRearS[t];
        S.FrontS = R.FinalFrontS[t];
        S.Index = static_cast<int>(t);
        Stopped.push_back(S);
    }

    // WITH NO WALKWAYS, EVERY TRAIN IS UNREACHABLE — and that is the honest
    // reading of this layout as it ships. No preset carries catwalks, because
    // nobody has surveyed one, and a check that quietly passed on a track with
    // no evacuation provision at all would be worthless.
    {
        const FEvacVerdict V = CheckEvacuation({}, Stopped, ETotal);
        assert(!V.bEveryoneCanWalkOff);
        assert(V.Findings.size() == R.FinalRearS.size());
    }

    // NOW FIT ONE OVER EVERY HOLDING BLOCK. Not a suggestion the model made —
    // a person places these — but the natural place to put them, and the claim
    // worth measuring: does this layout's own set of holding devices, given
    // catwalks, actually serve every train an E-stop can leave behind?
    std::vector<FWalkwaySpan> Walks;
    for (std::size_t b = 0; b < C.Boundaries.size(); ++b)
    {
        const double Start = C.Boundaries[b];
        const double End = (b + 1 < C.Boundaries.size()) ? C.Boundaries[b + 1] : ETotal;
        // Only blocks that can hold a train: a catwalk down a 696 m free run is
        // not what any real ride does, and the question is whether the places
        // trains ACTUALLY stop are served.
        if (ETrain.FindHoldZoneAt(0.5 * (Start + End)) < 0) { continue; }
        FWalkwaySpan W;
        W.StartS = Start;
        W.EndS = End;
        W.Side = EWalkway::Both;
        Walks.push_back(W);
    }
    assert(Walks.size() == 5);   // station, mid-course, outer, transfer, inner

    const FEvacVerdict V = CheckEvacuation(Walks, Stopped, ETotal);
    std::printf("  E-STOP EVACUATION: %zu holding blocks catwalked, %.0f m of route, "
                "%s\n", Walks.size(), V.TrackCoverageM,
                V.bEveryoneCanWalkOff ? "every train reachable"
                                      : "SOMEBODY IS STRANDED");
    for (const FEvacFinding& F : V.Findings)
    {
        std::printf("    train %d stranded: %.1f m unserved from S=%.0f\n",
                    F.Train, F.UnservedMetres, F.WorstGapStartS);
    }
    assert(V.bEveryoneCanWalkOff);
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
    // A LINE-FOR-LINE STAND-IN FOR ATUCoasterRide::SimStep, because that function
    // cannot be compiled without Unreal and this is the only place its policy can
    // be checked at all. Same order, same defaults:
    //
    // The actor's Tick is now an accumulator that calls SimStep at a FIXED 240 Hz
    // — the same rate this loop has always used. Until that change the actor ran
    // one scan per rendered frame, so this test and the thing you played were
    // stepping at different rates and only one of them was reproducible. They now
    // agree by construction, which is what makes "line-for-line" true rather than
    // aspirational.
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
    std::vector<FPlatform> Platforms = BuildPlatforms(C);

    for (int Frame = 0; Frame < 240 * 600; ++Frame)
    {
        // THE SCAN CYCLE, and it is the whole shape of the tick: read the inputs,
        // run the program for every train against that one snapshot, let the drives
        // ramp, write the outputs, then step the world. Interleaving the program
        // with the physics — serving train 1 after train 0 has already moved — is
        // what a game does and what a PLC cannot.
        ScanStopMarks(Marks, All, true, T.TotalLength());
        ServeStations(Platforms, All, Marks, Drives, Dt);
        for (std::size_t t = 0; t < 2; ++t)
        {
            ServeHolds(*Trains[t], Sig, t, C, Marks, Drives, Platforms);
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

// THE SHOWCASE CIRCUIT: the small-batch oval with the trim split out of the
// fill straight, exactly as ATUCoasterRide::ShowcaseLayout() derives it. The
// trim is the piece that matters to signalling: it BOUNDS a block and cannot
// hold a train, so a train dispatched into its block is committed to the next
// device -- which is why "trains = holding places - 1" over-counts here. The
// formula counts places; a committed block consumes headway the formula never
// sees.
//
// The actor's showcase also specifies per-device rates (a 10 m/s^2 launch, an
// 8 m/s^2 pad on the mid-course). This transcription runs the default grip,
// deliberately: if the capacity ceiling reproduces WITHOUT the rates, the cause
// is the block topology, not the tuning -- which is the claim under test.
//
// bWithHelix adds one full turn (2*pi*R of arc) to the final banked turn. A
// full circle returns to its own start point and heading, so the seam is
// untouched; what it changes is LAP TIME, and this harness is the first thing
// to measure that against the interlocking rather than against a single train
// with every signal green.
std::vector<FItem> ShowcaseCircuitLayout(bool bWithHelix)
{
    const double Up = Deg(26.0);
    const double Dn = Deg(32.0);
    const double R = 35.0;
    const double Ease = 50.0;
    const double Arc = Pi * R - Ease;
    const double DropLen = 15.6847323;
    const double FillLen = 75.5024975;
    const double TrimLen = 40.0;

    // THE RATES ARE THE POINT. The first version of this transcription carried
    // none, ran every device at the one Grip constant, and measured six and
    // seven trains CLEAN on a layout whose actor tripped a signalling violation
    // at six -- a harness whose devices are all the same machine measures a
    // different ride. These are ShowcaseLayout()'s numbers, transcribed.
    std::vector<FItem> Out;
    AddStraight(Out, 10.0, EZone::StationUnload, 1.5, false, 1.0, 1.0);
    AddStraight(Out, 10.0, EZone::StationLoad, 1.5, false, 1.0, 1.0);
    AddStraight(Out, 10.0, EZone::StationLoad, 1.5, true, 1.0, 1.0);
    AddStraight(Out, 10.0, EZone::StationLoad, 1.5, true, 1.0, 1.0);
    AddStraight(Out, 136.0, EZone::Launch, 38.0, false, 10.0);
    AddEasedPitch(Out, Up, 0.0130);
    AddStraight(Out, 40.0);
    AddEasedPitch(Out, -Up, 0.0130);
    AddBankedTurn(Out, R, Arc, Ease, BankDegreesFor(14.2, R));
    AddEasedPitch(Out, -Dn, 0.0150);
    AddStraight(Out, DropLen);
    AddEasedPitch(Out, Dn, 0.0150);
    AddEasedPitch(Out, Deg(20.0), 0.024);
    AddEasedPitch(Out, Deg(-40.0), 0.024);
    AddEasedPitch(Out, Deg(20.0), 0.024);
    // The split: trim first, then what is left of the fill -- the same order
    // ShowcaseLayout()'s Insert produces. Total length unchanged, so the
    // closure that lives in the leg lengths comes along untouched.
    AddStraight(Out, TrimLen, EZone::Brake, 24.0, false, -1.0, -1.0, 3.0);
    AddStraight(Out, FillLen - TrimLen);
    // The two-machine mid-course: a pad biting at 8 and tyres conveying at 1.5,
    // which is an ordinary specification and the whole reason the two rates are
    // separate fields.
    AddStraight(Out, 130.0, EZone::BlockBrake, 20.0, false, 1.5, 1.5, 8.0);
    const std::size_t Turn2 = Out.size();
    // bWithHelix IS NO LONGER THE SHIPPED LAYOUT. It adds a full turn to a LEVEL
    // arc, which is a circle driven round twice rather than a helix, and it is
    // kept here only so the measurement that withdrew it stays runnable. See
    // TestTheShowcaseCapacityAndTheHelix.
    AddBankedTurn(Out, R, bWithHelix ? Arc + 2.0 * Pi * R : Arc, Ease,
                  BankDegreesFor(18.1, R));

    // KICKER TYRES OUT OF THE MID-COURSE, on both variants now. A train
    // restarting from a standing hold at the MCBR gets a few metres of tyre push
    // and nothing else; real rides bolt drive tyres to the brake's exit for
    // exactly this, and here that is the turn's own entry easement carrying a
    // Launch zone. A Launch cannot hold a train, so the capacity table does not
    // move; it is a zone on EXISTING geometry, so the closure does not either;
    // and a Launch has no braking authority, so a train already above 22 m/s
    // passes untouched.
    Out[Turn2].Zone = EZone::Launch;
    Out[Turn2].Speed = 22.0;
    Out[Turn2].Accel = 10.0;
    AddStraight(Out, 24.0);
    AddStraight(Out, 37.5, EZone::BlockBrake, 6.0);
    AddStraight(Out, 27.0, EZone::Lift, 4.0, false, 1.0, 1.0);
    AddStraight(Out, 37.5, EZone::BlockBrake, 2.0);
    return Out;
}

// THE MISSING CONSUMER. Two things shipped tonight on the strength of a
// single-train ride profile -- a check that runs one train with every signal
// green and structurally cannot see headway -- and both tripped the real ride:
// six trains violated on the plain layout, and the helix was withdrawn
// unmeasured because it was implicated. This is the check both needed first.
void TestTheShowcaseCapacityAndTheHelix()
{
    const std::vector<FItem> Plain = ShowcaseCircuitLayout(false);
    const std::vector<FItem> Helix = ShowcaseCircuitLayout(true);

    // The geometry claims first: the split keeps the length, the helix adds
    // exactly one turn, and BOTH still close at the seam.
    {
        const FTrack A = BuildTrack(BuildCircuitFrom(nullptr, Plain, BatchTrainLen).Doc);
        const FTrack B = BuildTrack(BuildCircuitFrom(nullptr, Helix, BatchTrainLen).Doc);
        const double R = 35.0;
        std::printf("\nSHOWCASE: plain %.2f m, helix %.2f m (+%.2f, one turn is %.2f)\n",
                    A.TotalLength(), B.TotalLength(),
                    B.TotalLength() - A.TotalLength(), 2.0 * Pi * R);
        assert(std::fabs(B.TotalLength() - A.TotalLength() - 2.0 * Pi * R) < 1e-6);
        const FTrackProfile Cross;
        for (const FTrack* T : {&A, &B})
        {
            const FTrackFrame S = T->EvaluateAt(0.0);
            const FTrackFrame E = T->EvaluateAt(T->TotalLength());
            const double Seam = std::sqrt((E.Position.X - S.Position.X) * (E.Position.X - S.Position.X)
                                        + (E.Position.Y - S.Position.Y) * (E.Position.Y - S.Position.Y)
                                        + (E.Position.Z - S.Position.Z) * (E.Position.Z - S.Position.Z));
            assert(Seam < 1e-3);

            // AND DOES IT HIT ITSELF. This test measured capacity, laps and
            // violations -- every question the SIGNALLING can answer -- and never
            // asked the geometric one, which is how a "helix" that is a flat
            // circle driven round twice passed everything it was shown.
            const bool bIsShipped = (T == &A);
            const FClearanceReport Cl = AnalyseSelfClearance(*T, Cross, 0.5, 12.0, true);
            std::printf("  %s: closest self-approach %.2f m at %.1f / %.1f m%s\n",
                        bIsShipped ? "plain" : "helix", Cl.ClosestApproach, Cl.AtS, Cl.AndS,
                        Cl.bStructureOverlaps ? "  *** STRUCTURE OVERLAPS ***" : "");
            if (bIsShipped)
            {
                assert(!Cl.bStructureOverlaps);
                assert(Cl.ClosestApproach > 10.0);   // 11.68 m, in the far turn
            }
            else
            {
                // THE WITHDRAWAL, AS A MEASUREMENT RATHER THAN A MEMORY. Adding a
                // full turn to a LEVEL arc puts the last 98 degrees on top of the
                // first: 0.09 m apart at two arc lengths exactly one turn apart.
                //
                // And it is not a bug with a fix in plan. Any closure-neutral
                // addition of turning returns to its own start point and heading,
                // so it MUST touch itself -- vertical separation is the only
                // answer and that breaks the hand-solved closure this layout
                // exists to inherit. If this assertion ever fails, somebody has
                // given the helix height, and every published figure needs
                // re-measuring before it ships.
                assert(Cl.bStructureOverlaps);
                assert(std::fabs((Cl.AndS - Cl.AtS) - 2.0 * Pi * R) < 1.0);
            }
        }
    }

    for (int Variant = 0; Variant < 2; ++Variant)
    {
        const std::vector<FItem>* Items = Variant == 0 ? &Plain : &Helix;
        for (std::size_t N = 1; N <= 7; ++N)
        {
            const FRunResult R = RunTrains(N, 1, 600.0, -1.0, Items, BatchTrainLen);
            int TotalLaps = 0;
            for (int L : R.Laps) { TotalLaps += L; }
            std::printf("  %s %zu trains: %zu violation(s), %d laps, "
                        "diverge %s, counter %s%s, drive %s\n",
                        Variant == 0 ? "plain" : "helix", N, R.Violations, TotalLaps,
                        R.FirstDivergence < 0.0 ? "never" : "YES",
                        (R.bCounterOverOccupied || R.bCounterInconsistent) ? "BAD" : "ok",
                        R.bOverspeed ? ", OVERSPEED E-STOP" : "",
                        R.bDriveFaulted ? "FAULTED" : "ok");
            if (R.bDriveFaulted)
            {
                std::printf("    first faulted drive: zone %d\n", R.FirstFaultedDrive);
            }
            if (R.bOverspeed)
            {
                std::printf("    trap: zone %d at %.1f m/s, needs %.0f m of %.0f m, %.1f s in\n",
                            R.OverspeedZone, R.OverspeedSpeed, R.OverspeedNeeded,
                            R.OverspeedHave, R.FirstOverspeed);
            }
            if (R.Violations > 0)
            {
                std::printf("    first: train %d at %.1f m, %.1f s in\n",
                            R.FirstViolationTrain, R.FirstViolationS, R.FirstViolation);
            }
            // Zero laps is not a pass, it is a parked ride — say where.
            if (TotalLaps == 0)
            {
                for (std::size_t t = 0; t < R.FinalS.size(); ++t)
                {
                    std::printf("    train %zu ended at %.1f m, %.2f m/s, hold zone %d\n",
                                t, R.FinalS[t], R.FinalSpeed[t], R.FinalHoldZone[t]);
                }
            }

            // ===================== WHAT IS ASSERTED, AND WHAT IS ONLY REPORTED ==
            //
            // PLAIN IS CLEAN THROUGH SIX TRAINS in this harness, with real laps —
            // which is exactly what the actor DID NOT do: it tripped a signalling
            // violation at six. That disagreement is a fidelity gap this file has
            // not closed yet, and it is stated here rather than papered over. The
            // assertion holds the harness to its own measurement; the actor's trip
            // still needs reproducing before capacity claims transfer.
            //
            // THE HELIX VARIANT IS NOT SHIPPED and its runs are evidence rather
            // than a claim about the product — the geometry block above measures
            // why. It is still run because the reason it was withdrawn is
            // geometric, not operational, and a variant that quietly started
            // violating would be worth knowing about if it ever comes back with
            // height on it.
            //
            // THE KICKER SURVIVED THE WITHDRAWAL, on both. It was built for the
            // helix — a train restarting from a standing hold at the mid-course
            // gets a few metres of tyre push and stalled at rest halfway round
            // 400 m of banked helix — and the reason generalises: a real MCBR
            // sits high with a drop after it, and the ones that do not get drive
            // tyres bolted to their exit.
            const bool bAsserted = N <= 6;
            if (bAsserted)
            {
                assert(R.Violations == 0);
                assert(!R.bOverspeed);
                assert(R.FirstDivergence < 0.0);
                assert(!R.bCounterOverOccupied && !R.bCounterInconsistent);
                assert(TotalLaps > 0 && "a clean run with no laps is a parked ride");
                // MEASURED ALL ALONG AND THROWN AWAY. RunTrains has filled
                // bDriveFaulted since drives existed, two other tests assert it,
                // and the one test written to catch what the actor trips on did
                // not look at it -- so "the harness says the showcase is clean"
                // was a claim about violations only. Free: it has always been
                // false here.
                assert(!R.bDriveFaulted);
            }
        }
    }
}

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
    TestAFailedBrakeAndWhatDoesAndDoesNOTCatchIt();
    TestTheRideIsIDENTICALWithTheShowLayerAbsent();
    TestASessionCanBeRECORDEDAndREPLAYEDBitForBit();
    TestSeveralHoldingDevicesInARowStaySeveral();
    TestADispatchWaitsForTheLaunchToBeArmed();
    TestTwoIndependentMeansOfKnowingAgreeOnEveryBlock();
    TestTheDrivesTellTheStoryOfTheRide();
    TestTheSameRunTwiceIsTheSameRun();
    TestTheSmallBatchCircuitIsTheSameOvalAndStillCloses();
    TestASmallBatchPlatformWorksThreeTrainsAtOnce();
    TestWhereASlowLoadStartsCosting();
    TestOrdinaryVariationIsAbsorbedAtTheBackAndNotTheFront();
    TestAnEmergencyStopStopsTheRideNotTheTrains();
    TestTheCircuitCarriesFourTrains();
    TestTheActorsOwnLoopRunsTwoTrains();
    TestTheShowcaseCapacityAndTheHelix();

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

    // The small-batch platform, which is a different layout and a different point:
    // what three loading positions BUY, in seconds.
    {
        const FBatchResult Even = RunSmallBatch(8.0, 99);
        const FBatchResult Rear = RunSmallBatch(60.0, 0);
        const FBatchResult Front = RunSmallBatch(60.0, 2);
        std::printf("\nSMALL-BATCH PLATFORM, three loading positions\n");
        std::printf("  even 8 s loads      departures at %.1f, %.1f, %.1f s\n",
                    Even.LeftPlatformAt[0], Even.LeftPlatformAt[1], Even.LeftPlatformAt[2]);
        std::printf("  60 s load at REAR   %.1f, %.1f, %.1f s   costs the ride %.1f s\n",
                    Rear.LeftPlatformAt[0], Rear.LeftPlatformAt[1], Rear.LeftPlatformAt[2],
                    Rear.LeftPlatformAt[2] - Even.LeftPlatformAt[2]);
        std::printf("  60 s load at FRONT  %.1f, %.1f, %.1f s   costs the ride %.1f s\n",
                    Front.LeftPlatformAt[0], Front.LeftPlatformAt[1], Front.LeftPlatformAt[2],
                    Front.LeftPlatformAt[2] - Even.LeftPlatformAt[2]);
    }

    std::printf("\ntest_twotrains: all assertions passed\n");
    return 0;
}
