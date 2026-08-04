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
// WHAT THIS DOES NOT DO: there is no lap. FTrain clamps at the end of the track,
// so a train finishing the course stops in the last block rather than arriving
// back in the station. That is honest — the layout closes in HEIGHT but has never
// been closed in position, heading and roll, and wrapping a train across a seam
// that does not meet would fake continuity rather than simulate it. What is
// tested here is one full pre-station queueing cycle, which is where the
// interlocking actually does its work.

#include "../BlockSignal/RideSignals.h"
#include "../TrackSpline/TrackIO.h"
#include "RideProfile.h"
#include "TrainPhysics.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

namespace
{

const double Pi = 3.14159265358979323846;
const double Grip = 6.0;          // as ATUCoasterRide sets it for every zone
const double TrainLen = 15.0;

double Deg(double D) { return D * Pi / 180.0; }

// Mirrors ETUSegmentZone. BlockBrake is the kind the actor gained for this: a
// brake run WITH drive tyres, which is what every real block brake is and the
// only thing that can both stop a train and let it go again. It is a separate
// enumerator rather than a reused Lift because block boundaries fall where the
// KIND changes, and two holding devices in a row have to stay two blocks.
enum class EZone { None, Lift, Launch, Brake, BlockBrake };

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
    const double Up = Deg(28.0);
    const double Dn = Deg(-30.0);
    const double Climb = 41.7685;      // solved so the ride ends at station height
    const double BrakeLen = 37.5;      // 2.5 train lengths
    const double TransferLen = 27.0;   // 1.8 train lengths

    std::vector<FItem> Out;
    AddStraight(Out, 26.0, EZone::Lift, 1.5);              // 0 STATION, drive tyres
    AddStraight(Out, 150.0, EZone::Launch, 38.0);          // 1 LAUNCH
    AddEasedPitch(Out, Up, 0.0195);
    AddStraight(Out, Climb);
    AddEasedPitch(Out, Dn - Up, 0.0300);
    AddStraight(Out, 34.0);
    AddEasedPitch(Out, -Dn, 0.012);
    AddBankedTurn(Out, 48.0, Pi * 48.0, 34.0, BankDegreesFor(31.5, 48.0));
    AddStraight(Out, 45.0, EZone::Brake, 20.0);            // 3 MID-COURSE TRIM
    AddEasedPitch(Out, Deg(12.0), 0.010);
    AddEasedPitch(Out, Deg(-12.0), 0.010);
    AddBankedTurn(Out, 30.0, Pi * 30.0, 24.0, BankDegreesFor(10.6, 30.0));
    AddStraight(Out, 24.0);
    AddStraight(Out, BrakeLen, EZone::BlockBrake, 6.0);    // 5 OUTER, holds
    AddStraight(Out, TransferLen, EZone::Lift, 4.0);       // 6 TRANSFER tyres, holds
    AddStraight(Out, BrakeLen, EZone::BlockBrake, 2.0);    // 7 INNER, holds
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
    std::vector<double> HoldStartS;  // where a train may stand: drive tyres only
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
            // Brakes AND drive tyres, so identical in shape to a lift. The
            // enumerator is separate for the block boundary, not for the physics.
            if (Tr) { Tr->AddZone(MakeLift(OpenS, EndS, OpenSpeed, Grip)); }
            break;
        default:
            return;
        }
        C.Authored.push_back(OpenSpeed);
        if (Open == EZone::Lift || Open == EZone::BlockBrake)
        {
            C.HoldStartS.push_back(OpenS);
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
void ServeHolds(FTrain& Tr, const FRideSignals& Sig, std::size_t Id,
                const std::vector<double>& Authored)
{
    const int Z = Tr.FindHoldZoneAt(Tr.GetDistance());
    if (Z < 0)
    {
        return;   // not standing at a holding device; nothing to command
    }
    const bool bGranted = Sig.CanRelease(Id, Tr.GetDistance());
    Tr.SetZoneTargetSpeed(static_cast<std::size_t>(Z),
                          bGranted ? Authored[static_cast<std::size_t>(Z)] : 0.0);
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

    assert(T.NumSegments() == 25);
    assert(std::fabs(T.TotalLength() - 1072.46) < 0.1);
    assert(T.IsCurvatureContinuous(1e-9));
    assert(std::fabs(T.EvaluateAt(T.TotalLength()).Position.Z) < 1e-6);

    assert(C.Boundaries.size() == 8);
    assert(C.Authored.size() == 6);   // station, launch, trim, outer, transfer, inner

    // Eight blocks is what makes this the only preset that can hold two trains.
    // Every other one has three, because a boundary falls only where a powered run
    // starts or ends and they each have just two powered runs.
    const double Want[8] = {0.0, 26.0, 176.0, 675.44, 720.44, 970.46, 1007.96, 1034.96};
    for (std::size_t i = 0; i < 8; ++i)
    {
        assert(std::fabs(C.Boundaries[i] - Want[i]) < 0.05);
    }
}

void TestTheMidCourseBrakeCannotHoldAndSaysSo()
{
    // RECORDED, NOT FIXED, and the reason the holds are where they are. A block
    // brake has to stop the train in the length it has: 28.19 m/s arrives at
    // 675.44 m and 6 m/s^2 needs 66.2 m, against the 45 m the block is. Making it
    // a BlockBrake would produce a device that closes and gets run straight
    // through — worse than a trim, because it would look like an interlock.
    //
    // If this ever fails because the block was lengthened, delete it and move the
    // hold there; that is a better circuit, not a broken test.
    const FCircuit Shape = BuildCircuit(nullptr);
    const FTrack T = BuildTrack(Shape.Doc);
    FTrainConfig Cfg;
    Cfg.TrainLength = TrainLen;
    FTrain Tr(T, Cfg);
    BuildCircuit(&Tr);
    const FRideProfile P = RunRideProfile(Tr, T, 1.0);

    const double Arrives = SpeedAt(P, 675.44);
    const double Needs = Arrives * Arrives / (2.0 * Grip);
    assert(Arrives > 28.0 && Arrives < 28.4);
    assert(Needs > 45.0);                       // longer than the block itself

    // So it is authored as a plain Brake, and FindHoldZoneAt refuses it.
    assert(Tr.FindHoldZoneAt(700.0) < 0);

    // The three pre-station devices all pass the same test, which is why they are
    // the ones that hold.
    const double Where[3] = {970.46, 1007.96, 1034.96};
    for (double S : Where)
    {
        const double V = SpeedAt(P, S);
        assert(V * V / (2.0 * Grip) < 10.0);
        assert(Tr.FindHoldZoneAt(S + 5.0) >= 0);
    }
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
    const double StartB = 950.0;

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

    A.Place(1020.0, 0.0);                       // standing at the transfer tyres
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

    for (int Frame = 0; Frame < 240 * 90; ++Frame)
    {
        ServeHolds(A, Sig, 0, C.Authored);
        ServeHolds(B, Sig, 1, C.Authored);

        A.Step(Dt);
        B.Step(Dt);

        // Both trains, then ONE tick. Overlaps live on blocks, not on trains.
        assert(Sig.Update(0, A.GetRearS(), A.GetFrontS()));
        assert(Sig.Update(1, B.GetRearS(), B.GetFrontS()));
        Sig.Tick(Dt);

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
    assert(B.GetDistance() > 1007.96);

    // 4. A ran on ahead and B never caught it.
    assert(A.GetDistance() > B.GetDistance());
    assert(Sig.BlockAt(A.GetDistance()) == 7);

    // 5. And nothing anywhere was granted into an occupied block. This is the
    //    assertion the old single-train FRideSignals could not have failed,
    //    because it reported zero violations while two trains sat in one block.
    assert(!bEverShared);
    assert(Sig.Violations() == 0);
}

void TestTheActorsOwnLoopRunsTwoTrains()
{
    // A LINE-FOR-LINE STAND-IN FOR ATUCoasterRide::Tick, because that function
    // cannot be compiled without Unreal and this is the only place its policy can
    // be checked at all. Same order, same defaults, same teleport:
    //
    //   for each train: ServeHolds, Step, Signals->Update
    //   Signals->Tick once
    //   any train arrived in the last block -> back to the station after a dwell
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

    assert(C.HoldStartS.size() == 4);   // station, outer, transfer, inner
    assert(C.HoldStartS[0] == 0.0);

    // The actor's defaults: 5 s overlap, lookahead 2, 3 s dwell before restart.
    FRideSignals Sig(C.Boundaries, 5.0, 2, 2);
    assert(Sig.Lookahead() == 2);

    CloseAllHolds(A, C.Boundaries, T.TotalLength());
    CloseAllHolds(B, C.Boundaries, T.TotalLength());

    // Train 0 in the station; the rest at the holding places in track order.
    A.Place(0.0, 0.0);
    B.Place(C.HoldStartS[1] + TrainLen, 0.0);
    Sig.Update(0, A.GetRearS(), A.GetFrontS());
    Sig.Update(1, B.GetRearS(), B.GetFrontS());

    const double Dt = 1.0 / 240.0;
    const double Dwell = 3.0;
    const std::size_t Last = Sig.NumBlocks() - 1;
    double StoppedFor[2] = {0.0, 0.0};
    int Laps[2] = {0, 0};
    bool bShared = false;

    for (int Frame = 0; Frame < 240 * 600; ++Frame)
    {
        for (std::size_t t = 0; t < 2; ++t)
        {
            ServeHolds(*Trains[t], Sig, t, C.Authored);
            Trains[t]->Step(Dt);
            Sig.Update(t, Trains[t]->GetRearS(), Trains[t]->GetFrontS());
        }
        Sig.Tick(Dt);

        for (std::size_t b = 0; b < Sig.NumBlocks(); ++b)
        {
            if (Sig.OccupiedBy(0, b) && Sig.OccupiedBy(1, b))
            {
                bShared = true;
            }
        }

        for (std::size_t t = 0; t < 2; ++t)
        {
            const bool bSettled = Sig.BlockAt(Trains[t]->GetDistance()) == Last
                && (Trains[t]->GetSpeed() <= 0.0 || Trains[t]->IsAtEnd());
            if (!bSettled)
            {
                StoppedFor[t] = 0.0;
                continue;
            }
            StoppedFor[t] += Dt;
            if (StoppedFor[t] >= Dwell && !Sig.Occupies(0))
            {
                Trains[t]->Place(0.0, 0.0);
                Sig.Update(t, Trains[t]->GetRearS(), Trains[t]->GetFrontS());
                StoppedFor[t] = 0.0;
                ++Laps[t];
            }
        }
    }

    // Ten minutes of ride at a ~90 s circuit. BOTH trains keep going round — the
    // deadlock this is looking for would show up as one of these stuck at zero
    // while the other cycles, which is what a fail-closed rule set does when two
    // trains each hold what the other is waiting for.
    assert(Laps[0] >= 3);
    assert(Laps[1] >= 3);

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
    TestTheMidCourseBrakeCannotHoldAndSaysSo();
    TestTwoTrainsQueueBeforeTheStation();
    TestTheActorsOwnLoopRunsTwoTrains();

    std::printf("test_twotrains: all assertions passed\n");
    return 0;
}
