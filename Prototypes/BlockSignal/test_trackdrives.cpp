// Build & run:  clang++ -std=c++17 -Wall -Wextra -O2 -o test_trackdrives test_trackdrives.cpp && ./test_trackdrives
//
// Drives FTrackDrives with bare numbers. No FTrain, no FTrack, no engine — the
// same discipline as test_tracksensors and test_ridesignals, and for the same
// reason: a drive takes a speed and reports a speed, and anything it needs beyond
// that is a design mistake.

#include "TrackDrives.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace
{

const double Dt = 1.0 / 240.0;

void TestACommandIsARequestNotAnOutput()
{
    // THE WHOLE POINT OF THE LAYER. Before this existed, ServeHolds wrote a speed
    // and the track was at that speed the same instant. A command is a request;
    // how fast the drive gets there is the drive's business.
    FDriveSpec S;
    S.AccelRampMs2 = 2.0;
    FTrackDrives D(1);
    assert(D.Configure(0, S));

    assert(D.Command(0, 10.0));
    assert(D.Read(0).Commanded == 10.0);
    assert(D.Output(0) == 0.0);          // asked for, not yet delivered

    D.Tick(Dt);
    assert(D.Output(0) > 0.0);
    assert(D.Output(0) < 10.0);          // on its way

    // 2 m/s^2 to 10 m/s is five seconds, and it must not overshoot its command.
    for (int i = 0; i < 240 * 6; ++i) { D.Tick(Dt); }
    assert(std::fabs(D.Output(0) - 10.0) < 1e-9);

    // Down again, at the DECEL ramp, which is a separate number because a drive
    // that coasts down slowly can still stop hard.
    S.DecelRampMs2 = 10.0;
    assert(D.Configure(0, S));
    D.Command(0, 0.0);
    for (int i = 0; i < 240; ++i) { D.Tick(Dt); }
    assert(std::fabs(D.Output(0)) < 1e-9);
}

void TestNoRampIsTheOldBehaviourExactly()
{
    // The default, and it matters: every number this project measured before
    // drives existed was measured with the output equal to the command, so a
    // default that ramped would silently move all of them.
    FTrackDrives D(1);
    D.Command(0, 17.5);
    D.Tick(Dt);
    assert(D.Output(0) == 17.5);

    D.Command(0, 0.0);
    D.Tick(Dt);
    assert(D.Output(0) == 0.0);
}

void TestPresetSkipsTheRamp()
{
    // A ride opens with its drives already running. Without this every drive ramps
    // up from zero on the first frame of the session, which is a lift chain
    // standing still when the first train reaches it.
    FDriveSpec S;
    S.AccelRampMs2 = 0.1;                // absurdly slow, so a ramp would show
    FTrackDrives D(1);
    D.Configure(0, S);

    assert(D.Preset(0, 4.0));
    assert(D.Output(0) == 4.0);
    assert(D.Read(0).Commanded == 4.0);
    D.Tick(Dt);
    assert(D.Output(0) == 4.0);          // already there, nothing to ramp
}

void TestAnUnloadedDriveCannotSlip()
{
    // A drive with nothing on it is free-running, not slipping. Without the
    // feedback sweep it would keep the last speed it saw for ever and slip against
    // its own stale reading — which faults every drive on the ride within seconds
    // of the first train leaving it.
    FTrackDrives D(1);
    D.Preset(0, 5.0);

    // A train on it, doing nothing, at full torque: that IS slip.
    D.BeginFeedback();
    D.ReportFeedback(0, 0.0, 1.0);
    D.EndFeedback();
    assert(D.Read(0).bLoaded);
    assert(std::fabs(D.Slip(0) - 5.0) < 1e-9);

    // The train leaves. Nobody reports it, so the drive reads its own output back
    // and no torque.
    D.BeginFeedback();
    D.EndFeedback();
    assert(!D.Read(0).bLoaded);
    assert(std::fabs(D.Slip(0)) < 1e-9);
    assert(D.Read(0).Load == 0.0);

    // And the fault timer does not accumulate against nothing, however long.
    for (int i = 0; i < 240 * 60; ++i)
    {
        D.BeginFeedback();
        D.EndFeedback();
        D.Tick(Dt);
    }
    assert(!D.IsFaulted(0));
}

void TestAFaultNeedsSlipAndTorqueAndTimeTogether()
{
    // THE DIAGNOSTIC THIS LAYER EXISTS FOR, and the reason it takes all three. A
    // block brake taking a train from 26 m/s to a stop slips hard at full torque
    // for several seconds and is working perfectly; a lift chain turning under a
    // train that is not moving is a stalled ride. The only thing separating them
    // is how long it goes on.
    FDriveSpec S;
    S.SlipTripMs = 3.0;
    S.SlipTripSeconds = 5.0;

    // 1. All three. A chain at 5 m/s under a train doing nothing, flat out.
    {
        FTrackDrives D(1);
        D.Configure(0, S);
        D.Preset(0, 5.0);
        for (int i = 0; i < 240 * 4; ++i)
        {
            D.BeginFeedback();
            D.ReportFeedback(0, 0.0, 1.0);
            D.EndFeedback();
            D.Tick(Dt);
        }
        assert(!D.IsFaulted(0));         // four seconds is not yet five
        for (int i = 0; i < 240 * 2; ++i)
        {
            D.BeginFeedback();
            D.ReportFeedback(0, 0.0, 1.0);
            D.EndFeedback();
            D.Tick(Dt);
        }
        assert(D.IsFaulted(0));
        assert(D.AnyFaulted());
    }

    // 2. Slip and time but NOT torque — the drive is keeping up easily and simply
    //    running faster than its load, which is a drive that is not trying.
    {
        FTrackDrives D(1);
        D.Configure(0, S);
        D.Preset(0, 5.0);
        for (int i = 0; i < 240 * 30; ++i)
        {
            D.BeginFeedback();
            D.ReportFeedback(0, 0.0, 0.4);
            D.EndFeedback();
            D.Tick(Dt);
        }
        assert(!D.IsFaulted(0));
    }

    // 3. Torque and time but NOT slip — a chain hauling a train up a hill AT chain
    //    speed. Flat out for a minute and entirely healthy, which is exactly what
    //    a lift hill is, and the case a torque-only rule would report as broken.
    {
        FTrackDrives D(1);
        D.Configure(0, S);
        D.Preset(0, 5.0);
        for (int i = 0; i < 240 * 60; ++i)
        {
            D.BeginFeedback();
            D.ReportFeedback(0, 5.0, 1.0);
            D.EndFeedback();
            D.Tick(Dt);
        }
        assert(!D.IsFaulted(0));
    }

    // 3b. THE ONE THAT TOOK A MEASUREMENT TO FIND: a launch. Sustained slip at
    //     full torque, for longer than any threshold worth setting — 0 to 38 m/s
    //     at 6 m/s^2 is 6.3 seconds — and completely healthy. Slip, torque and time
    //     alone report it as a failure, and did: the two-train circuit faulted its
    //     launch drive on the first dispatch. What makes it different from a stall
    //     is that the gap is CLOSING.
    {
        FTrackDrives D(1);
        D.Configure(0, S);
        D.Preset(0, 38.0);
        double V = 0.0;
        for (int i = 0; i < 240 * 7; ++i)
        {
            V = std::min(38.0, V + 6.0 * Dt);
            D.BeginFeedback();
            D.ReportFeedback(0, V, 1.0);
            D.EndFeedback();
            D.Tick(Dt);
        }
        assert(V > 37.0);                // it really did launch
        assert(!D.IsFaulted(0));
        assert(D.Read(0).SlippingFor == 0.0);
    }

    // 4. Slip and torque but NOT time — the block brake arrival. Three seconds of
    //    everything the pad has, then the train is stopped and matched.
    {
        FTrackDrives D(1);
        D.Configure(0, S);
        D.Preset(0, 0.0);
        for (int i = 0; i < 240 * 3; ++i)
        {
            D.BeginFeedback();
            D.ReportFeedback(0, 26.0, 1.0);
            D.EndFeedback();
            D.Tick(Dt);
        }
        assert(!D.IsFaulted(0));
        assert(D.Read(0).SlippingFor > 2.9);

        // Stopped: slip gone, and the timer resets rather than remembering.
        D.BeginFeedback();
        D.ReportFeedback(0, 0.0, 0.0);
        D.EndFeedback();
        D.Tick(Dt);
        assert(D.Read(0).SlippingFor == 0.0);
    }
}

void TestAFaultDoesNotClearItself()
{
    // A fault an operator has not looked at is a fault that has not been dealt
    // with, which is why a real drive needs a reset rather than clearing itself
    // once the condition passes.
    FDriveSpec S;
    S.SlipTripSeconds = 1.0;
    FTrackDrives D(1);
    D.Configure(0, S);
    D.Preset(0, 5.0);

    for (int i = 0; i < 240 * 2; ++i)
    {
        D.BeginFeedback();
        D.ReportFeedback(0, 0.0, 1.0);
        D.EndFeedback();
        D.Tick(Dt);
    }
    assert(D.IsFaulted(0));

    // Condition gone, fault stays.
    for (int i = 0; i < 240 * 10; ++i)
    {
        D.BeginFeedback();
        D.ReportFeedback(0, 5.0, 0.2);
        D.EndFeedback();
        D.Tick(Dt);
    }
    assert(D.IsFaulted(0));

    // AND A RESET IS REFUSED UNTIL IT HAS BEEN ACKNOWLEDGED. Every real console
    // has ACKNOWLEDGE and RESET as two separate controls, usually not even on the
    // same coloured field, and the order between them is the point: acknowledging
    // says "I have SEEN this", resetting says "I have DEALT with it". A reset that
    // could be pressed without anyone reading what tripped clears faults nobody
    // knows about, and the two controls become the same button twice.
    assert(D.IsFaulted(0));
    assert(!D.IsAcknowledged(0));
    assert(D.AnyUnacknowledged());
    assert(!D.ResetFault(0));            // refused - nobody has looked
    assert(D.IsFaulted(0));

    D.AcknowledgeFault(0);
    assert(D.IsAcknowledged(0));
    assert(!D.AnyUnacknowledged());      // seen, but NOT fixed
    assert(D.IsFaulted(0));              // acknowledging changes nothing else

    assert(D.ResetFault(0));
    assert(!D.IsFaulted(0));
    assert(!D.AnyFaulted());
    assert(!D.IsAcknowledged(0));        // cleared with the fault, ready for the next
}

void TestAFaultIsReportedAndNotActedOn()
{
    // Deliberate, and worth an assertion so nobody "helpfully" makes a faulted
    // drive coast. What a ride does about a failed drive is an E-stop policy and
    // the PLC's decision; a motor that quietly stops the ride on its own is a
    // motor making that decision for it.
    FDriveSpec S;
    S.SlipTripSeconds = 1.0;
    FTrackDrives D(1);
    D.Configure(0, S);
    D.Preset(0, 5.0);

    for (int i = 0; i < 240 * 2; ++i)
    {
        D.BeginFeedback();
        D.ReportFeedback(0, 0.0, 1.0);
        D.EndFeedback();
        D.Tick(Dt);
    }
    assert(D.IsFaulted(0));
    assert(D.Output(0) == 5.0);          // still driving
}

void TestMalformedInputIsRefused()
{
    // The same trust boundary FTrain::SetZoneTargetSpeed guards, one layer
    // earlier: a NaN admitted here reaches the energy accounting a frame later,
    // having gone round the check that exists to catch it.
    FTrackDrives D(2);

    assert(!D.Command(0, -1.0));
    assert(!D.Command(0, std::nan("")));
    assert(!D.Command(5, 1.0));          // no such drive
    assert(!D.Preset(5, 1.0));
    assert(D.Read(0).Commanded == 0.0);

    FDriveSpec Bad;
    Bad.AccelRampMs2 = -1.0;
    assert(!D.Configure(0, Bad));
    Bad.AccelRampMs2 = std::nan("");
    assert(!D.Configure(0, Bad));
    Bad.AccelRampMs2 = 1.0;
    Bad.SlipTripSeconds = -2.0;
    assert(!D.Configure(0, Bad));
    assert(!D.Configure(5, FDriveSpec()));

    // And the timestep, which is the one guard FRideSignals::Tick also keeps to
    // itself rather than letting a wrapper clamp.
    D.Configure(0, FDriveSpec());
    D.Command(0, 4.0);
    D.Tick(0.0);
    D.Tick(-1.0);
    D.Tick(std::nan(""));
    assert(D.Output(0) == 0.0);          // nothing ramped, nothing jumped
    D.Tick(Dt);
    assert(D.Output(0) == 4.0);
}

void TestOneDrivePerZoneEvenWhereThereIsNoMotor()
{
    // A friction-only trim brake has no motor, but it keeps its slot, because an
    // index that means the same thing in the zone list and the drive list is worth
    // more than the empty entry. It simply never gets commanded to anything but
    // its authored speed.
    FTrackDrives D(4);
    assert(D.Num() == 4);
    for (std::size_t i = 0; i < 4; ++i)
    {
        assert(D.Output(i) == 0.0);
        assert(!D.IsFaulted(i));
    }
    assert(D.Output(9) == 0.0);          // out of range reads as stopped, not UB
}

void TestAnEmergencyStopCutsThePowerAndCannotBeTalkedOutOfIt()
{
    // IT LIVES IN THE DRIVES BECAUSE A REAL ONE CUTS POWER TO THE MOTORS. Put one
    // layer up — as "the dispatcher also commands zero" — it would be a stop with a
    // hole in it the width of every caller that forgot, and the one thing that must
    // never have holes is the stop. So it overrides the OUTPUT, not the command:
    // the PLC may go on asking for whatever it likes and nothing turns.
    FTrackDrives D(3);
    D.Preset(0, 5.0);
    D.Preset(1, 20.0);
    D.Preset(2, 0.0);
    assert(D.Output(0) == 5.0);

    assert(D.TripEmergencyStop("test"));
    assert(D.IsEmergencyStopped());
    assert(D.Output(0) == 0.0);
    assert(D.Output(1) == 0.0);

    // Commanding through it changes nothing, ticked or not.
    D.Command(0, 12.0);
    D.Command(1, 30.0);
    D.Tick(Dt);
    assert(D.Output(0) == 0.0);
    assert(D.Output(1) == 0.0);
    assert(D.Read(0).Commanded == 12.0);   // the PLC still asked; nothing turned

    // LATCHED, AND THE FIRST REASON WINS. What went first is the only thing worth
    // knowing, so a later trip must not overwrite it.
    assert(!D.TripEmergencyStop("something later"));
    assert(std::strcmp(D.EmergencyStopReason(), "test") == 0);

    // Cleared only by a person, never because the condition passed. Outputs come
    // back from the commands that were standing all along.
    D.ResetEmergencyStop();
    assert(!D.IsEmergencyStopped());
    D.Tick(Dt);
    assert(D.Output(0) == 12.0);
    assert(D.Output(1) == 30.0);
}

// IEC 60204-1 stop categories. Cat 0 is immediate removal of power; Cat 1 is a
// controlled stop with power RETAINED to achieve it, and then removed.
//
// The distinction was previously not expressible, which was a gap rather than a
// wrong answer — the behaviour was "output to zero this frame" and there was no
// way to say whether that was a deliberate Cat 0.
void TestStopCategoriesAreDistinguishable()
{
    // ---- Category 0: gone now, no tick required.
    {
        FTrackDrives D(2);
        D.Preset(0, 20.0);
        assert(D.TripEmergencyStop("protective device", FTrackDrives::EStopCategory::Zero));
        assert(D.Output(0) == 0.0);        // BEFORE any tick
        assert(D.IsPowerRemoved());
        assert(D.EmergencyStopCategory() == FTrackDrives::EStopCategory::Zero);
    }

    // ---- Category 1: winds down under power, THEN power goes.
    {
        FTrackDrives D(2);
        D.Configure(0, FDriveSpec{4.0, 4.0, 0.5, 1.0});   // 4 m/s^2 decel ramp
        D.Preset(0, 20.0);

        assert(D.PressEmergencyStopButton("operator"));
        assert(D.EmergencyStopCategory() == FTrackDrives::EStopCategory::One);

        // Still driving, and that is the definition rather than a leak.
        assert(D.Output(0) == 20.0);
        assert(!D.IsPowerRemoved());

        D.Tick(1.0);
        assert(std::fabs(D.Output(0) - 16.0) < 1e-9);     // wound down, not cut
        assert(!D.IsPowerRemoved());

        // ...and it arrives, and power goes when it does.
        for (int i = 0; i < 40; ++i) { D.Tick(0.1); }
        assert(D.Output(0) == 0.0);
        assert(D.IsPowerRemoved());
        assert(D.IsEmergencyStopped());                   // still latched
    }

    // ---- The delay timer, which is what stops Cat 1 being a hole.
    //
    // A safety relay implementing SS1 does not ask the drive whether it
    // finished; it gives it a window and opens the contactor regardless. Here a
    // drive whose ramp would take 200 s must still lose power at the deadline.
    {
        FTrackDrives D(1);
        D.Configure(0, FDriveSpec{0.1, 0.1, 0.5, 1.0});   // 0.1 m/s^2: 200 s to stop
        D.Preset(0, 20.0);
        D.SetCat1DelaySeconds(2.0);

        assert(D.PressEmergencyStopButton("operator"));
        for (int i = 0; i < 15; ++i) { D.Tick(0.1); }     // 1.5 s
        assert(!D.IsPowerRemoved());
        assert(D.Output(0) > 15.0);                       // barely wound down

        for (int i = 0; i < 10; ++i) { D.Tick(0.1); }     // past 2.0 s
        assert(D.IsPowerRemoved());
        assert(D.Output(0) == 0.0);                       // cut, not finished
    }

    // ---- With ramps OFF, which is every shipped preset, the two are identical.
    // This is why nothing previously measured moves.
    {
        FTrackDrives Zero(1), One(1);
        Zero.Preset(0, 20.0);
        One.Preset(0, 20.0);
        Zero.TripEmergencyStop("x", FTrackDrives::EStopCategory::Zero);
        One.PressEmergencyStopButton("x");
        Zero.Tick(Dt);
        One.Tick(Dt);
        assert(Zero.Output(0) == One.Output(0));
        assert(Zero.IsPowerRemoved() && One.IsPowerRemoved());
    }

    // ---- The default is the HARDER stop, so an omission cannot weaken it.
    {
        FTrackDrives D(1);
        D.Configure(0, FDriveSpec{4.0, 4.0, 0.5, 1.0});
        D.Preset(0, 20.0);
        D.TripEmergencyStop("caller that did not say");
        assert(D.EmergencyStopCategory() == FTrackDrives::EStopCategory::Zero);
        assert(D.Output(0) == 0.0);
    }
}

// MONITORED RESET, edge-triggered 0-1-0.
//
// A taped reset button must not cause automatic restart. Same reasoning as the
// dispatch anti-tie-down, except a wedged dispatch only runs trains early where a
// wedged reset clears the E-STOP — so the ride restarts itself the moment
// whatever tripped it stops being true.
void TestResetIsMonitoredAndCannotBeTiedDown()
{
    // ---- The honest sequence: released, pressed, released.
    {
        FTrackDrives D(1);
        D.Preset(0, 5.0);
        D.TripEmergencyStop("test");

        assert(!D.ScanResetInput(false));   // seen low
        assert(!D.ScanResetInput(true));    // pressed — nothing yet
        assert(D.IsResetArmed());
        assert(D.IsEmergencyStopped());     // still stopped WHILE held
        assert(D.ScanResetInput(false));    // released — NOW it clears
        assert(!D.IsEmergencyStopped());
    }

    // ---- A BUTTON HELD FROM BEFORE THE TRIP BUYS NOTHING, however long.
    // The trip clears the seen-low latch, so the press already in progress is not
    // the leading edge of anything.
    {
        FTrackDrives D(1);
        D.Preset(0, 5.0);
        D.ScanResetInput(true);             // somebody is holding it
        D.TripEmergencyStop("test");

        for (int i = 0; i < 10000; ++i)
        {
            assert(!D.ScanResetInput(true));
        }
        assert(D.IsEmergencyStopped());
        assert(!D.IsResetArmed());          // never even armed

        // It only counts once they let go and press again.
        assert(!D.ScanResetInput(false));
        assert(!D.ScanResetInput(true));
        assert(D.ScanResetInput(false));
        assert(!D.IsEmergencyStopped());
    }

    // ---- A button taped down across a whole session cannot clear a later stop.
    {
        FTrackDrives D(1);
        D.Preset(0, 5.0);
        for (int i = 0; i < 100; ++i) { D.ScanResetInput(true); }   // taped
        D.TripEmergencyStop("test");
        for (int i = 0; i < 100; ++i) { D.ScanResetInput(true); }
        assert(D.IsEmergencyStopped());
    }

    // ---- Scanning a released button on a running ride does nothing at all.
    {
        FTrackDrives D(1);
        D.Preset(0, 5.0);
        assert(!D.ScanResetInput(false));
        assert(!D.ScanResetInput(true));
        assert(!D.IsEmergencyStopped());
        assert(D.Output(0) == 5.0);
    }
}

void TestAStoppedDriveIsNotAlsoAccusedOfSlipping()
{
    // A drive that has been CUT cannot usefully be accused of failing to reach its
    // output: it has no output. Without this an E-stop raises a drive fault a few
    // seconds later on every motor with a train on it, and the panel fills with
    // failures that are all the same failure and none of them real.
    FDriveSpec S;
    S.SlipTripSeconds = 1.0;
    FTrackDrives D(1);
    D.Configure(0, S);
    D.Preset(0, 10.0);

    D.TripEmergencyStop("cut");
    for (int i = 0; i < 240 * 10; ++i)
    {
        D.BeginFeedback();
        D.ReportFeedback(0, 10.0, 1.0);   // train still rolling through, full torque
        D.EndFeedback();
        D.Tick(Dt);
    }
    assert(!D.IsFaulted(0));
    assert(D.Read(0).SlippingFor == 0.0);
}

void TestReadyIsNotTheSameAsCommanded()
{
    // PRE-LAUNCH. "Clear" and "ready" are different questions, and the second one
    // is the device's to answer: a launch that has been TOLD to arm is not the same
    // as one that HAS armed. Those two states became distinguishable for free the
    // moment a command stopped taking effect instantly, which is the whole reason
    // Commanded and Output are two numbers rather than one.
    FDriveSpec S;
    S.AccelRampMs2 = 4.0;
    FTrackDrives D(1);
    D.Configure(0, S);

    // Sitting at zero, asked for nothing: ready, trivially.
    assert(D.IsReady(0));

    // Told to arm. NOT ready - it has been asked, not achieved.
    D.Command(0, 20.0);
    assert(!D.IsReady(0));
    D.Tick(Dt);
    assert(!D.IsReady(0));          // ramping

    for (int i = 0; i < 240 * 6; ++i) { D.Tick(Dt); }
    assert(D.IsReady(0));           // up to speed

    // A FAULTED DRIVE IS NEVER READY, however matched its numbers are. A motor at
    // full torque going nowhere is not something to hand a train to.
    FDriveSpec F;
    F.SlipTripSeconds = 1.0;
    FTrackDrives E(1);
    E.Configure(0, F);
    E.Preset(0, 5.0);
    assert(E.IsReady(0));
    for (int i = 0; i < 240 * 2; ++i)
    {
        E.BeginFeedback();
        E.ReportFeedback(0, 0.0, 1.0);
        E.EndFeedback();
        E.Tick(Dt);
    }
    assert(E.IsFaulted(0));
    assert(!E.IsReady(0));

    // AND AN E-STOPPED RIDE IS READY FOR NOTHING, whatever the individual drives
    // think. Power is gone.
    FTrackDrives G(2);
    G.Preset(0, 3.0);
    G.Preset(1, 0.0);
    assert(G.IsReady(0) && G.IsReady(1));
    G.TripEmergencyStop("test");
    assert(!G.IsReady(0));
    assert(!G.IsReady(1));
    G.ResetEmergencyStop();
    assert(G.IsReady(0));
}

void TestDegradedHardwareIsNotAFaultAndNotACommand()
{
    // A device that no longer delivers what it is rated for. FAULTS.md called this
    // the one fault the project could not express at all.
    //
    // THREE NUMBERS THAT STAY SEPARATE. The command is still correct, the drive
    // still writes it, and the output still reaches it — a glazed pad does not
    // change what the PLC asked for or what the caliper did about it. So a
    // degraded device is still READY, and it must be: refusing dispatch because a
    // pad is worn would be a fault detector wearing a permissive's clothes, and
    // nothing here has measured that the pad is worn.
    FTrackDrives D(2);
    D.Preset(0, 6.0);
    assert(D.DeliveredFraction(0) == 1.0);      // healthy until told otherwise
    assert(!D.AnyDegraded());

    D.SetDeliveredFraction(0, 0.3);
    assert(D.DeliveredFraction(0) == 0.3);
    assert(D.AnyDegraded());
    assert(D.Output(0) == 6.0);                 // the command is untouched
    assert(D.IsReady(0));                       // and so is readiness
    assert(!D.IsFaulted(0));                    // and nothing here knows it is bad

    // Clamped, because it is a fraction of rated authority and there is no such
    // thing as a brake delivering 140% of itself or less than nothing.
    D.SetDeliveredFraction(1, 4.0);
    assert(D.DeliveredFraction(1) == 1.0);
    D.SetDeliveredFraction(1, -2.0);
    assert(D.DeliveredFraction(1) == 0.0);
    D.SetDeliveredFraction(9, 0.5);             // no such drive, no crash
    assert(D.DeliveredFraction(9) == 1.0);
    std::printf("  degraded hardware is neither a command nor a fault\n");
}

} // namespace

int main()
{
    TestACommandIsARequestNotAnOutput();
    TestNoRampIsTheOldBehaviourExactly();
    TestPresetSkipsTheRamp();
    TestAnUnloadedDriveCannotSlip();
    TestAFaultNeedsSlipAndTorqueAndTimeTogether();
    TestAFaultDoesNotClearItself();
    TestAFaultIsReportedAndNotActedOn();
    TestMalformedInputIsRefused();
    TestOneDrivePerZoneEvenWhereThereIsNoMotor();
    TestAnEmergencyStopCutsThePowerAndCannotBeTalkedOutOfIt();
    TestStopCategoriesAreDistinguishable();
    TestResetIsMonitoredAndCannotBeTiedDown();
    TestAStoppedDriveIsNotAlsoAccusedOfSlipping();
    TestReadyIsNotTheSameAsCommanded();
    TestDegradedHardwareIsNotAFaultAndNotACommand();

    std::printf("test_trackdrives: all assertions passed\n");
    return 0;
}
