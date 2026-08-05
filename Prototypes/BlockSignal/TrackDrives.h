// TrackUnlimited: the drives a ride control system commands.
//
// The mirror of TrackSensors.h, and it exists for the same reason. Sensors are
// the PLC's inputs; drives are its outputs, and until this file the output side
// was a lie in exactly the way the input side had been: ServeHolds wrote a speed
// straight into FTrain, the track was at that speed the same instant, and there
// was nothing to disagree with and nothing to report back.
//
//   PLC (logical)  ->  DRIVES (physical)  ->  the track
//                  <-       feedback      <-
//
// WHAT A REAL ONE IS. A variable-frequency drive is the muscle for ONE motor. It
// takes a speed command, ramps its output frequency toward it at a configured
// rate, and turns a tyre or a chain at whatever that frequency means in m/s. It
// reports motor speed and current draw back, and those readings can DISAGREE
// with the command. There is one PLC and many VFDs; this holds all of them,
// because a bank of drives is what a control panel is a picture of.
//
// ================== THREE NUMBERS THAT CAN DISAGREE ==================
//
//   Commanded != Output    the drive is RAMPING. Normal, and transient.
//   Output    != Actual    the drive is SLIPPING. The tyres are turning at a
//                          speed the train is not doing.
//   Load       = 1         the drive is at FULL TORQUE and has nothing left.
//
// Each on its own is ordinary. Slip is how a train gets up to chain speed in the
// first place, and full torque is what a launch is. What is NOT ordinary is slip
// AND saturation AND time together: the drive is doing everything it can, the
// train is not responding, and it has been that way for a while. That is a
// stalled lift, a failed launch, or a brake that is not biting — and it is what
// a real drive trips on, which is why it is the fault condition here.
//
// A FAULT IS REPORTED, NOT ACTED ON. A real VFD trips its output and coasts; this
// one raises a flag and keeps driving, because deciding what a ride does about a
// faulted drive is the PLC's job and an E-stop policy, not a property of the
// motor. Nothing here should quietly stop a ride.
//
// Reference: VFDs, drive tyres and chain lifts as described in Weisenberger,
// "Coasters 101" (3rd ed.), chapters 6 and 7.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

// How one drive is set up. Every field is a property of the DRIVE rather than of
// the track it sits on, which is the distinction that makes this a separate thing
// from FTrackZone: the zone is where the motor is and how hard it can push, this
// is how the box driving it is configured.
struct FDriveSpec
{
    // How fast the drive may change its OWN output, in m/s per second. This is
    // not the same as the zone's tractive authority and is usually smaller: a
    // drive ramped faster than it can move its load is a drive that just slips.
    //
    // ZERO MEANS NO RAMP IS MODELLED and the output follows the command instantly.
    // That is what every caller did before this file existed, so it is the default
    // and nothing measured before this changes by adding drives.
    double AccelRampMs2 = 0.0;
    double DecelRampMs2 = 0.0;

    // The fault condition, and it needs all three. A gap this big (m/s), while the
    // drive is at full torque, for this long (seconds).
    //
    // The time is the load-bearing part. A block brake taking a train from 26 m/s
    // to a stop is slipping hard at full torque for several seconds and is working
    // perfectly, so a threshold short enough to catch that is a threshold that
    // reports every normal arrival as a failure.
    double SlipTripMs = 3.0;
    double SlipTripSeconds = 5.0;
};

// What a panel would show for one drive. Everything here is either commanded by
// the PLC, produced by the drive, or measured off the motor — nothing is a peek
// at the simulation.
struct FDriveReading
{
    double Commanded = 0.0;     // what the PLC wrote
    double Output = 0.0;        // what the drive has ramped its output to
    double Actual = 0.0;        // measured speed of what the motor is driving
    double Load = 0.0;          // 0..1 of tractive authority in use, i.e. torque
    double SlippingFor = 0.0;   // seconds of slip AT full torque, consecutively
    bool bLoaded = false;       // is there anything on this drive at all
    bool bFaulted = false;
};

class FTrackDrives
{
public:
    // One drive per zone, indexed the same way, because a zone is where exactly
    // one motor lives. Zones with no motor — a friction-only trim brake — still
    // get a drive; it is simply never commanded to anything but its authored
    // speed, and keeping the indices identical is worth more than the empty slot.
    explicit FTrackDrives(std::size_t NumDrives, const FDriveSpec& InDefault = FDriveSpec())
        : State(NumDrives)
        , Spec(NumDrives, InDefault)
        , LastSlip(NumDrives, 0.0)
    {
    }

    std::size_t Num() const { return State.size(); }

    // Refuses a malformed spec rather than storing it, mirroring FTrain::AddZone.
    // A negative ramp would drive the output away from its command for ever, and
    // the comparison form rejects NaN too.
    bool Configure(std::size_t Drive, const FDriveSpec& In)
    {
        if (Drive >= Spec.size() || !(In.AccelRampMs2 >= 0.0) || !(In.DecelRampMs2 >= 0.0)
            || !(In.SlipTripMs >= 0.0) || !(In.SlipTripSeconds >= 0.0))
        {
            return false;
        }
        Spec[Drive] = In;
        return true;
    }

    // THE PLC'S WRITE, and the only thing the control program is allowed to do to
    // a drive. Note what it does NOT do: it does not set the output. A command is
    // a request, and how fast the drive gets there is the drive's business.
    //
    // Validated on the same term FTrain::SetZoneTargetSpeed validates, because it
    // is the same number one layer earlier — a NaN admitted here would arrive in
    // the energy accounting a frame later, having passed the guard that exists to
    // catch it.
    bool Command(std::size_t Drive, double Speed)
    {
        if (Drive >= State.size() || !(Speed >= 0.0))
        {
            return false;
        }
        State[Drive].Commanded = Speed;
        return true;
    }

    // The ride opening with its drives already running: sets the command AND the
    // output, bypassing the ramp. Startup only. Without it every drive ramps up
    // from zero on the first frame of the session, which is a ride whose lift
    // chain is stationary when the first train reaches it.
    bool Preset(std::size_t Drive, double Speed)
    {
        if (!Command(Drive, Speed))
        {
            return false;
        }
        State[Drive].Output = Speed;
        State[Drive].Actual = Speed;
        return true;
    }

    // ===================== EMERGENCY STOP =====================
    //
    // IT LIVES HERE, INSIDE THE DRIVES, and that is the point. A real E-stop cuts
    // power to the motors; it is not a request the control program is invited to
    // honour. Implemented one layer up — as "the dispatcher also commands zero" —
    // it would be an E-stop with a hole in it the width of every caller that
    // forgot, and the one thing that must never have holes is the stop.
    //
    // So it overrides the output rather than the command. The PLC can go on
    // commanding whatever it likes; nothing turns.
    //
    // IT DOES NOT STOP TRAINS, IT STOPS THE RIDE, and that distinction falls out
    // of the model for free rather than being arranged. A train on open course
    // coasts, because no drive is touching it — it runs to the next brake and is
    // held there. A train in a brake run stops at once, because a brake commanded
    // to zero bites. That is exactly what a real E-stop does and exactly why a
    // ride is built out of block brakes in the first place.
    //
    // LATCHED, AND THE FIRST REASON WINS. An E-stop clears when a person clears
    // it, never because the condition passed; and a trip that overwrote its own
    // reason with whatever failed next would throw away the only thing worth
    // knowing, which is what went first.
    bool TripEmergencyStop(const char* InReason)
    {
        if (bEmergencyStopped)
        {
            return false;   // already stopped; keep the original cause
        }
        bEmergencyStopped = true;
        StopReason = InReason != nullptr ? InReason : "unspecified";
        return true;
    }

    bool IsEmergencyStopped() const { return bEmergencyStopped; }
    const char* EmergencyStopReason() const { return bEmergencyStopped ? StopReason : ""; }

    // Manual, and deliberately the only way out. Same reasoning as a drive fault
    // needing a reset: a stop nobody has looked at has not been dealt with.
    void ResetEmergencyStop() { bEmergencyStopped = false; StopReason = ""; }

    // Ramp every output toward its command, and age the fault timers. ONCE PER
    // FRAME, like FRideSignals::Tick and for the same reason — these are rates,
    // and a rate applied N times a frame is N times the rate.
    //
    // Deliberately does not re-guard the timestep beyond rejecting it: a wrapper
    // that clamped or substituted a wall clock would defeat the single test.
    void Tick(double DeltaSeconds)
    {
        if (!(DeltaSeconds > 0.0))
        {
            return;
        }
        if (bEmergencyStopped)
        {
            // Not ramped down. Power is gone, so the output is gone — and the fault
            // timers stop with it, because a drive that has been cut cannot be
            // usefully accused of slipping.
            for (FDriveReading& R : State)
            {
                R.Output = 0.0;
                R.SlippingFor = 0.0;
            }
            return;
        }
        for (std::size_t i = 0; i < State.size(); ++i)
        {
            FDriveReading& R = State[i];
            const FDriveSpec& Sp = Spec[i];

            if (R.Commanded > R.Output)
            {
                R.Output = Sp.AccelRampMs2 > 0.0
                    ? std::min(R.Commanded, R.Output + Sp.AccelRampMs2 * DeltaSeconds)
                    : R.Commanded;
            }
            else if (R.Commanded < R.Output)
            {
                R.Output = Sp.DecelRampMs2 > 0.0
                    ? std::max(R.Commanded, R.Output - Sp.DecelRampMs2 * DeltaSeconds)
                    : R.Commanded;
            }

            // Fault accounting reads the feedback from the last step, which is the
            // only feedback that exists — a drive cannot know this frame's outcome
            // before the frame happens, and neither can a real one.
            //
            // An UNLOADED drive cannot slip. Nothing is on it, so Output and Actual
            // have nothing to disagree about, and counting that as slip would fault
            // every drive on the ride within seconds of opening.
            const double SlipNow = std::fabs(R.Output - R.Actual);
            const bool bSlipping = R.bLoaded && SlipNow > Sp.SlipTripMs;
            const bool bSaturated = R.Load >= 0.999;

            // AND NOT WINNING, which is the condition that took a measurement to
            // find. A LAUNCH is sustained slip at full torque by definition — 0 to
            // 38 m/s at 6 m/s^2 is 6.3 seconds of the drive flat out against a
            // train nowhere near its output speed — and slip, torque and time alone
            // report that healthy launch as a failed one. Measured: the two-train
            // circuit faulted its launch drive on the first dispatch.
            //
            // What separates a launch from a stall is not how big the gap is or how
            // long it lasts, but whether it is CLOSING. A drive that is winning is a
            // drive doing its job, however hard it is working; a drive at full
            // torque making no progress is a stalled lift, a failed launch, or a
            // brake that is not biting.
            const bool bGaining = SlipNow < LastSlip[i] - 1e-9;
            LastSlip[i] = SlipNow;

            if (bSlipping && bSaturated && !bGaining)
            {
                R.SlippingFor += DeltaSeconds;
            }
            else
            {
                R.SlippingFor = 0.0;
            }
            if (Sp.SlipTripSeconds > 0.0 && R.SlippingFor >= Sp.SlipTripSeconds)
            {
                R.bFaulted = true;
            }
        }
    }

    // Feedback, in the same Begin/Report/End shape as FTrackSensors, and for the
    // same reason: a drive with nothing on it is a real and distinct state, and
    // "not reported this frame" is how it gets said. Without the sweep, a drive
    // whose train has left keeps the last speed it saw for ever and slips against
    // its own stale reading.
    void BeginFeedback()
    {
        for (FDriveReading& R : State)
        {
            R.bLoaded = false;
        }
    }

    // What the motor is actually turning at, and how much of its authority that is
    // taking. Both come from the physics; neither is a position.
    void ReportFeedback(std::size_t Drive, double ActualMs, double LoadFraction)
    {
        if (Drive >= State.size())
        {
            return;
        }
        FDriveReading& R = State[Drive];
        R.Actual = ActualMs;
        R.Load = std::max(0.0, std::min(1.0, LoadFraction));
        R.bLoaded = true;
    }

    // Unloaded drives spin at their own output with nothing to push, so they read
    // no slip and no torque. Free-running, which is what a lift chain does all day
    // between trains.
    void EndFeedback()
    {
        for (FDriveReading& R : State)
        {
            if (!R.bLoaded)
            {
                R.Actual = R.Output;
                R.Load = 0.0;
            }
        }
    }

    // WHAT THE TRACK IS TOLD. The one number that leaves this class and reaches
    // the physics — everything else here is for the panel and the diagnostics.
    //
    // Zero under an E-stop, tested HERE as well as in Tick. Belt and braces on
    // purpose: this is the single value that decides whether anything on the ride
    // moves, and a caller that reads it without having ticked first must not get a
    // stale non-zero out of it.
    double Output(std::size_t Drive) const
    {
        if (bEmergencyStopped)
        {
            return 0.0;
        }
        return Drive < State.size() ? State[Drive].Output : 0.0;
    }

    const FDriveReading& Read(std::size_t Drive) const { return State[Drive]; }

    // PRE-LAUNCH: is this drive ready to take a train?
    //
    // The step a real console has between "everything is secured" and "you may
    // go". It is the DEVICE declaring itself ready rather than the platform — a
    // launch armed and charged, a chain actually turning, tyres up to speed — and
    // a dispatch is not permitted until the thing about to take the train says it
    // can.
    //
    // Ready is output HAS REACHED command and nothing is faulted. A drive still
    // ramping is not ready, which is the whole reason Commanded and Output are two
    // numbers: the moment a command stopped taking effect instantly, "arming" and
    // "armed" became distinguishable for free.
    //
    // An E-stopped ride is ready for nothing.
    bool IsReady(std::size_t Drive) const
    {
        if (bEmergencyStopped || Drive >= State.size())
        {
            return false;
        }
        const FDriveReading& R = State[Drive];
        return !R.bFaulted && std::fabs(R.Commanded - R.Output) <= 1e-9;
    }
    const FDriveSpec& SpecOf(std::size_t Drive) const { return Spec[Drive]; }

    // Signed: positive means the drive is running faster than its load, which is a
    // drive pushing; negative means the load is running away from it, which is a
    // brake being overhauled or a train being carried by gravity through a zone.
    double Slip(std::size_t Drive) const
    {
        return Drive < State.size() ? State[Drive].Output - State[Drive].Actual : 0.0;
    }

    bool IsFaulted(std::size_t Drive) const
    {
        return Drive < State.size() && State[Drive].bFaulted;
    }

    bool AnyFaulted() const
    {
        for (const FDriveReading& R : State)
        {
            if (R.bFaulted) { return true; }
        }
        return false;
    }

    // Manual, and deliberately so — a fault an operator has not looked at is a
    // fault that has not been dealt with. Same reasoning as a real drive needing a
    // reset rather than clearing itself once the condition passes.
    void ResetFault(std::size_t Drive)
    {
        if (Drive < State.size())
        {
            State[Drive].bFaulted = false;
            State[Drive].SlippingFor = 0.0;
        }
    }

private:
    std::vector<FDriveReading> State;
    std::vector<FDriveSpec> Spec;

    // Last frame's slip magnitude, so "is the gap closing" can be asked. Private
    // rather than on the reading, because it is the fault rule's own scratch space
    // and not something a panel would ever show.
    std::vector<double> LastSlip;

    // Ride-wide rather than per drive, because that is what an E-stop is: one
    // circuit, one button, everything dead. A per-drive version would be a way of
    // stopping half a ride, which is not a thing anybody wants to be able to do.
    bool bEmergencyStopped = false;
    const char* StopReason = "";
};
