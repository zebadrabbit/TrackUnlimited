// TrackUnlimited: auditing the devices a layout ACTUALLY has, against the ride
// it actually produces. Plain C++17, no engine dependency.
//
// ===================== WHY THIS IS NOT THE VALIDATOR =====================
//
// `TrackValidate.h` reads the AUTHORED values and says whether they are
// self-consistent: a negative radius, a clothoid that does not meet its
// neighbour, a helix with no turns. It can do that with the numbers alone.
//
// Nothing here can be answered that way. Whether a brake is long enough depends
// on the train's LENGTH; whether it can stop what arrives depends on the SPEED
// at that point, which depends on every metre of track before it. These are
// properties of the ride rather than of the segment, so they can only be checked
// after the profile exists — which is why they are a separate pass and not more
// rules in the validator.
//
// ===================== REPORT THE CONSEQUENCE, NOT THE RULE ==============
//
// The project's rule everywhere else is REPORT, NEVER REPAIR. This adds the half
// that makes a report worth reading: say what will HAPPEN, with the numbers.
//
//   bad:  "block brake at 412 m is too short"
//   good: "block brake at 412 m: a train arrives at 30.4 m/s and needs 154 m to
//          stop at 3.0 m/s^2, but the device is 130 m — it leaves the block at
//          16.4 m/s"
//
// The first is a rule somebody has to look up and then work out whether they
// care about. The second is the failure, and it also happens to say what to
// change and by how much: 24 m longer, or a harder brake, or slow it down
// upstream. NOTHING HERE OFFERS TO DO ANY OF THAT — the fix for a brake that
// cannot stop a train is a design decision with three defensible answers, and a
// tool that picked one would be choosing on the author's behalf.
//
// Units: metres, seconds, m/s, m/s^2.

#pragma once

#include "TrackSensors.h"     // StoppingDistanceM

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

enum class EDeviceProblem
{
    // The stop mark lands past the device's own far end, so nothing trips it and
    // a train sent to park here crawls out into the next block.
    ShorterThanTrain,

    // It cannot stop what arrives. Reported with the speed it will still be
    // doing at the far end, which is the number that says how bad it is.
    CannotStopArrival,

    // A trim asked to be a block boundary. It can slow a train to a stand and
    // can never start one again, so a train held here stands for the session.
    HoldsButCannotRelease,

    // It can start a train but not stop one, so it cannot bound a block either.
    ReleasesButCannotHold,
};

struct FDeviceFinding
{
    double S = 0.0;                  // where the device starts
    EDeviceProblem Problem = EDeviceProblem::ShorterThanTrain;
    bool bIsError = false;           // false = worth knowing, true = will not work
    std::string What;                // the sentence, WITH the numbers in it
};

// One device as the block walk already derived it. Deliberately NOT the segment
// zone enum: this header is engine-free and the two authorities are what matter,
// not what the device is called. A lift chain and a block brake differ in every
// way except the pair of authorities, and it is the pair this reasons about.
struct FDeviceSpan
{
    double StartS = 0.0;
    double EndS = 0.0;
    double CommandedSpeed = 0.0;
    bool bCanHold = false;           // can bring a train to a stand and keep it there
    bool bCanRelease = false;        // can start a stopped train moving again
    bool bIsBlockBoundary = false;   // the interlocking parks trains here
    std::string Name;                // for the message, e.g. "block brake"

    double Length() const { return EndS - StartS; }
};

struct FDeviceAuditSettings
{
    double TrainLengthM = 15.0;

    // What a friction brake delivers at rider-comfortable rates. A knob because
    // it is a property of the hardware somebody chose, and the physical world
    // needs tuning a minimal model cannot see.
    double ServiceDecelMs2 = 3.0;

    // The margin the stop mark keeps from the far end, so a train parked on the
    // device does not protrude into the next zone through a defect.
    double NoseClearanceM = 1.0;
};

inline const char* DeviceProblemName(EDeviceProblem P)
{
    switch (P)
    {
    case EDeviceProblem::ShorterThanTrain:      return "device shorter than its train";
    case EDeviceProblem::CannotStopArrival:     return "device cannot stop what arrives";
    case EDeviceProblem::HoldsButCannotRelease: return "block boundary that cannot release";
    default:                                    return "block boundary that cannot hold";
    }
}

// `SpeedAt` is the ride profile: metres of arc length in, m/s out. Passed as a
// function rather than a table because the caller already has one and copying a
// few thousand samples to ask a handful of questions would be the expensive way
// round.
inline std::vector<FDeviceFinding> AuditDevices(
    const std::vector<FDeviceSpan>& Devices,
    const FDeviceAuditSettings& Settings,
    const std::function<double(double)>& SpeedAt)
{
    std::vector<FDeviceFinding> Out;
    char Buf[512];

    for (const FDeviceSpan& D : Devices)
    {
        const double L = D.Length();
        if (L <= 0.0)
        {
            continue;
        }

        // ---- Can a train be PARKED here at all?
        //
        // The stop mark is max(start + trainLength, end - clearance). On a device
        // shorter than the train the first term wins and the mark lands past the
        // far end, so nothing trips it. Reported rather than clamped: clamping
        // builds a device that stops a train with its nose over the boundary,
        // which is worse because it looks like it worked.
        if (D.bIsBlockBoundary && L < Settings.TrainLengthM + Settings.NoseClearanceM)
        {
            const double Mark = D.StartS + Settings.TrainLengthM;
            std::snprintf(Buf, sizeof(Buf),
                "%s at %.0f m is %.1f m and the train is %.1f m: the stop mark lands "
                "at %.0f m, which is %.1f m PAST the far end at %.0f m. Nothing trips "
                "it, so a train sent to park here crawls into the next block. "
                "Needs %.1f m.",
                D.Name.c_str(), D.StartS, L, Settings.TrainLengthM, Mark,
                Mark - D.EndS, D.EndS, Settings.TrainLengthM + Settings.NoseClearanceM);
            Out.push_back({D.StartS, EDeviceProblem::ShorterThanTrain, true, Buf});
        }

        // ---- Can it stop what ARRIVES?
        //
        // Only asked of a device that is supposed to stop trains. A trim that
        // cannot stop one is not failing at anything — trimming is what it is
        // for, and the speed it leaves at is the speed it was asked for.
        if (D.bIsBlockBoundary && D.bCanHold)
        {
            const double Entry = SpeedAt(D.StartS);
            const double Need = StoppingDistanceM(Entry, Settings.ServiceDecelMs2);
            const double Usable = L - Settings.NoseClearanceM;
            if (Need > Usable)
            {
                // What it will still be doing at the far end. This is the number
                // that separates "a metre short" from "will not stop at all".
                const double ExitSq = Entry * Entry - 2.0 * Settings.ServiceDecelMs2 * Usable;
                const double Exit = ExitSq > 0.0 ? std::sqrt(ExitSq) : 0.0;
                std::snprintf(Buf, sizeof(Buf),
                    "%s at %.0f m: a train arrives at %.1f m/s and needs %.0f m to stop "
                    "at %.1f m/s^2, but only %.0f m is usable. It leaves the block at "
                    "%.1f m/s. Lengthen it by %.0f m, brake harder, or slow the train "
                    "upstream.",
                    D.Name.c_str(), D.StartS, Entry, Need, Settings.ServiceDecelMs2,
                    Usable, Exit, Need - Usable);
                Out.push_back({D.StartS, EDeviceProblem::CannotStopArrival, true, Buf});
            }
        }

        // ---- Does it have BOTH authorities?
        //
        // A dispatcher needs to stop a train and start it again before it may
        // park one anywhere. Either half alone is a device, not a block: a trim
        // that stops a train strands it, and a launch that cannot stop one has
        // nothing to hold at the boundary it claims to be.
        if (D.bIsBlockBoundary && !D.bCanRelease)
        {
            std::snprintf(Buf, sizeof(Buf),
                "%s at %.0f m bounds a block but cannot start a train again. A train "
                "held here stands for the rest of the session — the block never "
                "clears and everything behind it queues. A block boundary needs "
                "brakes AND drive tyres.",
                D.Name.c_str(), D.StartS);
            Out.push_back({D.StartS, EDeviceProblem::HoldsButCannotRelease, true, Buf});
        }
        if (D.bIsBlockBoundary && !D.bCanHold)
        {
            std::snprintf(Buf, sizeof(Buf),
                "%s at %.0f m bounds a block but cannot stop a train. There is nothing "
                "to hold the train the interlocking is relying on it to hold.",
                D.Name.c_str(), D.StartS);
            Out.push_back({D.StartS, EDeviceProblem::ReleasesButCannotHold, true, Buf});
        }
    }
    return Out;
}

// ponytail: no capacity or deadlock analysis here. "Is there still a holding
// block between these two trains" is a property of the block GRAPH and the
// dispatch policy rather than of one device, and it wants the interlocking's own
// walk rather than a second one that could disagree with it. The four checks
// above are the ones answerable from a device and a speed, which is what makes
// them cheap enough to run on every edit.
