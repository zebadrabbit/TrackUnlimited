// TrackUnlimited Phase 1: the whole ride, measured at edit time.
// Plain C++17, no engine dependency, same conventions as TrackSpline.h.
//
// Runs the train once over the finished track and records what a rider would
// get at every point: speed, the three felt-G axes, roll rate, and height. That
// is the data behind any readout or graph, and it is deliberately computed here
// rather than in the editor so it can be tested without launching one.
//
// The reason this is worth its own pass rather than being read off a live ride:
// an author needs to know a hill is too tall BEFORE watching a train fail to
// crest it. `bCompleted` and `StalledAtS` answer that in one rebuild.
//
// It also carries the two channels that are structurally invisible elsewhere,
// which is most of the point:
//
//   ROLL RATE, because felt G has no roll-rate term — the rider is modelled as
//   a point at the heartline, so spinning it costs exactly nothing. A barrel
//   roll that reads perfectly smooth in G is half a G of head snap at 180 deg/s.
//
//   HEIGHT, because a G trace cannot tell you the track is underground.
//
// Units: metres, seconds, m/s. G values are multiples of gravity; roll rate is
// degrees per second, because that is the unit a human has intuition for.

#pragma once

#include "TrainPhysics.h"

#include <cmath>
#include <cstddef>
#include <vector>

struct FRideSample
{
    double S = 0.0;      // arc length, metres
    double Time = 0.0;   // seconds since dispatch
    double Speed = 0.0;  // m/s
    double Height = 0.0; // heartline height, metres

    double VerticalG = 0.0;   // +1 is sitting on level track; 0 is airtime
    double LateralG = 0.0;    // +ve presses the rider toward their right
    double TangentialG = 0.0; // fore-aft: acceleration and braking

    // The one felt G cannot contain. Degrees per second about the tangent, from
    // the resolved roll — so it includes the roll the GEOMETRY contributes and
    // not just the roll somebody typed.
    double RollRateDegPerSec = 0.0;
};

struct FRideProfile
{
    std::vector<FRideSample> Samples;

    // Did the train get round? A stall is the single most useful thing an
    // editor can tell an author, and the only way to know is to run it.
    bool bCompleted = false;
    double StalledAtS = 0.0;
    double StalledHeight = 0.0;

    // Rolled backwards, which only happens when FTrainConfig::bAllowRollback is
    // on. Reported separately from a stall because it is a different fault with
    // a different fix: a stall says "this hill is too tall", a rollback says the
    // same thing AND that the train is now loose on the track heading the wrong
    // way — which is a signalling problem as much as a physics one.
    //
    // The run stops here. Left going, the train would oscillate in the valley
    // until friction settled it, and an author watching an oscillation could
    // easily read it as the ride working.
    bool bRolledBack = false;
    double RolledBackAtS = 0.0;

    // Held by an anti-rollback catch: ratchets, chain dogs, a catch car. A THIRD
    // distinct outcome, not a softer rollback.
    //
    // A stall says the hill is too tall. A rollback says that AND that the train
    // is loose heading the wrong way. A catch says the hill is too tall and the
    // SAFETY DEVICE DID ITS JOB — the train is held, nobody is in danger, and the
    // layout is still wrong. Reporting it as a success because nothing bad
    // happened would be exactly the wrong lesson: the ride does not get round.
    bool bCaughtByAntiRollback = false;
    double CaughtAtS = 0.0;

    double Duration = 0.0; // seconds

    // Extremes, each with where it happened. "4.25 g" is a number; "4.25 g at
    // S=310 m" is something an author can go and look at.
    double TopSpeed = 0.0;
    double TopSpeedS = 0.0;
    double MaxVerticalG = 0.0;
    double MaxVerticalGS = 0.0;
    double MinVerticalG = 0.0;
    double MinVerticalGS = 0.0;
    double MaxAbsLateralG = 0.0;
    double MaxAbsLateralGS = 0.0;
    double MaxAbsRollRate = 0.0;
    double MaxAbsRollRateS = 0.0;
    double LowestHeight = 0.0;
    double HighestHeight = 0.0;
};

// Runs `Train` from a standing start at S=0 to the end of the track.
//
// The train must already have its zones. Sampling is by ARC LENGTH rather than
// by time, so the density of the data does not collapse where the ride is fast
// — which is exactly where the G is worth looking at.
inline FRideProfile RunRideProfile(FTrain& Train, const FTrack& Track,
                                   double SampleSpacingM = 1.0, double Dt = 1.0 / 240.0)
{
    const double Pi = 3.14159265358979323846;
    const double RadToDeg = 180.0 / Pi;

    FRideProfile P;
    const double Total = Track.TotalLength();
    if (!(Total > 0.0) || !(SampleSpacingM > 0.0) || !(Dt > 0.0))
    {
        return P;
    }

    Train.Place(0.0, 0.0);

    // A stall is "stopped, and not somewhere that is going to push it again".
    // Counting consecutive near-stationary steps rather than testing speed once
    // avoids calling the standing start at S=0 a stall.
    const double StallSpeed = 0.02;   // m/s
    const int StallSteps = static_cast<int>(2.0 / Dt); // two seconds of not moving

    int Stalled = 0;
    double NextSampleS = 0.0;
    double Time = 0.0;
    double PrevRoll = Track.EvaluateAt(0.0).Roll;
    double PrevS = 0.0;

    // Bounded so a bad layout cannot spin here forever. Ten minutes of ride at
    // this timestep is far beyond anything real.
    const int MaxSteps = static_cast<int>(600.0 / Dt);

    for (int Step = 0; Step < MaxSteps; ++Step)
    {
        const double S = Train.GetDistance();
        const FTrackFrame F = Train.GetFrame();
        const FGForces G = Train.GetForces();

        if (S >= NextSampleS || Train.IsAtEnd())
        {
            FRideSample Sample;
            Sample.S = S;
            Sample.Time = Time;
            Sample.Speed = Train.GetSpeed();
            Sample.Height = F.Position.Z;
            Sample.VerticalG = G.Vertical;
            Sample.LateralG = G.Lateral;
            Sample.TangentialG = Train.GetTangentialG();

            // Unwrapped: resolved roll carries an atan2 term for world-bank
            // segments, so a bank passing through +/-pi steps by 2pi with
            // nothing physical happening, and that fake step would otherwise be
            // the reported peak every time.
            const double Span = S - PrevS;
            if (Span > 1e-9)
            {
                double D = F.Roll - PrevRoll;
                while (D > Pi) { D -= 2.0 * Pi; }
                while (D < -Pi) { D += 2.0 * Pi; }
                Sample.RollRateDegPerSec = (D / Span) * Sample.Speed * RadToDeg;
            }
            PrevRoll = F.Roll;
            PrevS = S;

            P.Samples.push_back(Sample);
            NextSampleS = S + SampleSpacingM;
        }

        if (Train.IsAtEnd())
        {
            P.bCompleted = true;
            break;
        }

        if (Train.IsRollingBack())
        {
            P.bRolledBack = true;
            P.RolledBackAtS = S;
            P.StalledAtS = S;
            P.StalledHeight = F.Position.Z;
            break;
        }

        // Checked BEFORE the stall counter, because a caught train reads exactly
        // like a stall — stopped, and staying stopped — and the two want different
        // words. The device engaging is the more specific fact, so it wins.
        if (Train.GetRollbacksCaught() > 0 && !P.bCaughtByAntiRollback)
        {
            P.bCaughtByAntiRollback = true;
            P.CaughtAtS = S;
        }

        if (Train.GetSpeed() < StallSpeed)
        {
            if (++Stalled >= StallSteps)
            {
                P.StalledAtS = S;
                P.StalledHeight = F.Position.Z;
                break;
            }
        }
        else
        {
            Stalled = 0;
        }

        Train.Step(Dt);
        Time += Dt;
    }

    P.Duration = Time;
    if (P.Samples.empty())
    {
        return P;
    }

    P.LowestHeight = P.Samples[0].Height;
    P.HighestHeight = P.Samples[0].Height;
    P.MinVerticalG = P.Samples[0].VerticalG;
    for (const FRideSample& Sm : P.Samples)
    {
        if (Sm.Speed > P.TopSpeed) { P.TopSpeed = Sm.Speed; P.TopSpeedS = Sm.S; }
        if (Sm.VerticalG > P.MaxVerticalG) { P.MaxVerticalG = Sm.VerticalG; P.MaxVerticalGS = Sm.S; }
        if (Sm.VerticalG < P.MinVerticalG) { P.MinVerticalG = Sm.VerticalG; P.MinVerticalGS = Sm.S; }
        if (std::fabs(Sm.LateralG) > P.MaxAbsLateralG)
        {
            P.MaxAbsLateralG = std::fabs(Sm.LateralG);
            P.MaxAbsLateralGS = Sm.S;
        }
        if (std::fabs(Sm.RollRateDegPerSec) > P.MaxAbsRollRate)
        {
            P.MaxAbsRollRate = std::fabs(Sm.RollRateDegPerSec);
            P.MaxAbsRollRateS = Sm.S;
        }
        if (Sm.Height < P.LowestHeight) { P.LowestHeight = Sm.Height; }
        if (Sm.Height > P.HighestHeight) { P.HighestHeight = Sm.Height; }
    }
    return P;
}

// ponytail: one train, one lap, from a standing start. No multi-train
// interaction and no block signalling, because neither exists in the physics
// prototype yet — when BlockSignal is wired in, a stall is not the only way a
// ride fails and this wants to know about held trains too.
