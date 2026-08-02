// TrackUnlimited Phase 0 prototype: 1D train motion along the track.
// Plain C++, no engine dependencies — designed to port into UE5 C++ later.
// Design reference: Docs/PROJECT_PLAN.md Section 5, "Physics simulation".
//
// The train is a point constrained to the track, carrying one state pair:
// distance along (arc length) and speed. Not a rigid body — Chaos would fight
// for the precision this needs, and a coaster train genuinely is a 1D problem
// with the track supplying every constraint.
//
// Gravity is applied as an EXACT ENERGY EXCHANGE rather than an integrated
// force: the work done over a step is -g*dz, read straight off the track's own
// heights, so a frictionless circuit conserves energy to roundoff at any step
// size. That is the whole reason to phrase this as an energy model instead of
// summing forces — an explicit force integrator bleeds or gains energy every
// step, and on a coaster that shows up as a train that cannot quite crest a
// hill it physically should.
//
// Only the non-conservative terms (rolling resistance, air drag) and the
// powered sections need integrating, and they are the ones where a small
// timestep error is harmless.
//
// Arc length needs no inversion here: TrackSpline's parameter IS arc length to
// within a nanometre per kilometre, so advancing a train is S += v*dt.
//
// Units: metres, radians, seconds — same as TrackSpline.

#pragma once

#include "../TrackSpline/TrackSpline.h"

#include <algorithm>
#include <cmath>
#include <vector>

// A powered or braking section. One shape covers every case the plan names —
// lift chain, tire launch, brake run, station — because they differ only in
// target speed and in how hard they may push or hold back. A chain lift both
// pulls a slow train up and holds a fast one at chain speed; a friction brake
// only ever slows, which is MaxAccel = 0.
struct FTrackZone
{
    double StartS = 0.0;
    double EndS = 0.0;
    double TargetSpeed = 0.0; // m/s

    // TRACTIVE authority, not net authority: how hard the chain, tyres or
    // brakes can push on the train, before gravity and losses are subtracted.
    // A lift whose MaxAccel is below g*sin(grade) therefore fails to hold the
    // train on that grade — which is the physically correct outcome, and the
    // reason a real lift hill is specified against the steepest section it has
    // to pull.
    double MaxAccel = 0.0; // applied when the train is below target
    double MaxDecel = 0.0; // applied when the train is above target
};

inline FTrackZone MakeLift(double StartS, double EndS, double ChainSpeed, double Grip = 3.0)
{
    return {StartS, EndS, ChainSpeed, Grip, Grip};
}

inline FTrackZone MakeLaunch(double StartS, double EndS, double TargetSpeed, double Push)
{
    return {StartS, EndS, TargetSpeed, Push, 0.0};
}

inline FTrackZone MakeBrake(double StartS, double EndS, double ReleaseSpeed, double Bite)
{
    return {StartS, EndS, ReleaseSpeed, 0.0, Bite};
}

// The calibration knobs. Both are lumped coefficients standing in for a pile of
// real-world physics, and both are meant to be TUNED against reference rides
// rather than derived — a real train's resistance depends on wheel compound,
// bearing temperature, train length and how recently the track was greased,
// none of which a 1D model can see. Defaults are plausible steel-coaster
// starting points, not measurements.
struct FTrainConfig
{
    // Dimensionless rolling resistance. Scales with the normal load, so it
    // rises in a valley and through a hard banked turn and falls toward zero at
    // airtime — which is why it reads felt G rather than assuming 1 g.
    double RollingResistance = 0.006;

    // Lumped aerodynamic drag: 0.5 * rho * Cd * A / mass, so the deceleration
    // is DragK * v^2. Dominates at speed; negligible in a station.
    // 0.00045 is that formula evaluated for a loaded 7-car steel train — around
    // 8000 kg at a CdA of 5.5 m^2. Still a knob, but a defensible starting one.
    double DragK = 0.00045;
};

class FTrain
{
public:
    explicit FTrain(const FTrack& InTrack, FTrainConfig InConfig = FTrainConfig())
        : Track(InTrack)
        , Config(InConfig)
        , Current(InTrack.EvaluateAt(0.0))
    {
    }

    // Refuses a malformed zone rather than storing it, mirroring
    // FTrack::AddSegment. A negative MaxDecel would turn a brake run into an
    // unbounded launch, and an inverted span silently never fires. The
    // comparison form rejects NaN too.
    bool AddZone(const FTrackZone& Zone)
    {
        if (!(Zone.EndS > Zone.StartS) || !(Zone.TargetSpeed >= 0.0)
            || !(Zone.MaxAccel >= 0.0) || !(Zone.MaxDecel >= 0.0))
        {
            return false;
        }
        Zones.push_back(Zone);
        return true;
    }

    void Place(double S, double Speed)
    {
        DistanceAlong = std::max(0.0, std::min(Track.TotalLength(), S));
        SpeedMs = Speed;
        LastTangentialAccel = 0.0;
        Current = Track.EvaluateAt(DistanceAlong);
    }

    double GetDistance() const { return DistanceAlong; }
    double GetSpeed() const { return SpeedMs; }
    bool IsAtEnd() const { return DistanceAlong >= Track.TotalLength(); }

    // Cached, not recomputed — the step that moved the train here already paid
    // for it. EvaluateAt is O(track length), so holding onto the frame halves
    // the per-tick cost and makes the G readouts free. This is the cheap half
    // of the cached-sample-table upgrade the spline header names; the full
    // table is still not needed.
    const FFrame& GetFrame() const { return Current; }

    // Lateral and vertical G at the heartline. The geometric part comes
    // straight from the track; see TrackSpline.h.
    FGForces GetForces() const { return FeltG(GetFrame(), SpeedMs); }

    // Fore-aft G, the axis the track alone cannot know — what the rider feels
    // under launch and braking. Positive presses them back into the seat.
    // Averaged over the last step.
    //
    // Same apparent-gravity convention as GetForces, and the gravity term is
    // not optional: a freely rolling train feels NO fore-aft force even on a
    // 45-degree drop, because gravity is accelerating it rather than pushing
    // against it. Reporting raw dv/dt would put phantom fore-aft load on every
    // slope in the ride. (Lateral and vertical are untouched by this term —
    // tangential acceleration is perpendicular to both axes by construction.)
    double GetTangentialG() const
    {
        return (LastTangentialAccel + GravityMs2 * Current.Tangent.Z) / GravityMs2;
    }

    void Step(double DeltaSeconds)
    {
        // Rejects zero, negative and NaN in one comparison.
        if (!(DeltaSeconds > 0.0))
        {
            return;
        }

        const double Total = Track.TotalLength();
        const double S0 = DistanceAlong;
        const FFrame& F0 = Current;

        // Every influence is resolved to an acceleration along the track at the
        // START of the step, so that gravity, the losses and the powered
        // sections all pass through the SAME energy accounting. Applying a
        // powered section afterwards, as a per-time clamp on speed, lets it
        // ratchet against the per-distance energy exchange on a gradient and
        // manufacture energy out of nothing.
        const double GravityAccel = -GravityMs2 * F0.Tangent.Z;

        double Resistive = 0.0;
        if (SpeedMs > 0.0)
        {
            // Rolling resistance follows the normal load, so it is heavier
            // through a valley or a hard banked turn than on level track at the
            // same speed. This is the reason FeltG is worth reusing here rather
            // than assuming 1 g.
            const FGForces G = FeltG(F0, SpeedMs);
            const double NormalG = std::sqrt(G.Lateral * G.Lateral + G.Vertical * G.Vertical);
            Resistive = Config.RollingResistance * NormalG * GravityMs2
                      + Config.DragK * SpeedMs * SpeedMs;
        }

        // A powered section asks for whatever acceleration would land the train
        // exactly on its target speed this step, then gets clamped to what it
        // can actually deliver. That makes MaxAccel *tractive* authority rather
        // than net authority: a chain that cannot out-pull gravity on a hill
        // fails to hold the train, which is the physically correct outcome.
        bool bInZone = false;
        double ZoneAccel = 0.0;
        for (const FTrackZone& Zone : Zones)
        {
            if (S0 < Zone.StartS || S0 > Zone.EndS)
            {
                continue;
            }
            const double Needed = (Zone.TargetSpeed - SpeedMs) / DeltaSeconds - GravityAccel + Resistive;
            const double Applied = std::max(-Zone.MaxDecel, std::min(Zone.MaxAccel, Needed));
            // Overlapping zones are an authoring error, but a ride control
            // system should fail toward the slower answer, so the most
            // restrictive one wins rather than whichever was added last.
            ZoneAccel = bInZone ? std::min(ZoneAccel, Applied) : Applied;
            bInZone = true;
        }

        const double Accel = GravityAccel + ZoneAccel - Resistive;

        // Position carries the acceleration term. Without it, v == 0 is an
        // absorbing state on any gradient: a stationary train never moves, so
        // never changes height, so never gains speed — and a cart placed at the
        // top of a drop just sits there.
        double Advance;
        bool bStopsThisStep = false;
        if (Accel < 0.0 && SpeedMs + Accel * DeltaSeconds < 0.0)
        {
            // Comes to rest partway through the step: advance exactly the
            // distance it takes to stop. The speed is then forced to zero
            // rather than derived — S1 - S0 is not bit-identical to Advance
            // once S0 dwarfs it, and the residue leaves the train creeping at
            // 1e-9 m/s forever instead of standing still.
            Advance = -0.5 * SpeedMs * SpeedMs / Accel;
            bStopsThisStep = true;
        }
        else
        {
            Advance = SpeedMs * DeltaSeconds + 0.5 * Accel * DeltaSeconds * DeltaSeconds;
        }
        // ponytail: a train that runs out of energy stops dead instead of
        // rolling back. Reversal needs a signed velocity through the whole model
        // and the block system has to hear about it, which is a Phase 2/3
        // conversation; this is enough to prove ride feel on a track authored to
        // make it round. A valley-stall here is a design error to surface, not a
        // state to simulate.
        Advance = std::max(0.0, Advance);

        const double S1 = std::max(0.0, std::min(Total, S0 + Advance));
        const double Travelled = S1 - S0;
        const FFrame F1 = Track.EvaluateAt(S1);

        // Gravity: exact, no discretisation error, at any step size. Everything
        // else does work over the distance actually travelled.
        double SpeedSq = SpeedMs * SpeedMs
                       - 2.0 * GravityMs2 * (F1.Position.Z - F0.Position.Z)
                       + 2.0 * (ZoneAccel - Resistive) * Travelled;

        const double NewSpeed = bStopsThisStep ? 0.0 : (SpeedSq > 0.0 ? std::sqrt(SpeedSq) : 0.0);

        LastTangentialAccel = (NewSpeed - SpeedMs) / DeltaSeconds;
        SpeedMs = NewSpeed;
        DistanceAlong = S1;
        Current = F1;
    }

private:
    const FTrack& Track;
    FTrainConfig Config;
    std::vector<FTrackZone> Zones;

    double DistanceAlong = 0.0;
    double SpeedMs = 0.0;
    double LastTangentialAccel = 0.0;
    FFrame Current;
};
