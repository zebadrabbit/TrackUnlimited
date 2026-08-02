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

    // Metres, nose to tail. ZERO is a point mass at the heartline, which is
    // what every result recorded before Phase 2 was measured with — so it stays
    // the default and nothing already asserted moves.
    //
    // PHASE0_FINDINGS calls length "the one omission most likely to be FELT",
    // and the mechanism is one substitution: a train's speed is governed by the
    // height of its CENTRE OF MASS, not by where its front happens to be. A
    // train straddling a crest has its mass lower than the crest, so it arrives
    // travelling faster than a point would — which is why the back car gets
    // thrown over an airtime hill harder than the front, and why point-mass
    // coaster sims are known to feel wrong.
    //
    // It is also the leading suspect for the fitted DragK landing 3.2x above
    // its physically derived value: with no length in the model, the fit had
    // nowhere to put the discrepancy except into drag.
    double TrainLength = 0.0;
};

class FTrain
{
public:
    explicit FTrain(const FTrack& InTrack, FTrainConfig InConfig = FTrainConfig())
        : Track(InTrack)
        , Config(InConfig)
        , Current(InTrack.EvaluateAt(0.0))
    {
        // Populates the sample frames. Every mean the step takes reads them, so
        // they cannot be left empty even before anyone calls Place.
        Place(0.0, 0.0);
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

        // O(track length) per sample, but only on Place — Step advances these
        // incrementally afterwards.
        const int N = SampleCount();
        Samples.resize(static_cast<std::size_t>(N));
        SampleS.resize(static_cast<std::size_t>(N));
        for (int i = 0; i < N; ++i)
        {
            SampleS[static_cast<std::size_t>(i)] = ClampS(DistanceAlong + OffsetOf(i));
            Samples[static_cast<std::size_t>(i)] =
                Track.EvaluateAt(SampleS[static_cast<std::size_t>(i)]);
        }
        Current = Samples[static_cast<std::size_t>(N / 2)];
    }

    /** Centre of the train. Front and rear are half a length either side. */
    double GetDistance() const { return DistanceAlong; }
    double GetFrontS() const { return ClampS(DistanceAlong + Config.TrainLength * 0.5); }
    double GetRearS() const { return ClampS(DistanceAlong - Config.TrainLength * 0.5); }
    double GetSpeed() const { return SpeedMs; }
    bool IsAtEnd() const { return DistanceAlong >= Track.TotalLength(); }

    // Cached, not recomputed — the step that moved the train here already paid
    // for it. EvaluateAt is O(track length), so holding onto the frame halves
    // the per-tick cost and makes the G readouts free. This is the cheap half
    // of the cached-sample-table upgrade the spline header names; the full
    // table is still not needed.
    const FTrackFrame& GetFrame() const { return Current; }

    // Lateral and vertical G at the heartline. The geometric part comes
    // straight from the track; see TrackSpline.h.
    FGForces GetForces() const { return FeltG(GetFrame(), SpeedMs); }

    // What a rider OffsetM ahead of (+) or behind (-) the train's centre feels.
    // The whole train shares one speed — it is rigid and on rails — so the
    // difference between cars is purely the curvature each one is sitting on,
    // and that is enough: the back car crests an airtime hill while the centre
    // of mass is already descending, so it is doing so faster than the front
    // car was. With TrainLength = 0 every offset returns the same thing.
    FGForces GetForcesAt(double OffsetM) const
    {
        return FeltG(GetFrameAt(OffsetM), SpeedMs);
    }

    // The sample points themselves, rear to front. Uniformly spaced along the
    // train and each one an exact frame, so anything wanting to place a car per
    // point — a renderer, say — can use these rather than re-evaluating the
    // track, which is O(track length) a time.
    int NumSamplePoints() const { return static_cast<int>(Samples.size()); }

    const FTrackFrame& GetSamplePoint(int Index) const
    {
        const int N = NumSamplePoints();
        const int Clamped = Index < 0 ? 0 : (Index > N - 1 ? N - 1 : Index);
        return Samples[static_cast<std::size_t>(Clamped)];
    }

    // Nearest sampled frame to that offset. Sampling is uniform along the
    // train, so this is exact at the ends and at the centre.
    const FTrackFrame& GetFrameAt(double OffsetM) const
    {
        const int N = static_cast<int>(Samples.size());
        if (N <= 1 || !(Config.TrainLength > 0.0))
        {
            return Current;
        }
        const double A = (OffsetM / Config.TrainLength + 0.5) * (N - 1);
        int Index = static_cast<int>(A + 0.5);
        Index = Index < 0 ? 0 : (Index > N - 1 ? N - 1 : Index);
        return Samples[static_cast<std::size_t>(Index)];
    }

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

        // Every influence is resolved to an acceleration along the track at the
        // START of the step, so that gravity, the losses and the powered
        // sections all pass through the SAME energy accounting. Applying a
        // powered section afterwards, as a per-time clamp on speed, lets it
        // ratchet against the per-distance energy exchange on a gradient and
        // manufacture energy out of nothing.
        // Averaged over the train, not read off its centre. For a point mass
        // these means are the single sample and every number below is
        // bit-identical to what it was before length existed.
        //
        // The gravity term is the one that matters: a train is a rigid body on
        // rails, so what accelerates it is the slope its whole MASS sits on. A
        // train half over a crest is still being pulled backwards by the half
        // that has not crested yet.
        const double GravityAccel = -GravityMs2 * MeanTangentZ();

        double Resistive = 0.0;
        if (SpeedMs > 0.0)
        {
            // Rolling resistance follows the normal load, so it is heavier
            // through a valley or a hard banked turn than on level track at the
            // same speed. This is the reason FeltG is worth reusing here rather
            // than assuming 1 g — and with length, a train draped through a
            // valley has only some of its wheels being pressed hard, which is
            // why this averages too.
            const double NormalG = MeanNormalG();
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
        // Continue from the cached frame rather than re-evaluating from the
        // track start: O(one tick's travel) instead of O(track length). On a
        // 425 m layout that is ~33 integrator steps a frame instead of ~42,500,
        // which is the difference between fitting in a frame and not.
        const double MeanZ0 = MeanHeight();
        AdvanceSamples(Travelled);
        const FTrackFrame F1 = Samples[Samples.size() / 2];
        const double MeanZ1 = MeanHeight();

        // Gravity: exact, no discretisation error, at any step size. Everything
        // else does work over the distance actually travelled.
        //
        // The height that matters is the CENTRE OF MASS, which is what gives a
        // train its length. Straddling a crest, its mass is lower than the
        // crest, so it does not pay the full height and arrives faster than a
        // point would — the whole reason the back car gets thrown harder.
        double SpeedSq = SpeedMs * SpeedMs
                       - 2.0 * GravityMs2 * (MeanZ1 - MeanZ0)
                       + 2.0 * (ZoneAccel - Resistive) * Travelled;

        const double NewSpeed = bStopsThisStep ? 0.0 : (SpeedSq > 0.0 ? std::sqrt(SpeedSq) : 0.0);

        LastTangentialAccel = (NewSpeed - SpeedMs) / DeltaSeconds;
        SpeedMs = NewSpeed;
        DistanceAlong = S1;
        Current = F1;
    }

private:
    double ClampS(double S) const
    {
        const double Total = Track.TotalLength();
        return S < 0.0 ? 0.0 : (S > Total ? Total : S);
    }

    // Odd, so the middle sample sits exactly on the train's centre and
    // GetFrame() needs no separate frame of its own.
    //
    // ponytail: nine points, uniformly spaced, uniform mass. A 15 m train is
    // sampled every 1.9 m, which resolves the mean height of a smooth curve
    // far better than the model's other approximations. Real cars have gaps
    // and a real train is heavier at the back when it is full — give this a
    // mass distribution the day someone wants to model a specific train.
    int SampleCount() const { return Config.TrainLength > 0.0 ? 9 : 1; }

    double OffsetOf(int Index) const
    {
        const int N = SampleCount();
        if (N <= 1)
        {
            return 0.0;
        }
        return (static_cast<double>(Index) / (N - 1) - 0.5) * Config.TrainLength;
    }

    double MeanHeight() const
    {
        double Sum = 0.0;
        for (const FTrackFrame& F : Samples)
        {
            Sum += F.Position.Z;
        }
        return Sum / static_cast<double>(Samples.size());
    }

    double MeanTangentZ() const
    {
        double Sum = 0.0;
        for (const FTrackFrame& F : Samples)
        {
            Sum += F.Tangent.Z;
        }
        return Sum / static_cast<double>(Samples.size());
    }

    double MeanNormalG() const
    {
        double Sum = 0.0;
        for (const FTrackFrame& F : Samples)
        {
            const FGForces G = FeltG(F, SpeedMs);
            Sum += std::sqrt(G.Lateral * G.Lateral + G.Vertical * G.Vertical);
        }
        return Sum / static_cast<double>(Samples.size());
    }

    // Every sample moves by the same distance — the train is rigid — so each
    // one continues from its own cached frame. O(one tick's travel) per sample
    // rather than O(track length), same reason the centre frame is cached.
    //
    // Samples clamp at the track ends, so a train part-way onto a
    // point-to-point layout has its overhanging mass piled at the endpoint
    // rather than off the end. That dilutes the mean height slightly at the
    // very start and finish; both are stations on flat track, where height is
    // not changing anyway.
    void AdvanceSamples(double Travelled)
    {
        for (std::size_t i = 0; i < Samples.size(); ++i)
        {
            const double Next = ClampS(SampleS[i] + Travelled);
            Samples[i] = Track.AdvanceFrom(Samples[i], SampleS[i], Next);
            SampleS[i] = Next;
        }
        Current = Samples[Samples.size() / 2];
    }

    const FTrack& Track;
    FTrainConfig Config;
    std::vector<FTrackZone> Zones;

    double DistanceAlong = 0.0;
    double SpeedMs = 0.0;
    double LastTangentialAccel = 0.0;
    FTrackFrame Current;

    // Rear to front. One entry for a point mass, so every mean below collapses
    // to the value it had before length existed.
    std::vector<FTrackFrame> Samples;
    std::vector<double> SampleS;
};
