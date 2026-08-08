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
#include <utility>
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

    // ===================== THE BRAKE, AS A SECOND DEVICE =====================
    //
    // A real block brake is TWO machines sharing a stretch of track: a friction
    // or magnetic pad that can only ever remove energy, and drive tyres that can
    // push and hold. NL2 exposes them as separate devices with separate speeds
    // and separate rates, and it is right to — they are separate hardware, they
    // are specified separately, and one of them can fail without the other.
    //
    // Everything above this line is the TRANSPORT: a SETPOINT, driven toward
    // from either side. Everything below is the BRAKE: a CEILING, never a
    // setpoint. A train already under the limit is untouched, which is the whole
    // difference and the reason a trim cannot start a train — a ceiling can only
    // reduce.
    //
    // Collapsing the two into one number is what `ServeHolds` documents itself
    // doing: "commanding a crawl speed says both stages in one number". It gets
    // the right answer for the sequence and it cannot express a brake that bites
    // at 8 m/s^2 conveying at 0.5, which is an ordinary specification.
    //
    // NEGATIVE MEANS NO BRAKE DEVICE, so every zone built before this existed
    // behaves exactly as it did.
    double BrakeLimit = -1.0;   // m/s ceiling; < 0 is "no pad on this section"
    double BrakeDecel = 0.0;    // m/s^2, its own rate, not the transport's
};

// A section with drive tyres AND a pad, which is what every real block brake is.
// The pad bites at its own rate; the tyres convey at theirs.
inline FTrackZone MakeBlockBrake(double StartS, double EndS, double ConveySpeed,
                                 double Grip, double PadLimit, double PadBite)
{
    FTrackZone Z{StartS, EndS, ConveySpeed, Grip, Grip};
    Z.BrakeLimit = PadLimit;
    Z.BrakeDecel = PadBite;
    return Z;
}

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
    //
    // 0.024, and the previous 0.006 was justified against the WRONG MATERIAL.
    // It was written down as "a plausible steel-on-steel figure", but steel
    // wheel on steel rail is a railway — coaster running wheels are
    // POLYURETHANE on steel, typically around 95A, and the hysteresis loss as
    // that elastomer deforms under load is a real and continuous drain. Published
    // figures for polyurethane on steel sit around 0.01–0.03; steel-on-steel is
    // nearer 0.001–0.002, so the old default was not even right for the material
    // it claimed.
    //
    // Measured, not guessed: three NoLimits 2 recordings — two coaster types on
    // two layouts — fit 0.022 to 0.026. The cleanest is a purpose-built 621 m
    // flat coast-down launched at exactly 30 km/h, which fits Crr = 0.02602 with
    // a residual of 0.0005 m/s² against decelerations of 0.1–0.2. That is a 0.3%
    // fit, and it says the MODEL's shape was right all along; only this constant
    // was wrong. Reproduced bit-identically across two separate recordings of the
    // same configuration.
    //
    // It also explains something that made no sense before: rolling resistance
    // outweighing air drag 3:1 even at 100 km/h. Absurd for a steel railway
    // wheel, exactly right for polyurethane tyres.
    double RollingResistance = 0.026;

    // Lumped aerodynamic drag: 0.5 * rho * Cd * A / mass, so the deceleration
    // is DragK * v^2. Dominates at speed; negligible in a station.
    //
    // MEASURED 2026-08-06, and the previous 0.00045 was DERIVED and 4.5x too
    // high. That derivation evaluated the formula for a loaded 7-car steel train
    // — 8000 kg at CdA 5.5 m^2 — and was never checked against a ride, because
    // the only recording available topped out at 44.5 km/h where drag is under a
    // tenth of the loss and correlates with rolling resistance at 0.975.
    //
    // A purpose-built dead-flat straight launched to 142.5 km/h settled it. Over
    // the coast from 142.3 to 97.2 km/h, drag runs 38% of the loss down to 22% —
    // large enough to see and varying enough to separate — and the fit gives
    // 0.000100 with a residual of 0.00014 m/s^2 against decelerations of
    // 0.33-0.41. A 0.04% fit.
    //
    // Three things make it a measurement rather than a fit: rolling resistance
    // came out 0.02603 against the 0.02602 of a completely separate 30 km/h
    // recording on different track; the fast and slow halves of the coast fitted
    // separately give identical coefficients, where a drag term being absorbed
    // into friction would make them trade off; and DragK holds steady across
    // every train length, where before it flipped sign.
    //
    // What moved when this landed: nothing geometric, and every dynamic figure.
    // The reference layout's loop apex went +1.16 -> +1.78 g and the circuit's
    // lap 105 -> 100 s. The old numbers were a correct simulation of a train
    // dragged 4.5x harder than a real one.
    double DragK = 0.000100;

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

    // Whether a train that runs out of energy rolls BACK down the hill instead
    // of stopping dead where it is.
    //
    // Off by default, and not out of caution: stopping dead is what every
    // number recorded before Phase 2 was measured against, and one test asserts
    // it directly ("stays stopped rather than drifting or reversing"). Rolling
    // back is the more honest physics, but it is a behaviour change, so it is a
    // choice rather than a surprise.
    //
    // It is also not purely an improvement. A valley stall is a DESIGN ERROR to
    // surface, and a train that rolls back oscillates and eventually settles,
    // which can look like the ride working. The reporting is what makes it
    // useful — see FRideProfile::bRolledBack, which says where it happened.
    bool bAllowRollback = false;

    // Whether the end of the track joins the start, so a train runs LAPS instead
    // of stopping at the end.
    //
    // Off by default and NOT a preference: on a layout that does not close, a
    // wrap teleports the train across whatever gap is there and calls it
    // continuity. The caller is expected to have MEASURED closure — position,
    // heading and roll — before setting it. ATUCoasterRide does exactly that and
    // turns it on only for the layout that passes.
    bool bCircuit = false;
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

    // Retarget a zone while the ride is running. This is the whole difference
    // between a fixed trim brake and a BLOCK brake: a block brake holds at zero
    // until the signalling grants a permissive, then releases.
    //
    // Validated on the same term AddZone validates, so a NaN cannot arrive
    // through the back door and land in the energy accounting — where it would
    // spread to speed, distance and every frame after it.
    bool SetZoneTargetSpeed(std::size_t Index, double Speed)
    {
        if (Index >= Zones.size() || !(Speed >= 0.0))
        {
            return false;
        }
        Zones[Index].TargetSpeed = Speed;
        return true;
    }

    // THE PAD IS COMMANDED SEPARATELY FROM THE TYRES, which is the whole reason
    // they are two fields. A block brake's sequence is "pad bites, train stops,
    // pad releases, tyres convey" — four states of two devices, and a single
    // commanded speed can only say two of them.
    //
    // A NEGATIVE LIMIT RELEASES THE PAD, rather than there being a separate
    // release call. One number with a documented out-of-band value beats two
    // calls that can disagree about whether the pad is on.
    bool SetZoneBrakeLimit(std::size_t Index, double LimitMs)
    {
        if (Index >= Zones.size())
        {
            return false;
        }
        Zones[Index].BrakeLimit = LimitMs;
        return true;
    }

    double GetZoneBrakeLimit(std::size_t Index) const
    {
        return Index < Zones.size() ? Zones[Index].BrakeLimit : -1.0;
    }

    // ===================== A DEVICE THAT DOES NOT DELIVER =====================
    //
    // How much of its authored authority this device is actually producing,
    // 0..1. ONE is a healthy device and is the default, so nothing measured
    // moves until something injects a fault.
    //
    // This is what makes A FAILED BRAKE expressible, which FAULTS.md recorded as
    // the one fault nothing could model and the one with the least protection.
    // A brake that does not bite is not a brake commanded wrongly — the command
    // is right and the pad is glazed, the pressure is low, the tyre is worn. So
    // it scales what the device DELIVERS, leaving the command untouched, which
    // is also why the drive layer's Commanded-vs-Actual disagreement is the
    // thing that would detect it.
    //
    // Deliberately NOT a field on FTrackZone: a zone is authored data — extent,
    // target, authorities — and health is runtime fault state. Putting it there
    // would mean a track file could ship a broken brake, which is a different
    // and much worse idea.
    //
    // Scales BOTH authorities, because it is one device. A block brake is pads
    // and drive tyres in a single zone, and a failure that took out only one of
    // them would be modelling a distinction this vocabulary does not draw.
    bool SetZoneHealth(std::size_t Index, double Fraction)
    {
        if (Index >= Zones.size() || !(Fraction >= 0.0) || !(Fraction <= 1.0))
        {
            return false;
        }
        ZoneHealth.resize(Zones.size(), 1.0);
        ZoneHealth[Index] = Fraction;
        return true;
    }

    double GetZoneHealth(std::size_t Index) const
    {
        return Index < ZoneHealth.size() ? ZoneHealth[Index] : 1.0;
    }

    // Its current target, or negative if there is no such zone — so a dispatcher
    // can stash the authored speed it is about to overwrite without a second
    // bounds check, and put it back on release.
    double GetZoneTargetSpeed(std::size_t Index) const
    {
        return Index < Zones.size() ? Zones[Index].TargetSpeed : -1.0;
    }

    // The zone itself, by value, with a degenerate one for a bad index rather than
    // a reference into the vector — a dispatcher wants its span to work out WHERE
    // to stop a train, and a dangling reference is a poor reward for asking.
    FTrackZone GetZone(std::size_t Index) const
    {
        return Index < Zones.size() ? Zones[Index] : FTrackZone{};
    }

    // A stretch of track a train cannot roll backwards through: ratchets, chain
    // dogs, a catch car. NOT a zone — it has no speed, no authority and nothing to
    // command, which is the whole difference between a safety device and a control
    // device. A zone decides how fast; this decides which way is possible.
    //
    // Refused rather than stored if inverted, exactly as AddZone refuses a
    // malformed zone: a backwards span would silently never engage, and the only
    // symptom would be a train rolling through a lift hill it should have been
    // caught on.
    bool AddAntiRollback(double StartS, double EndS)
    {
        if (!(EndS > StartS))
        {
            return false;
        }
        Catches.push_back({StartS, EndS});
        return true;
    }

    bool IsAntiRollbackAt(double S) const
    {
        for (const std::pair<double, double>& C : Catches)
        {
            if (S >= C.first && S <= C.second)
            {
                return true;
            }
        }
        return false;
    }

    // How many times the catch has ENGAGED — rising edges, not steps held, so it
    // is a number that means the same thing at any timestep. Reported rather than
    // hidden, because a train being held by a ratchet means the ride FAILED to get
    // round: the device did its job and the layout did not. Same reasoning as
    // FRideProfile reporting a stall instead of quietly restarting.
    int GetRollbacksCaught() const { return RollbacksCaught; }

    // Right now, this instant. What a control panel lamp would read.
    bool IsHeldByCatch() const { return bHeldByCatch; }

    // The zone at S that can both STOP a train and START it again — brakes with
    // drive tyres, which is what a real block brake is. -1 if there is none.
    //
    // BOTH authorities are required, and each exclusion is the point of the
    // function rather than a detail of it:
    //
    //   MaxDecel 0 is a LAUNCH. It can push and cannot hold, so commanding it to
    //   zero does not stop a train — it only declines to push one that is already
    //   moving. Gating a launch mid-launch aborts it, which is the opposite of an
    //   interlock.
    //
    //   MaxAccel 0 is a FRICTION BRAKE. It can stop a train and can NEVER release
    //   it. Park a train there and it stands for the rest of the session, with no
    //   symptom but a ride that quietly stopped.
    //
    // So a dispatcher asks this rather than "which zone is S in": a trim brake is
    // not somewhere you may hold a train, however convenient its arc length looks.
    //
    // Whether the device can stop the train in the length available is a separate
    // question, and one only the layout can answer — v^2/2a against the block
    // length. On the two-train preset the mid-course brake fails it (28.19 m/s
    // arriving, 66.2 m needed, 45 m available) and is a trim, not a block brake.
    int FindHoldZoneAt(double S) const
    {
        for (std::size_t i = 0; i < Zones.size(); ++i)
        {
            if (S >= Zones[i].StartS && S <= Zones[i].EndS
                && Zones[i].MaxAccel > 0.0 && Zones[i].MaxDecel > 0.0)
            {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    // Is this train's centre inside that zone? Deliberately the plain span test
    // FindHoldZoneAt does WITHOUT the both-authorities filter — a drive wants to
    // know whether it has a train on it, which a trim brake and a launch also do.
    bool IsInZone(std::size_t Index, double S) const
    {
        return Index < Zones.size() && S >= Zones[Index].StartS && S <= Zones[Index].EndS;
    }

    // THE MOTOR'S FEEDBACK. What that zone's drive actually delivered on the last
    // step, in m/s^2 of tractive acceleration — positive pushing, negative holding
    // back. Nothing new is computed for this: Step already clamps the acceleration
    // a zone asks for to what the zone HAS, and that clamped value is the force on
    // the train. Reporting it is the whole of it.
    double GetZoneAppliedAccel(std::size_t Index) const
    {
        return Index < ZoneApplied.size() ? ZoneApplied[Index] : 0.0;
    }

    // The same number as a fraction of that zone's authority, which is TORQUE —
    // the bar on a VFD panel. One is the drive flat out with nothing left to give;
    // a lift chain holding a train at steady speed on a gradient sits at whatever
    // fraction the gradient and the losses need, which is the reading that says
    // how close a hill is to being too steep for its chain.
    //
    // NOT reliable for OVERLAPPING zones: each reports what it asked for, and only
    // the most restrictive one actually acted. Overlaps are an authoring error the
    // validator already reports, and resolving them here would hide it.
    double GetZoneLoad(std::size_t Index) const
    {
        if (Index >= ZoneApplied.size())
        {
            return 0.0;
        }
        const double A = ZoneApplied[Index];
        const double Authority = A >= 0.0 ? Zones[Index].MaxAccel : Zones[Index].MaxDecel;
        return Authority > 0.0 ? std::min(1.0, std::fabs(A) / Authority) : 0.0;
    }

    void Place(double S, double Speed)
    {
        DistanceAlong = std::max(0.0, std::min(Track.TotalLength(), S));
        VelocityMs = Speed;
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

    /** Metres, nose to tail. Zero for a point mass. */
    double GetLength() const { return Config.TrainLength; }

    /**
     * Centre of the train. Front and rear are half a length either side.
     *
     * "FRONT" MEANS THE +S END, NOT THE LEADING EDGE, and the names read as
     * though they promise otherwise. On a train running backwards the leading
     * edge is GetRearS().
     *
     * Nothing above is currently wrong because of it: every consumer takes the
     * PAIR and treats it as a span — FTrackSensors::Cover is a symmetric
     * interval test and FRideSignals::Update is a range diff, both deliberately.
     * So the trap is not a bug, it is the next person writing
     * `if (Train.GetFrontS() > Mark)` and being right for a year.
     *
     * If a leading-edge rule is ever genuinely needed, it wants asking for by
     * that name rather than inferring here — see Docs/DIRECTION_AND_ROUTES.md.
     */
    double GetDistance() const { return DistanceAlong; }
    double GetFrontS() const { return ClampS(DistanceAlong + Config.TrainLength * 0.5); }
    double GetRearS() const { return ClampS(DistanceAlong - Config.TrainLength * 0.5); }

    // Signed: negative means rolling backwards. GetSpeed stays a MAGNITUDE, so
    // every caller and every assertion written before rollback existed keeps
    // meaning what it meant.
    double GetVelocity() const { return VelocityMs; }
    bool IsRollingBack() const { return VelocityMs < 0.0; }
    double GetSpeed() const { return std::fabs(VelocityMs); }

    // A circuit has no end to be at, which is the point of it. Everything that
    // stops on this — the ride profile's walk, the actor's return-to-station —
    // therefore keeps working unchanged on an open layout and simply never fires
    // on a closed one.
    bool IsAtEnd() const { return !Config.bCircuit && DistanceAlong >= Track.TotalLength(); }

    // Set after construction rather than only in the config because the RIDE
    // PROFILE has to be taken with it off: the profile walks arc length forwards
    // and stops at the end, so on a circuit it would lap for ever and its samples
    // would overwrite each other. Build open, measure, then close.
    void SetCircuit(bool bIn) { Config.bCircuit = bIn; }
    bool IsCircuit() const { return Config.bCircuit; }

    // Cached, not recomputed — the step that moved the train here already paid
    // for it. EvaluateAt is O(track length), so holding onto the frame halves
    // the per-tick cost and makes the G readouts free. This is the cheap half
    // of the cached-sample-table upgrade the spline header names; the full
    // table is still not needed.
    const FTrackFrame& GetFrame() const { return Current; }

    // Lateral and vertical G at the heartline. The geometric part comes
    // straight from the track; see TrackSpline.h.
    FGForces GetForces() const { return FeltG(GetFrame(), VelocityMs); }

    // What a rider OffsetM ahead of (+) or behind (-) the train's centre feels.
    // The whole train shares one speed — it is rigid and on rails — so the
    // difference between cars is purely the curvature each one is sitting on,
    // and that is enough: the back car crests an airtime hill while the centre
    // of mass is already descending, so it is doing so faster than the front
    // car was. With TrainLength = 0 every offset returns the same thing.
    FGForces GetForcesAt(double OffsetM) const
    {
        return FeltG(GetFrameAt(OffsetM), VelocityMs);
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

        // Sign of travel: resistance always opposes it, which is what makes
        // this correct in reverse rather than an energy source.
        const double Dir = VelocityMs > 0.0 ? 1.0 : (VelocityMs < 0.0 ? -1.0 : 0.0);

        double Resistive = 0.0;
        if (Dir != 0.0)
        {
            // Rolling resistance follows the normal load, so it is heavier
            // through a valley or a hard banked turn than on level track at the
            // same speed. This is the reason FeltG is worth reusing here rather
            // than assuming 1 g — and with length, a train draped through a
            // valley has only some of its wheels being pressed hard, which is
            // why this averages too.
            const double NormalG = MeanNormalG();
            Resistive = Config.RollingResistance * NormalG * GravityMs2
                      + Config.DragK * VelocityMs * VelocityMs;
        }

        // A powered section asks for whatever acceleration would land the train
        // exactly on its target speed this step, then gets clamped to what it
        // can actually deliver. That makes MaxAccel *tractive* authority rather
        // than net authority: a chain that cannot out-pull gravity on a hill
        // fails to hold the train, which is the physically correct outcome.
        bool bInZone = false;
        double ZoneAccel = 0.0;
        // Per-zone tractive force, kept so a drive can report its own torque. An
        // assign on a same-sized vector does not reallocate, so this costs a fill.
        ZoneApplied.assign(Zones.size(), 0.0);
        for (std::size_t zi = 0; zi < Zones.size(); ++zi)
        {
            const FTrackZone& Zone = Zones[zi];
            if (S0 < Zone.StartS || S0 > Zone.EndS)
            {
                continue;
            }
            const double Needed = (Zone.TargetSpeed - VelocityMs) / DeltaSeconds - GravityAccel + Resistive;
            // What the device DELIVERS, which on a healthy one is all of it.
            const double Health = zi < ZoneHealth.size() ? ZoneHealth[zi] : 1.0;
            double Applied = std::max(-Zone.MaxDecel * Health,
                                      std::min(Zone.MaxAccel * Health, Needed));

            // ---- AND THE PAD, WHICH IS A SECOND DEVICE ON THE SAME TRACK.
            //
            // A CEILING, NOT A SETPOINT: a train already under the limit is
            // untouched, so this can never add energy however it is authored.
            // That is what makes it a brake rather than a slow drive, and the
            // std::min(0.0, ...) is where the guarantee lives — not in a comment
            // asking callers to pass sensible numbers.
            //
            // The harder of the two wins. Both machines are physically on the
            // train, and a ride control system fails toward the slower answer —
            // the same rule overlapping zones already follow one level down.
            if (Zone.BrakeLimit >= 0.0 && VelocityMs > Zone.BrakeLimit)
            {
                const double PadNeeded =
                    (Zone.BrakeLimit - VelocityMs) / DeltaSeconds - GravityAccel + Resistive;
                const double PadApplied =
                    std::max(-Zone.BrakeDecel * Health, std::min(0.0, PadNeeded));
                Applied = std::min(Applied, PadApplied);
            }
            ZoneApplied[zi] = Applied;
            // Overlapping zones are an authoring error, but a ride control
            // system should fail toward the slower answer, so the most
            // restrictive one wins rather than whichever was added last.
            ZoneAccel = bInZone ? std::min(ZoneAccel, Applied) : Applied;
            bInZone = true;
        }

        const double Accel = GravityAccel + ZoneAccel - Dir * Resistive;

        // Position carries the acceleration term. Without it, v == 0 is an
        // absorbing state on any gradient: a stationary train never moves, so
        // never changes height, so never gains speed — and a cart placed at the
        // top of a drop just sits there.
        double Advance;
        bool bStopsThisStep = false;
        // Turns round this step: velocity and its end-of-step value disagree in
        // sign. Written this way rather than "decelerating past zero" so it is
        // symmetric — a train rolling backwards up the far side of a valley
        // turns round by exactly the same rule.
        const double VEnd = VelocityMs + Accel * DeltaSeconds;
        const bool bTurns = (VelocityMs > 0.0 && VEnd < 0.0) || (VelocityMs < 0.0 && VEnd > 0.0);
        if (bTurns)
        {
            // Comes to rest partway through the step: advance exactly the
            // distance it takes to stop. The speed is then forced to zero
            // rather than derived — S1 - S0 is not bit-identical to Advance
            // once S0 dwarfs it, and the residue leaves the train creeping at
            // 1e-9 m/s forever instead of standing still.
            // Sign-correct in both directions already: forwards this is
            // positive, and rolling backwards both numerator and denominator
            // flip so it comes out negative.
            Advance = -0.5 * VelocityMs * VelocityMs / Accel;
            bStopsThisStep = true;
        }
        else
        {
            Advance = VelocityMs * DeltaSeconds + 0.5 * Accel * DeltaSeconds * DeltaSeconds;
        }
        // Without rollback a train that runs out of energy stops dead: the
        // backwards advance is thrown away, so it never changes height and
        // never regains speed. That was the Phase 0 behaviour and it is still
        // the default, because a valley stall is a design error to surface
        // rather than a state to simulate. With rollback on, the same
        // arithmetic simply keeps its sign.
        //
        // ANTI-ROLLBACK is that same clamp applied to one stretch of track rather
        // than to the whole ride. It is the most universal safety device on a
        // coaster — every lift hill ever built has ratchets, dogs or a catch car —
        // and it does not slow a train or hold it at a speed. It makes backward
        // movement IMPOSSIBLE, which is exactly what throwing away a negative
        // advance already does, so the behaviour is inherited rather than invented.
        //
        // Modelled as a continuous ratchet: infinitely fine dog spacing, so a
        // caught train gives up nothing at all. A real ratchet lets it fall back to
        // the last dog first, typically under 150 mm. Give the span a pitch when
        // somebody wants to hear the clack.
        const bool bCaught = Advance < 0.0 && IsAntiRollbackAt(S0);
        if (!Config.bAllowRollback || bCaught)
        {
            Advance = std::max(0.0, Advance);
        }
        if (bCaught)
        {
            // Counted on the RISING EDGE, so this is "the catch engaged once",
            // not "the catch was engaged for 56,685 steps". A held train trips the
            // test every step it is held, and a number that grows with the
            // timestep is not a number anybody can report.
            if (!bHeldByCatch)
            {
                ++RollbacksCaught;
            }
            bHeldByCatch = true;
            // ARRESTED, not merely stopped from moving. Throwing the advance away
            // on its own leaves the train with its backward velocity intact and
            // nowhere to spend it — pinned in place at 4 m/s for ever, which is
            // not a state any hardware can be in. A dog takes the kinetic energy
            // into the structure, so the train's is gone.
            bStopsThisStep = true;
        }
        else
        {
            bHeldByCatch = false;
        }

        // On a circuit the advance is never clipped, so the distance travelled is
        // the advance itself. Reading it back off S1 - S0 would be NEGATIVE by a
        // whole lap the step the train crosses the seam, which would reverse the
        // gravity and resistance work for that step.
        const double S1 = Config.bCircuit ? ClampS(S0 + Advance)
                                          : std::max(0.0, std::min(Total, S0 + Advance));
        const double Travelled = Config.bCircuit ? Advance : S1 - S0;
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
        // Resistance is charged over the DISTANCE COVERED, whichever way that
        // was. Multiplying it by a signed travel would turn friction into an
        // energy source the moment a train rolled backwards.
        double SpeedSq = VelocityMs * VelocityMs
                       - 2.0 * GravityMs2 * (MeanZ1 - MeanZ0)
                       + 2.0 * ZoneAccel * Travelled
                       - 2.0 * Resistive * std::fabs(Travelled);

        // Direction is carried from the velocity rather than read back off the
        // distance travelled: at the end of the track the advance is clamped to
        // zero, and inferring the sign from that would stop a moving train dead.
        double NewDir = VelocityMs > 0.0 ? 1.0 : (VelocityMs < 0.0 ? -1.0 : 0.0);
        if (NewDir == 0.0)
        {
            NewDir = Advance < 0.0 ? -1.0 : 1.0;
        }
        const double NewSpeed =
            bStopsThisStep ? 0.0 : NewDir * (SpeedSq > 0.0 ? std::sqrt(SpeedSq) : 0.0);

        LastTangentialAccel = (NewSpeed - VelocityMs) / DeltaSeconds;
        VelocityMs = NewSpeed;
        DistanceAlong = S1;
        Current = F1;
    }

private:
    // Wraps on a circuit, clamps otherwise. Every arc length in this class goes
    // through here — the train's own distance, the nose, the tail, every sample —
    // so a circuit needs no second code path anywhere else.
    double ClampS(double S) const
    {
        const double Total = Track.TotalLength();
        if (!(Total > 0.0))
        {
            return 0.0;
        }
        if (Config.bCircuit)
        {
            double W = std::fmod(S, Total);
            if (W < 0.0)
            {
                W += Total;   // fmod keeps the sign of S; a lap has no sign
            }
            return W;
        }
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
            const FGForces G = FeltG(F, VelocityMs);
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
        const double Total = Track.TotalLength();
        for (std::size_t i = 0; i < Samples.size(); ++i)
        {
            const double Raw = SampleS[i] + Travelled;
            const double Next = ClampS(Raw);
            // Crossing the seam, the wrapped target is BEHIND the cached frame, and
            // AdvanceFrom would walk a whole lap backwards to reach it. Re-evaluate
            // from the track start instead: O(track length), but once per sample
            // per lap — about nine calls every hundred seconds, against the ~33
            // integrator steps a frame this exists to avoid. The frame it returns
            // is the canonical start frame, which is why the seam has to be closed
            // in HEADING and ROLL and not merely in position: any residual there
            // shows up as a pop, once a lap, in exactly the same place.
            if (Config.bCircuit && (Raw >= Total || Raw < 0.0))
            {
                Samples[i] = Track.EvaluateAt(Next);
            }
            else
            {
                Samples[i] = Track.AdvanceFrom(Samples[i], SampleS[i], Next);
            }
            SampleS[i] = Next;
        }
        Current = Samples[Samples.size() / 2];
    }

    const FTrack& Track;
    FTrainConfig Config;
    std::vector<FTrackZone> Zones;

    // Tractive acceleration each zone delivered on the last step, so a drive can
    // report its own torque without the physics having to be asked twice. Written
    // by Step, sized from Zones there — a zone added mid-run therefore reads zero
    // until the next step rather than indexing out of range.
    std::vector<double> ZoneApplied;
    // Per zone, 0..1. Empty means every device is healthy, so the common
    // case costs nothing and no existing caller has to know this exists.
    std::vector<double> ZoneHealth;

    // Arc-length spans a train cannot roll back through. A pair rather than a
    // struct because that is genuinely all a catch is — no speed, no authority,
    // no state. If it ever grows a dog pitch it earns a struct.
    std::vector<std::pair<double, double>> Catches;
    int RollbacksCaught = 0;
    bool bHeldByCatch = false;

    double DistanceAlong = 0.0;
    // SIGNED. GetSpeed() reports the magnitude, so callers written before
    // rollback existed are unaffected.
    double VelocityMs = 0.0;
    double LastTangentialAccel = 0.0;
    FTrackFrame Current;

    // Rear to front. One entry for a point mass, so every mean below collapses
    // to the value it had before length existed.
    std::vector<FTrackFrame> Samples;
    std::vector<double> SampleS;
};
