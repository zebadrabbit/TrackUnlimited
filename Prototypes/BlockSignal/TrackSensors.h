// TrackUnlimited: the sensors a ride control system actually reads.
//
// Until this existed, FRideSignals was HANDED the truth — a train's exact rear
// and front arc length, every frame, for free. That is a cheat, and it is the
// difference between simulating a control system and simulating the answer a
// control system would have got. A real PLC has no idea where a train is. It
// knows that a switch at 872.1 m went high, and nothing else.
//
// So the layering is:
//
//   FTrain span  ->  SENSORS (physical)  ->  PLC (logical)
//
// This is the middle box, and it is the ONLY layer entitled to know where a
// train really is — because a proximity switch is a physical device that a
// physical train physically covers. Everything downstream reads booleans.
//
// WHAT A REAL ONE IS. A proximity switch is an electromagnetic sensor at a fixed
// point, tripped by a metal flag under each car. Counting flags is how a
// controller knows the WHOLE train has passed rather than just the front of it —
// which is the hardware's version of this project's nose-and-tail range, arrived
// at from the opposite direction. Photo eyes (a broken light beam) and mechanical
// limit switches do the same job less commonly.
//
// WHAT IT DELIBERATELY DOES NOT REPORT: which train. A switch says "metal is over
// me" and nothing more. Identity is something the logic layer INFERS from the
// order things trip in, which is only unambiguous because no two trains may share
// a block — the interlocking and the identity tracking hold each other up. Storing
// a train index in here would be inventing information the hardware does not have.
//
// Reference: proximity switches, photo eyes and limit switches as described in
// Weisenberger, "Coasters 101" (3rd ed.), chapter 6.

#pragma once

#include <cstddef>
#include <vector>

// One sensor's state, as the logic layer would see it on a scan.
struct FSensorReading
{
    bool bBlocked = false;   // something is over it right now
    int Rising = 0;          // times it has gone clear -> blocked
    int Falling = 0;         // times it has gone blocked -> clear
};

class FTrackSensors
{
public:
    // Positions are arc lengths in metres. Order does not matter to the sensors
    // themselves — they are independent devices — but a caller that keeps them
    // ascending can index them alongside block boundaries.
    explicit FTrackSensors(std::vector<double> InPositions)
        : At(std::move(InPositions))
        , State(At.size())
        , bCoveredThisScan(At.size(), false)
    {
    }

    std::size_t Num() const { return At.size(); }
    double PositionOf(std::size_t Sensor) const { return At[Sensor]; }

    // Begin a scan. Coverage is accumulated across every train and resolved in
    // EndScan, because a sensor is one device: two trains cannot each decide
    // whether it is blocked, and the second must not clear what the first covers.
    void BeginScan()
    {
        for (std::size_t i = 0; i < bCoveredThisScan.size(); ++i)
        {
            bCoveredThisScan[i] = false;
        }
    }

    // One train's span. Called once per train per scan, between Begin and End.
    //
    // bCircuit and Total exist for the seam: a train straddling it has a rear
    // GREATER than its front, and on a circuit that is a real state rather than a
    // caller error — the train covers [Rear, Total) and [0, Front].
    void Cover(double RearS, double FrontS, bool bCircuit = false, double Total = 0.0)
    {
        const bool bWraps = FrontS < RearS;
        if (bWraps && !bCircuit)
        {
            const double T = RearS;
            RearS = FrontS;
            FrontS = T;
        }
        for (std::size_t i = 0; i < At.size(); ++i)
        {
            const double S = At[i];
            const bool bHit = (bWraps && bCircuit)
                ? (S >= RearS || S <= FrontS)
                : (S >= RearS && S <= FrontS);
            if (bHit)
            {
                bCoveredThisScan[i] = true;
            }
        }
        (void)Total;
    }

    // Resolve the scan into edges. Edges are what a logic layer actually acts on,
    // and they are counted here rather than derived by the caller so that two
    // readers cannot disagree about whether one happened.
    void EndScan()
    {
        ++ScanCount;
        for (std::size_t i = 0; i < At.size(); ++i)
        {
            // THE FAULT IS APPLIED HERE, BEFORE THE EDGES, and that placement is
            // the whole of it: edges must come from what the sensor REPORTS, not
            // from what is physically true. A dead switch does not produce edges
            // nobody can see — it produces no edges, and every layer above
            // believes it.
            const bool bNow = Reported(i, bCoveredThisScan[i]);
            if (bNow && !State[i].bBlocked)
            {
                ++State[i].Rising;
            }
            else if (!bNow && State[i].bBlocked)
            {
                ++State[i].Falling;
            }
            State[i].bBlocked = bNow;
        }
    }

    // ===================== FAULT INJECTION =====================
    //
    // A sensor is the PLC's ONLY view of where the trains are, so a lying sensor
    // is the most consequential single failure in the system — and it is exactly
    // what the second detection method was built to catch. Until this existed,
    // that cross-check had only ever been proven by MUTATING the counter's own
    // rule, which shows the assertion bites but not that a real sensor failure is
    // caught. Different claims.
    //
    // Deliberately at the device rather than in a central fault manager: every
    // physical failure in a control system shows up as an input that lies, an
    // output that does not take effect, or feedback that disagrees. Each layer
    // already owns one of those, so each layer owns its own faults.
    enum class ESensorFault
    {
        None,
        Dead,      // never reports blocked. Failed prox, cut wire, dirty face.
        StuckOn,   // always reports blocked. Shorted, or debris across the lens.
        Chatter,   // toggles regardless of reality. A loose connection.
    };

    void Fail(std::size_t Sensor, ESensorFault Mode)
    {
        if (Sensor >= Faults.size()) { Faults.resize(At.size(), ESensorFault::None); }
        if (Sensor < Faults.size())  { Faults[Sensor] = Mode; }
    }
    void ClearFaults() { Faults.assign(At.size(), ESensorFault::None); }
    ESensorFault FaultOn(std::size_t Sensor) const
    {
        return Sensor < Faults.size() ? Faults[Sensor] : ESensorFault::None;
    }

    // Scans between chatter toggles. DETERMINISTIC on purpose — a scenario that
    // reproduces differently every run cannot be used to prove anything, and this
    // project has no random source anywhere for the same reason.
    int ChatterScans = 7;

    const FSensorReading& Read(std::size_t Sensor) const { return State[Sensor]; }
    bool IsBlocked(std::size_t Sensor) const { return State[Sensor].bBlocked; }

    // THE QUESTION A BLOCK SYSTEM ACTUALLY ASKS. Not "where is the train" but
    // "has a whole train gone past here": one rise and one matching fall. While a
    // train is over the sensor the rise has happened and the fall has not, which
    // is precisely the state that must NOT count as clear.
    bool HasPassedCompletely(std::size_t Sensor) const
    {
        return State[Sensor].Falling > 0 && State[Sensor].Falling == State[Sensor].Rising;
    }

private:
    // What the switch SAYS, given what is physically over it.
    bool Reported(std::size_t i, bool bTruth) const
    {
        if (i >= Faults.size()) { return bTruth; }
        switch (Faults[i])
        {
        case ESensorFault::Dead:    return false;
        case ESensorFault::StuckOn: return true;
        case ESensorFault::Chatter:
            return ChatterScans > 0 && ((ScanCount / ChatterScans) % 2) == 0;
        default:                    return bTruth;
        }
    }

    std::vector<double> At;
    std::vector<FSensorReading> State;
    std::vector<bool> bCoveredThisScan;
    std::vector<ESensorFault> Faults;
    long long ScanCount = 0;
};

// Block occupancy DERIVED FROM SENSORS ALONE, with no idea where any train is.
//
// This is the piece that proves the sensor layer is sufficient rather than
// decorative: given one sensor per block boundary, in travel order,
//
//     trains in block i  =  (times sensor i has been ENTERED)
//                         - (times sensor i+1 has been FULLY CLEARED)
//
// and that is exactly the nose-to-tail range the occupancy layer computes from
// perfect knowledge. Walk it through:
//
//   nose trips sensor i          -> +1: the train is in block i
//   train clears sensor i        ->  0 change: it is still in block i
//   nose trips sensor i+1        -> block i+1 gains one; block i KEEPS its one,
//                                   which is the straddle, held by both
//   tail clears sensor i+1       -> -1: block i is genuinely empty
//
// Rising on the way in and FALLING on the way out is the whole trick, and it is
// why the sensor counts both. A design that used one edge for both ends releases
// a block while a train is still lying across its boundary.
//
// This is a train counter, which is what railways have used for a century, and it
// carries the property that matters: a block reading TWO is a collision, detected
// without anything ever having known a position.
// IT ACCUMULATES, AND IT MUST BE SEEDED. A first version computed occupancy from
// LIFETIME edge totals — rising(i) - falling(i+1) — which is right in the middle
// of a run and wrong at the start of one: a train beginning inside block 0 pushes
// block 3 to MINUS ONE the moment its tail leaves the sensor at 0.0, because it
// gets counted out of a block it was never counted into.
//
// That is not a bug peculiar to this; it is the reason a real ride is SWEPT and
// its counters zeroed with every train at a known place before it opens. So the
// counter holds its own totals, takes a Seed for each train the operator has
// confirmed, and moves them on edges from then on.
class FBlockCounter
{
public:
    // Sensor i must sit at the start of block i, in travel order, and the ring
    // wraps — block N-1 is bounded by sensor N-1 and sensor 0.
    explicit FBlockCounter(const FTrackSensors& InSensors)
        : Sensors(InSensors)
        , Count(InSensors.Num(), 0)
        , SeenRising(InSensors.Num(), 0)
        , SeenFalling(InSensors.Num(), 0)
    {
        // Start from wherever the sensors already are, so seeding is not thrown
        // off by whatever happened before the counter existed.
        for (std::size_t i = 0; i < InSensors.Num(); ++i)
        {
            SeenRising[i] = InSensors.Read(i).Rising;
            SeenFalling[i] = InSensors.Read(i).Falling;
        }
    }

    std::size_t NumBlocks() const { return Count.size(); }

    // "There is a train in this block, and I know it." The operator's sweep.
    void Seed(std::size_t Block)
    {
        if (Block < Count.size())
        {
            ++Count[Block];
        }
    }

    // Read the sensors and move the totals. Once per scan, after the sensors have
    // been updated — the same once-per-frame contract the overlap ageing has, and
    // for the same reason: these are edges, and an edge read twice is two trains.
    void Scan()
    {
        const std::size_t N = Count.size();
        for (std::size_t i = 0; i < N; ++i)
        {
            const FSensorReading& R = Sensors.Read(i);
            const int NewRising = R.Rising - SeenRising[i];
            const int NewFalling = R.Falling - SeenFalling[i];
            SeenRising[i] = R.Rising;
            SeenFalling[i] = R.Falling;

            // A nose over sensor i has entered block i.
            Count[i] += NewRising;
            // A tail off sensor i has left the block BEFORE it.
            Count[(i + N - 1) % N] -= NewFalling;
        }
    }

    int TrainsIn(std::size_t Block) const { return Count[Block]; }
    bool IsOccupied(std::size_t Block) const { return Count[Block] > 0; }

    // More than one train counted into a block nothing has counted out of. A real
    // controller treats this as an E-stop condition and so should anything reading
    // it: the interlocking has already failed.
    bool IsOverOccupied(std::size_t Block) const { return Count[Block] > 1; }

    // Below zero is not a collision, it is a LIE — the counter has been told a
    // train left somewhere it was never told one arrived. Seeding is wrong, or a
    // trip was missed. Worth surfacing separately, because the fix is different.
    bool IsInconsistent() const
    {
        for (int C : Count)
        {
            if (C < 0) { return true; }
        }
        return false;
    }

private:
    const FTrackSensors& Sensors;
    std::vector<int> Count;
    std::vector<int> SeenRising;
    std::vector<int> SeenFalling;
};

// ===================== HOW FAST, FROM SWITCHES ALONE =====================
//
// A pair of switches a surveyed distance apart, and a clock. Speed is that
// distance over the time between their rising edges — no position, no train
// identity, nothing this layer is not entitled to. It is how a real installation
// measures a train's speed, and it is the shape FTrackSensors already had.
//
// WHY IT EXISTS, MEASURED RATHER THAN ASSUMED. With the outer brake dead and two
// trains on the circuit, the arriving train is not stopped, rolls into the next
// block — which was EMPTY — and circulates for four minutes still doing 30.5 m/s
// past a station it should be parked in, without one signalling violation. The
// interlocking answers "is the block ahead free". It was.
//
// So block signalling protects trains FROM EACH OTHER and was never a check on
// whether a device works. The question nothing here asked is the one this
// answers: IS THIS TRAIN GOING TOO FAST FOR WHAT IS IN FRONT OF IT.
//
// THE SEPARATION IS SURVEYED, NOT DERIVED. Sensor positions exist in this
// program and reading them would be simpler and could never drift — which is
// exactly why it is not done. A real trap is commissioned by measuring the gap
// once, and a mis-surveyed one is a real failure that this should be able to
// express. Same idiom as the stop mark consuming train length at placement:
// consume the position once, at survey, and never again.
//
// IT REPORTS A SPEED, IT DOES NOT DECIDE ANYTHING. A sensor says what it sees;
// what a ride does about a train arriving too fast is the PLC's job, which is the
// same rule the drives' fault detection runs on.
struct FSpeedTrapSpec
{
    std::size_t First = 0;      // the switch a train crosses first, in travel order
    std::size_t Second = 0;
    double SeparationM = 0.0;   // SURVEYED at commissioning, not read per scan

    // How long a trap stays armed after its first switch trips. A train that
    // stops between the two would otherwise leave it armed for ever, and the next
    // train through would be measured against a clock started minutes ago.
    // Generous — 20 s at 240 Hz — because the slowest thing that legitimately
    // crosses a trap is a train being trucked at a crawl.
    int ArmedScansLimit = 4800;
};

// What one trap knows.
struct FSpeedTrapReading
{
    double SpeedMs = 0.0;      // last measurement, 0 until there is one
    int OverScans = 0;         // how many scans it was measured over
    bool bArmed = false;       // first switch tripped, waiting for the second
    bool bValid = false;       // a measurement has been taken and is not stale
    long long AtScan = -1;     // when

    // Counted, not acted on. A train that trips the SECOND switch without the
    // first is going backwards, or the first switch missed it. Either is worth
    // knowing and neither is a speed.
    int Reversed = 0;
    int TimedOut = 0;
};

// Speed measured from switch edges. One instance owns every trap on the ride, so
// it scans once, in the same place FBlockCounter does.
class FSpeedTraps
{
public:
    explicit FSpeedTraps(const FTrackSensors& InSensors)
        : Sensors(InSensors)
    {
    }

    // SURVEY. Reads the two positions ONCE, here, which is what commissioning a
    // trap is. Refuses a pair whose separation is zero or negative, because a
    // trap measuring across no distance divides by it.
    bool Survey(std::size_t First, std::size_t Second, int ArmedScansLimit = 4800)
    {
        if (First >= Sensors.Num() || Second >= Sensors.Num() || First == Second)
        {
            return false;
        }
        const double Sep = Sensors.PositionOf(Second) - Sensors.PositionOf(First);
        if (!(Sep > 0.0))
        {
            return false;
        }
        return Add({First, Second, Sep, ArmedScansLimit});
    }

    bool Add(const FSpeedTrapSpec& In)
    {
        if (In.First >= Sensors.Num() || In.Second >= Sensors.Num()
            || In.First == In.Second || !(In.SeparationM > 0.0))
        {
            return false;
        }
        Spec.push_back(In);
        State.push_back(FSpeedTrapReading());
        SeenFirst.push_back(Sensors.Read(In.First).Rising);
        SeenSecond.push_back(Sensors.Read(In.Second).Rising);
        ArmedAt.push_back(-1);
        return true;
    }

    std::size_t Num() const { return Spec.size(); }
    const FSpeedTrapReading& Read(std::size_t Trap) const { return State[Trap]; }
    const FSpeedTrapSpec& SpecOf(std::size_t Trap) const { return Spec[Trap]; }

    // Once per scan, after the sensors are updated. Edges, so a scan read twice
    // is two trains — the same contract the counter and the overlap ageing have.
    void Scan(double DeltaSeconds)
    {
        ++ScanCount;
        for (std::size_t t = 0; t < Spec.size(); ++t)
        {
            const FSpeedTrapSpec& S = Spec[t];
            FSpeedTrapReading& R = State[t];

            const int FirstNow = Sensors.Read(S.First).Rising;
            const int SecondNow = Sensors.Read(S.Second).Rising;
            const bool bFirstTripped = FirstNow > SeenFirst[t];
            const bool bSecondTripped = SecondNow > SeenSecond[t];
            SeenFirst[t] = FirstNow;
            SeenSecond[t] = SecondNow;

            if (bSecondTripped && R.bArmed)
            {
                const int Scans = static_cast<int>(ScanCount - ArmedAt[t]);
                const double Seconds = static_cast<double>(Scans) * DeltaSeconds;
                if (Seconds > 0.0)
                {
                    R.SpeedMs = S.SeparationM / Seconds;
                    R.OverScans = Scans;
                    R.bValid = true;
                    R.AtScan = ScanCount;
                }
                R.bArmed = false;
                ArmedAt[t] = -1;
            }
            else if (bSecondTripped)
            {
                // Second without first: backwards, or the first switch missed it.
                // Counted, never guessed at.
                ++R.Reversed;
            }

            if (bFirstTripped)
            {
                R.bArmed = true;
                ArmedAt[t] = ScanCount;
            }
            else if (R.bArmed && S.ArmedScansLimit > 0
                     && ScanCount - ArmedAt[t] > S.ArmedScansLimit)
            {
                // A train stopped between the switches. Disarm rather than carry a
                // clock started minutes ago into the next train's measurement.
                R.bArmed = false;
                ArmedAt[t] = -1;
                ++R.TimedOut;
            }
        }
    }

    long long ScansTaken() const { return ScanCount; }

private:
    const FTrackSensors& Sensors;
    std::vector<FSpeedTrapSpec> Spec;
    std::vector<FSpeedTrapReading> State;
    std::vector<int> SeenFirst;
    std::vector<int> SeenSecond;
    std::vector<long long> ArmedAt;
    long long ScanCount = 0;
};

// ===================== A BOUNDARY THAT KNOWS WHICH WAY =====================
//
// `FBlockCounter` is right forwards and wrong backwards, and it is wrong for a
// reason no rule can patch: a rising edge means "metal arrived over me" and
// nothing more. Reading it as "a nose entered the block ahead" is true only
// while trains go one way. `test_tracksensors.cpp` measures exactly what that
// costs — a boundary crossed in reverse leaves one block at -1 and the other at
// 2, where the truth is 1 and 0.
//
// THE FIX IS A SECOND HEAD, NOT A CLEVERER RULE, and that is how real axle
// counters do it: two heads a surveyed gap apart, and the ORDER of their edges
// says which way the metal went. Same shape as FSpeedTraps — two switches and a
// distance read once at commissioning — because it is the same idea used for a
// different question.
//
// THE RULE, AND IT IS SYMMETRIC:
//
//   rise in order (First, Second)  -> something entered the block AFTER  the boundary
//   rise in order (Second, First)  -> something entered the block BEFORE the boundary
//   fall in order (First, Second)  -> something fully left  the block BEFORE the boundary
//   fall in order (Second, First)  -> something fully left  the block AFTER  the boundary
//
// Forwards that is +1 after and -1 before; in reverse it is +1 before and -1
// after. Neither direction is a special case, which is the property to keep: a
// train that rolls back and comes forward again returns to exactly the state it
// started in, because every event has an exact opposite.
//
// LATENCY IS ONE GAP, and it is real rather than hidden. The block is credited
// when the SECOND head rises, because that is the first moment direction is
// known — so occupancy lags the true crossing by however long the train takes to
// cover the gap. On real hardware the heads are centimetres apart and it does not
// matter; author them metres apart and it will.
struct FBoundaryHeads
{
    std::size_t First = 0;    // the head a forward-travelling train reaches first
    std::size_t Second = 0;
};

class FDirectionalCounter
{
public:
    // One entry per block, in travel order. Boundary i sits at the start of
    // block i, so its "after" is block i and its "before" is block i-1 — the
    // same convention FBlockCounter uses, so the two are directly comparable.
    FDirectionalCounter(const FTrackSensors& InSensors,
                        const std::vector<FBoundaryHeads>& InBoundaries)
        : Sensors(InSensors)
        , Boundaries(InBoundaries)
        , Count(InBoundaries.size(), 0)
        , Pending(InBoundaries.size())
    {
        for (std::size_t i = 0; i < Boundaries.size(); ++i)
        {
            Pending[i].SeenFirstRising = Read(Boundaries[i].First).Rising;
            Pending[i].SeenSecondRising = Read(Boundaries[i].Second).Rising;
            Pending[i].SeenFirstFalling = Read(Boundaries[i].First).Falling;
            Pending[i].SeenSecondFalling = Read(Boundaries[i].Second).Falling;
        }
    }

    std::size_t NumBlocks() const { return Count.size(); }
    void Seed(std::size_t Block) { if (Block < Count.size()) { ++Count[Block]; } }

    void Scan()
    {
        const std::size_t N = Count.size();
        for (std::size_t i = 0; i < N; ++i)
        {
            FPending& P = Pending[i];
            const FSensorReading& A = Read(Boundaries[i].First);
            const FSensorReading& B = Read(Boundaries[i].Second);

            const bool bARose = A.Rising > P.SeenFirstRising;
            const bool bBRose = B.Rising > P.SeenSecondRising;
            const bool bAFell = A.Falling > P.SeenFirstFalling;
            const bool bBFell = B.Falling > P.SeenSecondFalling;
            P.SeenFirstRising = A.Rising;
            P.SeenSecondRising = B.Rising;
            P.SeenFirstFalling = A.Falling;
            P.SeenSecondFalling = B.Falling;

            // BOTH IN ONE SCAN IS NOT A DIRECTION. The heads are a surveyed gap
            // apart, so a train crossing both between two scans is moving faster
            // than the gap per scan — and there is genuinely no information about
            // order left to read. Dropped rather than guessed, and the same
            // failure the single-scan sensor limit already documents.
            if (bARose && bBRose) { P.RisingFirst = ENone; }
            else if (bARose) { Apply(i, P.RisingFirst, EFirst, /*bRising*/ true); }
            else if (bBRose) { Apply(i, P.RisingFirst, ESecond, /*bRising*/ true); }

            if (bAFell && bBFell) { P.FallingFirst = ENone; }
            else if (bAFell) { Apply(i, P.FallingFirst, EFirst, /*bRising*/ false); }
            else if (bBFell) { Apply(i, P.FallingFirst, ESecond, /*bRising*/ false); }
        }
    }

    int TrainsIn(std::size_t Block) const { return Count[Block]; }
    bool IsOccupied(std::size_t Block) const { return Count[Block] > 0; }
    bool IsOverOccupied(std::size_t Block) const { return Count[Block] > 1; }
    bool IsInconsistent() const
    {
        for (int C : Count) { if (C < 0) { return true; } }
        return false;
    }

private:
    enum EWhich { ENone, EFirst, ESecond };

    struct FPending
    {
        int SeenFirstRising = 0;
        int SeenSecondRising = 0;
        int SeenFirstFalling = 0;
        int SeenSecondFalling = 0;
        EWhich RisingFirst = ENone;
        EWhich FallingFirst = ENone;
    };

    const FSensorReading& Read(std::size_t S) const { return Sensors.Read(S); }

    // One head changed. If it is the first of a pair, remember it; if it
    // completes one, the order says the direction and the pair is consumed.
    void Apply(std::size_t Boundary, EWhich& Latch, EWhich Now, bool bRising)
    {
        if (Latch == ENone) { Latch = Now; return; }
        if (Latch == Now)
        {
            // The same head twice with no partner. Something bounced, or the
            // other head is dead. Re-latch rather than invent a crossing:
            // a counter that guesses is worse than one that misses.
            return;
        }
        const bool bForward = (Latch == EFirst);
        Latch = ENone;

        const std::size_t N = Count.size();
        const std::size_t After = Boundary;
        const std::size_t Before = (Boundary + N - 1) % N;

        if (bRising)
        {
            ++Count[bForward ? After : Before];
        }
        else
        {
            --Count[bForward ? Before : After];
        }
    }

    const FTrackSensors& Sensors;
    std::vector<FBoundaryHeads> Boundaries;
    std::vector<int> Count;
    std::vector<FPending> Pending;
};

// How much track it takes to stop from this speed at this deceleration. The
// arithmetic the build-time check already does against every holding device; the
// only new thing a trap brings is a MEASURED speed instead of a predicted one.
inline double StoppingDistanceM(double SpeedMs, double DecelMs2)
{
    return DecelMs2 > 0.0 ? (SpeedMs * SpeedMs) / (2.0 * DecelMs2) : 1e30;
}

// THE QUANTISATION, AND IT IS NOT A DETAIL. A scan is a tick, so a trap can only
// resolve time to one of them: at 240 Hz a train doing 30 m/s covers 0.125 m per
// scan, and a 1 m trap therefore measures it in 8 ticks — ±12.5% before anything
// else goes wrong.
//
// So a measurement over few scans is not a measurement, and a safety decision
// taken on one is worse than none. Returned rather than enforced, because how
// much error a caller can live with depends on what it is about to do.
inline double SpeedResolutionFraction(const FSpeedTrapReading& R)
{
    return R.OverScans > 0 ? 1.0 / static_cast<double>(R.OverScans) : 1.0;
}
