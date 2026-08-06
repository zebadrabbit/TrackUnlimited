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
