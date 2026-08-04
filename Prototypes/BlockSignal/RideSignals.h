// TrackUnlimited: the mapping layer between a train's position along the track
// and the block-occupancy state machine.
//
// FBlockController knows about blocks and nothing else — no geometry, no train,
// no length, no position, no identity. This supplies all of that: which block an
// arc length falls in, the fact that a train has a nose and a tail and therefore
// occupies a RANGE of blocks, when that range changes, and one dispatch
// permissive keyed to where a train is being released TO.
//
// It consumes DOUBLES, not an FTrain. That is deliberate and load-bearing: this
// header includes BlockSignal.h and the standard library and nothing else, the
// assert suite drives it with bare numbers, and the UE actor is a caller like
// any other. Nothing here has to change when the physics does.
//
// Neither BlockSignal.h nor TrainPhysics.h is modified to support this.
//
// ============================ TRAIN IDENTITY ============================
//
// N trains share ONE FBlockController, and each carries an index. That index is
// the whole design: multi-train was never a widening of storage, it was three
// reads that had no notion of WHICH train they were about.
//
//   ENTRY. bHeldAlready asks "was I already in this block", and with one shared
//   tail/nose it silently meant "was the train that called Update LAST already
//   in it". Train B rolling into a block train A is parked in matched, so
//   OnTrainEnter — the only function in the system that can report a violation —
//   never ran. A collision was SUPPRESSED rather than missed. It now reads B's
//   own previous range, so the entry happens, the block is not clear, and the
//   violation is counted.
//
//   EXIT. The old loop released every block in "the" old range. Train A moving
//   on therefore walked out of train B's blocks on B's behalf, and a block read
//   CLEAR with a train parked in it. A block is now released only when NO other
//   train's current range covers it.
//
//   PERMISSIVE. CanDispatchInto skips blocks the asking train already holds,
//   because a train stopped short of the station must not deny its own dispatch.
//   With one shared range that skip fired for whichever train updated last, so
//   asked on behalf of B it skipped A's blocks as B's own and granted entry into
//   an OCCUPIED block. It now skips only the asker's.
//
// TICK IS ONCE PER FRAME, NOT ONCE PER TRAIN. Overlaps live on blocks, not on
// trains, so calling it per train ages a 5 s overlap in 5/N seconds. Nothing can
// detect that from in here — there is no frame counter and no clock — so it is a
// caller contract.
//
// WITHIN A FRAME, UPDATE ORDER IS OBSERVABLE, AND IT FAILS CLOSED. A train that
// has not been updated yet this frame still reads at its previous span, so a
// train entering the block it is about to leave is reported as a violation half
// a frame early. That is the safe direction, and at any block length worth
// having, two trains that close ARE the violation.
//
// See Docs/PHASE0_FINDINGS.md, "Two trains today means no interlocking at all",
// for what this replaced and how each failure was measured.
// ========================================================================

#pragma once

#include "BlockSignal.h"

#include <algorithm>
#include <cstddef>
#include <vector>

class FRideSignals
{
public:
    // Boundaries are arc lengths in metres. Block i spans [B[i], B[i+1]), the
    // last running to the end of the track.
    //
    // INDEX ORDER IS TRAVEL ORDER. FBlockController::CanDispatch scans ascending
    // indices modulo the count and cannot detect the absence of that, so it is
    // the caller's job to number blocks along the direction the train runs.
    //
    // Well-formed input is ascending, unique, and starts at 0.0. Malformed input
    // is repaired rather than rejected, because a constructor has to produce a
    // usable object and the alternative — an empty block list — indexes out of
    // range on the first Update. Call IsWellFormed() FIRST if you want to report
    // instead of repair; that is the split the rest of the project uses.
    //
    // InNumTrains is clamped to at least one: zero trains would make every
    // Update and every permissive fail closed, which is a ride that never starts
    // with nothing anywhere saying why — the same failure the lookahead clamp
    // below exists to prevent.
    FRideSignals(std::vector<double> InBoundaryS, double InBufferSeconds,
                 std::size_t InLookahead, std::size_t InNumTrains = 1)
        : Boundary(Repaired(std::move(InBoundaryS)))
        , Blocks(std::vector<FBlockConfig>(Boundary.size(), FBlockConfig{InBufferSeconds}))
        , Look(InLookahead)
        , Held(InNumTrains < 1 ? 1 : InNumTrains)
    {
        // Clamped, not trusted. FBlockController fails closed at both ends of its
        // range, so an out-of-range lookahead would deny every dispatch for the
        // life of the ride with no other symptom — a ride that simply never
        // starts, and nothing anywhere saying why.
        if (Look < 1)
        {
            Look = 1;
        }
        if (Boundary.size() >= 2 && Look > Boundary.size() - 1)
        {
            Look = Boundary.size() - 1;
        }
    }

    // Ascending, unique, starting at 0.0, at least one entry. Free to call before
    // constructing, so a caller can surface bad input rather than have it quietly
    // repaired underneath them.
    static bool IsWellFormed(const std::vector<double>& InBoundaryS)
    {
        if (InBoundaryS.empty() || InBoundaryS[0] != 0.0)
        {
            return false;
        }
        for (std::size_t i = 1; i < InBoundaryS.size(); ++i)
        {
            if (!(InBoundaryS[i] > InBoundaryS[i - 1]))
            {
                return false;
            }
        }
        return true;
    }

    std::size_t NumBlocks() const { return Blocks.NumBlocks(); }
    std::size_t NumTrains() const { return Held.size(); }
    std::size_t Lookahead() const { return Look; }
    const std::vector<double>& Boundaries() const { return Boundary; }

    // Which block an arc length falls in. Inclusive at the boundary that OPENS
    // the block. Cannot underflow: Boundary[0] is 0.0 after repair, and the first
    // branch is the only way S can precede it.
    std::size_t BlockAt(double S) const
    {
        const auto It = std::upper_bound(Boundary.begin(), Boundary.end(), S);
        return It == Boundary.begin()
            ? 0
            : static_cast<std::size_t>(It - Boundary.begin()) - 1;
    }

    // Ages the overlaps. Same dt as FTrain::Step, ONCE PER FRAME — not per block
    // and NOT PER TRAIN. Overlaps belong to blocks, so N calls a frame expire a
    // 5 s overlap in 5/N seconds and nothing in here can tell.
    //
    // Deliberately does NOT re-guard the timestep: the single !(dt > 0.0) test in
    // FBlockController::Tick already rejects zero, negative and NaN, and a wrapper
    // that clamped or substituted a wall clock would defeat it.
    void Tick(double DeltaSeconds) { Blocks.Tick(DeltaSeconds); }

    // One train's span, after its FTrain::Step. Pass GetRearS() and GetFrontS().
    //
    // Returns false if a block was entered that was not CLEAR. That boolean is
    // the ONLY record a signalling violation leaves anywhere in this system
    // besides the counter, so a caller that drops it has silently disabled the
    // one thing the interlocking exists to tell it. With two trains it is also
    // the collision report, so dropping it means running a circuit with no
    // interlocking at all and no symptom.
    //
    // An out-of-range train index denies rather than growing the list: a train
    // this instance was not sized for has no occupancy anywhere, so admitting it
    // would let it run the circuit invisibly.
    bool Update(std::size_t Train, double RearS, double FrontS)
    {
        if (Train >= Held.size())
        {
            return false;
        }
        FHeldRange& Me = Held[Train];
        if (FrontS < RearS)
        {
            std::swap(RearS, FrontS);
        }
        const std::size_t NewTail = BlockAt(RearS);
        const std::size_t NewNose = BlockAt(FrontS);
        bool bOk = true;

        // One range diff covers every case that would otherwise be special: a
        // straddle (the train holds BOTH blocks, the only thing FBlockController
        // can represent), a rollback (symmetric, so no direction logic at all),
        // and the lap-end teleport (the old range exits and arms its overlaps,
        // the new range enters).
        //
        // Enters run before exits. Measured honestly: with ONE train and a
        // polling reader that order is NOT observable — the held-already test
        // below reads the OLD range, so the final state is identical either way,
        // and reversing the two loops does not fail a single assertion in
        // test_ridesignals.cpp. It is kept because it costs nothing and is the
        // safe order if anything ever watches state mid-update. Do not mistake it
        // for something the suite covers.
        //
        // An earlier version of this comment also claimed it was the safe order
        // "the day a second train exists". That was wrong, and measuring it said
        // so: with two trains the ordering is not what fails. IDENTITY is, and it
        // fails long before the ordering could matter — see the class comment.
        //
        // "Already held" is MY range, not the last-updated train's. That one word
        // is the difference between reporting a collision and suppressing it.
        for (std::size_t b = NewTail; b <= NewNose; ++b)
        {
            const bool bHeldAlready = Me.bOccupies && b >= Me.Tail && b <= Me.Nose;
            if (!bHeldAlready)
            {
                const bool bWasClear = Blocks.OnTrainEnter(b);
                bOk = bOk && bWasClear;
            }
        }
        if (Me.bOccupies)
        {
            for (std::size_t b = Me.Tail; b <= Me.Nose; ++b)
            {
                if (b >= NewTail && b <= NewNose)
                {
                    continue;
                }
                // Leaving a block does not mean the block is empty. Another
                // train standing in it keeps it OCCUPIED — without this test a
                // train releases blocks it was never inside on another train's
                // behalf, and one reads CLEAR with a train parked in it.
                if (HeldByAnother(Train, b))
                {
                    continue;
                }
                // Cannot report exit-without-entry: every block in the old
                // range was entered when it became the range. Asserted in the
                // test rather than branched on here.
                Blocks.OnTrainExit(b);
            }
        }

        // ponytail: a block crossed ENTIRELY within one dt is never entered and
        // never arms its overlap. At 60 Hz a 30 m/s train covers 0.5 m a frame,
        // so this needs a block shorter than that to matter. Step the range one
        // block at a time if block lengths ever approach a frame of travel.
        Me.Tail = NewTail;
        Me.Nose = NewNose;
        Me.bOccupies = true;
        if (!bOk)
        {
            ++ViolationCount;
        }
        return bOk;
    }

    // May a train be released INTO this block? Checks the destination itself and
    // the next Lookahead - 1 downstream, wrapping. The destination is the first
    // thing a dispatch needs clear, which is exactly what FBlockController's
    // FromBlock form cannot express.
    //
    // Blocks THE ASKING TRAIN already holds are SKIPPED. FBlockController has no
    // train identity and deliberately does not want one, so the exclusion belongs
    // here, in the one layer that knows where each train is. Without it a train
    // stopped short of the station denies its own dispatch through a block it is
    // standing in — and does so permanently, because nothing will ever clear it.
    // Keyed to the wrong train it does the opposite and grants entry into a block
    // another train is sitting in.
    bool CanDispatchInto(std::size_t Train, std::size_t Block) const
    {
        const std::size_t N = Blocks.NumBlocks();
        if (N == 0 || Look == 0 || Look > N || Block >= N || Train >= Held.size())
        {
            return false; // fail closed, the same direction as FBlockController
        }
        const FHeldRange& Me = Held[Train];
        for (std::size_t i = 0; i < Look; ++i)
        {
            const std::size_t B = (Block + i) % N;
            if (Me.bOccupies && B >= Me.Tail && B <= Me.Nose)
            {
                continue;
            }
            if (Blocks.GetState(B) != EBlockState::Clear)
            {
                return false;
            }
        }
        return true;
    }

    // May the train standing at arc length S be let go? Its destination is simply
    // the next block along, which is the two lines every caller was writing for
    // itself — and the modulo is the one place a circuit is assumed rather than a
    // point-to-point layout.
    bool CanRelease(std::size_t Train, double S) const
    {
        const std::size_t N = Blocks.NumBlocks();
        return N != 0 && CanDispatchInto(Train, (BlockAt(S) + 1) % N);
    }

    // Single-train forms, unchanged in meaning and kept because most of the
    // project genuinely has one train. They DENY on a multi-train instance rather
    // than aliasing to train 0 — a two-train caller that forgot its index would
    // otherwise drive both trains through one slot and see no interlocking at
    // all, which is precisely the failure this class was rewritten to end.
    bool Update(double RearS, double FrontS)
    {
        return Held.size() == 1 && Update(0, RearS, FrontS);
    }

    bool CanDispatchInto(std::size_t Block) const
    {
        return Held.size() == 1 && CanDispatchInto(std::size_t{0}, Block);
    }

    // Polling surface, for the block diagram and the signal lamps. There is no
    // change notification anywhere in FBlockController and none is added here.
    EBlockState GetState(std::size_t Block) const { return Blocks.GetState(Block); }
    double GetBufferRemaining(std::size_t Block) const { return Blocks.GetBufferRemaining(Block); }
    std::size_t Violations() const { return ViolationCount; }

    // Any train at all. What a block lamp wants: a block does not care which
    // train is standing in it.
    bool Occupies(std::size_t Block) const
    {
        for (std::size_t t = 0; t < Held.size(); ++t)
        {
            if (OccupiedBy(t, Block))
            {
                return true;
            }
        }
        return false;
    }

    // That specific train. What a dispatcher wants, and the distinction the whole
    // rewrite turns on.
    bool OccupiedBy(std::size_t Train, std::size_t Block) const
    {
        if (Train >= Held.size())
        {
            return false;
        }
        const FHeldRange& H = Held[Train];
        return H.bOccupies && Block >= H.Tail && Block <= H.Nose;
    }

private:
    bool HeldByAnother(std::size_t Train, std::size_t Block) const
    {
        for (std::size_t t = 0; t < Held.size(); ++t)
        {
            if (t != Train && OccupiedBy(t, Block))
            {
                return true;
            }
        }
        return false;
    }

    static std::vector<double> Repaired(std::vector<double> In)
    {
        std::sort(In.begin(), In.end());
        In.erase(std::unique(In.begin(), In.end()), In.end());
        if (In.empty() || In.front() != 0.0)
        {
            In.insert(In.begin(), 0.0);
        }
        return In;
    }

    // The blocks one train spans, nose to tail. bOccupies is false only before
    // that train's first Update — a train that has never been placed holds
    // nothing, which is why the flag is not merely Tail <= Nose.
    struct FHeldRange
    {
        std::size_t Tail = 0;
        std::size_t Nose = 0;
        bool bOccupies = false;
    };

    std::vector<double> Boundary;
    FBlockController Blocks;
    std::size_t Look = 1;
    std::vector<FHeldRange> Held;

    // Circuit-wide, not per train: a violation is an E-stop condition for the
    // whole ride, and the pair of trains involved is not a thing either of them
    // owns. Update's return says which train hit one.
    std::size_t ViolationCount = 0;
};
