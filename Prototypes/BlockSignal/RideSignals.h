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
// ============================ ONE TRAIN ONLY ============================
//
// This class tracks ONE train, and there is no way to make it track two that
// does not FAIL OPEN. Both obvious attempts were measured, and neither reports
// anything at all — no denial, no violation, no counter:
//
//   ONE INSTANCE PER TRAIN. Each owns a private FBlockController, so not one bit
//   of occupancy is shared. Train B's permissive is GRANTED with train A standing
//   in the destination block, B's Update returns true ("was clear"), the two are
//   independently confirmed co-resident, and Violations() reads 0 on both. That
//   is not a weak interlock; it is no interlock. Each train validates against a
//   private fiction of the circuit.
//
//   ONE SHARED INSTANCE, UPDATED BY BOTH. The single tail/nose/occupies triple
//   below silently re-reads as "wherever the train that called Update LAST is",
//   and all three of its consumers then fail open. Each train's exit loop walks
//   the OTHER train's old range and releases it, so a block reads CLEAR with a
//   train parked in it. CanDispatchInto skips "blocks this train holds" using the
//   same triple, so asked on behalf of B while the triple describes A it skips
//   A's blocks as the asker's own and grants entry into an OCCUPIED block. And
//   the collision entry is SUPPRESSED: bHeldAlready is true, so OnTrainEnter —
//   the only function in the system that can report a violation — never runs.
//   The "occupied by me" test becomes a test of call order, not of identity.
//
// So multi-train is not a widening of storage. Two of the reads below change
// MEANING, and one changes from correct to fail-open. Tick gains a contract it
// never needed: once per FRAME, not once per train, or a 5 s overlap expires in
// 2.5 s with two trains and nothing says so.
//
// Until that work is done, construct exactly one of these. See
// Docs/PHASE0_FINDINGS.md, "Two trains today means no interlocking at all".
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
    FRideSignals(std::vector<double> InBoundaryS, double InBufferSeconds,
                 std::size_t InLookahead)
        : Boundary(Repaired(std::move(InBoundaryS)))
        , Blocks(std::vector<FBlockConfig>(Boundary.size(), FBlockConfig{InBufferSeconds}))
        , Look(InLookahead)
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

    // Ages the overlaps. Same dt as FTrain::Step, once per frame, not per block.
    //
    // Deliberately does NOT re-guard the timestep: the single !(dt > 0.0) test in
    // FBlockController::Tick already rejects zero, negative and NaN, and a wrapper
    // that clamped or substituted a wall clock would defeat it.
    void Tick(double DeltaSeconds) { Blocks.Tick(DeltaSeconds); }

    // The train's span, after FTrain::Step. Pass GetRearS() and GetFrontS().
    //
    // Returns false if a block was entered that was not CLEAR. That boolean is
    // the ONLY record a signalling violation leaves anywhere in this system
    // besides the counter, so a caller that drops it has silently disabled the
    // one thing the interlocking exists to tell it.
    bool Update(double RearS, double FrontS)
    {
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
        for (std::size_t b = NewTail; b <= NewNose; ++b)
        {
            const bool bHeldAlready = bOccupies && b >= TailBlock && b <= NoseBlock;
            if (!bHeldAlready)
            {
                const bool bWasClear = Blocks.OnTrainEnter(b);
                bOk = bOk && bWasClear;
            }
        }
        if (bOccupies)
        {
            for (std::size_t b = TailBlock; b <= NoseBlock; ++b)
            {
                if (b < NewTail || b > NewNose)
                {
                    // Cannot report exit-without-entry: every block in the old
                    // range was entered when it became the range. Asserted in the
                    // test rather than branched on here.
                    Blocks.OnTrainExit(b);
                }
            }
        }

        // ponytail: a block crossed ENTIRELY within one dt is never entered and
        // never arms its overlap. At 60 Hz a 30 m/s train covers 0.5 m a frame,
        // so this needs a block shorter than that to matter. Step the range one
        // block at a time if block lengths ever approach a frame of travel.
        TailBlock = NewTail;
        NoseBlock = NewNose;
        bOccupies = true;
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
    // Blocks this train already holds are SKIPPED. FBlockController has no train
    // identity and deliberately does not want one, so the exclusion belongs here,
    // in the one layer that knows where the asking train is. Without it a train
    // stopped short of the station denies its own dispatch through a block it is
    // standing in — and does so permanently, because nothing will ever clear it.
    bool CanDispatchInto(std::size_t Block) const
    {
        const std::size_t N = Blocks.NumBlocks();
        if (N == 0 || Look == 0 || Look > N || Block >= N)
        {
            return false; // fail closed, the same direction as FBlockController
        }
        for (std::size_t i = 0; i < Look; ++i)
        {
            const std::size_t B = (Block + i) % N;
            if (bOccupies && B >= TailBlock && B <= NoseBlock)
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

    // Polling surface, for the block diagram and the signal lamps. There is no
    // change notification anywhere in FBlockController and none is added here.
    EBlockState GetState(std::size_t Block) const { return Blocks.GetState(Block); }
    double GetBufferRemaining(std::size_t Block) const { return Blocks.GetBufferRemaining(Block); }
    std::size_t Violations() const { return ViolationCount; }

    bool Occupies(std::size_t Block) const
    {
        return bOccupies && Block >= TailBlock && Block <= NoseBlock;
    }

private:
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

    std::vector<double> Boundary;
    FBlockController Blocks;
    std::size_t Look = 1;

    // One train. Widen these three to vectors and add a train index to Update and
    // CanDispatchInto the day a second FTrain exists; nothing else changes.
    std::size_t TailBlock = 0;
    std::size_t NoseBlock = 0;
    bool bOccupies = false;

    std::size_t ViolationCount = 0;
};
