// TrackUnlimited Phase 0 prototype: block-occupancy + buffer state machine.
// Plain C++, no engine dependencies — designed to port into UE5 C++ later.
// Design reference: Docs/PROJECT_PLAN.md Section 5, "Signaling and ride control".
//
// Per-block state machine: CLEAR -> OCCUPIED -> BUFFER(x) -> CLEAR.
// "Buffer" is the real-railway "overlap": after a train physically exits a
// block, the block withholds CLEAR until a configurable safety margin elapses.

#pragma once

#include <cstddef>
#include <vector>

// ponytail: one train per block, by definition — there is no state for "two
// inside". Occupancy past a signalling violation is therefore not tracked, and
// none is needed: a violation is an E-stop condition in real ride control, not
// a recoverable state. Add an occupant count only if this ever has to keep
// simulating through a violation instead of halting on one.
enum class EBlockState
{
    Clear,
    Occupied,
    Buffer,
};

struct FBlockConfig
{
    // Seconds the block holds BUFFER after the train exits before reporting
    // CLEAR. Zero means the block clears immediately on exit.
    // ponytail: time-based overlap only; add distance-based overlap when the
    // physics prototype can supply train position/braking distance.
    double BufferSeconds = 0.0;
};

class FBlockController
{
public:
    explicit FBlockController(std::vector<FBlockConfig> InConfigs)
        : Configs(std::move(InConfigs))
        , States(Configs.size(), EBlockState::Clear)
        , BufferRemaining(Configs.size(), 0.0)
    {
    }

    std::size_t NumBlocks() const { return States.size(); }
    EBlockState GetState(std::size_t Block) const { return States[Block]; }

    // A train's nose has entered the block. Returns false if the block was
    // not CLEAR — a signaling violation (the upstream permissive logic should
    // have prevented this). Physical truth wins either way: the block is
    // marked OCCUPIED regardless, because a train really is inside it.
    bool OnTrainEnter(std::size_t Block)
    {
        const bool bWasClear = States[Block] == EBlockState::Clear;
        States[Block] = EBlockState::Occupied;
        BufferRemaining[Block] = 0.0;
        return bWasClear;
    }

    // The train's tail has fully exited the block. Enters BUFFER (or CLEAR
    // directly if the configured overlap is zero). Returns false if the block
    // was not OCCUPIED (exit without entry — a caller bug).
    bool OnTrainExit(std::size_t Block)
    {
        if (States[Block] != EBlockState::Occupied)
        {
            return false;
        }
        if (Configs[Block].BufferSeconds > 0.0)
        {
            States[Block] = EBlockState::Buffer;
            BufferRemaining[Block] = Configs[Block].BufferSeconds;
        }
        else
        {
            States[Block] = EBlockState::Clear;
        }
        return true;
    }

    // Advance buffer countdowns. Deterministic: driven by the caller's dt,
    // never wall-clock time.
    void Tick(double DeltaSeconds)
    {
        // Rejects zero, negative and NaN in one comparison. Negative dt would
        // grow a countdown past its configured ceiling; NaN would latch a block
        // in BUFFER permanently, and no later well-formed tick could free it.
        if (!(DeltaSeconds > 0.0))
        {
            return;
        }
        for (std::size_t i = 0; i < States.size(); ++i)
        {
            if (States[i] == EBlockState::Buffer)
            {
                BufferRemaining[i] -= DeltaSeconds;
                if (BufferRemaining[i] <= 0.0)
                {
                    BufferRemaining[i] = 0.0;
                    States[i] = EBlockState::Clear;
                }
            }
        }
    }

    // Dispatch permissive: may a train in `FromBlock` be released? True only
    // if the next `Lookahead` blocks downstream all report CLEAR. Blocks form
    // a circuit, so lookahead wraps past the end. Lookahead > 1 models the
    // braking-distance requirement for high-speed sections.
    bool CanDispatch(std::size_t FromBlock, std::size_t Lookahead = 1) const
    {
        const std::size_t N = States.size();
        // Deny both ends of the range. Looking ahead a full circuit or more
        // would include FromBlock itself (occupied by the very train asking).
        // Zero lookahead checks nothing at all, and dispatching always means
        // entering the next block, so it is never a meaningful query — without
        // this it returns an unconditional permissive even with every block
        // occupied, which is the wrong direction for a safety function to fail.
        if (Lookahead == 0 || Lookahead >= N)
        {
            return false;
        }
        for (std::size_t i = 1; i <= Lookahead; ++i)
        {
            if (States[(FromBlock + i) % N] != EBlockState::Clear)
            {
                return false;
            }
        }
        return true;
    }

    double GetBufferRemaining(std::size_t Block) const { return BufferRemaining[Block]; }

private:
    std::vector<FBlockConfig> Configs;
    std::vector<EBlockState> States;
    std::vector<double> BufferRemaining;
};
