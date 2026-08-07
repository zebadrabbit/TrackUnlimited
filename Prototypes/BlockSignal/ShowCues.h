// TrackUnlimited Tier 3: how TIME works inside a show.
// Plain C++17, no dependencies beyond ShowBus.
//
// `FShowBus` says a cue fired. `FDmxFixtures` says what it fires into. This is the
// part between them, and it is entirely about time.
//
// ===================== THREE TIME MODELS, AND REAL SHOWS USE ALL THREE =====================
//
// A - ABSOLUTE TIMECODE. A master clock, every cue carrying hh:mm:ss:ff, and the
//     frame rate as part of the contract. Deterministic, restartable anywhere,
//     and tolerant of one subsystem dying: a lighting desk that reboots mid-show
//     rejoins at the right frame because the timecode told it where everyone is.
//
// B - RELATIVE OFFSETS. Pre-wait, duration, post-wait — or follow and hang.
//
// C - TRIGGERED CHAINS. Event-condition-action with no time reference at all:
//     "when block 4 clears, play sequence 12." This is the glue between a
//     NONDETERMINISTIC ride and DETERMINISTIC shows, and it is why a coaster's
//     show system is not simply a long timeline.
//
// THEY COMPOSE THE WAY AN ATTRACTION DOES. The ride is model C at the top — a
// block state machine with no fixed timeline. Each scene is A or B, launched by a
// zone trigger and internally clock-driven for its 10-40 second life. Idle and
// ambient are C again, looping, pre-empted by scenes on priority.
//
// Get the composition right and the ride/show seam disappears. Get it wrong and
// everything either drifts, or has to become one long fixed timeline that a train
// running four seconds late destroys.
//
// ===================== THE DISTINCTION THAT BREAKS REAL SHOWS =====================
//
// Time measured from the previous cue's START is not time measured from its
// COMPLETION, and conflating the two is THE most common source of real
// show-programming bugs. Every console draws the distinction and every one names
// it differently — Eos calls them Follow and Hang, grandMA3 calls them Time and
// Follow, QLab calls them pre-wait and auto-continue.
//
// So both are modelled, separately, with the standard's ambiguity preserved rather
// than resolved into one concept. A cue list that only had one of them would work
// perfectly until a cue's duration changed.
//
// Units: seconds, and FRAMES where timecode is involved.

#pragma once

#include "ShowBus.h"

#include <cstdint>
#include <string>
#include <vector>

// ===================== A: TIMECODE =====================
//
// hh:mm:ss:ff, with the frame rate part of the contract. Two shows at different
// rates cannot share a clock, and the reason 29.97 drop-frame exists at all is
// that NTSC's rate is not 30 — so a "30 fps" show run against a 29.97 clock
// drifts about 3.6 seconds an hour, which over a park's operating day is minutes.
enum class ETimecodeRate
{
    Fps24,
    Fps25,
    Fps30,
    Fps2997Drop,   // NTSC drop-frame: skips frame numbers, never actual frames
};

inline double FramesPerSecond(ETimecodeRate R)
{
    switch (R)
    {
    case ETimecodeRate::Fps24:      return 24.0;
    case ETimecodeRate::Fps25:      return 25.0;
    case ETimecodeRate::Fps2997Drop: return 30000.0 / 1001.0;   // 29.97002997...
    default:                        return 30.0;
    }
}

struct FTimecode
{
    int Hours = 0, Minutes = 0, Seconds = 0, Frames = 0;

    // DROP FRAME DROPS NUMBERS, NOT FRAMES, which is the thing everybody gets
    // backwards. The count skips 00 and 01 at the start of every minute except
    // every tenth, so the LABEL keeps up with the wall clock while the tape runs
    // at 29.97. Converting without it makes an hour-long show land 3.6 s out.
    double ToSeconds(ETimecodeRate R) const
    {
        const double Fps = FramesPerSecond(R);
        if (R != ETimecodeRate::Fps2997Drop)
        {
            return Hours * 3600.0 + Minutes * 60.0 + Seconds + Frames / Fps;
        }
        const long long TotalMinutes = Hours * 60LL + Minutes;
        const long long DroppedNumbers = 2 * (TotalMinutes - TotalMinutes / 10);
        const long long FrameNumber =
            ((Hours * 3600LL + Minutes * 60LL + Seconds) * 30LL + Frames) - DroppedNumbers;
        return static_cast<double>(FrameNumber) / Fps;
    }
};

// ===================== B: FOLLOW vs HANG =====================
//
// The distinction that breaks real shows, kept as two separate things.
enum class ECueFollow
{
    // Manual: this cue does not advance by itself. Somebody or something else
    // triggers the next one.
    None,

    // FROM THE PREVIOUS CUE'S START. The next cue goes at a fixed offset from when
    // this one began, whatever this one is still doing. Eos calls it Follow.
    // Use it when the timing is musical: the offsets are the rhythm.
    FromStart,

    // FROM THE PREVIOUS CUE'S COMPLETION. The next cue waits for this one to
    // finish and THEN counts. Eos calls it Hang. Use it when the sequence is
    // mechanical: the next thing genuinely cannot begin until this one is done.
    //
    // Change one cue's duration and FromStart leaves everything after it where it
    // was, while FromCompletion shifts the entire rest of the sequence. That is
    // the whole difference and it is invisible until the day somebody edits a
    // duration.
    FromCompletion,
};

struct FShowCue
{
    std::string Name;
    int Fixture = 0;
    double Level = 1.0;

    // How long this cue's OUTPUT lasts. Not how long until the next one.
    double DurationSeconds = 0.0;

    ECueFollow Follow = ECueFollow::None;
    double FollowSeconds = 0.0;

    // Model A: if this is set, the cue is timecode-locked and the follow fields
    // are ignored. A locked cue fires when the clock reaches it, full stop —
    // which is what makes a timecoded show restartable anywhere.
    bool bTimecodeLocked = false;
    FTimecode At;

    bool bHazardous = false;
};

// One sequence: a scene, internally clock-driven for its 10-40 second life.
//
// PRIORITY is what makes idle and ambient work. A looping ambient sequence is
// pre-empted by a scene rather than mixed with it, because two things driving one
// fixture is a fight nobody wins — every real console resolves this with
// priority or LTP, and priority is the one a person can reason about.
struct FShowSequence
{
    std::string Name;
    std::vector<FShowCue> Cue;
    ETimecodeRate Rate = ETimecodeRate::Fps30;
    int Priority = 0;          // higher wins
    bool bLoop = false;        // idle and ambient
};

// What is firing right now.
struct FCueFiring
{
    std::size_t Sequence = 0;
    std::size_t Cue = 0;
    int Fixture = 0;
    double Level = 0.0;
    bool bStarting = false;    // this scan is its first
};

// The player: model C at the top, A or B inside each sequence.
class FShowPlayer
{
public:
    std::size_t Add(const FShowSequence& S)
    {
        Seq.push_back(S);
        State.push_back(FSeqState());
        return Seq.size() - 1;
    }

    std::size_t Num() const { return Seq.size(); }
    const FShowSequence& At(std::size_t i) const { return Seq[i]; }

    // MODEL C: an event starts a sequence. No time reference — the ride decides
    // when, and the ride has no timeline.
    //
    // Restarting a sequence already running RESTARTS it rather than stacking a
    // second copy. Two trains through one trigger inside a scene's life is a real
    // and ordinary thing, and two copies of a scene fighting over one fixture is
    // not what anybody meant.
    void Trigger(std::size_t Index)
    {
        if (Index >= Seq.size()) { return; }
        FSeqState& S = State[Index];
        S.bRunning = true;
        S.Clock = 0.0;
        S.Cursor = 0;
        S.CueStarted = 0.0;
        S.bCueOpen = false;
        ++S.Runs;
    }

    void Stop(std::size_t Index)
    {
        if (Index < Seq.size()) { State[Index].bRunning = false; }
    }
    bool IsRunning(std::size_t Index) const { return State[Index].bRunning; }
    std::size_t TimesRun(std::size_t Index) const { return State[Index].Runs; }

    // One scan. Returns everything firing, with the highest-priority sequence's
    // claim on a fixture winning.
    std::vector<FCueFiring> Scan(double DeltaSeconds)
    {
        std::vector<FCueFiring> Out;
        for (std::size_t i = 0; i < Seq.size(); ++i)
        {
            AdvanceOne(i, DeltaSeconds, Out);
        }
        return ResolveByPriority(Out);
    }

private:
    struct FSeqState
    {
        bool bRunning = false;
        double Clock = 0.0;        // seconds since this sequence started
        std::size_t Cursor = 0;    // next cue to consider
        double CueStarted = 0.0;   // when the current cue began, on this clock
        bool bCueOpen = false;     // a cue is producing output right now
        std::size_t Runs = 0;
    };

    void AdvanceOne(std::size_t i, double Dt, std::vector<FCueFiring>& Out)
    {
        const FShowSequence& Q = Seq[i];
        FSeqState& S = State[i];
        if (!S.bRunning || Q.Cue.empty()) { return; }
        S.Clock += Dt;

        // Fire everything that has come due this scan. A loop rather than a single
        // step, because two cues can legitimately fall inside one 4 ms scan and
        // dropping one would make the show frame-rate dependent.
        for (;;)
        {
            if (S.Cursor >= Q.Cue.size())
            {
                if (Q.bLoop)
                {
                    // IDLE AND AMBIENT LOOP, and the clock rewinds rather than
                    // resetting to zero, so a loop does not drift by up to one
                    // scan every pass. Over an operating day that is minutes.
                    S.Clock -= LengthOf(Q);
                    S.Cursor = 0;
                    S.CueStarted = 0.0;
                    continue;
                }
                S.bRunning = false;
                S.bCueOpen = false;
                return;
            }

            const FShowCue& C = Q.Cue[S.Cursor];
            const double Due = DueTime(Q, S, S.Cursor);
            if (S.Clock < Due) { break; }

            S.CueStarted = Due;
            S.bCueOpen = true;
            FCueFiring F;
            F.Sequence = i;
            F.Cue = S.Cursor;
            F.Fixture = C.Fixture;
            F.Level = C.Level;
            F.bStarting = true;
            Out.push_back(F);
            ++S.Cursor;
        }

        // A cue that is still within its duration keeps producing. DMX has no
        // fades, so "still producing" means the value is being re-sent every
        // frame — which is exactly what a console does.
        if (S.bCueOpen && S.Cursor > 0)
        {
            const FShowCue& Held = Q.Cue[S.Cursor - 1];
            if (S.Clock < S.CueStarted + Held.DurationSeconds)
            {
                FCueFiring F;
                F.Sequence = i;
                F.Cue = S.Cursor - 1;
                F.Fixture = Held.Fixture;
                F.Level = Held.Level;
                F.bStarting = false;
                Out.push_back(F);
            }
            else
            {
                S.bCueOpen = false;
            }
        }
    }

    // WHERE THE THREE MODELS MEET. A timecode-locked cue answers from the clock;
    // everything else answers from the cue before it, and FROM WHICH PART of it is
    // the distinction this file exists for.
    static double DueTime(const FShowSequence& Q, const FSeqState& S, std::size_t Index)
    {
        const FShowCue& C = Q.Cue[Index];
        if (C.bTimecodeLocked)
        {
            return C.At.ToSeconds(Q.Rate);
        }
        if (Index == 0) { return 0.0; }

        const FShowCue& Prev = Q.Cue[Index - 1];
        const double PrevDue = DueTimeCached(Q, S, Index - 1);
        switch (Prev.Follow)
        {
        case ECueFollow::FromStart:
            // Eos's Follow. Offset from when the previous cue BEGAN, whatever it
            // is still doing. Editing a duration moves nothing after it.
            return PrevDue + Prev.FollowSeconds;
        case ECueFollow::FromCompletion:
            // Eos's Hang. Wait for it to finish, THEN count. Editing a duration
            // shifts everything after it.
            return PrevDue + Prev.DurationSeconds + Prev.FollowSeconds;
        default:
            // Manual: never due by itself. A sequence that reaches one simply
            // stops advancing, which is what a "wait for GO" cue is.
            return 1e30;
        }
    }

    // Recursive by way of a small cache-free walk. A sequence is tens of cues, so
    // this is a handful of additions — and expressing it recursively is what keeps
    // the two follow rules in ONE place rather than in an accumulating loop that
    // would have to reimplement them.
    static double DueTimeCached(const FShowSequence& Q, const FSeqState& S, std::size_t Index)
    {
        return DueTime(Q, S, Index);
    }

    // A LOOP'S PERIOD IS WHEN CUE 0 COMES ROUND, WHICH THE LAST CUE'S FOLLOW SAYS.
    //
    // The first version used the last cue's DURATION instead, which is a different
    // number entirely and made a one-second loop fire every 100 ms. Duration is
    // how long a cue keeps producing output; follow is when the sequence advances,
    // and those are two separate fields for exactly this reason.
    //
    // So the period is the same arithmetic DueTime does, applied one step past the
    // end — which keeps the follow-versus-hang rule in one place rather than
    // reimplementing it here.
    static double LengthOf(const FShowSequence& Q)
    {
        FSeqState Tmp;
        if (Q.Cue.empty()) { return 0.0; }
        const std::size_t Last = Q.Cue.size() - 1;
        const double LastDue = DueTime(Q, Tmp, Last);
        const FShowCue& C = Q.Cue[Last];
        switch (C.Follow)
        {
        case ECueFollow::FromStart:
            return LastDue + C.FollowSeconds;
        case ECueFollow::FromCompletion:
            return LastDue + C.DurationSeconds + C.FollowSeconds;
        default:
            // A LOOP WHOSE LAST CUE IS MANUAL NEVER COMES ROUND, because nothing
            // says when to advance. Falling back to the end of its output is the
            // least surprising answer and it is what a console does with a cue
            // list that runs off the end — but it is a show nobody meant to write.
            return LastDue + C.DurationSeconds;
        }
    }

    // PRIORITY, NOT MIXING. Two sequences driving one fixture is a fight, and
    // every real console resolves it with priority or latest-takes-precedence.
    // Priority is the one a person can reason about at 2 a.m. with a show running.
    std::vector<FCueFiring> ResolveByPriority(const std::vector<FCueFiring>& In) const
    {
        std::vector<FCueFiring> Out;
        for (const FCueFiring& F : In)
        {
            bool bBeaten = false;
            for (const FCueFiring& Other : In)
            {
                if (Other.Fixture != F.Fixture) { continue; }
                const int Mine = Seq[F.Sequence].Priority;
                const int Theirs = Seq[Other.Sequence].Priority;
                if (Theirs > Mine || (Theirs == Mine && Other.Sequence < F.Sequence
                                      && Other.Cue != F.Cue))
                {
                    bBeaten = true;
                    break;
                }
            }
            if (!bBeaten) { Out.push_back(F); }
        }
        return Out;
    }

    std::vector<FShowSequence> Seq;
    std::vector<FSeqState> State;
};

// ponytail: no crossfades between sequences, no cue-level fade curves, no
// tracking (where a cue inherits unchanged values from the one before). Tracking
// especially is what a real console does and is a genuinely large idea — it needs
// a full state model per fixture rather than a list of firings. Add it when
// something has enough fixtures for the difference to be visible.
