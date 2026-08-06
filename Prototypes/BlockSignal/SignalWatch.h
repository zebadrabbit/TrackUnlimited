// TrackUnlimited: what changed, and when.
// Plain C++17, no engine dependency, same conventions as TrackDrives.h.
//
// A ride already logs the six things that go WRONG — an E-stop, a reset, an
// acknowledgement, a detection disagreement, a signalling violation, a drive
// fault. What it cannot answer is the ordinary question anybody asks after
// watching a ride: "did that lamp ever light?" Nothing routine is recorded, so a
// still frame is the whole of the evidence.
//
// This is the missing half. A real installation keeps an event historian, and it
// records STATE TRANSITIONS rather than state: a SCADA log that wrote every
// point every scan would be 30 lines a frame and unreadable within a second.
//
// SO THE RULE IS EDGE, NOT LEVEL — the same rule the sensors and the block
// counter already run on, for the same reason. Log when it changes; say nothing
// while it stays.
//
// This class is only the change detection. It deliberately holds no text and no
// clock: the caller owns the words, because the words differ per signal and
// formatting them every frame to discover they have not changed is the cost this
// exists to avoid. Channels are plain indices, so the common path is one integer
// compare and no allocation.

#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

class FSignalWatch
{
public:
    explicit FSignalWatch(std::size_t Channels = 0) { Reset(Channels); }

    void Reset(std::size_t Channels)
    {
        Last.assign(Channels, 0);
        bSeeded.assign(Channels, false);
    }

    std::size_t Num() const { return Last.size(); }

    // Grow on demand, so a caller does not have to count its channels up front —
    // a layout with more blocks simply observes more of them.
    void Ensure(std::size_t Channels)
    {
        if (Channels > Last.size())
        {
            Last.resize(Channels, 0);
            bSeeded.resize(Channels, false);
        }
    }

    // True exactly on the frame this channel's state differs from the last one
    // seen. The caller then formats and records; nothing else does.
    //
    // THE FIRST OBSERVATION OF A CHANNEL IS A SEED, NOT A CHANGE. Without that,
    // frame one reports every block, every platform, every drive and every lamp
    // as having just transitioned — thirty spurious events at t=0 burying the one
    // real thing that happens next. It is the same requirement FBlockCounter has
    // and for the same reason: a detector with no baseline cannot tell "it moved"
    // from "I have just started looking".
    bool Changed(std::size_t Channel, int State)
    {
        Ensure(Channel + 1);
        if (!bSeeded[Channel])
        {
            bSeeded[Channel] = true;
            Last[Channel] = State;
            return false;
        }
        if (Last[Channel] == State)
        {
            return false;
        }
        Last[Channel] = State;
        return true;
    }

    // What it was before the change, for a caller that wants to say "A -> B".
    // Valid only in the frame Changed() returned true, because the next call
    // overwrites it.
    int Previous() const { return PreviousState; }

    // As Changed(), but remembers what it moved FROM.
    bool ChangedFrom(std::size_t Channel, int State)
    {
        Ensure(Channel + 1);
        PreviousState = bSeeded[Channel] ? Last[Channel] : State;
        return Changed(Channel, State);
    }

    int Peek(std::size_t Channel) const
    {
        return Channel < Last.size() ? Last[Channel] : 0;
    }
    bool IsSeeded(std::size_t Channel) const
    {
        return Channel < bSeeded.size() && bSeeded[Channel];
    }

    // Forget a channel's baseline, so its next observation seeds again rather
    // than reporting a transition. For a rebuild: a layout that has just gained
    // or lost blocks has channels that no longer mean what they meant, and
    // carrying the old values across produces transitions that never happened.
    void Forget() { std::fill(bSeeded.begin(), bSeeded.end(), false); }

private:
    std::vector<int> Last;
    std::vector<bool> bSeeded;
    int PreviousState = 0;
};

// ponytail: channels are indices the caller allocates by hand, in blocks (e.g.
// blocks at 0, platforms at 100, drives at 200). A named registry would be
// tidier and is not worth it while there is one caller — but if a second one
// appears, give them separate FSignalWatch instances rather than sharing a range,
// because a collision here is silent and reads as a phantom transition.
