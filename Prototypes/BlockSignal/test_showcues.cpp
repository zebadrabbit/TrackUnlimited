// Asserts for ShowCues.h — how time works inside a show.
//
//   clang++ -std=c++17 -Wall -Wextra -O2 -o test_showcues test_showcues.cpp && ./test_showcues

#include "ShowCues.h"

#include <cassert>
#include <cmath>
#include <cstdio>

namespace
{

const double Dt = 1.0 / 240.0;

// Run a player for this long, recording when each fixture first fired.
std::vector<double> FirstFires(FShowPlayer& P, double Seconds, int NumFixtures)
{
    std::vector<double> First(static_cast<std::size_t>(NumFixtures), -1.0);
    double T = 0.0;
    for (int i = 0; i < static_cast<int>(Seconds / Dt); ++i)
    {
        T += Dt;
        for (const FCueFiring& F : P.Scan(Dt))
        {
            if (F.bStarting && F.Fixture < NumFixtures
                && First[static_cast<std::size_t>(F.Fixture)] < 0.0)
            {
                First[static_cast<std::size_t>(F.Fixture)] = T;
            }
        }
    }
    return First;
}

void TestFOLLOWAndHANGAreDIFFERENTAndThatIsThePoint()
{
    // THE DISTINCTION THAT BREAKS REAL SHOWS. Time from the previous cue's START
    // is not time from its COMPLETION, and conflating them is the most common
    // source of real show-programming bugs. Every console draws it and every one
    // names it differently — Eos Follow/Hang, grandMA3 Time/Follow, QLab
    // pre-wait/auto-continue.
    //
    // Cue 0 lasts 3 s. Cue 1 follows by 1 s. Where does cue 1 land?
    //
    //   FromStart      at 1 s  — it overlaps cue 0, which is still running
    //   FromCompletion at 4 s  — it waits for cue 0 to finish, THEN counts
    FShowSequence FromStart;
    FromStart.Name = "musical";
    FromStart.Cue.push_back({"a", 0, 1.0, 3.0, ECueFollow::FromStart, 1.0, false, {}, false});
    FromStart.Cue.push_back({"b", 1, 1.0, 1.0, ECueFollow::None, 0.0, false, {}, false});

    FShowSequence FromDone = FromStart;
    FromDone.Name = "mechanical";
    FromDone.Cue[0].Follow = ECueFollow::FromCompletion;

    FShowPlayer A;
    A.Trigger(A.Add(FromStart));
    const std::vector<double> Ta = FirstFires(A, 8.0, 2);

    FShowPlayer B;
    B.Trigger(B.Add(FromDone));
    const std::vector<double> Tb = FirstFires(B, 8.0, 2);

    assert(std::fabs(Ta[1] - 1.0) < 0.02);
    assert(std::fabs(Tb[1] - 4.0) < 0.02);
    std::printf("  follow lands cue 1 at %.2f s, hang at %.2f s, from identical offsets\n",
                Ta[1], Tb[1]);
}

void TestEditingADURATIONMovesOneAndNotTheOther()
{
    // AND HERE IS WHY IT MATTERS, which the timings above only hint at. The two
    // models are indistinguishable until somebody edits a duration — and then one
    // of them shifts the entire rest of the sequence and the other does not.
    //
    // That is the bug: a show that was right, a cue lengthened by a second, and
    // twelve cues downstream silently moved. Or did not move, when they should
    // have.
    for (int Which = 0; Which < 2; ++Which)
    {
        const ECueFollow Mode = Which == 0 ? ECueFollow::FromStart : ECueFollow::FromCompletion;

        FShowSequence Q;
        Q.Cue.push_back({"a", 0, 1.0, 3.0, Mode, 1.0, false, {}, false});
        Q.Cue.push_back({"b", 1, 1.0, 1.0, ECueFollow::FromStart, 2.0, false, {}, false});
        Q.Cue.push_back({"c", 2, 1.0, 1.0, ECueFollow::None, 0.0, false, {}, false});

        FShowPlayer P;
        P.Trigger(P.Add(Q));
        const std::vector<double> Before = FirstFires(P, 12.0, 3);

        Q.Cue[0].DurationSeconds = 5.0;      // the edit: cue 0 gets two seconds longer
        FShowPlayer R;
        R.Trigger(R.Add(Q));
        const std::vector<double> After = FirstFires(R, 12.0, 3);

        if (Mode == ECueFollow::FromStart)
        {
            // Nothing downstream moved. The offsets were the rhythm.
            assert(std::fabs(After[1] - Before[1]) < 0.02);
            assert(std::fabs(After[2] - Before[2]) < 0.02);
        }
        else
        {
            // Everything downstream moved by exactly the edit.
            assert(std::fabs((After[1] - Before[1]) - 2.0) < 0.02);
            assert(std::fabs((After[2] - Before[2]) - 2.0) < 0.02);
        }
    }
    std::printf("  lengthening a cue by 2 s moves nothing under follow and everything under hang\n");
}

void TestTIMECODEIsRestartableAnywhere()
{
    // MODEL A. A locked cue fires when the clock reaches it, full stop — which is
    // what makes a timecoded show restartable anywhere and tolerant of one
    // subsystem dying. A desk that reboots mid-show rejoins at the right frame.
    FShowSequence Q;
    Q.Rate = ETimecodeRate::Fps30;
    FShowCue C0{"tc", 0, 1.0, 0.5, ECueFollow::None, 0.0, true, {}, false};
    C0.At = {0, 0, 2, 15};                       // 2.5 s
    FShowCue C1{"tc", 1, 1.0, 0.5, ECueFollow::None, 0.0, true, {}, false};
    C1.At = {0, 0, 4, 0};                        // 4.0 s
    Q.Cue.push_back(C0);
    Q.Cue.push_back(C1);

    FShowPlayer P;
    P.Trigger(P.Add(Q));
    const std::vector<double> T = FirstFires(P, 6.0, 2);
    assert(std::fabs(T[0] - 2.5) < 0.02);
    assert(std::fabs(T[1] - 4.0) < 0.02);

    // The follow fields are IGNORED on a locked cue. A cue that is both timecoded
    // and following would have two answers to when it fires, and a show with two
    // answers has one bug.
    Q.Cue[0].Follow = ECueFollow::FromCompletion;
    Q.Cue[0].FollowSeconds = 99.0;
    FShowPlayer R;
    R.Trigger(R.Add(Q));
    const std::vector<double> T2 = FirstFires(R, 6.0, 2);
    assert(std::fabs(T2[1] - 4.0) < 0.02);
    std::printf("  timecode-locked cues fire from the clock and ignore follow entirely\n");
}

void TestDROPFRAMEDropsNUMBERSNotFrames()
{
    // The thing everybody gets backwards, and it is worth 3.6 seconds an hour —
    // which over a park's operating day is minutes of drift between the ride and
    // the soundtrack.
    //
    // 29.97 drop-frame skips frame NUMBERS 00 and 01 at the start of every minute
    // except every tenth, so the LABEL keeps up with the wall clock while the tape
    // runs at 29.97 fps. No frames are actually lost.
    const FTimecode OneHour{1, 0, 0, 0};

    // Non-drop 30: the label says an hour, the wall clock says an hour and 3.6 s.
    assert(std::fabs(OneHour.ToSeconds(ETimecodeRate::Fps30) - 3600.0) < 1e-9);

    // Drop-frame: the label says an hour and the wall clock agrees, to within a
    // couple of frames — which is the entire reason the format exists.
    const double Drop = OneHour.ToSeconds(ETimecodeRate::Fps2997Drop);
    assert(std::fabs(Drop - 3600.0) < 0.1);

    // And a naive 29.97 conversion WITHOUT the dropped numbers is the bug: it
    // lands 3.6 s late.
    const double Naive = 3600.0 * 30.0 / (30000.0 / 1001.0);
    assert(Naive - 3600.0 > 3.0);

    assert(std::fabs(FramesPerSecond(ETimecodeRate::Fps25) - 25.0) < 1e-12);
    std::printf("  drop-frame lands an hour at %.3f s; the naive conversion is %.1f s late\n",
                Drop, Naive - 3600.0);
}

void TestTRIGGEREDChainsAreTheGlueToANondeterministicRide()
{
    // MODEL C, and it is why a coaster's show system is not one long timeline.
    // The ride has no fixed schedule — a train is where the blocks let it be — so
    // an event starts a sequence and the sequence is internally clock-driven for
    // its ten-to-forty-second life.
    FShowSequence Scene;
    Scene.Name = "drop";
    Scene.Cue.push_back({"a", 0, 1.0, 1.0, ECueFollow::FromStart, 0.5, false, {}, false});
    Scene.Cue.push_back({"b", 1, 1.0, 1.0, ECueFollow::None, 0.0, false, {}, false});

    FShowPlayer P;
    const std::size_t S = P.Add(Scene);
    assert(!P.IsRunning(S));

    // Nothing happens until the ride says so.
    for (int i = 0; i < 240 * 5; ++i) { assert(P.Scan(Dt).empty()); }

    P.Trigger(S);
    assert(P.IsRunning(S));
    const std::vector<double> T = FirstFires(P, 4.0, 2);
    assert(T[0] > 0.0 && std::fabs(T[1] - T[0] - 0.5) < 0.02);

    // A SECOND TRAIN THROUGH THE SAME TRIGGER RESTARTS THE SCENE rather than
    // stacking a second copy. Two copies fighting over one fixture is not what
    // anybody meant, and on a ride with four trains it happens constantly.
    P.Trigger(S);
    P.Trigger(S);
    assert(P.TimesRun(S) == 3);
    std::printf("  the ride triggers a scene, and a second train restarts it rather than stacking\n");
}

void TestAMBIENTLoopsAndAScenePREEMPTSIt()
{
    // The composition the card describes: idle and ambient are model C again,
    // looping continuously, pre-empted by scene sequences on priority.
    //
    // PRIORITY, NOT MIXING. Two sequences driving one fixture is a fight, and
    // every real console resolves it with priority or latest-takes-precedence.
    // Priority is the one a person can reason about at 2 a.m. with a show running.
    FShowSequence Ambient;
    Ambient.Name = "ambient";
    Ambient.bLoop = true;
    Ambient.Priority = 0;
    Ambient.Cue.push_back({"glow", 5, 0.2, 1.0, ECueFollow::FromCompletion, 0.0, false, {}, false});

    FShowSequence Scene;
    Scene.Name = "scene";
    Scene.Priority = 10;
    Scene.Cue.push_back({"blast", 5, 1.0, 2.0, ECueFollow::None, 0.0, false, {}, false});

    FShowPlayer P;
    const std::size_t A = P.Add(Ambient);
    const std::size_t S = P.Add(Scene);
    P.Trigger(A);

    // Ambient alone, looping. It keeps producing indefinitely.
    double AmbientLevel = -1.0;
    for (int i = 0; i < 240 * 10; ++i)
    {
        for (const FCueFiring& F : P.Scan(Dt)) { if (F.Fixture == 5) { AmbientLevel = F.Level; } }
    }
    assert(P.IsRunning(A));
    assert(std::fabs(AmbientLevel - 0.2) < 1e-9);

    // The scene fires. It owns the fixture for its life, and the ambient does not
    // get a look in — one value, not a mix.
    P.Trigger(S);
    for (int i = 0; i < 240; ++i)
    {
        for (const FCueFiring& F : P.Scan(Dt))
        {
            if (F.Fixture == 5) { assert(std::fabs(F.Level - 1.0) < 1e-9); }
        }
    }

    // And when it ends, the ambient is still there — it never stopped, it was
    // only outranked.
    for (int i = 0; i < 240 * 3; ++i)
    {
        for (const FCueFiring& F : P.Scan(Dt)) { if (F.Fixture == 5) { AmbientLevel = F.Level; } }
    }
    assert(std::fabs(AmbientLevel - 0.2) < 1e-9);
    assert(P.IsRunning(A));
    std::printf("  a scene outranks the ambient for its life; the ambient never stopped\n");
}

void TestALOOPDoesNotDRIFT()
{
    // A loop that reset its clock to zero would lose up to one scan every pass.
    // At 240 Hz and a one-second loop that is 4 ms per pass — 14 seconds over an
    // operating day, which is exactly the sort of thing that is invisible in
    // testing and obvious by mid-afternoon.
    FShowSequence Q;
    Q.bLoop = true;
    Q.Cue.push_back({"tick", 0, 1.0, 0.1, ECueFollow::FromStart, 1.0, false, {}, false});

    FShowPlayer P;
    P.Trigger(P.Add(Q));

    int Fires = 0;
    const int Scans = 240 * 600;                 // ten minutes
    for (int i = 0; i < Scans; ++i)
    {
        for (const FCueFiring& F : P.Scan(Dt)) { if (F.bStarting) { ++Fires; } }
    }
    // 600 loops of one second, and drift would show up as a whole missing pass.
    assert(Fires >= 599 && Fires <= 601);
    std::printf("  a 1 s loop fires %d times in ten minutes: no drift\n", Fires);
}

void TestAMANUALCueSimplyWaits()
{
    // ECueFollow::None means the sequence does not advance by itself. That is a
    // "wait for GO" cue, and it is how a show hands control back to an operator
    // or to the ride.
    FShowSequence Q;
    Q.Cue.push_back({"a", 0, 1.0, 0.5, ECueFollow::None, 0.0, false, {}, false});
    Q.Cue.push_back({"b", 1, 1.0, 0.5, ECueFollow::None, 0.0, false, {}, false});

    FShowPlayer P;
    P.Trigger(P.Add(Q));
    const std::vector<double> T = FirstFires(P, 30.0, 2);
    assert(T[0] > 0.0);
    assert(T[1] < 0.0);                          // never fired, and never will
    std::printf("  a manual cue waits for ever rather than guessing at a time\n");
}

} // namespace

int main()
{
    std::printf("Show cues: three time models, and how they compose\n\n");

    TestFOLLOWAndHANGAreDIFFERENTAndThatIsThePoint();
    TestEditingADURATIONMovesOneAndNotTheOther();
    TestTIMECODEIsRestartableAnywhere();
    TestDROPFRAMEDropsNUMBERSNotFrames();
    TestTRIGGEREDChainsAreTheGlueToANondeterministicRide();
    TestAMBIENTLoopsAndAScenePREEMPTSIt();
    TestALOOPDoesNotDRIFT();
    TestAMANUALCueSimplyWaits();

    std::printf("\ntest_showcues: all assertions passed.\n");
    return 0;
}
