// Asserts for Scenario.h — a timeline of faults against the scan clock.
//
//   clang++ -std=c++17 -Wall -Wextra -o test_scenario test_scenario.cpp && ./test_scenario

#include "Scenario.h"
#include "StationProcess.h"

#include <cassert>
#include <cstdio>

namespace
{

const double Dt = 1.0 / 240.0;

void TestStepsComeDueOnTheirOwnScan()
{
    FScenario S;
    S.Add(100, EScenarioAction::JamRestraintGroup, 0, 2);
    S.Add(300, EScenarioAction::JamRestraintGroup, 0, -1);

    for (std::uint64_t Scan = 0; Scan < 500; ++Scan)
    {
        const auto Due = S.Due(Scan);
        if (Scan == 100)      { assert(Due.size() == 1 && Due[0].B == 2); }
        else if (Scan == 300) { assert(Due.size() == 1 && Due[0].B == -1); }
        else                  { assert(Due.empty()); }
    }
    assert(S.IsFinished());
    assert(S.MissedSteps() == 0);
    std::printf("  steps come due on their own scan and no other\n");
}

void TestOutOfOrderAdditionsStillFireInTimeOrder()
{
    FScenario S;
    S.Add(300, EScenarioAction::HealSensor, 1);
    S.Add(100, EScenarioAction::FailSensor, 1, 0);
    S.Add(200, EScenarioAction::PressEmergencyStop);

    std::vector<std::uint64_t> FiredAt;
    for (std::uint64_t Scan = 0; Scan <= 300; ++Scan)
    {
        for (const auto& St : S.Due(Scan)) { FiredAt.push_back(St.AtScan); }
    }
    assert((FiredAt == std::vector<std::uint64_t>{100, 200, 300}));
    std::printf("  a scenario written out of order still fires in time order\n");
}

void TestStepsSharingAScanKeepTheirWrittenOrder()
{
    // Two faults injected on the same scan are ordered by the person who wrote
    // them, and reordering would make the scenario mean something else. Hence a
    // STABLE sort rather than any sort.
    FScenario S;
    S.Add(50, EScenarioAction::FailSensor, 7, 0);
    S.Add(50, EScenarioAction::PressEmergencyStop);
    S.Add(50, EScenarioAction::JamGateSection, 1, 0);

    const auto Due = S.Due(50);
    assert(Due.size() == 3);
    assert(Due[0].Action == EScenarioAction::FailSensor);
    assert(Due[1].Action == EScenarioAction::PressEmergencyStop);
    assert(Due[2].Action == EScenarioAction::JamGateSection);
    std::printf("  steps sharing a scan keep the order they were written in\n");
}

void TestASKIPPEDScanIsCOUNTEDNotFiredLate()
{
    // A caller that does not pump every scan has not run the scenario as
    // written. Firing the step late would produce a run that reproduces
    // NOTHING — not the intended scenario and not a real one either — so the
    // step is dropped and counted, and the count is the evidence.
    FScenario S;
    S.Add(100, EScenarioAction::PressEmergencyStop);
    S.Add(200, EScenarioAction::ReleaseEmergencyStop);

    assert(S.Due(50).empty());
    assert(S.Due(150).empty());        // scan 100 was never asked for
    assert(S.MissedSteps() == 1);

    const auto Late = S.Due(200);
    assert(Late.size() == 1);          // this one was asked for on time
    assert(S.MissedSteps() == 1);
    std::printf("  a skipped scan is counted as missed, never fired late\n");
}

void TestRewindMakesItReplayable()
{
    // The property the whole file exists for: run it, run it again, get the
    // same run. Without this a scenario is a thing that happened once.
    FScenario S;
    S.Add(10, EScenarioAction::FailSensor, 3, 1);
    S.Add(20, EScenarioAction::PowerCyclePlc);

    std::vector<int> First;
    for (std::uint64_t i = 0; i <= 20; ++i)
    {
        for (const auto& St : S.Due(i)) { First.push_back(static_cast<int>(St.Action)); }
    }

    S.Rewind();
    std::vector<int> Second;
    for (std::uint64_t i = 0; i <= 20; ++i)
    {
        for (const auto& St : S.Due(i)) { Second.push_back(static_cast<int>(St.Action)); }
    }
    assert(First == Second);
    assert(S.MissedSteps() == 0);
    std::printf("  rewound and replayed, it produces the same run\n");
}

void TestAStuckHarnessSCENARIOHoldsTheRide()
{
    // THE WORKED EXAMPLE, and the reason this is not dead code. "Simulate a
    // stuck harness" was the developer's own phrase for what a scenario layer is
    // for, and until now every part of it existed except the thing that says
    // WHEN.
    //
    // Group 2 jams at scan 100 and frees at scan 2000. The dispatch must be held
    // for the whole of that and must recover afterwards without the platform
    // needing to redo the boarding.
    FScenario Sc;
    Sc.Add(100, EScenarioAction::JamRestraintGroup, 0, 2);
    Sc.Add(2000, EScenarioAction::JamRestraintGroup, 0, -1);

    FStationProcess Station(EStationRole::Combined);
    FAutoStationCrew Crew;
    Crew.UnloadSeconds = 0.2;
    Crew.LoadSeconds = 0.2;
    Crew.SecureSeconds = 0.2;
    Crew.Restraints.TravelSeconds = 0.2;
    Crew.Gates.TravelSeconds = 0.2;

    FStationInputs In;
    In.bTrainPresent = true;
    In.bTrainInPosition = true;

    bool bEverReadyWhileJammed = false;
    bool bReadyAfterFreed = false;

    for (std::uint64_t Scan = 0; Scan <= 3000; ++Scan)
    {
        for (const auto& St : Sc.Due(Scan))
        {
            if (St.Action == EScenarioAction::JamRestraintGroup) { Crew.StuckGroup = St.B; }
        }
        Station.Update(In);
        Crew.Serve(Station, In, Dt);

        if (Scan > 100 && Scan < 2000 && Station.IsReadyToDispatch())
        {
            bEverReadyWhileJammed = true;
        }
        if (Scan > 2200 && Station.IsReadyToDispatch()) { bReadyAfterFreed = true; }
    }

    // NOT ONE SCAN of permission while a bar was open. A stuck group is exactly
    // the failure a walk-round exists to find, and a dispatch granted through it
    // would mean the whole commanded-and-confirmed model bought nothing.
    assert(!bEverReadyWhileJammed);

    // And it RECOVERS. A fault that holds the ride for ever after it is fixed is
    // its own defect, and the difference between the two is only visible with
    // something that can free the fault as well as inject it.
    assert(bReadyAfterFreed);
    assert(Sc.MissedSteps() == 0);
    std::printf("  a stuck harness holds the dispatch for 1900 scans, then recovers\n");
}

void TestTheSameScenarioTwiceIsTheSameRun()
{
    // Determinism, at the scenario level. The scan clock is what buys it: a
    // timeline indexed by wall time would reproduce differently on a different
    // machine and be useless as evidence.
    auto Run = [](FScenario& Sc)
    {
        FStationProcess Station(EStationRole::Combined);
        FAutoStationCrew Crew;
        Crew.UnloadSeconds = 0.2; Crew.LoadSeconds = 0.2; Crew.SecureSeconds = 0.2;
        Crew.Restraints.TravelSeconds = 0.2; Crew.Gates.TravelSeconds = 0.2;
        FStationInputs In;
        In.bTrainPresent = true; In.bTrainInPosition = true;

        std::vector<int> Trace;
        for (std::uint64_t Scan = 0; Scan <= 1500; ++Scan)
        {
            for (const auto& St : Sc.Due(Scan))
            {
                if (St.Action == EScenarioAction::JamRestraintGroup) { Crew.StuckGroup = St.B; }
            }
            Station.Update(In);
            Crew.Serve(Station, In, Dt);
            Trace.push_back(static_cast<int>(Station.GetPhase()));
        }
        return Trace;
    };

    FScenario Sc;
    Sc.Add(200, EScenarioAction::JamRestraintGroup, 0, 1);
    Sc.Add(900, EScenarioAction::JamRestraintGroup, 0, -1);

    const auto A = Run(Sc);
    Sc.Rewind();
    const auto B = Run(Sc);
    assert(A == B);
    std::printf("  the same scenario twice produces the same phase trace\n");
}

} // namespace

int main()
{
    std::printf("Scenario: a timeline of faults against the scan clock\n\n");

    TestStepsComeDueOnTheirOwnScan();
    TestOutOfOrderAdditionsStillFireInTimeOrder();
    TestStepsSharingAScanKeepTheirWrittenOrder();
    TestASKIPPEDScanIsCOUNTEDNotFiredLate();
    TestRewindMakesItReplayable();
    TestAStuckHarnessSCENARIOHoldsTheRide();
    TestTheSameScenarioTwiceIsTheSameRun();

    std::printf("\ntest_scenario: all assertions passed.\n");
    return 0;
}
